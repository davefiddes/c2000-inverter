/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2021 David J. Fiddes <D.J@fiddes.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "device.h"
#include "driverlib.h"
#include "errormessage.h"
#include "focpwmgeneration.h"
#include "pmicdriver.h"
#include "c2000/current.h"
#include "c2000/encoder.h"
#include "c2000/gatedriver.h"
#include "c2000/motoranalogcapture.h"
#include "c2000/performancecounter.h"
#include "c2000/pmicspidriver.h"
#include "c2000/pwmdriver.h"
#include "c2000/pwmgeneration.h"
#include "c2000/scheduler.h"
#include <stdio.h>

// Pull in the whole C2000 namespace as this is platform specific code obviously
using namespace c2000;

void Param::Change(Param::PARAM_NUM paramNum)
{
}

typedef TeslaM3PowerWatchdog<PmicSpiDriver> PowerWatchdog;

// task added to scheduler to strobe the powerwatchdog
static void taskStrobePowerWatchdog()
{
    PowerWatchdog::Strobe();
}

void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    Device_initGPIO();

    //
    // Set up GPIO pinmux for EPWM
    //
    if (IsTeslaM3Inverter())
    {
        GPIO_setPinConfig(GPIO_0_EPWM1A);
        GPIO_setPinConfig(GPIO_8_EPWM5A);
        GPIO_setPinConfig(GPIO_9_EPWM5B);
        GPIO_setPinConfig(GPIO_10_EPWM6A);
        GPIO_setPinConfig(GPIO_11_EPWM6B);
        GPIO_setPinConfig(GPIO_12_EPWM7A);
        GPIO_setPinConfig(GPIO_13_EPWM7B);
    }
    else
    {
        GPIO_setPinConfig(GPIO_0_EPWM1A);
        GPIO_setPinConfig(GPIO_2_EPWM2A);
        GPIO_setPinConfig(GPIO_3_EPWM2B);
        GPIO_setPinConfig(GPIO_4_EPWM3A);
        GPIO_setPinConfig(GPIO_5_EPWM3B);
        GPIO_setPinConfig(GPIO_6_EPWM4A);
        GPIO_setPinConfig(GPIO_7_EPWM4B);
    }

    //
    // Set up the heartbeat LED
    //
    uint32_t heartbeatLedPin;

    if (IsTeslaM3Inverter())
    {
        heartbeatLedPin = DEVICE_TESLAM3_GPIO_PIN_LED1;
    }
    else
    {
        heartbeatLedPin = DEVICE_LAUNCHXL_GPIO_PIN_LED1;
    }
    GPIO_writePin(heartbeatLedPin, 1);
    GPIO_setPadConfig(heartbeatLedPin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(heartbeatLedPin, GPIO_DIR_MODE_OUT);

    //
    // Turn on the gate drive PSU
    //
    GPIO_writePin(DEVICE_GPIO_PIN_GATE_PSU_ENABLE, 0);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_GATE_PSU_ENABLE, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_GATE_PSU_ENABLE, GPIO_DIR_MODE_OUT);
    printf("Gate Drive PSU ON\n");

    //
    // Set up the gate drivers for PWM operation
    //
    printf("Gate Drive initialisation: ");
    if (GateDriver::Init())
    {
        printf("OK\n");
        GateDriver::Enable();
    }
    else
    {
        printf("Fail\n");
    }

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    Scheduler::Init();

    printf(
        "Pmic driver initialisation: %s\n",
        PowerWatchdog::Init() == PowerWatchdog::OK ? "OK" : "Fail");

    // add a task to strobe the power watchdog every 100ms
    Scheduler::AddTask(taskStrobePowerWatchdog, 100);

    // Set up the error message log and set operating parameters to default
    ErrorMessage::ResetAll();
    // TODO: Figure out where the timer tick comes from to increment this
    ErrorMessage::SetTime(1);
    Param::LoadDefaults();

    // Configure the PWM generation
    PwmGeneration::SetCurrentOffset(2048, 2048);

    // We need the pole pair ratio set to correctly calculate the rotation
    // frequency
    PwmGeneration::SetPolePairRatio(1);

    // Ensure the system thinks we should be going forwards
    Param::SetInt(Param::dir, 1);

    // initialise the controller gains from the default parameters
    PwmGeneration::SetControllerGains(
        Param::GetInt(Param::curkp),
        Param::GetInt(Param::curki),
        Param::GetInt(Param::fwkp));

    // Override the default deadtime as the C2000 uses values in nS rather
    // than a coded STM32 value
    Param::SetInt(Param::deadtime, 875);

    // Put in a bit of Q current to get the inverter to do something
    Param::Set(Param::manualiq, FP_FROMFLT(0.6));

    // Provide some neutral values for the phase currents
    Current::SetPhase1(2048);
    Current::SetPhase2(2048);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Turn on the global PWM buffer enable
    //
    GPIO_writePin(DEVICE_GPIO_PIN_PWM_ENABLE, 1);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_PWM_ENABLE, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_PWM_ENABLE, GPIO_DIR_MODE_OUT);

    // Go for manual mode
    PwmGeneration::SetOpmode(MANUAL);

    //
    // Loop Forever
    //
    int32_t lastLoad = PwmGeneration::GetCpuLoad();
    while (true)
    {
        DEVICE_DELAY_US(500000);

        printf(
            "PhaseA Current = %u, PhaseB Current = %u, Resolver Sine = %u, "
            "Resolver Cosine = %u\n",
            MotorAnalogCapture::PhaseACurrent(),
            MotorAnalogCapture::PhaseBCurrent(),
            MotorAnalogCapture::ResolverSine(),
            MotorAnalogCapture::ResolverCosine());

        printf("Gate Drive: %s\n", GateDriver::IsFaulty() ? "FAULT" : "OK");

        int32_t currentLoad = PwmGeneration::GetCpuLoad();
        printf("PWM cycles: %ld\n", currentLoad - lastLoad);
        lastLoad = currentLoad;

        GPIO_togglePin(heartbeatLedPin);
    }
}
