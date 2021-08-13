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

#include "c2000current.h"
#include "c2000encoder.h"
#include "c2000pwmdriver.h"
#include "c2000pwmgeneration.h"
#include "device.h"
#include "driverlib.h"
#include "errormessage.h"
#include "focpwmgeneration.h"
#include "performancecounter.h"
#include <stdio.h>

// Pull in the whole C2000 namespace as this is platform specific code obviously
using namespace c2000;

void parm_Change(Param::PARAM_NUM paramNum)
{
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
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setPinConfig(GPIO_5_EPWM3B);

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

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

    // Override the default deadtime as the C2000 uses values in nS rather than
    // a coded STM32 value
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

    // Go for manual mode
    PwmGeneration::SetOpmode(MANUAL);

    //
    // Loop Forever
    //
    int32_t lastLoad = PwmGeneration::GetCpuLoad();
    while (true)
    {
        DEVICE_DELAY_US(500000);

        int32_t currentLoad = PwmGeneration::GetCpuLoad();
        printf("PWM cycles: %ld\n", currentLoad - lastLoad);
        lastLoad = currentLoad;
    }
}
