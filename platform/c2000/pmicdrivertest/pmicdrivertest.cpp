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

#include "pmicdriver.h"
#include "c2000/pmicspidriver.h"
#include "c2000/scheduler.h"

#include "driverlib.h"
#include <stdio.h>

using namespace c2000;

// task added to scheduler to strobe the powerwatchdog
static void taskStrobePowerWatchdoc()
{
    TeslaM3PowerWatchdog<PmicSpiDriver>::Strobe();
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

    // Use only line-by-line buffering on stdout for easier debugging
    // We still need to use line buffering to avoid very poor IO performance
    // with C2000 semi-hosting
    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("Tesla M3 pmic driver test application\n\n");

    printf(
        "Running on: %s\n",
        IsTeslaM3Inverter() ? "Tesla M3 Inverter" : "TI Launchpad");

    //
    // Set up the LEDs
    //
    uint32_t greenLedPin;
    uint32_t redLedPin;

    if (IsTeslaM3Inverter())
    {
        greenLedPin = DEVICE_TESLAM3_GPIO_PIN_LED1;
        redLedPin = DEVICE_TESLAM3_GPIO_PIN_LED2;
    }
    else
    {
        greenLedPin = DEVICE_LAUNCHXL_GPIO_PIN_LED1;
        redLedPin = DEVICE_LAUNCHXL_GPIO_PIN_LED2;
    }

    GPIO_writePin(greenLedPin, 1);
    GPIO_setPadConfig(greenLedPin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(greenLedPin, GPIO_DIR_MODE_OUT);

    GPIO_writePin(redLedPin, 1);
    GPIO_setPadConfig(redLedPin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(redLedPin, GPIO_DIR_MODE_OUT);

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

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    printf("Pmic driver initialisation: ");
    TeslaM3PowerWatchdog<PmicSpiDriver>::Init();

    // add a task to strobe the power watchdog every 100ms
    Scheduler::AddTask(taskStrobePowerWatchdoc, 100);

    for (;;)
    {

        GPIO_writePin(redLedPin, 0);

        DEVICE_DELAY_US(300000);

        GPIO_writePin(redLedPin, 1);

        DEVICE_DELAY_US(300000);
    }
}
