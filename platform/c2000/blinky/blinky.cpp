//#############################################################################
//
// FILE:   led_ex1_blinky.c
//
// TITLE:  LED Blinky Example
//
//! \addtogroup driver_example_list
//! <h1> LED Blinky Example </h1>
//!
//! This example demonstrates how to blink a LED.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - None.
//!
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.12.00.00 $
// $Release Date: Fri Feb 12 19:03:23 IST 2021 $
// $Copyright:
// Copyright (C) 2013-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "device.h"
#include "driverlib.h"

//
// Defines
//
#define LOOP_COUNT 10

//
// Main
//
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

    uint32_t led1Pin;
    uint32_t led2Pin;

    if (IsTeslaM3Inverter())
    {
        led1Pin = DEVICE_TESLAM3_GPIO_PIN_LED1;
        led2Pin = DEVICE_TESLAM3_GPIO_PIN_LED2;
    }
    else
    {
        led1Pin = DEVICE_LAUNCHXL_GPIO_PIN_LED1;
        led2Pin = DEVICE_LAUNCHXL_GPIO_PIN_LED2;
    }
    GPIO_setPadConfig(led1Pin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(led1Pin, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(led2Pin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(led2Pin, GPIO_DIR_MODE_OUT);


    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Loop Forever
    //
    for (;;)
    {
        //
        // Turn on LED
        //
        GPIO_writePin(led1Pin, 0);
        GPIO_writePin(led2Pin, 1);

        //
        // Delay for a bit.
        //
        DEVICE_DELAY_US(300000);

        //
        // Turn off LED
        //
        GPIO_writePin(led1Pin, 1);
        GPIO_writePin(led2Pin, 0);

        //
        // Delay for a bit.
        //
        DEVICE_DELAY_US(300000);
    }
}

//
// End of File
//
