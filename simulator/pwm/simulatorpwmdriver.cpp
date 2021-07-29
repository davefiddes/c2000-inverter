/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
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

#include "simulatorpwmdriver.h"
#include "errormessage.h"
#include "params.h"
#include "simulatorpwmgeneration.h"
#include <inttypes.h>
#include <stdio.h>

namespace simulator {

bool PwmDriver::sm_chatty = false;

/**
 * Driver Initialisation
 */
void PwmDriver::DriverInit()
{
    printf("\"DriverInit\"\n");
}

/**
 * Enable master PWM output
 */
void PwmDriver::EnableMasterOutput()
{
    if (sm_chatty)
    {
        printf("\"EnableMasterOutput\"\n");
    }
}

/**
 * Disable master PWM output
 */
void PwmDriver::DisableMasterOutput()
{
    if (sm_chatty)
    {
        printf("\"DisableMasterOutput\"\n");
    }
}

/**
 * Enable timer PWM output
 */
void PwmDriver::EnableOutput()
{
    if (sm_chatty)
    {
        printf("\"EnableOutput\"\n");
    }
}

/**
 * Disable timer PWM output
 */
void PwmDriver::DisableOutput()
{
    if (sm_chatty)
    {
        printf("\"DisableOutput\"\n");
    }
}

/**
 * Set the PWM values for each phase
 */
void PwmDriver::SetPhasePwm(uint32_t phaseA, uint32_t phaseB, uint32_t phaseC)
{
    printf(
        "\"SetPhasePwm\", %" PRIu32 ", %" PRIu32 ", %" PRIu32 "\n",
        phaseA,
        phaseB,
        phaseC);
}

/**
 * Enable the PWM outputs for charging
 * \param opmode Operating mode only BUCK and BOOST will do anything
 */
void PwmDriver::EnableChargeOutput(__attribute__((__unused__)) Modes opmode)
{
}

/**
 * Enable the PWM outputs for AC heat
 */
void PwmDriver::EnableACHeatOutput()
{
}

/**
 * Program the PWM output compare units for our over current limits
 * \param limNeg    Negative over current limit
 * \param limPos    Positive over current limit
 */
void PwmDriver::SetOverCurrentLimits(int limNeg, int limPos)
{
    printf(
        "\"SetOverCurrentLimits\", %" PRId16 ", %" PRId16 "\n", limNeg, limPos);
}

/**
 * Setup main PWM timer
 *
 * \param[in] deadtime Deadtime between bottom and top (coded value, consult
 * STM32 manual)
 * \param[in] activeLow Set Output Polarity true=Active Low
 * \param[in] pwmdigits Number of PWM digits we are using
 * \return PWM ISR callback frequency
 */
uint16_t PwmDriver::TimerSetup(
    uint16_t deadtime,
    bool     activeLow,
    uint16_t pwmdigits)
{
    printf(
        "\"TimerSetup\", %" PRIu16 ", %s, %" PRIu16 "\n",
        deadtime,
        activeLow ? "true" : "false",
        pwmdigits);

    // Return a 1kHz PWM frequency, fast enough to do real work, slow enough not
    // to take forver to print out
    return 1000;
}

/**
 * Set up the timer for AC heat use
 */
void PwmDriver::AcHeatTimerSetup()
{
}

/**
 * Enable AC heat
 * \param ampnom    Notminal current to use for AC heat
 */
void PwmDriver::AcHeat(__attribute__((__unused__)) s32fp ampnom)
{
}

/**
 * Set the charge current target
 * \param dc    Target current
 */
void PwmDriver::SetChargeCurrent(__attribute__((__unused__)) int dc)
{
}

/**
 * Obtain how many PWM ticks we spend running the main control loop
 * \return Number of ticks
 */
int PwmDriver::GetCpuLoad()
{
    return 100;
}

/**
 * Reset the CPU load stored value for when the PWM is not running
 */
void PwmDriver::ResetCpuLoad()
{
}

/**
 * Enable the more verbose and frequent messages
 */
void PwmDriver::EnableChattyOutput()
{
    sm_chatty = true;
}


} // namespace simulator
