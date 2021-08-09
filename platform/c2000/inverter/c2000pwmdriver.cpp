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

#include "c2000pwmdriver.h"
#include "c2000pwmgeneration.h"
#include "errormessage.h"
#include "params.h"

namespace c2000 {

/**
 * Driver Initialisation
 */
void PwmDriver::DriverInit()
{
}

/**
 * Enable master PWM output
 */
void PwmDriver::EnableMasterOutput()
{
}

/**
 * Disable master PWM output
 */
void PwmDriver::DisableMasterOutput()
{
}

/**
 * Enable timer PWM output
 */
void PwmDriver::EnableOutput()
{
}

/**
 * Disable timer PWM output
 */
void PwmDriver::DisableOutput()
{
}

/**
 * Set the PWM values for each phase
 */
void PwmDriver::SetPhasePwm(uint32_t phaseA, uint32_t phaseB, uint32_t phaseC)
{
}

/** Store of number of PWM ticks we spend running the main PWM interrupt handler
 */
__attribute__((unused)) static int execTicks;

/**
 * Main PWM timer interrupt. Run the main motor control loop while measuring how
 * long we take
 */
extern "C" void pwm_timer_isr(void)
{
#if 0
    int start = timer_get_counter(PWM_TIMER);
    /* Clear interrupt pending flag */
    timer_clear_flag(PWM_TIMER, TIM_SR_UIF);
#endif
    PwmGeneration::Run();

#if 0
    int time = timer_get_counter(PWM_TIMER) - start;

    if (TIM_CR1(PWM_TIMER) & TIM_CR1_DIR_DOWN)
        time = (2 << PwmGeneration::GetPwmDigits()) -
               timer_get_counter(PWM_TIMER) - start;
    execTicks = ABS(time);
#endif
}

/**
 * Enable the PWM outputs for charging
 * \param opmode Operating mode only BUCK and BOOST will do anything
 */
void PwmDriver::EnableChargeOutput(Modes opmode)
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
void PwmDriver::SetOverCurrentLimits(int16_t limNeg, int16_t limPos)
{
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
    return 0;
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
void PwmDriver::AcHeat(s32fp ampnom)
{
}

/**
 * Set the charge current target
 * \param dc    Target current
 */
void PwmDriver::SetChargeCurrent(int16_t dc)
{
}

/**
 * Obtain how many PWM ticks we spend running the main control loop
 * \return Number of ticks
 */
int16_t PwmDriver::GetCpuLoad()
{
    return 100;
}

/**
 * Reset the CPU load stored value for when the PWM is not running
 */
void PwmDriver::ResetCpuLoad()
{
    execTicks = 0;
}

} // namespace c2000
