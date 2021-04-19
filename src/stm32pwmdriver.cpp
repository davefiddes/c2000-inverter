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

#include "stm32pwmdriver.h"
#include "digio.h"
#include "errormessage.h"
#include "hwdefs.h"
#include "params.h"
#include "pwmgeneration.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

/** PWM ISR callback frequency divider */
#define FRQ_DIVIDER 8192

/** Timer output compare channel to PWM channel mapping. Use to implement PWM
 * pin swapping */
tim_oc_id STM32PwmDriver::ocChannels[3];

/**
 * Driver Initialisation
 */
void STM32PwmDriver::DriverInit()
{
#if CONTROL == CTRL_FOC
    // FOC requires phase current measurements to match PWM outputs so PWM pins
    // are swappable to account for wiring errors
    if ((Param::GetInt(Param::pinswap) & SWAP_PWM13) > 0)
    {
        ocChannels[0] = TIM_OC3;
        ocChannels[1] = TIM_OC2;
        ocChannels[2] = TIM_OC1;
    }
    else if ((Param::GetInt(Param::pinswap) & SWAP_PWM23) > 0)
    {
        ocChannels[0] = TIM_OC1;
        ocChannels[1] = TIM_OC3;
        ocChannels[2] = TIM_OC2;
    }
    else
    {
        ocChannels[0] = TIM_OC1;
        ocChannels[1] = TIM_OC2;
        ocChannels[2] = TIM_OC3;
    }
#elif CONTROL == CTRL_SINE
    // Sine does not require configurable PWM output pins
    ocChannels[0] = TIM_OC1;
    ocChannels[1] = TIM_OC2;
    ocChannels[2] = TIM_OC3;
#endif
}

/**
 * Enable master PWM output
 */
void STM32PwmDriver::EnableMasterOutput()
{
    timer_enable_break_main_output(PWM_TIMER);
}

/**
 * Disable master PWM output
 */
void STM32PwmDriver::DisableMasterOutput()
{
    timer_disable_break_main_output(PWM_TIMER);
}

/**
 * Enable timer PWM output
 */
void STM32PwmDriver::EnableOutput()
{
    timer_enable_oc_output(PWM_TIMER, TIM_OC1);
    timer_enable_oc_output(PWM_TIMER, TIM_OC2);
    timer_enable_oc_output(PWM_TIMER, TIM_OC3);

    if (hwRev != HW_PRIUS)
    {
        timer_enable_oc_output(PWM_TIMER, TIM_OC1N);
        timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
        timer_enable_oc_output(PWM_TIMER, TIM_OC3N);
    }
}

/**
 * Disable timer PWM output
 */
void STM32PwmDriver::DisableOutput()
{
    timer_disable_oc_output(PWM_TIMER, TIM_OC1);
    timer_disable_oc_output(PWM_TIMER, TIM_OC2);
    timer_disable_oc_output(PWM_TIMER, TIM_OC3);
    timer_disable_oc_output(PWM_TIMER, TIM_OC1N);
    timer_disable_oc_output(PWM_TIMER, TIM_OC2N);
    timer_disable_oc_output(PWM_TIMER, TIM_OC3N);
}

/**
 * Set the PWM values for each phase
 */
void STM32PwmDriver::SetPhasePwm(
    uint32_t phaseA,
    uint32_t phaseB,
    uint32_t phaseC)
{
    timer_set_oc_value(PWM_TIMER, ocChannels[0], phaseA);
    timer_set_oc_value(PWM_TIMER, ocChannels[1], phaseB);
    timer_set_oc_value(PWM_TIMER, ocChannels[2], phaseC);
}

/**
 * Over current trip interrupt
 */
extern "C" void tim1_brk_isr(void)
{
    if (!DigIo::desat_in.Get() && hwRev != HW_REV1 && hwRev != HW_BLUEPILL)
        ErrorMessage::Post(ERR_DESAT);
    else if (!DigIo::emcystop_in.Get() && hwRev != HW_REV3)
        ErrorMessage::Post(ERR_EMCYSTOP);
    else if (!DigIo::mprot_in.Get() && hwRev != HW_BLUEPILL)
        ErrorMessage::Post(ERR_MPROT);
    else // if (ocur || hwRev == HW_REV1)
        ErrorMessage::Post(ERR_OVERCURRENT);

    timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
    Param::SetEnum(Param::opmode, Modes::OFF);
    DigIo::err_out.Set();
    PwmGeneration::SetOvercurrentTripped();
}

/** Store of number of PWM ticks we spend running the main PWM interrupt handler
 */
static int execTicks;

/**
 * Main PWM timer interrupt. Run the main motor control loop while measuring how
 * long we take
 */
extern "C" void pwm_timer_isr(void)
{
    int start = timer_get_counter(PWM_TIMER);
    /* Clear interrupt pending flag */
    timer_clear_flag(PWM_TIMER, TIM_SR_UIF);

    PwmGeneration::Run();

    int time = timer_get_counter(PWM_TIMER) - start;

    if (TIM_CR1(PWM_TIMER) & TIM_CR1_DIR_DOWN)
        time = (2 << PwmGeneration::GetPwmDigits()) -
               timer_get_counter(PWM_TIMER) - start;

    execTicks = ABS(time);
}

/**
 * Enable the PWM outputs for charging
 * \param opmode Operating mode only BUCK and BOOST will do anything
 */
void STM32PwmDriver::EnableChargeOutput(Modes opmode)
{
    /* Prius GEN2 inverter only has one control signal for the buck/boost
     * converter When we output a "high" the upper IGBT is switched on. So for
     * bucking output current and duty cycle are proportional For boosting we
     * need to output a "low" to enable the low side IGBT and thus the output
     * current and duty cycle are inverse proportional.
     */
    if (hwRev == HW_PRIUS)
    {
        // Disable other PWM source.
        timer_disable_oc_output(OVER_CUR_TIMER, TIM_OC4);

        if (opmode == Modes::BOOST)
            timer_set_oc_polarity_low(PWM_TIMER, TIM_OC2N);
        else if (opmode == Modes::BUCK)
            timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2N);

        timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
    }
    else
    {
        if (opmode == Modes::BOOST)
            timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
        else if (opmode == Modes::BUCK)
            timer_enable_oc_output(PWM_TIMER, TIM_OC2);
    }

    timer_enable_break_main_output(PWM_TIMER);
}

/**
 * Enable the PWM outputs for AC heat
 */
void STM32PwmDriver::EnableACHeatOutput()
{
    timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
    timer_enable_oc_output(PWM_TIMER, TIM_OC2);
    timer_enable_break_main_output(PWM_TIMER);
}

/**
 * Program the PWM output compare units for our over current limits
 * \param limNeg    Negative over current limit
 * \param limPos    Positive over current limit
 */
void STM32PwmDriver::SetOverCurrentLimits(int limNeg, int limPos)
{
    timer_set_oc_value(OVER_CUR_TIMER, OVER_CUR_NEG, limNeg);
    timer_set_oc_value(OVER_CUR_TIMER, OVER_CUR_POS, limPos);
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
uint16_t STM32PwmDriver::TimerSetup(
    uint16_t deadtime,
    bool     activeLow,
    uint16_t pwmdigits)
{
    /// There are two update events per PWM period
    /// One when counter reaches top, one when it reaches bottom
    /// We set the repetition counter in a way, that the ISR
    /// Callback frequency is constant i.e. independent from PWM frequency
    ///- for 17.6 kHz: call ISR every four update events (that is every other
    /// period)
    ///- for 8.8kHz: call ISR every other update event (that is once per PWM
    /// period)
    ///- for 4.4kHz: call ISR on every update event (that is twice per period)
    const uint8_t  repCounters[] = { 3, 1, 0 };
    const uint16_t pwmmax = 1U << pwmdigits;
    const uint8_t  outputMode = activeLow ? GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN :
                                            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL;

    // Disable output in active low mode before resetting timer, otherwise shoot
    // through will occur!
    if (activeLow)
    {
        gpio_set_mode(
            GPIOA,
            GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_FLOAT,
            GPIO8 | GPIO9 | GPIO10);
        gpio_set_mode(
            GPIOB,
            GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_FLOAT,
            GPIO13 | GPIO14 | GPIO15);
    }

    rcc_periph_reset_pulse(PWM_TIMRST);

    /* Center aligned PWM */
    timer_set_alignment(PWM_TIMER, TIM_CR1_CMS_CENTER_1);
    timer_enable_preload(PWM_TIMER);

    for (int channel = TIM_OC1; channel <= TIM_OC3N; channel++)
    {
        timer_enable_oc_preload(PWM_TIMER, (tim_oc_id)channel);
        timer_set_oc_mode(PWM_TIMER, (tim_oc_id)channel, TIM_OCM_PWM1);
        timer_set_oc_idle_state_unset(PWM_TIMER, (tim_oc_id)channel);
        timer_set_oc_value(PWM_TIMER, (tim_oc_id)channel, 0);

        if (activeLow)
            timer_set_oc_polarity_low(PWM_TIMER, (tim_oc_id)channel);
        else
            timer_set_oc_polarity_high(PWM_TIMER, (tim_oc_id)channel);
    }

    timer_disable_break_automatic_output(PWM_TIMER);

    if (hwRev == HW_BLUEPILL || hwRev == HW_PRIUS || hwRev == HW_TESLAM3)
        timer_set_break_polarity_low(PWM_TIMER);
    else
        timer_set_break_polarity_high(PWM_TIMER);

    timer_enable_break(PWM_TIMER);
    timer_set_enabled_off_state_in_run_mode(PWM_TIMER);
    timer_set_enabled_off_state_in_idle_mode(PWM_TIMER);
    timer_set_deadtime(PWM_TIMER, deadtime);
    timer_clear_flag(PWM_TIMER, TIM_SR_UIF | TIM_SR_BIF);
    timer_enable_irq(PWM_TIMER, TIM_DIER_UIE | TIM_DIER_BIE);

    timer_set_prescaler(PWM_TIMER, 0);
    /* PWM frequency */
    timer_set_period(PWM_TIMER, pwmmax);
    timer_set_repetition_counter(
        PWM_TIMER, repCounters[pwmdigits - MIN_PWM_DIGITS]);

    timer_generate_event(PWM_TIMER, TIM_EGR_UG);

    timer_enable_counter(PWM_TIMER);

    gpio_set_mode(
        GPIOA, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO8 | GPIO9 | GPIO10);
    gpio_set_mode(
        GPIOB, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO13 | GPIO14 | GPIO15);
    // Callback frequency is constant because we use the repetition counter
    return rcc_apb2_frequency / FRQ_DIVIDER;
}

/**
 * Set up the timer for AC heat use
 */
void STM32PwmDriver::AcHeatTimerSetup()
{
    timer_disable_counter(PWM_TIMER);
    timer_set_clock_division(PWM_TIMER, TIM_CR1_CKD_CK_INT_MUL_4);
    timer_set_deadtime(PWM_TIMER, 255);
    timer_set_period(PWM_TIMER, 8000);
    timer_set_oc_value(PWM_TIMER, TIM_OC2, 0);
    timer_generate_event(PWM_TIMER, TIM_EGR_UG);
    timer_enable_counter(PWM_TIMER);
}

/**
 * Enable AC heat
 * \param ampnom    Notminal current to use for AC heat
 */
void STM32PwmDriver::AcHeat(s32fp ampnom)
{
    // We need to make sure the negative output is NEVER permanently on.
    if (ampnom < FP_FROMFLT(20))
    {
        timer_disable_break_main_output(PWM_TIMER);
    }
    else
    {
        timer_enable_break_main_output(PWM_TIMER);
        int dc = FP_TOINT((ampnom * 30000) / 100);
        Param::SetInt(Param::amp, dc);
        timer_set_period(PWM_TIMER, dc);
        timer_set_oc_value(PWM_TIMER, TIM_OC2, dc / 2);
    }
}

/**
 * Set teh charge current target
 * \param dc    Target current
 */
void STM32PwmDriver::SetChargeCurrent(int dc)
{
    timer_set_oc_value(PWM_TIMER, TIM_OC2, dc);
}

/**
 * Obtain how many PWM ticks we spend running the main control loop
 * \return Number of ticks
 */
int STM32PwmDriver::GetCpuLoad()
{
    // PWM period 2x counter because of center aligned mode
    return (1000 * execTicks) / FRQ_DIVIDER;
}

/**
 * Reset the CPU load stored value for when the PWM is not running
 */
void STM32PwmDriver::ResetCpuLoad()
{
    execTicks = 0;
}
