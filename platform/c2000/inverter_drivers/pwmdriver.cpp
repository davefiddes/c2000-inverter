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

#include "c2000/pwmdriver.h"
#include "device.h"
#include "driverlib.h"
#include "errormessage.h"
#include "params.h"
#include "c2000/motoranalogcapture.h"
#include "c2000/performancecounter.h"
#include "c2000/pwmgeneration.h"

namespace c2000 {

/** Phase delay between the resolver exciter square wave and the phase PWM.
 * Measured as approx32.8 uSec between centre of square wave and peak of sine
 * wave on Tesla Model 3 980 inverter. Constant below adjusted to align the peak
 * of the sine wave with the centre of the phase PWM signals on a scope.
 */
static const uint16_t resolverPhaseDelay = 3230U;

/** Phase A EPWM Base Address */
uint32_t PwmDriver::sm_phaseAEpwmBase;

/** Phase B EPWM Base Address */
uint32_t PwmDriver::sm_phaseBEpwmBase;

/** Phase B EPWM Base Address */
uint32_t PwmDriver::sm_phaseCEpwmBase;

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
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseAEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseAEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseBEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseBEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseCEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseCEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);
}

/**
 * Disable master PWM output
 */
void PwmDriver::DisableMasterOutput()
{
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseAEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseAEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseBEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseBEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseCEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseCEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);
}

/**
 * Enable timer PWM output
 */
void PwmDriver::EnableOutput()
{
    // Not used on the C2000
}

/**
 * Disable timer PWM output
 */
void PwmDriver::DisableOutput()
{
    // Not used on the C2000
}

/**
 * Set the PWM values for each phase
 */
void PwmDriver::SetPhasePwm(uint32_t phaseA, uint32_t phaseB, uint32_t phaseC)
{
    // Load in the phase values - these will be loaded from the shadow registers
    // when the counter reaches zero
    EPWM_setCounterCompareValue(
        sm_phaseAEpwmBase, EPWM_COUNTER_COMPARE_A, phaseA);
    EPWM_setCounterCompareValue(
        sm_phaseBEpwmBase, EPWM_COUNTER_COMPARE_A, phaseB);
    EPWM_setCounterCompareValue(
        sm_phaseCEpwmBase, EPWM_COUNTER_COMPARE_A, phaseC);
}

/** Store of number of SYSCLOCK ticks we spend running the main PWM interrupt
 * handler
 */
static int32_t execTicks;

/**
 * Main motor control ADC data available interrupt. This fires when the PWM
 * triggered ADC conversion completes and runs the main motor control loop. It
 * will measuring how long we take
 */
__interrupt void motor_control_adc_isr(void)
{
    uint32_t startTime = PerformanceCounter::GetCount();

    PwmGeneration::Run();

    // Measure the time - handles timer overflows
    uint32_t totalTime = startTime - PerformanceCounter::GetCount();
    execTicks = execTicks + totalTime;

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if (ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

/**
 * Enable the PWM outputs for charging
 * \param opmode Operating mode only BUCK and BOOST will do anything
 */
void PwmDriver::EnableChargeOutput(Modes opmode)
{
    // Not relevant to this HW
}

/**
 * Enable the PWM outputs for AC heat
 */
void PwmDriver::EnableACHeatOutput()
{
    // Not relevant to this HW
}

/**
 * Program the PWM output compare units for our over current limits
 * \param limNeg    Negative over current limit
 * \param limPos    Positive over current limit
 */
void PwmDriver::SetOverCurrentLimits(int16_t limNeg, int16_t limPos)
{
    // TODO: Implement possibly via filtered analog compare and EPWM Trip-Zone
}

/**
 * Initialise a EPMW module to act as a resolver exciter
 *
 * We are aiming to generate a 50% duty cycle square wave
 *
 * \param[in] base The EPWM module to configure
 * \param[in] pwmmax The PWM period
 */
static void initResolverEPWM(uint32_t base, uint16_t pwmmax)
{
    //
    // Allow the PWM to continue when debugging
    //
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, pwmmax);
    EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0U);

    //
    // Set Compare values - 50% duty cycle
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, pwmmax / 2);

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Load shadow action qualifier force at the end of the period to get a
    // clean on-off transition
    //
    EPWM_setActionQualifierContSWForceShadowMode(
        base, EPWM_AQ_SW_SH_LOAD_ON_CNTR_PERIOD);

    //
    // Set actions to only use output A
    //
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
}

/**
 * Initialise an EPMW module just enough to act as a synchronisation source for
 * an EPWM module chain
 *
 * \param[in] base The EPWM module to configure
 * \param[in] pwmmax The PWM period
 */
static void initSyncEPWM(uint32_t base, uint16_t pwmmax)
{
    //
    // Allow the PWM to continue when debugging
    //
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, pwmmax);
    EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0U);

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
}

/**
 * Initialise an individual EPMW module
 *
 * \param[in] base The EPWM module to configure
 * \param[in] pwmmax The PWM period
 * \param[in] deadBandCount The size of the deadband in PWM counts
 * \param[in] phaseDelay The PWM phase delay from the synchonisation source
 */
static void initPhaseEPWM(
    uint32_t base,
    uint16_t pwmmax,
    uint16_t deadBandCount,
    uint16_t phaseDelay)
{
    //
    // Allow the PWM to continue when debugging
    //
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, pwmmax);
    EPWM_setPhaseShift(base, phaseDelay);
    EPWM_setTimeBaseCounter(base, phaseDelay);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, 0);

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Load shadow compare in the centre (zero)
    //
    EPWM_setCounterCompareShadowLoadMode(
        base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Load the period in response to synchonisation events only
    //
    EPWM_selectPeriodLoadEvent(base, EPWM_SHADOW_LOAD_MODE_SYNC);

    //
    // Load shadow action qualifier force at the end of the period to get a
    // clean on-off transition
    //
    EPWM_setActionQualifierContSWForceShadowMode(
        base, EPWM_AQ_SW_SH_LOAD_ON_CNTR_PERIOD);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Use EPWMA as the input for both RED and FED
    //
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);

    //
    // Set the RED and FED values
    //
    EPWM_setFallingEdgeDelayCount(base, deadBandCount);
    EPWM_setRisingEdgeDelayCount(base, deadBandCount);

    //
    // Invert only the Falling Edge delayed output (AHC)
    //
    EPWM_setDeadBandDelayPolarity(
        base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(
        base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    //
    // Use the delayed signals instead of the original signals
    //
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);

    //
    // DO NOT Switch Output A with Output B
    //
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);
}

/**
 * Setup main PWM timer
 *
 * \param[in] deadtime Deadtime between bottom and top (in nS)
 * \param[in] activeLow Set Output Polarity true=Active Low
 * \param[in] pwmdigits Number of PWM digits we are using
 * \return PWM ISR callback frequency
 */
uint16_t PwmDriver::TimerSetup(
    uint16_t deadtime,
    bool     activeLow,
    uint16_t pwmdigits)
{
    const uint32_t EPWM_FREQ = DEVICE_SYSCLK_FREQ / 2;

    // Determine the maximum counter value for PWM
    const uint16_t pwmmax = 1U << pwmdigits;

    // Determine the count for the required deadtime
    const uint16_t nsPerCount = 1e9 / EPWM_FREQ;
    const uint16_t deadBandCount = deadtime / nsPerCount;

    // Disable sync(Freeze clock to PWM as well).
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    if (IsTeslaM3Inverter())
    {
        // Store the EPWM modules used for each phase for normal operation
        sm_phaseAEpwmBase = EPWM5_BASE;
        sm_phaseBEpwmBase = EPWM7_BASE;
        sm_phaseCEpwmBase = EPWM6_BASE;

        //
        // Initialize EPWM1 as master clock and resolver exciter output
        //
        initResolverEPWM(EPWM1_BASE, pwmmax);

        //
        // Initialize EPWM4 just enough to pass on the synchronisation pulse
        // This EPWM module is otherwise redundant
        //
        initSyncEPWM(EPWM4_BASE, pwmmax);

        //
        // Initialize EPWM5, EPWM6 and EPWM7 as phase PWM outputs
        //
        initPhaseEPWM(EPWM5_BASE, pwmmax, deadBandCount, resolverPhaseDelay);
        initPhaseEPWM(EPWM6_BASE, pwmmax, deadBandCount, resolverPhaseDelay);
        initPhaseEPWM(EPWM7_BASE, pwmmax, deadBandCount, resolverPhaseDelay);

        // According to section 15.4.3.3 of the TMS320F2837xD Technical
        // Reference Manual no synchonisation chain should exceed 4 units. Our
        // synchonisation chains are:
        //   EPWM1 -> EPWM4 -> EPWM5 -> EPWM6
        //   EPWM1 -> EPWM4 -> EPWM7

        //
        // EPWM1 SYNCO is generated on CTR=0
        //
        EPWM_setSyncOutPulseMode(
            EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

        //
        // EPWM4 uses the EPWM 1 SYNCO as its SYNCIN.
        // EPWM4 SYNCO is generated from its SYNCIN
        //
        SysCtl_setSyncInputConfig(
            SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
        EPWM_setSyncOutPulseMode(
            EPWM4_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);

        //
        // EPWM5 uses the EPWM 4 SYNCO as its SYNCIN.
        // EPWM5 SYNCO is generated from its SYNCIN
        //
        EPWM_setSyncOutPulseMode(
            EPWM5_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);

        //
        // EPWM6 uses the EPWM 5 SYNCO as its SYNCIN.
        // This is the end of a sync chain so no config required
        //

        //
        // EPWM7 uses EPWM4 SYNCO as its SYNCIN
        //
        SysCtl_setSyncInputConfig(
            SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM4SYNCOUT);

        //
        // Enable all phase shifts.
        //
        EPWM_enablePhaseShiftLoad(EPWM4_BASE);
        EPWM_enablePhaseShiftLoad(EPWM5_BASE);
        EPWM_enablePhaseShiftLoad(EPWM6_BASE);
        EPWM_enablePhaseShiftLoad(EPWM7_BASE);
    }
    else
    {
        // Store the EPWM modules used for each phase for normal operation
        sm_phaseAEpwmBase = EPWM2_BASE;
        sm_phaseBEpwmBase = EPWM3_BASE;
        sm_phaseCEpwmBase = EPWM4_BASE;

        //
        // Initialize EPWM1 as master clock and resolver exciter output
        //
        initResolverEPWM(EPWM1_BASE, pwmmax);

        //
        // Initialize EPWM2, EPWM3 and EPWM4 as phase PWM outputs
        //
        initPhaseEPWM(EPWM2_BASE, pwmmax, deadBandCount, resolverPhaseDelay);
        initPhaseEPWM(EPWM3_BASE, pwmmax, deadBandCount, resolverPhaseDelay);
        initPhaseEPWM(EPWM4_BASE, pwmmax, deadBandCount, resolverPhaseDelay);

        // According to section 15.4.3.3 of the TMS320F2837xD Technical
        // Reference Manual no synchonisation chain should exceed 4 units. Our
        // synchonisation chains are:
        //   EPWM1 -> EPWM4
        //   EPWM1 -> EPWM2 -> EPWM3

        //
        // EPWM1 SYNCO is generated on CTR=0
        //
        EPWM_setSyncOutPulseMode(
            EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

        //
        // EPWM2 uses the EPWM1 SYNCO as its SYNCIN.
        // EPWM2 SYNCO is generated from its SYNCIN
        //
        EPWM_setSyncOutPulseMode(
            EPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);

        //
        // EPWM3 uses the EPWM2 SYNCO as its SYNCIN.
        //
        EPWM_setSyncOutPulseMode(
            EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);

        //
        // EPWM4 uses EPWM1 SYNCO as its SYNCIN
        //
        // This is the end of a sync chain so no config required

        //
        // Enable all phase shifts.
        //
        EPWM_enablePhaseShiftLoad(EPWM2_BASE);
        EPWM_enablePhaseShiftLoad(EPWM3_BASE);
        EPWM_enablePhaseShiftLoad(EPWM4_BASE);
    }

    //
    // Configure the analog capture of motor control signals to happen at the
    // centre of the phase PWM. The centre corresponds to the pwm counter
    // counting up to the configured value.
    // The MotorAnalogCapture class is responsible for configuring all ADC
    // channels it requires.
    // We scale the ADC trigger event prescale to allow operation of unoptimised
    // code.
    //
    MotorAnalogCapture::Init();

    const uint32_t motorAdcPwm = IsTeslaM3Inverter() ? EPWM5_BASE : EPWM2_BASE;
    EPWM_disableADCTrigger(motorAdcPwm, EPWM_SOC_A);
    EPWM_setADCTriggerSource(motorAdcPwm, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(motorAdcPwm, EPWM_SOC_A, 15U);

    MotorAnalogCapture::ConfigureSoc(
        IsTeslaM3Inverter() ? ADC_TRIGGER_EPWM5_SOCA : ADC_TRIGGER_EPWM2_SOCA);

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Ensure we have initialised the performance counter before we start
    // getting interrupts
    PerformanceCounter::Init();

    //
    // Configure interrupts for the main motor control loop to run when the ADC
    // has new data
    //
    Interrupt_register(INT_ADCA1, &motor_control_adc_isr);
    Interrupt_enable(INT_ADCA1);

    //
    // Set the ADC event going
    //
    EPWM_enableADCTrigger(motorAdcPwm, EPWM_SOC_A);

    // Return the pwm frequency. Because we use the up-down count mode divide by
    // 2 * the period
    return EPWM_FREQ / (2 * pwmmax);
}

/**
 * Set up the timer for AC heat use
 */
void PwmDriver::AcHeatTimerSetup()
{
    // Not relevant to this HW
}

/**
 * Enable AC heat
 * \param ampnom    Notminal current to use for AC heat
 */
void PwmDriver::AcHeat(s32fp ampnom)
{
    // Not relevant to this HW
}

/**
 * Set the charge current target
 * \param dc    Target current
 */
void PwmDriver::SetChargeCurrent(int16_t dc)
{
    // Not relevant to this HW
}

/**
 * Obtain how many SYSCLOCK cycles we spend running the main control loop
 * \return Number of ticks
 */
int32_t PwmDriver::GetCpuLoad()
{
    return execTicks;
}

/**
 * Reset the CPU load stored value for when the PWM is not running
 */
void PwmDriver::ResetCpuLoad()
{
    execTicks = 0;
}

} // namespace c2000
