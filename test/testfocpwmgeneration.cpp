/*
 * This file is part of the stm32_sine project.
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

#include "errormessage.h"
#include "focpwmgeneration.h"
#include "matcherhelper.h"
#include "mockanain.h"
#include "mockencoder.h"
#include "mockpwmdriver.h"
#include <gtest/gtest.h>

using PwmGeneration = FocPwmGeneration<MockAnaIn, MockEncoder, MockPwmDriver>;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::FloatNear;
using ::testing::Ge;
using ::testing::InSequence;
using ::testing::Le;
using ::testing::Ne;
using ::testing::Return;
using ::testing::Test;

void parm_Change([[maybe_unused]] Param::PARAM_NUM paramNum)
{
}

static const uint16_t DefaultPwmFrequency = 8789; // Hz

/**
 * Common test fixture
 */
class TestFocPwmGeneration : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ErrorMessage::ResetAll();
        ErrorMessage::SetTime(1);
        EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
        Param::LoadDefaults();
    }
};

TEST_F(TestFocPwmGeneration, InitCurrentOffsetSimple)
{
    MockPwmDriverImpl pwmDriver;
    // With a default current gain of 4.7 [dig/A] and over current limit of 100
    // [A] these should be 1578 and 2518. Tolerance of 3 digits to allow fixed
    // point maths to work
    EXPECT_CALL(
        pwmDriver, SetOverCurrentLimits(IntNear(1578, 3), IntNear(2518, 3)));

    PwmGeneration::SetCurrentOffset(2048, 2048);

    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

TEST_F(TestFocPwmGeneration, InitInvalidCurrentOffset)
{
    MockPwmDriverImpl pwmDriver;
    // With a default current gain of 4.7 [dig/A] and over current limit of 100
    // [A] these should be 1780 and 2720. There is assumed to only be a single
    // comparator so the two offsets have to be averaged. Tolerance of 3 digits
    // to allow fixed point maths to work
    EXPECT_CALL(
        pwmDriver, SetOverCurrentLimits(IntNear(1780, 3), IntNear(2720, 3)));

    // Current offsets are expected to be within the mid-point of the ADC range
    // approximately. Use deliberately out of range values
    PwmGeneration::SetCurrentOffset(1500, 3000);

    EXPECT_TRUE(ErrorMessage::HasErrorBeenPosted(ERR_HICUROFS1));
    EXPECT_TRUE(ErrorMessage::HasErrorBeenPosted(ERR_HICUROFS2));
}

TEST_F(TestFocPwmGeneration, SetRunMode)
{
    // The rather complicated expectations needed to start up the PWM driver
    MockPwmDriverImpl pwmDriver;
    {
        InSequence seq;

        EXPECT_CALL(pwmDriver, TimerSetup)
            .WillOnce(Return(DefaultPwmFrequency));
        EXPECT_CALL(pwmDriver, DriverInit);
        EXPECT_CALL(pwmDriver, EnableOutput);
    }

    MockEncoderImpl encoder;
    EXPECT_CALL(encoder, SetPwmFrequency(DefaultPwmFrequency));

    // Flip into normal RUN mode
    PwmGeneration::SetOpmode(Modes::RUN);

    // Verify that PWM get turned off correctly
    EXPECT_CALL(pwmDriver, ResetCpuLoad);
    EXPECT_CALL(pwmDriver, DisableOutput);
    PwmGeneration::SetOpmode(Modes::OFF);

    // Make sure that turning off PWM is idempotent
    EXPECT_CALL(pwmDriver, DisableOutput).Times(0);
    EXPECT_CALL(pwmDriver, DriverInit).Times(0);
    PwmGeneration::SetOpmode(Modes::OFF);

    // Teardown verify that nothing untoward was reported
    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

TEST_F(TestFocPwmGeneration, SetBoostMode)
{
    // The rather complicated expectations needed to start up the PWM driver
    MockPwmDriverImpl pwmDriver;
    {
        InSequence seq;

        EXPECT_CALL(pwmDriver, TimerSetup)
            .WillOnce(Return(DefaultPwmFrequency));
        EXPECT_CALL(pwmDriver, DriverInit);
        EXPECT_CALL(pwmDriver, DisableOutput);
        EXPECT_CALL(pwmDriver, EnableChargeOutput);
    }

    // Charging doesn't need an encoder obviously - lets check that
    MockEncoderImpl encoder;
    EXPECT_CALL(encoder, SetPwmFrequency(DefaultPwmFrequency));

    // Flip into boost converter mode
    PwmGeneration::SetOpmode(Modes::BOOST);

    // Verify that PWM get turned off correctly
    EXPECT_CALL(pwmDriver, ResetCpuLoad);
    EXPECT_CALL(pwmDriver, DisableOutput);
    PwmGeneration::SetOpmode(Modes::OFF);

    // Make sure that turning off PWM is idempotent
    EXPECT_CALL(pwmDriver, DisableOutput).Times(0);
    EXPECT_CALL(pwmDriver, DriverInit).Times(0);
    PwmGeneration::SetOpmode(Modes::OFF);

    // Teardown verify that nothing untoward was reported
    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

TEST_F(TestFocPwmGeneration, SetACHeatMode)
{
    // The rather complicated expectations needed to start up the PWM driver
    MockPwmDriverImpl pwmDriver;
    {
        InSequence seq;

        EXPECT_CALL(pwmDriver, TimerSetup)
            .WillOnce(Return(DefaultPwmFrequency));
        EXPECT_CALL(pwmDriver, DriverInit);
        EXPECT_CALL(pwmDriver, AcHeatTimerSetup);
        EXPECT_CALL(pwmDriver, DisableOutput);
        EXPECT_CALL(pwmDriver, EnableACHeatOutput);
    }

    // Charging doesn't need an encoder obviously - lets check that
    MockEncoderImpl encoder;
    EXPECT_CALL(encoder, SetPwmFrequency(DefaultPwmFrequency));

    // Flip into boost converter mode
    PwmGeneration::SetOpmode(Modes::ACHEAT);

    // Verify that PWM get turned off correctly
    EXPECT_CALL(pwmDriver, ResetCpuLoad);
    EXPECT_CALL(pwmDriver, DisableOutput);
    PwmGeneration::SetOpmode(Modes::OFF);

    // Make sure that turning off PWM is idempotent
    EXPECT_CALL(pwmDriver, DisableOutput).Times(0);
    EXPECT_CALL(pwmDriver, DriverInit).Times(0);
    PwmGeneration::SetOpmode(Modes::OFF);

    // Teardown verify that nothing untoward was reported
    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

TEST_F(TestFocPwmGeneration, ManualModeRun)
{
    MockPwmDriverImpl pwmDriver;

    EXPECT_CALL(pwmDriver, SetOverCurrentLimits);
    PwmGeneration::SetCurrentOffset(2048, 2048);

    {
        InSequence seq;

        EXPECT_CALL(pwmDriver, TimerSetup)
            .WillOnce(Return(DefaultPwmFrequency));
        EXPECT_CALL(pwmDriver, DriverInit);
        EXPECT_CALL(pwmDriver, EnableOutput);
    }

    // Set up an encoder that is going forwards
    MockEncoderImpl encoder;
    EXPECT_CALL(encoder, SetPwmFrequency(DefaultPwmFrequency));

    PwmGeneration::SetOpmode(Modes::MANUAL);

    // Ensure the system thinks we should be going forwards
    Param::SetInt(Param::dir, 1);

    // Provide some neutral values for the phase currents
    MockAnaIn::il1.Set(2048);
    MockAnaIn::il2.Set(2048);

    // Provide a valid DC bus voltage
    MockAnaIn::udc.Set(3500);

    // We need the pole pair ratio set to correctly calculate the rotation
    // frequency
    PwmGeneration::SetPolePairRatio(1);

    // initialise the controller gains from the default parameters
    PwmGeneration::SetControllerGains(
        Param::GetInt(Param::curkp),
        Param::GetInt(Param::curki),
        Param::GetInt(Param::fwkp));

    // Put in a bit of Q current to pretend were looking to check the syncofs
    Param::Set(Param::manualiq, FP_FROMFLT(0.6));

    // We need to wait for a certain number of Run() cycles before the system is
    // ready to run
    static const int StartupWait = DefaultPwmFrequency / 2 - 1;

    EXPECT_CALL(pwmDriver, SetOverCurrentLimits).Times(AtLeast(2));
    EXPECT_CALL(encoder, SeenNorthSignal)
        .Times(StartupWait)
        .WillRepeatedly(Return(true));
    EXPECT_CALL(encoder, GetRotorFrequency)
        .Times(StartupWait)
        .WillRepeatedly(Return(10));
    EXPECT_CALL(encoder, UpdateRotorAngle(1)).Times(StartupWait);
    EXPECT_CALL(encoder, GetRotorAngle)
        .Times(StartupWait)
        .WillRepeatedly(Return(32767));
    EXPECT_CALL(pwmDriver, DisableMasterOutput).Times(StartupWait);
    EXPECT_CALL(pwmDriver, EnableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, SetPhasePwm).Times(StartupWait);

    for (int i = 0; i < StartupWait; i++)
    {
        PwmGeneration::Run();
    }

    // Now we should be through the startup wait we can verify that PWM is
    // actually output
    EXPECT_CALL(encoder, SeenNorthSignal).WillRepeatedly(Return(true));
    EXPECT_CALL(encoder, GetRotorFrequency).WillRepeatedly(Return(10));
    EXPECT_CALL(encoder, UpdateRotorAngle(1));
    EXPECT_CALL(encoder, GetRotorAngle).WillOnce(Return(32767));
    EXPECT_CALL(pwmDriver, DisableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, EnableMasterOutput);
    EXPECT_CALL(pwmDriver, SetPhasePwm(Ne(2048), Ne(2048), Ne(2048)));

    PwmGeneration::Run();

    // Check that we turn off cleanly
    EXPECT_CALL(pwmDriver, ResetCpuLoad);
    EXPECT_CALL(pwmDriver, DisableOutput);
    PwmGeneration::SetOpmode(Modes::OFF);

    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

TEST_F(TestFocPwmGeneration, NormalModeRun)
{
    MockPwmDriverImpl pwmDriver;

    EXPECT_CALL(pwmDriver, SetOverCurrentLimits);
    PwmGeneration::SetCurrentOffset(2048, 2048);

    {
        InSequence seq;

        EXPECT_CALL(pwmDriver, TimerSetup)
            .WillOnce(Return(DefaultPwmFrequency));
        EXPECT_CALL(pwmDriver, DriverInit);
        EXPECT_CALL(pwmDriver, EnableOutput);
    }

    // Set up an encoder that is going forwards
    MockEncoderImpl encoder;
    EXPECT_CALL(encoder, SetPwmFrequency(DefaultPwmFrequency));

    PwmGeneration::SetOpmode(Modes::RUN);

    // Ensure the system thinks we should be going forwards
    Param::SetInt(Param::dir, 1);

    // Half-throttle accelerate I guess
    PwmGeneration::SetTorquePercent(FP_FROMFLT(0.5));

    // Provide some neutral values for the phase currents
    MockAnaIn::il1.Set(2048);
    MockAnaIn::il2.Set(2048);

    // Provide a valid DC bus voltage
    MockAnaIn::udc.Set(3500);

    // We need the pole pair ratio set to correctly calculate the rotation
    // frequency
    PwmGeneration::SetPolePairRatio(1);

    // initialise the controller gains from the default parameters
    PwmGeneration::SetControllerGains(
        Param::GetInt(Param::curkp),
        Param::GetInt(Param::curki),
        Param::GetInt(Param::fwkp));

    // We need to wait for a certain number of Run() cycles before the system is
    // ready to run
    static const int StartupWait = DefaultPwmFrequency / 2 - 1;

    EXPECT_CALL(pwmDriver, SetOverCurrentLimits).Times(AtLeast(2));
    EXPECT_CALL(encoder, SeenNorthSignal)
        .Times(StartupWait)
        .WillRepeatedly(Return(true));
    EXPECT_CALL(encoder, GetRotorFrequency)
        .Times(StartupWait)
        .WillRepeatedly(Return(10));
    EXPECT_CALL(encoder, UpdateRotorAngle(1)).Times(StartupWait);
    EXPECT_CALL(encoder, GetRotorAngle)
        .Times(StartupWait)
        .WillRepeatedly(Return(32767));
    EXPECT_CALL(pwmDriver, DisableMasterOutput).Times(StartupWait);
    EXPECT_CALL(pwmDriver, EnableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, SetPhasePwm).Times(StartupWait);

    for (int i = 0; i < StartupWait; i++)
    {
        PwmGeneration::Run();
    }

    // Now we should be through the startup wait we can verify that PWM is
    // actually output
    EXPECT_CALL(encoder, SeenNorthSignal).WillRepeatedly(Return(true));
    EXPECT_CALL(encoder, GetRotorFrequency).WillRepeatedly(Return(10));
    EXPECT_CALL(encoder, UpdateRotorAngle(1));
    EXPECT_CALL(encoder, GetRotorAngle).WillOnce(Return(32767));
    EXPECT_CALL(pwmDriver, DisableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, EnableMasterOutput);
    EXPECT_CALL(pwmDriver, SetPhasePwm(Ne(2048), Ne(2048), Ne(2048)));

    PwmGeneration::Run();

    // Check that we turn off cleanly
    EXPECT_CALL(pwmDriver, ResetCpuLoad);
    EXPECT_CALL(pwmDriver, DisableOutput);
    PwmGeneration::SetOpmode(Modes::OFF);

    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

TEST_F(TestFocPwmGeneration, BuckChargeModeRun)
{
    MockPwmDriverImpl pwmDriver;

    EXPECT_CALL(pwmDriver, SetOverCurrentLimits);
    PwmGeneration::SetCurrentOffset(2048, 2048);

    {
        InSequence seq;

        EXPECT_CALL(pwmDriver, TimerSetup)
            .WillOnce(Return(DefaultPwmFrequency));
        EXPECT_CALL(pwmDriver, DriverInit);
        EXPECT_CALL(pwmDriver, DisableOutput);
        EXPECT_CALL(pwmDriver, EnableChargeOutput);
    }

    // We shouldn't need an encoder but basic setup is established
    MockEncoderImpl encoder;
    EXPECT_CALL(encoder, SetPwmFrequency(DefaultPwmFrequency));

    PwmGeneration::SetOpmode(Modes::BUCK);

    // Ensure the system thinks we should be stationary
    Param::SetInt(Param::dir, 0);

    // Provide some neutral values for the phase currents
    MockAnaIn::il1.Set(2048);
    MockAnaIn::il2.Set(2048);

    // Provide a valid DC bus voltage
    MockAnaIn::udc.Set(3500);

    // initialise the controller gains from the default parameters
    PwmGeneration::SetControllerGains(
        Param::GetInt(Param::curkp),
        Param::GetInt(Param::curki),
        Param::GetInt(Param::fwkp));

    // Configure a charge current
    PwmGeneration::SetChargeCurrent(FP_FROMFLT(0.5));

    // We need to wait for a certain number of Run() cycles before the system is
    // ready to run
    static const int StartupWait = DefaultPwmFrequency / 2 - 1;

    EXPECT_CALL(pwmDriver, SetOverCurrentLimits).Times(0);
    EXPECT_CALL(encoder, SeenNorthSignal).Times(0);
    EXPECT_CALL(encoder, GetRotorFrequency).Times(0);
    EXPECT_CALL(encoder, UpdateRotorAngle(1)).Times(0);
    EXPECT_CALL(encoder, GetRotorAngle).Times(0);
    EXPECT_CALL(pwmDriver, DisableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, EnableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, SetPhasePwm).Times(0);
    EXPECT_CALL(pwmDriver, SetChargeCurrent).Times(StartupWait);

    for (int i = 0; i < StartupWait; i++)
    {
        PwmGeneration::Run();
    }

    // Now we should be through the startup wait we can verify that PWM is
    // actually output through the charge output but not the regular motor PWM
    EXPECT_CALL(encoder, SeenNorthSignal).Times(0);
    EXPECT_CALL(encoder, GetRotorFrequency).Times(0);
    EXPECT_CALL(encoder, UpdateRotorAngle(1)).Times(0);
    EXPECT_CALL(encoder, GetRotorAngle).Times(0);
    EXPECT_CALL(pwmDriver, DisableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, EnableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, SetPhasePwm).Times(0);
    EXPECT_CALL(pwmDriver, SetChargeCurrent);

    PwmGeneration::Run();

    // Check that we turn off cleanly
    EXPECT_CALL(pwmDriver, ResetCpuLoad);
    EXPECT_CALL(pwmDriver, DisableOutput);
    PwmGeneration::SetOpmode(Modes::OFF);

    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}
