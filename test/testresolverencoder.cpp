/*
 * This file is part of the stm32_sine project.
 *
 * Copyright (C) 2022 David J. Fiddes <D.J@fiddes.net>
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

#include "matcherhelper.h"
#include "mockresolversample.h"
#include "resolverencoder.h"
#include "utils.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <utility>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::FloatNear;
using ::testing::Ge;
using ::testing::InSequence;
using ::testing::Le;
using ::testing::Ne;
using ::testing::Return;
using ::testing::Test;

typedef encoder::ResolverEncoder<MockResolverSample> Resolver;

//! Use the same PWM frequency as found on the Tesla M3 inverter
static const uint16_t DefaultPwmFrequency =
    100000000 / (2 * 4096); // 12.207 kHz

//! Number of update cycles to wait before reporting valid readings
static const int StartupCount = 4000;

//! Frequency at which we update the resolver frequency
static const int FrequencyUpdate = 100;

//! Number of PWM updates for each frequency update
static const int FreqUpdatesPerPwm = DefaultPwmFrequency / FrequencyUpdate;

//
//! Common test fixture
//
class TestResolverEncoder : public ::testing::Test
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

//! Test the initial state of the resolver encoder
TEST(TestResolverEncoder, Initialisation)
{
    MockResolverSampleImpl sample;
    EXPECT_CALL(sample, ResolverSine).Times(0);
    EXPECT_CALL(sample, ResolverCosine).Times(0);

    Resolver::Reset();

    // Verify the initial state of the encoder - we shouldn't be going anywhere
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    // Teardown verify that nothing untoward was reported
    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

//! Test a static position with the PWM configured and running
TEST(TestResolverEncoder, StaticPosition)
{
    MockResolverSampleImpl sample;
    EXPECT_CALL(sample, ResolverSine).Times(0);
    EXPECT_CALL(sample, ResolverCosine).Times(0);

    Resolver::Reset();

    Resolver::SetPwmFrequency(DefaultPwmFrequency);

    // Initial state verification
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    // Figure out a sample position
    const float    TestAngle = 123.456;  // degrees
    const int16_t  TestAmplitude = 1800; // counts
    const int16_t  SineValue = TestAmplitude * sin(TestAngle * (pi / 180));
    const int16_t  CosineValue = TestAmplitude * cos(TestAngle * (pi / 180));
    const uint16_t ComputedAngle = (TestAngle * 65536) / 360;

    // Simulate an update from the main control loop indicating the first
    // set of sine/cosine values are available
    EXPECT_CALL(sample, ResolverSine).Times(1).WillOnce(Return(SineValue));
    EXPECT_CALL(sample, ResolverCosine).Times(1).WillOnce(Return(CosineValue));
    Resolver::UpdateRotorAngle(1);

    // Verify no position or speed readings are available yet
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    EXPECT_CALL(sample, ResolverSine)
        .Times(StartupCount)
        .WillRepeatedly(Return(SineValue));
    EXPECT_CALL(sample, ResolverCosine)
        .Times(StartupCount)
        .WillRepeatedly(Return(CosineValue));

    // Verify that nothing further changes as we complete the inital settling
    // period and start sampling for real
    for (int i = 0; i < StartupCount; i++)
    {
        // Simulate an update from the main control loop indicating new
        // sine/cosine values are available
        Resolver::UpdateRotorAngle(1);

        // Simulate a periodic update of frequency from an auxiliary control
        // loop
        if (i % FreqUpdatesPerPwm == 0)
        {
            Resolver::UpdateRotorFrequency(FrequencyUpdate);
        }

        // State verification
        EXPECT_EQ(Resolver::SeenNorthSignal(), true);
        EXPECT_THAT(Resolver::GetRotorAngle(), IntNear(ComputedAngle, 1));
        EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
        EXPECT_EQ(Resolver::GetSpeed(), 0);
        EXPECT_EQ(Resolver::GetFullTurns(), 0);
        EXPECT_EQ(Resolver::GetRotorDirection(), 0);
    }

    // Teardown verify that nothing untoward was reported
    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

//! Test a static position with insufficient amplitude
TEST(TestResolverEncoder, LowAmplitude)
{
    Resolver::Reset();

    Resolver::SetPwmFrequency(DefaultPwmFrequency);

    // Initial state verification
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    // Figure out a sample position
    const float    TestAngle = 56.789;  // degrees
    const int16_t  TestAmplitude = 200; // counts
    const int16_t  SineValue = TestAmplitude * sin(TestAngle * (pi / 180));
    const int16_t  CosineValue = TestAmplitude * cos(TestAngle * (pi / 180));

    // Simulate an a low amplitude input to the resolver sine/cosine signals
    MockResolverSampleImpl sample;
    EXPECT_CALL(sample, ResolverSine)
        .Times(StartupCount + 1)
        .WillRepeatedly(Return(SineValue));
    EXPECT_CALL(sample, ResolverCosine)
        .Times(StartupCount + 1)
        .WillRepeatedly(Return(CosineValue));

    // Complete the initial sampling period plus 1 sample
    for (int i = 0; i < StartupCount + 1; i++)
    {
        // Simulate an update from the main control loop indicating new
        // sine/cosine values are available
        Resolver::UpdateRotorAngle(1);

        // Simulate a periodic update of frequency from an auxiliary control
        // loop
        if (i % FreqUpdatesPerPwm == 0)
        {
            Resolver::UpdateRotorFrequency(FrequencyUpdate);
        }
    }

    // Verify final angle
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    // Verify that we trigger hit an error because of the low amplitude input
    EXPECT_EQ(ErrorMessage::GetLastError(), ERR_LORESAMP);
}
