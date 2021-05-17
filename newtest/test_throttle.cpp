/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2021 David J. Fiddes <D.J@fiddes.net>
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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
#include "my_fp.h"
#include "my_math.h"
#include "throttle.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::FloatNear;
using ::testing::Le;
using ::testing::Test;

/**
 * Common test fixture
 */
class TestThrottle : public ::testing::Test
{
protected:
    void SetUp() override
    {
        Throttle::potmin[0] = 1000;
        Throttle::potmax[0] = 2000;
        Throttle::potmin[1] = 3000;
        Throttle::potmax[1] = 4000;
        Throttle::brknom = FP_FROMFLT(30.0);
        Throttle::brknompedal = FP_FROMFLT(-50.0);
        Throttle::regenRamp = FP_FROMFLT(25.0);
        Throttle::brkmax = FP_FROMFLT(50.0);
        Throttle::idleSpeed = 100;
        Throttle::speedkp = FP_FROMFLT(0.25);
        Throttle::speedflt = 5;
        Throttle::idleThrotLim = FP_FROMFLT(30.0);
        Throttle::throtmin = FP_FROMFLT(-100.0);
        Throttle::throtmax = FP_FROMFLT(100.0);
        Throttle::throttleRamp = FP_FROMFLT(50.0);
    }
};

TEST_F(TestThrottle, PotRangeLimits)
{
    // Check throttle values inside the expected range
    int potval = 1500;
    EXPECT_TRUE(Throttle::CheckAndLimitRange(&potval, 0));
    EXPECT_EQ(potval, 1500);

    potval = 3000;
    EXPECT_TRUE(Throttle::CheckAndLimitRange(&potval, 1));
    EXPECT_EQ(potval, 3000);

    // Check values way below the expected range
    potval = 0;
    EXPECT_FALSE(Throttle::CheckAndLimitRange(&potval, 0));
    EXPECT_EQ(potval, 1000);

    potval = 2000;
    EXPECT_FALSE(Throttle::CheckAndLimitRange(&potval, 1));
    EXPECT_EQ(potval, 3000);

    // Check values way above the expected range
    potval = 3000;
    EXPECT_FALSE(Throttle::CheckAndLimitRange(&potval, 0));
    EXPECT_EQ(potval, 1000);

    // Check values a small amount below the expected range are passed but
    // clamped
    potval = 950;
    EXPECT_TRUE(Throttle::CheckAndLimitRange(&potval, 0));
    EXPECT_EQ(potval, 1000);

    potval = 2900;
    EXPECT_TRUE(Throttle::CheckAndLimitRange(&potval, 1));
    EXPECT_EQ(potval, 3000);

    // Check values a small amount above the expected range are passed but
    // clamped
    potval = 2100;
    EXPECT_TRUE(Throttle::CheckAndLimitRange(&potval, 0));
    EXPECT_EQ(potval, 2000);

    potval = 4096;
    EXPECT_TRUE(Throttle::CheckAndLimitRange(&potval, 1));
    EXPECT_EQ(potval, 4000);
}

TEST_F(TestThrottle, PotReadingToPercent)
{
    EXPECT_EQ(Throttle::DigitsToPercent(1500, 0), FP_FROMFLT(50.0));
    EXPECT_EQ(Throttle::DigitsToPercent(3200, 1), FP_FROMFLT(20.0));
    EXPECT_EQ(Throttle::DigitsToPercent(4000, 1), FP_FROMFLT(100.0));
}

TEST_F(TestThrottle, BasicThrottleAccelerate)
{
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(10.0), FP_FROMFLT(0.0), false),
        FPNear(0.0, 1));
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(40.0), FP_FROMFLT(0.0), false),
        FPNear(14.0, 1));
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(70.0), FP_FROMFLT(0.0), false),
        FPNear(57.0, 1));
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(100.0), FP_FROMFLT(0.0), false),
        FPNear(100.0, 1));
}

TEST_F(TestThrottle, TestBrkPedal)
{
    // Not quite sure why this old test makes multiple calls. Perhaps ramping of
    // throttles happened in CalcThrottle originally
    // TODO: Investigate history
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(50.0), FP_FROMFLT(0.0), true),
        FPNear(0.0, 1));
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(50.0), FP_FROMFLT(0.0), true),
        FPNear(0.0, 1));
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(50.0), FP_FROMFLT(0.0), true),
        FPNear(0.0, 1));
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(50.0), FP_FROMFLT(0.0), true),
        FPNear(0.0, 1));
}

TEST_F(TestThrottle, TestRegen)
{
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(100.0), FP_FROMFLT(0.0), false),
        FPNear(100.0, 1));
    EXPECT_THAT(
        Throttle::CalcThrottle(FP_FROMFLT(0.0), FP_FROMFLT(0.0), false),
        FPNear(0.0, 1));
}

TEST_F(TestThrottle, ThrottleRamp)
{
    // Mash the throttle and verify we take two calls (assumed to be at 10ms
    // intervals) to reach 100%
    EXPECT_THAT(Throttle::RampThrottle(FP_FROMFLT(120.0)), FPNear(50.0, 1));
    EXPECT_THAT(Throttle::RampThrottle(FP_FROMFLT(120.0)), FPNear(100.0, 1));
}
