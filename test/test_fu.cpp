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

#include "fu.h"
#include "matcherhelper.h"
#include "my_fp.h"
#include "my_math.h"
#include <gtest/gtest.h>

using ::testing::FloatNear;

TEST(TestMotorVoltage, TestNoBoost)
{
    MotorVoltage::SetBoost(0);
    MotorVoltage::SetMaxAmp(10000);
    MotorVoltage::SetWeakeningFrq(1000.0);

    // Test above minfrq of 0.2
    EXPECT_EQ(MotorVoltage::GetAmp(FP_FROMFLT(1.0)), 10);
}

TEST(TestMotorVoltage, TestBoost1)
{
    MotorVoltage::SetBoost(10);
    MotorVoltage::SetMaxAmp(10000);
    MotorVoltage::SetWeakeningFrq(1000.0);

    // Test below old minfrq parameter
    EXPECT_EQ(MotorVoltage::GetAmp(0), 0);

    // Test above minfrq of 0.2
    EXPECT_EQ(MotorVoltage::GetAmp(FP_FROMFLT(1.0)), 19);
}

TEST(TestMotorVoltage, TestFU1)
{
    MotorVoltage::SetBoost(10);
    MotorVoltage::SetMaxAmp(10000);
    MotorVoltage::SetWeakeningFrq(15.0);

    EXPECT_THAT(
        MotorVoltage::GetAmp(FP_FROMFLT(0.3)),
        IntNear(((10000 - 10) / 15.0) * 0.3 + 10, 20));
}

TEST(TestMotorVoltage, TestFU2)
{
    MotorVoltage::SetBoost(1700);
    MotorVoltage::SetMaxAmp(10000);
    MotorVoltage::SetWeakeningFrq(1000.0);

    float fac = (10000 - 1700) / 1000.0;

    EXPECT_THAT(
        MotorVoltage::GetAmp(FP_FROMFLT(5)), IntNear(fac * 5.0 + 1700, 1));
    EXPECT_THAT(
        MotorVoltage::GetAmp(FP_FROMFLT(9.5)), IntNear(fac * 9.5 + 1700, 1));
    // Slightly loose constraints due to fixed point maths precision
    EXPECT_THAT(
        MotorVoltage::GetAmp(FP_FROMFLT(500)), IntNear(fac * 500.0 + 1700, 11));
    EXPECT_EQ(MotorVoltage::GetAmp(FP_FROMFLT(1200)), 10000);
}

TEST(TestMotorVoltage, TestFUPerc)
{
    MotorVoltage::SetBoost(230);
    MotorVoltage::SetMaxAmp(10000);
    MotorVoltage::SetWeakeningFrq(1000.0);

    EXPECT_THAT(
        MotorVoltage::GetAmpPerc(FP_FROMFLT(5), FP_FROMFLT(50)),
        IntNear((((10000 - 230) / 1000.0) * 5 + 230) / 2, 5));
    EXPECT_EQ(
        MotorVoltage::GetAmpPerc(FP_FROMFLT(2350), FP_FROMFLT(50)), 10000);
}
