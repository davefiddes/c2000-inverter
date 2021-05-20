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

#include "my_fp.h"
#include "my_math.h"
#include "sine_core.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::FloatNear;

TEST(FixedPoint, TestMacros)
{
    EXPECT_EQ(
        FP_MUL(FP_FROMFLT(5.5), FP_FROMFLT(2.03125)),
        FP_FROMFLT(5.5 * 2.03125));
    EXPECT_EQ(
        FP_MUL(FP_FROMFLT(5.5), FP_FROMFLT(2.03225)),
        FP_FROMFLT(5.5 * 2.03125));
    EXPECT_EQ(
        FP_DIV(FP_FROMFLT(5.5), FP_FROMFLT(2.03125)),
        FP_FROMFLT(5.5 / 2.03125));
    EXPECT_EQ(FP_TOINT(FP_FROMFLT(12345.6)), 12345);
    EXPECT_THAT(FP_TOFLT(FP_FROMFLT(12345.6)), FloatNear(12345.6, 0.01));
}

TEST(FixedPoint, TestItoa)
{
    char buf[10];
    EXPECT_STREQ(fp_itoa(buf, FP_FROMFLT(2.03125)), "2.03");
    EXPECT_STREQ(fp_itoa(buf, FP_FROMFLT(-2.125)), "-2.12");
    EXPECT_STREQ(fp_itoa(buf, FP_FROMFLT(2.15624)), "2.12");
    EXPECT_STREQ(fp_itoa(buf, FP_FROMFLT(2.15625)), "2.15");
}

TEST(FixedPoint, TestAtoi)
{
    EXPECT_EQ(fp_atoi("-2.5", FRAC_DIGITS), FP_FROMFLT(-2.5));
    EXPECT_EQ(fp_atoi("2.155", FRAC_DIGITS), FP_FROMFLT(2.16));
}

TEST(FixedPoint, TestMedian3)
{
    EXPECT_EQ(MEDIAN3(1, 2, 3), 2);
    EXPECT_EQ(MEDIAN3(3, 2, 1), 2);
    EXPECT_EQ(MEDIAN3(1, 3, 2), 2);
    EXPECT_EQ(MEDIAN3(2, 3, 1), 2);
    EXPECT_EQ(MEDIAN3(2, 1, 3), 2);
}

TEST(FixedPoint, TestAtan2)
{
    uint16_t res;
    EXPECT_EQ(SineCore::Atan2(4096, 0), 0);         // 0°
    EXPECT_EQ(SineCore::Atan2(2896, 2896), 8192);   // 45°
    EXPECT_EQ(SineCore::Atan2(-4096, 0), 32768);    // 180°
    EXPECT_EQ(SineCore::Atan2(2048, -3547), 54613); // 300°
    EXPECT_EQ(SineCore::Atan2(2048, 3547), 10922);  // 60°
}

TEST(FixedPoint, TestLn)
{
    EXPECT_EQ(fp_ln(1), 0);
    EXPECT_THAT(FP_TOFLT(fp_ln(5389)), FloatNear(8.592115118, 0.1));
    EXPECT_THAT(FP_TOFLT(fp_ln(8290)), FloatNear(9.022805248, 0.1));
}
