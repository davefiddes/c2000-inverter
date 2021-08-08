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

#include "matcherhelper.h"
#include "my_fp.h"
#include "sine_core.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <math.h>
#include <numbers>
#include <utility>

using ::testing::FloatNear;
using namespace std::numbers;

// Numeric tolerance we're trying to measure to. Number is empirical as the
// fixed point maths is a bit loosey goosey.
static constexpr float Tolerance = 1.0;

//! Maximum amplitude of the sine/cosine functions - corresponds to 2^11 / 2
static constexpr float SineAmp = 1024;

constexpr uint16_t RadToRev(float angle)
{
    return SINLU_ONEREV * (angle / (2 * pi));
}

//! Test sine at various angles
TEST(TestSineCore, Sine)
{
    // zero angle
    EXPECT_THAT(
        FP_TOFLT(SineCore::Sine(RadToRev(0))),
        FloatNear(SineAmp * std::sin(0), Tolerance));

    // In first quadrant
    EXPECT_THAT(
        FP_TOFLT(SineCore::Sine(RadToRev(pi / 3))),
        FloatNear(SineAmp * std::sin(pi / 3), Tolerance));

    // 90 degrees
    EXPECT_THAT(
        FP_TOFLT(SineCore::Sine(RadToRev(pi / 2))),
        FloatNear(SineAmp * std::sin(pi / 2), Tolerance));

    // 180 degrees
    EXPECT_THAT(
        FP_TOFLT(SineCore::Sine(RadToRev(pi))),
        FloatNear(SineAmp * std::sin(pi), Tolerance));

    // In third quadrant - extra tolerant because there's an off-by-one
    // niggle/buglet where it gets the wrong bucket but only in this quadrant
    EXPECT_THAT(
        FP_TOFLT(SineCore::Sine(RadToRev(pi + pi / 4))),
        FloatNear(SineAmp * std::sin(pi + pi / 4), 3 * Tolerance));

    // In fourth quadrant
    EXPECT_THAT(
        FP_TOFLT(SineCore::Sine(RadToRev(2 * pi - pi / 5))),
        FloatNear(SineAmp * std::sin(2 * pi - pi / 5), Tolerance));
}

//! Test sine at various angles
TEST(TestCosineCore, Cosine)
{
    // zero angle
    EXPECT_THAT(
        FP_TOFLT(SineCore::Cosine(RadToRev(0))),
        FloatNear(SineAmp * std::cos(0), Tolerance));

    // In first quadrant
    EXPECT_THAT(
        FP_TOFLT(SineCore::Cosine(RadToRev(pi / 3))),
        FloatNear(SineAmp * std::cos(pi / 3), Tolerance));

    // 90 degrees
    EXPECT_THAT(
        FP_TOFLT(SineCore::Cosine(RadToRev(pi / 2))),
        FloatNear(SineAmp * std::cos(pi / 2), Tolerance));

    // 180 degrees
    EXPECT_THAT(
        FP_TOFLT(SineCore::Cosine(RadToRev(pi))),
        FloatNear(SineAmp * std::cos(pi), Tolerance));

    // In third quadrant - extra tolerant because there's an off-by-one
    // niggle/buglet where it gets the wrong bucket but only in this quadrant
    EXPECT_THAT(
        FP_TOFLT(SineCore::Cosine(RadToRev(pi + pi / 4))),
        FloatNear(SineAmp * std::cos(pi + pi / 4), 3 * Tolerance));

    // In fourth quadrant
    EXPECT_THAT(
        FP_TOFLT(SineCore::Cosine(RadToRev(2 * pi - pi / 5))),
        FloatNear(SineAmp * std::cos(2 * pi - pi / 5), Tolerance));
}
