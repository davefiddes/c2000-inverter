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

#include "foc.h"
#include "matcherhelper.h"
#include "my_fp.h"
#include "sine_core.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <math.h>
#include <utility>

#include "utils.h"

using ::testing::FloatNear;
#if HAVE_NUMBERS_H
using namespace std::numbers;
#endif

// Numeric tolerance we're trying to measure to. Number is empirical as the
// fixed point maths is a bit loosey goosey.
static constexpr float Tolerance = 0.6;

constexpr uint16_t RadToRev(float angle)
{
    return SINLU_ONEREV * (angle / (2 * pi));
}

/**
 * Alternative implementation of ParkClarke transform with floating point values
 * and radians for shaft angle
 */
std::pair<float, float> FloatParkClark(float phase1, float phase2, float theta)
{
    // Clarke transformation
    float ia = phase1;
    float ib = inv_sqrt3 * (phase1 + 2 * phase2);

    // Park transformation
    float id = ia * cos(theta) + ib * sin(theta);
    float iq = ib * cos(theta) - ia * sin(theta);

    return std::make_pair(id, iq);
}

TEST(TestFocCalc, ParkClarkeTransform)
{
    // No current, zero angle therefore no Id and Iq
    FOC::SetAngle(0);
    FOC::ParkClarke(FP_FROMFLT(0.0), FP_FROMFLT(0.0));
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(0.0, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(0.0, Tolerance));

    // Phase 1 current only, zero angle
    FOC::SetAngle(0);
    FOC::ParkClarke(FP_FROMFLT(1.0), FP_FROMFLT(0.0));
    auto result = FloatParkClark(1.0, 0.0, 0);
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(result.first, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(result.second, Tolerance));

    // Phase 1 and Phase 2 current, zero angle
    FOC::SetAngle(0);
    FOC::ParkClarke(FP_FROMFLT(100.0), FP_FROMFLT(100.0));
    result = FloatParkClark(100.0, 100.0, 0);
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(result.first, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(result.second, Tolerance));

    // In first quadrant
    FOC::SetAngle(RadToRev(pi / 3));
    FOC::ParkClarke(FP_FROMFLT(100.0), FP_FROMFLT(100.0));
    result = FloatParkClark(100.0, 100.0, pi / 3);
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(result.first, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(result.second, Tolerance));

    // 90 degrees
    FOC::SetAngle(RadToRev(pi / 2));
    FOC::ParkClarke(FP_FROMFLT(100.0), FP_FROMFLT(100.0));
    result = FloatParkClark(100.0, 100.0, pi / 2);
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(result.first, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(result.second, Tolerance));

    // 180 degrees
    FOC::SetAngle(RadToRev(pi));
    FOC::ParkClarke(FP_FROMFLT(100.0), FP_FROMFLT(100.0));
    result = FloatParkClark(100.0, 100.0, pi);
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(result.first, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(result.second, Tolerance));

    // In third quadrant
    FOC::SetAngle(RadToRev(pi + pi / 4));
    FOC::ParkClarke(FP_FROMFLT(100.0), FP_FROMFLT(100.0));
    result = FloatParkClark(100.0, 100.0, pi + pi / 4);
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(result.first, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(result.second, Tolerance));

    // In fourth quadrant
    FOC::SetAngle(RadToRev(2 * pi - pi / 5));
    FOC::ParkClarke(FP_FROMFLT(100.0), FP_FROMFLT(100.0));
    result = FloatParkClark(100.0, 100.0, 2 * pi - pi / 5);
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(result.first, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(result.second, Tolerance));

    // Different phase currents at 90 degrees
    FOC::SetAngle(RadToRev(pi / 2));
    FOC::ParkClarke(FP_FROMFLT(123.4), FP_FROMFLT(567.8));
    result = FloatParkClark(123.4, 567.8, pi / 2);
    EXPECT_THAT(FP_TOFLOAT(FOC::id), FloatNear(result.first, Tolerance));
    EXPECT_THAT(FP_TOFLOAT(FOC::iq), FloatNear(result.second, Tolerance));
}

float FloatGetQLimit(float maxVd)
{
    constexpr float modRange = 1 << 15;
    constexpr float modMax = modRange * 2 * inv_sqrt3;
    constexpr float modMaxPow2 = modMax * modMax;

    return std::sqrt(modMaxPow2 - std::pow(maxVd, 2));
}

TEST(TestFocCalc, GetQLimit)
{
    EXPECT_THAT(FOC::GetQLimit(0), IntNear(FloatGetQLimit(0), 2));
    EXPECT_THAT(FOC::GetQLimit(100), IntNear(FloatGetQLimit(100), 2));
    EXPECT_THAT(FOC::GetQLimit((int32_t)456.78), IntNear(FloatGetQLimit(456.78), 2));
}

float FloatGetVoltage(float ud, float uq)
{
    // pythagoras to the rescue
    return std::sqrt(ud * ud + uq * uq);
}

TEST(TestFocCalc, GetTotalVoltage)
{
    EXPECT_THAT(FOC::GetTotalVoltage(0, 0), IntNear(FloatGetVoltage(0, 0), 2));
    EXPECT_THAT(
        FOC::GetTotalVoltage(5000, 0), IntNear(FloatGetVoltage(5000, 0), 2));
    EXPECT_THAT(
        FOC::GetTotalVoltage(0, 6000), IntNear(FloatGetVoltage(0, 6000), 2));
    EXPECT_THAT(
        FOC::GetTotalVoltage(8000, 9000),
        IntNear(FloatGetVoltage(8000, 9000), 2));
}

TEST(TestFocCalc, InvParkClarke)
{
    // Zero volts should be 50% PWM duty cycle irrespective of the angle
    FOC::SetAngle(0);
    FOC::InvParkClarke(0, 0);
    EXPECT_EQ(FOC::DutyCycles[0], 32768);
    EXPECT_EQ(FOC::DutyCycles[1], 32768);
    EXPECT_EQ(FOC::DutyCycles[2], 32768);

    FOC::SetAngle(32768);
    FOC::InvParkClarke(0, 0);
    EXPECT_EQ(FOC::DutyCycles[0], 32768);
    EXPECT_EQ(FOC::DutyCycles[1], 32768);
    EXPECT_EQ(FOC::DutyCycles[2], 32768);

    FOC::SetAngle(60000);
    FOC::InvParkClarke(0, 0);
    EXPECT_EQ(FOC::DutyCycles[0], 32768);
    EXPECT_EQ(FOC::DutyCycles[1], 32768);
    EXPECT_EQ(FOC::DutyCycles[2], 32768);

    // PWM values from here on obtained by running and then modifying values
    // till the tests pass - There has to be a better way.

    // Ud volts only
    FOC::SetAngle(0);
    FOC::InvParkClarke(1000, 0);
    EXPECT_EQ(FOC::DutyCycles[0], 33517);
    EXPECT_EQ(FOC::DutyCycles[1], 32019);
    EXPECT_EQ(FOC::DutyCycles[2], 32019);

    // Uq volts only aligned with phase A
    FOC::SetAngle(0);
    FOC::InvParkClarke(0, 1000);
    EXPECT_EQ(FOC::DutyCycles[0], 32768);
    EXPECT_EQ(FOC::DutyCycles[1], 33633);
    EXPECT_EQ(FOC::DutyCycles[2], 31903);

    // Ud and Uq volts
    FOC::SetAngle(0);
    FOC::InvParkClarke(1000, 1000);
    EXPECT_EQ(FOC::DutyCycles[0], 33950);
    EXPECT_EQ(FOC::DutyCycles[1], 33316);
    EXPECT_EQ(FOC::DutyCycles[2], 31587);

    // Uq volts only aligned with phase C
    FOC::SetAngle(RadToRev(2 * pi / 3));
    FOC::InvParkClarke(0, 1000);
    EXPECT_EQ(FOC::DutyCycles[0], 31901);
    EXPECT_EQ(FOC::DutyCycles[1], 32770);
    EXPECT_EQ(FOC::DutyCycles[2], 33635);
}

//! Floating point implementation of Maximum Torque Per Amp calculation from
//! SPRACF3 Sensorless-FOC With Flux-Weakening and MTPA for IPMSM Motor Drives
//! https://www.ti.com/lit/pdf/spracf3
//! Adjusted as the by-the-book-version causes oscillation
std::pair<float, float> FloatMtpa(float is)
{
    constexpr float fluxLinkage = 0.09; // Wb
    constexpr float lqminusld = 0.0058; // Lq - Ld;

    float isSquared = is * is;
    float sign = is < 0 ? -1 : 1;
    float term1 = fluxLinkage * fluxLinkage + lqminusld * lqminusld * isSquared;
    term1 = std::sqrt(term1);
    float idref = (fluxLinkage - term1) / lqminusld;
    float iqref = sign * std::sqrt(isSquared - idref * idref);

    return std::make_pair(idref, iqref);
}

TEST(TestFocCalc, MaximumTorquePerAmp)
{
    int32_t idref;
    int32_t iqref;

    FOC::Mtpa(10, idref, iqref);
    auto iref = FloatMtpa(10);
    EXPECT_THAT(idref, IntNear(floor(iref.first), 1));
    EXPECT_THAT(iqref, IntNear(floor(iref.second), 1));
}

TEST(TestFocCalc, GetMaximumModulationIndex)
{
    constexpr int32_t modRange = 1 << 15;
    constexpr float   maxMod = modRange * 2 * inv_sqrt3;

    EXPECT_EQ(FOC::GetMaximumModulationIndex(), std::floor(maxMod));
}
