/*
 * This file is part of the stm32-sine project.
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
#ifndef MOCKPWMDRIVER_H
#define MOCKPWMDRIVER_H

#include "my_fp.h"
#include "params.h"
#include <gmock/gmock.h>
#include <stdint.h>

class MockPwmDriverImpl;

/**
 * Mock implementation of a motor PWM generator. This provides the static
 * methods expected by the openinverter PWM generator classes and acts as a
 * trampoline to the real mock methods in MockPwmDriverImpl.
 */
class MockPwmDriver
{
public:
    // TODO: Should this be variable/different from STM32 implementation?
    static const uint16_t MinPwmDigits = 11;

public:
    static void DriverInit();
    static void EnableMasterOutput();
    static void DisableMasterOutput();
    static void EnableOutput();
    static void DisableOutput();
    static void SetPhasePwm(uint32_t phaseA, uint32_t phaseB, uint32_t phaseC);
    static void EnableChargeOutput(Modes opmode);
    static void EnableACHeatOutput();
    static void SetOverCurrentLimits(int16_t limNeg, int16_t LimPos);
    static uint16_t TimerSetup(
        uint16_t deadtime,
        bool     activeLow,
        uint16_t pwmdigits);

    static void AcHeatTimerSetup();
    static void AcHeat(s32fp ampnom);
    static void SetChargeCurrent(int16_t dc);
    static void ResetCpuLoad();

private:
    static MockPwmDriverImpl* instance;
    friend class MockPwmDriverImpl;
};

class MockPwmDriverImpl
{
public:
    MockPwmDriverImpl();
    ~MockPwmDriverImpl();

    MOCK_METHOD(void, DriverInit, ());
    MOCK_METHOD(void, EnableMasterOutput, ());
    MOCK_METHOD(void, DisableMasterOutput, ());
    MOCK_METHOD(void, EnableOutput, ());
    MOCK_METHOD(void, DisableOutput, ());
    MOCK_METHOD(
        void,
        SetPhasePwm,
        (uint32_t phaseA, uint32_t phaseB, uint32_t phaseC));
    MOCK_METHOD(void, EnableChargeOutput, (Modes opmode));
    MOCK_METHOD(void, EnableACHeatOutput, ());
    MOCK_METHOD(void, SetOverCurrentLimits, (int16_t limNeg, int16_t LimPos));
    MOCK_METHOD(
        uint16_t,
        TimerSetup,
        (uint16_t deadtime, bool activeLow, uint16_t pwmdigits));
    MOCK_METHOD(int, GetPwmFrq, ());
    MOCK_METHOD(void, AcHeatTimerSetup, ());
    MOCK_METHOD(void, AcHeat, (s32fp ampnom));
    MOCK_METHOD(void, SetChargeCurrent, (int16_t dc));
    MOCK_METHOD(int, GetCpuLoad, ());
    MOCK_METHOD(void, ResetCpuLoad, ());
};

#endif // MOCKPWMDRIVER_H