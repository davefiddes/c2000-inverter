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
#ifndef MOCKENCODER_H
#define MOCKENCODER_H

#include "my_fp.h"
#include <gmock/gmock.h>
#include <stdint.h>

class MockEncoderImpl;

/**
 * Mock implementation of a motor shaft encoder. This provides the static
 * methods expected by the openinverter classes and acts as a trampoline to the
 * real mock methods in MockEncoderImpl.
 *
 * Note: Not all methods from Encoder are implemented but are present in the
 * MockEncoderImpl
 */
class MockEncoder
{
public:
    enum mode
    {
        SOMETHING,
        INVALID
    };

public:
    static bool SeenNorthSignal();
    static void UpdateRotorAngle(int dir);
    static void UpdateRotorFrequency(int callingFrequency);
    static void SetPwmFrequency(uint32_t frq);

    static uint16_t GetRotorAngle();
    static u32fp    GetRotorFrequency();
    static int      GetRotorDirection();

private:
    static MockEncoderImpl* instance;
    friend class MockEncoderImpl;
};

class MockEncoderImpl
{
public:
    MockEncoderImpl();
    ~MockEncoderImpl();

    using mode = MockEncoder::mode;

    MOCK_METHOD(void, Reset, ());
    MOCK_METHOD(void, SetMode, (mode encMode));
    MOCK_METHOD(bool, SeenNorthSignal, ());
    MOCK_METHOD(void, UpdateRotorAngle, (int dir));
    MOCK_METHOD(void, UpdateRotorFrequency, (int callingFrequency));
    MOCK_METHOD(void, SetPwmFrequency, (uint32_t frq));
    MOCK_METHOD(uint16_t, GetRotorAngle, ());
    MOCK_METHOD(uint32_t, GetSpeed, ());
    MOCK_METHOD(uint32_t, GetFullTurns, ());
    MOCK_METHOD(u32fp, GetRotorFrequency, ());
    MOCK_METHOD(int, GetRotorDirection, ());
    MOCK_METHOD(void, SetImpulsesPerTurn, (uint16_t imp));
    MOCK_METHOD(void, SwapSinCos, (bool swap));
    MOCK_METHOD(void, SetSinCosOffset, (uint16_t offset));
};

#endif // MOCKENCODER_H