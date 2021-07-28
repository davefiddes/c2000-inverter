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
#ifndef MOCKCURRENT_H
#define MOCKCURRENT_H

#include <stdint.h>

/**
 * Mock implementation of the Current class used to obtain analog values
 * of the phase currents. Allows each value to be set by test functions
 * as required rather than reading a physical value.
 */
class MockCurrent
{
public:
    static uint16_t Phase1()
    {
        return m_phase1Value;
    }

    static void SetPhase1(uint16_t value)
    {
        m_phase1Value = value;
    }

    static uint16_t Phase2()
    {
        return m_phase2Value;
    }

    static void SetPhase2(uint16_t value)
    {
        m_phase2Value = value;
    }

private:
    static uint16_t m_phase1Value;
    static uint16_t m_phase2Value;
};

uint16_t MockCurrent::m_phase1Value;
uint16_t MockCurrent::m_phase2Value;

#endif // MOCKCURRENT_H