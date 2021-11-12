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
#ifndef C2000ENCODER_H
#define C2000ENCODER_H

#include "my_fp.h"
#include <stdint.h>

namespace c2000 {

/**
 * Mock implementation of a motor shaft encoder. This provides the static
 * methods expected by the openinverter classes.
 */
class Encoder
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
};

} // namespace c2000

#endif // C2000ENCODER_H
