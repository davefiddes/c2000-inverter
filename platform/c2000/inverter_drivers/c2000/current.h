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
#ifndef C2000CURRENT_H
#define C2000CURRENT_H

#include "motoranalogcapture.h"
#include <stdint.h>

namespace c2000 {

/**
 * Adapt the MotorAnalogCapture phase current readings into a form that the
 * PwmGeneration class can use
 */
class Current
{
public:
    static uint16_t Phase1()
    {
        return MotorAnalogCapture::PhaseACurrent();
    }

    static uint16_t Phase2()
    {
        return MotorAnalogCapture::PhaseBCurrent();
    }
};

} // namespace c2000

#endif // C2000CURRENT_H
