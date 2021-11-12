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
#include "c2000/encoder.h"

namespace c2000 {

bool Encoder::SeenNorthSignal()
{
    return true;
}

void Encoder::UpdateRotorAngle(int dir)
{
}

void Encoder::UpdateRotorFrequency(int callingFrequency)
{
}

void Encoder::SetPwmFrequency(uint32_t frq)
{
}

uint16_t Encoder::GetRotorAngle()
{
    return 0;
}

u32fp Encoder::GetRotorFrequency()
{
    return 0;
}

int Encoder::GetRotorDirection()
{
    return 0;
}

} // namespace c2000
