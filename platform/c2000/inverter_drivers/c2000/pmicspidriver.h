/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2022 Bernd Ocklin <bernd@ocklin.de>
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

#ifndef C2000SPIDRIVER_H
#define C2000SPIDRIVER_H

#include "stdint.h"

class PmicSpiDriver {

private:
    static uint32_t m_base;

    static void InitGPIO(uint16_t, uint32_t);

public:
    static void InitGPIOs();
    static void InitSPIPort();

    static void WriteData(uint16_t data);
    static uint16_t ReadData();
};

#endif // C2000SPIDRIVER_H

