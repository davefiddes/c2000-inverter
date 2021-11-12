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
#ifndef SPIHELPER_H
#define SPIHELPER_H

#include "driverlib.h"

/**
 * \brief Define a scoped SPI transaction that manually asserts the ~CS line for
 * the duration of a SPI transaction.
 */
template <uint32_t CsPin>
struct SPITransaction
{
    SPITransaction()
    {
        GPIO_writePin(CsPin, 0);
    }

    ~SPITransaction()
    {
        GPIO_writePin(CsPin, 1);
    }
};

#endif // SPIHELPER_H
