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
#ifndef EEPROM_H
#define EEPROM_H

#include "driverlib.h"

namespace c2000 {

/**
 * \brief Access the contents of the Tesla M3 Inverter EEPROM
 */
class EEPROM
{
public:
    /**
     * Initialise the SPI port connected to the EEPROM
     */
    static void InitSPI();

    /**
     * Read a single 8-bit value from the EEPROM
     * \param address    Address of the value to read (0-0x8000)
     */
    static uint16_t Read8Bits(uint16_t address);

    /**
     * Read a 32-bit value from the EEPROM
     * \param address    Address of the value to read (0-0x8000)
     */
    static uint32_t Read32Bits(uint16_t address);
};

} // namespace c2000

#endif // EEPROM_H
