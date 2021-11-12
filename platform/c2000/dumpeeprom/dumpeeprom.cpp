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

#include "device.h"
#include "driverlib.h"
#include "c2000/eeprom.h"
#include "c2000/spihelper.h"
#include <stdio.h>

using namespace c2000;

void DumpByteByByte()
{
    for (uint32_t address = 0; address < 0x8000; address++)
    {
        uint16_t data = EEPROM::Read8Bits(address);
        printf("%.2x ", data);

        if (address % 0x10 == 0xF)
        {
            printf("\n");
        }
    }
}

void Dump32BitsAtATime()
{
    for (uint32_t address = 0; address < 0x8000; address += 4)
    {
        uint32_t data = EEPROM::Read32Bits(address);
        printf(
            "%.2x %.2x %.2x %.2x ",
            (uint16_t)data & 0xFF,
            (uint16_t)(data >> 8) & 0xFF,
            (uint16_t)(data >> 16) & 0xFF,
            (uint16_t)(data >> 24) & 0xFF);

        if (address % 0x10 == 0xC)
        {
            printf("\n");
        }
    }
}

void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    Device_initGPIO();

    // Use only line-by-line buffering on stdout for easier debugging
    // We still need to use line buffering to avoid very poor IO performance
    // with C2000 semi-hosting
    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("Tesla M3 EEPROM dumper\n\n");

    printf(
        "Running on: %s\n",
        IsTeslaM3Inverter() ? "Tesla M3 Inverter" : "TI Launchpad");

    EEPROM::InitSPI();

#if 0
    printf("\nDumpByteByByte()\n");
    DumpByteByByte();
#else
    printf("\nDump32BitsAtATime()\n");
    Dump32BitsAtATime();
#endif
}
