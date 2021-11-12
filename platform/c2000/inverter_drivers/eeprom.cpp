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
#include "c2000/eeprom.h"
#include "device.h"
#include "driverlib.h"
#include "c2000/spihelper.h"

namespace c2000 {

typedef SPITransaction<DEVICE_TESLAM3_GPIO_PIN_EEPROM1_CS> EEPROMTransaction;

// SPI EEPROM commands

//! Read data from memory array beginning at selected address
#define EEPROM_CMD_READ 0x03

//! Write data to memory array beginning at selected address
#define EEPROM_CMD_WRITE 0x02

//! Reset the write enable latch(disable write operations)
#define EEPROM_CMD_WRDI 0x04

//! Set the write enable latch(enable write operations)
#define EEPROM_CMD_WREN 0x06

//! Read STATUS register
#define EEPROM_CMD_RDSR 0x05

//! Write STATUS register
#define EEPROM_CMD_WRSR 0x01

void EEPROM::InitSPI()
{
    // Assert the SPI ~CS line before we turn off the pull up to avoid glitches
    GPIO_writePin(DEVICE_TESLAM3_GPIO_PIN_EEPROM1_CS, 1);
    GPIO_setPadConfig(DEVICE_TESLAM3_GPIO_PIN_EEPROM1_CS, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(
        DEVICE_TESLAM3_GPIO_PIN_EEPROM1_CS, GPIO_DIR_MODE_OUT);

    // Configure the SPI hardware ports
    GPIO_setPinConfig(DEVICE_TESLAM3_GPIO_CFG_EEPROM_CLK);
    GPIO_setPinConfig(DEVICE_TESLAM3_GPIO_CFG_EEPROM_MOSI);
    GPIO_setPinConfig(DEVICE_TESLAM3_GPIO_CFG_EEPROM_MISO);

    // SPI initialization;
    // The 25LC256 requires CPOL = 0, CPHA = 0 and MSB transfers of variable
    // length runs at up to 2MHz
    SPI_disableModule(DEVICE_TESLAM3_EEPROM_SPI);
    SPI_setConfig(
        DEVICE_TESLAM3_EEPROM_SPI,
        DEVICE_LSPCLK_FREQ,
        SPI_PROT_POL0PHA1, // TI inverts the meaning of PHA
        SPI_MODE_MASTER,
        2000000,
        16);
    SPI_disableFIFO(DEVICE_TESLAM3_EEPROM_SPI);
    SPI_setEmulationMode(
        DEVICE_TESLAM3_EEPROM_SPI, SPI_EMULATION_STOP_AFTER_TRANSMIT);
    SPI_enableModule(DEVICE_TESLAM3_EEPROM_SPI);
}

uint16_t EEPROM::Read8Bits(uint16_t address)
{

    EEPROMTransaction transaction;

    SPI_transmitByte(DEVICE_TESLAM3_EEPROM_SPI, EEPROM_CMD_READ);
    SPI_transmit16Bits(DEVICE_TESLAM3_EEPROM_SPI, address);
    return SPI_receiveByte(DEVICE_TESLAM3_EEPROM_SPI, 0);
}

uint32_t EEPROM::Read32Bits(uint16_t address)
{

    EEPROMTransaction transaction;

    SPI_transmitByte(DEVICE_TESLAM3_EEPROM_SPI, EEPROM_CMD_READ);
    SPI_transmit16Bits(DEVICE_TESLAM3_EEPROM_SPI, address);
    return SPI_receive32Bits(
        DEVICE_TESLAM3_EEPROM_SPI, SPI_DATA_LITTLE_ENDIAN, 0, 0);
}

} // namespace c2000
