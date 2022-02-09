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

#include "c2000/pmicspidriver.h"

#include "device.h"

uint32_t PmicSpiDriver::m_base = 0;
bool     PmicSpiDriver::m_readAfterWrite = false;

void PmicSpiDriver::InitGPIO(uint16_t pin, uint32_t cfg)
{
    GPIO_setMasterCore(pin, GPIO_CORE_CPU1);
    GPIO_setPinConfig(cfg);
    GPIO_setPadConfig(pin, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(pin, GPIO_QUAL_ASYNC);
}

/**
 * \brief Initialise the SPI port's GPIO lines
 */
void PmicSpiDriver::InitGPIOs()
{

    if (IsTeslaM3Inverter())
    {

        m_base = DEVICE_TESLAM3_PMIC_SPI;
        m_readAfterWrite = true;

        // SPISOMIA.
        InitGPIO(
            DEVICE_TESLAM3_GPIO_PIN_PMIC_MISO,
            DEVICE_TESLAM3_GPIO_CFG_PMIC_MISO);

        // SPISIMOA
        InitGPIO(
            DEVICE_TESLAM3_GPIO_PIN_PMIC_MOSI,
            DEVICE_TESLAM3_GPIO_CFG_PMIC_MOSI);

        // SPISTEA / CS
        InitGPIO(
            DEVICE_TESLAM3_GPIO_PIN_PMIC_CS, DEVICE_TESLAM3_GPIO_CFG_PMIC_CS);

        // SPICLK.
        InitGPIO(
            DEVICE_TESLAM3_GPIO_PIN_PMIC_CLK, DEVICE_TESLAM3_GPIO_CFG_PMIC_CLK);
    }
    else
    {

        m_base = DEVICE_LAUNCHXL_PMIC_SPI;

        // SPISOMIA.
        InitGPIO(
            DEVICE_LAUNCHXL_GPIO_PIN_PMIC_MISO,
            DEVICE_LAUNCHXL_GPIO_CFG_PMIC_MISO);

        // SPISIMOA
        InitGPIO(
            DEVICE_LAUNCHXL_GPIO_PIN_PMIC_MOSI,
            DEVICE_LAUNCHXL_GPIO_CFG_PMIC_MOSI);

        // SPISTEA / CS
        InitGPIO(
            DEVICE_LAUNCHXL_GPIO_PIN_PMIC_CS, DEVICE_LAUNCHXL_GPIO_CFG_PMIC_CS);

        // SPICLK.
        InitGPIO(
            DEVICE_LAUNCHXL_GPIO_PIN_PMIC_CLK,
            DEVICE_LAUNCHXL_GPIO_CFG_PMIC_CLK);
    }
}

/**
 * \brief Initialise the SPI port and associated clocks
 */
void PmicSpiDriver::InitSPIPort()
{
    // TODO

    // Assert the SPI ~CS enable line before we turn off the
    // pull up to avoid glitches

    SPI_disableModule(m_base);

    // Tesla run the device at 2.4MHz so we also run slower than rated 5MHz
    // to be on the safe side.
    // SLI/SDO rising edge, shift falling edge of clock (CPHA 0)
    // CLK low when idle (CPOL 0)
    // The TLF35584 requires CPOL = 0, CPHA = 0 and 16-bit MSB transfers with
    // software controlled chip-select control around each transfer. MASTER
    // mode, unidirectional full duplex

    SPI_setConfig(
        m_base,
        DEVICE_LSPCLK_FREQ,
        SPI_PROT_POL0PHA1,
        SPI_MODE_MASTER,
        500000,
        16);

    SPI_disableFIFO(m_base);
    SPI_disableLoopback(m_base);
    SPI_setEmulationMode(m_base, SPI_EMULATION_STOP_AFTER_TRANSMIT);

    SPI_enableModule(m_base);
}

bool PmicSpiDriver::ReadDataAfterWrite()
{
    return m_readAfterWrite;
}

uint16_t PmicSpiDriver::TransferData(uint16_t data)
{
    return SPI_pollingNonFIFOTransaction(m_base, 16U, data);
}
