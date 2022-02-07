/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2022 David J. Fiddes <D.J@fiddes.net>
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

#include "gatedriverinterface.h"
#include "device.h"
#include "driverlib.h"
#include "hw/stgap1as_gate_driver.h"

namespace c2000 {

/**
 * \brief Set up the SPI bus connected to the STGAP1AS gate drivers
 */
void GateDriverInterface::Init()
{
    // Set up the hardware mapping depending on the platform we are running on
    // The Launchpad uses different IO because it is easier to attach a logic
    // analyser to those pins
    if (IsTeslaM3Inverter())
    {
        m_gateShutdownPin = DEVICE_TESLAM3_GPIO_PIN_GATE_SHUTDOWN;
        m_gateCsPin = DEVICE_TESLAM3_GPIO_PIN_GATE_CS;
        m_gateClkCfg = DEVICE_TESLAM3_GPIO_CFG_GATE_CLK;
        m_gateMosiCfg = DEVICE_TESLAM3_GPIO_CFG_GATE_MOSI;
        m_gateMisoCfg = DEVICE_TESLAM3_GPIO_CFG_GATE_MISO;
        m_gateSpiBase = DEVICE_TESLAM3_GATE_SPI;
    }
    else
    {
        m_gateShutdownPin = DEVICE_LAUNCHXL_GPIO_PIN_GATE_SHUTDOWN;
        m_gateCsPin = DEVICE_LAUNCHXL_GPIO_PIN_GATE_CS;
        m_gateClkCfg = DEVICE_LAUNCHXL_GPIO_CFG_GATE_CLK;
        m_gateMosiCfg = DEVICE_LAUNCHXL_GPIO_CFG_GATE_MOSI;
        m_gateMisoCfg = DEVICE_LAUNCHXL_GPIO_CFG_GATE_MISO;
        m_gateSpiBase = DEVICE_LAUNCHXL_GATE_SPI;
    }

    // Start with the ~SD line asserted (i.e. in Shutdown state)
    GPIO_writePin(m_gateShutdownPin, 0);
    GPIO_setPadConfig(m_gateShutdownPin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(m_gateShutdownPin, GPIO_DIR_MODE_OUT);

    // Assert the SPI ~CS line before we turn off the pull up to avoid glitches
    GPIO_writePin(m_gateCsPin, 1);
    GPIO_setPadConfig(m_gateCsPin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(m_gateCsPin, GPIO_DIR_MODE_OUT);

    // Configure the SPI hardware ports
    GPIO_setPinConfig(m_gateClkCfg);
    GPIO_setPinConfig(m_gateMosiCfg);
    GPIO_setPinConfig(m_gateMisoCfg);

    // SPI initialization;
    // The STGAP1AS requires CPOL = 0, CPHA = 1 and 16-bit MSB transfers and
    // runs at up to 5MHz
    // TI terminology is different so the CPHA has to be inverted(!)
    SPI_disableModule(m_gateSpiBase);
    SPI_setConfig(
        m_gateSpiBase,
        DEVICE_LSPCLK_FREQ,
        SPI_PROT_POL0PHA0,
        SPI_MODE_MASTER,
        5000000,
        16);
    SPI_disableFIFO(m_gateSpiBase);
    SPI_setEmulationMode(m_gateSpiBase, SPI_EMULATION_STOP_AFTER_TRANSMIT);
    SPI_enableModule(m_gateSpiBase);
}

/**
 * \brief Initiate an SPI transaction with the gate drivers
 *
 * Encapsulates all of the hardware dependent work required to communicate with
 * an array of daisy-chained STGAP1AS gate driver chips. Inter-message timing
 * delays are not included.
 *
 * \param writeData Data to be sent to the STGAP1AS driver chips
 * \param readData Data buffer for data read from the chips. May be NULL if the
 * received data is not required
 */
void GateDriverInterface::SendData(DataBuffer writeData, DataBuffer readData)
{
    // Manually assert the ~CS pin and add a delay to allow it to settle and
    // match the required set-up time for the STGAP1AS
    GPIO_writePin(m_gateCsPin, 0);
    DEVICE_DELAY_US(1);

    // Run the SPI transaction with a 2 cycle delay between 16-bit words
    SPI_pollingFIFOTransaction(
        m_gateSpiBase, 16U, writeData, readData, NumDriverChips, 2U);

    // Manually de-assert the ~CS pin and ensure that we have waited sufficient
    // time for the data being sent byt the chips to arrive
    DEVICE_DELAY_US(1);
    GPIO_writePin(m_gateCsPin, 1);
}

/**
 * \brief Assert the ~SD line on the STGAP1AS gate drivers allowing them to be
 * configured
 */
void GateDriverInterface::Shutdown()
{
    GPIO_writePin(m_gateShutdownPin, 0);
}

/**
 * \brief De-assert the ~SD line on the STGAP1AS gate drivers to allow them to
 * run normally
 */
void GateDriverInterface::Resume()
{
    GPIO_writePin(m_gateShutdownPin, 1);
}

/**
 * \brief Return whether the STGAP1AS gate drivers are enabled
 */
bool GateDriverInterface::IsShutdown()
{
    return GPIO_readPin(m_gateShutdownPin) == 0;
}

} // namespace c2000
