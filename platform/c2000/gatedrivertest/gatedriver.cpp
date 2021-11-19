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

#include "gatedriver.h"
#include "crc8.h"
#include "device.h"
#include "driverlib.h"
#include "hw/stgap1as_gate_driver.h"
#ifdef DEBUG_STATE
#include <stdio.h>
#endif
#include <string.h>

/**
 * \brief STGAP1AS gate driver register set up sequence
 *
 *  The register set up sequence for each of 6 chips on the Tesla Model 3
 * Inverter. Settings are applied to all chips or high/low-side drivers as
 * required
 */
const TeslaM3GateDriver::Register
    TeslaM3GateDriver::GateDriverRegisterSetup[] = {
        { STGAP1AS_REG_CFG1,
          STGAP1AS_REG_CFG1_CRC_SPI | STGAP1AS_REG_CFG1_SD_FLAG |
              STGAP1AS_REG_CFG1_DT_800NS | STGAP1AS_REG_CFG1_IN_FILTER_500NS,
          All },
        { STGAP1AS_REG_CFG2,
          STGAP1AS_REG_CFG2_DESAT_CUR_500UA | STGAP1AS_REG_CFG2_DESAT_TH_8V,
          All },
        { STGAP1AS_REG_CFG3,
          STGAP1AS_REG_CFG3_2LTO_TH_10V | STGAP1AS_REG_CFG3_2LTO_TIME_DISABLED,
          All },
        { STGAP1AS_REG_CFG4,
          STGAP1AS_REG_CFG4_UVLO_LATCHED | STGAP1AS_REG_CFG4_VLON_TH_NEG_3V |
              STGAP1AS_REG_CFG4_VHON_TH_12V,
          Odd },
        { STGAP1AS_REG_CFG4,
          STGAP1AS_REG_CFG4_UVLO_LATCHED | STGAP1AS_REG_CFG4_VLON_TH_DISABLED |
              STGAP1AS_REG_CFG4_VHON_TH_12V,
          Even },
        { STGAP1AS_REG_CFG5,
          STGAP1AS_REG_CFG5_2LTO_EN | STGAP1AS_REG_CFG5_DESAT_EN,
          All },
        { STGAP1AS_REG_DIAG1CFG,
          STGAP1AS_REG_DIAG1CFG_UVLOD_OVLOD |
              STGAP1AS_REG_DIAG1CFG_UVLOH_UVLOL |
              STGAP1AS_REG_DIAG1CFG_OVLOH_OVLOL |
              STGAP1AS_REG_DIAG1CFG_DESAT_SENSE | STGAP1AS_REG_DIAG1CFG_TSD,
          All },
        { STGAP1AS_REG_DIAG2CFG, 0, All }
    };

const uint16_t TeslaM3GateDriver::RegisterSetupSize =
    sizeof(TeslaM3GateDriver::GateDriverRegisterSetup) /
    sizeof(GateDriverRegisterSetup[0]);

const TeslaM3GateDriver::Register TeslaM3GateDriver::NullGateDriverRegister = {
    0,
    0,
    All
};

// Delays from STGAP1AS datasheet Table 6. DC operation electrical
// characteristics - SPI Section
static const __attribute__((__unused__)) int ResetStatusDelay = 50;   // uSec
static const __attribute__((__unused__)) int LocalRegReadDelay = 1;   // uSec
static const __attribute__((__unused__)) int RemoteRegReadDelay = 30; // uSec
static const __attribute__((__unused__)) int StartConfigDelay = 22;   // uSec
static const __attribute__((__unused__)) int StopConfigDelay = 5;     // uSec
static const __attribute__((__unused__)) int OtherCommandDelay = 1;   // uSec

/**
 * Hardware variant definitions
 */
static uint16_t sg_gateShutdownPin;

static uint16_t sg_gateCsPin;

static uint32_t sg_gateClkCfg;
static uint32_t sg_gateMosiCfg;
static uint32_t sg_gateMisoCfg;

static uint32_t sg_gateSpiBase;

/**
 * \brief Define a scoped SPI transaction that manually asserts the ~CS line for
 * the duration of a SPI transaction.
 */
struct SPITransaction
{
    SPITransaction()
    {
        GPIO_writePin(sg_gateCsPin, 0);
        DEVICE_DELAY_US(1);
    }

    ~SPITransaction()
    {
        DEVICE_DELAY_US(1);
        GPIO_writePin(sg_gateCsPin, 1);
    }
};

/**
 * \brief Invert and mask off the lower 8 bits only
 *
 * \return The inverted value
 */
static uint16_t invert_byte(uint16_t input)
{
    return ~input & 0xFF;
}

/**
 * \brief Build a command with it's CRC
 *
 * \return The complete command
 */
static uint16_t BuildCommand(uint16_t cmd)
{
    return cmd << 8 | invert_byte(crc8(STGAP1AS_SPI_CRC_INIT_VALUE, cmd));
}

/**
 * \brief Set up the isolated gate drivers
 *
 * \return bool - True if gate drivers successfully initialised and verified
 */
bool TeslaM3GateDriver::Init()
{
    // Set up the hardware mapping depending on the platform we are running on
    // The Launchpad uses different IO because it is easier to attach a logic
    // analyser to those pins
    if (IsTeslaM3Inverter())
    {
        sg_gateShutdownPin = DEVICE_TESLAM3_GPIO_PIN_GATE_SHUTDOWN;
        sg_gateCsPin = DEVICE_TESLAM3_GPIO_PIN_GATE_CS;
        sg_gateClkCfg = DEVICE_TESLAM3_GPIO_CFG_GATE_CLK;
        sg_gateMosiCfg = DEVICE_TESLAM3_GPIO_CFG_GATE_MOSI;
        sg_gateMisoCfg = DEVICE_TESLAM3_GPIO_CFG_GATE_MISO;
        sg_gateSpiBase = DEVICE_TESLAM3_GATE_SPI;
    }
    else
    {
        sg_gateShutdownPin = DEVICE_LAUNCHXL_GPIO_PIN_GATE_SHUTDOWN;
        sg_gateCsPin = DEVICE_LAUNCHXL_GPIO_PIN_GATE_CS;
        sg_gateClkCfg = DEVICE_LAUNCHXL_GPIO_CFG_GATE_CLK;
        sg_gateMosiCfg = DEVICE_LAUNCHXL_GPIO_CFG_GATE_MOSI;
        sg_gateMisoCfg = DEVICE_LAUNCHXL_GPIO_CFG_GATE_MISO;
        sg_gateSpiBase = DEVICE_LAUNCHXL_GATE_SPI;
    }

    // Assert the ~SD line before we configure anything
    GPIO_writePin(sg_gateShutdownPin, 0);
    GPIO_setPadConfig(sg_gateShutdownPin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(sg_gateShutdownPin, GPIO_DIR_MODE_OUT);

    InitSPIPort();
    SetupGateDrivers();
    return VerifyGateDriverConfig();
}

/**
 * \brief Check for a fault on all gate drivers
 *
 * \return bool - There is a fault on one or more gate drivers
 */
bool TeslaM3GateDriver::IsFaulty()
{
    bool result = !VerifyRegister(STGAP1AS_REG_STATUS1, 0);

    result = result && !VerifyRegister(STGAP1AS_REG_STATUS2, 0);
    result = result && !VerifyRegister(STGAP1AS_REG_STATUS3, 0);

    return result;
}

/**
 * \brief Enable the gate drivers
 */
void TeslaM3GateDriver::Enable()
{
    // De-assert the gate driver shutdown line
    GPIO_writePin(sg_gateShutdownPin, 1);
}

/**
 * \brief Disable the gate drivers
 */
void TeslaM3GateDriver::Disable()
{
    // Assert the gate driver shutdown line
    GPIO_writePin(sg_gateShutdownPin, 0);
}

/**
 * \brief Initialise the SPI port and associated clocks and GPIO ports
 */
void TeslaM3GateDriver::InitSPIPort()
{
    // Assert the SPI ~CS line before we turn off the pull up to avoid glitches
    GPIO_writePin(sg_gateCsPin, 1);
    GPIO_setPadConfig(sg_gateCsPin, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(sg_gateCsPin, GPIO_DIR_MODE_OUT);

    // Configure the SPI hardware ports
    GPIO_setPinConfig(sg_gateClkCfg);
    GPIO_setPinConfig(sg_gateMosiCfg);
    GPIO_setPinConfig(sg_gateMisoCfg);

    // SPI initialization;
    // The STGAP1AS requires CPOL = 0, CPHA = 1 and 16-bit MSB transfers and
    // runs at up to 5MHz
    // TI terminology is different so the CPHA has to be inverted(!)
    SPI_disableModule(sg_gateSpiBase);
    SPI_setConfig(
        sg_gateSpiBase,
        DEVICE_LSPCLK_FREQ,
        SPI_PROT_POL0PHA0,
        SPI_MODE_MASTER,
        5000000,
        16);
    SPI_disableFIFO(sg_gateSpiBase);
    SPI_setEmulationMode(sg_gateSpiBase, SPI_EMULATION_STOP_AFTER_TRANSMIT);
    SPI_enableModule(sg_gateSpiBase);
}

/**
 * \brief Run through the set up sequence for all gate driver chips
 */
void TeslaM3GateDriver::SetupGateDrivers()
{
    SendCommand(STGAP1AS_CMD_RESET_STATUS);
    DEVICE_DELAY_US(ResetStatusDelay);

    SendCommand(STGAP1AS_CMD_START_CONFIG);
    DEVICE_DELAY_US(StartConfigDelay);

    for (uint8_t i = 0; i < RegisterSetupSize; i++)
    {
        WriteRegister(GateDriverRegisterSetup[i]);
        DEVICE_DELAY_US(OtherCommandDelay);
    }

    SendCommand(STGAP1AS_CMD_STOP_CONFIG);
    DEVICE_DELAY_US(StopConfigDelay);
}

/**
 * \brief Verify the configuration has been correctly set up
 *
 * \return bool - true if the configuration has been verified
 */
bool TeslaM3GateDriver::VerifyGateDriverConfig()
{
    uint16_t regValues[NumDriverChips];
    bool     result = true;
    for (uint8_t i = 1; i < RegisterSetupSize; i++)
    {
        const Register& reg = GateDriverRegisterSetup[i];
        ReadRegister(reg.reg, regValues);
        DEVICE_DELAY_US(RemoteRegReadDelay);

        uint16_t mask = 1;
        for (int chip = 0; chip < NumDriverChips; chip++)
        {
            if (reg.mask & mask)
            {
                result = result && (regValues[chip] == reg.value);
            }
            mask = mask << 1;
        }
    }

    return result;
}

/**
 * \brief Send a command to all driver chips
 *
 * \param cmd STGAP1AS_CMD_xx command to send
 */
void TeslaM3GateDriver::SendCommand(uint16_t cmd)
{
    DataBuffer cmdBuffer;
    memset(cmdBuffer, BuildCommand(cmd), NumDriverChips);

    SPI_pollingFIFOTransaction(
        sg_gateSpiBase, 16U, cmdBuffer, NULL, NumDriverChips, 2U);
}

/**
 * \brief Write a specific register
 *
 * \param reg Register structure detailing the register, value and required
 * chips
 */
void TeslaM3GateDriver::WriteRegister(const Register& reg)
{
    uint16_t cmd = STGAP1AS_CMD_WRITE_REG(reg.reg);
    uint16_t cmdCrc = crc8(cmd, STGAP1AS_SPI_CRC_INIT_VALUE);
    uint16_t dataCrc = crc8(reg.value, cmdCrc);

    // Invert the CRC as required by the STGAP1AS SPI protocol
    cmdCrc = ~cmdCrc;
    dataCrc = ~dataCrc;

    cmd = cmd << 8 | cmdCrc;

    const uint16_t nop = STGAP1AS_CMD_NOP << 8 |
                         (~crc8(STGAP1AS_CMD_NOP, STGAP1AS_SPI_CRC_INIT_VALUE));

    // Send the register write command ignoring any response (which is
    // undefined)
    {
        DataBuffer cmdBuffer;

        // Assemble a command buffer with all the commands or nops required for
        // each chip
        uint16_t mask = 1;
        for (int chip = 0; chip < NumDriverChips; chip++)
        {
            if (reg.mask & mask)
            {
                cmdBuffer[chip] = cmd;
            }
            else
            {
                cmdBuffer[chip] = nop;
            }
            mask = mask << 1;
        }

        SPITransaction transaction;
        SPI_pollingFIFOTransaction(
            sg_gateSpiBase, 16U, cmdBuffer, NULL, NumDriverChips, 2U);
    }

    DEVICE_DELAY_US(OtherCommandDelay);

    // Send the register data to be written ignoring any response (which is
    // undefined)
    {
        DataBuffer cmdDataBuffer;

        uint16_t data = reg.value << 8 | dataCrc;
        uint16_t mask = 1;
        for (int chip = 0; chip < NumDriverChips; chip++)
        {
            if (reg.mask & mask)
            {
                cmdDataBuffer[chip] = data;
            }
            else
            {
                cmdDataBuffer[chip] = nop;
            }
            mask = mask << 1;
        }

        SPITransaction transaction;
        SPI_pollingFIFOTransaction(
            sg_gateSpiBase, 16U, cmdDataBuffer, NULL, NumDriverChips, 2U);
    }
}

#ifdef DEBUG_STATE
/**
 * \brief Dump the device status to stdio
 *
 */
void TeslaM3GateDriver::DumpStatus()
{
    uint16_t statusValues[NumDriverChips];

    ReadRegister(STGAP1AS_REG_STATUS3, statusValues);
    for (int chip = 0; chip < NumDriverChips; chip++)
    {
        uint16_t crc =
            crc8(statusValues[chip] >> 8, STGAP1AS_SPI_CRC_INIT_VALUE);
        printf(
            "DumpStatus[%d]: %x, crc: %s\n",
            chip,
            statusValues[chip],
            crc == (statusValues[chip] & 0xFF) ? "CRC OK" : "CRC Fail");
    }
}
#endif

/**
 * \brief Read a specific register
 *
 * \param regNum Register number to read
 * \param values Register values array retrieved from all chips
 */
void TeslaM3GateDriver::ReadRegister(uint16_t regNum, uint16_t* values)
{
    uint16_t cmd = STGAP1AS_CMD_READ_REG(regNum);
    uint16_t cmdCrc = invert_byte(crc8(cmd, STGAP1AS_SPI_CRC_INIT_VALUE));
    cmd = cmd << 8 | cmdCrc;

    const uint16_t nop =
        STGAP1AS_CMD_NOP << 8 |
        invert_byte(crc8(STGAP1AS_CMD_NOP, STGAP1AS_SPI_CRC_INIT_VALUE));

    // Send the register read command ignoring any response (which is
    // undefined)
    {
        uint16_t cmdBuffer[NumDriverChips];
        memset(cmdBuffer, cmd, NumDriverChips);

        SPITransaction transaction;

        SPI_pollingFIFOTransaction(
            sg_gateSpiBase, 16U, cmdBuffer, NULL, NumDriverChips, 2U);
    }

    // Pessimistic for local reads but we'll assume that's not performance
    // critical
    DEVICE_DELAY_US(RemoteRegReadDelay);

    // Send a NOP while reading the data back from the register
    {
        uint16_t cmdBuffer[NumDriverChips];
        memset(cmdBuffer, nop, NumDriverChips);

        SPITransaction transaction;

        SPI_pollingFIFOTransaction(
            sg_gateSpiBase, 16U, cmdBuffer, values, NumDriverChips, 2U);
    }
}

/**
 * \brief Verify the status of a given register
 *
 * \param regNum Register number to read
 * \param value  Desired register value
 * \return True if the status matches the expected value
 */
bool TeslaM3GateDriver::VerifyRegister(uint16_t regNum, uint16_t value)
{
    uint16_t values[NumDriverChips];
    bool     result = true;
    uint16_t expectedValue =
        (value << 8) | crc8(value, STGAP1AS_SPI_CRC_INIT_VALUE);

    ReadRegister(regNum, values);

    for (int chip = 0; chip < NumDriverChips; chip++)
    {
        result = result && (values[chip] == expectedValue);
    }

    return result;
}
