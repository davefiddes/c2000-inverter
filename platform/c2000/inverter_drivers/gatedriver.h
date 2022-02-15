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
#ifndef GATEDRIVER_H
#define GATEDRIVER_H

#include "gatedriverinterface.h"
#include <stdint.h>

namespace c2000 {

/**
 * \brief Control the STGAP1AS gate driver chain on the Tesla M3 inverter
 */
class GateDriver
{
public:
    static bool Init();
    static bool IsFaulty();
    static void Enable();
    static void Disable();

    static void GetErrorStatus(
        uint16_t  regPos,
        uint16_t* regNo,
        uint16_t* status,
        uint16_t  statusLen);
    static void GetCrcStatus(
        uint16_t  regNo,
        uint16_t* status,
        uint16_t  statusLen);

    static uint16_t GetMaxStatusRegisters() 
    {
        return NumStatusErrorRegister;
    }
    static uint16_t GetMaxDriverChips()
    {
        return NumDriverChips;
    }

private:
    enum ChipMask
    {
        All = 0x3F,
        Odd = 0x15,
        Even = 0x2A
    };
    struct Register
    {
        uint16_t reg;
        uint16_t value;
        ChipMask mask;
        uint16_t validBitMask;
    };

private:
    static const uint16_t NumStatusErrorRegister = 3;
    static const uint16_t NumDriverChips = 6;
    static const Register GateDriverRegisterSetup[];
    static const uint16_t RegisterSetupSize;
    static const Register NullGateDriverRegister;

private:
    typedef uint16_t DataBuffer[NumDriverChips];

    // placing verify STATUSx register errors
    static uint16_t LastErrorStatus[NumStatusErrorRegister][NumDriverChips];
    // STATUSx crc failure encountered
    static uint16_t LastErrorCrc[NumStatusErrorRegister][NumDriverChips];
    // array with the available status registers and their register code
    static uint16_t StatusRegisterNumbers[NumStatusErrorRegister];

private:
    static void SetupGateDrivers();
    static bool VerifyGateDriverConfig();

    static void ResetLastErrors();

    static void SendCommand(uint16_t cmd);
    static void WriteRegister(const Register& reg);
    static void ReadRegister(uint16_t regNum, uint16_t* values);
    static bool VerifyRegister(
        uint16_t regNum,
        uint16_t validBits,
        uint16_t value);

private:
    static GateDriverInterface sm_interface;
};

} // namespace c2000

#endif // GATEDRIVER_H
