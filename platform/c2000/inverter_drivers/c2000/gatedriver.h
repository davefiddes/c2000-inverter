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

#include "c2000/gatedriverinterface.h"
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
    static const uint16_t NumDriverChips = 6;
    static const Register GateDriverRegisterSetup[];
    static const uint16_t RegisterSetupSize;
    static const Register NullGateDriverRegister;

private:
    typedef uint16_t DataBuffer[NumDriverChips];

private:
    static void SetupGateDrivers();
    static bool VerifyGateDriverConfig();

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
