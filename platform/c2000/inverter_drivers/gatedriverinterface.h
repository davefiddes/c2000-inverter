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
#ifndef GATEDRIVERINTERFACE_H
#define GATEDRIVERINTERFACE_H

#include <stdint.h>

namespace c2000 {

/**
 * \brief Low level hardware interface to the STGAP1AS gate driver chain in the
 * Tesla M3 inverter
 */
class GateDriverInterface
{
public:
    static const uint16_t NumDriverChips = 6;

    typedef uint16_t DataBuffer[NumDriverChips];

public:
    void Init();
    void SendData(DataBuffer writeData, DataBuffer readData);
    void Shutdown();
    void Resume();
    bool IsShutdown();

private:
    uint16_t m_gateShutdownPin;

    uint16_t m_gateCsPin;

    uint32_t m_gateClkCfg;
    uint32_t m_gateMosiCfg;
    uint32_t m_gateMisoCfg;

    uint32_t m_gateSpiBase;
};

} // namespace c2000

#endif // GATEDRIVERINTERFACE_H
