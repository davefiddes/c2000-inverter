/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef C2000PWMDRIVER_H
#define C2000PWMDRIVER_H

#include "my_fp.h"
#include "params.h"
#include <stdint.h>

namespace c2000 {

class PwmDriver
{
public:
    static const uint16_t MinPwmDigits = 11;

public:
    static void DriverInit();
    static void EnableMasterOutput();
    static void DisableMasterOutput();
    static void EnableOutput();
    static void DisableOutput();
    static void SetPhasePwm(uint32_t phaseA, uint32_t phaseB, uint32_t phaseC);
    static void EnableChargeOutput(Modes opmode);
    static void EnableACHeatOutput();
    static void SetOverCurrentLimits(int limNeg, int LimPos);
    static uint16_t TimerSetup(
        uint16_t deadtime,
        bool     activeLow,
        uint16_t pwmdigits);
    static int  GetPwmFrq();
    static void AcHeatTimerSetup();
    static void AcHeat(s32fp ampnom);
    static void SetChargeCurrent(int dc);
    static int  GetCpuLoad();
    static void ResetCpuLoad();
};

} // namespace c2000

#endif // C2000PWMDRIVER_H
