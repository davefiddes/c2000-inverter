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
#include "mockpwmdriver.h"

void MockPwmDriver::DriverInit()
{
    instance->DriverInit();
}

void MockPwmDriver::EnableMasterOutput()
{
    instance->EnableMasterOutput();
}

void MockPwmDriver::DisableMasterOutput()
{
    instance->DisableMasterOutput();
}

void MockPwmDriver::EnableOutput()
{
    instance->EnableOutput();
}

void MockPwmDriver::DisableOutput()
{
    instance->DisableOutput();
}

void MockPwmDriver::SetPhasePwm(
    uint32_t phaseA,
    uint32_t phaseB,
    uint32_t phaseC)
{
    instance->SetPhasePwm(phaseA, phaseB, phaseC);
}

void MockPwmDriver::EnableChargeOutput(Modes opmode)
{
    instance->EnableChargeOutput(opmode);
}

void MockPwmDriver::EnableACHeatOutput()
{
    instance->EnableACHeatOutput();
}

void MockPwmDriver::SetOverCurrentLimits(int16_t limNeg, int16_t LimPos)
{
    instance->SetOverCurrentLimits(limNeg, LimPos);
};

uint16_t MockPwmDriver::TimerSetup(
    uint16_t deadtime,
    bool     activeLow,
    uint16_t pwmdigits)
{
    uint16_t pwmfrq = instance->TimerSetup(deadtime, activeLow, pwmdigits);
    return pwmfrq;
}

void MockPwmDriver::AcHeatTimerSetup()
{
    instance->AcHeatTimerSetup();
}

void MockPwmDriver::AcHeat(s32fp ampnom)
{
    instance->AcHeat(ampnom);
}

void MockPwmDriver::SetChargeCurrent(int16_t dc)
{
    instance->SetChargeCurrent(dc);
}

void MockPwmDriver::ResetCpuLoad()
{
    instance->ResetCpuLoad();
}

MockPwmDriverImpl* MockPwmDriver::instance = nullptr;

MockPwmDriverImpl::MockPwmDriverImpl()
{
    EXPECT_EQ(MockPwmDriver::instance, nullptr);
    MockPwmDriver::instance = this;
}

MockPwmDriverImpl::~MockPwmDriverImpl()
{
    EXPECT_EQ(MockPwmDriver::instance, this);
    MockPwmDriver::instance = nullptr;
}