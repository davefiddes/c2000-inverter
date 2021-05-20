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
#include "mockencoder.h"

bool MockEncoder::SeenNorthSignal()
{
    return instance->SeenNorthSignal();
}

void MockEncoder::UpdateRotorAngle(int dir)
{
    instance->UpdateRotorAngle(dir);
}

void MockEncoder::UpdateRotorFrequency(int callingFrequency)
{
    instance->UpdateRotorFrequency(callingFrequency);
}

void MockEncoder::SetPwmFrequency(uint32_t frq)
{
    instance->SetPwmFrequency(frq);
}

uint16_t MockEncoder::GetRotorAngle()
{
    return instance->GetRotorAngle();
}

u32fp MockEncoder::GetRotorFrequency()
{
    return instance->GetRotorFrequency();
}

int MockEncoder::GetRotorDirection()
{
    return instance->GetRotorDirection();
}

MockEncoderImpl* MockEncoder::instance = nullptr;

MockEncoderImpl::MockEncoderImpl()
{
    EXPECT_EQ(MockEncoder::instance, nullptr);
    MockEncoder::instance = this;
}

MockEncoderImpl::~MockEncoderImpl()
{
    EXPECT_EQ(MockEncoder::instance, this);
    MockEncoder::instance = nullptr;
}
