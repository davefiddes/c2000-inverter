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
#include "mockresolversample.h"

int16_t MockResolverSample::ResolverSine()
{
    return instance->ResolverSine();
}

int16_t MockResolverSample::ResolverCosine()
{
    return instance->ResolverCosine();
}

MockResolverSampleImpl* MockResolverSample::instance = nullptr;

MockResolverSampleImpl::MockResolverSampleImpl()
{
    EXPECT_EQ(MockResolverSample::instance, nullptr);
    MockResolverSample::instance = this;
}

MockResolverSampleImpl::~MockResolverSampleImpl()
{
    EXPECT_EQ(MockResolverSample::instance, this);
    MockResolverSample::instance = nullptr;
}
