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
#ifndef MOCKRESOLVERSAMPLE_H
#define MOCKRESOLVERSAMPLE_H

#include "my_fp.h"
#include <gmock/gmock.h>
#include <stdint.h>

class MockResolverSampleImpl;

/**
 * Mock implementation of a resolver sample source. This provides the static
 * methods expected by ResolverEncoder and acts as a trampoline to the
 * real mock methods in MockResolverSampleImpl.
 */
class MockResolverSample
{
public:
    static int16_t ResolverSine();
    static int16_t ResolverCosine();

private:
    static MockResolverSampleImpl* instance;
    friend class MockResolverSampleImpl;
};

class MockResolverSampleImpl
{
public:
    MockResolverSampleImpl();
    ~MockResolverSampleImpl();

    MOCK_METHOD(int16_t, ResolverSine, ());
    MOCK_METHOD(int16_t, ResolverCosine, ());
};

#endif // MOCKRESOLVERSAMPLE_H
