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
#ifndef MOTORANALOGCAPTURE_H
#define MOTORANALOGCAPTURE_H

#include "driverlib.h"
#include <stdint.h>

namespace c2000 {

/**
 * \brief PWM synchronised capture of critical analog signals required for
 * operation of motor control
 */
class MotorAnalogCapture
{
public:
    static void Init();
    static void ConfigureSoc(ADC_Trigger trigger);

    static uint16_t PhaseACurrent();
    static uint16_t PhaseBCurrent();
    static uint16_t ResolverSine();
    static uint16_t ResolverCosine();

private:
    static void InitAdcChannel(uint32_t base);
};

} // namespace c2000

#endif // MOTORANALOGCAPTURE_H
