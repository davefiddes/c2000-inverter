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

#include "c2000current.h"
#include "c2000encoder.h"
#include "c2000pwmdriver.h"
#include "focpwmgeneration.h"

// Global declaration of our PWM generation class - we only support Field
// Oriented Control at this point
typedef FocPwmGeneration<c2000::Current, c2000::Encoder, c2000::PwmDriver>
    PwmGeneration;
