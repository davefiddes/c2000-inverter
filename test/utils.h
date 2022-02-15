/*
 * This file is part of the stm32_sine project.
 *
 * Copyright (C) 2022 Bernd Ocklin <bernd@ocklin.de>
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

#ifndef UTILS_H
#define UTILS_H

#if HAVE_NUMBERS_H
#include <numbers>
#else
#ifndef HAVE_NUMBERS_PI
    static constexpr float pi = 3.14159265358979;
#endif
#ifndef HAVE_NUMBERS_INV_SQRT3
    static constexpr float inv_sqrt3 = 0.57735026919;
#endif
#endif

#endif // UTILS_H
