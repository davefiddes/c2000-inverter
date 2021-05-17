/*
 * This file is part of the tumanako_vc project.
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

#ifndef MATCHERHELPER_H
#define MATCHERHELPER_H

#include "my_fp.h"
#include <gmock/gmock.h>

/**
 * GMock matcher for matching integer values within a range
 */
MATCHER_P2(IntNear, value, range, "")
{
    return (arg <= (value + range)) && (arg >= (value - range));
}

/**
 * GMock matcher for matching Fixed Point (s32fp) values within a range
 */
MATCHER_P2(FPNear, value, range, "")
{
    return (arg <= FP_FROMFLT(value + range)) &&
           (arg >= FP_FROMFLT(value - range));
}

#endif // MATCHERHELPER_H
