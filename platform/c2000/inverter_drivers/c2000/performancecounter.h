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
#ifndef C2000PERFORMANCECOUNTER_H
#define C2000PERFORMANCECOUNTER_H

#include "driverlib.h"
#include <stdint.h>

namespace c2000 {

/**
 * Simple performance counter using TIMER 0
 */
class PerformanceCounter
{
public:
    static const uint32_t perfTimer = CPUTIMER2_BASE;

public:
    static void Init()
    {
        CPUTimer_setPeriod(perfTimer, 0xFFFFFFFF);
        CPUTimer_setPreScaler(perfTimer, CPUTIMER_CLOCK_PRESCALER_1);
        CPUTimer_stopTimer(perfTimer);
        CPUTimer_reloadTimerCounter(perfTimer);
        CPUTimer_setEmulationMode(
            perfTimer, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
        CPUTimer_startTimer(perfTimer);
    }

    static uint32_t GetCount()
    {
        return CPUTimer_getTimerCount(perfTimer);
    }
};

} // namespace c2000

#endif // C2000PERFORMANCECOUNTER_H
