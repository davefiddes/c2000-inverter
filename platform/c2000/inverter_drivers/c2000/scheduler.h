/*
 * This file is part of the Tesla M3 OSS Inverter project.
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

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

class Scheduler {

    // making these public to allow extern "C" interrupt handlers to reach them
public:

    static const uint16_t maxTasks = 3;

    static uint32_t periods[maxTasks];

    static void (*functions[maxTasks]) (void);
	
public:
    // init the scheduler / timer
    static void Init();

    // add a task with a specific period */
	static void AddTask(void (*function)(void), uint32_t msPeriod);

private:
    static void InitCPUTimers();
    static void ConfigureCPUTimer(uint32_t cpuTimer, uint32_t msPeriod);
};

#endif // SCHEDULER_H
