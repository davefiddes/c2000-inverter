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

extern "C" {
    #include "device.h"
}
#include "c2000/scheduler.h"

uint32_t Scheduler::periods[Scheduler::maxTasks] = {0, 0, 0};

void (*Scheduler::functions[Scheduler::maxTasks])(void) = {0, 0, 0};

const uint32_t interruptTimers[3] = {INT_TIMER0, INT_TIMER1, INT_TIMER2};
const uint32_t cpuTimerBases[3] = {CPUTIMER0_BASE, CPUTIMER1_BASE, CPUTIMER2_BASE};

extern "C" {
// Irq handler for CPU Timer 0
__interrupt void cpuTimer0ISR(void) {

    if(Scheduler::functions[0] != 0) {
        Scheduler::functions[0]();
    }

    // ack the interrupt to be able to see more interrupts
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

// Irq handler for CPU Timer 1
__interrupt void cpuTimer1ISR(void) {
    if(Scheduler::functions[1] != 0) Scheduler::functions[1]();
}

// Irq handler for CPU Timer 2
__interrupt void cpuTimer2ISR(void) {
    if(Scheduler::functions[2] != 0) Scheduler::functions[2]();
}
}

/*
 * All timers are initialized even if there are no tasks.
 */
void Scheduler::Init() {

    // ISRs for each CPU Timer interrupt
    Interrupt_register(interruptTimers[0], &cpuTimer0ISR);
    Interrupt_register(interruptTimers[1], &cpuTimer1ISR);
    Interrupt_register(interruptTimers[2], &cpuTimer2ISR);

    // Initializes the Device Peripheral. Initialize the CPU Timers.
    Scheduler::InitCPUTimers();
}

void Scheduler::AddTask(void (*function)(void), uint32_t msPeriod) {

    // check if there is a free timer slot available
    uint16_t freeTimer = 0xffff;

    uint16_t t = 0;
    for(t = 0; t < Scheduler::maxTasks; t++) {

        if((Scheduler::periods[t] == 0) && (Scheduler::functions[t] == 0)) {
            freeTimer = t;
            break;
        }
    }

    if(freeTimer >= Scheduler::maxTasks) {
        // TODO: error
        return;
    }

    Scheduler::functions[freeTimer] = function;
    Scheduler::periods[freeTimer] = msPeriod;

    // Starts CPU-Timer
    Scheduler::ConfigureCPUTimer(cpuTimerBases[freeTimer], msPeriod);

    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in configCPUTimer and initCPUTimers, the below settings must also
    // be updated.
    CPUTimer_enableInterrupt(cpuTimerBases[freeTimer]);

    // Enables CPU int1, int13, and int14 which are connected to
    // CPU-Timer 0, CPU-Timer 1, and CPU-Timer 2 respectively.
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    Interrupt_enable(interruptTimers[freeTimer]);

    CPUTimer_startTimer(cpuTimerBases[freeTimer]);
}

/*
 * Initialize all CPU timers
 */
void Scheduler::InitCPUTimers(void)
{
    uint16_t t = 0;
    for(t = 0; t < Scheduler::maxTasks; t++) {

        // Initialize timer period to maximum
        CPUTimer_setPeriod(cpuTimerBases[t], 0xFFFFFFFF);

        // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
        CPUTimer_setPreScaler(cpuTimerBases[t], 0);

        // Make sure timer is stopped
        CPUTimer_stopTimer(cpuTimerBases[t]);

        // Reload all counter register with period value
        CPUTimer_reloadTimerCounter(cpuTimerBases[t]);
    }
}


/* \brief This function initializes the selected timer
 *     with a period in milli seconds
 */
void Scheduler::ConfigureCPUTimer(uint32_t cpuTimer, uint32_t msPeriod) {

    // Initialize timer period:
    uint32_t timerPeriod = (uint32_t)(DEVICE_SYSCLK_FREQ / 1000 * msPeriod);

    CPUTimer_setPeriod(cpuTimer, timerPeriod);

    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    CPUTimer_setPreScaler(cpuTimer, 0);

    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    CPUTimer_enableInterrupt(cpuTimer);
}
