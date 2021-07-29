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
#include "errormessage.h"
#include "focpwmgeneration.h"
#include "simulatorcurrent.h"
#include "simulatorencoder.h"
#include "simulatorpwmdriver.h"
#include "simulatorpwmgeneration.h"
#include <stdio.h>

// Pull in the whole C2000 namespace as this is platform specific code obviously
using namespace simulator;

void parm_Change(__attribute__((__unused__)) Param::PARAM_NUM paramNum)
{
}

int main()
{
    printf("PWM generation simulator\n\n");

    // Enable chatty mode to get all events
    PwmDriver::EnableChattyOutput();

    // Set up the error message log and set operating parameters to default
    ErrorMessage::ResetAll();
    Param::LoadDefaults();

    // Configure the PWM generation
    PwmGeneration::SetCurrentOffset(2048, 2048);

    // We need the pole pair ratio set to correctly calculate the rotation
    // frequency
    PwmGeneration::SetPolePairRatio(1);

    // Ensure the system thinks we should be going forwards
    Param::SetInt(Param::dir, 1);

    // initialise the controller gains from the default parameters
    PwmGeneration::SetControllerGains(
        Param::GetInt(Param::curkp),
        Param::GetInt(Param::curki),
        Param::GetInt(Param::fwkp));

    // Put in a bit of D and/or Q current to get it to do something
    // Param::Set(Param::manualid, FP_FROMFLT(0.6));
    Param::Set(Param::manualiq, FP_FROMFLT(0.6));

    // Provide some neutral values for the phase currents
    Current::SetPhase1(2048);
    Current::SetPhase2(2048);

    // Go for manual mode
    PwmGeneration::SetOpmode(MANUAL);

    // Run for a fixed number of iterations and print out what happens
    for (int ticks = 0; ticks < 5000; ticks++)
    {
        ErrorMessage::SetTime(ticks);
        PwmGeneration::Run();
    }

    return 0;
}
