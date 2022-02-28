/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 * Copyright (C) 2019 Nail Güzel
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
#ifndef RESOLVERENCODER_H
#define RESOLVERENCODER_H

#include "errormessage.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include <math.h>
#include <stdint.h>

namespace encoder {

//! namespace for internal details to avoid polluting the interface
namespace details {

//! Number of integer counts that represent a full rotation of the rotor
static const uint32_t FullRotationInt = 65536;

static const float Pi = 3.14159265359;

static const float TwoPi = 2 * Pi;

//! Angle difference at which we assume jitter to become irrelevant
static const float StableAngle = (10.0 * TwoPi) / 360.0;

//! Minimum amplitude the resolver signals must reach during start up
//! stablisation period
static const uint16_t MinResolverAmplitude = 1000;

} // namespace details

template <typename ResolverSampleT>
class ResolverEncoder
{
public:
    //
    //! \brief Return the encoder to a known initial state
    //
    static void Reset()
    {
        sm_fullTurns = 0;
        sm_pwmFrq = 1;
        sm_resolverMin = 0;
        sm_resolverMax = 0;
        sm_lastFrequency = 0.0f;
        sm_detectedDirection = Stationary;
        sm_startupDelay = 4000;
        sm_lastAngle = 0.0f;
        sm_angle = 0.0f;
        sm_poleCounter = 0;
        sm_turnsSinceLastSample = 0;
    }

    //
    //! \brief Return whether the north position has been seen and the absolute
    //! position is known. This is always true for a resolver.
    //
    static bool SeenNorthSignal()
    {
        return true;
    }

    //
    //! \brief Call at PWM frequency to update the encoder's position
    //!
    //! \param dir   Desired direction of rotation -1 (backwards), 0
    //! (stationary) or 1 (forwards)
    //
    static void UpdateRotorAngle(__attribute__((__unused__)) int dir)
    {
        sm_angle = DecodeAngle();

        UpdateTurns();

        if ((sm_lastAngle <= details::Pi) && (sm_angle > details::Pi))
        {
            if (sm_poleCounter == 0)
            {
                sm_fullTurns++;
                sm_poleCounter = Param::GetInt(Param::respolepairs);
            }
            else
            {
                sm_poleCounter--;
            }
        }

        sm_startupDelay = sm_startupDelay > 0 ? sm_startupDelay - 1 : 0;
        sm_lastAngle = sm_angle;
    }

    //
    //! Update rotor frequency.
    //!
    //! \param callingFrequency Frequency at which this function is called in Hz
    //
    static void UpdateRotorFrequency(int callingFrequency)
    {
        int absTurns = ABS(sm_turnsSinceLastSample);
        if (sm_startupDelay == 0 && absTurns > details::StableAngle)
        {
            sm_lastFrequency = (callingFrequency * absTurns) / details::TwoPi;
            sm_detectedDirection =
                sm_turnsSinceLastSample > 0 ? Forwards : Backwards;
        }
        else
        {
            sm_lastFrequency = 0;
        }
        sm_turnsSinceLastSample = 0;
    }

    //
    //! \brief Set the current PWM frequency
    //!
    //! \param frq Frequency in Hz
    //
    static void SetPwmFrequency(uint32_t frq)
    {
        sm_pwmFrq = frq;
    }

    //
    //! \brief Returns current angle of motor shaft to some arbitrary 0-axis
    //!
    //! \return angle in digit (2Pi=65536)
    //
    static uint16_t GetRotorAngle()
    {
        return (sm_angle * details::FullRotationInt) / details::TwoPi;
    }

    //
    //! \brief Get current speed in rpm
    //
    static uint32_t GetSpeed()
    {
        return (60 * sm_lastFrequency) / Param::GetInt(Param::respolepairs);
    }

    //
    //! \brief Return the number of complete turns since power on
    //
    static uint32_t GetFullTurns()
    {
        return sm_fullTurns;
    }

    //
    //! \brief Return rotor frequency in Hz
    //
    static u32fp GetRotorFrequency()
    {
        return FP_FROMFLT(sm_lastFrequency);
    }

    //
    //! \brief Return the rotor direction with -1 (backwards), 0 (stationary) or
    //! 1 (forwards)
    //
    static int GetRotorDirection()
    {
        return sm_detectedDirection;
    }

private:
    enum Direction
    {
        Backwards = -1,
        Stationary = 0,
        Forwards = 1
    };

private:
    //
    //! Figure out how many rotations we have had
    //!
    static void UpdateTurns()
    {
        float signedDiff = sm_angle - sm_lastAngle;
        float absDiff = fabs(signedDiff);
        int   sign = signedDiff < 0 ? -1 : 1;

        if (absDiff > details::Pi) // wrap detection
        {
            sign = -sign;
            signedDiff += sign * details::TwoPi;
            absDiff = fabs(signedDiff);
        }

        sm_turnsSinceLastSample += signedDiff;
    }

    //
    //! Calculates angle from sin and cos value
    //!
    //! Assumes that the samples values were captured at the positive peak of
    //! the exciter sine wave
    //
    static float DecodeAngle()
    {
        int16_t sin = ResolverSampleT::ResolverSine();
        int16_t cos = ResolverSampleT::ResolverCosine();
        ;

        // Wait for signal to reach usable amplitude
        if ((sm_resolverMax - sm_resolverMin) > details::MinResolverAmplitude)
        {
            return atan2(sin, cos);
        }
        else
        {
            int16_t temp = MIN(sin, cos);
            sm_resolverMin = MIN(temp, sm_resolverMin);
            temp = MAX(sin, cos);
            sm_resolverMax = MAX(temp, sm_resolverMax);

            if (sm_startupDelay == 0)
            {
                ErrorMessage::Post(ERR_LORESAMP);
            }
            return 0;
        }
    }

private:
    //! Number of complete turns since power on
    static uint32_t sm_fullTurns;

    //! PWM Frequency in Hz
    static uint32_t sm_pwmFrq;

    //! Resolver signal minimum amplitude
    static int32_t sm_resolverMin;

    //! Resolver signal maximum amplitude
    static int32_t sm_resolverMax;

    //! Number of calls to UpdateRotorAngle() before we are happy to start
    //! normal operation
    static int32_t sm_startupDelay;

    //! Most recently calculated rotor frequency in Hz */
    static float sm_lastFrequency;

    //! Rotor direction
    static Direction sm_detectedDirection;

    //! Current rotor angle
    static float sm_angle;

    //! Last recorded rotor angle
    static float sm_lastAngle;

    //! Number of poles we have seen in the current rotation
    static int sm_poleCounter;

    static int32_t sm_turnsSinceLastSample;
};

// Instances of each member variable
template <typename ResolverSampleT>
uint32_t ResolverEncoder<ResolverSampleT>::sm_fullTurns;

template <typename ResolverSampleT>
uint32_t ResolverEncoder<ResolverSampleT>::sm_pwmFrq;

template <typename ResolverSampleT>
int32_t ResolverEncoder<ResolverSampleT>::sm_resolverMin;

template <typename ResolverSampleT>
int32_t ResolverEncoder<ResolverSampleT>::sm_resolverMax;

template <typename ResolverSampleT>
int32_t ResolverEncoder<ResolverSampleT>::sm_startupDelay;

template <typename ResolverSampleT>
float ResolverEncoder<ResolverSampleT>::sm_lastFrequency;

template <typename ResolverSampleT>
ResolverEncoder<ResolverSampleT>::Direction
    ResolverEncoder<ResolverSampleT>::sm_detectedDirection;

template <typename ResolverSampleT>
float ResolverEncoder<ResolverSampleT>::sm_angle;

template <typename ResolverSampleT>
float ResolverEncoder<ResolverSampleT>::sm_lastAngle;

template <typename ResolverSampleT>
int ResolverEncoder<ResolverSampleT>::sm_poleCounter;

template <typename ResolverSampleT>
int32_t ResolverEncoder<ResolverSampleT>::sm_turnsSinceLastSample;

} // namespace encoder

#endif // RESOLVERENCODER_H
