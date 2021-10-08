/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef PWMGENERATIONBASE_H
#define PWMGENERATIONBASE_H

#include "errormessage.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include "picontroller.h"
#include "sine_core.h"
#include <stdint.h>

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
class PwmGenerationBase
{
public:
    static void Run()
    {
        PwmModeT::Run();
    }

    static uint16_t GetAngle()
    {
        return angle;
    }

    static bool Tripped()
    {
        return tripped;
    }

    static void SetOpmode(Modes _opmode)
    {
        if (opmode == _opmode)
            return;
        opmode = _opmode;

        if (opmode != OFF)
        {
            tripped = false;
            pwmdigits = PwmDriverT::MinPwmDigits + Param::GetInt(Param::pwmfrq);
            shiftForTimer = SineCore::BITS - pwmdigits;
            PwmModeT::PwmInit();
        }

        switch (opmode)
        {
        default:
        case OFF:
            PwmDriverT::DisableOutput();
            PwmDriverT::ResetCpuLoad();
            break;

        case ACHEAT:
            PwmDriverT::DisableOutput();
            PwmDriverT::EnableACHeatOutput();
            break;

        case BOOST:
            PwmDriverT::DisableOutput();
            PwmDriverT::EnableChargeOutput(opmode);
            ConfigureChargeController();
            break;

        case BUCK:
            PwmDriverT::DisableOutput();
            PwmDriverT::EnableChargeOutput(opmode);
            ConfigureChargeController();
            break;

        case MANUAL:
        case RUN:
        case SINE:
            PwmDriverT::EnableOutput();
            break;
        }
    }

    static void SetAmpnom(s32fp amp)
    {
        ampnom = amp;
    }

    static void SetFslip(s32fp _fslip)
    {
        slipIncr = FrqToAngle(_fslip);
        fslip = _fslip;
    }

    static void SetTorquePercent(s32fp torque)
    {
        PwmModeT::SetTorquePercent(torque);
    }

    static void SetCurrentOffset(int32_t offset1, int32_t offset2)
    {
        ilofs[0] = FP_FROMINT(offset1);
        ilofs[1] = FP_FROMINT(offset2);

        if (CHK_BIPOLAR_OFS(offset1))
        {
            ErrorMessage::Post(ERR_HICUROFS1);
        }
        if (CHK_BIPOLAR_OFS(offset2))
        {
            ErrorMessage::Post(ERR_HICUROFS2);
        }

        SetCurrentLimitThreshold(Param::Get(Param::ocurlim));
    }

    //! Return the specified current offset - useful for testing
    static s32fp GetCurrentOffset(uint16_t num)
    {
        return ilofs[num];
    }

    static void SetCurrentLimitThreshold(s32fp ocurlim)
    {
        // We use the average offset and gain values because we only
        // have one reference channel per polarity
        s32fp iofs = (ilofs[0] + ilofs[1]) / 2;
        s32fp igain =
            (Param::Get(Param::il1gain) + Param::Get(Param::il2gain)) / 2;

        ocurlim = FP_MUL(igain, ocurlim);
        int16_t limNeg = FP_TOINT(iofs - ocurlim);
        int16_t limPos = FP_TOINT(iofs + ocurlim);

        PwmDriverT::SetOverCurrentLimits(limNeg, limPos);
    }

    static int32_t GetCpuLoad()
    {
        return PwmDriverT::GetCpuLoad();
    }

    static void SetChargeCurrent(s32fp cur)
    {
        chargeController.SetRef(cur);
    }

    static void SetPolePairRatio(int16_t ratio)
    {
        polePairRatio = ratio;
    }

    static uint16_t GetPwmDigits()
    {
        return pwmdigits;
    }

    static void SetOvercurrentTripped()
    {
        tripped = true;
    }

protected:
    static void Charge()
    {
        static s32fp iFlt;
        s32fp        il1 = GetCurrent(
            CurrentT::Phase1(), ilofs[0], Param::Get(Param::il1gain));
        s32fp il2 = GetCurrent(
            CurrentT::Phase2(), ilofs[1], Param::Get(Param::il2gain));

        il1 = ABS(il1);
        il2 = ABS(il2);

        s32fp ilMax = MAX(il1, il2);

        iFlt = IIRFILTER(iFlt, ilMax, Param::GetInt(Param::chargeflt));

        int32_t dc = chargeController.Run(iFlt);

        if (opmode == BOOST)
            Param::SetFixed(
                Param::idc, FP_MUL((FP_FROMINT(100) - ampnom), iFlt) / 100);
        else
            Param::SetFixed(Param::idc, iFlt);

        Param::SetInt(Param::amp, dc);
        Param::SetFixed(Param::il1, il1);
        Param::SetFixed(Param::il2, il2);

        PwmDriverT::SetChargeCurrent(dc);
    }

    static s32fp GetCurrent(uint16_t input, s32fp offset, s32fp gain)
    {
        s32fp il = FP_FROMINT((int32_t)input);
        il -= offset;
        return FP_DIV(il, gain);
    }

    static void ConfigureChargeController()
    {
        int32_t pwmin =
            FP_TOINT((Param::Get(Param::chargepwmin) * (1 << pwmdigits)) / 100);
        int32_t pwmax =
            FP_TOINT((Param::Get(Param::chargepwmax) * (1 << pwmdigits)) / 100);

        chargeController.SetCallingFrequency(pwmfrq);
        chargeController.SetMinMaxY(pwmin, pwmax);
        chargeController.SetGains(
            Param::GetInt(Param::chargekp), Param::GetInt(Param::chargeki));
        chargeController.SetRef(0);
        chargeController.PreloadIntegrator(pwmin);
    }

    static int32_t FrqToAngle(s32fp frq)
    {
        return FP_TOINT((frq << SineCore::BITS) / pwmfrq);
    }

    static s32fp DigitToDegree(int32_t angle)
    {
        return FP_FROMINT(angle) / (65536L / 360L);
    }

protected:
    static uint16_t      pwmfrq;
    static uint16_t      angle;
    static s32fp         ampnom;
    static uint16_t      slipIncr;
    static s32fp         fslip;
    static s32fp         frq;
    static uint_least8_t shiftForTimer;
    static Modes         opmode;
    static s32fp         ilofs[2];
    static int16_t       polePairRatio;
    static bool          tripped;
    static uint_least8_t pwmdigits;
    static PiController  chargeController;
};

// Instances of each member variable
template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
uint16_t PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::pwmfrq;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
uint16_t PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::angle;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
s32fp PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::ampnom;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
uint16_t PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::slipIncr;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
s32fp PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::fslip;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
s32fp PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::frq;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
uint_least8_t PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::shiftForTimer;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
Modes PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::opmode;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
s32fp PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::ilofs[2];

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
int16_t PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::polePairRatio;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
bool PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::tripped;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
uint_least8_t PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::pwmdigits;

template <typename PwmModeT, typename CurrentT, typename PwmDriverT>
PiController
    PwmGenerationBase<PwmModeT, CurrentT, PwmDriverT>::chargeController;

#endif // PWMGENERATIONBASE_H
