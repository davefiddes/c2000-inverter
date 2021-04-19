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

#include "anain.h"
#include "errormessage.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include "picontroller.h"
#include "sine_core.h"
#include <stdint.h>

template <typename PwmModeT, typename PwmDriverT>
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

        if (opmode != Modes::OFF)
        {
            tripped = false;
            pwmdigits = MIN_PWM_DIGITS + Param::GetInt(Param::pwmfrq);
            shiftForTimer = SineCore::BITS - pwmdigits;
            PwmDriverT::DriverInit();
        }

        switch (opmode)
        {
        default:
        case Modes::OFF:
            PwmDriverT::DisableOutput();
            PwmDriverT::ResetCpuLoad();
            break;

        case Modes::ACHEAT:
            PwmDriverT::DisableOutput();
            PwmDriverT::EnableACHeatOutput();
            break;

        case Modes::BOOST:
            PwmDriverT::DisableOutput();
            PwmDriverT::EnableChargeOutput(opmode);
            ConfigureChargeController();
            break;

        case Modes::BUCK:
            PwmDriverT::DisableOutput();
            PwmDriverT::EnableChargeOutput(opmode);
            ConfigureChargeController();
            break;

        case Modes::MANUAL:
        case Modes::RUN:
        case Modes::SINE:
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

    static void SetCurrentOffset(int offset1, int offset2)
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

    static void SetCurrentLimitThreshold(s32fp ocurlim)
    {
        // We use the average offset and gain values because we only
        // have one reference channel per polarity
        s32fp iofs = (ilofs[0] + ilofs[1]) / 2;
        s32fp igain =
            (Param::Get(Param::il1gain) + Param::Get(Param::il2gain)) / 2;

        ocurlim = FP_MUL(igain, ocurlim);
        int limNeg = FP_TOINT(iofs - ocurlim);
        int limPos = FP_TOINT(iofs + ocurlim);
        limNeg = MAX(0, limNeg);
        limPos = MIN(OCURMAX, limPos);

        PwmDriverT::SetOverCurrentLimits(limNeg, limPos);
    }

    static int GetCpuLoad()
    {
        return PwmDriverT::GetCpuLoad();
    }

    static void SetChargeCurrent(s32fp cur)
    {
        chargeController.SetRef(cur);
    }

    static void SetPolePairRatio(int ratio)
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
        s32fp        il1 =
            GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
        s32fp il2 =
            GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));

        il1 = ABS(il1);
        il2 = ABS(il2);

        s32fp ilMax = MAX(il1, il2);

        iFlt = IIRFILTER(iFlt, ilMax, Param::GetInt(Param::chargeflt));

        int dc = chargeController.Run(iFlt);

        if (opmode == Modes::BOOST)
            Param::SetFlt(
                Param::idc, FP_MUL((FP_FROMINT(100) - ampnom), iFlt) / 100);
        else
            Param::SetFlt(Param::idc, iFlt);

        Param::SetInt(Param::amp, dc);
        Param::SetFlt(Param::il1, il1);
        Param::SetFlt(Param::il2, il2);

        PwmDriverT::SetChargeCurrent(dc);
    }

    static s32fp GetCurrent(AnaIn& input, s32fp offset, s32fp gain)
    {
        s32fp il = FP_FROMINT(input.Get());
        il -= offset;
        return FP_DIV(il, gain);
    }

    static void ConfigureChargeController()
    {
        int pwmin =
            FP_TOINT((Param::Get(Param::chargepwmin) * (1 << pwmdigits)) / 100);
        int pwmax =
            FP_TOINT((Param::Get(Param::chargepwmax) * (1 << pwmdigits)) / 100);

        chargeController.SetCallingFrequency(pwmfrq);
        chargeController.SetMinMaxY(pwmin, pwmax);
        chargeController.SetGains(
            Param::GetInt(Param::chargekp), Param::GetInt(Param::chargeki));
        chargeController.SetRef(0);
        chargeController.PreloadIntegrator(pwmin);
    }

    static int FrqToAngle(s32fp frq)
    {

        return FP_TOINT((frq << SineCore::BITS) / pwmfrq);
    }

    static s32fp DigitToDegree(int angle)
    {
        return FP_FROMINT(angle) / (65536 / 360);
    }

protected:
    static uint16_t     pwmfrq;
    static uint16_t     angle;
    static s32fp        ampnom;
    static uint16_t     slipIncr;
    static s32fp        fslip;
    static s32fp        frq;
    static uint8_t      shiftForTimer;
    static Modes        opmode;
    static s32fp        ilofs[2];
    static int          polePairRatio;
    static bool         tripped;
    static uint8_t      pwmdigits;
    static PiController chargeController;
};

// Instances of each member variable
template <typename PwmModeT, typename PwmDriverT>
uint16_t PwmGenerationBase<PwmModeT, PwmDriverT>::pwmfrq;

template <typename PwmModeT, typename PwmDriverT>
uint16_t PwmGenerationBase<PwmModeT, PwmDriverT>::angle;

template <typename PwmModeT, typename PwmDriverT>
s32fp PwmGenerationBase<PwmModeT, PwmDriverT>::ampnom;

template <typename PwmModeT, typename PwmDriverT>
uint16_t PwmGenerationBase<PwmModeT, PwmDriverT>::slipIncr;

template <typename PwmModeT, typename PwmDriverT>
s32fp PwmGenerationBase<PwmModeT, PwmDriverT>::fslip;

template <typename PwmModeT, typename PwmDriverT>
s32fp PwmGenerationBase<PwmModeT, PwmDriverT>::frq;

template <typename PwmModeT, typename PwmDriverT>
uint8_t PwmGenerationBase<PwmModeT, PwmDriverT>::shiftForTimer;

template <typename PwmModeT, typename PwmDriverT>
Modes PwmGenerationBase<PwmModeT, PwmDriverT>::opmode;

template <typename PwmModeT, typename PwmDriverT>
s32fp PwmGenerationBase<PwmModeT, PwmDriverT>::ilofs[2];

template <typename PwmModeT, typename PwmDriverT>
int PwmGenerationBase<PwmModeT, PwmDriverT>::polePairRatio;

template <typename PwmModeT, typename PwmDriverT>
bool PwmGenerationBase<PwmModeT, PwmDriverT>::tripped;

template <typename PwmModeT, typename PwmDriverT>
uint8_t PwmGenerationBase<PwmModeT, PwmDriverT>::pwmdigits;

template <typename PwmModeT, typename PwmDriverT>
PiController PwmGenerationBase<PwmModeT, PwmDriverT>::chargeController;

#endif // PWMGENERATIONBASE_H
