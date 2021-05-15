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
#ifndef SINEPWMGENERATIONFOC_H
#define SINEPWMGENERATIONFOC_H

#include "fu.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include "pwmgenerationbase.h"
#include "sine_core.h"
#include <stdint.h>

template <typename AnaInT, typename EncoderT, typename PwmDriverT>
class SinePwmGeneration : public PwmGenerationBase<
                              SinePwmGeneration<AnaInT, EncoderT, PwmDriverT>,
                              AnaInT,
                              PwmDriverT>
{
    using BaseT = PwmGenerationBase<
        SinePwmGeneration<AnaInT, EncoderT, PwmDriverT>,
        AnaInT,
        PwmDriverT>;

    // We need to allow PwmGenerationBase to call PwmInit
    friend BaseT;

public:
    static void Run()
    {
        if (BaseT::opmode == Modes::MANUAL || BaseT::opmode == Modes::RUN ||
            BaseT::opmode == Modes::SINE)
        {
            int dir = Param::GetInt(Param::dir);

            EncoderT::UpdateRotorAngle(dir);
            s32fp ampNomLimited = LimitCurrent();

            if (BaseT::opmode == Modes::SINE)
                CalcNextAngleConstant(dir);
            else
                CalcNextAngleAsync(dir);

            uint32_t amp = MotorVoltage::GetAmpPerc(BaseT::frq, ampNomLimited);

            SineCore::SetAmp(amp);
            Param::SetInt(Param::amp, amp);
            Param::SetFlt(Param::fstat, BaseT::frq);
            Param::SetFlt(Param::angle, BaseT::DigitToDegree(BaseT::angle));
            SineCore::Calc(BaseT::angle);

            /* Shut down PWM on zero voltage request */
            if (0 == amp || 0 == dir)
            {
                PwmDriverT::DisableMasterOutput();
            }
            else
            {
                PwmDriverT::EnableMasterOutput();
            }

            /* Match to PWM resolution */
            PwmDriverT::SetPhasePwm(
                SineCore::DutyCycles[0] >> BaseT::shiftForTimer,
                SineCore::DutyCycles[1] >> BaseT::shiftForTimer,
                SineCore::DutyCycles[2] >> BaseT::shiftForTimer);
        }
        else if (BaseT::opmode == Modes::BOOST || BaseT::opmode == Modes::BUCK)
        {
            BaseT::Charge();
        }
        else if (BaseT::opmode == Modes::ACHEAT)
        {
            PwmDriverT::AcHeat(BaseT::ampnom);
        }
    }

    static void SetTorquePercent(s32fp torque)
    {
        s32fp fslipmin = Param::Get(Param::fslipmin);
        s32fp ampmin = Param::Get(Param::ampmin);
        s32fp slipstart = Param::Get(Param::slipstart);
        s32fp ampnomLocal;
        s32fp fslipspnt = 0;

        if (torque >= 0)
        {
            /* In async mode first X% throttle commands amplitude, X-100% raises
             * slip */
            ampnomLocal =
                ampmin + (100 - FP_TOINT(ampmin)) * FP_DIV(torque, slipstart);

            if (torque >= slipstart)
            {
                s32fp fstat = Param::Get(Param::fstat);
                s32fp fweak = Param::Get(Param::fweakcalc);
                s32fp fslipmax = Param::Get(Param::fslipmax);

                if (fstat > fweak)
                {
                    s32fp fconst = Param::Get(Param::fconst);
                    s32fp fslipconstmax = Param::Get(Param::fslipconstmax);
                    // Basically, for every Hz above fweak we get a fraction of
                    // the difference between fslipconstmax and fslipmax
                    // of additional slip
                    fslipmax += FP_MUL(
                        FP_DIV(fstat - fweak, fconst - fweak),
                        fslipconstmax - fslipmax);
                    fslipmax = MIN(
                        fslipmax, fslipconstmax); // never exceed fslipconstmax!
                }

                s32fp fslipdiff = fslipmax - fslipmin;
                fslipspnt =
                    fslipmin + (FP_MUL(fslipdiff, (torque - slipstart)) /
                                (100 - FP_TOINT(slipstart)));
            }
            else
            {
                fslipspnt = fslipmin;
            }
        }
        else
        {
            u32fp brkrampstr = (u32fp)Param::Get(Param::brkrampstr);

            ampnomLocal = -torque;

            fslipspnt = -fslipmin;
            if (EncoderT::GetRotorFrequency() < brkrampstr)
            {
                ampnomLocal = FP_TOINT(
                    FP_DIV(EncoderT::GetRotorFrequency(), brkrampstr) *
                    ampnomLocal);
            }
        }

        ampnomLocal = MIN(ampnomLocal, FP_FROMINT(100));
        // anticipate sudden changes by filtering
        BaseT::ampnom = IIRFILTER(BaseT::ampnom, ampnomLocal, 4);
        BaseT::fslip = IIRFILTER(BaseT::fslip, fslipspnt, 4);
        Param::Set(Param::ampnom, BaseT::ampnom);
        Param::Set(Param::fslipspnt, BaseT::fslip);

        BaseT::slipIncr = BaseT::FrqToAngle(BaseT::fslip);
    }

private:
    enum EdgeType // Sine
    {
        NoEdge,
        PosEdge,
        NegEdge
    };

private:
    static const uint16_t SHIFT_180DEG = 32768;
    static const uint16_t SHIFT_90DEG = 16384;

protected:
    static void PwmInit()
    {
        BaseT::SetCurrentOffset(AnaInT::il1.Get(), AnaInT::il2.Get());
        BaseT::pwmfrq = PwmDriverT::TimerSetup(
            Param::GetInt(Param::deadtime),
            Param::GetInt(Param::pwmpol),
            BaseT::pwmdigits);
        BaseT::slipIncr = BaseT::FrqToAngle(BaseT::fslip);
        EncoderT::SetPwmFrequency(BaseT::pwmfrq);

        PwmDriverT::DriverInit();

        if (BaseT::opmode == Modes::ACHEAT)
            PwmDriverT::AcHeatTimerSetup();
    }

private:
    static s32fp ProcessCurrents()
    {
        static s32fp    currentMax[2];
        static int      samples[2] = { 0 };
        static int      sign = 1;
        static EdgeType lastEdge[2] = { PosEdge, PosEdge };

        s32fp il1 = BaseT::GetCurrent(
            AnaInT::il1, BaseT::ilofs[0], Param::Get(Param::il1gain));
        s32fp il2 = BaseT::GetCurrent(
            AnaInT::il2, BaseT::ilofs[1], Param::Get(Param::il2gain));
        s32fp    rms;
        s32fp    il1PrevRms = Param::Get(Param::il1rms);
        s32fp    il2PrevRms = Param::Get(Param::il2rms);
        EdgeType edge = CalcRms(
            il1, lastEdge[0], currentMax[0], rms, samples[0], il1PrevRms);

        if (edge != NoEdge)
        {
            Param::SetFlt(Param::il1rms, rms);

            if (BaseT::opmode != Modes::BOOST || BaseT::opmode != Modes::BUCK)
            {
                // rough approximation as we do not take power factor into
                // account
                s32fp idc = (SineCore::GetAmp() * rms) / SineCore::MAXAMP;
                idc = FP_DIV(
                    idc, FP_FROMFLT(1.2247)); // divide by sqrt(3)/sqrt(2)
                idc *= BaseT::fslip < 0 ? -1 : 1;
                Param::SetFlt(Param::idc, idc);
            }
        }
        if (CalcRms(
                il2, lastEdge[1], currentMax[1], rms, samples[1], il2PrevRms))
        {
            Param::SetFlt(Param::il2rms, rms);
        }

        s32fp ilMax = sign * GetIlMax(il1, il2);

        Param::SetFlt(Param::il1, il1);
        Param::SetFlt(Param::il2, il2);
        Param::SetFlt(Param::ilmax, ilMax);

        return ilMax;
    }

    static void CalcNextAngleAsync(int dir)
    {
        static uint16_t slipAngle = 0;
        uint16_t        rotorAngle = EncoderT::GetRotorAngle();

        BaseT::frq =
            BaseT::polePairRatio * EncoderT::GetRotorFrequency() + BaseT::fslip;
        slipAngle += dir * BaseT::slipIncr;

        if (BaseT::frq < 0)
            BaseT::frq = 0;

        BaseT::angle = BaseT::polePairRatio * rotorAngle + slipAngle;
    }

    static void CalcNextAngleConstant(int dir)
    {
        BaseT::frq = BaseT::fslip;
        BaseT::angle += dir * BaseT::slipIncr;

        if (BaseT::frq < 0)
            BaseT::frq = 0;
    }

    static s32fp GetIlMax(s32fp il1, s32fp il2)
    {
        s32fp il3 = -il1 - il2;
        s32fp offset = SineCore::CalcSVPWMOffset(il1, il2, il3) / 2;
        offset = ABS(offset);
        il1 = ABS(il1);
        il2 = ABS(il2);
        il3 = ABS(il3);
        s32fp ilMax = MAX(il1, il2);
        ilMax = MAX(ilMax, il3);
        ilMax -= offset;

        return ilMax;
    }

    static s32fp LimitCurrent()
    {
        static s32fp curLimSpntFiltered = 0, slipFiltered = 0;
        s32fp        slipmin = Param::Get(Param::fslipmin);
        s32fp        imax = Param::Get(Param::iacmax);
        s32fp        ilMax = ProcessCurrents();

        // setting of 0 disables current limiting
        if (imax == 0)
            return BaseT::ampnom;

        s32fp a = imax / 20; // Start acting at 80% of imax
        s32fp imargin = imax - ilMax;
        s32fp curLimSpnt = FP_DIV(100 * imargin, a);
        s32fp slipSpnt = FP_DIV(FP_MUL(BaseT::fslip, imargin), a);
        slipSpnt = MAX(slipmin, slipSpnt);
        curLimSpnt = MAX(FP_FROMINT(40), curLimSpnt); // Never go below 40%
        int filter = Param::GetInt(
            curLimSpnt < curLimSpntFiltered ? Param::ifltfall :
                                              Param::ifltrise);
        curLimSpntFiltered = IIRFILTER(curLimSpntFiltered, curLimSpnt, filter);
        slipFiltered = IIRFILTER(slipFiltered, slipSpnt, 1);

        s32fp ampNomLimited = MIN(BaseT::ampnom, curLimSpntFiltered);
        slipSpnt = MIN(BaseT::fslip, slipFiltered);
        BaseT::slipIncr = BaseT::FrqToAngle(slipSpnt);

        if (curLimSpnt < BaseT::ampnom)
            ErrorMessage::Post(ERR_CURRENTLIMIT);

        return ampNomLimited;
    }

    static EdgeType CalcRms(
        s32fp     il,
        EdgeType& lastEdge,
        s32fp&    max,
        s32fp&    rms,
        int&      samples,
        s32fp     prevRms)
    {
        const s32fp oneOverSqrt2 = FP_FROMFLT(0.707106781187);
        int         minSamples = BaseT::pwmfrq / (4 * FP_TOINT(BaseT::frq));
        EdgeType    edgeType = NoEdge;

        minSamples = MAX(10, minSamples);

        if (samples > minSamples)
        {
            if (lastEdge == NegEdge && il > 0)
                edgeType = PosEdge;
            else if (lastEdge == PosEdge && il < 0)
                edgeType = NegEdge;
        }

        if (edgeType != NoEdge)
        {
            rms = (FP_MUL(oneOverSqrt2, max) + prevRms) /
                  2; // average with previous rms reading

            max = 0;
            samples = 0;
            lastEdge = edgeType;
        }

        il = ABS(il);
        max = MAX(il, max);
        samples++;

        return edgeType;
    }
};

#endif // SINEPWMGENERATION_H
