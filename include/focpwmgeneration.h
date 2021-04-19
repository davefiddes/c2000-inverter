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
#ifndef FOCPWMGENERATION_H
#define FOCPWMGENERATION_H

#include "foc.h"
#include "inc_encoder.h"
#include "my_fp.h"
#include "picontroller.h"
#include "anain.h"
#include "pwmgenerationbase.h"

template <typename PwmDriverT>
class FocPwmGeneration
: public PwmGenerationBase<FocPwmGeneration<PwmDriverT>, PwmDriverT>
{
    using BaseT = PwmGenerationBase<FocPwmGeneration<PwmDriverT>, PwmDriverT>;

public:
    static void SetControllerGains(int kp, int ki, int fwkp)
    {
        qController.SetGains(kp, ki);
        dController.SetGains(kp, ki);
        fwBaseGain = fwkp;
        curki = ki;
    }

    static void Run()
    {
        if (BaseT::opmode == Modes::MANUAL || BaseT::opmode == Modes::RUN)
        {
            static s32fp frqFiltered = 0, idcFiltered = 0;
            int          dir = Param::GetInt(Param::dir);
            int          kifrqgain = Param::GetInt(Param::curkifrqgain);
            s32fp        id, iq;

            Encoder::UpdateRotorAngle(dir);

            CalcNextAngleSync(dir);

            frqFiltered = IIRFILTER(frqFiltered, BaseT::frq, 8);
            int moddedKi = curki + kifrqgain * FP_TOINT(frqFiltered);

            qController.SetIntegralGain(moddedKi);
            dController.SetIntegralGain(moddedKi);
            fwController.SetProportionalGain(fwBaseGain * dir);

            ProcessCurrents(id, iq);

            if (BaseT::opmode == Modes::RUN && initwait == 0)
            {
                s32fp fwIdRef = idref <= 0 ? fwController.Run(iq) : 0;
                dController.SetRef(idref + fwIdRef);
            }
            else if (BaseT::opmode == Modes::MANUAL)
            {
                idref = Param::Get(Param::manualid);
                dController.SetRef(idref);
                qController.SetRef(Param::Get(Param::manualiq));
            }

            int32_t ud = dController.Run(id);
            int32_t qlimit = FOC::GetQLimit(ud);
            qController.SetMinMaxY(-qlimit, qlimit);
            int32_t uq = qController.Run(iq);
            FOC::InvParkClarke(ud, uq, BaseT::angle);

            s32fp idc = (iq * uq + id * ud) / FOC::GetMaximumModulationIndex();
            idc = FP_MUL(idc, dcCurFac);
            idcFiltered = IIRFILTER(idcFiltered, idc, 10);

            Param::SetFlt(Param::fstat, BaseT::frq);
            Param::SetFlt(Param::angle, BaseT::DigitToDegree(BaseT::angle));
            Param::SetFlt(Param::idc, idcFiltered);
            Param::SetInt(Param::uq, uq);
            Param::SetInt(Param::ud, ud);

            /* Shut down PWM on stopped motor, neutral gear or init phase */
            if ((0 == BaseT::frq && 0 == idref && 0 == qController.GetRef()) ||
                0 == dir || initwait > 0)
            {
                PwmDriverT::DisableMasterOutput();
                dController.ResetIntegrator();
                qController.ResetIntegrator();
                fwController.ResetIntegrator();
                RunOffsetCalibration();
            }
            else
            {
                PwmDriverT::EnableMasterOutput();
            }

            PwmDriverT::SetPhasePwm(
                FOC::DutyCycles[0] >> BaseT::shiftForTimer,
                FOC::DutyCycles[1] >> BaseT::shiftForTimer,
                FOC::DutyCycles[2] >> BaseT::shiftForTimer);
        }
        else if (BaseT::opmode == Modes::BOOST || BaseT::opmode == Modes::BUCK)
        {
            initwait = 0;
            BaseT::Charge();
        }
        else if (BaseT::opmode == Modes::ACHEAT)
        {
            initwait = 0;
            PwmDriverT::AcHeat(BaseT::ampnom);
        }
    }

    static void SetTorquePercent(s32fp torquePercent)
    {
        static int32_t heatCurRamped = 0;
        s32fp          brkrampstr = Param::Get(Param::brkrampstr);
        int            direction = Param::GetInt(Param::dir);
        int            heatCur = Param::GetInt(Param::heatcur);

        heatCur = MIN(400, heatCur);

        if (BaseT::frq < brkrampstr && torquePercent < 0)
        {
            torquePercent =
                FP_MUL(FP_DIV(BaseT::frq, brkrampstr), torquePercent);
        }

        if (torquePercent < 0)
        {
            direction = Encoder::GetRotorDirection();
        }

        int32_t is = FP_TOINT(
            FP_MUL(Param::Get(Param::throtcur), direction * torquePercent));
        int32_t id, iq;

        if (heatCur > 0 && torquePercent < FP_FROMINT(30))
        {
            int speed = Param::GetInt(Param::speed);

            if (speed == 0 && torquePercent <= 0)
            {
                iq = 0;
                heatCurRamped = RAMPUP(heatCurRamped, heatCur, 10);
                id = heatCurRamped;
            }
            /*else if (torquePercent > 0)
            {
               id = FP_TOINT((-heatCur * torquePercent) / 30);
            }*/
            else
            {
                FOC::Mtpa(is, id, iq);
                heatCurRamped = 0;
            }
        }
        else
        {
            FOC::Mtpa(is, id, iq);
            heatCurRamped = 0;
        }

        qController.SetRef(FP_FROMINT(iq));
        fwController.SetRef(FP_FROMINT(iq));
        idref = FP_FROMINT(id);
    }

private:
    static void PwmInit()
    {
        int32_t maxVd = FOC::GetMaximumModulationIndex() - 2000;
        BaseT::pwmfrq = PwmDriverT::TimerSetup(
            Param::GetInt(Param::deadtime),
            Param::GetInt(Param::pwmpol),
            BaseT::pwmdigits);
        BaseT::slipIncr = BaseT::FrqToAngle(BaseT::fslip);
        Encoder::SetPwmFrequency(BaseT::pwmfrq);
        initwait = BaseT::pwmfrq / 2; // 0.5s
        idref = 0;
        qController.ResetIntegrator();
        qController.SetCallingFrequency(BaseT::pwmfrq);
        qController.SetMinMaxY(-maxVd, maxVd);
        dController.ResetIntegrator();
        dController.SetCallingFrequency(BaseT::pwmfrq);
        dController.SetMinMaxY(-maxVd, maxVd);
        fwController.ResetIntegrator();
        fwController.SetCallingFrequency(BaseT::pwmfrq);
        fwController.SetMinMaxY(
            -50 * Param::Get(Param::throtcur),
            0); // allow 50% of max current for extra field weakening

        PwmDriverT::DriverInit();

        if (BaseT::opmode == Modes::ACHEAT)
            PwmDriverT::AcHeatTimerSetup();
    }

    static s32fp ProcessCurrents(s32fp& id, s32fp& iq)
    {

        if (initwait > 0)
        {
            initwait--;
        }

        s32fp il1 = BaseT::GetCurrent(
            AnaIn::il1, BaseT::ilofs[0], Param::Get(Param::il1gain));
        s32fp il2 = BaseT::GetCurrent(
            AnaIn::il2, BaseT::ilofs[1], Param::Get(Param::il2gain));

        if ((Param::GetInt(Param::pinswap) & SWAP_CURRENTS) > 0)
            FOC::ParkClarke(il2, il1, BaseT::angle);
        else
            FOC::ParkClarke(il1, il2, BaseT::angle);
        id = FOC::id;
        iq = FOC::iq;

        Param::SetFlt(Param::id, FOC::id);
        Param::SetFlt(Param::iq, FOC::iq);
        Param::SetFlt(Param::il1, il1);
        Param::SetFlt(Param::il2, il2);

        return 0;
    }

    static void CalcNextAngleSync(int dir)
    {
        if (Encoder::SeenNorthSignal())
        {
            uint16_t syncOfs = Param::GetInt(Param::syncofs);
            uint16_t rotorAngle = Encoder::GetRotorAngle();

            // Compensate rotor movement that happened between sampling and
            // processing
            syncOfs += FP_TOINT(dir * BaseT::frq * 10);

            BaseT::angle = BaseT::polePairRatio * rotorAngle + syncOfs;
            BaseT::frq = BaseT::polePairRatio * Encoder::GetRotorFrequency();
        }
        else
        {
            BaseT::frq = BaseT::fslip;
            BaseT::angle += dir * BaseT::FrqToAngle(BaseT::fslip);
        }
    }

    static void RunOffsetCalibration()
    {
        static int il1Avg = 0, il2Avg = 0, samples = 0;
        const int  offsetSamples = 512;

        if (samples < offsetSamples)
        {
            il1Avg += AnaIn::il1.Get();
            il2Avg += AnaIn::il2.Get();
        }
        else
        {
            BaseT::SetCurrentOffset(
                il1Avg / offsetSamples, il2Avg / offsetSamples);
            il1Avg = il2Avg = 0;
            samples = 0;
        }

        samples++;
    }

private:
    static int         initwait;
    static int         fwBaseGain;
    static s32fp       idref;
    static int         curki;
    static const s32fp dcCurFac = FP_FROMFLT(
        0.81649658092772603273 * 1.05); // sqrt(2/3)*1.05 (inverter losses)
    static PiController qController;
    static PiController dController;
    static PiController fwController;
};

// Instances of each member variable
template <typename PwmDriverT>
int FocPwmGeneration<PwmDriverT>::initwait;

template <typename PwmDriverT>
int FocPwmGeneration<PwmDriverT>::fwBaseGain;

template <typename PwmDriverT>
s32fp FocPwmGeneration<PwmDriverT>::idref;

template <typename PwmDriverT>
int FocPwmGeneration<PwmDriverT>::curki;

template <typename PwmDriverT>
PiController FocPwmGeneration<PwmDriverT>::qController;

template <typename PwmDriverT>
PiController FocPwmGeneration<PwmDriverT>::dController;

template <typename PwmDriverT>
PiController FocPwmGeneration<PwmDriverT>::fwController;

#endif // FOCPWMGENERATION_H
