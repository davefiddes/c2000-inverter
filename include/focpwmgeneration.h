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
#include "my_fp.h"
#include "picontroller.h"
#include "pwmgenerationbase.h"

template <typename CurrentT, typename EncoderT, typename PwmDriverT>
class FocPwmGeneration : public PwmGenerationBase<
                             FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>,
                             CurrentT,
                             PwmDriverT>
{
    typedef PwmGenerationBase<
        FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>,
        CurrentT,
        PwmDriverT>
        BaseT;

    // We need to allow PwmGenerationBase to call PwmInit
    friend class PwmGenerationBase<
        FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>,
        CurrentT,
        PwmDriverT>;

public:
    static void SetControllerGains(int32_t kp, int32_t ki, int32_t fwkp)
    {
        qController.SetGains(kp, ki);
        dController.SetGains(kp, ki);
        fwBaseGain = fwkp;
        curki = ki;
    }

    static void Run()
    {
        if (BaseT::opmode == MANUAL || BaseT::opmode == RUN)
        {
            static s32fp frqFiltered = 0, idcFiltered = 0;
            int32_t      dir = Param::GetInt(Param::dir);
            int32_t      kifrqgain = Param::GetInt(Param::curkifrqgain);
            s32fp        id, iq;

            EncoderT::UpdateRotorAngle(dir);

            CalcNextAngleSync(dir);

            frqFiltered = IIRFILTER(frqFiltered, BaseT::frq, 8);
            int32_t moddedKi = curki + kifrqgain * FP_TOINT(frqFiltered);

            qController.SetIntegralGain(moddedKi);
            dController.SetIntegralGain(moddedKi);
            fwController.SetProportionalGain(fwBaseGain * dir);

            ProcessCurrents(id, iq);

            if (BaseT::opmode == RUN && initwait == 0)
            {
                s32fp fwIdRef = idref <= 0 ? fwController.Run(iq) : 0;
                dController.SetRef(idref + fwIdRef);
            }
            else if (BaseT::opmode == MANUAL)
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
        else if (BaseT::opmode == BOOST || BaseT::opmode == BUCK)
        {
            initwait = 0;
            BaseT::Charge();
        }
        else if (BaseT::opmode == ACHEAT)
        {
            initwait = 0;
            PwmDriverT::AcHeat(BaseT::ampnom);
        }
    }

    static void SetTorquePercent(s32fp torquePercent)
    {
        static int32_t heatCurRamped = 0;
        s32fp          brkrampstr = Param::Get(Param::brkrampstr);
        int32_t        direction = Param::GetInt(Param::dir);
        int32_t        heatCur = Param::GetInt(Param::heatcur);

        heatCur = MIN(400, heatCur);

        if (BaseT::frq < brkrampstr && torquePercent < 0)
        {
            torquePercent =
                FP_MUL(FP_DIV(BaseT::frq, brkrampstr), torquePercent);
        }

        if (torquePercent < 0)
        {
            direction = EncoderT::GetRotorDirection();
        }

        int32_t is = FP_TOINT(
            FP_MUL(Param::Get(Param::throtcur), direction * torquePercent));
        int32_t id, iq;

        if (heatCur > 0 && torquePercent < FP_FROMINT(30))
        {
            int32_t speed = Param::GetInt(Param::speed);

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

protected:
    static void PwmInit()
    {
        int32_t maxVd = FOC::GetMaximumModulationIndex() - 2000;
        BaseT::pwmfrq = PwmDriverT::TimerSetup(
            Param::GetInt(Param::deadtime),
            Param::GetInt(Param::pwmpol),
            BaseT::pwmdigits);
        BaseT::slipIncr = BaseT::FrqToAngle(BaseT::fslip);
        EncoderT::SetPwmFrequency(BaseT::pwmfrq);
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

        if (BaseT::opmode == ACHEAT)
            PwmDriverT::AcHeatTimerSetup();
    }

private:
    static s32fp ProcessCurrents(s32fp& id, s32fp& iq)
    {

        if (initwait > 0)
        {
            initwait--;
        }

        s32fp il1 = BaseT::GetCurrent(
            CurrentT::Phase1(), BaseT::ilofs[0], Param::Get(Param::il1gain));
        s32fp il2 = BaseT::GetCurrent(
            CurrentT::Phase2(), BaseT::ilofs[1], Param::Get(Param::il2gain));

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

    static void CalcNextAngleSync(int32_t dir)
    {
        if (EncoderT::SeenNorthSignal())
        {
            uint16_t syncOfs = Param::GetInt(Param::syncofs);
            uint16_t rotorAngle = EncoderT::GetRotorAngle();

            // Compensate rotor movement that happened between sampling and
            // processing
            syncOfs += FP_TOINT(dir * BaseT::frq * 10);

            BaseT::angle = BaseT::polePairRatio * rotorAngle + syncOfs;
            BaseT::frq = BaseT::polePairRatio * EncoderT::GetRotorFrequency();
        }
        else
        {
            BaseT::frq = BaseT::fslip;
            BaseT::angle += dir * BaseT::FrqToAngle(BaseT::fslip);
        }
    }

    static void RunOffsetCalibration()
    {
        static int32_t il1Avg = 0, il2Avg = 0, samples = 0;
        const int32_t  offsetSamples = 512;

        if (samples < offsetSamples)
        {
            il1Avg += CurrentT::Phase1();
            il2Avg += CurrentT::Phase2();
            samples++;
        }
        else
        {
            BaseT::SetCurrentOffset(
                il1Avg / offsetSamples, il2Avg / offsetSamples);
            il1Avg = il2Avg = 0;
            samples = 0;
        }
    }

private:
    static int32_t     initwait;
    static int32_t     fwBaseGain;
    static s32fp       idref;
    static int32_t     curki;
    static const s32fp dcCurFac = FP_FROMFLT(
        0.81649658092772603273 * 1.05); // sqrt(2/3)*1.05 (inverter losses)
    static PiController qController;
    static PiController dController;
    static PiController fwController;
};

// Instances of each member variable
template <typename CurrentT, typename EncoderT, typename PwmDriverT>
int32_t FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>::initwait;

template <typename CurrentT, typename EncoderT, typename PwmDriverT>
int32_t FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>::fwBaseGain;

template <typename CurrentT, typename EncoderT, typename PwmDriverT>
s32fp FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>::idref;

template <typename CurrentT, typename EncoderT, typename PwmDriverT>
int32_t FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>::curki;

template <typename CurrentT, typename EncoderT, typename PwmDriverT>
PiController FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>::qController;

template <typename CurrentT, typename EncoderT, typename PwmDriverT>
PiController FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>::dController;

template <typename CurrentT, typename EncoderT, typename PwmDriverT>
PiController FocPwmGeneration<CurrentT, EncoderT, PwmDriverT>::fwController;

#endif // FOCPWMGENERATION_H
