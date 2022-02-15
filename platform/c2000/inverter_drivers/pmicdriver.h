/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2021 David J. Fiddes <D.J@fiddes.net>
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
#ifndef TESLAM3PMIC_H
#define TESLAM3PMIC_H

#include <stdint.h>

#include "hw/tlf35584_safety_psu.h"

namespace c2000 {
struct Register
{
    uint16_t reg;
    uint16_t value;
};

/**
 * \brief TLF35584 safety power supply and watchdog register set up sequence
 *
 */
static const Register RegisterConfig[6] = {
    { TLF35584_WDCFG1, TLF35584_WDCFG1_WDSLPEN | TLF35584_WDCFG1_FWDETHR(14) },
    { TLF35584_WDCFG0,
      TLF35584_WDCFG0_WWDETHR(14) | TLF35584_WDCFG0_WWDEN |
          TLF35584_WDCFG0_FWDEN | TLF35584_WDCFG0_WWDTSEL |
          TLF35584_WDCFG0_WDCYC_1MS },
    { TLF35584_SYSPCFG1,
      TLF35584_SYSPCFG1_SS2DEL_0MS | TLF35584_SYSPCFG1_ERRREC_1MS },
    { TLF35584_FWDCFG, TLF35584_FWDCFG_WDHBTP_CYCLES(250) },
    { TLF35584_WWDCFG0, TLF35584_WWDCFG0_CW_CYCLES(50) },
    { TLF35584_WWDCFG1, TLF35584_WWDCFG1_OW_CYCLES(100) }
};

static const uint16_t RegisterConfigSize =
    sizeof(RegisterConfig) / sizeof(RegisterConfig[0]);

/**
 * Functional Watchdog response sequence structure
 */
struct WatchdogResponse
{
    uint16_t resp3;
    uint16_t resp2;
    uint16_t resp1;
    uint16_t resp0;
};

/**
 * Canned Functional Watchdog response sequences from Table 26 in Section 15.3
 * functional Watchdog of the TLF35584 datasheet
 */
static const WatchdogResponse WatchdogResponses[0x10] = {
    // clang-format off
    { 0xFF, 0x0F, 0xF0, 0x00 },
    { 0xB0, 0x40, 0xBF, 0x4F },
    { 0xE9, 0x19, 0xE6, 0x16 },
    { 0xA6, 0x56, 0xA9, 0x59 },
    { 0x75, 0x85, 0x7A, 0x8A },
    { 0x3A, 0xCA, 0x35, 0xC5 },
    { 0x63, 0x93, 0x6C, 0x9C },
    { 0x2C, 0xDC, 0x23, 0xD3 },
    { 0xD2, 0x22, 0xDD, 0x2D },
    { 0x9D, 0x6D, 0x92, 0x62 },
    { 0xC4, 0x34, 0xCB, 0x3B },
    { 0x8B, 0x7B, 0x84, 0x74 },
    { 0x58, 0xA8, 0x57, 0xA7 },
    { 0x17, 0xE7, 0x18, 0xE8 },
    { 0x4E, 0xBE, 0x41, 0xB1 },
    { 0x01, 0xF1, 0x0E, 0xFE },
    // clang-format on
};

static const uint16_t WatchdogResponsesSize =
    sizeof(WatchdogResponses) / sizeof(WatchdogResponses[0]);

static const int StateTransitionDelay = 100; // uS

/**
 * \brief Verify that function x didn't fail. Return immediately if it did.
 * Assumes a local variable "result" of type TeslaM3PowerWatchdog::Error
 * Undefined at the end of the header
 */
#ifdef CHECK
#undef CHECK
#endif
#define CHECK(x)                                                               \
    if ((result = x) != OK)                                                    \
    {                                                                          \
        return result;                                                         \
    }

template <typename SpiDriverT>
class TeslaM3PowerWatchdog
{
public:
    enum Error
    {
        OK = 0,
        WriteFail,
        ReadParityFail,
        StateTransitionFail
    };

public:
    /**
     * \brief Set up the power management watchdog
     *
     * \return Error - Error code if initialisation failed otherwise Error::OK
     */
    static Error Init()
    {
        SpiDriverT::Init();
        return SetupPowerManagement();
    }

    /**
     * \brief Combined window and functional strobe during normal run
     *     Alternating functional watchdog between fetching quest and answering
     * it.
     *
     *  Window Watchdog is defined with a 100ms period
     *  Functional Watchdog with 250ms.
     *
     *  Thus a functional watchdog quest only needs to be answered every second
     * WW period.
     */
    static Error Strobe()
    {
        Error result;

        CHECK(TeslaM3PowerWatchdog::StrobeWindowWatchdog());

        if (lastFunctionalWatchdogQuest & ~TLF35584_FWDSTAT0_FWDQUEST_MASK)
        {
            CHECK(TeslaM3PowerWatchdog::FunctionalWatchdogReadQuest());
        }
        else
        {
            CHECK(TeslaM3PowerWatchdog::StrobeFunctionalWatchdog());
        }

        return OK;
    }

private:
    // last quest is kept track of as it is read at a different time than the
    // response
    static uint16_t lastFunctionalWatchdogQuest;

    /**
     * \brief Strobe the window watchdog - this is on a 100ms cycle typically
     *
     * \return Error - Error code if stobe failed otherwise Error::OK
     */
    static Error StrobeWindowWatchdog()
    {
        Error result;

        uint16_t windowStatus;
        CHECK(ReadRegister(TLF35584_WWDSCMD, windowStatus));

        // Invert the window watchdog status and write back in the lowest bit
        windowStatus = (~(windowStatus >> 7)) & TLF35584_WWDSCMD_TRIG;
        CHECK(WriteRegister(TLF35584_WWDSCMD, windowStatus));

        return OK;
    }

    /**
     * \brief Read the quest of functional watchdog
     *   - this is on a 200ms cycle typically
     *   - done together with every other strobing the window watchdog
     *
     * \return Error - Error code if stobe failed otherwise Error::OK
     */
    static Error FunctionalWatchdogReadQuest()
    {

        Error result;

        uint16_t functionalStatus;
        CHECK(ReadRegister(TLF35584_FWDSTAT0, functionalStatus));

        // Determine which response the functional watchdog is expecting from us
        lastFunctionalWatchdogQuest =
            functionalStatus & TLF35584_FWDSTAT0_FWDQUEST_MASK;

        return OK;
    }

    /**
     * \brief Answer the functional watchdog quest - this is on a 200ms cycle
     * typically
     *
     * \return Error - Error code if stobe failed otherwise Error::OK
     */
    static Error StrobeFunctionalWatchdog()
    {

        Error result;

        if (lastFunctionalWatchdogQuest & ~TLF35584_FWDSTAT0_FWDQUEST_MASK)
        {
            // TODO ERROR
        }

        // Determine which response the functional watchdog is expecting from us
        const WatchdogResponse& response = WatchdogResponses
            [TeslaM3PowerWatchdog::lastFunctionalWatchdogQuest];

        CHECK(WriteRegister(TLF35584_FWDRSP, response.resp3));
        CHECK(WriteRegister(TLF35584_FWDRSP, response.resp2));
        CHECK(WriteRegister(TLF35584_FWDRSP, response.resp1));
        CHECK(WriteRegister(TLF35584_FWDRSPSYNC, response.resp0));

        // reset, indicating that we need to fetch a new
        lastFunctionalWatchdogQuest = ~TLF35584_FWDSTAT0_FWDQUEST_MASK;

        return OK;
    }

    /**
     * \brief Combined window and functional strobe during setup
     */
    static Error InitStrobe()
    {
        Error result;

        // indicate that we still need to fetch a new one
        lastFunctionalWatchdogQuest = ~TLF35584_FWDSTAT0_FWDQUEST_MASK;

        CHECK(TeslaM3PowerWatchdog::StrobeWindowWatchdog());
        CHECK(TeslaM3PowerWatchdog::FunctionalWatchdogReadQuest());
        CHECK(TeslaM3PowerWatchdog::StrobeFunctionalWatchdog());

        return OK;
    }

    static Error UnlockRegister()
    {
        Error result;

        CHECK(WriteRegister(TLF35584_PROTCFG, TLF35584_PROTCFG_UNLOCK_KEY1));
        CHECK(WriteRegister(TLF35584_PROTCFG, TLF35584_PROTCFG_UNLOCK_KEY2));
        CHECK(WriteRegister(TLF35584_PROTCFG, TLF35584_PROTCFG_UNLOCK_KEY3));
        CHECK(WriteRegister(TLF35584_PROTCFG, TLF35584_PROTCFG_UNLOCK_KEY4));

        return OK;
    }

    static Error LockRegister()
    {
        Error result;

        CHECK(WriteRegister(TLF35584_PROTCFG, TLF35584_PROTCFG_LOCK_KEY1));
        CHECK(WriteRegister(TLF35584_PROTCFG, TLF35584_PROTCFG_LOCK_KEY2));
        CHECK(WriteRegister(TLF35584_PROTCFG, TLF35584_PROTCFG_LOCK_KEY3));
        CHECK(WriteRegister(TLF35584_PROTCFG, TLF35584_PROTCFG_LOCK_KEY4));

        return OK;
    }

    /**
     * \brief Run through the set up sequence for the power supply and watchdog
     */
    static Error SetupPowerManagement()
    {
        Error result;

        // Write the initial device configuration
        Register reg;
        uint16_t r = 0;

        CHECK(UnlockRegister())
        DEVICE_DELAY_US(8);
        for (r = 0; r < RegisterConfigSize; r++)
        {
            reg = RegisterConfig[r];
            CHECK(WriteRegister(reg.reg, reg.value));
        }
        DEVICE_DELAY_US(8);
        CHECK(LockRegister())

        // Strobe the watchdog so that everything is happy before changing the
        // initial state. This closes the Long Open Window of the Window
        // Watchdog. We need to be careful not to strobe for 50ms until the
        // Closed Window period finishes
        CHECK(InitStrobe());

        // resetting registers
        CHECK(WriteRegister(TLF35584_INITERR, 0xFF));
        CHECK(WriteRegister(TLF35584_OTFAIL, 0xFF));
        CHECK(WriteRegister(TLF35584_OTWRNSF, 0xFF));
        CHECK(WriteRegister(TLF35584_MONSF0, 0xFF));
        CHECK(WriteRegister(TLF35584_MONSF1, 0xFF));
        CHECK(WriteRegister(TLF35584_MONSF2, 0xFF));
        CHECK(WriteRegister(TLF35584_MONSF3, 0xFF));
        CHECK(WriteRegister(TLF35584_SPISF, 0xFF));
        CHECK(WriteRegister(TLF35584_WKSF, 0xFF));

        // Move the device into the NORMAL state
        const uint16_t NewState =
            TLF35584_DEVCTRL_TRK2EN | TLF35584_DEVCTRL_TRK1EN |
            TLF35584_DEVCTRL_COMEN | TLF35584_DEVCTRL_VREFEN |
            TLF35584_DEVCTRL_STATEREQ_NORMAL;
        // DEVCTRLN reg write MUST come directly after DEVCTRL
        // and it MUST write inverted value of DEVCTRL
        CHECK(WriteRegister(TLF35584_DEVCTRL, NewState));
        CHECK(WriteRegister(TLF35584_DEVCTRLN, ~NewState));

        return OK;
    }

    /**
     * \brief Evaluate the parity
     *
     * Algorithm from
     * https://graphics.stanford.edu/~seander/bithacks.html#ParityParallel
     *
     * \param value Value to checked for parity
     *
     * \return True if there are an odd number of bits set
     */
    static bool HasOddParity(uint16_t value)
    {
        value ^= value >> 8;
        value ^= value >> 4;
        value ^= value >> 2;
        value ^= value >> 1;
        return value & 1;
    }

    /**
     * \brief Write a specific register
     *
     * \param reg Register to write to
     * \param value Value to be written
     *
     * \return Communication failure
     */
    static Error WriteRegister(uint16_t reg, uint16_t value)
    {
        uint16_t out = TLF35584_SPI_REG_WRITE | TLF35584_SPI_CMD(reg) |
                       TLF35584_SPI_DATA(value);

        if (HasOddParity(out))
        {
            out = out | TLF35584_SPI_PARITY_MASK;
        }

        // A working TLF35584 will echo back any write commands so we can use
        // this to verify communication somewhat
        uint16_t res = SpiDriverT::TransferData(out);

        // only verify reading the written value echoed if 
        // there is a device with echo - e.g. handy for launchxl 
        if (SpiDriverT::ReadDataAfterWrite() && (res != out))
        {
            return WriteFail;
        }
        return OK;
    }

    /**
     * \brief Read back a register
     *
     * \param reg Register we want to read back
     *
     * \return Register value
     */

    static Error ReadRegister(uint16_t reg, uint16_t& value)
    {
        uint16_t request = TLF35584_SPI_CMD(reg);

        if (HasOddParity(request))
        {
            request = request | TLF35584_SPI_PARITY_MASK;
        }

        uint16_t response = 0;

        response = SpiDriverT::TransferData(request);

        // only verify result if there is one - testing only
        if (SpiDriverT::ReadDataAfterWrite())
        {

            bool parityOk = (response & TLF35584_SPI_PARITY_MASK) ==
                            HasOddParity(response >> 1);

            if (!parityOk)
            {
                return ReadParityFail;
            }
        }
        value = (response & TLF35584_SPI_DATA_MASK) >> TLF35584_SPI_DATA_SHIFT;

        return OK;
    }
};

// quests are &0x000F, set something illegal
template <typename SpiDriverT>
uint16_t TeslaM3PowerWatchdog<SpiDriverT>::lastFunctionalWatchdogQuest =
    ~TLF35584_FWDSTAT0_FWDQUEST_MASK;

#undef CHECK

} // namespace c2000

#endif // TESLAM3PMIC_H
