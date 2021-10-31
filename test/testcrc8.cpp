/*
 * This file is part of the stm32_sine project.
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

#include "crc8.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <utility>

//! Test crc8 using examples from Table 12 of the STGAP1AS datasheet
TEST(TestCRC8, crc8Data)
{
    EXPECT_EQ(crc8(0x00, 0xFF), 0xF3);
    EXPECT_EQ(crc8(0xEA, 0xFF), 0x6B);
    EXPECT_EQ(crc8(0xF5, 0xFF), 0x36);
    EXPECT_EQ(crc8(0x2A, 0xFF), 0x25);
}

//! invert the lower 8 bits only
uint16_t invert_byte(uint16_t input)
{
    return ~input & 0xFF;
}

//! Test crc8 using examples from Table 11 of the STGAP1AS datasheet
//! This requires an invert function because of the way the STGAP1AS uses CRC8
TEST(TestCRC8, crc8Command)
{
    // StopConfig
    EXPECT_EQ(invert_byte(crc8(0x3A, 0xFF)), 0xAA);

    uint16_t temp;

    // WriteReg(CFG1, 0x20)
    temp = crc8(0x8C, 0xFF);
    EXPECT_EQ(invert_byte(temp), 0xA1);
    EXPECT_EQ(invert_byte(crc8(0x20, temp)), 0x82);

    // WriteReg(CFG5, 0x06)
    temp = crc8(0x99, 0xFF);
    EXPECT_EQ(invert_byte(temp), 0xCA);
    EXPECT_EQ(invert_byte(crc8(0x06, temp)), 0x66);

    // ResetStatus
    EXPECT_EQ(invert_byte(crc8(0xD0, 0xFF)), 0x32);

    // ReadReg(CFG3)
    EXPECT_EQ(invert_byte(crc8(0xBE, 0xFF)), 0x3F);
}

//! Test crc8 with extraneous bits on the data and CRC initialisation and ensure
//! these are ignored
TEST(TestCRC8, crc8ExtraBits)
{
    EXPECT_EQ(crc8(0xAA00, 0x55FF), 0xF3);
}