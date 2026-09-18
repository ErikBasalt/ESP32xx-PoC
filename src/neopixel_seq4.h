#pragma once
/*
*****************************************************************************************
    Bitstream sequence for Neopixels that use 4 transmit bits ("seq4") per color bit.

    Copyright (c) 2026 Erik Basalt. All rights reserved.
    This file is released under the MIT License. See the LICENSE file for details.
*****************************************************************************************
*/
#define NEOPIXEL_SEQ4_BYTES_PER_COLOR (4)

static constexpr uint8_t encode_seq4_nibble[16][2] = {
    {0x88, 0x88}, // 0000
    {0x88, 0x8c}, // 0001
    {0x8c, 0x88}, // 0010
    {0x8c, 0x8c}, // 0011
    {0x88, 0xc8}, // 0100
    {0x88, 0xcc}, // 0101
    {0x8c, 0xc8}, // 0110
    {0x8c, 0xcc}, // 0111
    {0xc8, 0x88}, // 1000
    {0xc8, 0x8c}, // 1001
    {0xcc, 0x88}, // 1010
    {0xcc, 0x8c}, // 1011
    {0xc8, 0xc8}, // 1100
    {0xc8, 0xcc}, // 1101
    {0xcc, 0xc8}, // 1110
    {0xcc, 0xcc}  // 1111
};
