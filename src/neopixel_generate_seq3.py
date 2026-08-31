OUTPUT_FILE = "neopixel_seq3_color_map.txt"
"""
Generate the color map for Neopixels using 3 transmit bits per color bit,
in short the "seq3" protocol.

Copyright (c) 2026 Erik Basalt. All rights reserved.
This file is released under the MIT License. See the LICENSE file for details.
"""
def convert_3seq(byteValue):
    """
    Convert each of the 8 bits in byteValue to 3 bits:
        0 -> 100
        1 -> 110

    Returns three 8-bit values.
    """
    binary = f"{byteValue:08b}"

    # Replace each bit with its 3-bit encoding
    encoded = "".join("100" if bit == "0" else "110" for bit in binary)

    # Split 24-bit encoded value into three 8-bit values
    col1 = int(encoded[0:8], 2)
    col2 = int(encoded[8:16], 2)
    col3 = int(encoded[16:24], 2)

    return col1, col2, col3


with open(OUTPUT_FILE, "w") as file:
    for index in range(256):
        col1, col2, col3 = convert_3seq(index)

        file.write(
            f"{{0x{col1:02x}, 0x{col2:02x}, 0x{col3:02x}}}, // 0x{index:02x}={index}\n"
        )

print(f"Created {OUTPUT_FILE}")
""""
The generated file will contain:

{0x92, 0x49, 0x24}, // 0x00=0
{0x92, 0x49, 0x26}, // 0x01=1
...
{0x9b, 0x6d, 0xb6}, // 0x7f=127
{0xd2, 0x49, 0x24}, // 0x80=128
...
{0xdb, 0x6d, 0xb4}, // 0xfe=254
{0xdb, 0x6d, 0xb6}, // 0xff=255
"""
