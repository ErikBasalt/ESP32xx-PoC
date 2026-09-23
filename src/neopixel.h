#pragma once
/*
***************************************************************************************************
    Header file for ESP32xx Neopixel Driver

    Copyright (c) 2026 Erik Basalt
    Released under the MIT License, see the LICENSE file for details.
***************************************************************************************************
*/
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "neopixel_i2s.h" // use the I2S implementation for Neopixel data transmission

#ifdef __cplusplus
/*
===================================================================================================
    Union PixelColor

    Can be used as one 32-bit value,
    or by addressing the four individual r, g, b, w bytes
    (for examples see primary colors below)

    In case of GRB, keep the white component = 0
===================================================================================================
*/
typedef union alignas(uint32_t) UnionPixelColor {
    uint32_t value; // set as 0x(ww)rrggbb

    struct StructPixelColor { // set as .b, .g, .r (, .w) bytes, order CANNOT be changed (big/little-endian issue)
        uint8_t b;
        uint8_t g;
        uint8_t r;
        uint8_t w; // white, only used for RGBW Neopixels with separate white LED, ignored for RGB Neopixels
    } color;
} PixelColor;

// Primary colors at full brightness
inline constexpr PixelColor neopixelBlack = {.value = 0x00000000};
inline constexpr PixelColor neopixelWhite_RGB = {.color = {.b = 0xff, .g = 0xff, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelWhite_RGBW = {.color = {.b = 0, .g = 0, .r = 0, .w = 0xff}};
inline constexpr PixelColor neopixelRed = {.color = {.b = 0, .g = 0, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelGreen = {.color = {.b = 0, .g = 0xff, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelBlue = {.color = {.b = 0xff, .g = 0, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelCyan = {.color = {.b = 0xff, .g = 0xff, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelMagenta = {.color = {.b = 0xff, .g = 0, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelYellow = {.color = {.b = 0, .g = 0xff, .r = 0xff, .w = 0}};

/*
===================================================================================================
    Neopixel communication methods

    NOTE: see also explicit template instantiations in .cpp file
===================================================================================================
*/
enum class PixelType {
    GRB_SEQ3,  // GRB colors, using seq3 encoding (1/3 and 2/3 duty cycle)
    GRBW_SEQ3, // GRBW colors, using seq3 encoding (1/3 and 2/3 duty cycle)
    GRBW_SEQ4  // GRBW colors, using seq4 encoding (1/4 and 2/4 duty cycle)
};

/*
===================================================================================================
    Neopixel Driver class

    Instantiate with a specific PixelType, for example:
    npx = NeopixelDriver<PixelType::GRB_SEQ3>;
===================================================================================================
*/
template <PixelType Mode>
class NeopixelDriver {
  private:
    // Neopixel config
    size_t txBytesPerColor; // number of bytes to be sent per R/G/B/(W) color component, depends on seq3/seq4 timing
    size_t txBytesPerPixel; // number of bytes per Neopixel (all colors)

    // Data size
    size_t nrPixels = 0;       // number of Neopixels to drive
    uint8_t *buffer = nullptr; // data buffer to be sent to the Neopixels
    size_t bufferSize = 0;     // [bytes]

    // Transmission tracking
    NeopixelTransmitControl txControl; // class to control Neopixel data transmission, initialized with the stats pointer in the constructor

    // Private method to fill a range of pixels with the specified color
    void _fillPixelRange(size_t startIndex, size_t nrPixelsInRange, const PixelColor color) {
        // First use setPixel for two (!) pixels,
        // to avoid issue with odd number of bytesPerPixel and little-endian byte order
        setPixel(startIndex, color);
        if (nrPixelsInRange > 1) {
            setPixel(startIndex + 1, color);

            // Next duplicate to fill the rest of the range efficiently
            size_t copiedBytes = txBytesPerPixel * 2;
            size_t restBytes = (nrPixelsInRange * txBytesPerPixel) - copiedBytes;
            auto startBufferRange = &buffer[startIndex * txBytesPerPixel];

            while (restBytes >= copiedBytes) {
                memcpy(&startBufferRange[copiedBytes], startBufferRange, copiedBytes);
                restBytes -= copiedBytes;
                copiedBytes *= 2;
            }

            // Finally copy any remaining bytes that didn't fit into the doubling loop
            if (restBytes > 0) {
                memcpy(&startBufferRange[copiedBytes], startBufferRange, restBytes);
            }
        } // else: just one (1) Neopixel in the range
    }

  public:
    uint8_t brightness = 255;                     // global brightness (min=0...max=255)
    struct NeopixelTransmitStatistics stats = {}; // statistics for Neopixel data transmission

    // (De)Constructors
    NeopixelDriver(void) : txControl(&stats) {} // initialize txControl with pointer to the statistics structure here

    // empty, use begin() to initialize the driver

    ~NeopixelDriver(void) {

        txControl.deinit();
        if (buffer != nullptr) {
            free(buffer);
            buffer = nullptr;
        }
        bufferSize = 0;
        nrPixels = 0;
    }

    // Basic functions
    bool begin(const size_t nrPixels, const gpio_num_t dataPin); // in cpp, will allocate buffer
    void setPixel(const size_t index, const PixelColor color);   // in cpp, will set one pixel in buffer

    void show(void) {
        txControl.startTransmit(buffer, bufferSize);
    }

    // Extra functions
    void setAllPixels(const PixelColor color) {
        _fillPixelRange(0, nrPixels, color);
    }

    void setPixelRange(size_t startIndex, size_t endIndex, const PixelColor color) {
        if (startIndex > endIndex) {
            // Swap the Start and End if they are in the wrong order
            auto tmpIndex = startIndex;
            startIndex = endIndex;
            endIndex = tmpIndex;
        }

        // Protect against invalid Start or End
        if ((startIndex >= nrPixels) || (endIndex >= nrPixels)) {
            return;
        }

        _fillPixelRange(startIndex, (endIndex - startIndex + 1), color);
    }
};
#endif // __cplusplus
