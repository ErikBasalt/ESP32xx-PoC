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
#include <driver/i2s_std.h>
#include <driver/i2s_common.h>

// Enable or disable using a task to wait for I2S transmission completion
#define ENABLE_I2S_TASK_VERSION 0

#if (ENABLE_I2S_TASK_VERSION)
#include "neopixel_i2s.h"
#else
// Enable or disable output at every write to the Neopixels, for scope triggering on the Enable signal
//@@@TODO: remove, enable/disable should be done outside of this driver
#define NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE 1
#endif

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

class NeopixelTransmitControl; // forward declaration, to use as friend class

template <PixelType Mode>
class NeopixelDriver {
  private:
    // I2S
    i2s_chan_handle_t i2s; // the I2S channel handle in use (ESP32 and ESP32-S2 have 2 channels) @@@TODO: move to NeopixelTransmitControl ?

    // Neopixel config
    size_t txBytesPerColor; // number of bytes to be sent per R/G/B/(W) color component, depends on seq3/seq4 timing
    size_t txBytesPerPixel; // number of bytes per Neopixel (all colors)

    // Data size
    size_t nrPixels = 0;       // number of Neopixels to drive
    uint8_t *buffer = nullptr; // data buffer to be sent to the Neopixels
    size_t bufferSize = 0;     // [bytes]

    // Transmission tracking
#if (ENABLE_I2S_TASK_VERSION)
    NeopixelTransmitControl txControl; // the class controlling the I2S transmissions
#else
    SemaphoreHandle_t allSentSemaphore; // all chunks have been sent to the Neopixels
    int totalNrChunks;                  // total number of DMA chunks (descriptors) for the complete Neopixel data transmission (incl data flush)
    int sentNrChunks;                   // actual number of chunks (being) sent, used for tracking the transmit progress

    static IRAM_ATTR bool onSentCallback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *classContext); // in cpp
#endif

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

#if (ENABLE_I2S_TASK_VERSION)
    friend class NeopixelTransmitControl; // allow this other class to access private members here
#endif

  public:
    // Global brightness (min=0...max=255)
    uint8_t brightness = 255;

#if (ENABLE_I2S_TASK_VERSION)
    struct NeopixelTransmitControl::NeopixelStatistics *txControlStats = &txControl.stats;
#else
    struct NeopixelStatistics {
        int64_t maxSendMicros;
        uint32_t maxNrChunksSent;
        //@@@TODO: add some error counters (e.g., for DMA transfer failures)
    } stats = {};
#endif
    NeopixelDriver(void) {} // empty, use begin() to initialize the driver

    ~NeopixelDriver(void) {
        //@@@TODO: add delay?
#if (ENABLE_I2S_TASK_VERSION)
        txControl.deinit();
#else
        i2s_del_channel(i2s);
#endif
        if (buffer != nullptr) {
            free(buffer);
        }
    }

    // Basic functions
    bool begin(const size_t nrPixels, const gpio_num_t dataPin); // in cpp
    void setPixel(const size_t index, const PixelColor color);   // in cpp
    bool show(void);                                             // in cpp

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
