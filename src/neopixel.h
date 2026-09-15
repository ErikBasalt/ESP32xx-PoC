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

// Enable or disable output at every write to the Neopixels for better scope triggering (using Channel 2)
#define NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE 1

#ifdef __cplusplus
extern "C" {
#endif

// Union PixelColor can be used as one 32-bit value, or by addressing the four individual r, g, b, w bytes
typedef union alignas(uint32_t) UnionPixelColor {
    uint32_t value; // set as 0x(W)RGB, eg: PixelColor dimRed = {.value = 0x280000};

    //@@@TODO: why not simply use RGB(W) or (W)RGB order?
    struct StructPixelColor { // set as BGR(W), eg: PixelColor dimmedRed = {.color = {.b=0, .g=0, .r=0x10, .w=0}};
        uint8_t b;            // little-endian for whole ESP32xx family, do NOT change order of these bytes, otherwise the color will be wrong!
        uint8_t g;
        uint8_t r;
        uint8_t w; // white, only used for SK6812B (RGBW) Neopixels, ignored for WS2812B (RGB) Neopixels
    } color;
} PixelColor;

// Primary colors at full brigntness
inline constexpr PixelColor neopixelBlack = {.value = 0x00000000};
inline constexpr PixelColor neopixelWhite_RGB = {.color = {.b = 0xff, .g = 0xff, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelWhite_RGBW = {.color = {.b = 0, .g = 0, .r = 0, .w = 0xff}};
inline constexpr PixelColor neopixelRed = {.color = {.b = 0, .g = 0, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelGreen = {.color = {.b = 0, .g = 0xff, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelBlue = {.color = {.b = 0xff, .g = 0, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelCyan = {.color = {.b = 0xff, .g = 0xff, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelMagenta = {.color = {.b = 0xff, .g = 0, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelYellow = {.color = {.b = 0, .g = 0xff, .r = 0xff, .w = 0}};

typedef enum {
    NEOPIXEL_MODE_WS2812B, // RGB
    NEOPIXEL_MODE_SK6812B, // RGBW
} eNeopixelMode;

class NeopixelDriver {
  private:
    i2s_chan_handle_t i2s; // the I2S channel handle in use (ESP32 and ESP32-S2 have 2 channels)

    uint8_t *buffer = nullptr; // data buffer to be sent to the Neopixels
    size_t bufferSize;         // [bytes]

    size_t nrPixels;      // number of Neopixels to drive
    size_t bytesPerPixel; // number of bytes per pixel based on the Neopixel type (eg: 3 for WS2812B, 4 for SK6812B)

    SemaphoreHandle_t allSentSemaphore; // all chunks have been sent to the Neopixels
    int totalNrChunks;                  // total number of DMA chunks (descriptors) for the complete Neopixel data transmission (incl data flush)
    int sentNrChunks;                   // actual number of chunks (being) sent, used for tracking the transmit progress

    static bool onSentCallback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *classContext); // in cpp

    void _setPixel_ws2812b(const size_t index, const PixelColor color);                     // in cpp
    void _setPixel_sk6812b(const size_t index, const PixelColor color);                     // in cpp
    void (NeopixelDriver::*_selected_setPixel)(const size_t index, const PixelColor color); // set by begin() to one _setPixel_<name> function based on the Neopixel type

    void _fillPixelRange(size_t startIndex, size_t nrPixelsInRange, const PixelColor color) {
        setPixel(startIndex, color);
        if (nrPixelsInRange > 1) {
            setPixel(startIndex + 1, color); // use setPixel for first two pixels, to avoid issue with odd number of bytesPerPixel and little-endian byte order

            size_t copiedBytes = bytesPerPixel * 2;
            size_t restBytes = (nrPixelsInRange * bytesPerPixel) - copiedBytes;
            auto startBufferRange = &buffer[startIndex * bytesPerPixel];

            while (restBytes >= copiedBytes) {
                memcpy(&startBufferRange[copiedBytes], startBufferRange, copiedBytes);
                restBytes -= copiedBytes;
                copiedBytes *= 2;
            }

            if (restBytes > 0) {
                memcpy(&startBufferRange[copiedBytes], startBufferRange, restBytes);
            }
        } // else: just one (1) Neopixel in the range
    }

  public:
    struct NeopixelStatistics {
        int64_t maxSendMicros;
        uint32_t maxNrChunksSent;
        //@@@TODO: add some error counters (e.g., for DMA transfer failures)
    } stats;

    NeopixelDriver(void) {} // empty, use begin() to initialize the driver

    ~NeopixelDriver(void) {
        //@@@TODO: add delay?
        i2s_del_channel(i2s);
        if (buffer != nullptr) {
#if (200 == 200)
            free(buffer);
#else
            heap_caps_free(buffer);
#endif
        }
    }

    bool begin(const size_t nrPixels, const gpio_num_t dataPin, const eNeopixelMode mode); // in cpp
    bool show(void);                                                                       // in cpp

    void setPixel(const size_t index, const PixelColor color) {
        if (index >= nrPixels) {
            return;
        }
        (this->*_selected_setPixel)(index, color); // call the selected Neopixel type specific function
    }

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

#ifdef __cplusplus
}
#endif
