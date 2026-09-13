#pragma once
/* \copyright 2023-2026 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ESP32 Neopixel Driver
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
// union PixelColor can be used as one 32-bit value, or by addressing the four individual r, g, b, w bytes
struct StructPixelColor {
    uint8_t b; // little-endian for whole ESP32xx family, do NOT change order of these bytes, otherwise the color will be wrong!
    uint8_t g;
    uint8_t r;
    uint8_t w; // white, only used for SK6812B (RGBW) Neopixels, ignored for WS2812B (RGB) Neopixels
};

typedef union UnionPixelColor {
    struct StructPixelColor color; // set as BGR(W), eg: PixelColor dimRed = {.color = {0, 0, 0x28, 0}};
    uint32_t value;                // set as 0x(W)RGB, eg: PixelColor dimRed = {.value = 0x280000};
} PixelColor;

// Primary colors at full brigntness
inline constexpr PixelColor neopixelBlack = {.color = {.b = 0, .g = 0, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelWhite_RGB = {.color = {.b = 0xff, .g = 0xff, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelWhite_RGBW = {.color = {.b = 0, .g = 0, .r = 0, .w = 0xff}};
inline constexpr PixelColor neopixelRed = {.color = {.b = 0, .g = 0, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelGreen = {.color = {.b = 0, .g = 0xff, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelBlue = {.color = {.b = 0xff, .g = 0, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelCyan = {.color = {.b = 0xff, .g = 0xff, .r = 0, .w = 0}};
inline constexpr PixelColor neopixelMagenta = {.color = {.b = 0xff, .g = 0, .r = 0xff, .w = 0}};
inline constexpr PixelColor neopixelYellow = {.color = {.b = 0, .g = 0xff, .r = 0xff, .w = 0}};

typedef void (*pfnSetPixel)(void *c, uint32_t index, const PixelColor pixel);
typedef void (*pfnSetAllSameColor)(void *c, const PixelColor pixel);

struct neoPixelStatistics {
    uint32_t chunksSent; //@@@TODO: not only statistical, but also required to track actual DMA progress -> move to tNpContext
    uint32_t maxChunksSent;
    uint32_t sentBytestAtMaxChunksSent;
};

typedef struct sNpContext {
    SemaphoreHandle_t dataSent; // all data has been sent to the Neopixels, but not fully ready for new data yet
    i2s_chan_handle_t i2s;
    uint32_t nrPixels;
    uint32_t bytesSent;              // to keep track of DMA progress
    struct neoPixelStatistics stats; // statistics for debugging and performance monitoring only

    uint8_t *buffer;
    uint32_t bufferSize;
    uint32_t totalNrChunks; // total number of DMA chunks (descriptors) for the complete Neopixel data transmission (incl data flush)
    pfnSetPixel setpixel;
    pfnSetAllSameColor setAllSameColor;
    uint32_t bitrate;
} tNpContext;

typedef void *tNeopixelContext;

typedef enum {
    NEOPIXEL_MODE_WS2812B, /* RGB */
    NEOPIXEL_MODE_SK6812B, /* RGBW */
} eNeopixelMode;

class NeopixelDriver {
  private:
    size_t nrPixels;
    gpio_num_t dout_pin;
    size_t bytesPerPixel;
    uint8_t *buffer;
    uint32_t bufferSize;

  public:
    NeopixelDriver(size_t nrPixels, gpio_num_t dout_pin, eNeopixelMode mode);
    ~NeopixelDriver();
    bool begin();
    void setPixel(const size_t index, const PixelColor color);
    void _fillPixelRange(size_t startIndex, size_t nrPixelsInRange, const PixelColor color);
    void setAllPixels(const PixelColor color);
    void setPixelRange(size_t startIndex, size_t endIndex, const PixelColor color);
    bool show();
};

/*! \brief Create a neopixel context
 * \param nrPixels Number of pixels
 * \param dout_pin Physical pin to send neopixel data (e.g. GPIO_NUM_27)
 * \param mode Neopixel mode (one of NEOPIXEL_MODE_*)
 * \returns Pointer to neopixel context, used as the first parameter
 *          to subsequent neopixel function calls
 */
tNeopixelContext neopixel_Initialize(uint32_t nrPixels, gpio_num_t dout_pin, eNeopixelMode mode);

void neopixel_SetColor(tNeopixelContext ctx, uint32_t index, const PixelColor pixel);
bool neopixel_Show(tNeopixelContext ctx);

void setAllSameColor(tNeopixelContext ctx, const PixelColor color);

/*! \brief Destroy an existing neopixel context and all associated resources
 *  \param ctx Neopixel context received from successful neopixel_Init calls
 */
void neopixel_Deinit(tNeopixelContext ctx);

#ifdef __cplusplus
}
#endif
