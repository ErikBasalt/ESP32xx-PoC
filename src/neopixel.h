#pragma once
/* \copyright 2023-2026 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ESP32 Neopixel Driver
 */

#include <freertos/FreeRTOS.h> //@@@TODO: do we need all these here, or can move partly to .cpp ?
#include <freertos/semphr.h>
#include <driver/i2s_std.h>
#include <driver/i2s_common.h>

#include <stdint.h>
#include <stdbool.h>

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif
// union ErikPixel can be used as one 32-bit value, or by addressing the four individual r, g, b, w bytes
struct StructErikPixel {
    uint8_t b; // little-endian for whole ESP32xx family, do NOT change order of these bytes, otherwise the color will be wrong!
    uint8_t g;
    uint8_t r;
    uint8_t w;
};

typedef union UnionErikPixel {
    struct StructErikPixel bytes; // set as BGR(W), eg: ErikPixel softRed = {.bytes = {0, 0, 0x28, 0}};
    uint32_t value;               // set as 0x(W)RGB, eg: ErikPixel  softRed = {.value = 0x280000};
} ErikPixel;

typedef void (*pfnSetPixel)(void *c, uint32_t index, const ErikPixel pixel);

typedef struct sNpContext {
    portMUX_TYPE lock;
    SemaphoreHandle_t newData;
    SemaphoreHandle_t dataSent;
    SemaphoreHandle_t erik_isReady;
    i2s_chan_handle_t i2s;
    uint32_t pixels;
    bool terminate;
    uint32_t bytesSent;
    uint32_t erikChunksSent;
    uint32_t erikMaxChunksSent;
    uint32_t erikOverflowCount;
    uint32_t erikTaskOverrunCount;

    uint8_t *buffer;
    uint32_t bufferSize;
    pfnSetPixel setpixel;
    uint32_t bitrate;
} tNpContext;

typedef void *tNeopixelContext;

typedef enum {
    NEOPIXEL_MODE_WS2812B, /* RGB */
    NEOPIXEL_MODE_SK6812B, /* RGBW */
} eNeopixelMode;

/*! \brief Create a neopixel context
 * \param pixels Number of pixels
 * \param dout_pin Physical pin to send neopixel data (e.g. GPIO_NUM_27)
 * \param mode Neopixel mode (one of NEOPIXEL_MODE_*)
 * \returns Pointer to neopixel context, used as the first parameter
 *          to subsequent neopixel function calls
 */
tNeopixelContext neopixel_Initialize(uint32_t pixels, gpio_num_t dout_pin, eNeopixelMode mode);

// Erik
void erik_SetNeopixel(tNeopixelContext ctx, uint32_t index, const ErikPixel pixel);

bool erik_ShowNeopixels(tNeopixelContext ctx);
bool erik_ShowNeopixels_noWait(tNeopixelContext ctx);
void erik_ShowRing_noTask(tNeopixelContext ctx);

/*! \brief Get minimum number of ticks between neopixel_SetPixel calls
 *  \param ctx Neopixel context received from successful neopixel_Init calls
 *  \returns Minimum number of ticks to wait between neopixel_SetPixel calls
 *           to ensure each neopixel_SetPixel call is displayed. If the time
 *           delta between neopixel_SetPixel calls is less than this, some
 *           neopixel_SetPixel data will simply not be displayed; no other
 *           ill-effects will result.
 */
uint32_t neopixel_GetRefreshRate(tNeopixelContext ctx);

/*! \brief Destroy an existing neopixel context and all associated resources
 *  \param ctx Neopixel context received from successful neopixel_Init calls
 */
void neopixel_Deinit(tNeopixelContext ctx);

#ifdef __cplusplus
}
#endif
