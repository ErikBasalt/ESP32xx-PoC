/* \copyright 2023-2026 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ESP32 Neopixel Driver
 */
// #include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_log.h>
// #include <freertos/task.h>

// next inclides are already part of neopixel.h
// #include <driver/i2s_std.h>
// #include <driver/i2s_common.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/semphr.h>
// #include <stdint.h>

#include "neopixel.h"
#include "ws2812b_protocol.h"
#include "sk6812b_protocol.h"

#define TAG "NPIX"
#define I2S_TIMEOUT_TICKS 1000
#define NEOPIXEL_TASK_PRIORITY (configMAX_PRIORITIES - 1)

// Enabling the I2S channel only once (at init) would make sense, but does NOT work properly.
// Seems like the sent data is somehow multiplied by nr of DMA buffers, so with dma_desc_num=6 you get 6 red pixels i/o 1
#define ERIK_ENABLE_I2S_CHANNEL_ONLY_ONCE 0

static void neopixel_task(void *arg);
static bool i2s_tx_queue_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx);

static bool i2s_tx_queue_overflow_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx);

static void setpixel_ws2812b(void *c, uint32_t index, const PixelColor color);
static void setpixel_sk6812b(void *c, uint32_t index, const PixelColor color);

/* -------------------------------------------------------------------------------------------------------------
 * Exported Functions
 */

tNeopixelContext neopixel_Initialize(uint32_t nrPixels, gpio_num_t dout_pin, eNeopixelMode mode) {
    tNpContext *c = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
#if (0 == 1)
    // test with smaller DMA
    chan_cfg.dma_desc_num = 2;
    chan_cfg.dma_frame_num = 128;
#elif (0 == 2)
    chan_cfg.dma_desc_num = 3;
    // chan_cfg.dma_frame_num = 128;
#else
// Just use the default DMA config
#endif
    ESP_LOGI(TAG, "DMA buffers=%d, per buffer I2S frames=%d, total DMA bytes=%d", chan_cfg.dma_desc_num, chan_cfg.dma_frame_num, (2 * chan_cfg.dma_desc_num * chan_cfg.dma_frame_num));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(0), // rate is configured later
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_GPIO_UNUSED,
            .ws = I2S_GPIO_UNUSED,
            .dout = dout_pin,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    i2s_event_callbacks_t callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
        .on_sent = i2s_tx_queue_sent_callback,
        .on_send_q_ovf = NULL, // i2s_tx_queue_overflow_callback makes no sense, will increase anyway whether or not using enable/disable I2S channel
    };

    c = (tNpContext *)malloc(sizeof(*c));
    if (NULL == c) {
        ESP_LOGE(TAG, "Failed to allocate context");
        return NULL;
    }
    memset(c, 0, sizeof(*c));

    c->nrPixels = nrPixels;
    switch (mode) {
    case NEOPIXEL_MODE_WS2812B:
        c->bitrate = WS2812B_BITRATE;
        c->bufferSize = (c->nrPixels * WS2812B_BYTES_PER_PIXEL) + WS2812B_RESET_BYTES;
        c->setpixel = setpixel_ws2812b;
        break;
    case NEOPIXEL_MODE_SK6812B:
        c->bitrate = SK6812B_BITRATE;
        c->bufferSize = (c->nrPixels * SK6812B_BYTES_PER_PIXEL) + SK6812B_RESET_BYTES;
        c->setpixel = setpixel_sk6812b;
        break;
    default:
        ESP_LOGE(TAG, "Invalid mode (%d)", mode);
        free(c);
        return NULL;
    }
    ESP_LOGI(TAG, "nrPixels=%d, bit buffer size=%d bytes, bitrate=%d bps", c->nrPixels, c->bufferSize, c->bitrate);

    std_cfg.clk_cfg.sample_rate_hz = c->bitrate / 16 / 2;
    portMUX_INITIALIZE(&c->lock);
    c->newData = xSemaphoreCreateBinary();
    c->dataSent = xSemaphoreCreateBinary();
    c->isReady = xSemaphoreCreateBinary();
    c->terminate = false;
    c->bytesSent = 0;
    c->stats = {}; // reset all statistics to zero

    c->buffer = (uint8_t *)malloc(c->bufferSize);
    memset(c->buffer, 0, c->bufferSize);                        /* initializes the reset bytes to zero */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &c->i2s, NULL)); /* Tx channel only (no Rx) */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(c->i2s, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(c->i2s, &callbacks, c));
#if (ERIK_ENABLE_I2S_CHANNEL_ONLY_ONCE == 1)
    ESP_LOGI(TAG, "Enabling I2S channel only once at init");
    ESP_ERROR_CHECK(i2s_channel_enable(c->i2s));
#else
    ESP_LOGI(TAG, "Enabling/Disabling I2S channel at each transfer");
#endif
    ESP_LOGI(TAG, "I2S channel id=%d, interrupt priority=%d", chan_cfg.id, chan_cfg.intr_priority);

    xTaskCreate(&neopixel_task, TAG, 1024, (void *)c, NEOPIXEL_TASK_PRIORITY, NULL);

    return (tNeopixelContext)c;
}

void neopixel_Deinit(tNeopixelContext ctx) {
    tNpContext *c = (tNpContext *)ctx;
    if (NULL == c)
        return;

    /* Signal the thread to terminate */
    c->terminate = true;
    xSemaphoreGive(c->newData);

    for (int retries = 0; c->terminate && retries < 100; ++retries)
        vTaskDelay(pdMS_TO_TICKS(1));
    if (c->terminate) {
        ESP_LOGE(TAG, "[%s] Failed waiting for thread to terminate\n", __func__);
    }

    i2s_del_channel(c->i2s);
    free(c->buffer);
    free(c);
}

void neopixel_SetColor(tNeopixelContext ctx, uint32_t index, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;

    if (index < c->nrPixels) {
        c->setpixel(c, index, color);
    }
}

bool neopixel_ShowNoWait(tNeopixelContext ctx) {
    tNpContext *c = (tNpContext *)ctx;
    if (xSemaphoreTake(c->isReady, 0) != pdTRUE) {
        // Still busy sending previous data to the Neopixel ring, so skip this iteration (Neopixels will NOT be updated)
        return (false);
    }
    xSemaphoreGive(c->newData); // signal Task to send the data to the Neopixel ring
    return (true);
}

bool neopixel_Show(tNeopixelContext ctx) {
    tNpContext *c = (tNpContext *)ctx;
    unsigned long startMicros = esp_timer_get_time();
    unsigned long waitMicros = 0;
    static unsigned long maxWaitMicros = 0;

    xSemaphoreTake(c->isReady, portMAX_DELAY); // wait until previous data has been sent to the Neopixel ring

    waitMicros = esp_timer_get_time() - startMicros;
    if (waitMicros > maxWaitMicros) {
        maxWaitMicros = waitMicros;
        ESP_LOGI(TAG, "maxWaitMicros=%lu", maxWaitMicros);
    }
    xSemaphoreGive(c->newData); // signal Task to send the data to the Neopixel ring
    return (true);              // always true, but keep return value to be comptatible with neopixel_ShowNoWait()
}

void erik_ShowRing_noTask(tNeopixelContext ctx) { // Did NOT get it to work so far...

    tNpContext *c = (tNpContext *)ctx;

// Ready to send new data
// Semaphore will be given back once new data has been sent

// Fill buffer
//@@@TODO: only now translate pixels to timing bit buffer
#if (0 == 1)
    static uint8_t erik_buffer[(84 * WS2812B_BYTES_PER_PIXEL) + WS2812B_RESET_BYTES]; //@@@TODO: use PIXEL_COUNT instead of 84
    memcpy(erik_buffer, c->buffer, c->bufferSize);
#else
    // no copy
    uint8_t *erik_buffer = c->buffer;
#endif

    // Send buffer
    size_t bytesLoaded;
    c->bytesSent = 0;
    c->stats.chunksSent = 0;

    i2s_channel_preload_data(c->i2s, erik_buffer, c->bufferSize, &bytesLoaded);
    i2s_channel_enable(c->i2s);
    if (bytesLoaded < c->bufferSize) {
        i2s_channel_write(c->i2s, &erik_buffer[bytesLoaded], c->bufferSize - bytesLoaded,
                          NULL, I2S_TIMEOUT_TICKS);
    } //@@@TODO: else??

    // Do NOT wait until all data has been sent
    // Instead, use polling in loopNeopixelRing() to check if the data has been sent
    // @@@TODO: add noWait param?
    // xSemaphoreTake(c->erikDataSent, portMAX_DELAY);
}

uint32_t neopixel_GetRefreshRate(tNeopixelContext ctx) {
    tNpContext *c = (tNpContext *)ctx;
    return c->bitrate / (c->bufferSize * 8);
}

/* -------------------------------------------------------------------------------------------------------------
 * Helper Functions
 */

static IRAM_ATTR bool i2s_tx_queue_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    // Finished sending one (1) DMA buffer
    tNpContext *c = (tNpContext *)user_ctx;
    c->bytesSent += event->size;
    c->stats.chunksSent++;
    if (c->bytesSent >= c->bufferSize) {
        // Erik added
        if (c->stats.chunksSent > c->stats.maxChunksSent) {
            c->stats.maxChunksSent = c->stats.chunksSent;
        }
        xSemaphoreGive(c->dataSent);
    }
    return false; // no need for RTOS to check immediately for higher priority task
}

static IRAM_ATTR bool i2s_tx_queue_overflow_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    // @@@TODO: these overflows seem to be common?? for now, disabled this callback, but keep it here for future reference
    tNpContext *c = (tNpContext *)user_ctx;
    c->stats.overflowCount++;
    return false; // no need for RTOS to check immediately for higher priority task
}

static void neopixel_task(void *arg) {
    tNpContext *c = (tNpContext *)arg;
    size_t bytesLoaded;
    uint8_t *buffer;

    buffer = (uint8_t *)malloc(c->bufferSize);
    if (NULL == buffer) {
        ESP_LOGE(TAG, "[%s] Failed to allocate buffer", __func__);
        return;
    }

    ESP_LOGD(TAG, "[%s] Started", __func__);

    xSemaphoreGive(c->isReady); // ready for first request

    while (!c->terminate) {
        /* block task, waiting for an update */
        if (xSemaphoreTake(c->newData, portMAX_DELAY) != pdTRUE) {
            // Infinite wait, so should NOT get here
            vTaskDelay(pdMS_TO_TICKS(10)); /* prevent tight loops */
            continue;
        }
        if (c->terminate)
            continue;

        /* Make a local copy of the current pixel buffer to be sent to the hardware */
        taskENTER_CRITICAL(&c->lock);
        memcpy(buffer, c->buffer, c->bufferSize);
        taskEXIT_CRITICAL(&c->lock);

        c->bytesSent = 0;
        c->stats.chunksSent = 0;
#if (ERIK_ENABLE_I2S_CHANNEL_ONLY_ONCE == 1)
        // No preload possible, don't do enable/disable, just write the data to the I2S channel
        bytesLoaded = 0; // nothing preloaded
#else
        i2s_channel_preload_data(c->i2s, buffer, c->bufferSize, &bytesLoaded);
        i2s_channel_enable(c->i2s);
#endif
        if (bytesLoaded < c->bufferSize) {
            i2s_channel_write(c->i2s, &buffer[bytesLoaded], c->bufferSize - bytesLoaded,
                              NULL, I2S_TIMEOUT_TICKS);
        } //@@@TODO: else??
        xSemaphoreTake(c->dataSent, portMAX_DELAY); /* Wait for buffer to be transferred to hardware */
#if (ERIK_ENABLE_I2S_CHANNEL_ONLY_ONCE == 1)
        // No disable after sending
#else
        i2s_channel_disable(c->i2s);
#endif
        xSemaphoreGive(c->isReady); // signal ready for new request
#if (1 == 1)
        // Discard already waiting new request, if any, to prevent actual overruns
        if (xSemaphoreTake(c->newData, 0) == pdTRUE) {
            c->stats.taskOverrunCount++;
        }
#endif
    }
    ESP_LOGD(TAG, "[%s] Finished", __func__);

    free(buffer);
    c->terminate = false;
    vTaskDelete(NULL); /* Destroy context */
}

static void setpixel_ws2812b(void *ctx, uint32_t index, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;
    uint8_t *buffer = c->buffer;
    uint32_t offset = index * WS2812B_BYTES_PER_PIXEL;

    const uint8_t *sequence = ws2812b_color_map[color.bytes.g];
    for (int i = 0; i < WS2812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = ws2812b_color_map[color.bytes.r];
        if (i == 6)
            sequence = ws2812b_color_map[color.bytes.b];
        buffer[offset ^ 1] = sequence[i % WS2812B_BYTES_PER_COLOR]; // fill buffer in 16-bit little-endian format
    }
}

static void setpixel_sk6812b(void *ctx, uint32_t index, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;
    uint8_t *buffer = c->buffer;
    uint32_t offset = index * SK6812B_BYTES_PER_PIXEL;

    const uint8_t *sequence = sk6812b_color_map[color.bytes.g];
    for (int i = 0; i < SK6812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = sk6812b_color_map[color.bytes.r];
        if (i == 6)
            sequence = sk6812b_color_map[color.bytes.b];
        if (i == 9)
            sequence = sk6812b_color_map[color.bytes.w];
        buffer[offset ^ 1] = sequence[i % SK6812B_BYTES_PER_COLOR]; // fill buffer in 16-bit little-endian format
    }
}
