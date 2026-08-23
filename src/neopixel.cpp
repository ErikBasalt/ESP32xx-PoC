/* \copyright 2023-2026 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ESP32 Neopixel Driver
 */
#include <esp_system.h>
#include <esp_log.h>

#include "neopixel.h"
#include "ws2812b_protocol.h"
#include "sk6812b_protocol.h"

#if (0 == 14)
#include "esp_cache.h"
#endif

#if (50 == 0)
#include "esp_cache.h"
#endif

#define TAG "NPIX"
#define I2S_TIMEOUT_TICKS 1000
#define NEOPIXEL_TASK_PRIORITY (configMAX_PRIORITIES - 1)

// Enabling the I2S channel only once (at init) would make sense, but does NOT work properly.
// Seems like the sent data is somehow multiplied by nr of DMA buffers, so with dma_desc_num=6 you get 6 red pixels i/o 1
#define ENABLE_I2S_CHANNEL_ONLY_ONCE 1
#define ENABLE_I2S_TASK_VERSION 0

#if (ENABLE_I2S_TASK_VERSION == 1)
static void neopixel_task(void *arg);
static bool i2s_tx_queue_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx);
#endif
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
#if (0 == 25)
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
#else
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), // Erik: org
#endif
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

#if (0 == 25)
    // Dwing de driver om géén padding of fractionele frame-afrondingen te gebruiken:
    std_cfg.slot_cfg.ws_width = I2S_DATA_BIT_WIDTH_32BIT;
#endif
#if (49 == 0)
    // Cruciaal voor de ESP32-S3: dwing de fysieke slots en WS-klok naar 16-bit registers
    // helpt niet
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.ws_width = I2S_DATA_BIT_WIDTH_16BIT;
#endif
#if (30 == 30)
    // Dwing de ESP32 hardware om de bytes om te draaien tijdens de DMA-overdracht
// Alleen beschikbaar op C3/C6/S3 architecturen in v5.5:
#if !SOC_I2S_HW_VERSION_1
    std_cfg.slot_cfg.big_endian = true;
#endif
#endif

    i2s_event_callbacks_t callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
#if (ENABLE_I2S_TASK_VERSION == 1)
        .on_sent = i2s_tx_queue_sent_callback,
#else
        .on_sent = NULL,
#endif
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
#if (41 == 0)
        c->bufferSize = (c->nrPixels * WS2812B_BYTES_PER_PIXEL);
        // Do the reset bytes in a separate DMA buffer, separate write call
#elif (43 == 0)
        c->bufferSize = (c->nrPixels * WS2812B_BYTES_PER_PIXEL) + 128;
#elif (44 == 44)
        c->bufferSize = (c->nrPixels * WS2812B_BYTES_PER_PIXEL) + WS2812B_RESET_BYTES;
        //@@@TODO: bepaal minimum gebaseerd op bitrate
#define MIN_DMA_BUFFER_SIZE 896 // minimum DMA buffer size that can be sent without glitches, 768 is too small
        if (c->bufferSize < MIN_DMA_BUFFER_SIZE) {
            ESP_LOGW(TAG, "Buffer size=%d bytes is less than minimum for DMA, incrementing to %d bytes", c->bufferSize, MIN_DMA_BUFFER_SIZE);
            c->bufferSize = MIN_DMA_BUFFER_SIZE;
        }
#else
        // reset bits are needed here, even when using a separate DMA buffer for the reset bits
        c->bufferSize = (c->nrPixels * WS2812B_BYTES_PER_PIXEL) + WS2812B_RESET_BYTES;
#endif
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

#if (7 == 7)
#if (15 == 15)
    c->bufferSize = (c->bufferSize + 63) & ~0x3f; // round up to multiple of 64 bytes
                                                  // Calculate DMA frame_num based on bufferSize, but keep it within reasonable limits (e.g. 128-512 frames per buffer)
#if (45 == 45)
    if ((c->bufferSize % 256) == 0) { // buffer size is a multiple of 256 bytes, which seems to cause glitches in the data signal (PXD)
#define EXTRA_DMA_BUFFER_SIZE 64      // add bytes to avoid glitches
        ESP_LOGW(TAG, "Buffer size=%d bytes is a multiple of 256 bytes, add %d bytes to avoid glitches", c->bufferSize, EXTRA_DMA_BUFFER_SIZE);
        c->bufferSize += EXTRA_DMA_BUFFER_SIZE; // add 64 bytes to avoid glitches
    }
#endif
    int frameSize = /*stereo=Slots*/ 2 * /*bytesPerSlot=*/2; // 16-bit stereo
    chan_cfg.dma_frame_num = c->bufferSize / frameSize;
#if (0 == 22)
    chan_cfg.dma_frame_num = 256;
#endif
#else
    c->bufferSize = (c->bufferSize + 3) & ~3; // round up to multiple of 4 bytes
    // Calculate DMA frame_num based on bufferSize, but keep it within reasonable limits (e.g. 128-512 frames per buffer)
    int frameSize = /*stereo=Slots*/ 2 * /*bytesPerSlot=*/2; // 16-bit stereo
    chan_cfg.dma_frame_num = c->bufferSize / frameSize;
#endif
#if (8 == 0)
    //@@@TODO: helpt niet, remove
    chan_cfg.dma_frame_num = (chan_cfg.dma_frame_num + 3) & ~3; // round up to multiple of 4 frames
    c->bufferSize = chan_cfg.dma_frame_num * frameSize;         // adjust bufferSize to match frame_num
#endif
// chan_cfg.dma_frame_num /= 2; //@@@TODO: remove
#if (0 == 9)
    chan_cfg.dma_desc_num = 4; //@@@TODO: remove, helpt niet
#else
    chan_cfg.dma_desc_num = 2;
#endif
#if (0 == 11)
    //@@@TODO: helpt niet, remove
    chan_cfg.dma_frame_num += 100;
#endif
    ESP_LOGI(TAG, "ADJUSTED: buffer size=%d bytes, DMA frames per buffer=%d, frameSize=%d, DMA buffers=%d", c->bufferSize, chan_cfg.dma_frame_num, frameSize, chan_cfg.dma_desc_num);
    //    chan_cfg.auto_clear_before_cb = true; // do not repeat old data once done
    chan_cfg.auto_clear = true; // do not repeat old data once done
// std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT;  // @@@TODO: DEFAULT bestaat niet
// std_cfg.slot_cfg.left_align = false; //@@@TODO: helpt niet, remove later
// chan_cfg.intr_priority = 7; // set to relativly high priority @@@TODO: remove, helpt niet
#endif

#if (0 == 16)
    std_cfg.clk_cfg.sample_rate_hz = 78125; //@@@TODO: hierdoor ineens dips in de amplitude van het data signaal (PXD) !?
#else
    std_cfg.clk_cfg.sample_rate_hz = c->bitrate / 16 / 2; // lahirunirmalx: 93750
#endif

    ESP_LOGI(TAG, "I2S sample rate=%d Hz", std_cfg.clk_cfg.sample_rate_hz);
#if (ENABLE_I2S_TASK_VERSION == 1)
    portMUX_INITIALIZE(&c->lock);

    c->newData = xSemaphoreCreateBinary();
    c->dataSent = xSemaphoreCreateBinary();
    c->isReady = xSemaphoreCreateBinary();
    c->terminate = false;
#endif
    c->bytesSent = 0;
    c->stats = {}; // reset all statistics to zero

#if (0 == 1)
    c->buffer = (uint8_t *)malloc(c->bufferSize);
#elif (47 == 47)
    // esp32-s3 requires buffer to start at 64-byte aligned address
    //@@@TODO: helpt niet, remove?
    c->buffer = (uint8_t *)heap_caps_aligned_alloc(64, c->bufferSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
#else
    c->buffer = (uint8_t *)heap_caps_malloc(c->bufferSize, MALLOC_CAP_DMA);
#endif
    memset(c->buffer, 0, c->bufferSize);                        /* initializes the reset bytes to zero */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &c->i2s, NULL)); /* Tx channel only (no Rx) */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(c->i2s, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(c->i2s, &callbacks, c));
#if (ENABLE_I2S_CHANNEL_ONLY_ONCE == 1)
    ESP_LOGI(TAG, "Enabling I2S channel only once at init");
    ESP_ERROR_CHECK(i2s_channel_enable(c->i2s));
#else
    ESP_LOGI(TAG, "Enabling/Disabling I2S channel at each transfer");
#endif
    ESP_LOGI(TAG, "I2S channel id=%d, interrupt priority=%d", chan_cfg.id, chan_cfg.intr_priority);

#if (ENABLE_I2S_TASK_VERSION == 1)
    ESP_LOGI(TAG, "Using I2S channel with separate task");
    xTaskCreate(&neopixel_task, TAG, 1024, (void *)c, NEOPIXEL_TASK_PRIORITY, NULL);
#else
    ESP_LOGI(TAG, "Using I2S channel without task");
#endif

    return (tNeopixelContext)c;
}

#if (15 == 15)
void neopixel_clear_buffer(tNeopixelContext ctx) {
    tNpContext *c = (tNpContext *)ctx;
    static const PixelColor black = {.bytes = {0, 0, 0, 0}};
    memset(c->buffer, 0, c->bufferSize); /* initializes the reset bytes to zero */
    for (int i = 0; i < c->nrPixels; i++) {
        neopixel_SetColor(c, i, black);
    }
}
#endif

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

bool neopixel_Show_noTask(tNeopixelContext ctx) { // Did NOT get it to work so far...

    tNpContext *c = (tNpContext *)ctx;
#if (0 == 1)
    c->stats.newDataCounter++; //@@@TODO: needed for debugging only, remove later??

    // Send buffer
    c->bytesSent = 0;
    c->stats.chunksSent = 0;

    static uint32_t idxWritten = 0;
    i2s_channel_write(c->i2s, c->buffer, c->bufferSize, &c->stats.bytesWritten[idxWritten], I2S_TIMEOUT_TICKS);

    idxWritten = (idxWritten + 1) % 10;
#else
    static unsigned long maxWriteMicros = 0;
    unsigned long startMicros = esp_timer_get_time();
    unsigned long writeMicros;
    size_t bytesWritten;
#if (0 == 14)
    // esp_cache_msync(c->buffer, c->bufferSize, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    // Voeg de UNALIGNED flag toe om de foutmelding te negeren/repareren
    esp_cache_msync(c->buffer, c->bufferSize, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#endif
#if (50 == 0)
    // 2. DE ESP32-S3 FIX: Forceer de CPU om de cache direct in het fysieke RAM te duwen.
    // Dit synchroniseert de L1-cache met het DMA-geheugen.
    // werkt niet, geeft foutmelding:
    // E (8222) cache: esp_cache_msync(113): invalid addr or null pointer
    // E (8222) NPIX: Cache sync mislukt: 258
    esp_err_t cache_err = esp_cache_msync(c->buffer, c->bufferSize,
                                          ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    if (cache_err != ESP_OK) {
        // Mocht hij hier tóch falen, dan is de buffer niet DMA-safe gealloceerd!
        ESP_LOGE("NPIX", "Cache sync mislukt: %d", cache_err);
    }

#endif
#if (0 == 24)
    if (c->bufferSize != 832) {
        ESP_LOGE(TAG, "bufferSize=%d, expected=%d", c->bufferSize, 832);
    }
#endif
#if (0 == 19)
    vTaskSuspendAll(); //@@@TODO: remove, bevriest alle RTOS taken!!
#endif
#if (0 == 10)
    //@@@TODO: helpt niet, remove
    esp_err_t rv = i2s_channel_write(c->i2s, c->buffer, c->bufferSize, &bytesWritten, portMAX_DELAY);
#else
    esp_err_t rv = i2s_channel_write(c->i2s, c->buffer, c->bufferSize, &bytesWritten, I2S_TIMEOUT_TICKS);
#endif
#if (0 == 20)
    i2s_channel_tx_wait_done(c->i2s, portMAX_DELAY);)   //@@@TODO: remove, bestaat niet meer in ESP-IDF 5.x
#endif
#if (0 == 12)
    //@@@TODO: helpt niet, remove
    esp_rom_delay_us(6000);
#endif
#if (0 == 19)
    esp_rom_delay_us(200); // meer dan genoeg tijd om de data naar de Neopixels te sturen
    xTaskResumeAll();      //@@@TODO: remove, start scheduler weer
#endif
    if (bytesWritten != c->bufferSize) {
        ESP_LOGE(TAG, "i2s_channel_write() wrote %d bytes, expected %d bytes", bytesWritten, c->bufferSize);
    }
    switch (rv) {
    case ESP_OK:
        writeMicros = esp_timer_get_time() - startMicros;
        if (writeMicros > maxWriteMicros) {
            maxWriteMicros = writeMicros;
            ESP_LOGI(TAG, "maxWriteMicros=%lu", maxWriteMicros);
        }
        break;
    case ESP_ERR_TIMEOUT:
        c->stats.writeTimeoutCount++;
        break;
    case ESP_ERR_INVALID_ARG:
        c->stats.writeInvalidArgCount++;
        break;
    case ESP_ERR_INVALID_STATE:
        c->stats.writeInvalidStateCount++;
        break;
    default:
        c->stats.writeOtherErrorCount++;
        break;
    }
#endif
    // Do NOT wait until all data has been sent
    // Instead, use polling in loopNeopixelRing() to check if the data has been sent
    // xSemaphoreTake(c->dataSent, portMAX_DELAY);
#if (0 == 40)
    // Do a dummy flush of zeros in separate DMA buffer, to ensure that the real data DMA buffer is completely sent before the driver stops
    static const uint8_t dummy_flush[64] = {0};
    i2s_channel_write(c->i2s, dummy_flush, sizeof(dummy_flush), nullptr, I2S_TIMEOUT_TICKS);
#endif
#if (42 == 42)
    //@@@TODO: is dit nog nodig, of is 1 groot data buffer met reset bytes aan het einde voldoende??
    //@@@TODO: kan NIET zondermeer weg, dan weer glitches
    // Do a dummy flush of zeros in separate DMA buffer, to ensure that the real data DMA buffer is completely sent before the driver stops
    static const uint8_t dummy_flush[48] = {0};
    i2s_channel_write(c->i2s, dummy_flush, sizeof(dummy_flush), nullptr, I2S_TIMEOUT_TICKS);
#endif
    return true; // @@@TODO: return value should indicate if the data has been sent or not, but for now always return true
}

bool neopixel_Show_wrapper(tNeopixelContext ctx) {
#if (ENABLE_I2S_TASK_VERSION == 1)
    return neopixel_Show(ctx);
#else
    return neopixel_Show_noTask(ctx);
#endif
}

uint32_t neopixel_GetRefreshRate(tNeopixelContext ctx) {
    tNpContext *c = (tNpContext *)ctx;
    return c->bitrate / (c->bufferSize * 8);
}

/* -------------------------------------------------------------------------------------------------------------
 * Helper Functions
 */
#if (ENABLE_I2S_TASK_VERSION == 1)
static IRAM_ATTR bool i2s_tx_queue_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    // Finished sending one (1) DMA buffer
    tNpContext *c = (tNpContext *)user_ctx;
    c->bytesSent += event->size;
    c->stats.chunksSent++;
    if (c->bytesSent >= c->bufferSize) {
        if (c->stats.chunksSent > c->stats.maxChunksSent) {
            c->stats.maxChunksSent = c->stats.chunksSent;
        }
        xSemaphoreGive(c->dataSent);
    }
    return false; // no need for RTOS to check immediately for higher priority task
}
#endif

static IRAM_ATTR bool i2s_tx_queue_overflow_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    // @@@TODO: these overflows seem to be common?? for now, disabled this callback, but keep it here for future reference
    tNpContext *c = (tNpContext *)user_ctx;
    c->stats.overflowCount++;
    return false; // no need for RTOS to check immediately for higher priority task
}

#if (ENABLE_I2S_TASK_VERSION == 1)
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

        c->stats.newDataCounter++; //@@@TODO: needed for debugging only, remove later??

        /* Make a local copy of the current pixel buffer to be sent to the hardware */
        taskENTER_CRITICAL(&c->lock);
        memcpy(buffer, c->buffer, c->bufferSize);
        taskEXIT_CRITICAL(&c->lock);

        c->bytesSent = 0;
        c->stats.chunksSent = 0;
#if (ENABLE_I2S_CHANNEL_ONLY_ONCE == 1)
        // No preload possible, don't do enable/disable, just write the data to the I2S channel
        bytesLoaded = 0; // nothing preloaded
#else
        i2s_channel_preload_data(c->i2s, buffer, c->bufferSize, &bytesLoaded);
        i2s_channel_enable(c->i2s);
#endif
        if (bytesLoaded < c->bufferSize) {
            static uint32_t idxWritten = 0;
            esp_err_t rv = i2s_channel_write(c->i2s, &buffer[bytesLoaded], c->bufferSize - bytesLoaded,
                                             &c->stats.bytesWritten[idxWritten], I2S_TIMEOUT_TICKS /*@@@TODO portMAX_DELAY*/);
            idxWritten = (idxWritten + 1) % 10;
            switch (rv) {
            case ESP_OK:
                break;
            case ESP_ERR_TIMEOUT:
                c->stats.writeTimeoutCount++;
                break;
            case ESP_ERR_INVALID_ARG:
                c->stats.writeInvalidArgCount++;
                break;
            case ESP_ERR_INVALID_STATE:
                c->stats.writeInvalidStateCount++;
                break;
            default:
                c->stats.writeOtherErrorCount++;
                break;
            }
        } //@@@TODO: else??
        xSemaphoreTake(c->dataSent, portMAX_DELAY); /* Wait for buffer to be transferred to hardware */
#if (ENABLE_I2S_CHANNEL_ONLY_ONCE == 1)
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
#endif

#if (0 == 1)
inline static void setpixel_ws2812b(uint8_t *buffer, uint32_t offset, const PixelColor color) {
    const uint8_t *sequence = ws2812b_color_map[color.bytes.g];
    for (int i = 0; i < WS2812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = ws2812b_color_map[color.bytes.r];
        if (i == 6)
            sequence = ws2812b_color_map[color.bytes.b];
        buffer[offset ^ 1] = sequence[i % WS2812B_BYTES_PER_COLOR]; // fill buffer in 16-bit little-endian format
    }
}

//@@@TODO: call this function before transmit
//@@@TODO call using function ptr in context, so universal for both ws2812b and sk6812b
void neopixel_Transcode_ws2812b(uint8_t *bitBuffer, size_t nrPixels, const PixelColor *colorBuffer) { //@@@TODO: logical arg order?
    uint8_t blackPattern[WS2812B_BYTES_PER_PIXEL];                                                    //@@@TDDO: determine once at init
    setpixel_ws2812b(blackPattern, 0, (PixelColor){.value = 0x000000});                               //@@@TODO: not sure if this works with this [offset ^ 1] stuff

    int offset = 0;
    for (uint32_t i = 0; i < nrPixels; ++i) {
        if (colorBuffer[i].value == 0) {
            memcpy(&bitBuffer[offset], blackPattern, WS2812B_BYTES_PER_PIXEL);
            continue;
        }
        setpixel_ws2812b(&bitBuffer[0], offset, colorBuffer[i]);
        offset += WS2812B_BYTES_PER_PIXEL;
    }
}
#endif

static void setpixel_ws2812b(void *ctx, uint32_t index, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;
    uint8_t *buffer = c->buffer;
    uint32_t offset = index * WS2812B_BYTES_PER_PIXEL;

    if (index >= c->nrPixels) { //@@@TODO: needed?
        ESP_LOGE(TAG, "setpixel_ws2812b: index %d out of range (0-%d)", index, c->nrPixels - 1);
        return;
    }
    const uint8_t *sequence = ws2812b_color_map[color.bytes.g];
    for (int i = 0; i < WS2812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = ws2812b_color_map[color.bytes.r];
        if (i == 6)
            sequence = ws2812b_color_map[color.bytes.b];
#if (30 == 30)
        buffer[offset] = sequence[i % WS2812B_BYTES_PER_COLOR]; // fill buffer linearly, no 16-bit little-endian format, let the driver swap bytes
#else
        buffer[offset ^ 1] = sequence[i % WS2812B_BYTES_PER_COLOR]; // fill buffer in 16-bit little-endian format
#endif
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
