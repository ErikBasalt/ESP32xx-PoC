/*
***************************************************************************************************
    ESP32xx Neopixel Driver

    Copyright (c) 2026 Erik Basalt
    Released under the MIT License, see the LICENSE file for details.
***************************************************************************************************
*/
#include <esp_system.h>
#include <esp_log.h>

#include "neopixel.h"
#include "neopixel_seq3_protocols.h"

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
// For enable/disable TX output at each transmit, allowing better scope triggering
//@@@TODO: remove, enable/disable should be done outside of this driver
#include "hal.h"
#endif

#define TAG "NPIX"

// Minimum and maximum size of one single DMA buffer
// Ensure yourself that calculated MIN and MAX values are integers (no fractions)
static const size_t BYTES_PER_I2S_FRAME = 4;                                 // 1 frame = 4 bytes (16-bit stereo)
static const uint32_t MIN_FRAMES_PER_DMA_CHUNK = 32 / BYTES_PER_I2S_FRAME;   // 8 frames, too small will make driver unstable
static const uint32_t MAX_FRAMES_PER_DMA_CHUNK = 4000 / BYTES_PER_I2S_FRAME; // 1000 frames, limited by ESP32 DMA hardware (absolute max is 4032 for newer ESP32xx types)
static const size_t DUMMY_FLUSH_BYTES = MIN_FRAMES_PER_DMA_CHUNK * BYTES_PER_I2S_FRAME;

#define I2S_TIMEOUT_TICKS 1000

#if (SOC_I2S_HW_VERSION_1)
// NOTE: !! VSC is not aware of this define, therefore syntax highlighting does NOT work here !!

//-------------------------------------------------------------------
// SOC_I2S_HW_VERSION_1 chip (the original ESP32 and ESP32-S2)
//-------------------------------------------------------------------

// There is no simple big_endian boolean flag in the I2S configuration struct
// The hardware architecture inherently expects data to be fed into the FIFO in Little-Endian format
// The software (here) must handle any necessary byte swapping for big-endian data.
#define NEOPIXEL_ENABLE_BIG_ENDIAN 0
#else
//-------------------------------------------------------------------
// SOC_I2S_HW_VERSION_2 chip (ESP32-S3 and later, incl C3 and C6)
//-------------------------------------------------------------------
// I2S hardware can handle big-endian mode directly
#define NEOPIXEL_ENABLE_BIG_ENDIAN 1
#endif

// Statistical
#define NEOPIXEL_MEASURE_MAX_WRITE_MICROS 1

//@@@TODO: remove when using class implementation
static void setpixel_ws2812b(void *c, uint32_t index, const PixelColor color);
static void setpixel_sk6812b(void *c, uint32_t index, const PixelColor color);

static void setAllSameColor_ws2812b(tNeopixelContext ctx, const PixelColor color);
static void setAllSameColor_sk6812b(tNeopixelContext ctx, const PixelColor color);

/*
---------------------------------------------------------------------------------------------------
    Interrupt callback for I2S transmission completion of one single DMA chunk
---------------------------------------------------------------------------------------------------
*/
static IRAM_ATTR bool i2s_tx_queue_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    // Finished sending one (1) DMA buffer
    tNpContext *c = (tNpContext *)user_ctx;
    c->bytesSent += event->size;
    c->stats.chunksSent++;
    if (c->bytesSent >= c->bufferSize) {                    //@@@TODO: note that bufferSize is EXCLUDING the data flush chunk
        if (c->stats.chunksSent > c->stats.maxChunksSent) { //@@@TODO: why only check here, when sent>=buffer?
            c->stats.maxChunksSent = c->stats.chunksSent;
            c->stats.sentBytestAtMaxChunksSent = c->bytesSent;
        }
    }

    if (c->stats.chunksSent == c->totalNrChunks) { //@@@TODO: decide on using chunks or bytesSent. Decide on exact match or ">="
        xSemaphoreGive(c->dataSent);
    }
    return false; // no need for RTOS to check immediately for higher priority task
}

/*
---------------------------------------------------------------------------------------------------
    Set the DMA configuration for the I2S peripheral to properly handle Neopixel data:
    - Adjust the Neopixel buffer size to match the actual number of bytes to be transmitted
    - Configure the DMA channel with the appropriate number of DMA chunks and frames per chunk
    - Ensure DMA chunk(s) contain complete frames (no fractions) of Neopixel data
    - Prevent I2S driver from repeating the last DMA chunk
---------------------------------------------------------------------------------------------------
*/
static void setDMAconfig(uint32_t &neopixelBufferSize, // [bytes]. When called: just the neopixel data size. On return: adjusted to actual number of bytes to be transmitted with I2S.
                         i2s_chan_config_t *cfg)       // DMA channel configuration to be set for the I2S peripheral
{
    // Required number of frames to contain all the Neopixel data (rounded up)
    uint32_t totalNrFrames = (neopixelBufferSize + BYTES_PER_I2S_FRAME - 1) / BYTES_PER_I2S_FRAME;

    // Required number of DMA chunks for sending Neopixel data (rounded up)
    uint32_t nrChunks = (totalNrFrames + MAX_FRAMES_PER_DMA_CHUNK - 1) / MAX_FRAMES_PER_DMA_CHUNK;

    // Required number of frames in one single DMA chunk (rounded up)
    uint32_t nrFramesPerChunk = (totalNrFrames + nrChunks - 1) / nrChunks;
    if (nrFramesPerChunk < MIN_FRAMES_PER_DMA_CHUNK) {
        nrFramesPerChunk = MIN_FRAMES_PER_DMA_CHUNK;
    }

    // Adjust the Neopixel neopixelBufferSize to the actual number of bytes to be transmitted
    // (adjusted size can be more than just the neopixel data, due to 4-byte alignment and multiple DMA buffers)
    // This will ensure that the Data Flush is done in a separate DMA chunk.
    neopixelBufferSize = nrChunks * nrFramesPerChunk * BYTES_PER_I2S_FRAME;

    // Configure the DMA channel
    cfg->dma_desc_num = nrChunks + 1; // always 1 more than needed for Neopixel data only
    cfg->dma_frame_num = nrFramesPerChunk;
    cfg->auto_clear_before_cb = true; // prevent the I2S driver from repeating the last DMA buffer once it has been sent (and before the channel is really disabled)
}

/*
===================================================================================================
    Init the driver
===================================================================================================
*/
tNeopixelContext neopixel_Initialize(uint32_t nrPixels, gpio_num_t dout_pin, eNeopixelMode mode) {
    tNpContext *c = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
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

#if (NEOPIXEL_ENABLE_BIG_ENDIAN)
    std_cfg.slot_cfg.big_endian = true; // let the ESP32xx hardware handle big-endian mode
    ESP_LOGI(TAG, "Big-endian mode");
#else
    // Do little-endian byte swapping in software
    ESP_LOGI(TAG, "Little-endian mode (ESP32, ESP32-S2)");
#endif

    i2s_event_callbacks_t callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
        .on_sent = i2s_tx_queue_sent_callback,
        .on_send_q_ovf = NULL,
    };

    c = (tNpContext *)malloc(sizeof(*c)); //@@@TODO: replace context by class implementation
    if (NULL == c) {
        ESP_LOGE(TAG, "Failed to allocate context");
        return NULL;
    }
    memset(c, 0, sizeof(*c));

    c->nrPixels = nrPixels;

    switch (mode) {
    case NEOPIXEL_MODE_WS2812B:
        c->bitrate = WS2812B_BITRATE;
        c->bufferSize = c->nrPixels * WS2812B_BYTES_PER_PIXEL;
        c->setpixel = setpixel_ws2812b;
        c->setAllSameColor = setAllSameColor_ws2812b;
        break;
    case NEOPIXEL_MODE_SK6812B:
        c->bitrate = SK6812B_BITRATE;
        c->bufferSize = c->nrPixels * SK6812B_BYTES_PER_PIXEL;
        c->setpixel = setpixel_sk6812b;
        c->setAllSameColor = setAllSameColor_sk6812b;
        break;
    default:
        ESP_LOGE(TAG, "Invalid mode (%d)", mode);
        free(c);
        return NULL;
    }

    ESP_LOGI(TAG, "nrPixels=%d, bit buffer size=%d bytes, bitrate=%d bps", c->nrPixels, c->bufferSize, c->bitrate);

    setDMAconfig(c->bufferSize, &chan_cfg); // bufferSize called by reference, it can be increased

    ESP_LOGI(TAG, "ADJUSTED: buffer size=%d bytes, frames/chunk=%d, bytes/frame=%d, DMA chunks=%d", c->bufferSize, chan_cfg.dma_frame_num, BYTES_PER_I2S_FRAME, chan_cfg.dma_desc_num);

    std_cfg.clk_cfg.sample_rate_hz = c->bitrate / (BYTES_PER_I2S_FRAME * 8); // frames per sec

    ESP_LOGI(TAG, "I2S sample rate=%d frames/sec", std_cfg.clk_cfg.sample_rate_hz);
    ESP_LOGI(TAG, "I2S data TX time=%lld us", (int64_t)(chan_cfg.dma_frame_num * chan_cfg.dma_desc_num) * 1000000 / std_cfg.clk_cfg.sample_rate_hz);

    c->dataSent = xSemaphoreCreateBinary();
    c->totalNrChunks = chan_cfg.dma_desc_num; // to check in callback if all chunks have been sent

    c->bytesSent = 0; //@@@TODO: needed here?
    c->stats = {};    // reset all statistics to zero

    c->buffer = (uint8_t *)heap_caps_malloc(c->bufferSize, MALLOC_CAP_DMA); //@@@TODO: is DMA capability really needed?
    memset(c->buffer, 0, c->bufferSize);                                    /* initialise the reset bytes to zero */

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &c->i2s, NULL)); /* Tx channel only (no Rx) */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(c->i2s, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(c->i2s, &callbacks, c));
    ESP_LOGI(TAG, "I2S channel id=%d, interrupt priority=%d", chan_cfg.id, chan_cfg.intr_priority);

    return (tNeopixelContext)c;
}

void neopixel_Deinit(tNeopixelContext ctx) {
    tNpContext *c = (tNpContext *)ctx;
    if (NULL == c)
        return;

    i2s_del_channel(c->i2s);
    free(c->buffer);
    free(c);
}

/*
===================================================================================================
    Set one Neopixel color in the buffer
===================================================================================================
*/
void neopixel_SetColor(tNeopixelContext ctx, uint32_t index, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;

    if (index < c->nrPixels) {
        c->setpixel(c, index, color);
    }
}

/*
===================================================================================================
    Show the buffer, by sending it to the Neopixels using I2S
===================================================================================================
*/
#define NEOPIXEL_MINIMUM_INTERVAL_US (1000)

bool neopixel_Show(tNeopixelContext ctx) { // Did NOT get it to work so far...

    tNpContext *c = (tNpContext *)ctx;
#if (NEOPIXEL_MEASURE_MAX_WRITE_MICROS)
    static unsigned long maxWriteMicros = 0;
    int64_t startMicros = esp_timer_get_time();
    static int64_t endMicros = 0 - NEOPIXEL_MINIMUM_INTERVAL_US;
#endif

    if (startMicros < (endMicros + NEOPIXEL_MINIMUM_INTERVAL_US)) {
        // After Disable, the I2S driver needs some time to cleanup before the next data transfer
        // Without this delay and driving just 1 neopixel at full speed, after some time esp32c3 will show extra green pixel and may even crash (RTC_SW_CPU_RST)
        // Implicitly, this delay also ensures the Reset timing for neopixels (some types require >=280 us)
        // ESP_LOGW(TAG, "startMicros is less than %d us after endMicros, delta=%lld", NEOPIXEL_MINIMUM_INTERVAL_US, endMicros - startMicros);
        vTaskDelay(pdMS_TO_TICKS(1));
        startMicros = esp_timer_get_time(); // don't measure the delay
    }

    size_t bytesLoaded = 0;
    esp_err_t rv = i2s_channel_preload_data(c->i2s, c->buffer, c->bufferSize, &bytesLoaded);
    if (rv != ESP_OK) {
        // Never happens
        ESP_LOGE(TAG, "i2s_channel_preload_data() failed: rv=%d", rv);
    } else {
        if (bytesLoaded != c->bufferSize) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_preload_data() incomplete: bytesLoaded=%d", bytesLoaded);
        }

        // Add dummy DMA chunk, that can be retransmitted without any harm while the I2S channel is finalising
        // 4 bytes seem to be working already, using NEOPIXEL_MIN_DMA_BUFFER_SIZE to be sure
        // I2S driver will pad the DMA chunk with zeros if necessary
        static uint8_t dummy_flush[DUMMY_FLUSH_BYTES];
        memset(dummy_flush, 0, sizeof(dummy_flush));

        rv = i2s_channel_preload_data(c->i2s, dummy_flush, sizeof(dummy_flush), &bytesLoaded);
        if (rv != ESP_OK) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_preload_data() of DUMMY flush failed: rv=%d", rv);
        } else {
            if (bytesLoaded != sizeof(dummy_flush)) {
                // Never happens
                ESP_LOGE(TAG, "i2s_channel_preload_data() incomplete: bytesLoaded=%d", bytesLoaded);
            }
        }
    }

    c->bytesSent = 0;
    c->stats.chunksSent = 0;

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
    hal.setNeoPixelEnable(true); // enable the data output
#endif

    {
        esp_err_t rv;
        rv = i2s_channel_enable(c->i2s);
        if (rv != ESP_OK) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_enable() failed: rv=%d", rv);
        }
    }

    if (xSemaphoreTake(c->dataSent, pdMS_TO_TICKS(1000)) != pdTRUE) { // wait until DMA transfer is complete
        // Never happens (mostly tested with 500ms)
        ESP_LOGE(TAG, "Timeout waiting for DMA transfer to complete");
    }

    {
        esp_err_t rv;
        rv = i2s_channel_disable(c->i2s);
        if (rv != ESP_OK) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_disable() failed: rv=%d", rv);
        }
    }

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
    hal.setNeoPixelEnable(false); // enable the data output
#endif

#if (NEOPIXEL_MEASURE_MAX_WRITE_MICROS)
    endMicros = esp_timer_get_time();
    auto writeMicros = endMicros - startMicros;
    if (writeMicros > maxWriteMicros) {
        maxWriteMicros = writeMicros;
        ESP_LOGI(TAG, "maxWriteMicros=%lu", maxWriteMicros);
    }
#endif

    return true; // @@@TODO: return value should indicate if the data has been sent or not, but for now always return true
}

/*
---------------------------------------------------------------------------------------------------
    WS2812B specific function to set a single pixel's color in the buffer
---------------------------------------------------------------------------------------------------
*/
static void setpixel_ws2812b(void *ctx, uint32_t index, const PixelColor pixel) {
    tNpContext *c = (tNpContext *)ctx;
    uint8_t *buffer = c->buffer;
    uint32_t offset = index * WS2812B_BYTES_PER_PIXEL;

    if (index >= c->nrPixels) { //@@@TODO: needed?
        ESP_LOGE(TAG, "setpixel_ws2812b: index %d out of range (0-%d)", index, c->nrPixels - 1);
        return;
    }
    const uint8_t *sequence = neopixel_seq3_color_map[pixel.color.g];
    for (int i = 0; i < WS2812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = neopixel_seq3_color_map[pixel.color.r];
        if (i == 6)
            sequence = neopixel_seq3_color_map[pixel.color.b];
#if (NEOPIXEL_ENABLE_BIG_ENDIAN)
        buffer[offset] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // using Big-endian, no need to swap bytes
#else
        // buffer[offset] = __builtin_bswap32(sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]); // fill buffer in 16-bit Little-endian format
        buffer[offset ^ 1] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // fill buffer in 16-bit Little-endian format
#endif
    }
}

/*
---------------------------------------------------------------------------------------------------
    SK6812B specific function to set a single pixel's color in the buffer
---------------------------------------------------------------------------------------------------
*/
static void setpixel_sk6812b(void *ctx, uint32_t index, const PixelColor pixel) {
    tNpContext *c = (tNpContext *)ctx;
    uint8_t *buffer = c->buffer;
    uint32_t offset = index * SK6812B_BYTES_PER_PIXEL;

    const uint8_t *sequence = neopixel_seq3_color_map[pixel.color.g];
    for (int i = 0; i < SK6812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = neopixel_seq3_color_map[pixel.color.r];
        if (i == 6)
            sequence = neopixel_seq3_color_map[pixel.color.b];
        if (i == 9)
            sequence = neopixel_seq3_color_map[pixel.color.w];
#if (NEOPIXEL_ENABLE_BIG_ENDIAN)
        buffer[offset] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // using Big-endian, no need to swap bytes
#else
        // buffer[offset] = __builtin_bswap32(sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]); // fill buffer in 16-bit Little-endian format
        buffer[offset ^ 1] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // fill buffer in 16-bit Little-endian format
#endif
    }
}

#if (93 == 93) //@@@TODO: replace by improved implementation with class
static void setAllSameColor_ws2812b(tNeopixelContext ctx, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;

    // Fill the first 2 pixels with the color
    // Must be 2 pixels, because the buffer can be filled in 16-bit little-endian format, so different for odd and even neopixels
    setpixel_ws2812b(c, 0, color);
    if (c->nrPixels > 1) {
        setpixel_ws2812b(c, 1, color);
        size_t copiedBytes = WS2812B_BYTES_PER_PIXEL * 2;
        size_t restBytes = (c->nrPixels * WS2812B_BYTES_PER_PIXEL) - copiedBytes;

        // Fill the rest of the buffer with copies of the first 2 pixels
        // In each iteration increase the copy size by a factor of 2, until it does not fit anymore
        while (copiedBytes <= restBytes) {
            memcpy(&c->buffer[copiedBytes], c->buffer, copiedBytes);
            restBytes -= copiedBytes;
            copiedBytes *= 2;
        }

        // Next copy the remaining bytes (not a power of 2)
        if (restBytes > 0) {
            memcpy(&c->buffer[copiedBytes], c->buffer, restBytes);
        }
    }
}

static void setAllSameColor_sk6812b(tNeopixelContext ctx, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;

    // Fill the first 2 pixels with the color
    // Must be 2 pixels, because the buffer can be filled in 16-bit little-endian format, so different for odd and even neopixels
    setpixel_sk6812b(c, 0, color);
    if (c->nrPixels > 1) {
        setpixel_sk6812b(c, 1, color);
        size_t copiedBytes = SK6812B_BYTES_PER_PIXEL * 2;
        size_t restBytes = (c->nrPixels * SK6812B_BYTES_PER_PIXEL) - copiedBytes;

        // Fill the rest of the buffer with copies of the first 2 pixels
        // In each iteration increase the copy size by a factor of 2, until it does not fit anymore
        while (copiedBytes <= restBytes) {
            memcpy(&c->buffer[copiedBytes], c->buffer, copiedBytes);
            restBytes -= copiedBytes;
            copiedBytes *= 2;
        }

        // Next copy the remaining bytes (not a power of 2)
        if (restBytes > 0) {
            memcpy(&c->buffer[copiedBytes], c->buffer, restBytes);
        }
    }
}

void setAllSameColor(tNeopixelContext ctx, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;
    c->setAllSameColor(c, color);
}

#else //@@@TODO: use this improved implementation with class
void NeopixelDriver::_fillPixelRange(size_t startIndex, size_t nrPixelsInRange, const PixelColor color) {
    setPixel(startIndex, color);
    if (nrPixelsInRange > 1) {
        setPixel(startIndex + 1, color);

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
    }
}

void NeopixelDriver::setAllPixels(const PixelColor color) {
    _fillPixelRange(0, nrPixels, color);
}

void NeopixelDriver::setPixelRange(size_t startIndex, size_t endIndex, const PixelColor color) {
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
#endif
