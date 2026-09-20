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
#include "neopixel_seq3.h"
#include "neopixel_seq4.h"

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
// For enable/disable TX output at each transmit, allowing better scope triggering
#include "hal.h"
#endif

#define TAG "NPIX"

// Explicit template instantiation for all supported PixelType variants
// (see "enum class PixelType" in .h file for definition of PixelType)
template class NeopixelDriver<PixelType::GRB_SEQ3>;
template class NeopixelDriver<PixelType::GRBW_SEQ3>;
template class NeopixelDriver<PixelType::GRBW_SEQ4>;

// Minimum and maximum size of one single DMA chunk
// Ensure yourself that calculated MIN and MAX values are integers (no fractions)
static const size_t BYTES_PER_I2S_FRAME = 4;                                 // 1 frame = 4 bytes (16-bit stereo)
static const uint32_t MIN_FRAMES_PER_DMA_CHUNK = 32 / BYTES_PER_I2S_FRAME;   // 8 frames, too small will make driver unstable
static const uint32_t MAX_FRAMES_PER_DMA_CHUNK = 4000 / BYTES_PER_I2S_FRAME; // 1000 frames, limited by ESP32 DMA hardware (absolute max is 4032 for newer ESP32xx types)
static const size_t DUMMY_FLUSH_BYTES = MIN_FRAMES_PER_DMA_CHUNK * BYTES_PER_I2S_FRAME;

static const uint32_t NEOPIXEL_I2S_TRANSMIT_TIMEOUT_MS = 1000;

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

#if (ENABLE_I2S_TASK_VERSION == 0)
/*
---------------------------------------------------------------------------------------------------
    Interrupt callback for I2S transmission completion of one single DMA chunk

    Per Neopixel transmission, two or more multiple DMA chunks are used:
    - one (or more) for the raw Neopixel data
    - one for the dummy flush with all zeros
---------------------------------------------------------------------------------------------------
*/
template <PixelType Mode>
IRAM_ATTR bool NeopixelDriver<Mode>::onSentCallback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *classContext) {
    auto *c = (NeopixelDriver<Mode> *)classContext;

    c->sentNrChunks++;
    if (c->sentNrChunks > c->stats.maxNrChunksSent) {
        // Sometimes the driver cannot stop immediately and repeats the last DMA chunk at the end of the transmission,
        // leading to more chunks being sent than the raw Neopixel data requires
        // Since the last chunk is the dummy flush with all zeros this does not hurt,
        // this statistic only tracks the occurrence.
        c->stats.maxNrChunksSent = c->sentNrChunks;
    }

#if (0 == 1)
    if (c->sentNrChunks == c->totalNrChunks) {
        // All Neopixel DMA chunks (incl dummy flush) have been sent, signal the waiting task it can continue now
        xSemaphoreGive(c->allSentSemaphore); //@@@TODO: use xSemaphoreGiveFromISR() if called from ISR ?!!
    }
    return (false); // no need for RTOS to check immediately for higher priority task
#else
    BaseType_t high_task_woken = pdFALSE;
    if (c->sentNrChunks == c->totalNrChunks) {
        // All Neopixel DMA chunks (incl dummy flush) have been sent, signal the waiting task it can continue now

        xSemaphoreGiveFromISR(c->allSentSemaphore, &high_task_woken);
    }
    return (high_task_woken == pdTRUE);
#endif
}
#endif

/*
@@@@@@@@@@@@@@@@@@@@@@@
bool IRAM_ATTR onI2SSent(
    i2s_chan_handle_t handle,
    i2s_event_data_t *event,
    void *user_ctx)
{
    BaseType_t high_task_woken = pdFALSE;

    xSemaphoreGiveFromISR(
        (SemaphoreHandle_t)user_ctx,
        &high_task_woken
    );

    return high_task_woken == pdTRUE;
}
@@@@@@@@@@
{
    BaseType_t high_task_woken = pdFALSE;

    xTaskNotifyFromISR(
        (TaskHandle_t)user_ctx,
        EVT_SENT,
        eSetBits,
        &high_task_woken
    );

    return high_task_woken == pdTRUE;
}
@@@@@@@@@@@@@@@@@@@@@@@@
*/

/*
---------------------------------------------------------------------------------------------------
    Set the DMA configuration for the I2S peripheral to properly handle Neopixel data:
    - Adjust the Neopixel buffer size to match the actual number of bytes to be transmitted
    - Configure the DMA channel with the appropriate number of DMA chunks and frames per chunk
    - Ensure DMA chunk(s) contain complete frames (no fractions) of Neopixel data
    - (try to) Prevent I2S driver from repeating the last DMA chunk, not sure if this always works
---------------------------------------------------------------------------------------------------
*/
static void setDMAconfig(size_t &neopixelBufferSize, // [bytes]. When called: just the Neopixel data size. On return: adjusted to actual number of bytes to be transmitted with I2S.
                         i2s_chan_config_t *cfg)     // DMA channel configuration to be set for the I2S peripheral
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
    // (adjusted size can be more than just the Neopixel data, due to 4-byte alignment and multiple DMA buffers)
    // This will ensure that the Data Flush is done in a separate DMA chunk.
    neopixelBufferSize = nrChunks * nrFramesPerChunk * BYTES_PER_I2S_FRAME;

    // Configure the DMA channel
    cfg->dma_desc_num = nrChunks + 1; // always 1 more than needed for Neopixel data only
    cfg->dma_frame_num = nrFramesPerChunk;
    cfg->auto_clear_before_cb = true; // (try to) prevent the I2S driver from repeating the last DMA buffer once it has been sent (and before the channel is really disabled)
}

/*
===================================================================================================
    Init the driver
===================================================================================================
*/
template <PixelType Mode>
bool NeopixelDriver<Mode>::begin(const size_t arg_nrPixels, const gpio_num_t dataPin) {
    uint32_t bitRate;

    if (arg_nrPixels == 0) {
        ESP_LOGE(TAG, "Number of pixels must be greater than zero");
        return (false);
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(0), // rate is configured later
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_GPIO_UNUSED,
            .ws = I2S_GPIO_UNUSED,
            .dout = dataPin,
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

    if constexpr (Mode == PixelType::GRB_SEQ3) {
        //---------------------------------------
        //  GRB, seq3 timing
        //---------------------------------------
        ESP_LOGI(TAG, "GRB Neopixels, seq3 timing");
        txBytesPerColor = NEOPIXEL_SEQ3_BYTES_PER_COLOR; // seq3 encoding uses 3 bits per color bit, so 3 bytes per R/G/B color component
        txBytesPerPixel = txBytesPerColor * 3;           // 3 color components (R, G, B), 9 bytes in total
        bitRate = (800000UL * txBytesPerColor);          // Neopixel at 800kHz * 3 bits = 2.4 Mbps (417 ns/bit)
    } else if constexpr (Mode == PixelType::GRBW_SEQ3) {
        //---------------------------------------
        //  GRBW, seq3 timing
        //---------------------------------------
        ESP_LOGI(TAG, "GRBW Neopixels, seq3 timing");
        txBytesPerColor = NEOPIXEL_SEQ3_BYTES_PER_COLOR; // seq3 encoding uses 3 bits per color bit, so 3 bytes per R/G/B/W color component
        txBytesPerPixel = txBytesPerColor * 4;           // 4 color components (R, G, B, W), 12 bytes in total
        bitRate = (800000UL * txBytesPerColor);          // Neopixel at 800kHz * 3 bits = 2.4 Mbps (417 ns/bit)
    } else if constexpr (Mode == PixelType::GRBW_SEQ4) {
        //---------------------------------------
        //  GRBW, seq4 timing
        //---------------------------------------
        ESP_LOGI(TAG, "GRBW Neopixels, seq4 timing");
        txBytesPerColor = NEOPIXEL_SEQ4_BYTES_PER_COLOR; // seq4 encoding uses 4 bits per color bit, so 4 bytes per R/G/B color component
        txBytesPerPixel = txBytesPerColor * 4;           // 4 color components (G, R, B, W), 16 bytes in total
        bitRate = (800000UL * txBytesPerColor);          // Neopixel at 800kHz * 4 bits = 3.2 Mbps (312.5 ns/bit)
    } else {
        ESP_LOGE(TAG, "Unknown pixel type=%d", static_cast<int>(Mode));
        return (false);
    }

    // Define buffer and DMA sizes
    bufferSize = arg_nrPixels * txBytesPerPixel;
    ESP_LOGI(TAG, "nrPixels=%d, txBytesPerPixel=%d, raw buffer size=%d bytes, bitRate=%d bps", arg_nrPixels, txBytesPerPixel, bufferSize, bitRate);

    setDMAconfig(bufferSize, &chan_cfg); // NOTE: bufferSize called by reference, it can be increased

    ESP_LOGI(TAG, "Optimised buffer size=%d bytes, frames/chunk=%d, bytes/frame=%d, DMA chunks=%d", bufferSize, chan_cfg.dma_frame_num, BYTES_PER_I2S_FRAME, chan_cfg.dma_desc_num);

    buffer = (uint8_t *)malloc(bufferSize);
    if (buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate buffer of size %d bytes", bufferSize);
        return (false);
    }
    memset(buffer, 0, bufferSize); // esp. to ensure the unused bytes in last frame are zeroed

    // Calculate speed
    std_cfg.clk_cfg.sample_rate_hz = bitRate / (BYTES_PER_I2S_FRAME * 8); // frames per sec

    ESP_LOGI(TAG, "I2S sample rate=%d frames/sec", std_cfg.clk_cfg.sample_rate_hz);
    ESP_LOGI(TAG, "I2S data TX time=%lld us", (int64_t)(chan_cfg.dma_frame_num * chan_cfg.dma_desc_num) * 1000000 / std_cfg.clk_cfg.sample_rate_hz);

    // Housekeeping stuff
#if (ENABLE_I2S_TASK_VERSION == 0)
    allSentSemaphore = xSemaphoreCreateBinary(); // to get notified when all DMA chunks data has been transmitted by I2S
#endif
    stats = {}; // reset all statistics to zero

    // Let's go
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s, nullptr)); // create TX channel only (no RX)
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s, &std_cfg));  // initialize it
    ESP_LOGI(TAG, "I2S channel id=%d, interrupt priority=%d", chan_cfg.id, chan_cfg.intr_priority);

    i2s_event_callbacks_t callbacks = {
        .on_recv = nullptr,
        .on_recv_q_ovf = nullptr,
#if (ENABLE_I2S_TASK_VERSION)
        .on_sent = &txControl.onSentCallback, //@@@TODO: why pointer (&) here, and not below?
#else
        .on_sent = NeopixelDriver::onSentCallback,
#endif

        .on_send_q_ovf = nullptr,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(i2s, &callbacks, this));

#if (ENABLE_I2S_TASK_VERSION)
    ESP_LOGI(TAG, "Use task for TX control");
    txControl.init(xTaskGetCurrentTaskHandle(), i2s, chan_cfg.dma_desc_num);
#else
    ESP_LOGI(TAG, "No separate tasks");
    totalNrChunks = chan_cfg.dma_desc_num; // to check in callback if all chunks have been sent
#endif

    // Only now store the nrPixels
    // (when it remains 0, it means begin() was not called successfully)
    nrPixels = arg_nrPixels;

    return (true);
}

/*
===================================================================================================
    Show the buffer, by sending it to the Neopixels using I2S
===================================================================================================
*/
#define NEOPIXEL_MINIMUM_INTERVAL_US (1000)

template <PixelType Mode>
bool NeopixelDriver<Mode>::show(void) {

#if (ENABLE_I2S_TASK_VERSION)
    int64_t startMicros = esp_timer_get_time();
    int64_t endMicros; // @@@TODO: only needed below?

    txControl.waitUntilReady();
#else
    //-------------------------------------------
    //  Prevent sending too frequently
    //-------------------------------------------
    static int64_t endMicros = 0 - NEOPIXEL_MINIMUM_INTERVAL_US;
    int64_t startMicros = esp_timer_get_time();

    if (startMicros < (endMicros + NEOPIXEL_MINIMUM_INTERVAL_US)) {
        // After Disable, the I2S driver needs some time to cleanup before the next data transfer
        // Without this delay and driving just 1 Neopixel at full speed, after some time ESP32-C3 will show extra green pixel and may even crash (RTC_SW_CPU_RST)
        // Implicitly, this delay also ensures the Reset timing for Neopixels (some types require >=280 us)
        vTaskDelay(pdMS_TO_TICKS(1));       // 1 ms
        startMicros = esp_timer_get_time(); // do not include this delay in the write timing calculation
    } // else: sufficient time has passed since the previous end time
#endif
    //-------------------------------------------
    //  Preload the data into I2S
    //-------------------------------------------
    size_t bytesLoaded = 0;
    esp_err_t rv = i2s_channel_preload_data(i2s, buffer, bufferSize, &bytesLoaded);
    if (rv != ESP_OK) {
        // Never happens
        ESP_LOGE(TAG, "i2s_channel_preload_data() failed: rv=%d", rv);
    } else {
        if (bytesLoaded != bufferSize) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_preload_data() incomplete: bytesLoaded=%d", bytesLoaded);
        }

        // Add dummy flush DMA chunk, that can be retransmitted without any harm while the I2S channel is finalising
        // 1 frame of 4 bytes seem to be working already, using some more to be sure
        // If necessary, the I2S driver will pad this DMA chunk with zeros to match the Neopixel chunk sizes
        static const uint8_t dummy_flush[DUMMY_FLUSH_BYTES] = {}; // initialise with all zeros

        rv = i2s_channel_preload_data(i2s, dummy_flush, sizeof(dummy_flush), &bytesLoaded);
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

#if (ENABLE_I2S_TASK_VERSION == 0)
    sentNrChunks = 0;
#endif

    //-------------------------------------------
    //  Enable the channel,
    //  this will start sending to the Neopixels
    //-------------------------------------------

#if (ENABLE_I2S_TASK_VERSION)
    txControl.startTransmit();
#else

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
    hal.setNeoPixelEnable(true); // enable the data output
#endif

    {
        esp_err_t rv;
        rv = i2s_channel_enable(i2s);
        if (rv != ESP_OK) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_enable() failed: rv=%d", rv);
        }
    }

    //-------------------------------------------
    //  Wait until sending is done
    //-------------------------------------------
    if (xSemaphoreTake(allSentSemaphore, pdMS_TO_TICKS(NEOPIXEL_I2S_TRANSMIT_TIMEOUT_MS)) != pdTRUE) { // wait until all DMA transfers are done
        // Never happens (mostly tested with 500ms)
        ESP_LOGE(TAG, "Timeout waiting for DMA transfer to complete");
    } // else: transmit completed in time

    //-------------------------------------------
    //  Disable the channel,
    //  to ensure I2S really stops sending
    //-------------------------------------------
    {
        esp_err_t rv;
        rv = i2s_channel_disable(i2s);
        if (rv != ESP_OK) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_disable() failed: rv=%d", rv);
        }
    }

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
    hal.setNeoPixelEnable(false); // disable the data output
#endif

#endif
    //-------------------------------------------
    //  Measure elapsed time
    //  (also to prevent sending too frequently)
    //-------------------------------------------
    endMicros = esp_timer_get_time();
    int64_t writeMicros = endMicros - startMicros;
    if (writeMicros > stats.maxSendMicros) {
        stats.maxSendMicros = writeMicros;
        ESP_LOGI(TAG, "maxSendMicros=%lld", stats.maxSendMicros); //@@@TODO: remove, show statistics on request
    }

    return (true); // @@@TODO: return value should indicate if the data has been sent or not, but for now always return true
}

/*
===================================================================================================
    Generic function to set the pulse transmit sequence of one single pixel in the buffer
===================================================================================================
*/
template <PixelType Mode>
void NeopixelDriver<Mode>::setPixel(const size_t index, PixelColor pixel) {
    if (index >= nrPixels) {
        return; // silently ignore
    }

    if (brightness != 255) {
        //---------------------------------------
        //  Apply global brighness
        //---------------------------------------
        if (pixel.color.r) pixel.color.r = (pixel.color.r * brightness) >> 8;
        if (pixel.color.g) pixel.color.g = (pixel.color.g * brightness) >> 8;
        if (pixel.color.b) pixel.color.b = (pixel.color.b * brightness) >> 8;
        if constexpr ((Mode == PixelType::GRBW_SEQ3) || (Mode == PixelType::GRBW_SEQ4)) {
            // Only when the Neopixels actually have a white component
            if (pixel.color.w) pixel.color.w = (pixel.color.w * brightness) >> 8;
        }
    } // else: max brightness, no adjustment

    if constexpr (Mode == PixelType::GRB_SEQ3) {
        //---------------------------------------
        //  Set one GRB pixel, seq3 timing
        //---------------------------------------
        size_t offset = index * txBytesPerPixel;

        const uint8_t *sequence = neopixel_seq3_color_map[pixel.color.g];
        for (int i = 0; i < txBytesPerPixel; i++, offset++) { //@@@TODO: consider replacing for-loop with written-out statements
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
    } else if constexpr (Mode == PixelType::GRBW_SEQ3) {
        //---------------------------------------
        //  Set one GRBW pixel, seq3 timing
        //---------------------------------------
        size_t offset = index * txBytesPerPixel;

        const uint8_t *sequence = neopixel_seq3_color_map[pixel.color.g];
        for (int i = 0; i < txBytesPerPixel; i++, offset++) {
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
    } else if constexpr (Mode == PixelType::GRBW_SEQ4) {
        //---------------------------------------
        //  Set one GRBW pixel, seq4 timing
        //---------------------------------------
        auto set_seq4_color = [](uint8_t colorByte, uint8_t *buffer) { // lambda to set one color byte in seq4 format
            const uint8_t *hi = encode_seq4_nibble[colorByte >> 4];
            const uint8_t *lo = encode_seq4_nibble[colorByte & 0x0F];
            buffer[0] = hi[0];
            buffer[1] = hi[1];
            buffer[2] = lo[0];
            buffer[3] = lo[1];
        };

        size_t offset = index * txBytesPerPixel;
        set_seq4_color(pixel.color.g, &buffer[offset]);
        set_seq4_color(pixel.color.r, &buffer[offset + NEOPIXEL_SEQ4_BYTES_PER_COLOR]);
        set_seq4_color(pixel.color.b, &buffer[offset + (2 * NEOPIXEL_SEQ4_BYTES_PER_COLOR)]);
        set_seq4_color(pixel.color.w, &buffer[offset + (3 * NEOPIXEL_SEQ4_BYTES_PER_COLOR)]);
    } // else: unknown pixel type, should not occur
}
