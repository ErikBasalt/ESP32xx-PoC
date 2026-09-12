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
#if (80 == 80)
#include "neopixel_seq3_protocols.h"
#else
#include "ws2812b_protocol.h"
#include "sk6812b_protocol.h"
#endif

#if (0 == 14)
#include "esp_cache.h"
#endif

#if (50 == 0)
#include "esp_cache.h"
#endif

#if (120 == 120)
// For enable/disable TX output at each transmit, allowing better scope triggering
#include "hal.h"
#endif

#define TAG "NPIX"

#if (150 == 150)
// Minimum and maximum size of one single DMA buffer
// Ensure yourself that calculated MIN and MAX values are integers (no fractions)
static const size_t BYTES_PER_I2S_FRAME = 4;                                 // 1 frame = 4 bytes (16-bit stereo)
static const uint32_t MIN_FRAMES_PER_DMA_CHUNK = 32 / BYTES_PER_I2S_FRAME;   // 8 frames, too small will make driver unstable
static const uint32_t MAX_FRAMES_PER_DMA_CHUNK = 4000 / BYTES_PER_I2S_FRAME; // 1000 frames, limited by ESP32 DMA hardware (absolute max is 4032 for newer ESP32xx types)
static const size_t DUMMY_FLUSH_BYTES = MIN_FRAMES_PER_DMA_CHUNK * BYTES_PER_I2S_FRAME;
#else
#define NEOPIXEL_MIN_DMA_BUFFER_SIZE 32   // 16 is too small
#define NEOPIXEL_MAX_DMA_BUFFER_SIZE 4032 // max for newer ESP32xx chips (original ESP32 can go upto 4092)
#endif

#define I2S_TIMEOUT_TICKS 1000
#define NEOPIXEL_TASK_PRIORITY (configMAX_PRIORITIES - 1)

#if (SOC_I2S_HW_VERSION_1)
// NOTE: !! VSC is not aware of this define, therefore syntax highlighting does NOT work here !!

//-------------------------------------------------------------------
// SOC_I2S_HW_VERSION_1 chip (the original ESP32 and ESP32-S2)
//-------------------------------------------------------------------

// There is no simple big_endian boolean flag in the I2S configuration struct
// The hardware architecture inherently expects data to be fed into the FIFO in Little-Endian format
// The software (here) must handle any necessary byte swapping for big-endian data.
#define NEOPIXEL_ENABLE_BIG_ENDIAN 0

// The software (here) must prevent I2S transmit overruns
// Without this, the I2S channel will be disabled too early (using enable/disable at every tx), causing last 13 neopixels staying black
#define NEOPIXEL_PREVENT_OVERRUNS 1
#else
//-------------------------------------------------------------------
// SOC_I2S_HW_VERSION_2 chip (ESP32-S3 and later, incl C3 and C6)
//-------------------------------------------------------------------
// I2S hardware can handle big-endian mode directly
#define NEOPIXEL_ENABLE_BIG_ENDIAN 1

// I2S driver blocks until ready, no need for extra measures to prevent transmit overruns
#define NEOPIXEL_PREVENT_OVERRUNS 0 //@@@TODO: could also be 1, any benefit?
#endif

// @@@TODO: evaluate why using preload prevents having to clear the buffer twice
#define NEOPIXEL_USE_PRELOAD 1

#if (NEOPIXEL_USE_PRELOAD)
#if (NEOPIXEL_PREVENT_OVERRUNS == 0)
// Enforce preventing overruns, otherwise not getting write completion feedback
#undef NEOPIXEL_PREVENT_OVERRUNS
#define NEOPIXEL_PREVENT_OVERRUNS 1
#endif
#endif

// Statistical
#define NEOPIXEL_MEASURE_MAX_WRITE_MICROS 1

// Enabling the I2S channel only once (at init) would make sense, but does NOT work properly.
// Seems like the sent data is somehow multiplied by nr of DMA buffers, so with dma_desc_num=6 you get 6 red pixels i/o 1
#define ENABLE_I2S_CHANNEL_ONLY_ONCE 0 //@@TODO: legacy try for Task version, remove
#define ENABLE_I2S_CHANNEL_EVERY_WRITE 1

#define ENABLE_I2S_TASK_VERSION 0
#if (ENABLE_I2S_TASK_VERSION)
static void neopixel_task(void *arg);
#endif

#define ENABLE_I2S_SENT_CALLBACK 1
#if (ENABLE_I2S_SENT_CALLBACK)
static bool i2s_tx_queue_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx);
#endif

#define ENABLE_I2S_OVERFLOW_CALLBACK 0 // @@@TODO: when using preload, it will generate overflow at every preload, unless you preload (1 byte) less than the buffer size!?
#if (ENABLE_I2S_OVERFLOW_CALLBACK)
static bool i2s_tx_queue_overflow_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx);
#endif

static void setpixel_ws2812b(void *c, uint32_t index, const PixelColor color);
static void setpixel_sk6812b(void *c, uint32_t index, const PixelColor color);

static void setAllSameColor_ws2812b(tNeopixelContext ctx, const PixelColor color);
static void setAllSameColor_sk6812b(tNeopixelContext ctx, const PixelColor color);

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
#if (151 == 0)
    // Std config, will not be used, no need to show
    ESP_LOGI(TAG, "DMA buffers=%d, per buffer I2S frames=%d, total DMA bytes=%d", chan_cfg.dma_desc_num, chan_cfg.dma_frame_num, (2 * chan_cfg.dma_desc_num * chan_cfg.dma_frame_num));
#endif
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

#if (NEOPIXEL_ENABLE_BIG_ENDIAN)
    std_cfg.slot_cfg.big_endian = true; // let the ESP32xx hardware handle big-endian mode
    ESP_LOGI(TAG, "Big-endian mode");
#else
    // Do little-endian byte swapping in software
    ESP_LOGI(TAG, "Little-endian mode (ESP32, ESP32-S2)");
#endif
#if (NEOPIXEL_PREVENT_OVERRUNS)
    ESP_LOGI(TAG, "Software is preventing I2S transmit overruns");
#else
    ESP_LOGI(TAG, "Software is NOT preventing I2S transmit overruns");
#endif

#if (NEOPIXEL_USE_PRELOAD)
    ESP_LOGI(TAG, "Using preload for I2S transmit");
#else
    ESP_LOGI(TAG, "NOT using preload for I2S transmit");
#endif

    i2s_event_callbacks_t callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
#if (ENABLE_I2S_SENT_CALLBACK)
        .on_sent = i2s_tx_queue_sent_callback,
#else
        .on_sent = NULL,
#endif

#if (ENABLE_I2S_OVERFLOW_CALLBACK)
        .on_send_q_ovf = i2s_tx_queue_overflow_callback,
#else
        .on_send_q_ovf = NULL,
#endif
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
#elif (44 == 0)
        c->bufferSize = (c->nrPixels * WS2812B_BYTES_PER_PIXEL) + WS2812B_RESET_BYTES;
        //@@@TODO: bepaal minimum gebaseerd op bitrate
#define MIN_DMA_BUFFER_SIZE 896 // minimum DMA buffer size that can be sent without glitches, 768 is too small
        if (c->bufferSize < MIN_DMA_BUFFER_SIZE) {
            ESP_LOGW(TAG, "Buffer size=%d bytes is less than minimum for DMA, incrementing to %d bytes", c->bufferSize, MIN_DMA_BUFFER_SIZE);
            c->bufferSize = MIN_DMA_BUFFER_SIZE;
        }
#else
#if (100 == 0)
        // reset bits are needed here, even when using a separate DMA buffer for the reset bits
        c->bufferSize = (c->nrPixels * WS2812B_BYTES_PER_PIXEL) + WS2812B_RESET_BYTES;
#else
        c->bufferSize = c->nrPixels * WS2812B_BYTES_PER_PIXEL;
#endif
#endif
        c->setpixel = setpixel_ws2812b;
        c->setAllSameColor = setAllSameColor_ws2812b;
        break;
    case NEOPIXEL_MODE_SK6812B:
        c->bitrate = SK6812B_BITRATE;
#if (100 == 0)
        c->bufferSize = (c->nrPixels * SK6812B_BYTES_PER_PIXEL) + SK6812B_RESET_BYTES;
#else
        c->bufferSize = c->nrPixels * SK6812B_BYTES_PER_PIXEL;
#endif
        c->setpixel = setpixel_sk6812b;
        c->setAllSameColor = setAllSameColor_sk6812b;
        break;
    default:
        ESP_LOGE(TAG, "Invalid mode (%d)", mode);
        free(c);
        return NULL;
    }

    ESP_LOGI(TAG, "nrPixels=%d, bit buffer size=%d bytes, bitrate=%d bps", c->nrPixels, c->bufferSize, c->bitrate);

#if (150 == 150)
    setDMAconfig(c->bufferSize, &chan_cfg);
#else
///////////////////////////
#if (7 == 7)
#if (15 == 0)
    c->bufferSize = (c->bufferSize + 63) & ~0x3f; // round up to multiple of 64 bytes
                                                  // Calculate DMA frame_num based on bufferSize, but keep it within reasonable limits (e.g. 128-512 frames per buffer)
#if (45 == 0)
    if ((c->bufferSize % 256) == 0) { // buffer size is a multiple of 256 bytes, which seems to cause glitches in the data signal (PXD)
#define EXTRA_DMA_BUFFER_SIZE 64 // add bytes to avoid glitches
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
    int frameSize = /*stereo=Slots*/ 2 * /*bytesPerSlot=*/2; // 16-bit stereo
    c->bufferSize = (c->bufferSize + 3) & ~3;                // round up to multiple of 1 frame = 4 bytes
#if (105 == 105)
    if (c->bufferSize < NEOPIXEL_MIN_DMA_BUFFER_SIZE) {
        c->bufferSize = NEOPIXEL_MIN_DMA_BUFFER_SIZE;
        ESP_LOGW(TAG, "DMA buffer size too small, adjusting to %d", c->bufferSize);
    }
#endif
// Calculate DMA frame_num based on bufferSize, but keep it within reasonable limits (e.g. 128-512 frames per buffer)
#if (141 == 141)
    if (c->bufferSize <= NEOPIXEL_MAX_DMA_BUFFER_SIZE) {
        chan_cfg.dma_frame_num = c->bufferSize / frameSize;
    } else {
        chan_cfg.dma_frame_num = NEOPIXEL_MAX_DMA_BUFFER_SIZE / frameSize; // limit buffer size to 4032 bytes to avoid glitches
    }
#else
    // Seems to work, but maybe allocatess to much DMA memory?
    chan_cfg.dma_frame_num = c->bufferSize / frameSize;
#endif
#endif
#if (8 == 0)
    //@@@TODO: helpt niet, remove
    chan_cfg.dma_frame_num = (chan_cfg.dma_frame_num + 3) & ~3; // round up to multiple of 4 frames
    c->bufferSize = chan_cfg.dma_frame_num * frameSize;         // adjust bufferSize to match frame_num
#endif

#if (140 == 140)
    chan_cfg.dma_desc_num = 2 + (c->bufferSize / 4032); // at least 2 DMA buffers, plus additional when driving many Neopixels
#else
    chan_cfg.dma_desc_num = 2;
#endif

#if (0 == 11)
    //@@@TODO: helpt niet, remove
    chan_cfg.dma_frame_num += 100;
#endif
#endif

#if (70 == 0)
// with enable/diable channel, auto_clear is not needed anyway, and may cause extra cpu load or delay
// additionally, should not be related to NEOPIXEL_ENABLE_BIG_ENDIAN
#if ((NEOPIXEL_ENABLE_BIG_ENDIAN == 1)
    //    chan_cfg.auto_clear_before_cb = true; // do not repeat old data once done
    chan_cfg.auto_clear = true; // do not repeat old data once done (not really needed when enable/disable I2S channel at each transfer)
#else
    ESP_LOGW(TAG, "No auto_clear"); //@@@TODO: somehow this seems to cause the "missing 13 neopixels" issue ?!
#endif
#endif

#if (104 == 104)
    // STILL needed in case of using dummy_buffer
    // Auto clear prevents the I2S driver to repeat old data once the transfer is done
    // This also is needed when enable/disable I2S per transmit
    ESP_LOGW(TAG, "Auto_clear");
    // chan_cfg.auto_clear = true; // do not repeat old data once done
    chan_cfg.auto_clear_before_cb = true; // do not repeat old data once done
#else
    ESP_LOGW(TAG, "No auto_clear");
#endif

// std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT;  // @@@TODO: DEFAULT bestaat niet
// std_cfg.slot_cfg.left_align = false; //@@@TODO: helpt niet, remove later
// chan_cfg.intr_priority = 7; // set to relativly high priority @@@TODO: remove, helpt niet
///////////////
#endif

    ESP_LOGI(TAG, "ADJUSTED: buffer size=%d bytes, frames/chunk=%d, bytes/frame=%d, DMA chunks=%d", c->bufferSize, chan_cfg.dma_frame_num, BYTES_PER_I2S_FRAME, chan_cfg.dma_desc_num);

#if (0 == 16)
    std_cfg.clk_cfg.sample_rate_hz = 78125; //@@@TODO: hierdoor ineens dips in de amplitude van het data signaal (PXD) !?
#elif (60 == 60)
    std_cfg.clk_cfg.sample_rate_hz = c->bitrate / (BYTES_PER_I2S_FRAME * 8); // frames per sec
#else
    std_cfg.clk_cfg.sample_rate_hz = c->bitrate / 16 / 2; // lahirunirmalx: 93750
#endif

    ESP_LOGI(TAG, "I2S sample rate=%d frames/sec", std_cfg.clk_cfg.sample_rate_hz);
    ESP_LOGI(TAG, "Raw I2S data transmit microseconds=%lld", (int64_t)(chan_cfg.dma_frame_num * chan_cfg.dma_desc_num) * 1000000 / std_cfg.clk_cfg.sample_rate_hz);
#if (ENABLE_I2S_TASK_VERSION)
    portMUX_INITIALIZE(&c->lock);

    c->newData = xSemaphoreCreateBinary();
    c->isReady = xSemaphoreCreateBinary();
    c->terminate = false;
#endif
#if ((ENABLE_I2S_TASK_VERSION) || (NEOPIXEL_PREVENT_OVERRUNS))
    c->dataSent = xSemaphoreCreateBinary();
    c->totalNrChunks = chan_cfg.dma_desc_num; // to check in callback if all chunks have been sent

#if (65 == 0)
#if (NEOPIXEL_PREVENT_OVERRUNS) //@@@TODO: wasalso needed for Task version?
    xSemaphoreGive(c->dataSent);
#endif
#endif

#endif
    c->bytesSent = 0; //@@@TODO: needed here?
    c->stats = {};    // reset all statistics to zero

#if (0 == 1)
    c->buffer = (uint8_t *)malloc(c->bufferSize);
#elif (47 == 0)
    // esp32-s3 requires buffer to start at 64-byte aligned address
    //@@@TODO: helpt niet, remove?
    c->buffer = (uint8_t *)heap_caps_aligned_alloc(64, c->bufferSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
#else
    c->buffer = (uint8_t *)heap_caps_malloc(c->bufferSize, MALLOC_CAP_DMA);
#endif
    memset(c->buffer, 0, c->bufferSize);                        /* initialise the reset bytes to zero */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &c->i2s, NULL)); /* Tx channel only (no Rx) */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(c->i2s, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(c->i2s, &callbacks, c));
#if (ENABLE_I2S_CHANNEL_EVERY_WRITE)
    ESP_LOGI(TAG, "Enabling/Disabling I2S channel at each transfer");
#else
    ESP_LOGI(TAG, "Enabling I2S channel only once at init");
    ESP_ERROR_CHECK(i2s_channel_enable(c->i2s));
#endif
    ESP_LOGI(TAG, "I2S channel id=%d, interrupt priority=%d", chan_cfg.id, chan_cfg.intr_priority);

#if (ENABLE_I2S_TASK_VERSION)
    ESP_LOGI(TAG, "Using I2S channel with separate task");
    xTaskCreate(&neopixel_task, TAG, 1024, (void *)c, NEOPIXEL_TASK_PRIORITY, NULL);
#else
    ESP_LOGI(TAG, "Using I2S channel without task");
#endif

    return (tNeopixelContext)c;
}

#if (15 == 0)
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

#define NEOPIXEL_MINIMUM_INTERVAL_US (1000)

bool neopixel_Show_noTask(tNeopixelContext ctx) { // Did NOT get it to work so far...

    tNpContext *c = (tNpContext *)ctx;
#if (NEOPIXEL_MEASURE_MAX_WRITE_MICROS)
    static unsigned long maxWriteMicros = 0;
    int64_t startMicros = esp_timer_get_time();
    static int64_t endMicros = 0 - NEOPIXEL_MINIMUM_INTERVAL_US;
#endif

#if (108 == 108)
    if (startMicros < (endMicros + NEOPIXEL_MINIMUM_INTERVAL_US)) {
        // After Disable, the I2S driver needs some time to cleanup before the next data transfer
        // Without this delay and driving just 1 neopixel at full speed, after some time esp32c3 will show extra green pixel and may even crash (RTC_SW_CPU_RST)
        // Implicitly, this delay also ensures the Reset timing for neopixels (some types require >=280 us)
        // ESP_LOGW(TAG, "startMicros is less than %d us after endMicros, delta=%lld", NEOPIXEL_MINIMUM_INTERVAL_US, endMicros - startMicros);
        vTaskDelay(pdMS_TO_TICKS(1));
        startMicros = esp_timer_get_time(); // don't measure the delay
    }
#endif

#if (NEOPIXEL_USE_PRELOAD)
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
#if (111 == 111)
// Add dummy DMA chunk, that can be retransmitted without any harm while the I2S channel is finalising
// 4 bytes seem to be working already, using NEOPIXEL_MIN_DMA_BUFFER_SIZE to be sure
// I2S driver will pad the DMA chunk with zeros if necessary
#if (150 == 150)
        static uint8_t dummy_flush[DUMMY_FLUSH_BYTES];
#else
        static uint8_t dummy_flush[NEOPIXEL_MIN_DMA_BUFFER_SIZE];
#endif
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
#endif
    }
#endif

#if (ENABLE_I2S_SENT_CALLBACK)
    c->bytesSent = 0;
    c->stats.chunksSent = 0;
#endif

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
    hal.setNeoPixelEnable(true); // enable the data output
#endif

#if (ENABLE_I2S_CHANNEL_EVERY_WRITE == 1)
    {
        esp_err_t rv;
        rv = i2s_channel_enable(c->i2s);
        if (rv != ESP_OK) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_enable() failed: rv=%d", rv);
        }
    }
#endif

#if (0 == 1)
    c->stats.newDataCounter++; //@@@TODO: needed for debugging only, remove later??

    // Send buffer
    c->bytesSent = 0;

    static uint32_t idxWritten = 0;
    i2s_channel_write(c->i2s, c->buffer, c->bufferSize, &c->stats.bytesWritten[idxWritten], I2S_TIMEOUT_TICKS);

    idxWritten = (idxWritten + 1) % 10;
#else
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

#if (NEOPIXEL_USE_PRELOAD)
    // only preload, no write
#else
    // Write
    size_t bytesWritten;
    esp_err_t rv = i2s_channel_write(c->i2s, c->buffer, c->bufferSize, &bytesWritten, I2S_TIMEOUT_TICKS);
    if (bytesWritten != c->bufferSize) { //@@@TODO: remove, never happens as long as buffers fits in max DMA buffer size (4096)
        ESP_LOGE(TAG, "i2s_channel_write() wrote %d bytes, expected %d bytes", bytesWritten, c->bufferSize);
    }

    switch (rv) { //@@@TODO: simplify to one error counter
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
#endif

#if (NEOPIXEL_PREVENT_OVERRUNS)
    //@@@TODO: investigate why this delay is necessary
    // This delay seems to fix the "missing last 13 neopixels" issue, but causes massive overflowCounts
    // Keeping auto_clear = false seems to solve the massive overflowCounts
    // Still having maxChunksSent=2 with ESP32, not with other platforms
    // Maybe use semaphore from sent callback instead of fixed delay?
    // vTaskDelay(pdMS_TO_TICKS(3));
#if (106 == 106)
    if (xSemaphoreTake(c->dataSent, pdMS_TO_TICKS(1000)) != pdTRUE) { // wait until DMA transfer is complete
        // Never happens (mostly tested with 500ms)
        ESP_LOGE(TAG, "Timeout waiting for DMA transfer to complete");
    }
#else
    xSemaphoreTake(c->dataSent, portMAX_DELAY); // wait until DMA transfer is complete
#endif
#if (104 == 0)
    // When Auto Clear is NOT active, occasionally repeated transmits can be seen on the scope, even when no delay here
    // When Auto Clear is active, occasionally repeated transmits can be seen on the scope, even when no delay here

    // This, together with auto_clear(_before_cb), seem to cause maxChunksSent>1 on ESP32C3 (and others?)
    // Nope. still occurs with pixels=2 and loop delay=100ms

    //@@@TODO: is this extra delay required for ESP32(-S2) only, or also for ESP32-C3, C6 and S3?
    //@@@TODO: this could also be helpful to match the >280 us Reset period of newer WB2812B (V5) sprc
    vTaskDelay(pdMS_TO_TICKS(1)); // extra delay to ensure full DMA transfer completion, before disabling the channel
#endif
#endif

#if (ENABLE_I2S_CHANNEL_EVERY_WRITE == 1)
    {
        esp_err_t rv;
        rv = i2s_channel_disable(c->i2s);
        if (rv != ESP_OK) {
            // Never happens
            ESP_LOGE(TAG, "i2s_channel_disable() failed: rv=%d", rv);
        }

#if (107 == 107)
        i2s_chan_info_t chan_info;
        rv = i2s_channel_get_info(c->i2s, &chan_info);
        if (rv == ESP_OK) {
            if (chan_info.is_enabled) {
                // Never happens
                ESP_LOGW(TAG, "I2S channel is still enabled");
            }
        }
    }
#endif
#endif
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

#endif
    // Do NOT wait until all data has been sent
    // Instead, use polling in loopNeopixelRing() to check if the data has been sent
    // xSemaphoreTake(c->dataSent, portMAX_DELAY);
#if (0 == 40)
    // Do a dummy flush of zeros in separate DMA buffer, to ensure that the real data DMA buffer is completely sent before the driver stops
    static const uint8_t dummy_flush[64] = {0};
    i2s_channel_write(c->i2s, dummy_flush, sizeof(dummy_flush), nullptr, I2S_TIMEOUT_TICKS);
#endif
#if (ENABLE_I2S_CHANNEL_EVERY_WRITE == 0)
#if (42 == 42)
    //@@@TODO: is dit nog nodig, of is 1 groot data buffer met reset bytes aan het einde voldoende??
    //@@@TODO: kan NIET zondermeer weg, dan weer glitches
    // Do a dummy flush of zeros in separate DMA buffer, to ensure that the real data DMA buffer is completely sent before the driver stops
    static const uint8_t dummy_flush[48] = {0};
    i2s_channel_write(c->i2s, dummy_flush, sizeof(dummy_flush), nullptr, I2S_TIMEOUT_TICKS);
#endif
#endif
#if (105 == 0)
    // Moved delay to start, only when called faster than the minimum interval
    if (writeMicros < 1000L) {
        vTaskDelay(pdMS_TO_TICKS(1)); // Prevent overruns
    }
#endif
    return true; // @@@TODO: return value should indicate if the data has been sent or not, but for now always return true
}

bool neopixel_Show_wrapper(tNeopixelContext ctx) {
#if (ENABLE_I2S_TASK_VERSION)
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
#if (ENABLE_I2S_SENT_CALLBACK)
static IRAM_ATTR bool i2s_tx_queue_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    // Finished sending one (1) DMA buffer
    tNpContext *c = (tNpContext *)user_ctx;
    //    xSemaphoreGive(c->dataSent); //@@@TODO:remove
    c->bytesSent += event->size;
    c->stats.chunksSent++;
    if (c->bytesSent >= c->bufferSize) {
        if (c->stats.chunksSent > c->stats.maxChunksSent) {
            c->stats.maxChunksSent = c->stats.chunksSent;
            c->stats.sentBytestAtMaxChunksSent = c->bytesSent;
        }
#if (110 == 0)
#if ((ENABLE_I2S_TASK_VERSION) || (NEOPIXEL_PREVENT_OVERRUNS))
        xSemaphoreGive(c->dataSent);
#endif
#endif
    }
#if (130 == 130)
    if (c->stats.chunksSent == c->totalNrChunks) {
        xSemaphoreGive(c->dataSent);
    }
#elif (110 == 110)
    if (c->stats.chunksSent >= 2) {
        // First chunk is Neopixel data, second chunk is the dummy flush, after that we are done
        //@@@TODO: only give when chunks==2?
        xSemaphoreGive(c->dataSent);
    }
#endif
    return false; // no need for RTOS to check immediately for higher priority task
}
#endif

#if (ENABLE_I2S_OVERFLOW_CALLBACK)
static IRAM_ATTR bool i2s_tx_queue_overflow_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    // @@@TODO: these overflows seem to be common?? for now, disabled this callback, but keep it here for future reference
    tNpContext *c = (tNpContext *)user_ctx;
    c->stats.overflowCount++;
    return false; // no need for RTOS to check immediately for higher priority task
}
#endif

#if (ENABLE_I2S_TASK_VERSION)
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
    const uint8_t *sequence = neopixel_seq3_color_map[color.bytes.g];
    for (int i = 0; i < WS2812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = neopixel_seq3_color_map[color.bytes.r];
        if (i == 6)
            sequence = neopixel_seq3_color_map[color.bytes.b];
        buffer[offset ^ 1] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // fill buffer in 16-bit little-endian format
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
    const uint8_t *sequence = neopixel_seq3_color_map[color.bytes.g];
    for (int i = 0; i < WS2812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = neopixel_seq3_color_map[color.bytes.r];
        if (i == 6)
            sequence = neopixel_seq3_color_map[color.bytes.b];
#if (NEOPIXEL_ENABLE_BIG_ENDIAN)
        buffer[offset] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // using Big-endian, no need to swap bytes
#else
        // buffer[offset] = __builtin_bswap32(sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]); // fill buffer in 16-bit Little-endian format
        buffer[offset ^ 1] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // fill buffer in 16-bit Little-endian format
#endif
    }
}

static void setpixel_sk6812b(void *ctx, uint32_t index, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;
    uint8_t *buffer = c->buffer;
    uint32_t offset = index * SK6812B_BYTES_PER_PIXEL;

    const uint8_t *sequence = neopixel_seq3_color_map[color.bytes.g];
    for (int i = 0; i < SK6812B_BYTES_PER_PIXEL; ++i, ++offset) {
        if (i == 3)
            sequence = neopixel_seq3_color_map[color.bytes.r];
        if (i == 6)
            sequence = neopixel_seq3_color_map[color.bytes.b];
        if (i == 9)
            sequence = neopixel_seq3_color_map[color.bytes.w];
#if (NEOPIXEL_ENABLE_BIG_ENDIAN)
        buffer[offset] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // using Big-endian, no need to swap bytes
#else
        // buffer[offset] = __builtin_bswap32(sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]); // fill buffer in 16-bit Little-endian format
        buffer[offset ^ 1] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // fill buffer in 16-bit Little-endian format
#endif
    }
}

#if (93 == 93)
static void setAllSameColor_ws2812b(tNeopixelContext ctx, const PixelColor color) {
    tNpContext *c = (tNpContext *)ctx;

    //@@@TODO: make universal, by storing WS2812B_BYTES_PER_PIXEL in context, and use c->setpixel() instead of setpixel_ws2812b()

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

    //@@@TODO: make universal, by storing SK6812B_BYTES_PER_PIXEL in context, and use c->setpixel() instead of setpixel_ws2812b()

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

#else
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

#if (90 == 90)
void NeopixelDriver::setAllPixels(const PixelColor color) {
    _fillPixelRange(0, nrPixels, color);
}
#else
void NeopixelDriver::setAllPixels(const PixelColor color) {
    // Fill the first 2 pixels with the color
    // Must be 2 pixels, because the buffer can be filled in 16-bit little-endian format, so different for odd and even Neopixels
    setPixel(0, color);
    if (nrPixels > 1) {
        setPixel(1, color);
        size_t copiedBytes = bytesPerPixel * 2;
        size_t restBytes = (nrPixels * bytesPerPixel) - copiedBytes;

        // Fill the rest of the buffer with copies of the first 2 pixels
        // In each iteration increase the copy size by a factor of 2, until it does not fit anymore
        while (restBytes >= copiedBytes) {
            // Append whole buffer to itself, doubling the size each time
            memcpy(&buffer[copiedBytes], buffer, copiedBytes);
            restBytes -= copiedBytes;
            copiedBytes *= 2;
        }

        // Next copy the remaining bytes (not a power of 2)
        if (restBytes > 0) {
            memcpy(&buffer[copiedBytes], buffer, restBytes);
        }
    }
}
#endif

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
