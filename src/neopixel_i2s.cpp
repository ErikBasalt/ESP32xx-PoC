/*
***************************************************************************************************
    ESP32xx Neopixel Driver - I2S implementation

    Copyright (c) 2026 Erik Basalt
    Released under the MIT License, see the LICENSE file for details.
***************************************************************************************************
*/
#include "neopixel_i2s.h"

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
#include "hal.h"
#endif

#define TAG "NI2S"

// Minimum and maximum size of one single DMA chunk
// Ensure yourself that calculated MIN and MAX values are integers (no fractions)
static const size_t BYTES_PER_I2S_FRAME = 4;                                 // 1 frame = 4 bytes (16-bit stereo)
static const uint32_t MIN_FRAMES_PER_DMA_CHUNK = 32 / BYTES_PER_I2S_FRAME;   // 8 frames, too small will make driver unstable
static const uint32_t MAX_FRAMES_PER_DMA_CHUNK = 4000 / BYTES_PER_I2S_FRAME; // 1000 frames, limited by ESP32 DMA hardware (absolute max is 4032 for newer ESP32xx types)
static const size_t DUMMY_FLUSH_BYTES = MIN_FRAMES_PER_DMA_CHUNK * BYTES_PER_I2S_FRAME;

static const uint32_t NEOPIXEL_I2S_TRANSMIT_TIMEOUT_MS = 1000;

// Notification commands from Main (or callback) to Task
constexpr uint32_t CMD_TRANSMIT = 1 << 0; // command to transmit the preloaded data
constexpr uint32_t CMD_SHUTDOWN = 1 << 1; // command to shutdown the Task
constexpr uint32_t EVT_SENT = 1 << 2;     // Event (from sent callback) indicating that all data has been transmitted

// Notification responses from Task to Main
constexpr uint32_t RSP_READY = 1 << 0;   // I2S driver is in Ready state, new data can be preloaded and then transmitted
constexpr uint32_t RSP_STOPPED = 1 << 1; // Shutdown complete

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
    Init the I2S
===================================================================================================
*/
bool NeopixelTransmitControl::init(gpio_num_t dataPin, uint32_t bitRate, size_t nrPixels, size_t txBytesPerPixel, size_t *requiredBufferSizePtr) {
    if (nrPixels == 0) {
        ESP_LOGE(TAG, "Number of pixels must be greater than zero");
        return (false);
    }

    if (dataPin == I2S_GPIO_UNUSED) {
        ESP_LOGE(TAG, "Data pin must be specified");
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

    // Define buffer and DMA sizes
    size_t bufferSize = nrPixels * txBytesPerPixel;
    ESP_LOGI(TAG, "nrPixels=%d, txBytesPerPixel=%d, raw buffer size=%d bytes, bitRate=%d bps", nrPixels, txBytesPerPixel, bufferSize, bitRate);

    setDMAconfig(bufferSize, &chan_cfg); // NOTE: bufferSize called by reference, it can be increased

    ESP_LOGI(TAG, "Optimised buffer size=%d bytes, frames/chunk=%d, bytes/frame=%d, DMA chunks=%d", bufferSize, chan_cfg.dma_frame_num, BYTES_PER_I2S_FRAME, chan_cfg.dma_desc_num);

    // Calculate speed
    std_cfg.clk_cfg.sample_rate_hz = bitRate / (BYTES_PER_I2S_FRAME * 8); // frames per sec

    ESP_LOGI(TAG, "I2S sample rate=%d frames/sec", std_cfg.clk_cfg.sample_rate_hz);
    ESP_LOGI(TAG, "I2S data TX time=%lld us", (int64_t)(chan_cfg.dma_frame_num * chan_cfg.dma_desc_num) * 1000000 / std_cfg.clk_cfg.sample_rate_hz);

    // Let's go
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s, nullptr)); // create TX channel only (no RX)
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s, &std_cfg));  // initialize it
    ESP_LOGI(TAG, "I2S channel id=%d, interrupt priority=%d", chan_cfg.id, chan_cfg.intr_priority);

    i2s_event_callbacks_t callbacks = {
        .on_recv = nullptr,
        .on_recv_q_ovf = nullptr,
        .on_sent = onSentCallback,
        .on_send_q_ovf = nullptr,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(i2s, &callbacks, this));

    ESP_LOGI(TAG, "Use RTOS task for TX control");
    if (xTaskCreatePinnedToCore(
            transmitTask,        // Task function
            "transmitTask",      // Task name
            10000,               // stack size (bytes)     @@@@TODO: reduce stack (current usage is only 404 bytes) 31dec24: min=1644 !? door logging tijdens skippen van een puls?
            this,                // Task parameter: reference to this class instance
            2,                   // priority is just above the normal program   //@@@TODO
            &transmitTaskHandle, // Task handle
            1) != pdPASS) {      // which core to run on (0=system, 1=app)  //@@@TODO: why pinned to core?
        ESP_LOGE(TAG, "Failed to create transmit task");
        return (false);
    }
    return (true);
}

void NeopixelTransmitControl::deinit(void) {
    uint32_t response;

    // Tell Task to shutdown
    xTaskNotify(transmitTaskHandle, CMD_SHUTDOWN, eSetBits);

    // Wait for feedback from Task
    for (int i = 0; i < 100; i++) {
        xTaskNotifyWait(
            0,                // keep all bits on entry
            UINT32_MAX,       // clear all bits on exit
            &response,        // the response from Task
            pdMS_TO_TICKS(10) // wait max 10ms
        );

        if (response & RSP_STOPPED) {
            i2s_del_channel(i2s);
            return;
        } // else: wait and ignore other responses (eg from previous transmit command)
    }

    // Task did not respond, kill it from here
    vTaskDelete(transmitTaskHandle);
    i2s_del_channel(i2s); // @@@TODO: DRY
}

/*
---------------------------------------------------------------------------------------------------
    Interrupt callback for I2S transmission completion of one single DMA chunk

    Per Neopixel transmission, two or more multiple DMA chunks are used:
    - one (or more) for the raw Neopixel data
    - one for the dummy flush with all zeros
---------------------------------------------------------------------------------------------------
*/
IRAM_ATTR bool NeopixelTransmitControl::onSentCallback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *classContext) {
    auto *c = (NeopixelTransmitControl *)classContext;
    BaseType_t higherPrioTaskWakeup = pdFALSE;

    c->sentNrChunks++;
    if (c->sentNrChunks > c->stats.maxNrChunksSent) {
        // Sometimes the driver cannot stop immediately and repeats the last DMA chunk at the end of the transmission,
        // leading to more chunks being sent than the raw Neopixel data requires
        // Since the last chunk is the dummy flush with all zeros this does not hurt,
        // this statistic only tracks the occurrence.
        c->stats.maxNrChunksSent = c->sentNrChunks;
    }

    if (c->sentNrChunks == c->totalNrChunks) {
        // All Neopixel DMA chunks (incl dummy flush) have been sent, signal the waiting task it can continue now

        // Npotofy the transmit Task
        xTaskNotifyFromISR(
            c->transmitTaskHandle,
            EVT_SENT,
            eSetBits,
            &higherPrioTaskWakeup);
    }
    return (higherPrioTaskWakeup == pdTRUE);
}

/*
===================================================================================================
    Start transmitting over I2S

    NOTE:
    At init() the required bufferSize is determined, based on the number of Neopixels.
    Use the same bufferSize here.
===================================================================================================
*/
void NeopixelTransmitControl::startTransmit(uint8_t *buffer, size_t bufferSize) {

    //---------------------------------------------------
    //  Lambda function to wait until Task is ready for new transmit
    //---------------------------------------------------
    auto waitUntilReady = [&](void) {
        uint32_t response;

        for (int i = 0; i < 10; i++) {
            xTaskNotifyWait(
                0,                 // keep all bits on entry
                UINT32_MAX,        // clear all bits on exit
                &response,         // the response from Task
                pdMS_TO_TICKS(100) // wait max 10*100=1000ms, should be enough for more than 10000 Neopixels
            );

            if (response & RSP_READY) {
                return (true);
            } // else: wait and ignore other responses (eg from previous transmit command)
        }
        return (false); // timeout
    };

    int64_t startMicros = esp_timer_get_time();

    //-------------------------------------------
    //  Wait until ready
    //-------------------------------------------
    if (!waitUntilReady()) {
        // Timeout waiting for the I2S Task to become ready
        return;
    }

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

        //-------------------------------------------
        //  Start Transmit
        //-------------------------------------------
        xTaskNotify(transmitTaskHandle, CMD_TRANSMIT, eSetBits);

        int64_t endMicros = esp_timer_get_time();
        int64_t writeMicros = endMicros - startMicros;
        if (writeMicros > stats.maxSendMicros) {
            stats.maxSendMicros = writeMicros;
            ESP_LOGI(TAG, "maxSendMicros=%lld", stats.maxSendMicros); //@@@TODO: remove, show statistics on request
        }
    }
}

/*
***************************************************************************************************
    Task to start sending the preloaded data and wait for completion

    This Task only knows the number of preloaded DMA chunks to be sent, nothing about the contents
    Will also take care for enable/disable the I2S channel
***************************************************************************************************
*/
void NeopixelTransmitControl::transmitTask(void *taskArg) {
    NeopixelTransmitControl *here = (NeopixelTransmitControl *)taskArg; // pointer to class instance that started this task
    here->minimumFreeStack = uxTaskGetStackHighWaterMark(NULL);         // init Task stack usage

    // TASKLOG("Starting i2sTask"); // NOTE: this will NOT work when logging at main task is not active yet

    //---------------------------------------------------
    //  Lambda function to shutdown this Task
    //---------------------------------------------------
    auto shutdown = [&](void) {
        // Do any other cleanup owned by this task here

        // Tell main task that we are completely finished
        xTaskNotify(here->parentTaskHandle,
                    RSP_STOPPED,
                    eSetBits);

        here->transmitTaskHandle = nullptr; // forget this task
        vTaskDelete(nullptr);               // kill this task
        // No code should be executed after this
    };

    //---------------------------------------------------
    //  Enter the infinite Task loop
    //---------------------------------------------------
    for (;;) {
        uint32_t notification;
        bool isTxRunning = false;
        bool isPendingShutdown = false;

        xTaskNotifyWait(
            0,             // keep all bits on entry
            UINT32_MAX,    // clear all bits on exit
            &notification, // the command from parent task
            portMAX_DELAY  // infinite wait
        );

        if (notification & CMD_SHUTDOWN) {
            //-------------------------------------------------------
            //  Shutdown command from parent task
            //-------------------------------------------------------
            if (isTxRunning) {
                // Postpone shutdown until transmission is complete
                isPendingShutdown = true;
            } else {
                shutdown();
            }
        }

        if (notification & CMD_TRANSMIT) {
            //-------------------------------------------------------
            //  Transmit command from parent task
            //-------------------------------------------------------
            if (!isTxRunning) {
                isTxRunning = true;
#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
                hal.setNeoPixelEnable(true); // enable the data output
#endif
                // Start transmitting the preloaded data
                here->sentNrChunks = 0;

                i2s_channel_enable(here->i2s);

                // Now wait for EVT_SENT, indicating all the DMA chunks were sent

            } else {
                // Transmit overrun, caller should have waited for RSP_READY before issuing a new CMD_TRANSMIT
                // This new transmit command will be ignored
                here->stats.nrOverruns++;
            }
        }

        if (notification & EVT_SENT) {
            //-------------------------------------------------------
            //  Event from I2S "on_sent" callback,
            //  indicating all DMA chunks have been sent
            //-------------------------------------------------------
            i2s_channel_disable(here->i2s);
#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
            hal.setNeoPixelEnable(false); // disable the data output
#endif
            // Delay to ensure I2S is fully done
            vTaskDelay(pdMS_TO_TICKS(1));

            if (isPendingShutdown) {
                // Shutdown was pending, do it now
                shutdown();
            } else {
                // I2S Driver is Ready for new transmit command
                isTxRunning = false;

                // Notify parent
                xTaskNotify(here->parentTaskHandle,
                            RSP_READY,
                            eSetBits);
            }
        }

        // Update Task stack usage
        UBaseType_t currentFreeStack = uxTaskGetStackHighWaterMark(NULL);
        if (currentFreeStack < here->minimumFreeStack) {
            here->minimumFreeStack = currentFreeStack;
        }
    }
}
