/*
***************************************************************************************************
    ESP32xx Neopixel Driver - I2S implementation

    Copyright (c) 2026 Erik Basalt
    Released under the MIT License, see the LICENSE file for details.
***************************************************************************************************
*/
#include "neopixel_i2s.h"

// Enable or disable output at every write to the Neopixels, for scope triggering on the Enable signal
//@@@TODO: remove, enable/disable should be done outside of this driver
#define NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE 1

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
#include "hal.h"
#endif

#define TAG "I2S_"

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
    Helper to wait for a specific notification
---------------------------------------------------------------------------------------------------
*/
static bool waitForNotification(uint32_t expectedNotification) {
    uint32_t notification;

    for (int i = 0; i < 2; i++) {
        xTaskNotifyWait(
            0,                 // keep all bits on entry
            UINT32_MAX,        // clear all bits on exit
            &notification,     // the response from Task
            pdMS_TO_TICKS(500) // wait max 2*500=1000ms, should be enough for more than 10000 Neopixels
        );

        if (notification & expectedNotification) {
            return (true); // Received expected Notification
        } // else: clear and ignore other Notifications and wait longer
    }
    return (false); // timeout
};

/*
---------------------------------------------------------------------------------------------------
    Set the DMA configuration for the I2S peripheral to properly handle Neopixel data:
    - Adjust the Neopixel buffer size to match the actual number of bytes to be transmitted
    - Configure the DMA channel with the appropriate number of DMA chunks and frames per chunk
    - Ensure DMA chunk(s) contain complete frames (no fractions) of Neopixel data
    - (try to) Prevent I2S driver from repeating the last DMA chunk, not sure if this always works
---------------------------------------------------------------------------------------------------
*/
static void setDMAconfig(
    size_t &neopixelBufferSize, // [bytes]. When called: just the Neopixel data size. On return: adjusted to actual number of bytes to be transmitted with I2S.
    i2s_chan_config_t *cfg) {   // DMA channel configuration to be set for the I2S peripheral

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
bool NeopixelTransmitControl::init(
    gpio_num_t dataPin,              // GPIO pin used for the I2S data output
    uint32_t bitRate,                // [bps] bit rate for the I2S transmission
    size_t rawDataSize,              // [bytes] size of the raw Neopixel data
    size_t *requiredBufferSizePtr) { // pointer to store the required buffer size [bytes] for the I2S transmission

    if (rawDataSize == 0) {
        ESP_LOGE(TAG, "Raw data size must be greater than zero");
        return (false);
    }

    if (dataPin == I2S_GPIO_UNUSED) {
        ESP_LOGE(TAG, "Data pin must be specified");
        return (false);
    }

    // Store the RTOS handle to parent task, for sending Notifications to
    // This CANNOT be done in the constructor, when the task is not active yet
    parentTaskHandle = xTaskGetCurrentTaskHandle();

    // Configure I2S
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

#if (NEOPIXEL_USE_BIG_ENDIAN_DATA)
    std_cfg.slot_cfg.big_endian = true; // let the ESP32xx hardware handle big-endian data encoding
    ESP_LOGI(TAG, "Big-endian mode");
#else
    // Do little-endian byte swapping in software
    ESP_LOGI(TAG, "Little-endian mode (ESP32, ESP32-S2)");
#endif

    // Define buffer and DMA sizes
    ESP_LOGI(TAG, "Raw data size=%u bytes, bitRate=%d bps", rawDataSize, bitRate);

    size_t bufferSize = rawDataSize;
    setDMAconfig(bufferSize, &chan_cfg); // NOTE: bufferSize called by reference, it can be increased

    ESP_LOGI(TAG, "Optimised buffer size=%d bytes, frames/chunk=%d, bytes/frame=%d, DMA chunks=%d", bufferSize, chan_cfg.dma_frame_num, BYTES_PER_I2S_FRAME, chan_cfg.dma_desc_num);
    totalNrChunks = chan_cfg.dma_desc_num; // to check in callback if all chunks have been sent
    *requiredBufferSizePtr = bufferSize;   // for caller to allocate buffer

    // Calculate speed
    std_cfg.clk_cfg.sample_rate_hz = bitRate / (BYTES_PER_I2S_FRAME * 8); // frames per sec

    ESP_LOGI(TAG, "I2S sample rate=%d frames/sec", std_cfg.clk_cfg.sample_rate_hz);
    ESP_LOGI(TAG, "I2S data TX time=%lld us", (int64_t)(chan_cfg.dma_frame_num * chan_cfg.dma_desc_num) * 1000000 / std_cfg.clk_cfg.sample_rate_hz);

    // Let's go
    if (i2s_new_channel(&chan_cfg, &i2s, nullptr) != ESP_OK) { // create TX channel only (no RX)
        ESP_LOGE(TAG, "Failed to initialize I2S channel");
        return (false);
    }
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s, &std_cfg));
    ESP_LOGI(TAG, "I2S interrupt priority=%d", chan_cfg.intr_priority);

    i2s_event_callbacks_t callbacks = {
        .on_recv = nullptr,
        .on_recv_q_ovf = nullptr,
        .on_sent = onSentCallback,
        .on_send_q_ovf = nullptr,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(i2s, &callbacks, this));

    i2s_chan_info_t chan_info;
    i2s_channel_get_info(i2s, &chan_info);
    ESP_LOGI(TAG, "I2S channel=%d, total DMA buffer size=%u", chan_info.id, chan_info.total_dma_buf_size);

#if (ENABLE_I2S_TASK_VERSION)
    UBaseType_t priority = uxTaskPriorityGet(NULL); // Parent task (this) priority
    if (priority < (configMAX_PRIORITIES / 2)) {
        priority++; // Task prio is 1 higher
    } // else: Already at very high prio, have Task at same

    BaseType_t core = xPortGetCoreID(); // run Task on same core as Parent, for fastest Notifications

    ESP_LOGI(TAG, "Starting TX control Task on core=%u, priority=%u", core, priority);

    if (xTaskCreatePinnedToCore(
            transmitTask, // Task function to run
            "NeopixelTX", // Task name in RTOS
            1000,         // [bytes] stack size, 23sep26: max usage on ESP32=566 bytes, S3=772 (!), C3=320, C6=296
            this,         // Task parameter: reference to this Parent class instance
            priority,
            &transmitTaskHandle, // created Task handle
            core) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        return (false);
    }
#else
    ESP_LOGI(TAG, "No task");
#endif
    return (true);
}

/*
===================================================================================================
    De-init the I2S
===================================================================================================
*/
void NeopixelTransmitControl::deinit(void) {
#if (ENABLE_I2S_TASK_VERSION)
    TaskHandle_t tmpHandle = transmitTaskHandle; // Use a temporary handle to safely delete the task (and not ourself)
    if (tmpHandle) {
        // Tell Task to shutdown
        xTaskNotify(transmitTaskHandle, CMD_SHUTDOWN, eSetBits);
        if (waitForNotification(RSP_STOPPED)) {
            ESP_LOGI(TAG, "Transmit task stopped successfully");
        } else {
            ESP_LOGW(TAG, "Transmit task did not respond to shutdown command, killing it");
            tmpHandle = transmitTaskHandle; // check once again
            if (tmpHandle) {
                vTaskDelete(tmpHandle);
                transmitTaskHandle = nullptr;
            } // else: Task was stopped anyway
        }
    }
#endif
    i2s_del_channel(i2s);
    i2s = nullptr;
}

/*
---------------------------------------------------------------------------------------------------
    Interrupt callback for I2S transmission completion of one single DMA chunk

    Per Neopixel transmission, two or more DMA chunks are used:
    - one (or more) for the raw Neopixel data
    - one for the dummy flush with all zeros
---------------------------------------------------------------------------------------------------
*/
IRAM_ATTR bool NeopixelTransmitControl::onSentCallback(
    i2s_chan_handle_t handle, // the I2S channel handle that triggered the callback
    i2s_event_data_t *event,  // event data associated with the I2S transmission
    void *initiatorContext) { // pointer to the context of the transmit initiator

    auto *c = (NeopixelTransmitControl *)initiatorContext;
    BaseType_t higherPrioTaskWoken = pdFALSE;

    c->sentNrChunks++;
    if (c->sentNrChunks > c->statsPtr->maxNrChunksSent) {
        // Sometimes the driver cannot stop immediately and repeats the last DMA chunk at the end of the transmission,
        // leading to more chunks being sent than the raw Neopixel data requires
        // Since the last chunk is the dummy flush with all zeros this does not hurt,
        // this statistic only tracks the occurrence.
        c->statsPtr->maxNrChunksSent = c->sentNrChunks;
    }

    if (c->sentNrChunks == c->totalNrChunks) {
        // All Neopixel DMA chunks (incl dummy flush) have been sent, signal the waiting task it can continue now

        // Notify the Task that initiated the transmission
        //@@@TODO: check if the task handle != nullptr? might happen in deinit() when transmitting is still ongoing
        xTaskNotifyFromISR(
#if (ENABLE_I2S_TASK_VERSION)
            c->transmitTaskHandle,
#else
            c->parentTaskHandle,
#endif
            EVT_SENT,
            eSetBits,
            &higherPrioTaskWoken);
    }
    return (higherPrioTaskWoken == pdTRUE);
}

/*
===================================================================================================
    Start transmitting over I2S

    NOTE:
    At init() the required bufferSize is determined, based on the number of Neopixels.
    Use the same bufferSize here.
===================================================================================================
*/
#define NEOPIXEL_MINIMUM_INTERVAL_US (1000)

void NeopixelTransmitControl::startTransmit(
    uint8_t *buffer,     // pointer to the data buffer containing the Neopixel data
    size_t bufferSize) { // size of the buffer [bytes], should match the bufferSize determined at init()

    static int64_t endMicros = 0 - NEOPIXEL_MINIMUM_INTERVAL_US;
    int64_t startMicros = esp_timer_get_time();

#if (ENABLE_I2S_TASK_VERSION)
    //-------------------------------------------
    //  Wait until Task is Ready
    //-------------------------------------------
    if (!waitForNotification(RSP_READY)) {
        // Timeout waiting for the I2S Task to become ready
        return;
    }
#else
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
    if (i2s_channel_preload_data(i2s, buffer, bufferSize, &bytesLoaded) == ESP_OK) {
        if (bytesLoaded != bufferSize) {
            ESP_LOGE(TAG, "i2s_channel_preload_data() of buffer incomplete: bytesLoaded=%d", bytesLoaded); // Never happened
            return;
        }
    } else {
        ESP_LOGE(TAG, "i2s_channel_preload_data() of buffer failed"); // Never happened
        return;
    }

    // Add dummy flush DMA chunk, that can be retransmitted without any harm while the I2S channel is finalising
    // 1 frame of 4 bytes seem to be working already, using some more to be sure
    // If necessary, the I2S driver will pad this DMA chunk with zeros to match the Neopixel chunk sizes
    static const uint8_t dummy_flush[DUMMY_FLUSH_BYTES] = {}; // initialise with all zeros
    if (i2s_channel_preload_data(i2s, dummy_flush, sizeof(dummy_flush), &bytesLoaded) == ESP_OK) {
        if (bytesLoaded != sizeof(dummy_flush)) {
            ESP_LOGE(TAG, "i2s_channel_preload_data() of dummy flush incomplete: bytesLoaded=%d", bytesLoaded); // Never happened
            return;
        }
    } else {
        ESP_LOGE(TAG, "i2s_channel_preload_data() of dummy flush failed"); // Never happened
        return;
    }

    //-------------------------------------------
    //  Start the actual data transmission
    //-------------------------------------------
#if (ENABLE_I2S_TASK_VERSION)
    xTaskNotify(transmitTaskHandle, CMD_TRANSMIT, eSetBits);
#else
    transmitNoTask();
#endif

    endMicros = esp_timer_get_time();
    int64_t deltaMicros = endMicros - startMicros;
    if (deltaMicros > statsPtr->maxSendMicros) {
        statsPtr->maxSendMicros = deltaMicros;
        ESP_LOGI(TAG, "maxSendMicros=%lld", statsPtr->maxSendMicros); //@@@TODO: remove, show statistics on request
    }
}

#if (ENABLE_I2S_TASK_VERSION)
/*
***************************************************************************************************
    Task to start sending the preloaded data and wait for completion

    This Task only knows the number of preloaded DMA chunks to be sent, nothing about the contents
    Will also take care for enable/disable the I2S channel

    /!\ NOTE: do NOT use logging in this Task, stack is too small for that.
***************************************************************************************************
*/
void NeopixelTransmitControl::transmitTask(
    void *initiatorContext) { // pointer to context of the initiator of this Task

    NeopixelTransmitControl *here = (NeopixelTransmitControl *)initiatorContext;

    here->statsPtr->minimumFreeStack = uxTaskGetStackHighWaterMark(NULL); // init Task stack usage

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
    //  Lambda function to notify parent
    //  that this task is ready
    //---------------------------------------------------
    auto notifyReady = [&](void) {
        xTaskNotify(here->parentTaskHandle,
                    RSP_READY,
                    eSetBits);
    };

    notifyReady(); // initial notification to parent that this task is Ready

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
                here->statsPtr->nrOverruns++;
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
                notifyReady();
            }
        }

        // Update Task stack usage
        UBaseType_t currentFreeStack = uxTaskGetStackHighWaterMark(NULL);
        if (currentFreeStack < here->statsPtr->minimumFreeStack) {
            here->statsPtr->minimumFreeStack = currentFreeStack;
        }
    }
}
#else
/*
---------------------------------------------------------------------------------------------------
    Transmit Neopixel data immediately without using a separate RTOS task.
---------------------------------------------------------------------------------------------------
*/
void NeopixelTransmitControl::transmitNoTask(void) {
    sentNrChunks = 0;

    //-------------------------------------------
    //  Enable the channel,
    //  this will start sending to the Neopixels
    //-------------------------------------------
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

    if (!waitForNotification(EVT_SENT)) {
        ESP_LOGE(TAG, "Timeout waiting until all DMA Chunks have been sent");
        // Continue anyway, even if timeout occurred
    }

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
}
#endif
