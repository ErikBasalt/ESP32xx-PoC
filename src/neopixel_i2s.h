#pragma once
/*
***************************************************************************************************
    ESP32xx Neopixel Driver - I2S implementation

    Copyright (c) 2026 Erik Basalt
    Released under the MIT License, see the LICENSE file for details.
***************************************************************************************************
*/
#include <freertos/FreeRTOS.h>
#include <driver/i2s_std.h>
#include <driver/i2s_common.h>

#define NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE 1

/*
---------------------------------------------------------------------------------------------------
    Interface to Task that controls the transmission of Neopixel data via I2S
---------------------------------------------------------------------------------------------------
 */
#define NEOPIXEL_TASK_PRIORITY (configMAX_PRIORITIES - 1) //@@@TODO: check

class NeopixelTransmitControl {
  private:
    // Neopixel config
    // size_t txBytesPerColor; // number of bytes to be sent per R/G/B/(W) color component, depends on seq3/seq4 timing
    // size_t txBytesPerPixel; // number of bytes per Neopixel (all colors)

    // Data size
    // size_t nrPixels = 0; // number of Neopixels to drive
    // uint8_t *buffer = nullptr; // data buffer to be sent to the Neopixels
    // size_t bufferSize = 0; // [bytes]

    // Notifications
    TaskHandle_t parentTaskHandle = xTaskGetCurrentTaskHandle(); // RTOS handle to the parent task, used by Task to send responses
    TaskHandle_t transmitTaskHandle = nullptr;                   // RTOS handle to newly created Task itself, used by Parent to send commands

    // I2S channel to use
    i2s_chan_handle_t i2s = nullptr;

    // Housekeeping
    UBaseType_t minimumFreeStack = 0; // minimum free stack [bytes] of this Task

    int totalNrChunks; // total number of DMA chunks (descriptors) to send at each transmission
    int sentNrChunks;  // actual number of chunks (being) sent, used for tracking the transmit progress

    // Function declarations
    static void transmitTask(void *here);                                                                        // in cpp, function for the RTOS Task, needs to be static and have pointer to the instance
    static IRAM_ATTR bool onSentCallback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *classContext); // in cpp, function for the sent callback, needs to be static

  public:
    struct NeopixelStatistics {
        int64_t maxSendMicros;
        uint32_t maxNrChunksSent;
        uint32_t nrOverruns = 0; // number of times transmit command is given while not Ready yet, use waitUntilReady() to avoid overruns
        //@@@TODO: add some error counters (e.g., for DMA transfer failures)
    } stats = {};

    NeopixelTransmitControl(void) = default;

    ~NeopixelTransmitControl(void) {
        deinit();
    }

    virtual bool init(gpio_num_t dataPin, uint32_t bitRate, size_t arg_nrPixels, size_t txBytesPerPixel, size_t *requiredBufferSizePtr);
    virtual void deinit(void);
    virtual void startTransmit(uint8_t *buffer, size_t bufferSize);
};
