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

// Enable or disable using a separate RTOS task to wait for I2S transmission completion
#define ENABLE_I2S_TASK_VERSION 1

#define NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE 1

#if (SOC_I2S_HW_VERSION_1)
// NOTE: !! VSC is not aware of this define, therefore syntax highlighting does NOT work here !!

//-------------------------------------------------------------------
// SOC_I2S_HW_VERSION_1 chip (the original ESP32 and ESP32-S2)
//-------------------------------------------------------------------

// There is no simple Big-Endian setting in the I2S configuration.
// The hardware architecture inherently expects data to be preloaded/written in Little-Endian format.
// The software (here) must do the necessary byte swapping for it..
#define NEOPIXEL_USE_BIG_ENDIAN_DATA 0
#else
//-------------------------------------------------------------------
// SOC_I2S_HW_VERSION_2 chip (ESP32-S3 and later, incl C3 and C6)
//-------------------------------------------------------------------
// I2S hardware can handle Big-Endian mode directly, just set the I2S configuration flag accordingly.
// The software (here) can then simply write the bytes linearly (faster than with byte swapping).
#define NEOPIXEL_USE_BIG_ENDIAN_DATA 1
#endif
/*
---------------------------------------------------------------------------------------------------
    Interface to Task that controls the transmission of Neopixel data via I2S
---------------------------------------------------------------------------------------------------
 */
struct NeopixelTransmitStatistics {
#if (ENABLE_I2S_TASK_VERSION)
    UBaseType_t minimumFreeStack = 0; // minimum free stack [bytes] of this Task
    uint32_t nrOverruns;              // number of times transmit command is given while Task is not Ready yet
#endif
    int64_t maxSendMicros;
    uint32_t maxNrChunksSent;
    //@@@TODO: add some error counters (e.g., for DMA transfer failures)
};

class NeopixelTransmitControl {
  private:
    // For the Notifications
    TaskHandle_t parentTaskHandle = nullptr; // RTOS handle to the parent task, for sending response Notifications to
#if (ENABLE_I2S_TASK_VERSION)
    TaskHandle_t transmitTaskHandle = nullptr; // RTOS handle to trasnmit Task, for sending command Notifications to
#endif

    // I2S channel to use
    i2s_chan_handle_t i2s = nullptr;

    // Housekeeping
    int totalNrChunks; // total number of DMA chunks (descriptors) to send at each transmission
    int sentNrChunks;  // actual number of chunks (being) sent, used for tracking the transmit progress

    // Statistics, stored externally
    struct NeopixelTransmitStatistics *statsPtr = nullptr;

    // Functions
#if (ENABLE_I2S_TASK_VERSION)
    static void transmitTask(void *here); // in cpp, function for the RTOS Task, needs to be static and have pointer to the instance
#else
    void transmitNoTask(void);
#endif
    static IRAM_ATTR bool onSentCallback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *classContext); // in cpp, function for the sent callback, needs to be static

  public:
    NeopixelTransmitControl(NeopixelTransmitStatistics *arg_statsPtr) : statsPtr(arg_statsPtr) {
    }

    ~NeopixelTransmitControl(void) {
        deinit();
    }

    virtual bool init(gpio_num_t dataPin, uint32_t bitRate, size_t rawDataSize, size_t *requiredBufferSizePtr);
    virtual void deinit(void);
    virtual void startTransmit(uint8_t *buffer, size_t bufferSize);
};
