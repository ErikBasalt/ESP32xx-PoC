#pragma once
#include <freertos/FreeRTOS.h>

#define NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE 1

/*
---------------------------------------------------------------------------------------------------
    Interface to Task that controls the transmission of Neopixel data via I2S
---------------------------------------------------------------------------------------------------
 */
#define NEOPIXEL_TASK_PRIORITY (configMAX_PRIORITIES - 1) //@@@TODO: check

//@@@TODO: use enums?
// Notification commands from Main (or callback) to Task
constexpr uint32_t CMD_TRANSMIT = 1 << 0; // command to transmit the preloaded data
constexpr uint32_t CMD_SHUTDOWN = 1 << 1; // command to shutdown the Task
constexpr uint32_t EVT_SENT = 1 << 2;     // Event (from sent callback) indicating that all data has been transmitted

// Notification responses from Task to Main
constexpr uint32_t RSP_READY = 1 << 0;   // I2S driver is in Ready state, new data can be preloaded and then transmitted
constexpr uint32_t RSP_STOPPED = 1 << 1; // Shutdown complete

class NeopixelTransmitControl {
  private:
    // Notifications
    TaskHandle_t parentTaskHandle = nullptr;   // RTOS handle to the parent task, used by Task to send responses
    TaskHandle_t transmitTaskHandle = nullptr; // RTOS handle to newly created Task itself, used by Parent to send commands

    // I2S channel to use
    i2s_chan_handle_t i2s = nullptr;

    // Housekeeping
    UBaseType_t minimumFreeStack = 0; // minimum free stack [bytes] of this Task

    int totalNrChunks; // total number of DMA chunks (descriptors) to send at each transmission
    int sentNrChunks;  // actual number of chunks (being) sent, used for tracking the transmit progress

    // Statistics only
    uint32_t maxNrChunksSent = 0; // max number of chunks that have been sent for one complete transmission, can be > totalNrChunks
    int nrOverruns = 0;           // number of times transmit command is given while not Ready yet, use waitUntilReady() to avoid overruns

    // Function declarations
    static void transmitTask(void *here);                                                                        // in cpp, function for the RTOS Task, needs to be static and have pointer to the instance
    static IRAM_ATTR bool onSentCallback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *classContext); // in cpp, function for the sent callback, needs to be static

  public:
    NeopixelTransmitControl() = default;

    ~NeopixelTransmitControl() {
        deinit();
    }

    //-------------------------------------------
    //  Init
    //
    //  Call to create and init the Task
    //-------------------------------------------
    virtual void init(TaskHandle_t arg_parent, i2s_chan_handle_t arg_i2s, int arg_nrChunks); // in cpp

    //-------------------------------------------
    //  Deinit
    //
    //  Call to shutdown the Task and cleanup
    //-------------------------------------------
    void deinit() {
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
                return;
            } // else: wait and ignore other responses (eg from previous transmit command)
        }

        // Task did not respond, kill it from here
        vTaskDelete(transmitTaskHandle);
    }

    //-------------------------------------------
    //  Wait until Task is ready for new transmit
    //
    //  Call before preloading new data
    //-------------------------------------------
    bool waitUntilReady() {
        uint32_t response;

        xTaskNotifyWait(
            0,                  // keep all bits on entry
            UINT32_MAX,         // clear all bits on exit
            &response,          // the response from Task
            pdMS_TO_TICKS(1000) // wait max 1000ms, should be enough for more than 10000 Neopixels
        );
        return (response & RSP_READY); // false when an unexpected response is received
        //@@@TODO: what if another response? restart waiting?
    }

    //-------------------------------------------
    //  Start Transmit
    //
    //  Call after preloading new data
    //-------------------------------------------
    void startTransmit() { // call after preloading
        xTaskNotify(transmitTaskHandle, CMD_TRANSMIT, eSetBits);
    }
};
