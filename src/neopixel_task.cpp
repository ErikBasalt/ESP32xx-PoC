#include <driver/i2s_std.h>
#include <driver/i2s_common.h>
#include "neopixel_task.h"

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE)
#include "hal.h"
#endif

#define TAG "PIXT"

/*
---------------------------------------------------------------------------------------------------
    Init

    Using init() i/o constructor, to only start it when the required arguments are known
---------------------------------------------------------------------------------------------------
*/
void NeopixelTransmitControl::init(TaskHandle_t arg_parent, i2s_chan_handle_t arg_i2s, int arg_nrChunks) {
    parentTaskHandle = arg_parent; // parent task, to send responses to
    i2s = arg_i2s;                 // I2S channel to use    //@@@TODO: not nice, also stored in class NeopixelDriver
    totalNrChunks = arg_nrChunks;  // total number of DMA chunks (descriptors) to send at each transmission

    xTaskCreatePinnedToCore(
        transmitTask,        // Task function
        "transmitTask",      // Task name
        10000,               // stack size (bytes)     @@@@TODO: reduce stack (current usage is only 404 bytes) 31dec24: min=1644 !? door logging tijdens skippen van een puls?
        this,                // Task parameter: reference to this class instance
        2,                   // priority is just above the normal program   //@@@TODO
        &transmitTaskHandle, // Task handle
        1);                  // which core to run on (0=system, 1=app)  //@@@TODO: why pinned to core?

    //@@@TODO check on pdPASS return value?
}

/*
---------------------------------------------------------------------------------------------------
    Task to start sending the preloaded data and wait for completion

    This Task only knows the number of preloaded DMA chunks to be sent, nothing about the contents
    Will also take care for enable/disable the I2S channel
---------------------------------------------------------------------------------------------------
*/
void NeopixelTransmitControl::transmitTask(void *taskArg) {
    NeopixelTransmitControl *here = (NeopixelTransmitControl *)taskArg; // pointer to class instance that started this task
    here->minimumFreeStack = uxTaskGetStackHighWaterMark(NULL);         // init Task stack usage

    // TASKLOG("Starting i2sTask"); // NOTE: this will NOT work when logging at main task is not active yet

    //---------------------------------------------------
    //  Lambda function to shutdown this Task
    //---------------------------------------------------
    auto shutdownTask = [&](void) {
        // Do any other cleanup owned by this task here

        // Tell main task that we are completely finished
        xTaskNotify(here->parentTaskHandle,
                    RSP_STOPPED,
                    eSetBits);

        here->transmitTaskHandle = nullptr; // forget this task
        vTaskDelete(nullptr);               // kill this task
        // No code should be executed after this
    };

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
                shutdownTask();
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
                here->nrOverruns++;
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
                shutdownTask();
            } else {
                // I2S Driver is Ready for new transmit command
                isTxRunning = false;

                // Notify parent
                xTaskNotify(here->parentTaskHandle,
                            RSP_READY,
                            eSetBits);
            }
        }
    }
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
    BaseType_t high_task_woken = pdFALSE;

    c->sentNrChunks++;
    if (c->sentNrChunks == c->totalNrChunks) {
        // All Neopixel DMA chunks (incl dummy flush) have been sent, signal the waiting task it can continue now

        // Npotofy the transmit Task
        xTaskNotifyFromISR(
            c->transmitTaskHandle,
            EVT_SENT,
            eSetBits,
            &high_task_woken);
    }
    return (high_task_woken == pdTRUE);
}
