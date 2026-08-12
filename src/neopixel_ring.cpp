// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "neopixel.h"
#include "neopixel_ring.h"

// Erik
#include "hal.h"

#define TAG "RING"
#define PIXEL_COUNT 84 // 2 rings in series, 60+24

#define I2S_TIMEOUT_TICKS 1000

static const PixelColor black = {.bytes = {0, 0, 0, 0}};
static const PixelColor cyan = {.bytes = {0x28, 0x28, 0, 0}};

static const PixelColor blue = {.bytes = {0x28, 0, 0, 0}}; // BGR(W), .bytesBGRW = ...
static const PixelColor green = {.bytes = {0, 0x28, 0, 0}};
static const PixelColor red = {.bytes = {0, 0, 0x28, 0}};

static const PixelColor red32 = {.value = 0x280000}; // (W)RGB, .valueWRGB = ...
static const PixelColor green32 = {.value = 0x002800};
static const PixelColor blue32 = {.value = 0x000028};
static const PixelColor cyan32 = {.value = 0x002828};
static const PixelColor yellow32 = {.value = 0x282800};

static tNeopixelContext npxContext = nullptr;

static unsigned int minIntervalMillis = 0; // minimum time [ms] between ring updates

static void allBlack(tNeopixelContext ctx) {
    tNpContext *c = (tNpContext *)ctx;

    for (int i = 0; i < PIXEL_COUNT; i++) {
        neopixel_SetColor(c, i, black);
    }
    neopixel_Show(c); // send the data to the Neopixel ring
}

bool startNeopixelRing(void) {
    gpio_num_t dataPin = hal.get_neopixel_data_pin();

    if (dataPin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Neopixel data pin is not configured");
        return (false);
    }

    ESP_LOGI(TAG, "Initializing NeoPixel ring on pin=%d with %d pixels", dataPin, PIXEL_COUNT);
    npxContext = neopixel_Initialize(PIXEL_COUNT, dataPin, NEOPIXEL_MODE_WS2812B);
    if (npxContext == nullptr) {
        ESP_LOGE(TAG, "Failed to initialize NeoPixel ring");
        return (false);
    }

    uint32_t refreshRate = neopixel_GetRefreshRate(npxContext);
    if (refreshRate != 0) {
        minIntervalMillis = (1000 / neopixel_GetRefreshRate(npxContext)) + 1; // minimum time [ms] between ring updates (+1 for integer rounding)
        minIntervalMillis++;                                                  // add 1 ms for overhead (RTOS task scheduling, enable/disable I2S channel, etc)
    }

    if (minIntervalMillis < 3) {
        minIntervalMillis = 3; // absolute minimum time [ms] between ring updates, to avoid overrun of the I2S channel
    }
    ESP_LOGI(TAG, "Neopixel ring minimum update interval=%d ms", minIntervalMillis);

    hal.setNeoPixelEnable(true); // enable the data output
    delay(10);                   // allow time for the 74HCT126 to enable the output
    allBlack(npxContext);        // ring may have spurious colors on power-up, make them all black
    return (true);
}

void allBlackNeopixelRing(void) { // for console command
    allBlack(npxContext);
}

void loopNeopixelRing(unsigned long currentMillis) {
    static int coloredIndex = 0;
    static int blackIndex = PIXEL_COUNT - 1;
    static int loopStartMillis = 0;
    static int maxMillisPerLoop = 0;

#if (0 == 1)
    // Throttle the ring updates to allow time for the previous neopixel_SetPixel() to complete before sending new data
    static unsigned long timeoutMillis = 0;
    if ((long)(currentMillis - timeoutMillis) < 0) {
        return; // allow time for the previous neopixel_SetPixel() to complete before sending new data
    }
    timeoutMillis = currentMillis + minIntervalMillis; // allow time to send new data to the Neopixel LEDs
#endif

    if (npxContext == nullptr) {
        // No Neopixel ring, silently ignore
        return;
    }

    neopixel_SetColor(npxContext, blackIndex, black); // erase previously colored pixel
    neopixel_SetColor(npxContext, coloredIndex, red); // set new colored pixel
    if (neopixel_Show(npxContext)) {                  // send the data to the Neopixel ring
        // Update the pixel indexes for the next iteration
        blackIndex = coloredIndex;
        if (++coloredIndex >= PIXEL_COUNT) {
            // New loop
            coloredIndex = 0;

            // Ring loop speed
            if (loopStartMillis != 0) {
                int loopMillis = currentMillis - loopStartMillis;
                if (loopMillis > maxMillisPerLoop) {
                    maxMillisPerLoop = loopMillis;
                    ESP_LOGI(TAG, "Max millis per loop = %d", maxMillisPerLoop);
                }
            }
            loopStartMillis = currentMillis;

            // Used chunks
            tNpContext *c = (tNpContext *)npxContext;
            static int reportedMaxChunksSent = 0;
            if (c->stats.maxChunksSent > reportedMaxChunksSent) {
                reportedMaxChunksSent = c->stats.maxChunksSent;
                ESP_LOGI(TAG, "maxChunksSent=%d", reportedMaxChunksSent);
            }

            // Transmit overflows - Currently, on_send_q_ovf callback is not used in neopixel.cpp
            static int reportedMaxOverflowCount = 0;
            if (c->stats.overflowCount > reportedMaxOverflowCount) {
                reportedMaxOverflowCount = c->stats.overflowCount;
                ESP_LOGW(TAG, "overflowCount=%d", reportedMaxOverflowCount);
            }

            // Task overruns
            static int reportedMaxTaskOverrunCount = 0;
            if (c->stats.taskOverrunCount > reportedMaxTaskOverrunCount) {
                reportedMaxTaskOverrunCount = c->stats.taskOverrunCount;
                ESP_LOGW(TAG, "taskOverrunCount=%d", reportedMaxTaskOverrunCount);
            }
        }
    } // else: busy, try again later
}
