#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "neopixel.h"

// Erik
#include "hal.h"

#define TAG "NPIX"
#define PIXEL_COUNT 84 // 2 rings in series, 60+24

static tNeopixelContext npxContext = nullptr;

static unsigned int minIntervalMillis = 0; // minimum time [ms] between ring updates

#if (0 == 1)
tNeopixel redPixel = {0, 0x280000};   // red pixel
tNeopixel greenPixel = {0, 0x002800}; // green pixel
tNeopixel bluePixel = {0, 0x000028};  // blue pixel
#endif

void eraseNeopixelRing(void) {
    tNeopixel pixel[PIXEL_COUNT] = {}; // all black pixels
    if (npxContext != nullptr) {
        for (int i = 0; i < PIXEL_COUNT; i++) {
            pixel[i].index = i; // set index for each pixel
        }
        neopixel_SetPixel(npxContext, &pixel[0], PIXEL_COUNT); // all black
    }
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

    // neopixel_SetPixel(npxContext, &blackPixel, PIXEL_COUNT); // all black
    uint32_t refreshRate = neopixel_GetRefreshRate(npxContext);
    if (refreshRate != 0) {
        minIntervalMillis = (1000 / neopixel_GetRefreshRate(npxContext)) + 1; // minimum time [ms] between ring updates (+1 for integer rounding)
    }

    if (minIntervalMillis < 3) {
        minIntervalMillis = 3; // to allow for RTOS task scheduling
    }
    ESP_LOGI(TAG, "Neopixel ring minimum update interval=%d ms", minIntervalMillis);

    hal.setNeoPixelEnable(true); // enable the data output
    delay(10);                   // allow time for the 74HCT126 to enable the output
    eraseNeopixelRing();         // erase possible old pixels
    return (true);
}

void loopNeopixelRing(unsigned long currentMillis) {
    static unsigned long timeoutMillis = 0;
    static int redIndex = 0;
    static int blackIndex = PIXEL_COUNT - 1;
    tNeopixel pixel[2]; // for Red and Black pixel

    if ((long)(currentMillis - timeoutMillis) < 0) {
        return; // allow time for the previous neopixel_SetPixel() to complete before sending new data
    }

    if (npxContext == nullptr) {
        // No Neopixel ring, silently ignore
        return;
    }

    // Set the current pixel to red, previous to black
    pixel[0].index = blackIndex;
    pixel[0].rgb = 0x000000; // Black pixel
    pixel[1].index = redIndex;
    pixel[1].rgb = 0x280000; // Red pixel, moderate brightness
    neopixel_SetPixel(npxContext, &pixel[0], 2);

    // Update the pixel indexes for the next iteration
    blackIndex = redIndex;
    if (++redIndex >= PIXEL_COUNT) {
        redIndex = 0;
    }
    timeoutMillis = currentMillis + minIntervalMillis; // allow time to send new data to the Neopixel LEDs
}
