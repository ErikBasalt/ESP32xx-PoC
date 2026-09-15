#include <esp_log.h>
#include <driver/gpio.h>

#include "hal.h"
#include "neopixel.h"
#include "neopixel_ring.h"

#define TAG "RING"

#define NEOPIXEL_RGBW 1
#if (NEOPIXEL_RGBW)
// #define PIXEL_COUNT (1 + 8 + 12 + 16 + 24 + 32 + 40 + 48 + 60) // 1 assembly 9 rings in total
#define PIXEL_COUNT (8 + 12 + 16 + 24 + 32 + 40 + 48 + 60) // test: 1 pixel less
// #define PIXEL_COUNT 1
#else
// #define PIXEL_COUNT (60 + 24 + 1 + 8 + 12 + 16 + 24 + 32) // 1 ring of 60, 1 ring of 24, 1 assembly of 6 rings
#define PIXEL_COUNT (60 + 24 + 8 + 12 + 16 + 24 + 32) // test: 1 pixel less
// #define PIXEL_COUNT 1
#endif

inline constexpr PixelColor neopixelDimmedWhite = {.color = {.b = 0, .g = 0, .r = 0, .w = 0x28}};
inline constexpr PixelColor neopixelDimmedRed = {.color = {.b = 0, .g = 0, .r = 0x10, .w = 0}};
inline constexpr PixelColor neopixelBackgroundColor = {.color = {.b = 0x10, .g = 0, .r = 0, .w = 0}};

#define I2S_TIMEOUT_TICKS 1000

NeopixelDriver npx;

void allBlackNeopixelRing(void) {              // for console
    npx.setAllPixels(neopixelBackgroundColor); //@@@TODO: change to neopixelBlack
    npx.show();
}

bool startNeopixelRing(void) {
    gpio_num_t dataPin = hal.get_neopixel_data_pin();

    if (dataPin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Neopixel data pin is not configured");
        return (false);
    }

    ESP_LOGI(TAG, "Initializing NeoPixel ring on pin=%d with %d pixels", dataPin, PIXEL_COUNT);
#if (NEOPIXEL_RGBW)
    npx.begin(PIXEL_COUNT, dataPin, NEOPIXEL_MODE_SK6812B);
#else
    npx.begin(PIXEL_COUNT, dataPin, NEOPIXEL_MODE_WS2812B);
#endif
    //@@@TODO: error handling

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE == 0)
    hal.setNeoPixelEnable(true); // enable the data output
#endif

    npx.setAllPixels(neopixelBlack); // set all pixels to black
    npx.show();                      // send the data to the Neopixel ring
    return (true);
}

void statistics(unsigned long currentMillis) {
    static int loopStartMillis = 0;
    static int maxMillisPerLoop = 0;
    // Ring loop speed
    if (loopStartMillis != 0) {
        int loopMillis = currentMillis - loopStartMillis;
        if (loopMillis > maxMillisPerLoop) {
            maxMillisPerLoop = loopMillis;
            ESP_LOGI(TAG, "Max millis per ring loop = %d", maxMillisPerLoop);
        }
    }
    loopStartMillis = currentMillis;

    // Used chunks
    static int reportedMaxChunksSent = 0;
    if (npx.stats.maxNrChunksSent > reportedMaxChunksSent) {
        reportedMaxChunksSent = npx.stats.maxNrChunksSent;
        ESP_LOGI(TAG, "maxNrChunksSent=%d", reportedMaxChunksSent);
    }
}

void animateSinglePixel(unsigned long currentMillis) {
    static PixelColor pixel = {value : 0};
    static int colorMode = 0;

    switch (colorMode) {
    case 0:
        if (pixel.color.r++ >= 64) {
            pixel.color.r = 0;
            colorMode = 1; // switch to the next color mode
        }
        break;
    case 1:
        if (pixel.color.g++ >= 64) {
            pixel.color.g = 0;
            colorMode = 2; // switch to the next color mode
        }
        break;
    default:
        if (pixel.color.b++ >= 64) {
            pixel.color.b = 0;
            colorMode = 0; // switch to the next color mode
            statistics(currentMillis);
        }
        break;
    }
    npx.setPixel(0, pixel);
    npx.show(); // send the data to the Neopixel ring
}

void movingPixel(unsigned long currentMillis) {
    static int coloredIndex = 0;
    static int blackIndex = PIXEL_COUNT - 1;

    npx.setPixel(blackIndex, neopixelBlack); // erase previously colored pixel
#if (NEOPIXEL_RGBW)
    npx.setPixel(coloredIndex, neopixelDimmedWhite); // set new colored pixel
#else
    npx.setPixel(coloredIndex, neopixelDimmedRed); // set new colored pixel
#endif

    if (npx.show()) { // send the data to the Neopixel ring
        // Update the pixel indexes for the next iteration
        blackIndex = coloredIndex;
        if (++coloredIndex >= PIXEL_COUNT) {
            // New loop
            coloredIndex = 0;

            statistics(currentMillis);
        } // else: busy, try again later
    }
}

void loopNeopixelRing(unsigned long currentMillis) {
    // Throttle the ring updates for better visibility (if needed)
    static unsigned long timeoutMillis = 0;
    if ((long)(currentMillis - timeoutMillis) < 0) {
        return;
    }
    timeoutMillis = currentMillis + 0; // "+ 0" is full speed

    if (PIXEL_COUNT == 1) {
        animateSinglePixel(currentMillis);
    } else {
        movingPixel(currentMillis);
    }
}
