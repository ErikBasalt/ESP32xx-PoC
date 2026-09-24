#include <esp_log.h>
#include <driver/gpio.h>

#include "hal.h"
#include "neopixel.h"
#include "neopixel_ring.h"

#define TAG "RING"

#define NEOPIXEL_ENABLE_SECOND_RING 0 // to enable 2nd ring, also disable the Dotmatrix in main.cpp

#if (1 == 1)
//-----------------
//  9x RGBW ring
//-----------------
// NeopixelDriver<PixelType::SK6812B_RGBW> npx;
NeopixelDriver<PixelType::GRBW_SEQ4> npx;
//  NeopixelDriver<static_cast<PixelType>(666)> npx;

inline constexpr PixelColor neopixelBackgroundColor = {.color = {.b = 0x03, .g = 0, .r = 0, .w = 0}}; // dimmed Blue
                                                                                                      // inline constexpr PixelColor neopixelColored = {.color = {.b = 0, .g = 0, .r = 0, .w = 0x03}};         // dimmed White
inline constexpr PixelColor neopixelColored = {.color = {.b = 0x30, .g = 0x20, .r = 0, .w = 0}};      // fairly brigh cyan, should be dimmed

// #define PIXEL_COUNT (1 + 8 + 12 + 16 + 24 + 32 + 40 + 48 + 60) // 1 assembly 9 rings in total
#define PIXEL_COUNT (1 + 8 + 12 + 16 + 24 + 32 + 40 + 48 + 60 - 1) // test: 1 pixel less
// #define PIXEL_COUNT (1)
#else
//-----------------
//  3x RGB ring
//-----------------
NeopixelDriver<PixelType::GRB_SEQ4> npx;
inline constexpr PixelColor neopixelBackgroundColor = {.color = {.b = 0x03, .g = 0, .r = 0, .w = 0}}; // dimmed Blue
inline constexpr PixelColor neopixelColored = {.color = {.b = 0, .g = 0, .r = 0x10, .w = 0}};         // dimmed Red

// #define PIXEL_COUNT (60 + 24 + 1 + 8 + 12 + 16 + 24 + 32) // 1 ring of 60, 1 ring of 24, 1 assembly of 6 rings
#define PIXEL_COUNT (60 + 24 + 1 + 8 + 12 + 16 + 24 + 32 - 1) // test: 1 pixel less
// #define PIXEL_COUNT (60 + 24 + 1 + 8 + 12 + 16 + 24 + 32 + 1) // test: 1 pixel more
// #define PIXEL_COUNT 61
#endif

void allBlackNeopixelRing(void) {              // for console
    npx.setAllPixels(neopixelBackgroundColor); //@@@TODO: change to neopixelBlack
    npx.show();
}

#if (NEOPIXEL_ENABLE_SECOND_RING)
/*
===================================================================================================
    Second NeoPixel ring
===================================================================================================
*/
NeopixelDriver<PixelType::GRB_SEQ3> npx2;

inline constexpr PixelColor neopixelBackgroundColor2 = {.color = {.b = 0x10, .g = 0, .r = 0, .w = 0}}; // dimmed Blue
inline constexpr PixelColor neopixelColored2 = {.color = {.b = 0, .g = 0, .r = 0x20, .w = 0}};         // fairly bright Red, should be dimmed

// #define PIXEL2_COUNT (60 + 24 + 1 + 8 + 12 + 16 + 24 + 32) // 1 ring of 60, 1 ring of 24, 1 assembly of 6 rings
#define PIXEL2_COUNT (60 + 24 + 1 + 8 + 12 + 16 + 24 + 32 - 1) // test: 1 pixel less
// #define PIXEL2_COUNT (60 + 24 + 1 + 8 + 12 + 16 + 24 + 32 + 1) // test: 1 pixel more
// #define PIXEL2_COUNT 61

bool startNeopixelRing2(void) {
    gpio_num_t dataPin = hal.get_spi_MOSI_pin();

    if (dataPin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Neopixel data pin is not configured");
        return (false);
    }

    ESP_LOGI(TAG, "Initializing second NeoPixel ring on pin=%d with %d pixels", dataPin, PIXEL2_COUNT);
    npx2.begin(PIXEL2_COUNT, dataPin);
    //@@@TODO: error handling

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE == 0) //@@@TODO: do not enable/disable for every write, will give CONFLICT between ring1 and ring2
    hal.setNeoPixelEnable(true);              // enable the data output
#endif

    npx2.setAllPixels(neopixelBlack); // set all pixels to black
    npx2.show();                      // send the data to the Neopixel ring
    return (true);
}
#endif

bool startNeopixelRing(void) {
    gpio_num_t dataPin = hal.get_neopixel_data_pin();

    if (dataPin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Neopixel data pin is not configured");
        return (false);
    }

    ESP_LOGI(TAG, "Initializing NeoPixel ring on pin=%d with %d pixels", dataPin, PIXEL_COUNT);
    npx.begin(PIXEL_COUNT, dataPin);
    //@@@TODO: error handling

#if (NEOPIXEL_ENABLE_OUTPUT_EVERY_WRITE == 0)
    hal.setNeoPixelEnable(true); // enable the data output
#endif

    npx.setAllPixels(neopixelBlack); // set all pixels to black
    npx.show();                      // send the data to the Neopixel ring
#if (NEOPIXEL_ENABLE_SECOND_RING)
    startNeopixelRing2();
#endif
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

#if (ENABLE_I2S_TASK_VERSION)
    // Used Task stack
    static UBaseType_t reportedMinFreeTaskStack = UINT_MAX;
    if (npx.stats.minimumFreeStack < reportedMinFreeTaskStack) {
        reportedMinFreeTaskStack = npx.stats.minimumFreeStack;
        ESP_LOGI(TAG, "minimumFreeStack=%u", reportedMinFreeTaskStack);
    }
#endif
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

    npx.setPixel(blackIndex, neopixelBlack);     // erase previously colored pixel
    npx.setPixel(coloredIndex, neopixelColored); // set new colored pixel
    npx.show();

#if (NEOPIXEL_ENABLE_SECOND_RING)
    npx2.setPixel(blackIndex / 2, neopixelBlack);      // erase previously colored pixel
    npx2.setPixel(coloredIndex / 2, neopixelColored2); // set new colored pixel for the second ring
    npx2.show();
#endif

    // Update the pixel indexes for the next iteration
    blackIndex = coloredIndex;
    if (++coloredIndex >= PIXEL_COUNT) {
        // New loop
        coloredIndex = 0;

        statistics(currentMillis);
    }
}

uint16_t readPoti(void) {
    static uint32_t avg = 0; // 32 bit integer is easily large enough to max 3300 (12 bits) + 4x shift (avg << 4)

    avg = ((avg << 4) - avg + hal.readAnalog()) >> 4; // 0...3300 (mV)

    return (avg);
}

/*
=========================================================================================
    Loop
=========================================================================================
*/
void loopNeopixelRing(unsigned long currentMillis) {
    //---------------------------------------------------------------
    //  Throttle the ring updates for better visibility (if needed)
    //---------------------------------------------------------------
    static unsigned long timeoutMillis = 0;
    if ((long)(currentMillis - timeoutMillis) < 0) {
        return;
    }
    timeoutMillis = currentMillis + 0; // update the timeout for the next iteration

    //---------------------------------------------------------------
    //  Adjust brightness based on potentiometer reading
    //---------------------------------------------------------------
    static unsigned long readPotiTimeoutMillis = 0;

    if ((long)(currentMillis - readPotiTimeoutMillis) >= 0) {
        static uint16_t potiValue = 0;
        uint16_t newPotiValue = readPoti();
        if (newPotiValue != potiValue) {
            npx.brightness = ((unsigned)potiValue * 255) / 3300; // scale 0...3300 to 0...64
#if (NEOPIXEL_ENABLE_SECOND_RING)
            npx2.brightness = npx.brightness;
#endif
            // ESP_LOGI("Neopixel", "Poti=%u, brightness=%u", potiValue, npx.brightness);
            potiValue = newPotiValue;
        }
        readPotiTimeoutMillis = currentMillis + 10;
    }

    //---------------------------------------------------------------
    //  Animate the Neopixel ring
    //---------------------------------------------------------------
    if (PIXEL_COUNT == 1) {
        animateSinglePixel(currentMillis);
    } else {
        movingPixel(currentMillis);
    }
}
