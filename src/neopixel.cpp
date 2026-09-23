/*
***************************************************************************************************
    ESP32xx Neopixel Driver

    Copyright (c) 2026 Erik Basalt
    Released under the MIT License, see the LICENSE file for details.
***************************************************************************************************
*/
#include <esp_system.h>
#include <esp_log.h>

#include "neopixel.h"
#include "neopixel_seq3.h"
#include "neopixel_seq4.h"

#define TAG "NPIX"

// Explicit template instantiation for all supported PixelType variants
// (see "enum class PixelType" in .h file for definition of PixelType)
template class NeopixelDriver<PixelType::GRB_SEQ3>;
template class NeopixelDriver<PixelType::GRBW_SEQ3>;
template class NeopixelDriver<PixelType::GRBW_SEQ4>;

/*
===================================================================================================
    Init the driver
===================================================================================================
*/
template <PixelType Mode>
bool NeopixelDriver<Mode>::begin(const size_t arg_nrPixels, const gpio_num_t dataPin) {
    uint32_t bitRate;

    if (arg_nrPixels == 0) {
        ESP_LOGE(TAG, "Number of pixels must be greater than zero");
        return (false);
    }

    if constexpr (Mode == PixelType::GRB_SEQ3) {
        //---------------------------------------
        //  GRB, seq3 timing
        //---------------------------------------
        ESP_LOGI(TAG, "GRB Neopixels, seq3 timing");
        txBytesPerColor = NEOPIXEL_SEQ3_BYTES_PER_COLOR; // seq3 encoding uses 3 bits per color bit, so 3 bytes per R/G/B color component
        txBytesPerPixel = txBytesPerColor * 3;           // 3 color components (R, G, B), 9 bytes in total
        bitRate = (800000UL * txBytesPerColor);          // Neopixel at 800kHz * 3 bits = 2.4 Mbps (417 ns/bit)
    } else if constexpr (Mode == PixelType::GRBW_SEQ3) {
        //---------------------------------------
        //  GRBW, seq3 timing
        //---------------------------------------
        ESP_LOGI(TAG, "GRBW Neopixels, seq3 timing");
        txBytesPerColor = NEOPIXEL_SEQ3_BYTES_PER_COLOR; // seq3 encoding uses 3 bits per color bit, so 3 bytes per R/G/B/W color component
        txBytesPerPixel = txBytesPerColor * 4;           // 4 color components (R, G, B, W), 12 bytes in total
        bitRate = (800000UL * txBytesPerColor);          // Neopixel at 800kHz * 3 bits = 2.4 Mbps (417 ns/bit)
    } else if constexpr (Mode == PixelType::GRBW_SEQ4) {
        //---------------------------------------
        //  GRBW, seq4 timing
        //---------------------------------------
        ESP_LOGI(TAG, "GRBW Neopixels, seq4 timing");
        txBytesPerColor = NEOPIXEL_SEQ4_BYTES_PER_COLOR; // seq4 encoding uses 4 bits per color bit, so 4 bytes per R/G/B color component
        txBytesPerPixel = txBytesPerColor * 4;           // 4 color components (G, R, B, W), 16 bytes in total
        bitRate = (800000UL * txBytesPerColor);          // Neopixel at 800kHz * 4 bits = 3.2 Mbps (312.5 ns/bit)
    } else {
        ESP_LOGE(TAG, "Unknown pixel type=%d", static_cast<int>(Mode));
        return (false);
    }

    //---------------------------------------
    //  Init the transmit control class
    //---------------------------------------
    if (!txControl.init(dataPin, bitRate, arg_nrPixels, txBytesPerPixel, &bufferSize)) {
        ESP_LOGE(TAG, "Failed to initialize TX control task");
        return (false);
    }

    //---------------------------------------
    //  Allocate the data buffer
    //  for the Neopixel transmission
    //---------------------------------------
    buffer = (uint8_t *)malloc(bufferSize);
    if (buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate buffer of size %d bytes", bufferSize);
        txControl.deinit(); // cleanup the task
        return (false);
    }
    memset(buffer, 0, bufferSize); // esp. to ensure the unused bytes in last frame are zeroed

    // Only now store the nrPixels
    // (when it remains 0, it means begin() was not called successfully)
    nrPixels = arg_nrPixels;
    return (true);
}

/*
===================================================================================================
    Generic function to set the pulse transmit sequence of one single pixel in the buffer
===================================================================================================
*/
template <PixelType Mode>
void NeopixelDriver<Mode>::setPixel(const size_t index, PixelColor pixel) {
    if (index >= nrPixels) {
        return; // silently ignore
    }

    if (brightness != 255) {
        //---------------------------------------
        //  Apply global brighness
        //---------------------------------------
        if (pixel.color.r) pixel.color.r = (pixel.color.r * brightness) >> 8;
        if (pixel.color.g) pixel.color.g = (pixel.color.g * brightness) >> 8;
        if (pixel.color.b) pixel.color.b = (pixel.color.b * brightness) >> 8;
        if constexpr ((Mode == PixelType::GRBW_SEQ3) || (Mode == PixelType::GRBW_SEQ4)) {
            // Only when the Neopixels actually have a white component
            if (pixel.color.w) pixel.color.w = (pixel.color.w * brightness) >> 8;
        }
    } // else: max brightness, no adjustment

    if constexpr (Mode == PixelType::GRB_SEQ3) {
        //---------------------------------------
        //  Set one GRB pixel, seq3 timing
        //---------------------------------------
        size_t offset = index * txBytesPerPixel;

        const uint8_t *sequence = neopixel_seq3_color_map[pixel.color.g];
        for (int i = 0; i < txBytesPerPixel; i++, offset++) { //@@@TODO: consider replacing for-loop with written-out statements
            if (i == 3)
                sequence = neopixel_seq3_color_map[pixel.color.r];
            if (i == 6)
                sequence = neopixel_seq3_color_map[pixel.color.b];
#if (NEOPIXEL_ENABLE_BIG_ENDIAN)
            buffer[offset] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // using Big-endian, no need to swap bytes
#else
            // buffer[offset] = __builtin_bswap32(sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]); // fill buffer in 16-bit Little-endian format
            buffer[offset ^ 1] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // fill buffer in 16-bit Little-endian format
#endif
        }
    } else if constexpr (Mode == PixelType::GRBW_SEQ3) {
        //---------------------------------------
        //  Set one GRBW pixel, seq3 timing
        //---------------------------------------
        size_t offset = index * txBytesPerPixel;

        const uint8_t *sequence = neopixel_seq3_color_map[pixel.color.g];
        for (int i = 0; i < txBytesPerPixel; i++, offset++) {
            if (i == 3)
                sequence = neopixel_seq3_color_map[pixel.color.r];
            if (i == 6)
                sequence = neopixel_seq3_color_map[pixel.color.b];
            if (i == 9)
                sequence = neopixel_seq3_color_map[pixel.color.w];
#if (NEOPIXEL_ENABLE_BIG_ENDIAN)
            buffer[offset] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // using Big-endian, no need to swap bytes
#else
            // buffer[offset] = __builtin_bswap32(sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]); // fill buffer in 16-bit Little-endian format
            buffer[offset ^ 1] = sequence[i % NEOPIXEL_SEQ3_BYTES_PER_COLOR]; // fill buffer in 16-bit Little-endian format
#endif
        }
    } else if constexpr (Mode == PixelType::GRBW_SEQ4) {
        //---------------------------------------
        //  Set one GRBW pixel, seq4 timing
        //---------------------------------------
        auto set_seq4_color = [](uint8_t colorByte, uint8_t *buffer) { // lambda to set one color byte in seq4 format
            const uint8_t *hi = encode_seq4_nibble[colorByte >> 4];
            const uint8_t *lo = encode_seq4_nibble[colorByte & 0x0F];
            buffer[0] = hi[0];
            buffer[1] = hi[1];
            buffer[2] = lo[0];
            buffer[3] = lo[1];
        };

        size_t offset = index * txBytesPerPixel;
        set_seq4_color(pixel.color.g, &buffer[offset]);
        set_seq4_color(pixel.color.r, &buffer[offset + NEOPIXEL_SEQ4_BYTES_PER_COLOR]);
        set_seq4_color(pixel.color.b, &buffer[offset + (2 * NEOPIXEL_SEQ4_BYTES_PER_COLOR)]);
        set_seq4_color(pixel.color.w, &buffer[offset + (3 * NEOPIXEL_SEQ4_BYTES_PER_COLOR)]);
    } // else: unknown pixel type, should not occur
}
