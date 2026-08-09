#pragma once
#include <Arduino.h>

class Hal {
  private:
    // GPIO pins
    gpio_num_t button_gpio = GPIO_NUM_NC; // external button input, active Low

    gpio_num_t statusLed_gpio = GPIO_NUM_NC; // status LED output; external when enough GPIO, otherwise onboard LED
    bool statusLed_activeLevel = HIGH;       // output level to activate (switch On) the status LED, depends on ESPxx board

    gpio_num_t analogInput_gpio = GPIO_NUM_NC; // Analog input

    gpio_num_t neopixel_enable_gpio = GPIO_NUM_NC; // NeoPixel enable output
    bool neopixel_enable_activeLevel = HIGH;       // output level to activate (switch On) the NeoPixel output, HIGH for 74HCT126
    gpio_num_t neopixel_data_gpio = GPIO_NUM_NC;   // NeoPixel data output

    gpio_num_t spi_SCK_gpio = GPIO_NUM_NC;  // SPI clock output
    gpio_num_t spi_MOSI_gpio = GPIO_NUM_NC; // SPI MOSI output
    gpio_num_t spi_CS_gpio = GPIO_NUM_NC;   // SPI Chip Select output

    gpio_num_t i2c_SCL_gpio = GPIO_NUM_NC; // I2C SCL output
    gpio_num_t i2c_SDA_gpio = GPIO_NUM_NC; // I2C SDA output

    bool setGpioPins(void); // in cpp file

    void initGpio(bool isRoundDisplay) {
        // Input button: active Low, so needs pullup
        if (button_gpio != GPIO_NUM_NC) {
            pinMode(button_gpio, INPUT_PULLUP);
        }

        // Status LED
        if (statusLed_gpio != GPIO_NUM_NC) {
            pinMode(statusLed_gpio, OUTPUT);
            setStatusLed(false); // default to inactive LED
        }

        // Analog input
        if (analogInput_gpio != GPIO_NUM_NC) {
            // No need to set pinMode() for analog input, because analogRead() will do that automatically
        }

        if (isRoundDisplay) {
            // The round LCD dispaly uses a libray driver with fixed #defined GPIO pins
            // so cannot use the HAL for that
        } else {
            // Neopixel
            if (neopixel_enable_gpio != GPIO_NUM_NC) {
                pinMode(neopixel_enable_gpio, OUTPUT);
                setNeoPixelEnable(false); // default to disabled
            }
#if (0 == 1)
            //@@@TODO: keep signal generator, or move to Neopixel only?
            if (neopixel_data_gpio != GPIO_NUM_NC) {
                pinMode(neopixel_data_gpio, OUTPUT);
                digitalWrite(neopixel_data_gpio, LOW); // default to LOW
            }
#endif
            // SPI
            if (spi_SCK_gpio != GPIO_NUM_NC) {
                pinMode(spi_SCK_gpio, OUTPUT);
                digitalWrite(spi_SCK_gpio, LOW); // default to LOW
            }
            if (spi_MOSI_gpio != GPIO_NUM_NC) {
                pinMode(spi_MOSI_gpio, OUTPUT);
                digitalWrite(spi_MOSI_gpio, LOW); // default to LOW
            }
            if (spi_CS_gpio != GPIO_NUM_NC) {
                pinMode(spi_CS_gpio, OUTPUT);
                digitalWrite(spi_CS_gpio, HIGH); // on SPI, CS is always active LOW, so default is HIGH
            }
        }
    }

  public:
    // Begin initializes the GPIO pins, based on the used ESP32xx model
    void begin(bool isRoundDisplay) {
        if (setGpioPins()) {
            initGpio(isRoundDisplay);
        }
    }

    // Button input function, returns true if button is pressed
    bool isButtonPressed(void) {
        if (button_gpio != GPIO_NUM_NC) {
            return (digitalRead(button_gpio) == LOW); // active LOW
        } else {
            return (false); // not configured, always false
        }
    }

    // Status LED control functions
    void setStatusLed(bool isActive) {
        if (statusLed_gpio != GPIO_NUM_NC) {
            digitalWrite(statusLed_gpio, isActive ? statusLed_activeLevel : !statusLed_activeLevel);
        }
    }

    void toggleStatusLed(void) {
        if (statusLed_gpio != GPIO_NUM_NC) {
            digitalWrite(statusLed_gpio, !digitalRead(statusLed_gpio));
        }
    }

    uint16_t readAnalog(void) {
        if (analogInput_gpio != GPIO_NUM_NC) {
            // On ESP32-C6 chip revision 0.1 there is a bug in the ADC, which limits the max 12-bit output to 3300 instead of 4095
            // Therefore use analogReadMilliVolts() to get the correct CALIBRATED voltage in mV, instead of the raw ADC value
            // https://github.com/espressif/arduino-esp32/issues/11324
            //
            // For consistency, do the same for the other ESP32xx chips
            // This also solves the issue with the 13-bit ADC on ESP32-S2 and ESP32-S3
            return (analogReadMilliVolts(analogInput_gpio)); // 0...(approx) 3300 mV
        } else {
            return (0); // not configured, always 0
        }
    }

    // NeoPixel control functions
    void setNeoPixelEnable(bool isActive) {
        if (neopixel_enable_gpio != GPIO_NUM_NC) {
            digitalWrite(neopixel_enable_gpio, isActive ? neopixel_enable_activeLevel : !neopixel_enable_activeLevel);
        }
    }

    void toggleNeoPixelEnable(void) {
        if (neopixel_enable_gpio != GPIO_NUM_NC) {
            digitalWrite(neopixel_enable_gpio, !digitalRead(neopixel_enable_gpio));
        }
    }

    void toggleNeoPixelData(void) {
        if (neopixel_data_gpio != GPIO_NUM_NC) {
            digitalWrite(neopixel_data_gpio, !digitalRead(neopixel_data_gpio));
        }
    }

    void setNeoPixelDataSignalGenerator(bool isGenerator) {
        if (isGenerator) {
            // Generate a 1 MHz signal for the PXD test
            // Actual Neopixel speed is 833 kHz (1200 ns pulse duration; dutycycle 25%=Off, 75%=On))
            if (neopixel_data_gpio != GPIO_NUM_NC) {
                ledcAttach(neopixel_data_gpio, /*freq=*/1000000, /*resolution=*/4); // 4 bits resolution
                ledcWrite(neopixel_data_gpio, /*dutycycle*/ 8);                     // 50% duty cycle
            }
        } else {
            // On/Off operation
            if (neopixel_data_gpio != GPIO_NUM_NC) {
                // Stop the signal generator, make it normal output again
                ledcDetach(neopixel_data_gpio);
                pinMode(neopixel_data_gpio, OUTPUT);
                // Default is Off
                digitalWrite(neopixel_data_gpio, LOW);
            }
        }
    }

    // SPI control functions
    void toggleSpiClock(void) {
        if (spi_SCK_gpio != GPIO_NUM_NC) {
            digitalWrite(spi_SCK_gpio, !digitalRead(spi_SCK_gpio));
        }
    }

    void toggleSpiMosi(void) {
        if (spi_MOSI_gpio != GPIO_NUM_NC) {
            digitalWrite(spi_MOSI_gpio, !digitalRead(spi_MOSI_gpio));
        }
    }

    void toggleSpiChipSelect(void) {
        if (spi_CS_gpio != GPIO_NUM_NC) {
            digitalWrite(spi_CS_gpio, !digitalRead(spi_CS_gpio));
        }
    }

    // Getters for I2C pin numbers, to allow init outside this HAL
    gpio_num_t get_i2c_SCL_gpio() { return i2c_SCL_gpio; }

    gpio_num_t get_i2c_SDA_gpio() { return i2c_SDA_gpio; }

    // Getters for SPI pin numbers, to allow init outside this HAL
    gpio_num_t get_spi_CS_pin() { return spi_CS_gpio; }

    gpio_num_t get_spi_SCK_pin() { return spi_SCK_gpio; }

    gpio_num_t get_spi_MOSI_pin() { return spi_MOSI_gpio; }

    // Getter for Neopixel data pin number, to allow init outside this HAL
    // (enable can be done by the HAL itself)
    gpio_num_t get_neopixel_data_pin() { return neopixel_data_gpio; }
};

extern Hal hal; // global instance of HAL, to be used in other modules
