#include "hal.h"

Hal hal; // instance of the HAL

/*
---------------------------------------------------------------------------------------------------
    Set the GPIO pins, based on the used ESP32xx chip

    In fact, it also depends on the used ESP32xx breakout board,
    because the available GPIO pins can differ between boards with same ESP32xx chip
    However, I simply use just one of each per ESP32xx chip.

    Use numbers and (gpio_num_t) typecasts instead of GPIO_NUM_XX constants,
    because the available GPIO_NUM_XX constants differ between ESP32xx models and boards.
---------------------------------------------------------------------------------------------------
*/
bool Hal::setGpioPins(void) {
    bool rv = true;
    const char *esp32Model = ESP.getChipModel();

    if (strcmp(esp32Model, "ESP32-D0WD-V3") == 0) {
        //-----------------------------------------------------------
        // "ESP32 D1 Mini" breakout board
        //
        // ESP32-D0WD-V3 chip, dual core
        // PCB antenna
        // Has 4 pin rows, only the 2 inner rows are used
        //-----------------------------------------------------------
        statusLed_gpio = (gpio_num_t)16; // onboard (blue) LED is GPIO_2 and can be used, however: use GPIO_16 for external (yellow) LED on big PCB, to be consistent with ESP32-S3 board
        neopixel_enable_gpio = (gpio_num_t)26;
        neopixel_data_gpio = (gpio_num_t)19;
        spi_SCK_gpio = (gpio_num_t)18;
        spi_MOSI_gpio = (gpio_num_t)23;
        spi_CS_gpio = (gpio_num_t)5;
        analogInput_gpio = (gpio_num_t)36; // ADC1_CH0
        i2c_SCL_gpio = (gpio_num_t)22;
        i2c_SDA_gpio = (gpio_num_t)21;
    } else if (strcmp(esp32Model, "ESP32-S2") == 0) {
        //-----------------------------------------------------------
        // @@@TODO: Wemos ESP32-S2 D1 Mini breakout board
        //-----------------------------------------------------------
        statusLed_gpio = (gpio_num_t)15;
        neopixel_enable_gpio = (gpio_num_t)-1;
        neopixel_data_gpio = (gpio_num_t)-1;
        spi_SCK_gpio = (gpio_num_t)-1;
        spi_MOSI_gpio = (gpio_num_t)-1;
        spi_CS_gpio = (gpio_num_t)-1;
        analogInput_gpio = (gpio_num_t)-1;
        i2c_SCL_gpio = (gpio_num_t)-1;
        i2c_SDA_gpio = (gpio_num_t)-1;
    } else if (strcmp(esp32Model, "ESP32-S3") == 0) {
        //-----------------------------------------------------------
        // "Lilygo T7 S3", breakout board
        //
        // ESP32-S3 chip, dual core
        // PCB antenna
        // Has 4 pin rows, only the 2 inner rows are used
        //-----------------------------------------------------------
        statusLed_gpio = (gpio_num_t)12; // onboard (green) LED is GPIO_17 but in use for NeoPixel Data, so use GPIO_12 for external (yellow) LED on big PCB
        neopixel_enable_gpio = (gpio_num_t)16;
        neopixel_data_gpio = (gpio_num_t)17;
        spi_SCK_gpio = (gpio_num_t)18;
        spi_MOSI_gpio = (gpio_num_t)8;
        spi_CS_gpio = (gpio_num_t)5;
        analogInput_gpio = (gpio_num_t)15; // ADC2_CH4
        i2c_SCL_gpio = (gpio_num_t)14;
        i2c_SDA_gpio = (gpio_num_t)13;
    } else if (strcmp(esp32Model, "ESP32-C3") == 0) {
        //-----------------------------------------------------------
        // "ESP32-C3 Mini Pro" breakout board
        //
        // Big ceramic antenna
        // U.FL connector for external antenna (requires soldering)
        //-----------------------------------------------------------
        statusLed_gpio = (gpio_num_t)8; // onboard (blue) LED
        statusLed_activeLevel = LOW;    // this board used an active LOW output to switch On the (blue)status LED
        neopixel_enable_gpio = (gpio_num_t)10;
        neopixel_data_gpio = (gpio_num_t)5;
        spi_SCK_gpio = (gpio_num_t)1;
        spi_MOSI_gpio = (gpio_num_t)3;
        spi_CS_gpio = (gpio_num_t)4;
        analogInput_gpio = (gpio_num_t)0; // ADC1_CH0
        i2c_SCL_gpio = (gpio_num_t)6;
        i2c_SDA_gpio = (gpio_num_t)7;
    } else if (strcmp(esp32Model, "ESP32-C6") == 0) {
        //-----------------------------------------------------------
        // "Seeed ESP32-C6" breakout board
        //
        // Very small with 2x7 pins, metal shielding
        // Ceramic antenna
        // U.FL connector for external antenna (software selectable)
        //-----------------------------------------------------------
        statusLed_gpio = (gpio_num_t)15;
        neopixel_enable_gpio = (gpio_num_t)21;
        neopixel_data_gpio = (gpio_num_t)2;
        spi_SCK_gpio = (gpio_num_t)19;
        spi_MOSI_gpio = (gpio_num_t)18;
        spi_CS_gpio = (gpio_num_t)20;
        analogInput_gpio = (gpio_num_t)0; // ADC1_CH0
        i2c_SCL_gpio = (gpio_num_t)23;
        i2c_SDA_gpio = (gpio_num_t)22;
    } else {
        // Unknown breakout board, no GPIO pins
        rv = false;
    }
    return rv;
}
