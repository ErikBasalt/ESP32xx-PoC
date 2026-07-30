#include "oled.h"
#include "hal.h"

#define U8LOG_WIDTH 16
#define U8LOG_HEIGHT 8

U8X8LOG oledLog;

static uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];

void startOledLog(void) {
    static U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/*reset=*/U8X8_PIN_NONE, /*scl=*/hal.get_i2c_SCL_gpio(), /*sda=*/hal.get_i2c_SDA_gpio()); // for 128x64 OLED via I2C
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    oledLog.begin(u8x8, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
    oledLog.setRedrawMode(/*update screen on every char=*/0);
}
