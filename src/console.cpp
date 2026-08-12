#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>

#include "hal.h"
#include "logger.h"
#include "oled.h"
#include "mynetwork.h"
#include "system.h"
#include "dotmatrix.h"
#include "neopixel_ring.h"

#define TAG "CONS"

static const int IO_STATUS_LED = 0;
static const int IO_NEOPIXEL_ENABLE = 1;
static const int IO_NEOPIXEL_DATA = 2;
static const int IO_SPI_CLOCK = 3;
static const int IO_SPI_MOSI = 4;
static const int IO_SPI_CS = 5;

static int selectedIo = IO_STATUS_LED; // default selected I/O

static bool isNeopixelAnimationActive = false; // default: Neopixel ring is NOT active

void toggleSelectedIo(void) {
    switch (selectedIo) {
    case IO_STATUS_LED:
        hal.toggleStatusLed();
        break;
    case IO_NEOPIXEL_ENABLE:
        hal.toggleNeoPixelEnable();
        break;
    case IO_NEOPIXEL_DATA:
        hal.toggleNeoPixelData();
        break;
    case IO_SPI_CLOCK:
        hal.toggleSpiClock();
        break;
    case IO_SPI_MOSI:
        hal.toggleSpiMosi();
        break;
    case IO_SPI_CS:
        hal.toggleSpiChipSelect();
        break;
    default:
        LOGE("Unknown selected I/O=%d", selectedIo);
        break;
    }
}

void selectNextIo(void) {
    switch (selectedIo) {
    case IO_STATUS_LED:
        selectedIo = IO_NEOPIXEL_ENABLE;
        LOGI("I/O=NeoPixel Enable");
        break;
    case IO_NEOPIXEL_ENABLE:
        selectedIo = IO_NEOPIXEL_DATA;
        LOGI("I/O=NeoPixel Data");
        break;
    case IO_NEOPIXEL_DATA:
        selectedIo = IO_SPI_CLOCK;
        LOGI("I/O=SPI Clock");
        break;
    case IO_SPI_CLOCK:
        selectedIo = IO_SPI_MOSI;
        LOGI("I/O=SPI MOSI");
        break;
    case IO_SPI_MOSI:
        selectedIo = IO_SPI_CS;
        LOGI("I/O=SPI Chip Select");
        break;
    case IO_SPI_CS:
        selectedIo = IO_STATUS_LED;
        LOGI("I/O=Status LED");
        break;
    default:
        LOGE("Unknown selected I/O=%d, using Status LED instead", selectedIo);
        selectedIo = IO_STATUS_LED;
        break;
    }
}

void consoleLoop(unsigned long currentMillis) {
    // Simple console, using single char commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
        case '\r':
        case '\n':
            // ignore
            break;
        case '?':
            LOG_PRINTF("* = restart\n");
            LOG_PRINTF("! = crash\n");
            LOG_PRINTF("a = analog read (mV)\n");
            LOG_PRINTF("d = disconnect WiFi (and reconnect) - using reconnect()\n");
            LOG_PRINTF("D = disconnect WiFi (and reconnect) - using disconnect()\n");
            LOG_PRINTF("f = forget network settings (and disconnect)\n");
            LOG_PRINTF("l = endless loop (Watchdog should reset the system)\n");
            LOG_PRINTF("m = memory info\n");
            LOG_PRINTF("n = network info\n");
            LOG_PRINTF("o = show millis() on OLED\n");
            LOG_PRINTF("O = clear OLED\n");
            LOG_PRINTF("p = PXD normal output\n");
            LOG_PRINTF("P = PXD 1 MHz signal output\n");
            LOG_PRINTF("q = toggle Neopixel ring animation\n");
            LOG_PRINTF("Q = erase Neopixel ring\n");
            LOG_PRINTF("s = system info\n");
            LOG_PRINTF("x = show `Hello!` on dotMatrix\n");
            LOG_PRINTF("X = clear dotMatrix\n");
            LOG_PRINTF("z = toggle selected I/O\n");
            LOG_PRINTF("Z = select next I/O\n");
            LOG_PRINTF("0...6 = set log level for the console\n");
            break;
        case '*':
            LOG_PRINTF("Do restart...\n");
            Serial.flush();
            ESP.restart();
            break;
        case '!':
            LOG_PRINTF("Do crash...\n");
            Serial.flush();
            {
                char *cptr = nullptr;
                *cptr = '!';
            }
            break;
        case 'a': {
            LOG_PRINTF("Analog read: %u mV\n", hal.readAnalog());
        } break;
        case 'd':
            LOG_PRINTF("Do disconnect/reconnect WiFi - using reconnect()\n");
            if (!WiFi.reconnect()) {
                LOGE("ERROR: Failed to reconnect WiFi");
            }
            break;
        case 'D':
            LOG_PRINTF("Do disconnect/reconnect WiFi - using disconnect()\n");
            if (!WiFi.disconnect(/*radio Off*/ false, /*erase AP info*/ false)) {
                LOGE("ERROR: Failed to disconnect WiFi");
            }
            break;
        case 'f':
            forgetNetwork();
            break;
        case 'l':
            LOG_PRINTF("Entering endless loop, the Watchdog should reset the system soon...\n");
            while (true) {
                // Endless loop
            }
            break;
        case 'm':
            showMemoryInfo();
            break;
        case 'n':
            showNetworkInfo();
            break;
        case 'o':
            oledLog.print(millis());
            oledLog.writeChar('\n');
            break;
        case 'O':
            oledLog.writeChar('\f');
            // Note: used text lib u8x8 does not support setPowerSave()
            break;
        case 'p':
            LOG_PRINTF("PXD normal output\n");
            hal.setNeoPixelDataSignalGenerator(false);
            break;
        case 'P':
            LOG_PRINTF("PXD 1 MHz signal output\n");
            hal.setNeoPixelDataSignalGenerator(true);
            break;
        case 'q':
            isNeopixelAnimationActive = !isNeopixelAnimationActive;
            break;
        case 'Q':
            allBlackNeopixelRing();
            break;
        case 'x':
            dotMatrixPtr->showMessage("Hello!");
            break;
        case 'X':
            dotMatrixPtr->showMessage("");
            break;
        case 's':
            showSystemInfo();
            break;
        case 'z':
            toggleSelectedIo();
            break;
        case 'Z':
            selectNextIo();
            break;
        case ('0' + ESP_LOG_NONE):
        case ('0' + ESP_LOG_ERROR):
        case ('0' + ESP_LOG_WARN):
        case ('0' + ESP_LOG_INFO):
        case ('0' + ESP_LOG_DEBUG):
        case ('0' + ESP_LOG_VERBOSE):
        case ('0' + ESP_LOG_VERBOSE + 1): // out of range, use to test error handling
            LOG_SET_LEVEL((esp_log_level_t)(cmd - '0'));
            LOG_SHOW_LEVEL();
            break;
        default:
            LOG_PRINTF("Unknown command=`%c`\n", cmd);
            break;
        }
    }
    if (isNeopixelAnimationActive) {
        loopNeopixelRing(currentMillis);
    }
}
