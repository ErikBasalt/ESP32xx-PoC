#include <Arduino.h>
#include "hal.h"
#include "logger.h"
#include "oled.h"
#include "mynetwork.h"
#include "console.h"
#include "httpserver.h"
#include "system.h" // for loop() statistics
#include "dotmatrix.h"
#include "neopixel_ring.h"

static const char *TAG = "MAIN";

void setup() {
    hal.begin(/*isRoundDisplay=*/false); // initialize the GPIO pins, based on the used ESP32xx model
    hal.setStatusLed(true);              // LED=On: setup() is running

    startLogger(ESP_LOG_INFO); // default log level for all modules in src

    // Instantiate the dotmatrix display only AFTER starting the HAL (to have the gpio pin numbers set)
    // Keep it in (heap) memory and set a global pointer to it
    static DotMatrix dotMatrix(/*MOSI=*/hal.get_spi_MOSI_pin(), /*SCK=*/hal.get_spi_SCK_pin(), /*CS=*/hal.get_spi_CS_pin());
    dotMatrixPtr = &dotMatrix;

    startOledLog(); // initialize the OLED display, if present
    startWifi();
    startHttpServer();
    startNeopixelRing(); // initialize the Neopixel ring, if present
    enableLoopWDT();     // enable the Watchdog for this task

    LOGI("Setup complete");
    LOG_PRINTF("Starting console (`?` for menu)\n");

    hal.setStatusLed(false); // LED=Off: setup() is complete
}

void loop() {
    static int loopCounter = 0;
    static time_t previousTime = 0;
    static unsigned long previousMillis = 0;

    const unsigned long currentMillis = millis();
    time_t timeNow;

    //--------------------------------------------------
    // Determine maximum time between loop calls
    //--------------------------------------------------
    if (previousMillis != 0) {
        unsigned int deltaMillis = currentMillis - previousMillis;
        if (deltaMillis > maxLoopIntervalMillis) {
            maxLoopIntervalMillis = deltaMillis;
        }
    }
    previousMillis = currentMillis;

    //--------------------------------------------------
    // Console
    //--------------------------------------------------
    consoleLoop(currentMillis);

    //--------------------------------------------------
    // Every new microprocessor second
    //--------------------------------------------------
    time(&timeNow);
    if (timeNow != previousTime) {
        hal.toggleStatusLed(); // LED=Blinking: loop() is running

        if (previousTime) {
            loopsPerSecond = loopCounter; // execution speed, in loops per second
        }
        previousTime = timeNow;
        loopCounter = 0;
    }

    loopCounter++;
}