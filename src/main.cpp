#include <Arduino.h>
#include "hal.h"
#include "logger.h"
#include "oled.h"
#include "mynetwork.h"
#include "console.h"
#include "httpserver.h"
#include "neopixel_rmt.h"
#include "system.h" // for loop() statistics
#include "dotmatrix.h"

static const char *TAG = "MAIN";

void setup() {
    hal.begin();            // initialize the GPIO pins, based on the used ESP32xx model
    hal.setStatusLed(true); // LED=On: setup() is running

#if (0 == 1)
    initNeoPixelRmt();
    setAllNeoPixels(0, 0, 16); // dim blue while setup is running
#endif

    startLogger(ESP_LOG_INFO); // default log level for all modules in src

    // Instantiate the dotmatrix display only AFTER starting the HAL (to have the gpio pin numbers set)
    // Keep it in (heap) memory and set a global pointer to it
    static DotMatrix dotMatrix(/*MOSI=*/hal.get_spi_MOSI_pin(), /*SCK=*/hal.get_spi_SCK_pin(), /*CS=*/hal.get_spi_CS_pin());
    dotMatrixPtr = &dotMatrix;

    startOledLog(); // initialize the OLED display, if present
    startWifi();
    startHttpServer();
    enableLoopWDT(); // enable the Watchdog for this task

    LOGI("Setup complete");
    LOG_PRINTF("Starting console (`?` for menu)\n");
    setAllNeoPixels(0, 0, 0); // clear boot test color
    hal.setStatusLed(false);  // LED=Off: setup() is complete
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
    consoleLoop();

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