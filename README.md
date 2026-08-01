# ESP32xx Connected PoC

Simple Proof of Concept (PoC) application with web interface and basic console commands.
Intended to be used as start for making something usefull, and to test new features like a DotMatrix display.

What's in:

- **Logger**, based on ESP-IDF (not Arduino):
  - LOGx macros, eg LOGI("Starting up")
  - SET_LOG_LEVEL(level) macro, to set the level for this module (TAG)
  - SHOW_LOG_LEVEL macro,  to see which level is currently active in the module
- **Console**, with single char commands, eg:
  - [ ? ] = show menu
  - [ ! ] = crash (and restart again)
  - [ d ] = disconnect WiFi (and reconnect again)
  - [ s ] = system info
  - [ 0 ]...[ 6 ] = set log level for console only (to test the log mechanism)
- **WiFi Manager**:
  - Connect to your local WiFi using a captive portal to set SSID and password
  - Also set **hostname** on that captive portal, and save it as Preference in NVS
  - Optional push button, to clear the SSID, password and hostname
- **HTTP Server** with in-memory "Hello world" page
- **OLED display** will scroll automatically the lines you write to it
- **DotMatrix display** consisting of 8x8 units, console command to write a static text

## Tested ESP32xx devices

Nothing special, just added a few possible builds in platformio.ini and tested them.  
Below the measured values with Console commands directly after reboot, with bare minimum implemented (w/o OLED, DotMatrix, etc).

| ESP32xx module           | Free heap | Largest free block | Loops/sec |
|:-------------------------|:---------:|:------------------:|:---------:|
| ESP32 (D1 mini)          | 226960    | 110580             | 38200     |
| ESP32-C3 (Mini Pro)      | 195920    | 114676             | 61900     |
| ESP32-C6 (Seeed)         | 314428    | 294900             | 56300     |
| ESP32-S3 (Lilygo T7)     | 258796    | 217076             | 54400     |

> It's unclear why the "Largest free block" of ESP32 and ESP32-C3 is relatively small.  
Also checked directly after startup: same picture, so not due to the application.
>
> I do have an ESP32-S2 (Wemos S2 Mini), but I cannot get the console logging to work.  
Have seen this before, also at others. Giving up for now.

## Build environment

Visual Studio Code (VSC), latest version (currently 1.131.0), with next Extensions:

- C/C++ and CMake extensions:
  - C/C++
  - C/C++ DevTools
  - C/C++ Extension Pack
  - C/C++ Themes
  - CMake
  - CMake Tools
- ESP Crash Decoder
- GitHub Repositories
- markdownlint (eg for this file)
- pioarduino IDE
- Prettier - Code formatter
- Prettier-Standard - JavaScript formatter
- Python
- Python Debugger
- Python Environments

Git for Windows, latest x64 version (currently 2.55.0) from [Git](https://www.git-scm.com)

## Compiler warnings

When compiling the full application including the used standard libraries, quite some warnings are generated:

- WiFi Manager: using %d to print uint32_t value
- ESP32C3 and C6 hal: missing initializer

These warnings should be solved by the libraries themselves.
