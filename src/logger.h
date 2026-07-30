#pragma once
#include <esp_log.h>
/*
=======================================================================================================================
    ESP-IDF based logging

    Typical platformio.ini settings:

    framework = arduino
    platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.39/platform-espressif32.zip
    build_flags =
        [...]
        ; Next line enables ESP-IDF logging, instead of (more bloated) Arduino based logging
        -D USE_ESP_IDF_LOG=1
        -D CONFIG_LOG_COLORS=1
        ; Maximum log level for whole application (can be overruled for src files with LOG_LOCAL_LEVEL in build_src_flags)
        -D CORE_DEBUG_LEVEL=ESP_LOG_INFO
        [...]

    ; Extend the default build flags (also for used libs) with additional flags for the src directory only
    build_src_flags =
        ; Maximum log level for src files, can be higher (or lower, equal) than CORE_DEBUG_LEVEL
        -D LOG_LOCAL_LEVEL=ESP_LOG_VERBOSE
=======================================================================================================================
*/

#if (1 == 1)
// Short logging aliases
#define LOGE(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#define LOGW(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define LOGI(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define LOGD(format, ...) ESP_LOGD(TAG, format, ##__VA_ARGS__)
#define LOGV(format, ...) ESP_LOGV(TAG, format, ##__VA_ARGS__)

// Log formatted text output without any prefix, no need for TAG
#define LOG_PRINTF(format, ...) printf(format, ##__VA_ARGS__)
#else
#define LOGE(format, ...) Serial.printf(format, ##__VA_ARGS__)
#define LOGW(format, ...) Serial.printf(format, ##__VA_ARGS__)
#define LOGI(format, ...) Serial.printf(format, ##__VA_ARGS__)
#define LOGD(format, ...) Serial.printf(format, ##__VA_ARGS__)
#define LOGV(format, ...) Serial.printf(format, ##__VA_ARGS__)
#define LOG_PRINTF(format, ...) Serial.printf(format, ##__VA_ARGS__)
#endif

// Show the current log level for the module that calls this macro
// By using a macro, the TAG of the calling module is used (instead of the TAG of the logger module)
#define LOG_SHOW_LEVEL()   \
    do {                   \
        showLogLevel(TAG); \
    } while (0)

// Set the log level for the module that calls this macro, requires TAG to be set
// By using a macro, the TAG of the calling module is used (instead of the TAG of the logger module)
// Will give a warning when the requested level is higher than defined in platformio.ini
// The level can either be:
// - ESP_LOG_* value, as defined in esp_log_level_t
// - single char representing the level, e.g. 'E' for ESP_LOG_ERROR
#define LOG_SET_LEVEL(level)     \
    do {                         \
        setLogLevel(TAG, level); \
    } while (0)

// The actual functions that are called by the macros above
void showLogLevel(const char *tag);
void setLogLevel(const char *tag, const esp_log_level_t level); // ESP_LOG_* value
void setLogLevel(const char *tag, const char level);            // single char representing ESP_LOG_* value, e.g. 'E' for ESP_LOG_ERROR

// To start the logger, call this function in setup() with the default log level for all modules, e.g. ESP_LOG_DEBUG
void startLogger(esp_log_level_t defaultLevel);
