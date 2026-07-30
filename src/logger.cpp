#include <Arduino.h>
#include "logger.h"
#include "system.h" // to show memory info ASAP at startup
#define TAG "LOG_"  // each source file that uses the logger.h should define its own TAG, to be used in the LOGE/W/I/D/V macros

static const char logLevelChars[] = "NEWIDV"; // single chars representing ESP_LOG_* values, same order as in esp_log_level_t

static bool isValidLogLevel(esp_log_level_t level) {
    return ((level >= 0) && (level < strlen(logLevelChars)));
}

static char levelToChar(esp_log_level_t level) {
    if (!isValidLogLevel(level)) {
        return '?';
    }
    return logLevelChars[level];
}

static esp_log_level_t getMaxLogLevel(void) {
    esp_log_level_t maxLevel = (esp_log_level_t)CONFIG_LOG_MAXIMUM_LEVEL;
#ifdef LOG_LOCAL_LEVEL
    // In platformio.ini the CORE_DEBUG_LEVEL is overwritten by LOG_LOCAL_LEVEL, use it instead
    maxLevel = (esp_log_level_t)LOG_LOCAL_LEVEL;
#endif
    return (maxLevel);
}

//-----------------------------------------------------------------------------
//  Show the log level for the 'tag' module
//
//  Usually called via the LOG_SHOW_LEVEL() macro that uses TAG to set 'tag'
//  Mainly used for debugging, to check the current log level for a module
//-----------------------------------------------------------------------------
void showLogLevel(const char *tag) {
    if (tag == NULL) {
        LOGE("ERROR: tag is NULL, cannot show log level");
        return;
    }
    esp_log_level_t level = esp_log_level_get(tag);

    if (isValidLogLevel(level)) {
        LOG_PRINTF("Log level=%d(%c) for tag=`%s`\n", level, levelToChar(level), tag);
    } else {
        LOG_PRINTF("ERROR: Invalid log level=%d for tag=`%s`\n", level, tag);
    }

    LOG_PRINTF("------------------------------------\n");
    ESP_LOGE(tag, "ERROR test message");
    ESP_LOGW(tag, "WARNING test message");
    ESP_LOGI(tag, "INFO test message");
    ESP_LOGD(tag, "DEBUG test message");
    ESP_LOGV(tag, "VERBOSE test message");
    LOG_PRINTF("------------------------------------\n");
}

//-----------------------------------------------------------------------------
//  Set the log level (ESP_LOG_* value) for the 'tag' module
//
//  Usually called via the LOG_SET_LEVEL() macro that uses TAG to set 'tag'
//-----------------------------------------------------------------------------
void setLogLevel(const char *tag, const esp_log_level_t level) {
    if (tag == NULL) {
        LOGE("ERROR: tag is NULL, cannot set log level");
        return;
    }
    if (isValidLogLevel(level)) {
        esp_log_level_t maxLevel = getMaxLogLevel();
        if (level > maxLevel) {
            LOGW("WARNING: Log level=%d(%c) for tag=`%s` is higher than limit=%d(%c) in platformio.ini",
                 level, levelToChar(level),
                 tag,
                 maxLevel, levelToChar(maxLevel));
        }
        esp_log_level_set(tag, level); // for this module
    } else {
        LOGE("ERROR: Invalid log level=%d for tag=`%s`, using DEBUG instead", level, tag);
        esp_log_level_set(tag, ESP_LOG_DEBUG); // for this module
    }
}

void setLogLevel(const char *tag, const char level) { // single char representing ESP_LOG_* value, e.g. 'E' for ESP_LOG_ERROR
    if (tag == NULL) {
        LOGE("ERROR: tag is NULL, cannot set log level");
        return;
    }
    for (int i = 0; i < strlen(logLevelChars); i++) {
        if (logLevelChars[i] == level) {
            setLogLevel(tag, (esp_log_level_t)i);
            return;
        }
    }
    LOGE("ERROR: Invalid log level=`%c` for tag=`%s`, using DEBUG instead", level, tag);
    esp_log_level_set(tag, ESP_LOG_DEBUG); // for this module
}

//-----------------------------------------------------------------------------
//  Start
//
//  Specify the default log level for all modules, e.g. ESP_LOG_DEBUG
//  Can be overruled per module with LOG_SET_LEVEL() macro
//-----------------------------------------------------------------------------
void startLogger(esp_log_level_t defaultLevel) {

    esp_log_level_set("*", ESP_LOG_DEBUG); // Set log level to include ESP_LOGD messages

    // Start Serial, for:
    // - LOG_PRINTF(), which actually translates to printf()
    // - libraries (e.g. WiFiManager) that do not use ESP-IDF based logging
    // - the console, to read commands
    Serial.begin(115200);        // for library debug output, and for Platformio monitor
    Serial.setDebugOutput(true); // enable debug output to serial, for library debug output
    delay(5000);                 // wait for Platformio monitor to open

    Serial.println("Serial logging started"); //@@@TODO: remove
    /*
    Serial.println("\n\n<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>\nStarted serial logging\n"); // to clearly see where new logging starts
    Serial.flush();                                                                       // wait for all serial output to be sent, before continuing with ESP-IDF logging
    Serial.println("Hello world");
    Serial.println("Hello world");
    Serial.println("Hello world");
    Serial.println("Hello world");
    Serial.println("Hello world");
    Serial.println("Hello world");
    Serial.flush(); // wait for all serial output to be sent, before continuing with ESP-IDF logging
    */
    LOG_PRINTF("\n\n####################################\nStarted serial logging\n"); // to clearly see where new logging starts
    logResetReason();
    showMemoryInfo();

    LOG_PRINTF("Starting version=%d of ESP-IDF native logger...\n", CONFIG_LOG_VERSION); // to clearly see where new logging starts
    esp_log_level_t maxLevel = getMaxLogLevel();
    LOG_PRINTF("Configured maximum log level=%d(%c)\n", maxLevel, levelToChar(maxLevel));
    LOG_PRINTF("Setting default: ");      // no "\n" here, so log line about setting the level continues on the same line
    setLogLevel("*", defaultLevel);       // default log level for all modules
    esp_log_level_set("*", defaultLevel); // default log level for all modules
    showLogLevel("*");                    // show the default log level for all modules
}
