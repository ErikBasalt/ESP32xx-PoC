#include <Arduino.h>

#include "logger.h"

#define TAG "SYS_"

unsigned int maxLoopIntervalMillis = 0;
unsigned int loopsPerSecond = 0;

//--------------------------------------------------
//  Show the reason why the system was restarted
//--------------------------------------------------
void logResetReason(void) {
    auto reason = esp_reset_reason();
    LOG_PRINTF("Reset reason=%d: ", (int)reason);
    switch (reason) {
    case ESP_RST_POWERON:
        LOG_PRINTF("Power On\n");
        break;
    case ESP_RST_EXT:
        LOG_PRINTF("External reset\n");
        break;
    case ESP_RST_SW:
        LOG_PRINTF("Software reset\n");
        break;
    case ESP_RST_PANIC:
        LOG_PRINTF("Panic or Exception\n");
        break;
    case ESP_RST_INT_WDT:
        LOG_PRINTF("Interrupt watchdog\n");
        break;
    case ESP_RST_TASK_WDT:
        LOG_PRINTF("Task watchdog\n");
        break;
    case ESP_RST_WDT:
        LOG_PRINTF("Other watchdog\n");
        break;
    case ESP_RST_DEEPSLEEP:
        LOG_PRINTF("Wake-up\n");
        break;
    case ESP_RST_BROWNOUT:
        LOG_PRINTF("Brownout\n");
        break;
    case ESP_RST_SDIO:
        LOG_PRINTF("SDIO\n");
        break;
    case ESP_RST_USB:
        LOG_PRINTF("USB\n");
        break;
    case ESP_RST_JTAG:
        LOG_PRINTF("JTAG\n");
        break;
    case ESP_RST_EFUSE:
        LOG_PRINTF("eFuse error\n");
        break;
    case ESP_RST_PWR_GLITCH:
        LOG_PRINTF("Power glitch\n");
        break;
    case ESP_RST_CPU_LOCKUP:
        LOG_PRINTF("Double exception\n");
        break;
    default:
        LOG_PRINTF("Unknown\n");
        break;
    }
}

//--------------------------------------------------
//  Show (log) the system info
//--------------------------------------------------
void showSystemInfo(void) {
    LOG_PRINTF("---System---\n");
    logResetReason();
    LOG_PRINTF("ESP-IDF version=`%s`\n", ESP.getSdkVersion());
    LOG_PRINTF("Arduino version=`%s`\n", ESP.getCoreVersion());
    LOG_PRINTF("App build timestamp=`%s`\n", __DATE__ ", " __TIME__);
    LOG_PRINTF("Chip model=`%s`\n", ESP.getChipModel());
    LOG_PRINTF("Chip revision=%u\n", (unsigned)ESP.getChipRevision());
    LOG_PRINTF("Number of cores=%u\n", (unsigned)ESP.getChipCores());
    LOG_PRINTF("RTOS tasks=%u\n", (unsigned)uxTaskGetNumberOfTasks());
    LOG_PRINTF("Max loop interval=%u ms\n", maxLoopIntervalMillis);
    LOG_PRINTF("Loops/sec=%u\n", loopsPerSecond);
}

//--------------------------------------------------
//  Show (log) the memory info
//--------------------------------------------------
void showMemoryInfo(void) {
    LOG_PRINTF("---Memory---\n");
    LOG_PRINTF("Free heap=%" PRIu32 "\n", ESP.getFreeHeap());
    LOG_PRINTF("Largest free block=%" PRIu32 "\n", ESP.getMaxAllocHeap());
    LOG_PRINTF("Min free heap=%" PRIu32 "\n", ESP.getMinFreeHeap());
    LOG_PRINTF("Free PSRAM=%" PRIu32 "\n", ESP.getFreePsram());
}
