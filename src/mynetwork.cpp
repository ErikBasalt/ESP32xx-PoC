#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Ticker.h>      // for reconnect timer
#include <Preferences.h> // for hostname in NVS

#include "logger.h"

#define TAG "NETW"

// static const gpio_num_t WiFi_reset_button_pin = GPIO_NUM_5; // Push button to GND, set to GPIO_NUM_NC if not used
static const gpio_num_t WiFi_reset_button_pin = GPIO_NUM_NC; // Push button to GND, set to GPIO_NUM_NC if not used

static const char *captivePortalSSID = "ConfigESP32"; // SSID of Access Point, when running captive portal to set the config

static bool shouldSaveConfig = false; // flag indicating that settings ere saved on the captive portal

static char hostname[21] = "testname"; // default hostname of this system, can be changed during WiFi config on captive portal

#define CHANGE_HOSTNAME_ON_PORTAL 1 // uncomment to allow changing hostname on captive portal, and saving it to NVS
#ifdef CHANGE_HOSTNAME_ON_PORTAL
//--------------------------------------------------
//  Hostname to/from NVS
//--------------------------------------------------
static const char *nvsNamespace = "mycfg";
static const char *nvsHostnameKey = "hostname";

static void readHostnameFromNvs(void) {
    Preferences prefs;
    if (!prefs.begin(nvsNamespace, /*readOnly=*/true)) {
        // Possibly first boot, so no NVS namespace yet
        LOGW("WARNING: Failed to open NVS namespace=`%s` for read, will use default hostname instead", nvsNamespace);
        return;
    }

    char tmpName[sizeof(hostname)] = "";
    if ((prefs.isKey(nvsHostnameKey)) && (prefs.getString(nvsHostnameKey, tmpName, (sizeof(tmpName) - 1)))) {
        LOGI("Have read hostname=`%s` from NVS", tmpName);
        strlcpy(hostname, tmpName, sizeof(hostname));
    } else {
        // Possibly deleted the hostname key from NVS
        LOGW("WARNING: Failed to read key=`%s` from NVS namespace=`%s`, will use default hostname instead", nvsHostnameKey, nvsNamespace);
    }
    prefs.end();
}

static void writeHostnameToNvs(const char *name) {
    Preferences prefs;
    LOGI("Write hostname=`%s` to NVS", name);
    if (!prefs.begin(nvsNamespace, /*readOnly=*/false)) {
        LOGE("ERROR: Failed to open NVS namespace=`%s` for write", nvsNamespace);
        return;
    }

    if (prefs.putString(nvsHostnameKey, name) == 0) {
        LOGE("ERROR: Failed to write hostname to NVS");
    }
    prefs.end();
    return;
}

static void deleteHostnameFromNvs(void) {
    Preferences prefs;
    LOGI("Deleting hostname from NVS");
    if (!prefs.begin(nvsNamespace, /*readOnly=*/false)) {
        LOGE("ERROR: Failed to open NVS namespace=`%s` to delete hostname", nvsNamespace);
        return;
    }

    if (!prefs.remove(nvsHostnameKey)) {
        LOGE("ERROR: Failed to delete hostname key=`%s` from NVS namespace=`%s`", nvsHostnameKey, nvsNamespace);
    }
    prefs.end();
}
#endif

//--------------------------------------------------
//  Forget network settings
//--------------------------------------------------
void forgetNetwork(void) {
    LOG_PRINTF("Do forget the network settings\n");
    WiFi.disconnect(/*radio Off*/ false, /*erase AP info*/ true);
#ifdef CHANGE_HOSTNAME_ON_PORTAL
    deleteHostnameFromNvs();
#endif
}

//--------------------------------------------------
//  Show network info
//--------------------------------------------------
void showNetworkInfo(void) {
    LOG_PRINTF("---Network---\n");
    LOG_PRINTF("Hostname=`%s`\n", WiFi.getHostname());
    LOG_PRINTF("IP address=`%s`\n", WiFi.localIP().toString().c_str());
    LOG_PRINTF("MAC address=`%s`\n", WiFi.macAddress().c_str());
    LOG_PRINTF("WiFi signal (dBm)=%d\n", WiFi.RSSI());
}

//--------------------------------------------------
//  Automatic reconnect WiFi
//--------------------------------------------------
#define WIFI_RECONNECT_WATCHDOG_SECONDS 10 // time (secs) before Watchdog will check whether WiFi is reconnected again, and retry if not
static Ticker wifiReconnectWatchdogTimer;

static void wifiReconnectWatchdog(void) {
    if (WiFi.status() != WL_CONNECTED) {
        LOGW("WiFi reconnect watchdog: reconnecting...");
        if (WiFi.reconnect()) {
            LOGI("WiFi reconnect watchdog: reconnected now, done");
        } else {
            LOGW("WiFi reconnect watchdog: could not reconnect, retrying...");
            wifiReconnectWatchdogTimer.once(WIFI_RECONNECT_WATCHDOG_SECONDS, wifiReconnectWatchdog); // retry to reconnect in X seconds
        }
    } else {
        LOGI("WiFi reconnect watchdog: already (re)connected, done");
    }
}

//--------------------------------------------------
//  WiFi callbacks
//--------------------------------------------------
static void wifi_config_mode_cb(WiFiManager *wm) {
    LOGI("Entered WiFi config mode, captive portal on SSID=`%s`", wm->getConfigPortalSSID().c_str());
}

static void wifi_saved_cb(void) {
    LOGI("WiFi config saved, connect to router=`%s`", WiFi.SSID().c_str());
    shouldSaveConfig = true; // flag to save hostname to NVS, after WiFiManager has finished
}

static void wifi_connected_cb(WiFiEvent_t event, WiFiEventInfo_t info) {
    // Don't try to log the IP address right now, because it may not be available yet
    LOGI("WiFi reconnected on channel=%u", (unsigned)info.wifi_sta_connected.channel);
}

static void wifi_disconnected_cb(WiFiEvent_t event, WiFiEventInfo_t info) {
    LOGW("WARNING: WiFi is disconnected, reason=%u", (unsigned)info.wifi_sta_disconnected.reason);
    if (!WiFi.getAutoReconnect()) {
        LOGI("Standard WiFi auto reconnect is NOT enabled -> Start watchdog");
        wifiReconnectWatchdogTimer.detach();                                                     // stop timer, to prevent duplicate reconnect in case we get reconnected automatically
        wifiReconnectWatchdogTimer.once(WIFI_RECONNECT_WATCHDOG_SECONDS, wifiReconnectWatchdog); // try to reconnect after X time in seconds
    } // else: standard WiFi auto reconnect will a handle the reconnect, no need for watchdog
}

//--------------------------------------------------
//  Start WiFi
//--------------------------------------------------
void startWifi(void) {
    WiFiManager wm; // local initialization, once its business is done, there is no need to keep it around
    LOGI("Starting WiFi...");

    //-----------------------------------------------------------
    // Reset WiFi button handling
    //-----------------------------------------------------------
    if (WiFi_reset_button_pin != GPIO_NUM_NC) {
        LOGI("WiFi reset button pin=%d", WiFi_reset_button_pin);
        pinMode(WiFi_reset_button_pin, INPUT_PULLUP);
        delay(100); // GPIO needs a little time to settle

        if (digitalRead(WiFi_reset_button_pin) == LOW) {
            LOGW("WARNING: WiFi Reset button pressed, resetting settings...");
            wm.resetSettings(); // results in starting the captive portal later on
#ifdef CHANGE_HOSTNAME_ON_PORTAL
            deleteHostnameFromNvs();
#endif
        }
    } else {
        LOGI("(no WiFi reset button)");
    }

    //-----------------------------------------------------------
    // Setup captive portal, in case we need it later on
    //-----------------------------------------------------------
    wm.setAPStaticIPConfig(IPAddress(10, 0, 1, 1), IPAddress(10, 0, 1, 1), IPAddress(255, 255, 255, 0)); // easier IP address, that nicely fits on the dotmatrix display
#ifdef CHANGE_HOSTNAME_ON_PORTAL
    // Add hostname parameter to captive portal, so it can be changed and saved to NVS
    WiFiManagerParameter hostnameParam("hostnameParamID", "Clock hostname", hostname, sizeof(hostname) - 1); // to change hostname on captive portal, if desired
    wm.addParameter(&hostnameParam);
    readHostnameFromNvs(); // read the hostname from NVS, so it can be used as default value on captive portal
#endif
    LOGI("Using hostname=`%s`", hostname);
    WiFi.setHostname(hostname); // do this BEFORE wm.autoConnect(), to make it working over 4G+VPN as well
    wm.setAPCallback(wifi_config_mode_cb);
    wm.setSaveConfigCallback(wifi_saved_cb);

    //-----------------------------------------------------------
    // Connect to WiFi router
    // either directly, or first via captive portal
    //-----------------------------------------------------------
    if (wm.autoConnect(captivePortalSSID)) { // Access Point without password, to be used as captive portal in case
        // Connected to the WiFi Router now
        delay(100); // wait a little for the WiFi Manager to complete sending log lines
        LOGI("Connected to WiFi=`%s`, RSSI (dBm)=%d", WiFi.SSID().c_str(), WiFi.RSSI());

        if (shouldSaveConfig) {
#ifdef CHANGE_HOSTNAME_ON_PORTAL
            // Save the hostname to NVS, so it can be used on next boot
            // (SSID and password are already saved by WiFiManager)
            const char *newHostname = hostnameParam.getValue();
            if ((newHostname != nullptr) && (newHostname[0] != '\0')) {
                strlcpy(hostname, newHostname, sizeof(hostname)); // use new
            } else {                                              // else: keep using the old of default hostname
                LOGW("WARNING: Hostname was empty on captive portal, keep using old or default hostname=`%s`", hostname);
            }
            // Always save (even when using default), bacause it may not be in NVS yet
            writeHostnameToNvs(hostname);
#endif
            // Without restart, HTTP Server will not respond properly
            LOGI("Restarting system to apply new WiFi config");
            ESP.restart();
            while (true) {
                delay(1000); // wait for restart
            }

        } else {
            LOGI("WiFi config unchanged");
        }

        //-----------------------------------------------------------
        // All good: WiFi is connected now
        // Continue with the rest of the program
        //-----------------------------------------------------------
        // Only Station mode from here (no need for having Access Point at the same time)
        WiFi.mode(WIFI_STA);

        // Reconnect WiFi after connection is lost
        // Seems to work pretty well, no need to use a watchdog timer to reconnect
        WiFi.setAutoReconnect(true); // is default, just to be sure

        // Callbacks to handle future WiFi (dis)connects
        WiFi.onEvent(wifi_connected_cb, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
        WiFi.onEvent(wifi_disconnected_cb, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    } else {
        // Should NOT come here
        LOGE("ERROR: Failed to connect");
    }
}
