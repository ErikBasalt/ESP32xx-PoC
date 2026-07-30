#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#include "logger.h"

static const char *TAG = "HTTP";

static const char *htmlContent = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Sample HTML</title>
</head>
<body>
    <h1>Hello, World!</h1>
    <p>Lorem ipsum dolor sit amet, consectetur adipiscing elit. Proin euismod, purus a euismod
    rhoncus, urna ipsum cursus massa, eu dictum tellus justo ac justo. Quisque ullamcorper
    arcu nec tortor ullamcorper, vel fermentum justo fermentum. Vivamus sed velit ut elit
    accumsan congue ut ut enim. Ut eu justo eu lacus varius gravida ut a tellus. Nulla facilisi.
    Integer auctor consectetur ultricies. Fusce feugiat, mi sit amet bibendum viverra, orci leo
    dapibus elit, id varius sem dui id lacus.</p>
</body>
</html>
)";

static const size_t htmlContentLength = strlen(htmlContent);

static AsyncWebServer server(80);

void startHttpServer(void) {
    LOGI("Starting HTTP server...");
    server.on("/", AsyncWebRequestMethod::HTTP_GET, // fully qualified name, to avoid conflict with same name in WiFiManager.h
              [](AsyncWebServerRequest *request) {
                  LOGI("Request from %s", request->client()->remoteIP().toString().c_str());
                  request->send(200, "text/html", (uint8_t *)htmlContent, htmlContentLength);
              });

    server.begin();
    LOGI("HTTP server started on `%s` =  http://%s", WiFi.getHostname(), WiFi.localIP().toString().c_str());
}
