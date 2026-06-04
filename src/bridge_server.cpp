#include "bridge_server.h"
#include "ssl_cert.h"
#include <Arduino.h>

static const char kLandingHtml[] PROGMEM =
    "<!DOCTYPE html>\n"
    "<html lang=\"en\">\n"
    "<head>\n"
    "<meta charset=\"UTF-8\">\n"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
    "<title>GoPro Bridge</title>\n"
    "<style>body{font-family:Arial,Helvetica,sans-serif;background:#101114;color:#e8e8e8;"
    "margin:0;padding:24px}a{color:#4cc2ff}code{background:#222;padding:2px 4px;border-radius:4px}" 
    "</style>\n"
    "</head>\n"
    "<body>\n"
    "<h1>GoPro Bridge</h1>\n"
    "<p>This page is served over HTTP so you can install the HTTPS certificate.</p>\n"
    "<ol>\n"
    "<li>Download the certificate: <a href=\"/cert.crt\">/cert.crt</a></li>\n"
    "<li>Install and trust it on your device.</li>\n"
    "<li>Open <a href=\"https://192.168.4.1/\">https://192.168.4.1</a></li>\n"
    "</ol>\n"
    "<p>Live video requires a secure context (HTTPS).</p>\n"
    "</body>\n"
    "</html>\n";

void serverStart(AsyncWebServer& server, BridgeConfig& cfg) {
    (void)cfg;

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        auto* r = req->beginResponse(200, "text/html", kLandingHtml);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    server.on("/cert.crt", HTTP_GET, [](AsyncWebServerRequest* req) {
        auto* r = req->beginResponse(200, "application/x-x509-ca-cert",
                                     reinterpret_cast<const uint8_t*>(kServerCertPem),
                                     kServerCertPemLen);
        r->addHeader("Content-Disposition", "attachment; filename=\"gopro-bridge.crt\"");
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    server.onNotFound([](AsyncWebServerRequest* req) {
        if (req->method() == HTTP_OPTIONS) {
            req->send(204);
            return;
        }
        String host = req->host();
        int colon = host.indexOf(':');
        if (colon >= 0) host = host.substring(0, colon);
        if (host.length() == 0) host = "192.168.4.1";
        req->redirect("https://" + host + req->url());
    });

    server.begin();
    Serial.println("[Server] HTTP cert server started on :80");
}
