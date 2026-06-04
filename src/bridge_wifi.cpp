#include "bridge_wifi.h"
#include <WiFi.h>

static BridgeConfig   _cfg;
static unsigned long  _lastCheck = 0;
static unsigned long  _lastBegin = 0;
static bool           _staArmed  = false;

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("[WiFi] STA: associated with AP.");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.printf("[WiFi] STA: connected! IP=%s  GoPro API at http://10.5.5.9:8080\n",
                          WiFi.localIP().toString().c_str());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.printf("[WiFi] STA: disconnected (reason=%d)\n",
                          (int)info.wifi_sta_disconnected.reason);
            break;
        default:
            break;
    }
}

void wifiStart(const BridgeConfig& cfg) {
    _cfg = cfg;
    WiFi.onEvent(onWiFiEvent);
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect(false, true);  // clear NVS credentials — prevent auto-reconnect before BLE ready

    IPAddress apIP(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apIP, subnet);
    WiFi.softAP(cfg.bridge_ssid.c_str(), cfg.bridge_pass.c_str());
    Serial.printf("[WiFi] AP: SSID=\"%s\"  IP=%s\n",
                  cfg.bridge_ssid.c_str(), WiFi.softAPIP().toString().c_str());
    Serial.println("[WiFi] STA: waiting for BLE to enable GoPro WiFi AP...");
}

void wifiStaConnect() {
    _staArmed  = true;
    _lastCheck = millis();
    _lastBegin = millis();
    Serial.printf("[WiFi] STA: connecting to \"%s\"...\n", _cfg.gopro_ssid.c_str());
    WiFi.begin(_cfg.gopro_ssid.c_str(), _cfg.gopro_pass.c_str());
}

bool wifiStaConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool wifiApHasClients() {
    return WiFi.softAPgetStationNum() > 0;
}

String wifiStaIP() {
    return WiFi.localIP().toString();
}

void wifiWatchdog() {
    if (!_staArmed) return;
    if (millis() - _lastCheck < 10000) return;
    _lastCheck = millis();

    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) return;

    // Laisser 25 s a un WiFi.begin() en cours avant de le tuer/relancer :
    // une connexion complete (scan + assoc + DHCP) prend souvent 10-20 s.
    if (millis() - _lastBegin < 25000) return;

    Serial.printf("[WiFi] STA retry (status=%d)...\n", (int)st);
    WiFi.disconnect(false);
    WiFi.begin(_cfg.gopro_ssid.c_str(), _cfg.gopro_pass.c_str());
    _lastBegin = millis();
}
