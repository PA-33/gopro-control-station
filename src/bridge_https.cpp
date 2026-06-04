#include "bridge_https.h"
#include "bridge_ble.h"
#include "bridge_wifi.h"
#include "bridge_config.h"
#include "bridge_foc.h"
#include "bridge_led.h"
#include "ssl_cert.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include <esp_https_server.h>
#include <memory>

static BridgeConfig* _cfg = nullptr;
static httpd_handle_t _https = nullptr;
static String _baseUrl;

static const char* mimeTypeForPath(const String& path) {
    String lower = path;
    lower.toLowerCase();
    if (lower.endsWith(".html"))  return "text/html";
    if (lower.endsWith(".css"))   return "text/css";
    if (lower.endsWith(".js"))    return "application/javascript";
    if (lower.endsWith(".mjs"))   return "application/javascript";
    if (lower.endsWith(".json"))  return "application/json";
    if (lower.endsWith(".svg"))   return "image/svg+xml";
    if (lower.endsWith(".png"))   return "image/png";
    if (lower.endsWith(".jpg") || lower.endsWith(".jpeg")) return "image/jpeg";
    if (lower.endsWith(".gif"))   return "image/gif";
    if (lower.endsWith(".webp"))  return "image/webp";
    if (lower.endsWith(".ico"))   return "image/x-icon";
    if (lower.endsWith(".txt"))   return "text/plain";
    if (lower.endsWith(".map"))   return "application/json";
    if (lower.endsWith(".woff"))  return "font/woff";
    if (lower.endsWith(".woff2")) return "font/woff2";
    if (lower.endsWith(".ttf"))   return "font/ttf";
    if (lower.endsWith(".otf"))   return "font/otf";
    if (lower.endsWith(".mp4"))   return "video/mp4";
    if (lower.endsWith(".webm"))  return "video/webm";
    return "application/octet-stream";
}

static void addCors(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static const char* statusStr(int status) {
    switch (status) {
        case 200: return HTTPD_200;
        case 204: return HTTPD_204;
        case 400: return HTTPD_400;
        case 404: return HTTPD_404;
        case 415: return "415 Unsupported Media Type";
        case 502: return "502 Bad Gateway";
        case 503: return "503 Service Unavailable";
        default:  return "500 Internal Server Error";
    }
}

static esp_err_t sendJson(httpd_req_t* req, int status, const String& body) {
    httpd_resp_set_status(req, statusStr(status));
    httpd_resp_set_type(req, "application/json");
    addCors(req);
    return httpd_resp_send(req, body.c_str(), body.length());
}

static bool recvBody(httpd_req_t* req, String& out, size_t maxLen) {
    size_t len = req->content_len;
    if (len == 0) { out = ""; return true; }
    if (len > maxLen) return false;

    std::unique_ptr<char[]> buf(new char[len + 1]);
    size_t received = 0;
    while (received < len) {
        int r = httpd_req_recv(req, buf.get() + received, len - received);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) return false;
        received += static_cast<size_t>(r);
    }
    buf[len] = '\0';
    out = String(buf.get());
    return true;
}

// esp_http_server ne decode pas les %XX dans les query params. La GoPro
// rejette /gopro/media/thumbnail?path=100GOPRO%2FGOPR0441.JPG avec 400 si
// les %2F restent encodes. On decode tout %XX -> caractere correspondant.
static String urlDecode(const String& src) {
    String dst;
    dst.reserve(src.length());
    for (size_t i = 0; i < src.length(); i++) {
        char c = src[i];
        if (c == '+') {
            dst += ' ';
        } else if (c == '%' && i + 2 < src.length() &&
                   isxdigit(src[i+1]) && isxdigit(src[i+2])) {
            char hex[3] = { src[i+1], src[i+2], 0 };
            dst += (char)strtol(hex, nullptr, 16);
            i += 2;
        } else {
            dst += c;
        }
    }
    return dst;
}

static bool getQueryParam(httpd_req_t* req, const char* key, String& out) {
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen <= 1) return false;

    std::unique_ptr<char[]> buf(new char[qlen]);
    if (httpd_req_get_url_query_str(req, buf.get(), qlen) != ESP_OK) return false;

    std::unique_ptr<char[]> value(new char[qlen]);
    if (httpd_query_key_value(buf.get(), key, value.get(), qlen) != ESP_OK) return false;

    out = String(value.get());
    return true;
}

static String bleResult(bool ok) {
    return ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"BLE not connected\"}";
}

static esp_err_t handleOptions(httpd_req_t* req) {
    httpd_resp_set_status(req, HTTPD_204);
    addCors(req);
    return httpd_resp_send(req, nullptr, 0);
}

static esp_err_t handleBleShutterOn(httpd_req_t* req)    {
    bool ok = goproBLE.shutterOn();
    if (ok) {
        bridgeNotifyShutter();
    }
    return sendJson(req, 200, bleResult(ok));
}
static esp_err_t handleBleShutterOff(httpd_req_t* req)   { return sendJson(req, 200, bleResult(goproBLE.shutterOff())); }
// setActiveMode au lieu de loadPresetGroup : memorise le mode pour le resend
// periodique (cf. bridge_ble.cpp _MODE_RESEND_MS).
static esp_err_t handleBleModeVideo(httpd_req_t* req)    { goproBLE.setActiveMode(1000); return sendJson(req, 200, bleResult(goproBLE.isConnected())); }
static esp_err_t handleBleModePhoto(httpd_req_t* req)    { goproBLE.setActiveMode(1001); return sendJson(req, 200, bleResult(goproBLE.isConnected())); }
static esp_err_t handleBleModeTimelapse(httpd_req_t* req){ goproBLE.setActiveMode(1002); return sendJson(req, 200, bleResult(goproBLE.isConnected())); }
static esp_err_t handleBleHilight(httpd_req_t* req)      { return sendJson(req, 200, bleResult(goproBLE.hilight())); }
static esp_err_t handleBleSleep(httpd_req_t* req)        { return sendJson(req, 200, bleResult(goproBLE.cameraSleep())); }

static esp_err_t handleBleStatus(httpd_req_t* req) {
    String body = "{\"connected\":" + String(goproBLE.isConnected() ? "true" : "false") +
                  ",\"device\":\"" + goproBLE.deviceName() + "\"}";
    return sendJson(req, 200, body);
}

static esp_err_t handleFocBtnModeGet(httpd_req_t* req) {
    String body = String("{\"hold_continuous\":") +
                  (bridgeFocGetButtonHoldContinuous() ? "true" : "false") + "}";
    return sendJson(req, 200, body);
}

static esp_err_t handleFocBtnModePost(httpd_req_t* req) {
    String body;
    if (!recvBody(req, body, 256)) {
        return sendJson(req, 400, "{\"error\":\"bad request\"}");
    }
    JsonDocument doc;
    if (deserializeJson(doc, body) || doc["hold_continuous"].isNull()) {
        return sendJson(req, 400, "{\"error\":\"missing hold_continuous\"}");
    }
    bridgeFocSetButtonHoldContinuous(doc["hold_continuous"].as<bool>());
    String resp = String("{\"ok\":true,\"hold_continuous\":") +
                  (bridgeFocGetButtonHoldContinuous() ? "true" : "false") + "}";
    return sendJson(req, 200, resp);
}

// Telechargement du cert auto-signe. Le user accepte d'abord le warning
// "Connexion non securisee" sur https://192.168.4.1, ce qui suffit a charger
// la page et donc a recuperer cert.crt. Apres install + reload, plus de
// warning. Remplace l'ancien endpoint sur le serveur HTTP port 80.
static esp_err_t handleCertDownload(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/x-x509-ca-cert");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"gopro-bridge.crt\"");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    addCors(req);
    return httpd_resp_send(req, kServerCertPem, kServerCertPemLen);
}

static esp_err_t handleFocStepGet(httpd_req_t* req) {
    String body = String("{\"step_deg\":") + String(bridgeFocGetStepDeg(), 2) +
                  ",\"speed_dps\":" + String(bridgeFocGetHoldDegPerSec(), 1) + "}";
    return sendJson(req, 200, body);
}

static esp_err_t handleFocStepPost(httpd_req_t* req) {
    String body;
    if (!recvBody(req, body, 256)) {
        return sendJson(req, 400, "{\"error\":\"bad request\"}");
    }
    JsonDocument doc;
    if (deserializeJson(doc, body)) {
        return sendJson(req, 400, "{\"error\":\"bad JSON\"}");
    }
    if (!doc["step_deg"].isNull())  bridgeFocSetStepDeg(doc["step_deg"].as<float>());
    if (!doc["speed_dps"].isNull()) bridgeFocSetHoldDegPerSec(doc["speed_dps"].as<float>());
    String resp = String("{\"ok\":true,\"step_deg\":") + String(bridgeFocGetStepDeg(), 2) +
                  ",\"speed_dps\":" + String(bridgeFocGetHoldDegPerSec(), 1) + "}";
    return sendJson(req, 200, resp);
}

static esp_err_t handleStatus(httpd_req_t* req) {
    JsonDocument doc;
    doc["sta_connected"] = wifiStaConnected();
    doc["sta_ip"] = wifiStaIP();
    doc["ble_connected"] = goproBLE.isConnected();
    doc["ble_device"] = goproBLE.deviceName();
    doc["active_mode"] = goproBLE.activeMode();   // 1000=video 1001=photo 1002=timelapse
    doc["recording"]   = goproBLE.isRecording();
    String body; serializeJson(doc, body);
    return sendJson(req, 200, body);
}

static esp_err_t handleConfigGet(httpd_req_t* req) {
    JsonDocument doc;
    doc["gopro_ssid"]  = _cfg->gopro_ssid;
    doc["bridge_ssid"] = _cfg->bridge_ssid;
    doc["gopro_ip"]    = _cfg->gopro_ip;
    doc["gopro_port"]  = _cfg->gopro_port;
    String body; serializeJson(doc, body);
    return sendJson(req, 200, body);
}

static esp_err_t handleConfigPost(httpd_req_t* req) {
    String body;
    if (!recvBody(req, body, 2048)) {
        return sendJson(req, 400, "{\"error\":\"bad request\"}");
    }

    JsonDocument doc;
    if (deserializeJson(doc, body)) {
        return sendJson(req, 400, "{\"error\":\"bad JSON\"}");
    }

    if (!doc["gopro_ssid"].isNull())  _cfg->gopro_ssid  = doc["gopro_ssid"].as<String>();
    if (!doc["gopro_pass"].isNull())  _cfg->gopro_pass  = doc["gopro_pass"].as<String>();
    if (!doc["bridge_ssid"].isNull()) _cfg->bridge_ssid = doc["bridge_ssid"].as<String>();
    if (!doc["bridge_pass"].isNull()) _cfg->bridge_pass = doc["bridge_pass"].as<String>();
    if (!doc["gopro_ip"].isNull())    _cfg->gopro_ip    = doc["gopro_ip"].as<String>();
    if (!doc["gopro_port"].isNull())  _cfg->gopro_port  = doc["gopro_port"].as<uint16_t>();

    bool saved = configSave(*_cfg);
    return sendJson(req, 200, saved ? "{\"ok\":true,\"note\":\"restart required\"}"
                                    : "{\"ok\":false,\"error\":\"save failed\"}");
}

static esp_err_t handleJsonProxy(httpd_req_t* req, const String& url) {
    HTTPClient http;
    http.begin(url);
    http.setTimeout(8000);
    int code = http.GET();
    if (code <= 0) {
        http.end();
        String body = String("{\"error\":\"GoPro unreachable\",\"code\":") + String(code) + "}";
        return sendJson(req, 503, body);
    }

    if (code != 200) {
        String body = http.getString();
        http.end();
        if (body.isEmpty()) body = "{\"error\":\"upstream error\"}";
        return sendJson(req, code, body);
    }

    int contentLen = http.getSize();
    WiFiClient* stream = http.getStreamPtr();
    const unsigned long deadline = millis() + 8000;  // GoPro répond vite; 8s couvre les lenteurs réseau

    httpd_resp_set_type(req, "application/json");
    addCors(req);

    Serial.printf("[HTTPS] GET %s -> %d  len=%d  heap=%u\n",
                  url.c_str(), code, contentLen, ESP.getFreeHeap());

    uint8_t buf[4096];
    if (contentLen > 0) {
        size_t remaining = static_cast<size_t>(contentLen);
        while (remaining > 0 && millis() < deadline) {
            if (!stream->connected()) break;
            size_t avail = stream->available();
            if (!avail) {
                delay(1);
                continue;
            }
            size_t toRead = min(avail, min(sizeof(buf), remaining));
            int r = stream->readBytes(buf, toRead);
            if (r <= 0) break;
            if (httpd_resp_send_chunk(req, reinterpret_cast<const char*>(buf), r) != ESP_OK) break;
            remaining -= static_cast<size_t>(r);
        }
    } else {
        while (millis() < deadline) {
            if (!stream->connected() && !stream->available()) break;
            size_t avail = stream->available();
            if (!avail) {
                delay(1);
                continue;
            }
            size_t toRead = min(avail, sizeof(buf));
            int r = stream->readBytes(buf, toRead);
            if (r <= 0) break;
            if (httpd_resp_send_chunk(req, reinterpret_cast<const char*>(buf), r) != ESP_OK) break;
        }
    }

    httpd_resp_send_chunk(req, nullptr, 0);
    http.end();
    return ESP_OK;
}

static esp_err_t handleBinProxy(httpd_req_t* req, const String& url,
                                const char* contentType, size_t maxBytes,
                                const String& disposition = String()) {
    HTTPClient http;
    http.begin(url);
    // 8s : la GoPro met parfois 5-6 s a servir le premier thumbnail apres
    // un reveil de la cam ou un changement de mode. Trop court = tout
    // tombe en 502 et le frontend affiche ⚠ partout.
    http.setTimeout(8000);

    int code = http.GET();
    if (code != 200) {
        Serial.printf("[HTTPS] BIN %s FAILED -> code=%d  free=%u  largest=%u\n",
                      url.c_str(), code,
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMaxAllocHeap());
        http.end();
        return sendJson(req, 502, "{\"error\":\"upstream error\"}");
    }

    int contentLen = http.getSize();
    if (contentLen <= 0) {
        Serial.printf("[HTTPS] BIN %s no content-length -> contentLen=%d\n",
                      url.c_str(), contentLen);
        http.end();
        return sendJson(req, 502, "{\"error\":\"unknown content length\"}");
    }

    WiFiClient* stream = http.getStreamPtr();
    size_t remaining = min(static_cast<size_t>(contentLen), maxBytes);
    unsigned long deadline = millis() + 20000;  // binaires (photos) plus lents mais GoPro réactive vite

    httpd_resp_set_type(req, contentType);
    addCors(req);
    if (disposition.length()) {
        httpd_resp_set_hdr(req, "Content-Disposition", disposition.c_str());
    }

    Serial.printf("[HTTPS] BIN %s -> %d  len=%d  heap=%u\n",
                  url.c_str(), code, contentLen, ESP.getFreeHeap());

    uint8_t buf[4096];
    while (remaining > 0 && millis() < deadline) {
        if (!stream->connected()) break;
        size_t avail = stream->available();
        if (!avail) {
            delay(1);
            continue;
        }
        size_t toRead = min(avail, min(sizeof(buf), remaining));
        int r = stream->readBytes(buf, toRead);
        if (r <= 0) break;
        if (httpd_resp_send_chunk(req, reinterpret_cast<const char*>(buf), r) != ESP_OK) break;
        remaining -= static_cast<size_t>(r);
    }

    httpd_resp_send_chunk(req, nullptr, 0);
    http.end();
    return ESP_OK;
}

static esp_err_t handleMediaList(httpd_req_t* req) {
    return handleJsonProxy(req, _baseUrl + "/gopro/media/list");
}

static esp_err_t handleCameraState(httpd_req_t* req) {
    return handleJsonProxy(req, _baseUrl + "/gopro/camera/state");
}

static esp_err_t handleThumbnail(httpd_req_t* req) {
    String path;
    if (!getQueryParam(req, "path", path)) {
        return sendJson(req, 400, "{\"error\":\"missing path\"}");
    }
    path = urlDecode(path);
    return handleBinProxy(req, _baseUrl + "/gopro/media/thumbnail?path=" + path,
                          "image/jpeg", 256 * 1024);
}

static esp_err_t handleDownload(httpd_req_t* req) {
    String path;
    if (!getQueryParam(req, "path", path)) {
        return sendJson(req, 400, "{\"error\":\"missing path\"}");
    }
    path = urlDecode(path);

    String low = path; low.toLowerCase();
    if (low.endsWith(".mp4") || low.endsWith(".lrv")) {
        return sendJson(req, 415, "{\"error\":\"Video download not supported\"}");
    }

    String filename = path;
    int slash = filename.lastIndexOf('/');
    if (slash >= 0) filename = filename.substring(slash + 1);

    String disp = "attachment; filename=\"" + filename + "\"";
    return handleBinProxy(req, _baseUrl + "/videos/DCIM/" + path,
                          "image/jpeg", 10 * 1024 * 1024, disp);
}

static esp_err_t handleDelete(httpd_req_t* req) {
    String path;
    if (!getQueryParam(req, "path", path)) {
        return sendJson(req, 400, "{\"error\":\"missing path\"}");
    }
    path = urlDecode(path);
    return handleJsonProxy(req, _baseUrl + "/gopro/media/delete/file?path=" + path);
}

static esp_err_t handleStatic(httpd_req_t* req) {
    String path = req->uri;
    int q = path.indexOf('?');
    if (q >= 0) path = path.substring(0, q);
    if (path == "/") path = "/index.html";
    if (path.indexOf("..") >= 0) {
        httpd_resp_set_status(req, HTTPD_400);
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "bad path", HTTPD_RESP_USE_STRLEN);
    }

    if (!LittleFS.exists(path)) {
        httpd_resp_set_status(req, HTTPD_404);
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "not found", HTTPD_RESP_USE_STRLEN);
    }

    File f = LittleFS.open(path, "r");
    if (!f) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "open failed", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, mimeTypeForPath(path));
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char buf[1024];
    while (true) {
        size_t r = f.read(reinterpret_cast<uint8_t*>(buf), sizeof(buf));
        if (r == 0) break;
        if (httpd_resp_send_chunk(req, buf, r) != ESP_OK) {
            f.close();
            return ESP_FAIL;
        }
    }
    f.close();
    return httpd_resp_send_chunk(req, nullptr, 0);
}

static esp_err_t registerHandler(httpd_handle_t server, const char* uri,
                                 httpd_method_t method, esp_err_t (*handler)(httpd_req_t*)) {
    httpd_uri_t h = {};
    h.uri = uri;
    h.method = method;
    h.handler = handler;
    h.user_ctx = nullptr;
    return httpd_register_uri_handler(server, &h);
}

bool httpsStart(BridgeConfig& cfg) {
    _cfg = &cfg;
    _baseUrl = "http://" + cfg.gopro_ip + ":" + String(cfg.gopro_port);

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.httpd.uri_match_fn = httpd_uri_match_wildcard;
    conf.httpd.max_uri_handlers = 25;
    conf.httpd.stack_size = 12288;    // 8192 trop juste pour mbedTLS sous pression mémoire
    // 1 seule session TLS : choix de stabilite. Le navigateur queue tout sur
    // une seule connexion via HTTP/1.1 keepalive. Galerie un peu plus lente
    // mais zero risque de ALLOC_FAILED (-0x7F00) lie au heap fragmente.
    conf.httpd.max_open_sockets = 1;
    conf.httpd.lru_purge_enable = true; // ferme les sockets idle quand le pool est plein

    // TCP keepalive : sans ça, quand le client PC coupe son WiFi abruptement
    // (pas de FIN reçu), les sockets HTTPS restent zombies indéfiniment et
    // bouchent les slots. Sondes 5 s après inactivité, 3 retries -> socket
    // fermée au bout de ~20 s en cas de client disparu.
    conf.httpd.keep_alive_enable   = true;
    conf.httpd.keep_alive_idle     = 5;
    conf.httpd.keep_alive_interval = 5;
    conf.httpd.keep_alive_count    = 3;

    // Timeouts de recv/send raccourcis pour ne pas bloquer un slot pendant
    // qu'un client lent / mort retient un transfert (defaut = 5 s).
    conf.httpd.recv_wait_timeout = 3;
    conf.httpd.send_wait_timeout = 3;
    conf.port_secure = 443;
    conf.servercert = reinterpret_cast<const unsigned char*>(kServerCertPem);
    conf.servercert_len = kServerCertPemLen;
    conf.prvtkey_pem = reinterpret_cast<const unsigned char*>(kServerKeyPem);
    conf.prvtkey_len = kServerKeyPemLen;

    esp_err_t err = httpd_ssl_start(&_https, &conf);
    if (err != ESP_OK) {
        Serial.printf("[HTTPS] start failed: %d\n", (int)err);
        return false;
    }

    auto reg = [&](const char* uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t*)) {
        esp_err_t r = registerHandler(_https, uri, method, handler);
        if (r != ESP_OK) {
            Serial.printf("[HTTPS] register failed: %s (%d)\n", uri, (int)r);
        }
        return r;
    };

    reg("/ble/shutter/on",     HTTP_POST,   handleBleShutterOn);
    reg("/ble/shutter/off",    HTTP_POST,   handleBleShutterOff);
    reg("/ble/mode/video",     HTTP_POST,   handleBleModeVideo);
    reg("/ble/mode/photo",     HTTP_POST,   handleBleModePhoto);
    reg("/ble/mode/timelapse", HTTP_POST,   handleBleModeTimelapse);
    reg("/ble/hilight",        HTTP_POST,   handleBleHilight);
    reg("/ble/sleep",          HTTP_POST,   handleBleSleep);
    reg("/ble/status",         HTTP_GET,    handleBleStatus);

    reg("/status",             HTTP_GET,    handleStatus);
    reg("/config",             HTTP_GET,    handleConfigGet);
    reg("/config",             HTTP_POST,   handleConfigPost);

    reg("/foc/button-mode",    HTTP_GET,    handleFocBtnModeGet);
    reg("/foc/button-mode",    HTTP_POST,   handleFocBtnModePost);
    reg("/foc/step",           HTTP_GET,    handleFocStepGet);
    reg("/foc/step",           HTTP_POST,   handleFocStepPost);
    reg("/cert.crt",           HTTP_GET,    handleCertDownload);

    reg("/gopro/media/list",   HTTP_GET,    handleMediaList);
    reg("/gopro/camera/state", HTTP_GET,    handleCameraState);
    reg("/gopro/media/thumbnail", HTTP_GET, handleThumbnail);
    reg("/download",           HTTP_GET,    handleDownload);
    reg("/gopro/media/delete", HTTP_DELETE, handleDelete);

    reg("/*",                  HTTP_OPTIONS, handleOptions);
    reg("/*",                  HTTP_GET,     handleStatic);

    Serial.println("[HTTPS] server started on :443");
    return true;
}
