#include "bridge_config.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

static const char* CONFIG_PATH = "/config.json";

bool configLoad(BridgeConfig& cfg) {
    if (!LittleFS.exists(CONFIG_PATH)) return false;
    File f = LittleFS.open(CONFIG_PATH, "r");
    if (!f) return false;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;
    if (!doc["gopro_ssid"].isNull())  cfg.gopro_ssid  = doc["gopro_ssid"].as<String>();
    if (!doc["gopro_pass"].isNull())  cfg.gopro_pass  = doc["gopro_pass"].as<String>();
    if (!doc["bridge_ssid"].isNull()) cfg.bridge_ssid = doc["bridge_ssid"].as<String>();
    if (!doc["bridge_pass"].isNull()) cfg.bridge_pass = doc["bridge_pass"].as<String>();
    if (!doc["gopro_ip"].isNull())    cfg.gopro_ip    = doc["gopro_ip"].as<String>();
    if (!doc["gopro_port"].isNull())  cfg.gopro_port  = doc["gopro_port"].as<uint16_t>();
    return true;
}

bool configSave(const BridgeConfig& cfg) {
    File f = LittleFS.open(CONFIG_PATH, "w");
    if (!f) return false;
    JsonDocument doc;
    doc["gopro_ssid"]  = cfg.gopro_ssid;
    doc["gopro_pass"]  = cfg.gopro_pass;
    doc["bridge_ssid"] = cfg.bridge_ssid;
    doc["bridge_pass"] = cfg.bridge_pass;
    doc["gopro_ip"]    = cfg.gopro_ip;
    doc["gopro_port"]  = cfg.gopro_port;
    serializeJson(doc, f);
    f.close();
    return true;
}
