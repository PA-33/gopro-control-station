#pragma once
#include <Arduino.h>

struct BridgeConfig {
    String   gopro_ssid  = "GP26211187";
    String   gopro_pass  = "GBD-V2Q-dW8";
    String   bridge_ssid = "GoPro-Bridge";
    String   bridge_pass = "gopro1234";
    String   gopro_ip    = "10.5.5.9";
    uint16_t gopro_port  = 8080;
};

bool configLoad(BridgeConfig& cfg);
bool configSave(const BridgeConfig& cfg);
