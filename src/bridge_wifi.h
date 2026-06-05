#pragma once
#include <Arduino.h>
#include "bridge_config.h"

void   wifiStart(const BridgeConfig& cfg);  // AP only
void   wifiStaConnect();                    // start STA (called after BLE wifiApOn)
bool   wifiStaConnected();
String wifiStaIP();
bool   wifiApHasClients();
void   wifiWatchdog();                      // call from loop()
