#pragma once
#include <ESPAsyncWebServer.h>
#include "bridge_config.h"

// HTTP server on :80 for cert download and HTTPS redirect.
void serverStart(AsyncWebServer& server, BridgeConfig& cfg);
