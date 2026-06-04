#include "bridge_ble.h"
#include "bridge_wifi.h"

GoproBLE goproBLE;

// Toutes les 30 s on re-pousse le preset actif sur la GoPro. Sans ca, si
// quelqu'un touche le bouton Mode physique ou si la cam fait un reset
// silencieux, le mode peut basculer. C'est peu cher en bande BLE (1 commande
// de 5 octets) et complete bien le keepalive.
static const unsigned long _MODE_RESEND_MS = 30000;

// ── UUIDs ──────────────────────────────────────
static NimBLEUUID svcUUID   ((uint16_t)0xFEA6);
static NimBLEUUID cmdReqUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");
static NimBLEUUID cmdRspUUID("b5f90073-aa8d-11e3-9046-0002a5d5c51b");
static NimBLEUUID setReqUUID("b5f90074-aa8d-11e3-9046-0002a5d5c51b");
static NimBLEUUID setRspUUID("b5f90075-aa8d-11e3-9046-0002a5d5c51b");

// ── Fixed command bytes ─────────────────────────
static const uint8_t SHUTTER_ON[]  = {0x03, 0x01, 0x01, 0x01};
static const uint8_t SHUTTER_OFF[] = {0x03, 0x01, 0x01, 0x00};
static const uint8_t HILIGHT[]     = {0x01, 0x18};
static const uint8_t SLEEP[]       = {0x01, 0x05};
static const uint8_t WIFI_ON[]     = {0x03, 0x17, 0x01, 0x01};
static const uint8_t WIFI_OFF[]    = {0x03, 0x17, 0x01, 0x00};
static const uint8_t KEEPALIVE[]   = {0x03, 0x5B, 0x01, 0x3D};

// ── Client disconnect callback ─────────────────
class BLECb : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient*) override {
        goproBLE._connected = false;
        Serial.println("[BLE] Disconnected.");
    }
};

// ── Scan callback — stops as soon as GoPro found ─
class BLEScanCb : public NimBLEAdvertisedDeviceCallbacks {
    GoproBLE* _self;
public:
    explicit BLEScanCb(GoproBLE* s) : _self(s) {}
    void onResult(NimBLEAdvertisedDevice* dev) override {
        if (_self->_foundDevice) return;
        bool bySvc  = dev->haveServiceUUID() && dev->isAdvertisingService(svcUUID);
        String name = dev->getName().c_str();
        bool byName = !_self->_filter.isEmpty() && name.startsWith(_self->_filter);
        if (bySvc || byName) {
            _self->_devName    = name.length() ? name : dev->getAddress().toString().c_str();
            _self->_foundDevice = new NimBLEAdvertisedDevice(*dev);
            Serial.printf("[BLE] Found: \"%s\"  MAC:%s  (svc=%d name=%d)\n",
                          _self->_devName.c_str(),
                          dev->getAddress().toString().c_str(),
                          (int)bySvc, (int)byName);
            NimBLEDevice::getScan()->stop();
        }
    }
};

static void notifyCb(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
    Serial.printf("[BLE] Notify:");
    for (size_t i = 0; i < len; i++) Serial.printf(" %02X", d[i]);
    if (len >= 3 && d[2] == 0x00) Serial.print("  OK");
    Serial.println();
}

// ── Background task ────────────────────────────
void GoproBLE::_task(void* pv) {
    auto* self = static_cast<GoproBLE*>(pv);

    while (true) {
        if (!self->_connected) {
            Serial.println("[BLE] Scanning for GoPro (service 0xFEA6)...");

            self->_foundDevice = nullptr;
            NimBLEScan* scan = NimBLEDevice::getScan();
            scan->setAdvertisedDeviceCallbacks(new BLEScanCb(self), false);
            scan->setActiveScan(true);
            scan->setInterval(1349);
            scan->setWindow(449);
            scan->start(8, false);   // blocks up to 8 s, or until stop() called in callback
            scan->clearResults();

            NimBLEAdvertisedDevice* target = self->_foundDevice;
            self->_foundDevice = nullptr;

            if (target) {
                if (self->_client) {
                    if (self->_client->isConnected()) self->_client->disconnect();
                    NimBLEDevice::deleteClient(self->_client);
                    self->_client = nullptr;
                    self->_cmdReq = nullptr;
                    self->_setReq = nullptr;
                }

                self->_client = NimBLEDevice::createClient();
                self->_client->setClientCallbacks(new BLECb());

                Serial.printf("[BLE] Connecting to %s ...\n", self->_devName.c_str());
                if (self->_client->connect(target)) {
                    auto* svc = self->_client->getService(svcUUID);
                    if (svc) {
                        self->_cmdReq = svc->getCharacteristic(cmdReqUUID);
                        self->_setReq = svc->getCharacteristic(setReqUUID);

                        auto* cmdRsp = svc->getCharacteristic(cmdRspUUID);
                        if (cmdRsp && cmdRsp->canNotify()) cmdRsp->subscribe(true, notifyCb);

                        auto* setRsp = svc->getCharacteristic(setRspUUID);
                        if (setRsp && setRsp->canNotify()) setRsp->subscribe(true, notifyCb);

                        if (self->_cmdReq) {
                            self->_connected = true;
                            self->_lastKA    = millis();
                            Serial.printf("[BLE] Connected to %s  (mode actif: %u)\n",
                                          self->_devName.c_str(), (unsigned)self->_activeMode);
                            // Preset AVANT le WiFi : la cam est receptive juste
                            // apres la connexion BLE. Si on l'envoie pendant
                            // qu'elle demarre son AP, la commande se perd.
                            bool pok = self->loadPresetGroup(self->_activeMode);
                            Serial.printf("[BLE] preset initial %u -> ok=%d\n",
                                          (unsigned)self->_activeMode, (int)pok);
                            vTaskDelay(pdMS_TO_TICKS(500));
                            self->wifiApOn();                          // active l'AP WiFi de la GoPro
                            vTaskDelay(pdMS_TO_TICKS(8000));           // laisse l'AP GoPro se stabiliser
                            wifiStaConnect();                          // l'ESP se connecte a l'AP GoPro
                            // 1er resend mode rapide (~5 s) au cas ou le preset
                            // initial n'aurait pas pris pendant la transition.
                            self->_lastMode = millis() - (_MODE_RESEND_MS - 5000);
                        }
                    }
                } else {
                    Serial.println("[BLE] Connection failed.");
                }
                delete target;
            }
        }

        // Keep-alive every 3 s
        if (self->_connected && millis() - self->_lastKA > 3000) {
            self->_sendSetting(KEEPALIVE, sizeof(KEEPALIVE));
            self->_lastKA = millis();
        }

        // Re-pousse le preset mode (photo/video) periodiquement pour ramener
        // la cam si elle a glisse vers un autre mode. Skip pendant un
        // enregistrement actif : recharger le preset arrete la prise.
        if (self->_connected && !self->_recording &&
            millis() - self->_lastMode > _MODE_RESEND_MS) {
            self->loadPresetGroup(self->_activeMode);
            self->_lastMode = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(self->_connected ? 1000 : 5000));
    }
}

// ── Public API ─────────────────────────────────
void GoproBLE::begin(const char* nameFilter) {
    _filter = nameFilter;
    _bleMutex = xSemaphoreCreateMutex();
    NimBLEDevice::init("GoPro-Bridge");
    NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEDevice::setMTU(517);
    xTaskCreatePinnedToCore(_task, "ble_task", 16384, this, 1, NULL, 0);
}

bool GoproBLE::_send(const uint8_t* data, size_t len) {
    if (!_connected || !_cmdReq) return false;
    if (_bleMutex) xSemaphoreTake(_bleMutex, portMAX_DELAY);
    bool ok = _cmdReq->writeValue(data, len, true);  // wait for ack — needed for commands
    if (_bleMutex) xSemaphoreGive(_bleMutex);
    return ok;
}

bool GoproBLE::_sendSetting(const uint8_t* data, size_t len) {
    if (!_connected || !_setReq) return false;
    if (_bleMutex) xSemaphoreTake(_bleMutex, portMAX_DELAY);
    bool ok = _setReq->writeValue(data, len, false);  // no response wait — avoids BLE/WiFi coex timeout
    if (_bleMutex) xSemaphoreGive(_bleMutex);
    return ok;
}

bool GoproBLE::shutterOn() {
    bool ok = _send(SHUTTER_ON, sizeof(SHUTTER_ON));
    // Marque l'enregistrement uniquement en video/timelapse (pas en photo
    // ou la commande est one-shot).
    if (ok && (_activeMode == 1000 || _activeMode == 1002)) _recording = true;
    Serial.printf("[BLE] shutter ON  -> ok=%d  activeMode=%u  recording=%d\n",
                  (int)ok, (unsigned)_activeMode, (int)_recording);
    return ok;
}

bool GoproBLE::shutterOff() {
    bool ok = _send(SHUTTER_OFF, sizeof(SHUTTER_OFF));
    if (ok) _recording = false;
    Serial.printf("[BLE] shutter OFF -> ok=%d  recording=%d\n",
                  (int)ok, (int)_recording);
    return ok;
}

bool GoproBLE::isRecording() const { return _recording; }
bool GoproBLE::hilight()      { return _send(HILIGHT,     sizeof(HILIGHT));     }
bool GoproBLE::cameraSleep()  { return _send(SLEEP,       sizeof(SLEEP));       }
bool GoproBLE::wifiApOn()     { return _send(WIFI_ON,     sizeof(WIFI_ON));     }
bool GoproBLE::wifiApOff()    { return _send(WIFI_OFF,    sizeof(WIFI_OFF));    }

bool GoproBLE::loadPresetGroup(uint16_t groupId) {
    uint8_t cmd[] = {0x04, 0x3E, 0x02,
                     (uint8_t)(groupId >> 8),
                     (uint8_t)(groupId & 0xFF)};
    return _send(cmd, sizeof(cmd));
}

void GoproBLE::setActiveMode(uint16_t groupId) {
    _activeMode = groupId;
    _lastMode   = millis();
    _recording  = false;   // changement de mode -> enregistrement implicitement coupe
    loadPresetGroup(groupId);
}
