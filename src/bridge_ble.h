#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

class GoproBLE {
public:
    void   begin(const char* nameFilter = "GP");

    bool   isConnected() const { return _connected; }
    String deviceName()  const { return _devName; }

    bool   shutterOn();
    bool   shutterOff();
    bool   loadPresetGroup(uint16_t groupId);  // 1000=video 1001=photo 1002=timelapse
    bool   hilight();
    bool   cameraSleep();
    bool   wifiApOn();
    bool   wifiApOff();

    // Memorise le dernier preset choisi et le re-pousse periodiquement
    // (cf. _MODE_RESEND_MS) pour ramener la GoPro dans le mode voulu si
    // elle a ete bougee physiquement / s'est remise par defaut.
    void   setActiveMode(uint16_t groupId);

    // True entre un shutter/on et un shutter/off en mode video/timelapse.
    // Utilise par bridge_main pour faire clignoter la LED rouge et par le
    // resend periodique pour ne pas couper l'enregistrement.
    bool   isRecording() const;

    // Preset group actuellement actif (1000=video 1001=photo 1002=timelapse).
    // Sert a la page web pour se synchroniser sur le vrai mode de la cam.
    uint16_t activeMode() const { return _activeMode; }

    friend class BLECb;
    friend class BLEScanCb;

private:
    bool _send(const uint8_t* data, size_t len);
    bool _sendSetting(const uint8_t* data, size_t len);

    static void _task(void* pv);

    NimBLEAdvertisedDevice*     _foundDevice = nullptr;
    NimBLEClient*               _client   = nullptr;
    NimBLERemoteCharacteristic* _cmdReq   = nullptr;
    NimBLERemoteCharacteristic* _setReq   = nullptr;
    String                      _devName;
    volatile bool               _connected = false;
    volatile bool               _recording = false;
    unsigned long               _lastKA    = 0;
    unsigned long               _lastMode  = 0;
    uint16_t                    _activeMode = 1001;   // photo par defaut
    String                      _filter;
    // Serialise toutes les ecritures GATT : la tache BLE (keepalive, resend
    // mode) et la tache HTTPS (commandes shutter) ecrivent sinon en parallele
    // et se telescopent -> commande perdue (ex: stop video qui ne stoppe pas).
    SemaphoreHandle_t           _bleMutex  = nullptr;
};

extern GoproBLE goproBLE;
