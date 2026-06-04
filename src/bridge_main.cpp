/*M1:[251;320], M2[-265;-345]*/



#include <Arduino.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include "bridge_config.h"
#include "bridge_wifi.h"
#include "bridge_ble.h"
#include "bridge_foc.h"
#include "bridge_current.h"
#include "bridge_https.h"
// ESPAsyncWebServer / bridge_server retires : le serveur HTTP port 80
// bouffait ~20-30 ko de heap qui fragmentait le bloc max contigu sous le
// seuil mbedTLS. Le cert est maintenant servi via HTTPS (/cert.crt).

static BridgeConfig   cfg;

#ifndef BRIDGE_WS2815_PIN
#define BRIDGE_WS2815_PIN 4
#endif

#ifndef BRIDGE_WS2815_COUNT
#define BRIDGE_WS2815_COUNT 7
#endif

#ifndef BRIDGE_WS2815_BRIGHTNESS
#define BRIDGE_WS2815_BRIGHTNESS 80
#endif

#ifndef BRIDGE_WS2815_PIXEL_TYPE
#define BRIDGE_WS2815_PIXEL_TYPE (NEO_RGB + NEO_KHZ800)
#endif

#ifndef BRIDGE_BAT_ADC_PIN
#define BRIDGE_BAT_ADC_PIN 9
#endif

#ifndef BRIDGE_BAT_DIVIDER
#define BRIDGE_BAT_DIVIDER 5.546644154f
#endif

#ifndef BRIDGE_BAT_MAX_V
#define BRIDGE_BAT_MAX_V 12.6f
#endif

#ifndef BRIDGE_BAT_MIN_V
#define BRIDGE_BAT_MIN_V 9.3f
#endif

#ifndef BRIDGE_BAT_BLINK_V
#define BRIDGE_BAT_BLINK_V 9.0f
#endif

#ifndef BRIDGE_BAT_ALL_BLINK_V
#define BRIDGE_BAT_ALL_BLINK_V 8.8f
#endif

static Adafruit_NeoPixel statusStrip(BRIDGE_WS2815_COUNT, BRIDGE_WS2815_PIN, BRIDGE_WS2815_PIXEL_TYPE);

static uint32_t batteryMv = 0;
static float batteryV = 0.0f;
static uint8_t batteryRedCount = 0;
static bool batteryBlink = false;
static bool batteryAllBlink = false;
static bool blinkOn = true;
static uint32_t shutterFlashUntil = 0;
static bool stripInitialized = false;  // forcé à false par le callback de calibration


static float readBatteryVoltage(uint32_t* mvOut) {
    uint32_t mv = analogReadMilliVolts(BRIDGE_BAT_ADC_PIN);
    if (mvOut) {
        *mvOut = mv;
    }
    return (mv / 1000.0f) * BRIDGE_BAT_DIVIDER;
}

void bridgeNotifyShutter() {
    shutterFlashUntil = millis() + 500;
}

static bool shutterFlashActive() {
    return (int32_t)(millis() - shutterFlashUntil) < 0;
}

static void updateBatteryState(float voltage) {
    const float span = BRIDGE_BAT_MAX_V - BRIDGE_BAT_MIN_V;
    const float step = span / 3.0f;

    uint8_t redCount = 0;
    if (voltage < (BRIDGE_BAT_MAX_V - step)) {
        redCount = 1;
    }
    if (voltage < (BRIDGE_BAT_MAX_V - 2.0f * step)) {
        redCount = 2;
    }
    if (voltage <= BRIDGE_BAT_MIN_V) {
        redCount = 3;
    }

    batteryRedCount = redCount;
    batteryBlink = voltage < BRIDGE_BAT_BLINK_V;
    batteryAllBlink = voltage < BRIDGE_BAT_ALL_BLINK_V;
}


static void showBootRainbow(uint32_t durationMs, uint16_t stepMs) {
    if (statusStrip.numPixels() == 0) {
        return;
    }

    uint32_t start = millis();
    uint32_t hue = 0;

    while (millis() - start < durationMs) {
        for (uint16_t i = 0; i < statusStrip.numPixels(); ++i) {
            uint32_t pixelHue = hue + (i * 65536UL / statusStrip.numPixels());
            uint32_t color = statusStrip.gamma32(statusStrip.ColorHSV(pixelHue));
            statusStrip.setPixelColor(i, color);
        }
        statusStrip.show();
        hue += 512;
        delay(stepMs);
    }
}

static void onCalibrationActive(bool active) {
    if (active) {
        for (uint16_t i = 0; i < statusStrip.numPixels(); ++i) {
            statusStrip.setPixelColor(i, statusStrip.Color(64, 10, 30));  // rose
        }
        statusStrip.show();
    } else {
        stripInitialized = false;  // force un redessin complet à la fin de la calibration
    }
}

static void updateStatusStrip(bool bleConnected,
                              bool apClientConnected,
                              uint8_t batRedCount,
                              bool batBlink,
                              bool allBlink,
                              bool blinkState,
                              bool shutterFlash,
                              bool m1Off,
                              bool m2Off,
                              bool recording) {
    static bool lastBle = false;
    static bool lastAp  = false;
    static uint8_t lastBatRed = 0;
    static bool lastBatBlink = false;
    static bool lastAllBlink = false;
    static bool lastBlinkOn = false;
    static bool lastShutter = false;
    static bool lastM1Off = false;
    static bool lastM2Off = false;
    static bool lastRec  = false;

    if (stripInitialized && bleConnected == lastBle && apClientConnected == lastAp &&
        batRedCount == lastBatRed && batBlink == lastBatBlink &&
        allBlink == lastAllBlink && blinkState == lastBlinkOn &&
        shutterFlash == lastShutter &&
        m1Off == lastM1Off && m2Off == lastM2Off &&
        recording == lastRec) {
        return;
    }

    stripInitialized = true;
    lastBle = bleConnected;
    lastAp  = apClientConnected;
    lastBatRed = batRedCount;
    lastBatBlink = batBlink;
    lastAllBlink = allBlink;
    lastBlinkOn = blinkState;
    lastShutter = shutterFlash;
    lastM1Off = m1Off;
    lastM2Off = m2Off;
    lastRec  = recording;

    statusStrip.clear();

    if (allBlink) {
        if (blinkState) {
            for (uint16_t i = 0; i < statusStrip.numPixels(); ++i) {
                statusStrip.setPixelColor(i, statusStrip.Color(64, 0, 0));
            }
        }
        statusStrip.show();
        return;
    }

    const uint8_t bleR = bleConnected ? 0 : 64;
    const uint8_t bleG = bleConnected ? 64 : 0;
    statusStrip.setPixelColor(0, statusStrip.Color(bleR, bleG, 0));

    const uint8_t apR = apClientConnected ? 0 : 64;
    const uint8_t apG = apClientConnected ? 64 : 0;
    statusStrip.setPixelColor(1, statusStrip.Color(apR, apG, 0));

    // LED 2 : statut moteurs.  jaune = M1 stoppé, bleu = M2 stoppé,
    // rouge = les deux stoppés, éteinte = les deux actifs.
    if (m1Off && m2Off) {
        statusStrip.setPixelColor(2, statusStrip.Color(64,  0,  0));   // rouge
    } else if (m1Off) {
        statusStrip.setPixelColor(2, statusStrip.Color(64, 64,  0));   // jaune
    } else if (m2Off) {
        statusStrip.setPixelColor(2, statusStrip.Color( 0,  0, 64));   // bleu
    }
    // sinon : LED 2 reste éteinte (déjà fait par statusStrip.clear())

    if (statusStrip.numPixels() >= 3) {
        uint16_t batStart = statusStrip.numPixels() - 3;
        if (batBlink) {
            if (blinkState) {
                for (uint16_t i = 0; i < 3; ++i) {
                    statusStrip.setPixelColor(batStart + i, statusStrip.Color(64, 0, 0));
                }
            }
        } else {
            uint8_t clampedRed = min<uint8_t>(3, batRedCount);
            for (uint16_t i = 0; i < 3; ++i) {
                bool red = i >= (3 - clampedRed);
                uint8_t r = red ? 64 : 0;
                uint8_t g = red ? 0 : 64;
                statusStrip.setPixelColor(batStart + i, statusStrip.Color(r, g, 0));
            }
        }
    }

    // LED 3 : prioritaire = enregistrement video (rouge clignotant via
    // blinkState), sinon flash obturateur photo (blanc), sinon eteinte.
    if (statusStrip.numPixels() >= 4) {
        if (recording) {
            if (blinkState) statusStrip.setPixelColor(3, statusStrip.Color(64, 0, 0));
        } else if (shutterFlash) {
            statusStrip.setPixelColor(3, statusStrip.Color(64, 64, 64));
        }
    }

    statusStrip.show();
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // NOTE: RGB_BUILTIN/LED_BUILTIN is GPIO 48 on this board, which is the same
    // pin as DRV_EN. Driving it would put the DRV8313 in an undefined state.
    // Status is shown on the WS2815 strip (GPIO 4) instead.

    statusStrip.begin();
    statusStrip.setBrightness(BRIDGE_WS2815_BRIGHTNESS);
    statusStrip.clear();
    statusStrip.show();

    showBootRainbow(1500, 40);

    // ── ADC2 init AVANT le FOC : la calibration des butées va lire les
    //    sondes courant pendant que le Wi-Fi n'est pas encore actif (ADC2
    //    100% fiable, cf. esp32_adc2_wifi.md).
    bridgeCurrentInit();

    bridgeFocSetup();
    bridgeFocSetCalCallback(onCalibrationActive);

    analogReadResolution(12);
    analogSetPinAttenuation(BRIDGE_BAT_ADC_PIN, ADC_11db);

    // ── LittleFS ──────────────────────────────
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] LittleFS mount failed – formatting...");
        LittleFS.format();
        LittleFS.begin(false);
    }
    Serial.println("[FS] LittleFS mounted.");

    // ── Config ────────────────────────────────
    if (configLoad(cfg)) {
        Serial.println("[Config] Loaded from flash.");
    } else {
        Serial.println("[Config] Using defaults.");
    }
    Serial.printf("[Config] GoPro SSID: \"%s\"  Bridge SSID: \"%s\"\n",
                  cfg.gopro_ssid.c_str(), cfg.bridge_ssid.c_str());

    // ── WiFi AP+STA ───────────────────────────
    wifiStart(cfg);

    // ── BLE ───────────────────────────────────
    goproBLE.begin("GP");

    // ── HTTPS server only ─────────────────────
    if (!httpsStart(cfg)) {
        Serial.println("[HTTPS] Failed to start.");
    }

    Serial.printf("\n[Ready] Connect to WiFi \"%s\" then open http://192.168.4.1\n"
                  "         Install the cert, then use https://192.168.4.1\n\n",
                  cfg.bridge_ssid.c_str());

    // Lance le monitor périodique de surcourant maintenant que le Wi-Fi est
    // démarré (lectures avec retry pour encaisser le partage Wi-Fi/ADC2).
    bridgeCurrentStartMonitor();

    batteryV = readBatteryVoltage(&batteryMv);
    updateBatteryState(batteryV);
    updateStatusStrip(false, false, batteryRedCount, batteryBlink, batteryAllBlink, true, false,
                      false, false, false);
}

void loop() {
    // WiFi STA reconnect watchdog
    wifiWatchdog();

    bridgeFocLoop();

    // blinkOn toggle alimente : (1) blink batterie critique, (2) clignote rouge
    // LED 3 pendant un enregistrement video. Periode 500 ms pour le recording
    // (plus calme/cinema) - si batterie critique aussi, 300 ms l'emporte.
    static unsigned long blinkTs = 0;
    bool recording = goproBLE.isRecording();
    if (batteryBlink || batteryAllBlink) {
        if (millis() - blinkTs > 300) { blinkTs = millis(); blinkOn = !blinkOn; }
    } else if (recording) {
        if (millis() - blinkTs > 500) { blinkTs = millis(); blinkOn = !blinkOn; }
    } else {
        blinkOn = true;
    }

    static unsigned long stripTs = 0;
    if (millis() - stripTs > 200) {
        stripTs = millis();
        updateStatusStrip(goproBLE.isConnected(),
                  wifiApHasClients(),
                  batteryRedCount,
                  batteryBlink,
                  batteryAllBlink,
                  blinkOn,
                  shutterFlashActive(),
                  !bridgeFocIsMotorEnabled(1),
                  !bridgeFocIsMotorEnabled(2),
                  recording);
    }

    static unsigned long adcTs = 0;
    if (millis() - adcTs > 1000) {
        adcTs = millis();
        batteryV = readBatteryVoltage(&batteryMv);
        updateBatteryState(batteryV);
        // Serial.printf("[BAT] adc=%lumV v=%.2fV red=%u blink=%d all=%d\n",
        //               (unsigned long)batteryMv,
        //               batteryV,
        //               (unsigned)batteryRedCount,
        //               batteryBlink ? 1 : 0,
        //               batteryAllBlink ? 1 : 0);
    }

    // Heap watch : free = libre instantané, min = plus bas atteint depuis boot,
    // largest = plus gros bloc contigu (mbedTLS a besoin de ~30 ko contigus
    // par session TLS). Si largest tombe sous 30 ko -> ALLOC_FAILED en HTTPS.
    static unsigned long heapTs = 0;
    if (millis() - heapTs > 5000) {
        heapTs = millis();
        Serial.printf("[HEAP] free=%u  min=%u  largest=%u\n",
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMinFreeHeap(),
                      (unsigned)ESP.getMaxAllocHeap());
    }
}
