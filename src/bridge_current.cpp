#include "bridge_current.h"
#include "bridge_foc.h"
#include <Arduino.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

// ESP32-S3 mapping ADC2 :
//   GPIO15 -> ADC2_CH4 (sonde courant moteur 1)
//   GPIO16 -> ADC2_CH5 (sonde courant moteur 2)
#define M1_GPIO         15
#define M2_GPIO         16
#define M1_ADC_CHAN     ADC_CHANNEL_4
#define M2_ADC_CHAN     ADC_CHANNEL_5

// Atténuation choisie pour la lecture (pleine échelle ~0..3.1 V sur S3).
#define ADC_ATTEN       ADC_ATTEN_DB_12

// Sonde : INA180A1 (gain 20 V/V) + shunt 50 mOhm  =>  1.0 V/A
//   I_motor [A] = V_out [V] / (R_shunt * G) = V_mV / 1000
// soit 1 mV de sortie = 1 mA de courant moteur.
#define SHUNT_OHM       0.05f
#define INA_GAIN        20.0f
#define MV_PER_AMP      (SHUNT_OHM * INA_GAIN * 1000.0f)   // = 1000 mV/A

// ── Protection surcourant ──────────────────────────────────────────────────
// Mesures repos : 0 raw, holding équilibré : 0..4 raw, stall butée : 800..900.
// Seuil 400 raw ≈ 0.3 A : 100x au-dessus du holding, 2x sous le stall.
// N=5 lectures à 200 ms = 1 s avant déclenchement → filtre les transitoires
// d'accélération sans laisser le moteur chauffer.
#define STALL_RAW_THRESH    650
#define STALL_CONSECUTIVE   5

// Période de lecture / impression au moniteur série.
#define READ_PERIOD_MS  200

// Retry pattern décrit dans esp32_adc2_wifi.md : ADC2 partage le bloc avec le
// Wi-Fi, donc adc_oneshot_read() peut renvoyer une erreur transitoire pendant
// que le Wi-Fi tient le mutex. On retente jusqu'à MAX_RETRIES avant d'abandonner
// pour ce cycle (le driver thermique reste filet de sécurité primaire).
#define MAX_RETRIES     10

static adc_oneshot_unit_handle_t adc2 = nullptr;
static adc_cali_handle_t         cali = nullptr;   // null => pas de calib, on retombe sur l'approx linéaire

static int  m1_streak  = 0;
static int  m2_streak  = 0;
static bool m1_tripped = false;
static bool m2_tripped = false;

void bridgeCurrentClearStall(int idx) {
    if (idx == 1) { m1_streak = 0; m1_tripped = false; }
    else if (idx == 2) { m2_streak = 0; m2_tripped = false; }
}

static void evalStall(int idx, int raw, bool ok, int& streak, bool& tripped) {
    if (!ok) return;                    // lecture ratée : on n'incrémente ni ne reset
    if (tripped) return;                // déjà coupé, attend le réarmement utilisateur
    if (raw > STALL_RAW_THRESH) {
        if (++streak >= STALL_CONSECUTIVE) {
            tripped = true;
            Serial.printf("[STALL] M%d trip: raw=%d > %d sur %d lectures consécutives\n",
                          idx, raw, STALL_RAW_THRESH, streak);
            bridgeFocDisableMotor(idx);
        }
    } else {
        streak = 0;
    }
}

static bool readWithRetry(adc_channel_t ch, int& raw) {
    int attempts = 0;
    esp_err_t err;
    do {
        err = adc_oneshot_read(adc2, ch, &raw);
        if (err != ESP_OK) vTaskDelay(pdMS_TO_TICKS(1));
    } while (err != ESP_OK && ++attempts < MAX_RETRIES);
    return err == ESP_OK;
}

static int rawToMv(int raw) {
    int mv = 0;
    if (cali && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) return mv;
    // Fallback approximation si la calibration n'a pas pu être créée.
    return (int)((float)raw * 3100.0f / 4095.0f);
}

static void currentTask(void*) {
    while (true) {
        int r1 = 0, r2 = 0;
        bool ok1 = readWithRetry(M1_ADC_CHAN, r1);
        bool ok2 = readWithRetry(M2_ADC_CHAN, r2);
        int v1 = ok1 ? rawToMv(r1) : 0;
        int v2 = ok2 ? rawToMv(r2) : 0;
        float a1 = v1 / MV_PER_AMP;
        float a2 = v2 / MV_PER_AMP;

        evalStall(1, r1, ok1, m1_streak, m1_tripped);
        evalStall(2, r2, ok2, m2_streak, m2_tripped);

        Serial.printf("[I] M1 raw=%4d %4dmV %5.3fA %s%s   M2 raw=%4d %4dmV %5.3fA %s%s\n",
                      r1, v1, a1, ok1 ? "OK  " : "FAIL", m1_tripped ? " TRIP" : "",
                      r2, v2, a2, ok2 ? "OK  " : "FAIL", m2_tripped ? " TRIP" : "");
        vTaskDelay(pdMS_TO_TICKS(READ_PERIOD_MS));
    }
}

int bridgeCurrentReadM1Raw() {
    if (!adc2) return -1;
    int raw = 0;
    return readWithRetry(M1_ADC_CHAN, raw) ? raw : -1;
}

int bridgeCurrentReadM2Raw() {
    if (!adc2) return -1;
    int raw = 0;
    return readWithRetry(M2_ADC_CHAN, raw) ? raw : -1;
}

void bridgeCurrentInit() {
    if (adc2) return;       // déjà init

    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id  = ADC_UNIT_2;
    init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    if (adc_oneshot_new_unit(&init_cfg, &adc2) != ESP_OK) {
        Serial.println("[Current] adc_oneshot_new_unit(ADC2) failed");
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;   // 12 bits sur ESP32-S3
    chan_cfg.atten    = ADC_ATTEN;              // pleine échelle ~0..3.1 V

    if (adc_oneshot_config_channel(adc2, M1_ADC_CHAN, &chan_cfg) != ESP_OK ||
        adc_oneshot_config_channel(adc2, M2_ADC_CHAN, &chan_cfg) != ESP_OK) {
        Serial.println("[Current] adc_oneshot_config_channel failed");
        return;
    }

    // Calibration usine (curve fitting sur ESP32-S3) pour conversion raw -> mV
    // précise. Si la puce n'a pas la calib en eFuse, on tombera sur l'approx.
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id  = ADC_UNIT_2;
    cali_cfg.atten    = ADC_ATTEN;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali) != ESP_OK) {
        Serial.println("[Current] adc_cali curve_fitting indispo (utilisation approx)");
        cali = nullptr;
    }

    Serial.printf("[Current] ADC2 init OK : M1 GPIO%d (CH%d), M2 GPIO%d (CH%d)  cali=%s\n",
                  M1_GPIO, M1_ADC_CHAN, M2_GPIO, M2_ADC_CHAN,
                  cali ? "yes" : "no");
}

void bridgeCurrentStartMonitor() {
    if (!adc2) {
        Serial.println("[Current] StartMonitor: ADC pas init, skip");
        return;
    }
    xTaskCreatePinnedToCore(currentTask, "current_task", 4096, nullptr,
                            1 /* low priority */, nullptr, 1 /* core 1 */);
    Serial.println("[Current] monitor task started");
}
