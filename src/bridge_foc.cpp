#include "bridge_foc.h"
#include "bridge_ble.h"
#include "bridge_led.h"
#include "bridge_current.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <Preferences.h>

// Pinout from src/main.cpp (custom DRV8313 board)
#define M1_IN1    14
#define M1_IN2    17
#define M1_IN3    18
#define M1_FAULT   1

#define M2_IN1    21
#define M2_IN2    38
#define M2_IN3    47
#define M2_FAULT   2

#define DRV_EN    48
#define PIN_MOSI  11
#define PIN_SCLK  12
#define PIN_MISO  13
#define ENC1_CS   10
#define ENC2_CS    8

const int POLE_PAIRS = 11;
const float SUPPLY_VOLTAGE = 12.0f;
const float VOLTAGE_LIMIT = 6.0f;

// Button behavior selectable at runtime: false = step on press, true = hold to move.
static bool button_hold_continuous = false;

bool bridgeFocGetButtonHoldContinuous() { return button_hold_continuous; }
void bridgeFocSetButtonHoldContinuous(bool hold) { button_hold_continuous = hold; }

BLDCMotor motor1(POLE_PAIRS);
BLDCMotor motor2(POLE_PAIRS);
BLDCDriver3PWM driver1(M1_IN1, M1_IN2, M1_IN3);
BLDCDriver3PWM driver2(M2_IN1, M2_IN2, M2_IN3);
MagneticSensorSPI sensor1(AS5048_SPI, ENC1_CS);
MagneticSensorSPI sensor2(AS5048_SPI, ENC2_CS);
SPIClass spiBus(HSPI);

float target1 = 0.0f;
float target2 = 0.0f;
bool verbose = false;
// Step pas-a-pas et vitesse continue : ajustables via /foc/step et /foc/speed.
// Vitesse continue (deg/s) = HOLD_STEP_DEG * 1000 / HOLD_STEP_MS (defaut 100 deg/s).
float TARGET_STEP_DEG = 7.0f;
float HOLD_STEP_DEG = 2.0f;
const uint32_t HOLD_STEP_MS = 20;
float bridgeFocGetStepDeg() { return TARGET_STEP_DEG; }
void  bridgeFocSetStepDeg(float v) {
  if (v < 0.5f) v = 0.5f;
  if (v > 45.0f) v = 45.0f;
  TARGET_STEP_DEG = v;
}
float bridgeFocGetHoldDegPerSec() { return HOLD_STEP_DEG * 1000.0f / (float)HOLD_STEP_MS; }
void  bridgeFocSetHoldDegPerSec(float dps) {
  if (dps < 5.0f)   dps = 5.0f;
  if (dps > 500.0f) dps = 500.0f;
  HOLD_STEP_DEG = dps * (float)HOLD_STEP_MS / 1000.0f;
}
bool run_m1 = true;
bool run_m2 = true;
bool motors_enabled = false;

// ── Soft-limits (calibration butées via sondes courant) ────────────────────
// Détection de butée : on rampe lentement le target, et un pic de courant
// sustained = on a tapé en mécanique. On note l'angle, on inverse, idem.
//
// Paramètres tunables :
#define CAL_RAMP_DEG_PER_TICK   0.5f      // 0.5° par tick = ~25°/s à 50 Hz
#define CAL_TICK_MS             20         // 50 Hz
#define CAL_END_STOP_RAW        500        // ~0.4 A : > stall normal mais << OCP driver
#define CAL_CONSEC_HITS         3          // 3 ticks = 60 ms de courant sustained
#define CAL_BACKOFF_MS          400        // pause avant de repartir dans l'autre sens
#define CAL_SAFETY_MARGIN_DEG   5.0f       // marge soft entre limite physique et logicielle
#define CAL_TIMEOUT_MS          30000U     // sécurité globale, abandon si on n'a pas fini
// Butées physiques mesurées (degrés, lectures AS5048 via SimpleFOC)
// Soft-limits = butée ± CAL_SAFETY_MARGIN_DEG, identique à la calibration auto.
#define CAL_DEFAULT_M1_LIM_NEG_DEG   251.0f
#define CAL_DEFAULT_M1_LIM_POS_DEG   320.0f
#define CAL_DEFAULT_M2_LIM_NEG_DEG  -345.0f
#define CAL_DEFAULT_M2_LIM_POS_DEG  -265.0f
#define CAL_HOLD_MS             3000U      // maintien des 4 boutons pour déclencher une calibration
#define CAL_ALL_MASK            0x55u      // bits P0|P2|P4|P6 du PCF8574 (M1+, M2+, M1−, M2−)
#define VEL_LIMIT_NORMAL        30.0f      // velocity_limit normal
#define VEL_LIMIT_RETURN        1.0f       // velocity_limit pendant un retour-en-zone (≈57°/s)

static bool  m1_cal_ok = false, m2_cal_ok = false;
static float m1_lim_neg = 0, m1_lim_pos = 0;     // butées physiques (rad)
static float m2_lim_neg = 0, m2_lim_pos = 0;
static float m1_soft_neg = 0, m1_soft_pos = 0;   // limites logicielles (rad)
static float m2_soft_neg = 0, m2_soft_pos = 0;
static bool  m1_returning = false, m2_returning = false;
static bool     cal_pending     = false;
static uint32_t cal_hold_since  = 0;
static bool     cal_hold_active = false;
static BridgeFocCalCallback s_cal_cb = nullptr;
void bridgeFocSetCalCallback(BridgeFocCalCallback cb) { s_cal_cb = cb; }

static float clampM1(float t) {
  if (!m1_cal_ok) return t;
  if (t > m1_soft_pos) return m1_soft_pos;
  if (t < m1_soft_neg) return m1_soft_neg;
  return t;
}
static float clampM2(float t) {
  if (!m2_cal_ok) return t;
  if (t > m2_soft_pos) return m2_soft_pos;
  if (t < m2_soft_neg) return m2_soft_neg;
  return t;
}

// ── Surveillance broche FAULT du DRV8313 ────────────────────────────────────
// nFAULT actif bas (overtemp / overcurrent / undervoltage). Le hardware filtre
// déjà ~1 ms via R19+C17. On exige FAULT_CONSECUTIVE lectures basses
// consécutives, échantillonnage à FAULT_CHECK_PERIOD_MS, puis on coupe le
// moteur concerné. Le latch est levé via bridgeFocEnableMotor() (m1on/m2on).
#define FAULT_CONSECUTIVE     3
#define FAULT_CHECK_PERIOD_MS 5

static int      m1_fault_streak   = 0;
static int      m2_fault_streak   = 0;
static bool     m1_fault_tripped  = false;
static bool     m2_fault_tripped  = false;
static uint32_t last_fault_check  = 0;

void stepTargetM1(float deg_step);
void stepTargetM2(float deg_step);

bool bridgeFocIsMotorEnabled(int idx) {
  if (!motors_enabled) return false;
  if (idx == 1) return run_m1;
  if (idx == 2) return run_m2;
  return false;
}

void bridgeFocDisableMotor(int idx) {
  if (idx == 1) {
    run_m1 = false;
    motor1.disable();
    Serial.println("[FOC] M1 disabled");
  } else if (idx == 2) {
    run_m2 = false;
    motor2.disable();
    Serial.println("[FOC] M2 disabled");
  }
}

void bridgeFocEnableMotor(int idx) {
  if (idx == 1) {
    if (m1_cal_ok) {
      // Recentre au milieu de la range a la reactivation. Si le moteur a
      // tripe contre une butee, viser la position courante le ferait
      // re-triper en boucle (stuck). On l'eloigne donc de la butee, en
      // retour doux pour ne pas claquer.
      target1 = (m1_soft_neg + m1_soft_pos) * 0.5f;
      motor1.velocity_limit = VEL_LIMIT_RETURN;
      m1_returning = true;
      Serial.printf("[FOC] M1 re-enable -> recentre a %.1f deg (milieu)\n",
                    target1 * RAD_TO_DEG);
    } else {
      target1 = motor1.shaftAngle();
      motor1.velocity_limit = VEL_LIMIT_NORMAL;
      m1_returning = false;
      Serial.printf("[FOC] M1 enabled (target=%.1f deg)\n", target1 * RAD_TO_DEG);
    }
    run_m1 = true;
    motor1.enable();
    bridgeCurrentClearStall(1);
    m1_fault_streak  = 0;
    m1_fault_tripped = false;
  } else if (idx == 2) {
    if (m2_cal_ok) {
      target2 = (m2_soft_neg + m2_soft_pos) * 0.5f;
      motor2.velocity_limit = VEL_LIMIT_RETURN;
      m2_returning = true;
      Serial.printf("[FOC] M2 re-enable -> recentre a %.1f deg (milieu)\n",
                    target2 * RAD_TO_DEG);
    } else {
      target2 = motor2.shaftAngle();
      motor2.velocity_limit = VEL_LIMIT_NORMAL;
      m2_returning = false;
      Serial.printf("[FOC] M2 enabled (target=%.1f deg)\n", target2 * RAD_TO_DEG);
    }
    run_m2 = true;
    motor2.enable();
    bridgeCurrentClearStall(2);
    m2_fault_streak  = 0;
    m2_fault_tripped = false;
  }
}

// Échantillonne les broches nFAULT, débounce, et coupe le moteur concerné en
// cas de fault confirmée. Appelé à chaque tour de bridgeFocLoop().
static void checkDriverFaults() {
  uint32_t now = millis();
  if (now - last_fault_check < FAULT_CHECK_PERIOD_MS) return;
  last_fault_check = now;

  // M1
  if (!m1_fault_tripped) {
    if (digitalRead(M1_FAULT) == LOW) {
      if (++m1_fault_streak >= FAULT_CONSECUTIVE) {
        m1_fault_tripped = true;
        Serial.println("[FAULT] M1 driver fault (overtemp / overcurrent / UVLO) — disabling");
        bridgeFocDisableMotor(1);
      }
    } else {
      m1_fault_streak = 0;
    }
  }

  // M2
  if (!m2_fault_tripped) {
    if (digitalRead(M2_FAULT) == LOW) {
      if (++m2_fault_streak >= FAULT_CONSECUTIVE) {
        m2_fault_tripped = true;
        Serial.println("[FAULT] M2 driver fault (overtemp / overcurrent / UVLO) — disabling");
        bridgeFocDisableMotor(2);
      }
    } else {
      m2_fault_streak = 0;
    }
  }
}

// External buttons via PCF8574T (active LOW, button to GND)
const int PIN_I2C_SDA = 6;
const int PIN_I2C_SCL = 7;
const uint8_t PCF8574_ADDR = 0x20;
const uint8_t BTN_M1_PLUS_BIT = 0;   // P0
const uint8_t BTN_M1_MINUS_BIT = 4;  // P4
const uint8_t BTN_M2_PLUS_BIT = 2;   // P2
const uint8_t BTN_M2_MINUS_BIT = 6;  // P6
const uint8_t BTN_SHUTTER_BIT = 7;   // P7
const uint32_t PCF_DEBOUNCE_MS = 25;

bool pcf_ready = false;
uint8_t pcf_stable_raw = 0xFF;
uint8_t pcf_candidate_raw = 0xFF;
uint32_t pcf_candidate_ms = 0;
uint8_t pcf_pressed = 0;
uint32_t pcf_hold_step_ms = 0;

bool pcfWriteRaw(uint8_t value) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool pcfPing() {
  Wire.beginTransmission(PCF8574_ADDR);
  return (Wire.endTransmission() == 0);
}

bool pcfReadRaw(uint8_t &value) {
  // Keep all PCF pins as input with weak pull-up.
  if (!pcfWriteRaw(0xFF)) return false;
  if (Wire.requestFrom((int)PCF8574_ADDR, 1) != 1) return false;
  value = Wire.read();
  return true;
}

void handleExternalButtons() {
  if (!pcf_ready) return;

  uint8_t raw = 0xFF;
  if (!pcfReadRaw(raw)) return;

  uint32_t now = millis();
  if (raw != pcf_candidate_raw) {
    pcf_candidate_raw = raw;
    pcf_candidate_ms = now;
    return;
  }

  if ((now - pcf_candidate_ms) < PCF_DEBOUNCE_MS) return;
  if (pcf_stable_raw == pcf_candidate_raw) return;

  uint8_t old_pressed = (uint8_t)(~pcf_stable_raw);
  uint8_t new_pressed = (uint8_t)(~pcf_candidate_raw);
  pcf_stable_raw = pcf_candidate_raw;
  pcf_pressed = new_pressed;

  bool shutter_pressed = ((old_pressed >> BTN_SHUTTER_BIT) & 0x01) == 0 &&
                         ((new_pressed >> BTN_SHUTTER_BIT) & 0x01);
  if (shutter_pressed) {
    bool ok = goproBLE.shutterOn();
    if (ok) {
      bridgeNotifyShutter();
    } else {
      Serial.println("[BTN] Shutter press ignored (BLE not connected)");
    }
  }

  // Combo : appui simultané +/− d'un même moteur => réactive ce moteur si
  // arrêté (équivalent de m1on/m2on au moniteur série). Détection sur la
  // transition "pas-tous-les-deux-pressés" → "les-deux-pressés", donc un
  // seul déclenchement par combo, indépendant du mode step/continu.
  bool m1_both_now    = ((new_pressed >> BTN_M1_PLUS_BIT)  & 0x01) &&
                        ((new_pressed >> BTN_M1_MINUS_BIT) & 0x01);
  bool m1_both_before = ((old_pressed >> BTN_M1_PLUS_BIT)  & 0x01) &&
                        ((old_pressed >> BTN_M1_MINUS_BIT) & 0x01);
  if (m1_both_now && !m1_both_before && !bridgeFocIsMotorEnabled(1)) {
    Serial.println("[BTN] Combo M1+/M1-  ->  re-enable M1");
    bridgeFocEnableMotor(1);
  }
  bool m2_both_now    = ((new_pressed >> BTN_M2_PLUS_BIT)  & 0x01) &&
                        ((new_pressed >> BTN_M2_MINUS_BIT) & 0x01);
  bool m2_both_before = ((old_pressed >> BTN_M2_PLUS_BIT)  & 0x01) &&
                        ((old_pressed >> BTN_M2_MINUS_BIT) & 0x01);
  if (m2_both_now && !m2_both_before && !bridgeFocIsMotorEnabled(2)) {
    Serial.println("[BTN] Combo M2+/M2-  ->  re-enable M2");
    bridgeFocEnableMotor(2);
  }

  if (!button_hold_continuous) {
    bool m1_plus_pressed = ((old_pressed >> BTN_M1_PLUS_BIT) & 0x01) == 0 && ((new_pressed >> BTN_M1_PLUS_BIT) & 0x01);
    bool m1_minus_pressed = ((old_pressed >> BTN_M1_MINUS_BIT) & 0x01) == 0 && ((new_pressed >> BTN_M1_MINUS_BIT) & 0x01);
    bool m2_plus_pressed = ((old_pressed >> BTN_M2_PLUS_BIT) & 0x01) == 0 && ((new_pressed >> BTN_M2_PLUS_BIT) & 0x01);
    bool m2_minus_pressed = ((old_pressed >> BTN_M2_MINUS_BIT) & 0x01) == 0 && ((new_pressed >> BTN_M2_MINUS_BIT) & 0x01);

    if (m1_plus_pressed) stepTargetM1(TARGET_STEP_DEG);
    if (m1_minus_pressed) stepTargetM1(-TARGET_STEP_DEG);
    if (m2_plus_pressed) stepTargetM2(TARGET_STEP_DEG);
    if (m2_minus_pressed) stepTargetM2(-TARGET_STEP_DEG);
  }
}

void applyHeldButtons() {
  if (!button_hold_continuous) return;
  if (!pcf_ready) return;
  uint32_t now = millis();
  if (now - pcf_hold_step_ms < HOLD_STEP_MS) return;
  pcf_hold_step_ms = now;

  bool m1_plus = ((pcf_pressed >> BTN_M1_PLUS_BIT) & 0x01);
  bool m1_minus = ((pcf_pressed >> BTN_M1_MINUS_BIT) & 0x01);
  bool m2_plus = ((pcf_pressed >> BTN_M2_PLUS_BIT) & 0x01);
  bool m2_minus = ((pcf_pressed >> BTN_M2_MINUS_BIT) & 0x01);

  if (m1_plus && !m1_minus) stepTargetM1(HOLD_STEP_DEG);
  if (m1_minus && !m1_plus) stepTargetM1(-HOLD_STEP_DEG);
  if (m2_plus && !m2_minus) stepTargetM2(HOLD_STEP_DEG);
  if (m2_minus && !m2_plus) stepTargetM2(-HOLD_STEP_DEG);
}

void printPwmInfo() {
  uint32_t f_m1a = ledcReadFreq(M1_IN1);
  uint32_t f_m1b = ledcReadFreq(M1_IN2);
  uint32_t f_m1c = ledcReadFreq(M1_IN3);
  uint32_t f_m2a = ledcReadFreq(M2_IN1);
  uint32_t f_m2b = ledcReadFreq(M2_IN2);
  uint32_t f_m2c = ledcReadFreq(M2_IN3);
  Serial.printf("LEDC freq Hz -> M1[%lu,%lu,%lu] M2[%lu,%lu,%lu]\n",
    (unsigned long)f_m1a, (unsigned long)f_m1b, (unsigned long)f_m1c,
    (unsigned long)f_m2a, (unsigned long)f_m2b, (unsigned long)f_m2c);
}

void setMotorsEnabled(bool en) {
  if (en && !motors_enabled) {
    motor1.enable();
    motor2.enable();
    motors_enabled = true;
    Serial.println("Motors ENABLED");
  } else if (!en && motors_enabled) {
    motor1.disable();
    motor2.disable();
    motors_enabled = false;
    Serial.println("Motors DISABLED");
  }
}

void stepTargetM1(float deg_step) {
  float old = target1;
  target1 = clampM1(target1 + deg_step * DEG_TO_RAD);
  // log uniquement si target a réellement bougé (pas de spam au butoir)
  if (target1 != old) {
    Serial.printf("M1 step %+0.1f deg -> M1=%.1f deg\n", deg_step, target1 * RAD_TO_DEG);
  }
}

void stepTargetM2(float deg_step) {
  float old = target2;
  target2 = clampM2(target2 + deg_step * DEG_TO_RAD);
  if (target2 != old) {
    Serial.printf("M2 step %+0.1f deg -> M2=%.1f deg\n", deg_step, target2 * RAD_TO_DEG);
  }
}

void printHelp() {
  Serial.println("\n=== SIMPLE CLOSED-LOOP (ANGLE) ===");
  Serial.println("h      : help");
  Serial.println("External buttons:");
  Serial.println("  M1: P0=+10 deg, P4=-10 deg");
  Serial.println("  M2: P2=+10 deg, P6=-10 deg");
  Serial.println("m1on/m1off, m2on/m2off : enable/disable individual motor (clears stall+fault latch)");
  Serial.println("on/off  : enable/disable both motors manually");
  Serial.println("f      : print real LEDC frequencies");
  Serial.println("z      : current angle -> target");
  Serial.println("v      : toggle verbose");
  Serial.println("1t<deg>: target M1, ex: 1t30");
  Serial.println("2t<deg>: target M2, ex: 2t-45");
}

void parseCommand(const char *cmd) {
  if (strcmp(cmd, "h") == 0) {
    printHelp();
    return;
  }
  if (strcmp(cmd, "z") == 0) {
    target1 = motor1.shaftAngle();
    target2 = motor2.shaftAngle();
    Serial.println("Targets set to current angles");
    return;
  }
  if (strcmp(cmd, "m1on") == 0)  { bridgeFocEnableMotor(1);  return; }
  if (strcmp(cmd, "m1off") == 0) { bridgeFocDisableMotor(1); return; }
  if (strcmp(cmd, "m2on") == 0)  { bridgeFocEnableMotor(2);  return; }
  if (strcmp(cmd, "m2off") == 0) { bridgeFocDisableMotor(2); return; }
  if (strcmp(cmd, "on") == 0) { setMotorsEnabled(true); return; }
  if (strcmp(cmd, "off") == 0) { setMotorsEnabled(false); return; }
  if (strcmp(cmd, "f") == 0) { printPwmInfo(); return; }
  if (strcmp(cmd, "v") == 0) {
    verbose = !verbose;
    Serial.printf("verbose=%d\n", (int)verbose);
    return;
  }

  if ((cmd[0] == '1' || cmd[0] == '2') && cmd[1] == 't') {
    float deg = atof(cmd + 2);
    float rad = deg * DEG_TO_RAD;
    if (cmd[0] == '1') target1 = clampM1(rad);
    else               target2 = clampM2(rad);
    float final_deg = (cmd[0] == '1' ? target1 : target2) * RAD_TO_DEG;
    if (fabsf(final_deg - deg) > 0.05f) {
      Serial.printf("M%c target=%.2f deg (clampe a %.2f par soft-limit)\n", cmd[0], deg, final_deg);
    } else {
      Serial.printf("M%c target=%.2f deg\n", cmd[0], deg);
    }
    return;
  }

  Serial.println("Unknown command. Type h");
}

void handleSerial() {
  static char buf[32];
  static int idx = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n' || idx >= (int)sizeof(buf) - 1) {
      buf[idx] = '\0';
      idx = 0;
      if (buf[0] != '\0') parseCommand(buf);
      continue;
    }
    buf[idx++] = ch;
  }
}

void setupMotor(BLDCMotor &motor, BLDCDriver3PWM &driver, MagneticSensorSPI &sensor) {
  sensor.init(&spiBus);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = VOLTAGE_LIMIT;
  driver.pwm_frequency = 20000;
  driver.init();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::angle;
  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.velocity_limit = 30.0f;

  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = 3.0f;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = 0.06f;
  motor.P_angle.P = 8.0f;

  motor.useMonitoring(Serial);
  motor.init();
  int foc_status = motor.initFOC();
  Serial.printf("initFOC status=%d\n", foc_status);
}

// État de calibration par moteur : on rampe +, on détecte le pic, on rampe -,
// on détecte le pic, fini. Les deux moteurs tournent en parallèle dans la
// même boucle : leurs phases peuvent être désynchronisées.
enum CalPhase { PH_RAMP_POS, PH_BACKOFF_POS, PH_RAMP_NEG, PH_DONE };
struct CalState {
  int        idx;
  CalPhase   phase;
  int        consec;
  float      lim_pos;
  float      lim_neg;
  uint32_t   phase_t0;
};

static void calStep(BLDCMotor& motor, CalState& s, float& target, int rawCurrent) {
  // Lecture ratée : on n'incrémente pas le compteur, on ne reset pas non plus.
  bool spike = (rawCurrent > 0) && (rawCurrent > CAL_END_STOP_RAW);

  switch (s.phase) {
    case PH_RAMP_POS: {
      if (spike) {
        if (++s.consec >= CAL_CONSEC_HITS) {
          s.lim_pos = motor.shaftAngle();
          Serial.printf("[CAL] M%d butee + a %.2f deg\n", s.idx, s.lim_pos * RAD_TO_DEG);
          target = s.lim_pos - 2.0f * DEG_TO_RAD;     // recule 2° pour libérer la mécanique
          s.phase = PH_BACKOFF_POS;
          s.phase_t0 = millis();
          s.consec = 0;
        }
      } else {
        s.consec = 0;
        target += CAL_RAMP_DEG_PER_TICK * DEG_TO_RAD;
      }
      break;
    }
    case PH_BACKOFF_POS: {
      // attend que le moteur recule physiquement et que le courant retombe
      if (millis() - s.phase_t0 > CAL_BACKOFF_MS) {
        s.phase = PH_RAMP_NEG;
        s.consec = 0;
      }
      break;
    }
    case PH_RAMP_NEG: {
      if (spike) {
        if (++s.consec >= CAL_CONSEC_HITS) {
          s.lim_neg = motor.shaftAngle();
          Serial.printf("[CAL] M%d butee - a %.2f deg\n", s.idx, s.lim_neg * RAD_TO_DEG);
          // recentre target au milieu pour repartir en zone safe
          target = (s.lim_pos + s.lim_neg) * 0.5f;
          s.phase = PH_DONE;
          s.consec = 0;
        }
      } else {
        s.consec = 0;
        target -= CAL_RAMP_DEG_PER_TICK * DEG_TO_RAD;
      }
      break;
    }
    case PH_DONE: break;
  }
}

static void saveCalToNVS() {
  Preferences p;
  p.begin("gimbal-cal", false);
  p.putFloat("m1_sn", m1_soft_neg);
  p.putFloat("m1_sp", m1_soft_pos);
  p.putFloat("m2_sn", m2_soft_neg);
  p.putFloat("m2_sp", m2_soft_pos);
  p.putBool("valid",  true);
  p.end();
  Serial.println("[CAL] Limites sauvegardees en NVS.");
}

static bool loadCalFromNVS() {
  Preferences p;
  p.begin("gimbal-cal", true);
  bool ok = p.getBool("valid", false);
  if (ok) {
    m1_soft_neg = p.getFloat("m1_sn", 0.0f);
    m1_soft_pos = p.getFloat("m1_sp", 0.0f);
    m2_soft_neg = p.getFloat("m2_sn", 0.0f);
    m2_soft_pos = p.getFloat("m2_sp", 0.0f);
    m1_cal_ok = true;
    m2_cal_ok = true;
  }
  p.end();
  return ok;
}

// Retourne true si la calibration a abouti, false en cas de timeout.
// En cas de timeout, les soft-limits de l'appelant doivent etre restaurees
// pour ne pas laisser les moteurs sans protection logicielle.
static bool runCalibration() {
  if (s_cal_cb) s_cal_cb(true);
  Serial.println("[CAL] Demarrage calibration butees (ADC2 avec retry Wi-Fi)");

  // Suspend la detection de stall : le courant va volontairement spiker
  // a chaque end-stop, on ne veut pas que la tache de fond coupe les moteurs.
  bridgeCurrentSetStallEnabled(false);

  CalState c1 = { 1, PH_RAMP_POS, 0, 0, 0, 0 };
  CalState c2 = { 2, PH_RAMP_POS, 0, 0, 0, 0 };

  // Démarre target = angle courant pour ne pas sauter au démarrage
  target1 = motor1.shaftAngle();
  target2 = motor2.shaftAngle();

  uint32_t t_start = millis();
  uint32_t t_last_tick = 0;
  bool timed_out = false;

  while (c1.phase != PH_DONE || c2.phase != PH_DONE) {
    if (millis() - t_start > CAL_TIMEOUT_MS) {
      Serial.println("[CAL] TIMEOUT : abandon. Limites precedentes restaurees.");
      timed_out = true;
      break;
    }

    // FOC à pleine vitesse (sinon le moteur ne suit pas le target)
    motor1.loopFOC(); motor1.move(target1);
    motor2.loopFOC(); motor2.move(target2);

    // Tick de calibration à 50 Hz : ramp + lecture courant + transitions
    if (millis() - t_last_tick >= CAL_TICK_MS) {
      t_last_tick = millis();
      int r1 = bridgeCurrentReadM1Raw();
      int r2 = bridgeCurrentReadM2Raw();
      calStep(motor1, c1, target1, r1);
      calStep(motor2, c2, target2, r2);
    }

    // Yield obligatoire : sinon la task watchdog (5 s) tire un reset, et
    // les autres taches (BLE, HTTPS, currentTask sur core 1 quand meme,
    // mais ici on est sur le main loop) sont privees du CPU.
    vTaskDelay(0);
  }

  bridgeCurrentSetStallEnabled(true);

  if (timed_out) {
    if (s_cal_cb) s_cal_cb(false);
    return false;
  }

  // Pose les soft-limits avec la marge de sécurité
  m1_lim_pos = c1.lim_pos;  m1_lim_neg = c1.lim_neg;
  m2_lim_pos = c2.lim_pos;  m2_lim_neg = c2.lim_neg;
  m1_soft_pos = m1_lim_pos - CAL_SAFETY_MARGIN_DEG * DEG_TO_RAD;
  m1_soft_neg = m1_lim_neg + CAL_SAFETY_MARGIN_DEG * DEG_TO_RAD;
  m2_soft_pos = m2_lim_pos - CAL_SAFETY_MARGIN_DEG * DEG_TO_RAD;
  m2_soft_neg = m2_lim_neg + CAL_SAFETY_MARGIN_DEG * DEG_TO_RAD;
  m1_cal_ok = true;
  m2_cal_ok = true;

  Serial.printf("[CAL] M1  butees [%.1f, %.1f]  soft [%.1f, %.1f] deg\n",
    m1_lim_neg * RAD_TO_DEG,  m1_lim_pos * RAD_TO_DEG,
    m1_soft_neg * RAD_TO_DEG, m1_soft_pos * RAD_TO_DEG);
  Serial.printf("[CAL] M2  butees [%.1f, %.1f]  soft [%.1f, %.1f] deg\n",
    m2_lim_neg * RAD_TO_DEG,  m2_lim_pos * RAD_TO_DEG,
    m2_soft_neg * RAD_TO_DEG, m2_soft_pos * RAD_TO_DEG);
  saveCalToNVS();
  if (s_cal_cb) s_cal_cb(false);
  return true;
}

void bridgeFocSetup() {
  Serial.println("\n=== GIMBAL SIMPLEFOC CLOSED LOOP ===");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  pcf_ready = pcfPing();
  if (pcf_ready) {
    uint8_t raw = 0xFF;
    if (pcfReadRaw(raw)) {
      pcf_stable_raw = raw;
      pcf_candidate_raw = raw;
      pcf_pressed = (uint8_t)(~raw);
    }
    Serial.printf("PCF8574 ready at 0x%02X (SDA=%d SCL=%d)\n", PCF8574_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);
  } else {
    Serial.printf("WARN: PCF8574 not found at 0x%02X (SDA=%d SCL=%d)\n", PCF8574_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);
  }

  pinMode(M1_FAULT, INPUT_PULLUP);
  pinMode(M2_FAULT, INPUT_PULLUP);

  pinMode(DRV_EN, OUTPUT);
  digitalWrite(DRV_EN, HIGH);

  // Force both encoder CS pins HIGH BEFORE any SPI traffic. Otherwise the
  // second sensor's CS floats during the first sensor's init/initFOC and may
  // briefly respond on MISO at the same time as sensor 1, corrupting M1's
  // angle readings and producing a wrong electrical-angle offset — motor 1
  // then runs open-loop into its mechanical end-stop.
  pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
  pinMode(ENC2_CS, OUTPUT); digitalWrite(ENC2_CS, HIGH);

  spiBus.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);

  setupMotor(motor1, driver1, sensor1);
  setupMotor(motor2, driver2, sensor2);
  printPwmInfo();
  setMotorsEnabled(true);
  target1 = motor1.shaftAngle();
  target2 = motor2.shaftAngle();

  // Tente de restaurer les limites d'une calibration précédente (NVS).
  // Sinon, utilise les valeurs hardcodées mesurées sur le gimbal.
  if (loadCalFromNVS()) {
    Serial.printf("[CAL] NVS: M1 [%.1f, %.1f]  M2 [%.1f, %.1f] deg\n",
                  m1_soft_neg * RAD_TO_DEG, m1_soft_pos * RAD_TO_DEG,
                  m2_soft_neg * RAD_TO_DEG, m2_soft_pos * RAD_TO_DEG);
  } else {
    m1_soft_neg = (CAL_DEFAULT_M1_LIM_NEG_DEG + CAL_SAFETY_MARGIN_DEG) * DEG_TO_RAD;
    m1_soft_pos = (CAL_DEFAULT_M1_LIM_POS_DEG - CAL_SAFETY_MARGIN_DEG) * DEG_TO_RAD;
    m2_soft_neg = (CAL_DEFAULT_M2_LIM_NEG_DEG + CAL_SAFETY_MARGIN_DEG) * DEG_TO_RAD;
    m2_soft_pos = (CAL_DEFAULT_M2_LIM_POS_DEG - CAL_SAFETY_MARGIN_DEG) * DEG_TO_RAD;
    m1_cal_ok = true;
    m2_cal_ok = true;
    Serial.printf("[CAL] Defauts: M1 [%.1f, %.1f]  M2 [%.1f, %.1f] deg\n",
                  m1_soft_neg * RAD_TO_DEG, m1_soft_pos * RAD_TO_DEG,
                  m2_soft_neg * RAD_TO_DEG, m2_soft_pos * RAD_TO_DEG);
  }
  Serial.println("[CAL] Maintenez 4 boutons moteur 3 s pour recalibrer.");

  printHelp();
}

static void checkCalTrigger() {
  if (!pcf_ready) return;
  bool all4 = (pcf_pressed & CAL_ALL_MASK) == CAL_ALL_MASK;
  if (all4) {
    if (!cal_hold_active) {
      cal_hold_active = true;
      cal_hold_since  = millis();
      Serial.println("[CAL] 4 boutons maintenus - calibration dans 3 s...");
    } else if (!cal_pending && (millis() - cal_hold_since >= CAL_HOLD_MS)) {
      cal_pending = true;
      Serial.println("[CAL] Calibration declenchee par l'utilisateur !");
    }
  } else {
    if (cal_hold_active && !cal_pending)
      Serial.println("[CAL] Maintien interrompu.");
    cal_hold_active = false;
  }
}

void bridgeFocLoop() {
  checkCalTrigger();
  if (cal_pending) {
    cal_pending     = false;
    cal_hold_active = false;

    // Sauvegarde l'etat actuel pour pouvoir le restaurer si la calibration
    // echoue (timeout). Sans ca, m*_cal_ok reste a false apres l'echec et
    // les moteurs perdent toute protection logicielle jusqu'au reboot.
    bool  bk_m1_ok = m1_cal_ok,    bk_m2_ok = m2_cal_ok;
    float bk_m1_sn = m1_soft_neg,  bk_m1_sp = m1_soft_pos;
    float bk_m2_sn = m2_soft_neg,  bk_m2_sp = m2_soft_pos;

    m1_cal_ok = false;
    m2_cal_ok = false;
    bool ok = runCalibration();

    if (!ok) {
      // Restaure les limites precedentes : les nouvelles n'ont pas pu etre
      // determinees. Si aucune calibration n'a jamais ete faite, on retombe
      // sur les defauts hardcodes (m*_cal_ok etait true des le boot via
      // bridgeFocSetup).
      m1_cal_ok = bk_m1_ok;    m2_cal_ok = bk_m2_ok;
      m1_soft_neg = bk_m1_sn;  m1_soft_pos = bk_m1_sp;
      m2_soft_neg = bk_m2_sn;  m2_soft_pos = bk_m2_sp;
    }

    // Recentre EXPLICITEMENT les deux moteurs au milieu de leur range.
    // Sans ca, le moteur qui finit sa calibration en dernier n'a pas le
    // temps physique de bouger avant l'exit de runCalibration() et reste
    // colle a sa butee. Velocity_limit reduite pour un retour doux.
    if (m1_cal_ok) {
      target1 = (m1_soft_neg + m1_soft_pos) * 0.5f;
      motor1.velocity_limit = VEL_LIMIT_RETURN;
      m1_returning = true;
    } else {
      target1 = motor1.shaftAngle();
    }
    if (m2_cal_ok) {
      target2 = (m2_soft_neg + m2_soft_pos) * 0.5f;
      motor2.velocity_limit = VEL_LIMIT_RETURN;
      m2_returning = true;
    } else {
      target2 = motor2.shaftAngle();
    }
    return;
  }

  handleSerial();
  handleExternalButtons();
  applyHeldButtons();
  checkDriverFaults();

  // Restaure la velocity_limit normale une fois la cible de retour atteinte
  // (~2°). On teste la proximite a la cible, pas juste "en zone" : sinon un
  // moteur tripe a la butee (deja en zone) repasserait en vitesse normale
  // immediatement, sans benefice du retour doux vers le milieu.
  if (m1_returning) {
    if (fabsf(motor1.shaftAngle() - target1) < 2.0f * DEG_TO_RAD) {
      motor1.velocity_limit = VEL_LIMIT_NORMAL;
      m1_returning = false;
      Serial.println("[FOC] M1 retour termine, velocity_limit normal");
    }
  }
  if (m2_returning) {
    if (fabsf(motor2.shaftAngle() - target2) < 2.0f * DEG_TO_RAD) {
      motor2.velocity_limit = VEL_LIMIT_NORMAL;
      m2_returning = false;
      Serial.println("[FOC] M2 retour termine, velocity_limit normal");
    }
  }

  if (!motors_enabled) return;

  if (run_m1) {
    motor1.loopFOC();
    motor1.move(target1);
  } else {
    motor1.move(motor1.shaftAngle());
  }

  if (run_m2) {
    motor2.loopFOC();
    motor2.move(target2);
  } else {
    motor2.move(motor2.shaftAngle());
  }

  static uint32_t tlog = 0;
  if (verbose && millis() - tlog > 200) {
    tlog = millis();
    Serial.printf("M1 %.2f -> %.2f deg | M2 %.2f -> %.2f deg\n",
      motor1.shaftAngle() * RAD_TO_DEG, target1 * RAD_TO_DEG,
      motor2.shaftAngle() * RAD_TO_DEG, target2 * RAD_TO_DEG);
  }
}
