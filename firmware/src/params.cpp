#include "params.h"

#include <Preferences.h>

#include "control.h"

// nombre, defecto, mínimo, máximo
const ParamDef PARAM_DEFS[PARAM_COUNT] = {
    {"PITCH_KP", 0.03f, 0.0f, 1.0f},           // 1/°
    {"PITCH_KI", 0.01f, 0.0f, 1.0f},           // 1/(°·s)
    {"PITCH_KD", 0.005f, 0.0f, 0.5f},          // 1/(°/s)
    {"PITCH_MAX_OUT", 0.6f, 0.0f, 1.0f},       // fracción
    {"PITCH_SP_MAX", 30.0f, 0.0f, 60.0f},      // °
    {"DEPTH_BAND", 0.05f, 0.0f, 1.0f},         // m
    {"DEPTH_VMAX", 0.10f, 0.01f, 1.0f},        // m/s
    {"DEPTH_KP", 0.15f, 0.0f, 2.0f},           // 1/s       profundidad -> velocidad
    {"DEPTH_KV", 0.5f, 0.0f, 5.0f},            // carrera/(m/s)   velocidad -> lastre
    {"DEPTH_KI", 0.15f, 0.0f, 2.0f},           // carrera/(m/s·s) aprende el neutro
    {"HOLD_RATE", 0.10f, 0.0f, 1.0f},          // m/s
    {"BALLAST_STROKE", 20.0f, 1.0f, 120.0f},   // s
    {"BALLAST_TIMEOUT", 40.0f, 2.0f, 240.0f},  // s
    {"LOWBAT_WARN", 10.5f, 6.0f, 13.0f},       // V
    {"LOWBAT_CRIT", 9.9f, 6.0f, 13.0f},        // V
    {"SURFACE_LINKLOSS", 15.0f, 1.0f, 300.0f}, // s
    {"SURFACE_LOWBAT", 10.0f, 1.0f, 300.0f},   // s
    {"WATER_DENSITY", 997.0f, 990.0f, 1030.0f},// kg/m³
    {"MOTOR_MIN_DUTY", 0.0f, 0.0f, 0.5f},      // fracción
    {"RAMP_PER_S", 3.0f, 0.2f, 50.0f},         // escala completa por segundo
    {"VBAT_CAL", 1.0f, 0.8f, 1.2f},            // factor
    {"IMU_PITCH_OFS", 0.0f, -45.0f, 45.0f},    // °
    {"IMU_ROLL_OFS", 0.0f, -45.0f, 45.0f},     // °
};

static float values[PARAM_COUNT];
static Preferences prefs;
static const char* NS = "aquabot";
static const uint32_t PARAMS_VERSION = 2;

void paramsBegin() {
  for (int i = 0; i < PARAM_COUNT; i++) values[i] = PARAM_DEFS[i].def;
  if (!prefs.begin(NS, true)) return;  // aún no existe: valores por defecto
  uint32_t ver = prefs.getUInt("ver", 0);
  uint32_t n = prefs.getUInt("n", 0);
  if (ver == PARAMS_VERSION && n == PARAM_COUNT && prefs.getBytesLength("v") == sizeof(values)) {
    float stored[PARAM_COUNT];
    prefs.getBytes("v", stored, sizeof(stored));
    for (int i = 0; i < PARAM_COUNT; i++) paramSet(i, stored[i]);
  }
  prefs.end();
}

float P(ParamId id) { return values[id]; }

bool paramSet(uint8_t id, float v) {
  if (id >= PARAM_COUNT || v != v) return false;  // id inválido o NaN
  values[id] = ctl::clampf(v, PARAM_DEFS[id].min, PARAM_DEFS[id].max);
  return true;
}

void paramsSave() {
  if (!prefs.begin(NS, false)) return;
  prefs.putUInt("ver", PARAMS_VERSION);
  prefs.putUInt("n", PARAM_COUNT);
  prefs.putBytes("v", values, sizeof(values));
  prefs.end();
}
