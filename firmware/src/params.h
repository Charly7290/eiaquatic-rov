// AquaBot — parámetros ajustables desde el PC y guardados en flash (NVS).
// La tabla debe coincidir con PARAMS en pc/aquabot_pc/protocol.py (mismo orden).
#pragma once
#include <stdint.h>

enum ParamId : uint8_t {
  P_PITCH_KP = 0,
  P_PITCH_KI,
  P_PITCH_KD,
  P_PITCH_MAX_OUT,
  P_PITCH_SP_MAX,
  P_DEPTH_BAND,
  P_DEPTH_VMAX,
  P_DEPTH_KP,
  P_DEPTH_KV,
  P_DEPTH_KI,
  P_HOLD_RATE,
  P_BALLAST_STROKE,
  P_BALLAST_TIMEOUT,
  P_LOWBAT_WARN,
  P_LOWBAT_CRIT,
  P_SURFACE_LINKLOSS,
  P_SURFACE_LOWBAT,
  P_WATER_DENSITY,
  P_MOTOR_MIN_DUTY,
  P_RAMP_PER_S,
  P_VBAT_CAL,
  P_IMU_PITCH_OFS,
  P_IMU_ROLL_OFS,
  PARAM_COUNT
};

struct ParamDef {
  const char* name;
  float def, min, max;
};

extern const ParamDef PARAM_DEFS[PARAM_COUNT];

void paramsBegin();                 // carga de flash (o valores por defecto)
float P(ParamId id);                // valor actual
bool paramSet(uint8_t id, float v);  // aplica (con límites); false si id inválido
void paramsSave();                  // escribe todos en flash
