#include "control.h"

#include <math.h>

// Servo del pistón del hold de profundidad (elegidos con el barrido de test_host/robust.cpp).
#ifndef DH_START
#define DH_START 0.02f      // error de posición que arranca el motor (fracción de carrera)
#endif
#ifndef DH_MIN_GAP_S
#define DH_MIN_GAP_S 0.3f   // tiempo mínimo entre arranques
#endif
#ifndef DH_DV_TAU_S
#define DH_DV_TAU_S 1.0f    // filtro del error de velocidad
#endif

namespace ctl {

void mixHorizontal(float surge, float yaw, float power, float out[4]) {
  float left = surge + yaw;
  float right = surge - yaw;
  float m = fmaxf(fabsf(left), fabsf(right));
  if (m > 1.0f) {
    left /= m;
    right /= m;
  }
  power = clampf(power, 0.0f, 1.0f);
  out[0] = out[1] = left * power;
  out[2] = out[3] = right * power;
}

void Attitude::update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
  (void)gz;
  const float RAD2DEG = 57.2957795f;
  float accPitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
  float accRoll = atan2f(ay, az) * RAD2DEG;
  pitchRate = -gy;           // rotación +Y (Y a la izquierda) baja la nariz
  float rollRate = gx;       // rotación +X baja el lado derecho
  if (!initialized) {
    pitch = accPitch;
    roll = accRoll;
    initialized = true;
    return;
  }
  // Si la aceleración se aleja mucho de 1 g (golpes, empuje), se confía solo en el giroscopio.
  float aNorm = sqrtf(ax * ax + ay * ay + az * az);
  float a = (aNorm > 0.8f && aNorm < 1.2f) ? alpha : 1.0f;
  pitch = a * (pitch + pitchRate * dt) + (1.0f - a) * accPitch;
  roll = a * (roll + rollRate * dt) + (1.0f - a) * accRoll;
}

float PitchPID::update(float sp, float pitch, float rate, float dt, const PitchGains& g) {
  float e = sp - pitch;
  float p = g.kp * e;
  float d = -g.kd * rate;
  float u = p + g.ki * integ_ + d;
  // Anti-windup: integra solo si no está saturado empujando en el mismo sentido del error.
  bool saturated = fabsf(u) >= g.maxOut;
  if (!saturated || ((u > 0) != (e > 0))) integ_ += e * dt;
  if (g.ki > 1e-6f) {
    float lim = g.maxOut / g.ki;
    integ_ = clampf(integ_, -lim, lim);
  } else {
    integ_ = 0;
  }
  u = p + g.ki * integ_ + d;
  return clampf(u, -g.maxOut, g.maxOut);
}

int8_t Ballast::update(int8_t request, bool limFull, bool limEmpty, float dt, float strokeS, float timeoutS) {
  // --- Interpretación de los switches, a prueba de cableado:
  //  · uno activo: normal.
  //  · los dos activos (p. ej. pull-ups sin 3V3: al presionar uno caen los dos): es el tope hacia el que
  //    se movía el pistón al activarse; si estaba quieto, el más cercano según la posición estimada.
  //  · si siguen los dos activos aunque el pistón lleva >2 s alejándose de ese tope, las lecturas no
  //    son confiables (switch pegado o cable a GND): se ignoran y protege el timeout.
  bool both = limFull && limEmpty;
  if (both && !prevBoth_) {
    // Quieto: si la posición es conocida, el tope más cercano; si no (recién encendido), VACÍO,
    // que es donde arranca normalmente el ROV (y adonde lo lleva la emersión por pérdida de enlace).
    ambigSide_ = lastMove_ != 0 ? lastMove_ : ((posKnown && pos >= 0.5f) ? 1 : -1);
    awayTime_ = 0;
    switchFault = false;
  }
  if (!both) {
    awayTime_ = 0;
    switchFault = false;
  }
  prevBoth_ = both;

  bool full = limFull, empty = limEmpty;
  if (both) {
    if (switchFault) {
      full = empty = false;
    } else {
      full = ambigSide_ > 0;
      empty = !full;
    }
  }

  if (full) {
    pos = 1.0f;
    posKnown = true;
  } else if (empty) {
    pos = 0.0f;
    posKnown = true;
  }

  if (request == 0) timeoutFault = false;

  int8_t out = request;
  if (timeoutFault) out = 0;
  if (out > 0 && full) out = 0;
  if (out < 0 && empty) out = 0;

  if (out != 0) {
    if (out != dir) runTime_ = 0;
    runTime_ += dt;
    if (runTime_ > timeoutS) {
      timeoutFault = true;
      out = 0;
    }
  }
  if (out == 0) runTime_ = 0;

  // ¿se aleja del tope interpretado y los dos siguen activos? -> lecturas no confiables
  if (both && !switchFault && out != 0 && out == -ambigSide_) {
    awayTime_ += dt;
    if (awayTime_ > 2.0f) switchFault = true;
  }

  if (out != 0 && strokeS > 0.1f) pos = clampf(pos + out * dt / strokeS, 0.0f, 1.0f);
  if (out != 0) lastMove_ = out;
  dir = out;
  atFull = full;
  atEmpty = empty;
  return out;
}

void DepthHold::reset(float sp, float ballastPos) {
  setpoint = sp < 0 ? 0 : sp;
  neutral = posRef = clampf(ballastPos, 0, 1);  // se asume que el piloto dejó el ROV trimado
  vref = 0;
  dvFilt_ = 0;
  moving_ = 0;
  sinceStop_ = 10;
}

int8_t DepthHold::update(float depth, float vz, float ballastPos, float stickVert, float dt,
                         const DepthHoldParams& p) {
  setpoint -= stickVert * p.holdRate * dt;  // stick arriba -> setpoint menos profundo
  if (setpoint < 0) setpoint = 0;

  // 1) profundidad -> velocidad deseada
  float e = depth - setpoint;  // + = demasiado profundo
  float eb = 0;
  if (e > p.band) eb = e - p.band;
  else if (e < -p.band) eb = e + p.band;
  vref = clampf(-p.kp * eb, -p.vmax, p.vmax);

  // 2) PI de velocidad -> posición deseada del lastre (bajando de más -> menos lastre)
  float dv = vz - vref;
  dvFilt_ += clampf(dt / DH_DV_TAU_S, 0, 1) * (dv - dvFilt_);  // el ruido del sensor no debe mover el pistón
  neutral = clampf(neutral - p.ki * dv * dt, 0, 1);
  posRef = clampf(neutral - p.kv * dvFilt_, 0, 1);

  // 3) servo de posición del pistón con histéresis y tiempo mínimo entre arranques
  const float START = DH_START, MIN_GAP_S = DH_MIN_GAP_S;
  float err = posRef - ballastPos;
  sinceStop_ += dt;
  if (moving_ == 0) {
    if (sinceStop_ >= MIN_GAP_S) {
      if (err > START) moving_ = +1;
      else if (err < -START) moving_ = -1;
    }
  } else if (moving_ * err <= 0) {
    moving_ = 0;
    sinceStop_ = 0;
  }
  return moving_;
}

}  // namespace ctl
