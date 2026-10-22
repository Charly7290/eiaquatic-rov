// AquaBot — lógica de control. Sin dependencias de Arduino (se prueba en el PC).
#pragma once
#include <stdint.h>

namespace ctl {

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// --------------------------------------------------------------- Mezclador horizontal
// 4 propulsores alineados, 2 por lado. out = {M1 izq-inf, M2 izq-sup, M3 der-sup, M4 der-inf}.
// surge, yaw en -1..1 (yaw + = girar a la derecha); power en 0..1.
void mixHorizontal(float surge, float yaw, float power, float out[4]);

// --------------------------------------------------------------- Actitud (filtro complementario)
// Entradas en ejes del CUERPO (X adelante, Y izquierda, Z arriba): acel en g, giro en °/s.
// pitch + = nariz arriba; roll + = lado derecho abajo.
class Attitude {
 public:
  void update(float ax, float ay, float az, float gx, float gy, float gz, float dt);
  float pitch = 0, roll = 0;   // °, sin offset de nivelación
  float pitchRate = 0;         // °/s, + = nariz subiendo
  bool initialized = false;
  float alpha = 0.98f;
};

// --------------------------------------------------------------- PID de cabeceo
struct PitchGains {
  float kp, ki, kd, maxOut;
};

class PitchPID {
 public:
  void reset() { integ_ = 0; }
  // sp, pitch en °; rate en °/s (derivativo directo del giroscopio). Devuelve -maxOut..maxOut.
  float update(float sp, float pitch, float rate, float dt, const PitchGains& g);
  float integral() const { return integ_; }

 private:
  float integ_ = 0;
};

// --------------------------------------------------------------- Lastre
// Convención: +1 = LLENAR (hunde), -1 = VACIAR (sube), 0 = quieto.
class Ballast {
 public:
  // request: dirección pedida por la lógica de modos. Devuelve la dirección aplicada
  // después de fines de carrera, falla y timeout. Actualiza la posición estimada.
  int8_t update(int8_t request, bool limFull, bool limEmpty, float dt, float strokeS, float timeoutS);
  float pos = 0.5f;          // 0 = vacío, 1 = lleno (estimada)
  bool posKnown = false;
  bool timeoutFault = false;  // marcha continua demasiado larga; se limpia al soltar el pedido
  bool switchFault = false;   // los dos switches siguen activos aunque el pistón se alejó: lecturas
                              // no confiables, se ignoran (solo aviso; protege el timeout)
  bool atFull = false, atEmpty = false;  // topes ya interpretados (lo que se reporta al PC)
  int8_t dir = 0;

 private:
  float runTime_ = 0;
  // Los dos switches activos a la vez (22 oct: pull-ups sin 3V3 -> al presionar uno bajan los dos).
  // Se decide cuál es por el sentido en que se movía el pistón al activarse.
  bool prevBoth_ = false;
  int8_t ambigSide_ = 0;      // +1 = se toma como LLENO, -1 = VACÍO
  int8_t lastMove_ = 0;       // último sentido de marcha distinto de 0
  float awayTime_ = 0;        // tiempo alejándose del tope mientras siguen los dos activos
};

// --------------------------------------------------------------- Hold de profundidad
// Cascada de tres niveles:
//   1) profundidad -> velocidad vertical deseada  vref = -KP·(e fuera de la banda), limitada a ±vmax
//   2) PI de velocidad -> posición deseada del lastre  posRef = neutro - KV·(vz - vref)
//      el integrador "neutro" aprende solo la posición de flotabilidad neutra
//   3) servo de posición del pistón (con histéresis) -> pedido al motor del lastre
// El lazo de velocidad necesita saber cuánta flotabilidad ya hay aplicada; por eso usa la posición
// estimada del pistón (Ballast::pos). Una versión de "pulso y espera" sin esa información entraba en
// ciclos límite de hasta ±0.7 m en simulación.
struct DepthHoldParams {
  float band;      // m, zona sin corrección alrededor del setpoint
  float vmax;      // m/s, velocidad vertical máxima pedida
  float kp;        // 1/s, profundidad -> velocidad
  float kv;        // fracción de carrera por m/s
  float ki;        // fracción de carrera por (m/s · s)
  float holdRate;  // m/s del setpoint con el stick a fondo
};

class DepthHold {
 public:
  void reset(float setpoint, float ballastPos);
  // depth (m, + abajo), vz (m/s, + bajando), ballastPos (0 vacío..1 lleno), stickVert (-1..1, + subir).
  // Devuelve el pedido para el lastre (+1 llenar, -1 vaciar, 0).
  int8_t update(float depth, float vz, float ballastPos, float stickVert, float dt, const DepthHoldParams& p);
  float setpoint = 0;
  float vref = 0;     // velocidad deseada (m/s, + bajar)
  float neutral = 0.5f;  // posición neutra aprendida
  float posRef = 0.5f;   // posición deseada del lastre

 private:
  int8_t moving_ = 0;
  float dvFilt_ = 0, sinceStop_ = 10;
};

// --------------------------------------------------------------- Rampa
inline float ramp(float current, float target, float maxStep) {
  float d = clampf(target - current, -maxStep, maxStep);
  return current + d;
}

}  // namespace ctl
