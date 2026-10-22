// AquaBot USB 2026 — firmware del ROV (Categoría B, teledirigido por tether).
//
// Bucle cooperativo sin bloqueos:
//   enlace (cada vuelta) -> responde TELEM a cada CMD
//   MS5837 (máquina de estados, ~25 Hz) -> profundidad y velocidad vertical
//   MPU6050 (100 Hz) -> cabeceo/alabeo
//   DS18B20 (1 Hz, sin bloquear)
//   control (50 Hz): failsafe, batería, mezclador, PID de cabeceo, lastre
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "control.h"
#include "link.h"
#include "motors.h"
#include "mpu6050.h"
#include "ms5837.h"
#include "params.h"
#include "protocol.h"

#if DEBUG_PRINTS
#define DBG(...) Serial.printf(__VA_ARGS__)
#else
#define DBG(...) \
  do {           \
  } while (0)
#endif

static const float G = 9.80665f;

// ------------------------------------------------------------------ Módulos
static Link rovLink;
static MS5837 baro;
static MPU6050 imu;
static OneWire oneWire(PIN_DS18B20);
static DallasTemperature ds18(&oneWire);
static ctl::Attitude att;
static ctl::PitchPID pitchPid;
static ctl::DepthHold depthHold;
static ctl::Ballast ballast;

// ------------------------------------------------------------------ Estado
static proto::Cmd cmd = {};
static uint8_t prevCmdFlags = 0;
static uint32_t lastCmdMs = 0;
static bool linkOk = false;
static bool armed = false;
// Tras un failsafe o un reinicio de la ESP32 hay que desarmar y volver a armar desde el PC:
// arranca bloqueado para que un reinicio bajo el agua (watchdog, caída de voltaje) no rearme solo.
static bool armBlocked = true;
static float linkLostS = 0;

static bool pitchHoldOn = false, depthHoldOn = false, prevDepthHold = false;
static float pitchSp = 0;

// Profundidad
static bool zeroing = true;
static double zeroSum = 0;
static int zeroCount = 0;
static float p0Pa = 101325.0f;
static float depthM = 0, vzMs = 0;
static bool depthValid = false;
static uint32_t lastBaroMs = 0;

// Temperatura
static float waterTempC = NAN;
static bool tempOk = false;
static uint32_t tempReqMs = 0;
static uint32_t tempGoodMs = 0;          // última lectura válida
static uint16_t tempBadCount = 0;        // lecturas inválidas (diagnóstico)
#define TEMP_HOLD_MS 5000                // se conserva la última lectura válida hasta 5 s

// Batería
static float vbatFast = 0, vbatSlow = 0;
static bool batPresent = false, lowWarn = false, lowCrit = false;
static bool surfacing = false;

// Motores
static float motorOut[MOTOR_COUNT] = {0};
static bool limFull = false, limEmpty = false;

static uint32_t tImu = 0, tCtrl = 0, tDbg = 0;

// ------------------------------------------------------------------ Utilidades
static inline int16_t toI16(float v) {
  if (v != v) return 0;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static float readVbat() {
  return analogReadMilliVolts(PIN_VBAT) * VBAT_DIVIDER / 1000.0f * P(P_VBAT_CAL);
}

static void startDepthZero() {
  zeroing = true;
  zeroSum = 0;
  zeroCount = 0;
  depthValid = false;
}

// ------------------------------------------------------------------ Telemetría
static void sendTelemetry() {
  proto::Telem t = {};
  t.seq = cmd.seq;
  uint16_t f = 0;
  if (armed) f |= proto::TF_ARMED;
  if (pitchHoldOn) f |= proto::TF_PITCH_HOLD;
  if (depthHoldOn) f |= proto::TF_DEPTH_HOLD;
  if (!linkOk) f |= proto::TF_LINK_FAILSAFE;
  if (surfacing) f |= proto::TF_SURFACING;
  if (lowWarn) f |= proto::TF_LOWBAT_WARN;
  if (lowCrit) f |= proto::TF_LOWBAT_CRIT;
  if (limFull) f |= proto::TF_LIM_FULL;
  if (limEmpty) f |= proto::TF_LIM_EMPTY;
  if (depthValid) f |= proto::TF_DEPTH_OK;
  if (imu.ok()) f |= proto::TF_IMU_OK;
  if (tempOk) f |= proto::TF_TEMP_OK;
  if (ballast.timeoutFault || ballast.switchFault) f |= proto::TF_BALLAST_FAULT;
  if (ballast.posKnown) f |= proto::TF_BALLAST_POS_KNOWN;
  t.flags = f;
  t.depth_mm = toI16(depthM * 1000.0f);
  t.depth_sp_mm = toI16(depthHold.setpoint * 1000.0f);
  t.vz_mms = toI16(vzMs * 1000.0f);
  t.pressure_mbar = baro.ok() ? (uint16_t)lroundf(baro.pressureMbar()) : 0;
  t.water_temp_cC = tempOk ? toI16(waterTempC * 100.0f) : 0;
  t.ms_temp_cC = toI16(baro.temperatureC() * 100.0f);
  t.pitch_cdeg = toI16((att.pitch - P(P_IMU_PITCH_OFS)) * 100.0f);
  t.roll_cdeg = toI16((att.roll - P(P_IMU_ROLL_OFS)) * 100.0f);
  t.pitch_sp_cdeg = toI16(pitchSp * 100.0f);
  t.vbat_mV = (uint16_t)lroundf(vbatFast * 1000.0f);
  t.ballast_pos = ballast.posKnown ? (uint8_t)lroundf(ballast.pos * 100.0f) : 255;
  t.ballast_dir = ballast.dir;
  for (int i = 0; i < 5; i++) t.motor[i] = (int8_t)lroundf(motorOut[i] * 100.0f);
  t.uptime_ms = millis();
  t.rx_err = (uint16_t)(rovLink.errors() > 65535 ? 65535 : rovLink.errors());
  rovLink.send(proto::TELEM, &t, sizeof(t));
}

static void sendParam(uint8_t id) {
  proto::ParamValue pv = {};
  pv.count = PARAM_COUNT;
  if (id < PARAM_COUNT) {
    pv.id = id;
    pv.value = P((ParamId)id);
  } else {
    pv.id = 0xFF;
  }
  rovLink.send(proto::PARAM_VALUE, &pv, sizeof(pv));
}

// ------------------------------------------------------------------ Mensajes entrantes
static void handleFrame(uint32_t now) {
  const proto::Parser& fr = rovLink.frame();
  switch (fr.type()) {
    case proto::CMD: {
      if (fr.len() != sizeof(proto::Cmd)) return;
      memcpy(&cmd, fr.payload(), sizeof(cmd));
      lastCmdMs = now;
      linkOk = true;
      uint8_t rising = cmd.flags & ~prevCmdFlags;
      prevCmdFlags = cmd.flags;

      bool armReq = cmd.flags & proto::CF_ARM;
      if (!armReq) armBlocked = false;
      armed = armReq && !armBlocked;

      if (rising & proto::CF_ZERO_DEPTH) startDepthZero();
      if ((rising & proto::CF_LEVEL_IMU) && !armed && imu.ok()) {
        paramSet(P_IMU_PITCH_OFS, att.pitch);
        paramSet(P_IMU_ROLL_OFS, att.roll);
        paramsSave();
      }
      sendTelemetry();
      break;
    }
    case proto::PARAM_SET: {
      if (fr.len() != sizeof(proto::ParamSet)) return;
      proto::ParamSet ps;
      memcpy(&ps, fr.payload(), sizeof(ps));
      bool good = paramSet(ps.id, ps.value);
      if (good && ps.save) paramsSave();
      sendParam(good ? ps.id : 0xFF);
      break;
    }
    case proto::PARAM_GET: {
      if (fr.len() != sizeof(proto::ParamGet)) return;
      sendParam(fr.payload()[0]);
      break;
    }
    default:
      break;
  }
}

// ------------------------------------------------------------------ Sensores
static void processPressure(uint32_t now) {
  float pa = baro.pressurePa();
  if (zeroing) {
    zeroSum += pa;
    if (++zeroCount >= DEPTH_ZERO_SAMPLES) {
      p0Pa = (float)(zeroSum / zeroCount);
      zeroing = false;
      depthM = 0;
      vzMs = 0;
      depthValid = true;
    }
    lastBaroMs = now;
    return;
  }
  depthValid = true;
  float raw = (pa - p0Pa) / (P(P_WATER_DENSITY) * G);
  float dt = (now - lastBaroMs) / 1000.0f;
  lastBaroMs = now;
  float prev = depthM;
  depthM += 0.3f * (raw - depthM);  // EMA
  if (dt > 0.001f && dt < 0.5f) vzMs += 0.2f * ((depthM - prev) / dt - vzMs);
}

// Lectura rápida del único DS18B20 del bus (Skip ROM, sin búsqueda de dirección): ~7 ms bloqueado en vez
// de ~25 ms con getTempCByIndex(). Un bloqueo largo retrasa la respuesta al PC y aparecía como "pérdida"
// en el indicador de enlace (21 oct). Se valida con el CRC y con los bits fijos del registro de config.
static float readDs18Fast() {
  uint8_t sp[9];
  if (!oneWire.reset()) return DEVICE_DISCONNECTED_C;  // nadie respondió
  oneWire.skip();
  oneWire.write(0xBE);  // READ SCRATCHPAD
  for (uint8_t i = 0; i < 9; i++) sp[i] = oneWire.read();
  if (OneWire::crc8(sp, 8) != sp[8]) return DEVICE_DISCONNECTED_C;
  if ((sp[4] & 0x1F) != 0x1F) return DEVICE_DISCONNECTED_C;  // descarta un bus en 0 (CRC de ceros = 0)
  int16_t raw = (int16_t)((sp[1] << 8) | sp[0]);
  return raw / 16.0f;  // 12 bits: 0.0625 °C por cuenta
}

static void readTemperature(uint32_t now) {
  if (now - tempReqMs < TEMP_PERIOD_MS) return;
  float t = readDs18Fast();
  // 85.00 es el valor de encendido del DS18B20 (conversión no completada): no es una lectura válida.
  // En el agua nunca se llega a 80 °C, así que el rango ya lo excluye; se deja explícito por claridad.
  bool valid = (t != DEVICE_DISCONNECTED_C) && t != 85.0f && t > -20.0f && t < 80.0f;
  if (valid) {
    waterTempC = t;
    tempGoodMs = now;
  } else {
    tempBadCount++;
  }
  // Una lectura inválida suelta no tumba el sensor: se mantiene la última buena hasta TEMP_HOLD_MS.
  tempOk = tempGoodMs != 0 && (now - tempGoodMs) <= TEMP_HOLD_MS;
  ds18.requestTemperatures();  // no bloquea (setWaitForConversion(false))
  tempReqMs = now;
}

static void readImu(uint32_t now) {
  if (now - tImu < IMU_PERIOD_MS) return;
  float dt = (now - tImu) / 1000.0f;
  tImu = now;
  static uint32_t lastRetry = 0;
  if (!imu.ok()) {  // reintenta la inicialización cada segundo
    if (now - lastRetry >= 1000) {
      lastRetry = now;
      imu.begin(Wire, MPU6050_ADDR);
    }
    return;
  }
  if (!imu.read()) return;
  float ax, ay, az, gx, gy, gz;
  imu.toBody(ax, ay, az, gx, gy, gz);
  att.update(ax, ay, az, gx, gy, gz, dt > 0.1f ? 0.01f : dt);
}

// ------------------------------------------------------------------ Control (50 Hz)
static void updateBattery(float dt) {
  float v = readVbat();
  if (vbatFast == 0) vbatFast = vbatSlow = v;
  vbatFast += ctl::clampf(dt / 0.5f, 0, 1) * (v - vbatFast);
  vbatSlow += ctl::clampf(dt / 3.0f, 0, 1) * (v - vbatSlow);
  batPresent = vbatSlow > VBAT_PRESENT_V;
  if (!batPresent) {
    lowWarn = lowCrit = false;
    return;
  }
  const float HYST = 0.2f;
  lowWarn = lowWarn ? (vbatSlow < P(P_LOWBAT_WARN) + HYST) : (vbatSlow < P(P_LOWBAT_WARN));
  lowCrit = lowCrit ? (vbatSlow < P(P_LOWBAT_CRIT) + HYST) : (vbatSlow < P(P_LOWBAT_CRIT));
  // Solo avisos para el piloto (BATERÍA BAJA / CRÍTICA en la app). Decisión del equipo (21 oct):
  // sin emersión automática por batería; el piloto ve el voltaje todo el tiempo y decide.
}

static void control(uint32_t now, float dt) {
  // --- Enlace / failsafe
  linkOk = lastCmdMs != 0 && (int32_t)(now - lastCmdMs) < (int32_t)FAILSAFE_MS;  // con signo: robusto
  if (!linkOk) {
    if (armed) armBlocked = true;
    armed = false;
    linkLostS += dt;
  } else {
    linkLostS = 0;
  }

  updateBattery(dt);
  surfacing = lastCmdMs != 0 && linkLostS >= P(P_SURFACE_LINKLOSS);

  limFull = digitalRead(PIN_LIM_FULL) == LIM_PRESSED_LEVEL;
  limEmpty = digitalRead(PIN_LIM_EMPTY) == LIM_PRESSED_LEVEL;

  float surge = cmd.surge / 1000.0f, yaw = cmd.yaw / 1000.0f;
  float pitchIn = cmd.pitch / 1000.0f, vert = cmd.vert / 1000.0f;
  float power = cmd.power / 100.0f;
  if (!linkOk) surge = yaw = pitchIn = vert = 0;

  float target[MOTOR_COUNT] = {0};
  // Sin limitación de empuje por batería (decisión del equipo, 21 oct): la batería solo genera avisos.

  // --- Horizontales + cabeceo (solo armado)
  pitchHoldOn = armed && (cmd.flags & proto::CF_PITCH_HOLD) && imu.ok();
  if (armed) {
    float h[4];
    ctl::mixHorizontal(surge, yaw, power, h);
    for (int i = 0; i < 4; i++) target[i] = h[i];

    if (pitchHoldOn) {
      pitchSp = pitchIn * P(P_PITCH_SP_MAX);
      ctl::PitchGains g = {P(P_PITCH_KP), P(P_PITCH_KI), P(P_PITCH_KD), P(P_PITCH_MAX_OUT)};
      float pitchNow = att.pitch - P(P_IMU_PITCH_OFS);
      target[M_PITCH] = pitchPid.update(pitchSp, pitchNow, att.pitchRate, dt, g);
    } else {
      pitchPid.reset();
      pitchSp = 0;
      target[M_PITCH] = pitchIn;
    }
  } else {
    pitchPid.reset();
    pitchSp = 0;
  }

  // --- Lastre (funciona armado o desarmado: no es propulsión)
  depthHoldOn = linkOk && (cmd.flags & proto::CF_DEPTH_HOLD) && depthValid;
  if (depthHoldOn && !prevDepthHold) depthHold.reset(depthM, ballast.pos);
  prevDepthHold = depthHoldOn;

  int8_t req = 0;
  if (surfacing) {
    req = -1;  // vaciar para emerger
  } else if (!linkOk) {
    req = 0;
  } else if (depthHoldOn) {
    ctl::DepthHoldParams hp = {P(P_DEPTH_BAND), P(P_DEPTH_VMAX), P(P_DEPTH_KP),
                               P(P_DEPTH_KV),   P(P_DEPTH_KI),   P(P_HOLD_RATE)};
    req = depthHold.update(depthM, vzMs, ballast.pos, vert, dt, hp);
  } else {
    if (vert > 0.3f) req = -1;       // stick arriba -> vaciar -> subir
    else if (vert < -0.3f) req = +1;  // stick abajo  -> llenar -> bajar
  }
  if (!depthHoldOn) depthHold.setpoint = depthM;
  int8_t bdir = ballast.update(req, limFull, limEmpty, dt, P(P_BALLAST_STROKE), P(P_BALLAST_TIMEOUT));
  target[M_BALLAST] = bdir;  // siempre al 100 %: la posición se estima con el tiempo de carrera

  // --- Salidas: rampa en thrusters, lastre directo
  float step = P(P_RAMP_PER_S) * dt;
  for (int m = 0; m < 5; m++) {
    // Desarmado (o failsafe): corte inmediato, sin rampa. Armado: rampa para limitar picos de corriente.
    motorOut[m] = armed ? ctl::ramp(motorOut[m], target[m], step) : 0.0f;
    motorSet(m, motorOut[m], P(P_MOTOR_MIN_DUTY));
  }
  motorOut[M_BALLAST] = target[M_BALLAST];
  motorSet(M_BALLAST, motorOut[M_BALLAST]);
}

#if DEBUG_PRINTS
static void debugPrint() {
  DBG("[%lus] link=%d arm=%d hold(p=%d d=%d) depth=%.3f sp=%.3f vz=%.3f pitch=%.1f roll=%.1f T=%.2f V=%.2f "
      "lastre=%d pos=%s%.0f%% lim(F=%d E=%d) surf=%d err=%lu Tbad=%u\n",
      (unsigned long)(millis() / 1000), linkOk, armed, pitchHoldOn, depthHoldOn, depthM, depthHold.setpoint, vzMs,
      att.pitch - P(P_IMU_PITCH_OFS), att.roll - P(P_IMU_ROLL_OFS), waterTempC, vbatFast, ballast.dir,
      ballast.posKnown ? "" : "~", ballast.pos * 100, limFull, limEmpty, surfacing, (unsigned long)rovLink.errors(), tempBadCount);
}
#endif

// ------------------------------------------------------------------ Arduino
void setup() {
  motorsBegin();  // primero: todos los motores quietos
#if DEBUG_PRINTS
  Serial.begin(115200);
  delay(50);
  Serial.println("\nAquaBot ROV — arrancando");
#endif
  paramsBegin();

  pinMode(PIN_LIM_FULL, INPUT);  // GPIO34/35: solo entrada, pull-up externo
  pinMode(PIN_LIM_EMPTY, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(10);

  bool baroOk = baro.begin(Wire, MS5837_ADDR);
  bool imuOk = imu.begin(Wire, MPU6050_ADDR);
  if (imuOk) imu.calibrateGyro(300);  // ~1 s, el ROV debe estar quieto
  (void)baroOk;  // solo se usan en los mensajes de depuración

  ds18.begin();
  ds18.setResolution(12);
  ds18.setWaitForConversion(false);
  ds18.requestTemperatures();
  tempReqMs = millis();

  DBG("MS5837: %s (CRC %s) | MPU6050: %s (WHO_AM_I 0x%02X) | DS18B20: %d sensor(es)\n", baroOk ? "OK" : "FALLA",
      baro.crcOk() ? "ok" : "mal", imuOk ? "OK" : "FALLA", imu.whoAmI(), ds18.getDeviceCount());
  DBG("Enlace: %s a %d baud\n", LINK_USB ? "USB" : "RS-485 (Serial2)", LINK_BAUD);

  rovLink.begin();
  enableLoopWDT();  // reinicia la ESP32 si el loop se cuelga (~5 s)
  tImu = tCtrl = tDbg = millis();
}

void loop() {
  uint32_t now = millis();

  // Mismo 'now' para todo el ciclo: si lastCmdMs quedara 1 ms por delante de 'now', la resta sin
  // signo daría ~4.29e9 y el ROV se desarmaría solo (falla encontrada el 21 oct).
  while (rovLink.poll()) handleFrame(now);

  baro.update(now);
  if (baro.takeFresh()) processPressure(now);
  if (!baro.ok()) depthValid = false;

  readImu(now);
  readTemperature(now);

  if (now - tCtrl >= CONTROL_PERIOD_MS) {
    float dt = (now - tCtrl) / 1000.0f;
    tCtrl = now;
    control(now, dt > 0.2f ? 0.2f : dt);
  }

#if DEBUG_PRINTS
  if (now - tDbg >= 1000) {
    tDbg = now;
    debugPrint();
  }
#endif
}
