// AquaBot — programa de BRING-UP (integración y verificación de hardware).
// Menú por el monitor serial (USB, 115200). Escriba 'h' para la ayuda.
//
// ¡ATENCIÓN! Las pruebas de motores mueven las hélices: hélices libres, ROV sujeto o en agua.
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>

#include "config.h"
#include "control.h"
#include "motors.h"
#include "mpu6050.h"
#include "ms5837.h"
#include "params.h"

static MS5837 baro;
static MPU6050 imu;
static OneWire oneWire(PIN_DS18B20);
static DallasTemperature ds18(&oneWire);
static ctl::Attitude att;

static float duty = 0.30f;
static bool streaming = false;
static float p0Mbar = 0;
static uint32_t tStream = 0, tImu = 0, tTemp = 0;
static float waterTemp = NAN;

static const char* MOTOR_NAMES[MOTOR_COUNT] = {"M1 izq-delantero", "M2 izq-trasero", "M3 der-delantero",
                                               "M4 der-trasero",   "M5 cabeceo",     "M6 lastre"};
static const char* MOTOR_EXPECT[MOTOR_COUNT] = {
    "empujar el ROV hacia ADELANTE", "empujar el ROV hacia ADELANTE", "empujar el ROV hacia ADELANTE",
    "empujar el ROV hacia ADELANTE", "LEVANTAR la nariz",             "LLENAR el lastre (tomar agua)"};

static bool limFull() { return digitalRead(PIN_LIM_FULL) == HIGH; }
static bool limEmpty() { return digitalRead(PIN_LIM_EMPTY) == HIGH; }
static float vbat() { return analogReadMilliVolts(PIN_VBAT) * VBAT_DIVIDER / 1000.0f; }

// Lee el serial sin bloquear: true si llegó 'x' (abortar).
static bool abortRequested() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') return true;
  }
  return false;
}

// Espera ms milisegundos manteniendo los sensores al día. false si se abortó.
static bool waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    baro.update(millis());
    if (abortRequested()) return false;
    delay(2);
  }
  return true;
}

static void help() {
  Serial.println(F("\n=== AquaBot BRING-UP ==="));
  Serial.println(F(" h  ayuda"));
  Serial.println(F(" i  escanear bus I2C (esperado: 0x68 MPU6050, 0x76 MS5837)"));
  Serial.println(F(" s  activar/desactivar lectura continua de sensores (5 Hz)"));
  Serial.println(F(" z  cero de presión (tomar la presión actual como superficie)"));
  Serial.println(F(" 1..6  probar motor N: +duty 1.5 s, pausa, -duty 1.5 s"));
  Serial.println(F(" + / -  subir/bajar duty de prueba (pasos de 10 %)"));
  Serial.println(F(" f / e  lastre: llenar / vaciar 1 s (respeta fines de carrera)"));
  Serial.println(F(" b  caracterizar lastre: 3 carreras completas en cada sentido"));
  Serial.println(F(" x  PARAR todo / abortar"));
  Serial.printf(" duty actual: %.0f %%\n", duty * 100);
}

static void i2cScan() {
  Serial.println(F("Escaneando I2C..."));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  dispositivo en 0x%02X%s\n", a,
                    a == 0x68 ? " (MPU6050)" : a == 0x76 ? " (MS5837)" : "");
      n++;
    }
  }
  Serial.printf("%d dispositivo(s)\n", n);
}

static void testMotor(uint8_t m) {
  Serial.printf("\n%s: con duty POSITIVO debe %s.\n", MOTOR_NAMES[m], MOTOR_EXPECT[m]);
  Serial.println(F("Si gira al revés, cambie su signo en MOTOR_DIR (config.h). 'x' aborta."));
  for (int dir = 1; dir >= -1; dir -= 2) {
    if (m == M_BALLAST && ((dir > 0 && limFull()) || (dir < 0 && limEmpty()))) {
      Serial.println(F("  (fin de carrera activo en ese sentido, se omite)"));
      continue;
    }
    Serial.printf("  %s %.0f %%\n", dir > 0 ? "POSITIVO" : "NEGATIVO", duty * 100);
    motorSet(m, dir * duty);
    uint32_t t0 = millis();
    bool ok = true;
    while (millis() - t0 < 1500) {
      if (abortRequested()) {
        ok = false;
        break;
      }
      if (m == M_BALLAST && ((dir > 0 && limFull()) || (dir < 0 && limEmpty()))) {
        Serial.println(F("  fin de carrera alcanzado"));
        break;
      }
      delay(5);
    }
    motorSet(m, 0);
    if (!ok) {
      Serial.println(F("  ABORTADO"));
      return;
    }
    if (!waitMs(600)) return;
  }
  Serial.println(F("  listo"));
}

static void jogBallast(int dir) {
  if ((dir > 0 && limFull()) || (dir < 0 && limEmpty())) {
    Serial.println(F("Fin de carrera activo en ese sentido."));
    return;
  }
  motorSet(M_BALLAST, dir);
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    if ((dir > 0 && limFull()) || (dir < 0 && limEmpty()) || abortRequested()) break;
    delay(2);
  }
  motorSet(M_BALLAST, 0);
  Serial.printf("Lastre: switches LLENO=%d VACIO=%d\n", limFull(), limEmpty());
}

// Mueve el lastre hasta el switch indicado. Devuelve el tiempo en s, o -1 si timeout/abortado.
static float runToLimit(int dir, uint32_t timeoutMs) {
  if (limFull() && limEmpty()) {
    Serial.println(F("  ¡Ambos switches en ALTO! Revise el cableado (COM/NC)."));
    return -1;
  }
  bool (*lim)() = dir > 0 ? limFull : limEmpty;
  if (lim()) return 0;
  motorSet(M_BALLAST, dir);
  uint32_t t0 = millis();
  float result = -1;
  while (true) {
    if (lim()) {
      result = (millis() - t0) / 1000.0f;
      break;
    }
    if (millis() - t0 > timeoutMs) {
      Serial.println(F("  TIMEOUT: el switch no se activó. ¿Switch invertido o motor al revés?"));
      break;
    }
    if (abortRequested()) {
      Serial.println(F("  ABORTADO"));
      break;
    }
    delay(1);
  }
  motorSet(M_BALLAST, 0);
  return result;
}

static void characterizeBallast() {
  const uint32_t TIMEOUT = 120000;
  Serial.println(F("\nCaracterización del lastre al 100 % (use el voltaje real de motores, 8.4 V). 'x' aborta."));
  Serial.println(F("Llevando a VACÍO como punto de partida..."));
  if (runToLimit(-1, TIMEOUT) < 0) return;
  if (!waitMs(1000)) return;
  float fill[3], empty[3];
  for (int k = 0; k < 3; k++) {
    Serial.printf(" Ciclo %d: llenando...", k + 1);
    fill[k] = runToLimit(+1, TIMEOUT);
    if (fill[k] < 0) return;
    Serial.printf(" %.2f s | vaciando...", fill[k]);
    if (!waitMs(1000)) return;
    empty[k] = runToLimit(-1, TIMEOUT);
    if (empty[k] < 0) return;
    Serial.printf(" %.2f s\n", empty[k]);
    if (!waitMs(1000)) return;
  }
  float fa = (fill[0] + fill[1] + fill[2]) / 3, ea = (empty[0] + empty[1] + empty[2]) / 3;
  float stroke = fa > ea ? fa : ea;
  Serial.printf("\nPromedio: llenar %.2f s, vaciar %.2f s\n", fa, ea);
  Serial.printf("Sugerido: BALLAST_STROKE = %.1f s, BALLAST_TIMEOUT = %.0f s (ajústelos desde la app, F2)\n",
                stroke, stroke * 1.5f + 5);
}

static void streamSensors() {
  float ax, ay, az, gx, gy, gz;
  imu.toBody(ax, ay, az, gx, gy, gz);
  float pm = baro.pressureMbar();
  float depthCm = p0Mbar > 0 ? (pm - p0Mbar) * 100.0f / (997.0f * 9.80665f) * 100.0f : 0;
  Serial.printf("P=%.1f mbar prof=%.1f cm Tms=%.2f | Tagua=%.2f | acc(cuerpo)=%.2f %.2f %.2f pitch=%.1f roll=%.1f | "
                "lastre LLENO=%d VACIO=%d | Vbat=%.2f V\n",
                pm, depthCm, baro.temperatureC(), waterTemp, ax, ay, az, att.pitch, att.roll, limFull(),
                limEmpty(), vbat());
}

void setup() {
  motorsBegin();
  Serial.begin(115200);
  delay(300);
  paramsBegin();
  pinMode(PIN_LIM_FULL, INPUT);
  pinMode(PIN_LIM_EMPTY, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(10);

  bool b = baro.begin(Wire, MS5837_ADDR);
  bool m = imu.begin(Wire, MPU6050_ADDR);
  if (m) imu.calibrateGyro(300);
  ds18.begin();
  ds18.setResolution(12);
  ds18.setWaitForConversion(false);
  ds18.requestTemperatures();
  tTemp = tImu = millis();

  Serial.printf("\nMS5837: %s (CRC PROM %s)\n", b ? "OK" : "NO RESPONDE", baro.crcOk() ? "ok" : "MAL");
  Serial.printf("MPU6050: %s (WHO_AM_I 0x%02X)\n", m ? "OK" : "NO RESPONDE", imu.whoAmI());
  Serial.printf("DS18B20: %d sensor(es)", ds18.getDeviceCount());
  if (ds18.getDeviceCount() > 0)
    Serial.printf(" | alimentación: %s", ds18.isParasitePowerMode()
                                            ? "PARÁSITA (VDD/rojo sin conectar a 3V3?)"
                                            : "normal (VDD conectado)");
  Serial.println();
  Serial.printf("Vbat: %.2f V | lastre LLENO=%d VACIO=%d\n", vbat(), limFull(), limEmpty());
  help();
}

void loop() {
  uint32_t now = millis();
  baro.update(now);
  if (baro.takeFresh() && p0Mbar == 0 && baro.samples() > 5) p0Mbar = baro.pressureMbar();

  if (now - tImu >= IMU_PERIOD_MS) {
    float dt = (now - tImu) / 1000.0f;
    tImu = now;
    if (imu.read()) {
      float ax, ay, az, gx, gy, gz;
      imu.toBody(ax, ay, az, gx, gy, gz);
      att.update(ax, ay, az, gx, gy, gz, dt);
    }
  }
  if (now - tTemp >= TEMP_PERIOD_MS) {
    tTemp = now;
    float t = ds18.getTempCByIndex(0);
    waterTemp = (t == DEVICE_DISCONNECTED_C) ? NAN : t;
    // 85.00 = valor de encendido del DS18B20: no completó la conversión (falta de energía en VDD/pull-up).
    if (t == 85.0f) Serial.println(F("  DS18B20 = 85.00: conversión incompleta. Revise VDD (rojo) a 3V3 y pull-up 4.7k dato-3V3."));
    ds18.requestTemperatures();
  }
  if (streaming && now - tStream >= 200) {
    tStream = now;
    streamSensors();
  }

  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'h': case '?': help(); break;
    case 'i': i2cScan(); break;
    case 's': streaming = !streaming; break;
    case 'z': p0Mbar = baro.pressureMbar(); Serial.printf("Cero: %.1f mbar\n", p0Mbar); break;
    case '+': duty = ctl::clampf(duty + 0.1f, 0.1f, 1.0f); Serial.printf("duty %.0f %%\n", duty * 100); break;
    case '-': duty = ctl::clampf(duty - 0.1f, 0.1f, 1.0f); Serial.printf("duty %.0f %%\n", duty * 100); break;
    case 'f': jogBallast(+1); break;
    case 'e': jogBallast(-1); break;
    case 'b': characterizeBallast(); break;
    case 'x': motorsStopAll(); streaming = false; Serial.println(F("PARADO")); break;
    default:
      if (c >= '1' && c <= '6') testMotor(c - '1');
      break;
  }
}
