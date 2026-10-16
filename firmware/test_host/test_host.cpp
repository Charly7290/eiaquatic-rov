// Pruebas en el PC de protocol.cpp y control.cpp (sin Arduino).
//   g++ -std=c++17 -I../src test_host.cpp ../src/protocol.cpp ../src/control.cpp -o test_host && ./test_host
// Escribe vectores de prueba en vectors.txt para que pc/tests/test_protocol.py los decodifique.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "control.h"
#include "protocol.h"

static int fails = 0;
#define CHECK(c, ...)                         \
  do {                                        \
    if (!(c)) {                               \
      fails++;                                \
      printf("FALLA %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                    \
      printf("\n");                           \
    }                                         \
  } while (0)

static void hexline(FILE* f, const char* name, const uint8_t* b, size_t n) {
  fprintf(f, "%s ", name);
  for (size_t i = 0; i < n; i++) fprintf(f, "%02X", b[i]);
  fprintf(f, "\n");
}

static void testProtocol(FILE* vf) {
  const char* s = "123456789";
  CHECK(proto::crc16((const uint8_t*)s, 9) == 0x29B1, "CRC de control");

  proto::Cmd c = {7, proto::CF_ARM | proto::CF_DEPTH_HOLD, 500, -250, 1000, -1000, 80, 0};
  uint8_t buf[proto::MAX_FRAME];
  size_t n = proto::encode(proto::CMD, &c, sizeof(c), buf);
  CHECK(n == 18, "largo trama CMD");
  hexline(vf, "CMD", buf, n);

  proto::Parser p;
  int got = 0;
  uint8_t noise[] = {0x00, 0xAA, 0x13, 0xAA};
  for (uint8_t b : noise) got += p.feed(b);
  for (size_t i = 0; i < n; i++) got += p.feed(buf[i]);
  CHECK(got == 1 && p.type() == proto::CMD && p.len() == sizeof(c), "parser CMD");
  proto::Cmd d;
  memcpy(&d, p.payload(), sizeof(d));
  CHECK(d.surge == 500 && d.yaw == -250 && d.vert == -1000 && d.power == 80, "contenido CMD");

  buf[6] ^= 0x01;  // corromper
  got = 0;
  for (size_t i = 0; i < n; i++) got += p.feed(buf[i]);
  CHECK(got == 0 && p.errors() == 1, "CRC corrupto descartado");

  proto::Telem t = {};
  t.seq = 7;
  t.flags = proto::TF_ARMED | proto::TF_DEPTH_OK | proto::TF_IMU_OK;
  t.depth_mm = 1234;
  t.depth_sp_mm = 1000;
  t.vz_mms = -25;
  t.pressure_mbar = 1135;
  t.water_temp_cC = 2456;
  t.ms_temp_cC = 2500;
  t.pitch_cdeg = -150;
  t.roll_cdeg = 75;
  t.pitch_sp_cdeg = 0;
  t.vbat_mV = 11420;
  t.ballast_pos = 42;
  t.ballast_dir = -1;
  int8_t mot[5] = {50, 50, -30, -30, 12};
  memcpy(t.motor, mot, 5);
  t.uptime_ms = 123456;
  t.rx_err = 3;
  n = proto::encode(proto::TELEM, &t, sizeof(t), buf);
  CHECK(n == 42, "largo trama TELEM");
  hexline(vf, "TELEM", buf, n);

  proto::ParamValue pv = {5, 23, 0.1f};
  n = proto::encode(proto::PARAM_VALUE, &pv, sizeof(pv), buf);
  hexline(vf, "PARAM_VALUE", buf, n);
}

static void testMixer() {
  float o[4];
  ctl::mixHorizontal(1, 0, 1, o);
  CHECK(o[0] == 1 && o[2] == 1, "avance puro");
  ctl::mixHorizontal(0, 1, 1, o);
  CHECK(o[0] == 1 && o[2] == -1, "giro a la derecha: izq adelante, der atrás");
  ctl::mixHorizontal(1, 1, 0.5f, o);
  CHECK(fabsf(o[0] - 0.5f) < 1e-6 && fabsf(o[2]) < 1e-6, "normalización + potencia");
}

static void testBallast() {
  ctl::Ballast b;
  int8_t out = b.update(+1, false, false, 0.02f, 20, 40);
  CHECK(out == 1 && !b.posKnown, "llenar libre");
  out = b.update(+1, true, false, 0.02f, 20, 40);
  CHECK(out == 0 && b.posKnown && b.pos == 1.0f, "switch LLENO detiene y referencia");
  out = b.update(-1, true, false, 0.02f, 20, 40);
  CHECK(out == -1, "vaciar desde lleno");
  for (int i = 0; i < 250; i++) b.update(-1, false, false, 0.02f, 20, 40);  // 5 s
  CHECK(fabsf(b.pos - 0.75f) < 0.01f, "posición estimada %.3f", b.pos);
  out = b.update(-1, true, true, 0.02f, 20, 40);
  CHECK(out == 0 && b.switchFault, "ambos switches = falla");
  ctl::Ballast t;
  for (int i = 0; i < 300; i++) out = t.update(+1, false, false, 0.02f, 20, 5);  // 6 s > 5 s
  CHECK(out == 0 && t.timeoutFault, "timeout");
  out = t.update(0, false, false, 0.02f, 20, 5);
  out = t.update(+1, false, false, 0.02f, 20, 5);
  CHECK(out == 1 && !t.timeoutFault, "timeout se limpia al soltar");
}

static void testPitch() {
  ctl::PitchPID pid;
  ctl::PitchGains g = {0.03f, 0.01f, 0.005f, 0.6f};
  float u = pid.update(0, -10, 0, 0.02f, g);
  CHECK(u > 0.29f && u < 0.32f, "nariz abajo -> empuje positivo (sube nariz), u=%.3f", u);
  for (int i = 0; i < 5000; i++) u = pid.update(0, -40, 0, 0.02f, g);
  CHECK(u == 0.6f && pid.integral() <= 0.6f / 0.01f + 1e-3f, "saturación y anti-windup");
}

// Planta simple: lastre -> flotabilidad -> velocidad -> profundidad, con arrastre.
// (El barrido completo de robustez está en robust.cpp.)
static void testDepthHold() {
  ctl::DepthHoldParams p = {0.05f, 0.10f, 0.15f, 0.5f, 0.15f, 0.10f};
  ctl::DepthHold h;
  ctl::Ballast b;
  float depth = 0.3f, vz = 0, bTrue = 0.5f;
  const float neutral = 0.6f;  // el neutro real NO es la posición inicial: el integrador debe aprenderlo
  b.pos = 0.5f;
  h.reset(1.0f, b.pos);
  float maxOvershoot = 0, errAfter = 0;
  const float dt = 0.02f;
  for (int i = 0; i < (int)(240 / dt); i++) {
    int8_t req = h.update(depth, vz, b.pos, 0, dt, p);
    int8_t out = b.update(req, bTrue >= 1, bTrue <= 0, dt, 20, 60);
    bTrue = ctl::clampf(bTrue + out * dt / 20, 0, 1);
    float acc = 0.6f * (bTrue - neutral) - 2.0f * vz * fabsf(vz) - 0.3f * vz;
    vz += acc * dt;
    depth += vz * dt;
    if (depth < 0) {
      depth = 0;
      vz = 0;
    }
    if (depth > 1.0f) maxOvershoot = fmaxf(maxOvershoot, depth - 1.0f);
    if (i * dt > 150) errAfter = fmaxf(errAfter, fabsf(depth - 1.0f));
  }
  printf("  hold de profundidad: sobrepaso %.2f m, error máx. después de 150 s %.2f m, neutro aprendido %.2f\n",
         maxOvershoot, errAfter, h.neutral);
  CHECK(errAfter < 0.10f, "el hold no se mantiene cerca del setpoint");
  CHECK(fabsf(h.neutral - neutral) < 0.05f, "el integrador no aprendió el neutro");

  // Stick: con el hold activo, el stick arriba hace el setpoint menos profundo.
  ctl::DepthHold s;
  s.reset(1.0f, 0.5f);
  for (int i = 0; i < 100; i++) s.update(1.0f, 0, 0.5f, 1.0f, 0.02f, p);  // 2 s a fondo
  CHECK(fabsf(s.setpoint - 0.8f) < 1e-3f, "setpoint con stick: %.3f", s.setpoint);
}

int main() {
  FILE* vf = fopen("vectors.txt", "w");
  testProtocol(vf);
  fclose(vf);
  testMixer();
  testBallast();
  testPitch();
  testDepthHold();
  printf(fails ? "\n%d FALLA(S)\n" : "\nTodas las pruebas OK\n", fails);
  return fails ? 1 : 0;
}
