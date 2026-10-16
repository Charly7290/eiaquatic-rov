// AquaBot — driver NO bloqueante del MS5837-30BA (presión/profundidad).
// La librería habitual bloquea ~40 ms por lectura; esta reparte la conversión en una máquina de
// estados (D1 -> 20 ms -> D2 -> 20 ms -> cálculo), ~25 muestras por segundo.
#pragma once
#include <Wire.h>
#include <stdint.h>

class MS5837 {
 public:
  bool begin(TwoWire& wire, uint8_t addr = 0x76);  // reset + PROM + CRC (bloquea ~15 ms)
  void update(uint32_t nowMs);                     // llamar en cada vuelta del loop
  bool ok() const { return ok_; }
  bool crcOk() const { return crcOk_; }
  bool takeFresh() {  // true una vez por muestra nueva
    bool f = fresh_;
    fresh_ = false;
    return f;
  }
  float pressurePa() const { return P_ * 10.0f; }      // P_ en 0.1 mbar (30BA)
  float pressureMbar() const { return P_ / 10.0f; }
  float temperatureC() const { return TEMP_ / 100.0f; }
  uint32_t samples() const { return samples_; }

 private:
  bool command(uint8_t c);
  bool readAdc(uint32_t& v);
  void calculate();
  static uint8_t crc4(uint16_t prom[8]);

  TwoWire* wire_ = nullptr;
  uint8_t addr_ = 0x76;
  uint16_t C_[8] = {0};
  uint32_t D1_ = 0, D2_ = 0;
  int32_t TEMP_ = 0, P_ = 0;
  enum State : uint8_t { IDLE, CONV_D1, CONV_D2 } st_ = IDLE;
  uint32_t t0_ = 0, lastRetry_ = 0;
  uint8_t fails_ = 0;
  bool ok_ = false, crcOk_ = false, fresh_ = false;
  uint32_t samples_ = 0;
};
