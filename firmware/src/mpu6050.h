// AquaBot — driver mínimo de la MPU6050 (sin librerías externas).
#pragma once
#include <Wire.h>
#include <stdint.h>

class MPU6050 {
 public:
  bool begin(TwoWire& wire, uint8_t addr = 0x68);
  bool read();                   // actualiza a[] (g) y g[] (°/s, sin bias) en ejes del SENSOR
  void calibrateGyro(int n);     // ROV quieto: promedio del giroscopio (bloquea ~n*3 ms)
  void toBody(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) const;
  bool ok() const { return ok_; }
  uint8_t whoAmI() const { return who_; }
  float a[3] = {0, 0, 1}, g[3] = {0, 0, 0};

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  TwoWire* wire_ = nullptr;
  uint8_t addr_ = 0x68, who_ = 0;
  float bias_[3] = {0, 0, 0};
  bool ok_ = false;
  uint8_t fails_ = 0;
};
