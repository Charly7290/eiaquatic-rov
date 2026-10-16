#include "mpu6050.h"

#include <Arduino.h>

#include "config.h"

static const float ACC_LSB_PER_G = 8192.0f;   // ±4 g
static const float GYR_LSB_PER_DPS = 65.5f;   // ±500 °/s

bool MPU6050::writeReg(uint8_t reg, uint8_t val) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write(val);
  return wire_->endTransmission() == 0;
}

bool MPU6050::begin(TwoWire& wire, uint8_t addr) {
  wire_ = &wire;
  addr_ = addr;
  ok_ = writeReg(0x6B, 0x01)      // despertar, reloj = PLL del giroscopio X
        && writeReg(0x1A, 0x03)   // DLPF ~44 Hz (filtra vibración de motores)
        && writeReg(0x19, 0x04)   // 1 kHz / 5 = 200 Hz
        && writeReg(0x1B, 0x08)   // giroscopio ±500 °/s
        && writeReg(0x1C, 0x08);  // acelerómetro ±4 g
  wire_->beginTransmission(addr_);
  wire_->write(0x75);
  if (wire_->endTransmission() == 0 && wire_->requestFrom(addr_, (uint8_t)1) == 1) who_ = wire_->read();
  fails_ = 0;
  return ok_;
}

bool MPU6050::read() {
  if (!wire_) return false;
  wire_->beginTransmission(addr_);
  wire_->write(0x3B);
  bool good = wire_->endTransmission(false) == 0 && wire_->requestFrom(addr_, (uint8_t)14) == 14;
  if (!good) {
    if (++fails_ > 10) ok_ = false;
    return false;
  }
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (int16_t)((wire_->read() << 8) | wire_->read());
  for (int i = 0; i < 3; i++) {
    a[i] = raw[i] / ACC_LSB_PER_G;
    g[i] = raw[4 + i] / GYR_LSB_PER_DPS - bias_[i];  // raw[3] = temperatura interna
  }
  fails_ = 0;
  ok_ = true;
  return true;
}

void MPU6050::calibrateGyro(int n) {
  float sum[3] = {0, 0, 0};
  bias_[0] = bias_[1] = bias_[2] = 0;
  int got = 0;
  for (int k = 0; k < n; k++) {
    if (read()) {
      for (int i = 0; i < 3; i++) sum[i] += g[i];
      got++;
    }
    delay(3);
  }
  if (got > 0)
    for (int i = 0; i < 3; i++) bias_[i] = sum[i] / got;
}

static inline float pick(const float v[3], const int8_t map[2]) { return v[map[0]] * map[1]; }

void MPU6050::toBody(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) const {
  ax = pick(a, IMU_MAP_FWD);
  ay = pick(a, IMU_MAP_LEFT);
  az = pick(a, IMU_MAP_UP);
  gx = pick(g, IMU_MAP_FWD);
  gy = pick(g, IMU_MAP_LEFT);
  gz = pick(g, IMU_MAP_UP);
}
