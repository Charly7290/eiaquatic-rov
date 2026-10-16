#include "ms5837.h"

#include <Arduino.h>

static const uint8_t CMD_RESET = 0x1E;
static const uint8_t CMD_ADC_READ = 0x00;
static const uint8_t CMD_PROM = 0xA0;
static const uint8_t CMD_D1_8192 = 0x4A;
static const uint8_t CMD_D2_8192 = 0x5A;
static const uint32_t CONV_MS = 20;  // OSR 8192: 18.08 ms máx.

bool MS5837::command(uint8_t c) {
  wire_->beginTransmission(addr_);
  wire_->write(c);
  return wire_->endTransmission() == 0;
}

bool MS5837::begin(TwoWire& wire, uint8_t addr) {
  wire_ = &wire;
  addr_ = addr;
  ok_ = false;
  st_ = IDLE;
  fails_ = 0;
  if (!command(CMD_RESET)) return false;
  delay(10);
  for (uint8_t i = 0; i < 7; i++) {
    if (!command(CMD_PROM + i * 2)) return false;
    if (wire_->requestFrom(addr_, (uint8_t)2) != 2) return false;
    C_[i] = ((uint16_t)wire_->read() << 8) | wire_->read();
  }
  C_[7] = 0;
  uint8_t crcRead = C_[0] >> 12;
  uint16_t copy[8];
  for (int i = 0; i < 8; i++) copy[i] = C_[i];
  crcOk_ = (crc4(copy) == crcRead);
  ok_ = crcOk_;
  return ok_;
}

bool MS5837::readAdc(uint32_t& v) {
  if (!command(CMD_ADC_READ)) return false;
  if (wire_->requestFrom(addr_, (uint8_t)3) != 3) return false;
  v = ((uint32_t)wire_->read() << 16);
  v |= ((uint32_t)wire_->read() << 8);
  v |= wire_->read();
  return v != 0;  // 0 = conversión no terminada o inválida
}

void MS5837::update(uint32_t now) {
  if (!wire_) return;
  if (!ok_) {
    if (now - lastRetry_ >= 1000) {  // reintento de inicialización cada segundo
      lastRetry_ = now;
      begin(*wire_, addr_);
    }
    return;
  }
  bool good = true;
  switch (st_) {
    case IDLE:
      good = command(CMD_D1_8192);
      if (good) {
        st_ = CONV_D1;
        t0_ = now;
      }
      break;
    case CONV_D1:
      if (now - t0_ < CONV_MS) return;
      good = readAdc(D1_) && command(CMD_D2_8192);
      st_ = good ? CONV_D2 : IDLE;
      t0_ = now;
      break;
    case CONV_D2:
      if (now - t0_ < CONV_MS) return;
      good = readAdc(D2_);
      st_ = IDLE;
      if (good) {
        calculate();
        fresh_ = true;
        samples_++;
        fails_ = 0;
      }
      break;
  }
  if (!good && ++fails_ > 10) ok_ = false;
}

// Fórmulas de la hoja de datos del MS5837-30BA (compensación de 1er y 2do orden).
void MS5837::calculate() {
  int32_t dT = (int32_t)D2_ - (int32_t)((uint32_t)C_[5] * 256u);
  int64_t SENS = (int64_t)C_[1] * 32768LL + ((int64_t)C_[3] * dT) / 256LL;
  int64_t OFF = (int64_t)C_[2] * 65536LL + ((int64_t)C_[4] * dT) / 128LL;
  int64_t temp = 2000LL + ((int64_t)dT * C_[6]) / 8388608LL;

  int64_t Ti, OFFi, SENSi;
  int64_t t20 = temp - 2000;
  if (temp < 2000) {
    Ti = (3LL * dT * dT) / 8589934592LL;
    OFFi = (3LL * t20 * t20) / 2;
    SENSi = (5LL * t20 * t20) / 8;
    if (temp < -1500) {
      int64_t t15 = temp + 1500;
      OFFi += 7LL * t15 * t15;
      SENSi += 4LL * t15 * t15;
    }
  } else {
    Ti = (2LL * dT * dT) / 137438953472LL;
    OFFi = (t20 * t20) / 16;
    SENSi = 0;
  }
  int64_t OFF2 = OFF - OFFi;
  int64_t SENS2 = SENS - SENSi;
  TEMP_ = (int32_t)(temp - Ti);
  P_ = (int32_t)((((int64_t)D1_ * SENS2) / 2097152LL - OFF2) / 8192LL);
}

uint8_t MS5837::crc4(uint16_t n_prom[8]) {
  uint16_t n_rem = 0;
  n_prom[0] = n_prom[0] & 0x0FFF;
  n_prom[7] = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (i % 2 == 1) n_rem ^= (uint16_t)(n_prom[i >> 1] & 0x00FF);
    else n_rem ^= (uint16_t)(n_prom[i >> 1] >> 8);
    for (uint8_t bit = 8; bit > 0; bit--) {
      if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
      else n_rem = (n_rem << 1);
    }
  }
  return (n_rem >> 12) & 0x000F;
}
