#include "motors.h"

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <math.h>

#include "config.h"

// Canal LEDC = 2*motor + {0: IN1, 1: IN2} -> 12 canales (el ESP32 tiene 16).
static void pwmAttach(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, PWM_FREQ_HZ, PWM_BITS, ch);
#else
  ledcSetup(ch, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

static void pwmWrite(uint8_t m, uint8_t i, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PINS[m][i], duty);
#else
  ledcWrite(m * 2 + i, duty);
#endif
}

void motorsBegin() {
  for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
    for (uint8_t i = 0; i < 2; i++) {
      // Primero en bajo como GPIO para que el pin no flote mientras se configura el PWM.
      pinMode(MOTOR_PINS[m][i], OUTPUT);
      digitalWrite(MOTOR_PINS[m][i], LOW);
      pwmAttach(MOTOR_PINS[m][i], m * 2 + i);
      pwmWrite(m, i, 0);
    }
  }
}

void motorSet(uint8_t m, float v, float minDuty) {
  if (m >= MOTOR_COUNT) return;
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  v *= MOTOR_DIR[m];
  float a = fabsf(v);
  uint32_t duty = 0;
  if (a > 0.005f) {
    a = minDuty + a * (1.0f - minDuty);
    duty = (uint32_t)(a * PWM_MAX + 0.5f);
    if (duty > PWM_MAX) duty = PWM_MAX;
  }
  if (duty == 0) {
    pwmWrite(m, 0, 0);
    pwmWrite(m, 1, 0);
  } else if (v > 0) {
    pwmWrite(m, 1, 0);
    pwmWrite(m, 0, duty);
  } else {
    pwmWrite(m, 0, 0);
    pwmWrite(m, 1, duty);
  }
}

void motorsStopAll() {
  for (uint8_t m = 0; m < MOTOR_COUNT; m++) motorSet(m, 0);
}
