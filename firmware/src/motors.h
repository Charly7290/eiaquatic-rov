// AquaBot — PWM de los 6 DRV8871 (PWM en IN1/IN2, decaimiento rápido).
#pragma once
#include <stdint.h>

void motorsBegin();
// v en -1..1 (se aplica MOTOR_DIR). minDuty: duty mínimo cuando v != 0 (zona muerta del motor).
void motorSet(uint8_t m, float v, float minDuty = 0.0f);
void motorsStopAll();
