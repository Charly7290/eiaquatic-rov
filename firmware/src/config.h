// AquaBot — configuración de hardware. Todo lo que depende del cableado está aquí.
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------- Enlace
// LINK_USB=1 (env rov_usb): el protocolo va por el USB de la devboard (pruebas de banco).
// LINK_USB=0 (env rov):     el protocolo va por RS-485 (Serial2) y el USB queda para debug.
#ifndef LINK_USB
#define LINK_USB 0
#endif
#define LINK_BAUD        38400   // 22 oct: bajado de 115200; bits 3x más largos = más tolerancia al ruido de motores. DEBE coincidir con "baudios" en pc/config.json
#define PIN_RS485_RX     16   // RO del transceptor (con divisor si es MAX485 a 5 V)
#define PIN_RS485_TX     17   // DI
#define PIN_RS485_DE     4    // DE + RE# unidos: HIGH transmite, LOW recibe
#define FRAME_GAP_MS     20   // un hueco mayor entre bytes reinicia el parser

// ---------------------------------------------------------------- Motores
// Índices (vistos desde atrás del ROV, verificado en bring-up 21 oct):
//   M1 izq-inferior, M2 izq-superior, M3 der-superior, M4 der-inferior,
//   M5 cabeceo (vertical frontal), M6 lastre.
// (Los nombres del enum M_LF/M_LR/M_RF/M_RR son históricos: solo importa el lado izq/der.)
enum MotorId : uint8_t { M_LF = 0, M_LR = 1, M_RF = 2, M_RR = 3, M_PITCH = 4, M_BALLAST = 5, MOTOR_COUNT = 6 };

// {IN1, IN2} de cada DRV8871 (pinout aprobado, sección 4.2b del estado del proyecto).
static const uint8_t MOTOR_PINS[MOTOR_COUNT][2] = {
    {32, 33},  // M1
    {25, 26},  // M2
    {27, 14},  // M3
    {13, 12},  // M4  (GPIO12 = pin de arranque: 10k a GND en la baquela)
    {18, 19},  // M5
    {23, 2},   // M6  (GPIO2 = pin de arranque y LED azul de la DevKit V1)
};

// Sentido de cada motor: +1 normal, -1 invertido. Se ajusta con el programa de bring-up:
//  - M1..M4: con comando positivo deben empujar el ROV hacia ADELANTE.
//  - M5:     con comando positivo debe LEVANTAR la nariz.
//  - M6:     con comando positivo debe LLENAR el lastre (tomar agua).
// Bring-up 21 oct: M3 y M4 salieron invertidos.
static const int8_t MOTOR_DIR[MOTOR_COUNT] = {+1, +1, -1, -1, +1, +1};

// Modo de PWM del DRV8871. 1 = decaimiento lento (recomendado: con rápido, los BM70 no arrancaban
// por debajo de ~60 % de duty, bring-up 21 oct). 0 = decaimiento rápido (comportamiento anterior).
#define MOTOR_SLOW_DECAY 1

#define PWM_FREQ_HZ  20000  // (22 oct: se probó 5 kHz y el enlace perdió más; se vuelve a 20 kHz)
#define PWM_BITS     10
#define PWM_MAX      ((1u << PWM_BITS) - 1)

// ---------------------------------------------------------------- Sensores
#define PIN_SDA          21
#define PIN_SCL          22
#define I2C_CLOCK_HZ     100000
#define MS5837_ADDR      0x76
#define MPU6050_ADDR     0x68
#define PIN_DS18B20      5

// Microswitches del lastre (GPIO34/35 con 10k a 3V3; el switch conecta a GND).
// LIM_PRESSED_LEVEL = nivel que lee el pin con el switch PRESIONADO:
//   HIGH -> cableado COM/NC (recomendado: un cable roto también detiene el motor).
//   LOW  -> cableado COM/NO (cableado actual, 21 oct): presionado = 0, libre = 1.
//           Ojo: con NO, un cable suelto se lee como "libre" y el motor no se detiene en el tope.
#define LIM_PRESSED_LEVEL LOW
#define PIN_LIM_FULL     34   // switch A: lastre LLENO  (detiene el llenado)
#define PIN_LIM_EMPTY    35   // switch B: lastre VACÍO  (detiene el vaciado)

// Voltaje de batería: divisor 47k / 10k en GPIO39 (ADC1).
#define PIN_VBAT         39
#define VBAT_DIVIDER     (57.0f / 10.0f)
#define VBAT_PRESENT_V   6.0f  // por debajo: no hay batería (solo USB: el divisor lee ~3.6 V), sin alarmas.
                               // Un 3S nunca baja de ~9 V con carga, así que 6 V no oculta una batería real.

// Orientación de la MPU6050 respecto al cuerpo del ROV.
// Ejes del cuerpo: X hacia adelante, Y hacia la izquierda, Z hacia arriba.
// Cada entrada = {eje del sensor (0=x, 1=y, 2=z), signo}. Se verifica con el bring-up:
// ROV nivelado -> az ≈ +1 g; nariz arriba -> pitch positivo; lado derecho abajo -> roll positivo.
// Montaje en el ROV (16 oct): +Z del sensor hacia la proa, +X hacia arriba, Y hacia los lados.
// Signos verificados en bring-up (21 oct): nariz arriba -> pitch +, lado derecho abajo -> roll +.
// LEFT sale de la regla de la mano derecha: izquierda = arriba × adelante = X × Z = −Y.
static const int8_t IMU_MAP_FWD[2]  = {2, +1};
static const int8_t IMU_MAP_LEFT[2] = {1, -1};
static const int8_t IMU_MAP_UP[2]   = {0, +1};

// ---------------------------------------------------------------- Tiempos
#define CONTROL_PERIOD_MS   20    // 50 Hz
#define IMU_PERIOD_MS       10    // 100 Hz
#define TEMP_PERIOD_MS      1000  // DS18B20 a 12 bits tarda 750 ms
#define FAILSAFE_MS         1000  // sin CMD válido 1 s -> thrusters a 0 y desarmado (21 oct: 0.5 s era muy estricto con tether largo)
#define DEPTH_ZERO_SAMPLES  25    // ~1 s de muestras para el cero de profundidad

// Mensajes de depuración por el USB solo cuando el enlace NO usa el USB.
#define DEBUG_PRINTS (!LINK_USB)
