# AquaBot USB 2026 — Diseño de software

Versión 2 · 15 de octubre de 2026. Decisiones acordadas con el equipo. Complementa `aquabot_estado_proyecto.md` (hardware).

---

## 1. Decisiones

| Tema | Decisión |
|---|---|
| ROS2 | Fuera del camino crítico de la competencia. Si se quiere después, nodo puente en el PC sobre el mismo protocolo; el firmware no cambia (cierra #17 hacia "puente en PC"). |
| Protocolo | Binario con CRC16, PC maestro a 20 Hz, ROV responde. 115200 8N1 (cierra #24). |
| Firmware | PlatformIO, core Arduino-ESP32 2.0.x fijado. Bucle cooperativo sin bloqueos. Dos programas: `rov` y `bringup`. |
| Propulsión | 4 horizontales alineados (2 por lado): avance + giro, sin lateral. M5 vertical frontal: cabeceo. |
| Cabeceo | PID con la IMU (P + I limitado + D con el giroscopio). El I compensa el momento del lastre trasero. Solo con el ROV armado. |
| Profundidad | Lastre variable. Modo manual (trimado) y modo hold con control en cascada: profundidad → velocidad → posición del pistón (ver 3). |
| Detección (#28) | Visión clásica OpenCV, 3 detectores (cubo naranja, moneda, genérico). La visión propone, el piloto confirma. Todo grabado y registrado. |
| Interfaz | Python: pygame (ventana, mando, teclado) + OpenCV (captura, visión, grabación) + pyserial. Se graba la ventana completa. |
| Mando | Gamepad (API GameController de SDL, mapeo normalizado Xbox/PS) + teclado de respaldo. |
| PC | Linux (Ubuntu 22.04/24.04). La app también corre en Windows. |
| Failsafe | Enlace perdido 0.5 s → thrusters a 0 y desarmado. Enlace perdido 15 s, o batería crítica sostenida 10 s → vaciar lastre (emerger). |

---

## 2. Protocolo serial (#24)

**Capa física:** UART 115200 8N1. Competencia: RS-485 half-duplex (Serial2, GPIO16/17, DE/RE en GPIO4). Banco: USB de la devboard (compilación `rov_usb`).

**Maestro/esclavo:** solo el PC inicia. Cada 50 ms envía `CMD` y espera `TELEM` (timeout 40 ms). Como mucho un mensaje de parámetros por ciclo, después del `CMD` (el `CMD` nunca deja de salir a 20 Hz).

**Trama:** `AA 55 | TIPO | LEN | PAYLOAD (LEN bytes) | CRC16 (LSB, MSB)`
- CRC16-CCITT-FALSE (poli 0x1021, init 0xFFFF, sin reflejo) sobre TIPO, LEN y PAYLOAD. Valor de control: "123456789" → 0x29B1.
- Little-endian. LEN ≤ 64.

| Tipo | Dirección | Payload |
|---|---|---|
| 0x01 CMD | PC → ROV | 12 bytes |
| 0x02 PARAM_SET | PC → ROV | id u8, guardar u8 (1 = escribir en flash), valor f32 |
| 0x03 PARAM_GET | PC → ROV | id u8 |
| 0x81 TELEM | ROV → PC | 36 bytes, respuesta a CMD |
| 0x82 PARAM_VALUE | ROV → PC | id u8, nº de parámetros u8, valor f32 (id 0xFF = inválido) |

**CMD (12 bytes)** — `<BBhhhhBB`

| Campo | Tipo | Significado |
|---|---|---|
| seq | u8 | Contador |
| flags | u8 | b0 ARMAR · b1 HOLD_CABECEO · b2 HOLD_PROFUNDIDAD · b3 CERO_PROFUNDIDAD (flanco) · b4 NIVELAR_IMU (flanco, solo desarmado) |
| surge | i16 | Avance, −1000…1000 |
| yaw | i16 | Giro, + = derecha |
| pitch | i16 | Con hold: setpoint de ángulo (×PITCH_SP_MAX). Sin hold: empuje directo de M5 |
| vert | i16 | + = subir. Con hold de profundidad: velocidad del setpoint. Manual: lastre (umbral ±300) |
| power | u8 | Límite de potencia horizontal, 0–100 % |
| reservado | u8 | 0 |

**TELEM (36 bytes)** — `<BHhhhHhhhhhHBb5bIH`

| Campo | Tipo | Unidad |
|---|---|---|
| seq_eco | u8 | seq del último CMD |
| flags | u16 | b0 armado · b1 hold cabeceo · b2 hold profundidad · b3 failsafe enlace · b4 emergiendo · b5 batería baja · b6 batería crítica · b7 lastre lleno (switch) · b8 lastre vacío (switch) · b9 presión OK · b10 IMU OK · b11 temperatura OK · b12 falla de lastre (timeout) · b13 posición de lastre conocida |
| depth | i16 | mm (+ = hacia abajo) |
| depth_sp | i16 | mm |
| vz | i16 | mm/s (+ = bajando) |
| pressure | u16 | mbar |
| water_temp | i16 | centésimas de °C (DS18B20) |
| ms_temp | i16 | centésimas de °C (MS5837) |
| pitch / roll / pitch_sp | i16 | centésimas de grado (+ pitch = nariz arriba) |
| vbat | u16 | mV |
| ballast_pos | u8 | 0–100 % (0 = vacío), 255 = desconocida |
| ballast_dir | i8 | −1 vaciando, 0 quieto, +1 llenando |
| motor[5] | i8 ×5 | −100…100 % (M1…M5) |
| uptime | u32 | ms |
| rx_err | u16 | tramas descartadas |

---

## 3. Firmware

### Mezclador
- M1 izq-delantero, M2 izq-trasero, M3 der-delantero, M4 der-trasero (ajustable en `config.h`).
- Izquierda = surge + yaw; derecha = surge − yaw; normalizado a ±1; × power; × 0.4 si batería crítica.
- `MOTOR_DIR[]` invierte el sentido de cada motor (cableado/hélice); se verifica con el bring-up.
- Rampa de salida (RAMP_PER_S) para limitar picos de corriente.

### Cabeceo (M5)
- Con hold y armado: `u = Kp·e + Ki·∫e − Kd·ω_pitch`, `e = sp − pitch`, saturado a PITCH_MAX_OUT, anti-windup.
- Sin hold: `u = pitch` del mando.
- Desarmado: M5 = 0 (prueba A-2).

### Lastre (M6)
- Convención: LLENAR (+1) toma agua y hunde; VACIAR (−1) expulsa y hace subir.
- Switch LLENO (GPIO34) detiene el llenado; switch VACÍO (GPIO35) detiene el vaciado. ALTO = fin de carrera o cable roto. Los dos en ALTO = falla, todo quieto.
- Posición estimada por tiempo de marcha (BALLAST_STROKE_S), referenciada al tocar un switch.
- Tope de marcha continua (BALLAST_TIMEOUT_S) → falla hasta soltar el mando.
- El lastre funciona armado o desarmado (no es propulsión).

**Hold de profundidad — control en cascada** (`ctl::DepthHold`)

```
1) profundidad → velocidad:   e = depth − sp;  vref = −DEPTH_KP·(e fuera de ±DEPTH_BAND), limitada a ±DEPTH_VMAX
2) velocidad → lastre (PI):    neutro += −DEPTH_KI·(vz − vref)·dt      (aprende solo la posición neutra)
                               posRef = neutro − DEPTH_KV·filtro(vz − vref)
3) servo del pistón:           arranca si |posRef − pos| > 2 % de carrera, para al cruzar; ≥ 0.3 s entre arranques
```

Por qué cambió respecto al "pulso y espera" de la versión 1: en simulación (`firmware/test_host/robust.cpp`), el pulso y espera entraba en ciclos límite de hasta ±0.7 m según la autoridad del lastre y el ruido del sensor. El lastre integra (cambia la flotabilidad a cierta tasa), así que el lazo de velocidad necesita saber cuánta flotabilidad ya está aplicada: la posición estimada del pistón. Con la cascada, las 36 combinaciones probadas (autoridad del lastre ×7.5, carrera 10–40 s, neutro desconocido, ruido ±2 cm) quedan dentro de ±9 cm, con ≤0.4 arranques del motor por segundo.

**Requisito:** la posición del pistón se estima por tiempo de marcha, referenciada al tocar un fin de carrera. Antes de usar el hold, llevar el lastre a un extremo al menos una vez por inmersión (la interfaz muestra "posición ?" hasta entonces).

El stick vertical mueve el setpoint a HOLD_RATE m/s. Al activar el hold, setpoint = profundidad actual y neutro = posición actual del pistón.

### Seguridad
- Arranca desarmado y bloqueado: tras encender, tras un reinicio de la ESP32 o tras un failsafe, el PC debe desarmar y volver a armar (START). Un reinicio bajo el agua no rearma solo.
- Desarmado = corte inmediato de los 5 thrusters (sin rampa).
- El "emerger por batería crítica" queda enganchado hasta conectar un pack cargado.
- Batería: filtro rápido (pantalla) y lento (alarmas). Aviso LOWBAT_WARN_V, crítica LOWBAT_CRIT_V.
- Watchdog de 3 s.
- Cero de profundidad automático al arrancar (promedio de 1 s) y bajo demanda.

### Parámetros ajustables desde el PC (se guardan en flash)

| id | Nombre | Defecto | Unidad |
|---|---|---|---|
| 0 | PITCH_KP | 0.03 | 1/° |
| 1 | PITCH_KI | 0.01 | 1/(°·s) |
| 2 | PITCH_KD | 0.005 | 1/(°/s) |
| 3 | PITCH_MAX_OUT | 0.6 | fracción |
| 4 | PITCH_SP_MAX | 30 | ° |
| 5 | DEPTH_BAND | 0.05 | m |
| 6 | DEPTH_VMAX | 0.10 | m/s |
| 7 | DEPTH_KP | 0.15 | 1/s |
| 8 | DEPTH_KV | 0.5 | carrera/(m/s) |
| 9 | DEPTH_KI | 0.15 | carrera/(m/s·s) |
| 10 | HOLD_RATE | 0.10 | m/s |
| 11 | BALLAST_STROKE | 20 | s (medir con el bring-up) |
| 12 | BALLAST_TIMEOUT | 40 | s |
| 13 | LOWBAT_WARN | 10.5 | V |
| 14 | LOWBAT_CRIT | 9.9 | V |
| 15 | SURFACE_LINKLOSS | 15 | s |
| 16 | SURFACE_LOWBAT | 10 | s |
| 17 | WATER_DENSITY | 997 | kg/m³ |
| 18 | MOTOR_MIN_DUTY | 0.0 | fracción |
| 19 | RAMP_PER_S | 3.0 | 1/s |
| 20 | VBAT_CAL | 1.0 | factor |
| 21 | IMU_PITCH_OFS | 0 | ° (tecla L) |
| 22 | IMU_ROLL_OFS | 0 | ° (tecla L) |

---

## 4. Aplicación del PC

### Ventana (1280×768)
- Izquierda, 960×720: video con contornos de detección (amarillo = candidato, rojo = confirmado), etiqueta de clase y mira central.
- Derecha, 320 px: ROI ampliado del objeto + estado de detección; telemetría (batería, profundidad/setpoint/presión, temperatura del agua, horizonte artificial, lastre, enlace).
- Barra inferior: reloj con milisegundos, REC, armado/modos, potencia, barras de motores.
- Se graba la ventana completa a 25 fps.

### Detección (#28)
| Detector | Método |
|---|---|
| Cubo naranja | Umbral HSV + morfología + contorno con relación de aspecto ~1 y solidez alta |
| Moneda | Círculos (Hough) sobre gris + brillo/contraste local + rango de radio |
| Genérico | Píxeles que se apartan del color dominante del fondo (azul/cian), área mínima |

- Un candidato debe persistir N cuadros.
- **A / barra espaciadora = confirmar.** Si no hay candidato, se registra una marca manual del piloto.
- La confirmación dispara: banner rojo con número, clase y hora (3 s, queda en el video), pitido, foto del cuadro y del ROI, y línea en `eventos.csv`.
- Umbrales ajustables en vivo (F3) y guardados en `config.json`.

### Mando

| Gamepad | Teclado | Función |
|---|---|---|
| Stick izq. Y / X | W-S / A-D | Avance / giro |
| Stick der. Y | R-F | Vertical (setpoint o lastre) |
| RT − LT | T-G | Cabeceo |
| A | Espacio | Confirmar detección |
| X | C | Marca de temperatura |
| B | H | Hold de profundidad |
| Y | P | Hold de cabeceo |
| START | Enter | Armar / desarmar |
| BACK (1 s) | Z | Cero de profundidad |
| LB / RB | 1–4 | Potencia −/+ |
| — | L | Nivelar IMU (desarmado) |
| — | F2 / F3 / F5 | Parámetros / ajuste de visión / grabación |

### Registro por sesión (`sesiones/AAAAMMDD_HHMMSS/`)
`telemetria.csv`, `eventos.csv`, `video.mp4`, `capturas/`.

### Modos de prueba
- `--sim`: ROV simulado + escena sintética (piloteo y ajuste sin hardware).
- `--video archivo.mp4`: reproduce un video grabado para calibrar la detección.

---

## 5. Mediciones pendientes (son parámetros, no cambian la arquitectura)
1. Lastre: tiempo de carrera completa en ambos sentidos (el bring-up la mide), volumen desplazado, mecanismo.
2. Sentido de cada motor y hélices CW/CCW (bring-up).
3. Orientación de la IMU y nivelación (bring-up + tecla L).
4. Video real bajo agua con cubo naranja y monedas, para calibrar la detección.
5. Dimensiones de los objetos (reglamento, #19).
6. Chip de la capturadora (`lsusb`, #27).
7. Confirmar con el comité si mover el lastre durante los 20 s de A-2 cuenta como propulsión.
