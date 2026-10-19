# AquaBot USB 2026 — Software

Firmware de la ESP32 (ROV) y app de piloteo del PC. Diseño completo en `docs/diseno_software.md`.

```
aquabot_software/
├── setup.sh               instalación del entorno: bash setup.sh (una vez, y cada vez que cambien dependencias)
├── Makefile               comandos del día a día (make help)
├── aquabot.code-workspace abrir en VS Code (PlatformIO + Python ya configurados)
├── requirements-dev.txt   PlatformIO y pytest
├── .venv/                 entorno Python único (lo crea setup.sh, no va a git)
├── docs/diseno_software.md
├── firmware/              PlatformIO (ESP32, core Arduino 2.0.x)
│   ├── platformio.ini       entornos: rov, rov_usb, rov_v4, bringup
│   ├── src/config.h         pines, sentidos de motores, orientación de la IMU  <- lo que más se edita
│   ├── src/main_rov.cpp     programa de competencia
│   ├── src/main_bringup.cpp programa de pruebas de integración
│   └── test_host/           pruebas del firmware en el PC
└── pc/                    app de piloteo (Python)
    ├── piloto.py            punto de entrada
    ├── aquabot_pc/          código de la app
    ├── config.example.json  configuración de referencia (en git)
    ├── config.json          configuración de ESTE portátil (local, fuera de git)
    ├── requirements.txt
    ├── joytest.py           muestra ejes/botones de un mando
    ├── tests/
    └── sesiones/            registros de cada sesión (fuera de git)
```

---

## 1. Instalación (Ubuntu)

```bash
cd ~/aquabot_software
bash setup.sh
```

El script instala los paquetes del sistema (`python3-venv`, `v4l-utils`, `build-essential`, `git`), agrega su usuario al grupo `dialout` (puertos seriales), crea `.venv` con todas las dependencias (incluido PlatformIO), crea `pc/config.json`, descarga la plataforma ESP32 y corre las pruebas. Se puede repetir sin problema.

- Si lo agregó a `dialout`, **cierre sesión y vuelva a entrar** una vez.
- Opciones: `--no-apt` (sin sudo), `--no-fw` (sin descargar la plataforma ESP32).
- Si movió la carpeta desde otra ubicación, el entorno viejo `pc/.venv` ya no sirve: `rm -rf pc/.venv`.
- Usa el Python del sistema (`/usr/bin/python3`), no el de conda, y los comandos de `make` ignoran el `PYTHONPATH` de ROS2: puede tener ROS o conda cargados en la terminal sin que interfieran.

**VS Code:** abra `aquabot.code-workspace` (File → Open Workspace from File) e instale las extensiones recomendadas (PlatformIO IDE y Python). Python usa `.venv` y PlatformIO encuentra el firmware solos.

**Git** (recomendado, para poder volver atrás si algo se daña en la piscina):

```bash
git init && git add -A && git commit -m "AquaBot: estado inicial"
```

`config.json`, `sesiones/`, `.venv` y los compilados ya están en `.gitignore`.

## 2. Comandos (`make help`)

| Comando | Para qué |
|---|---|
| `make sim` | App con el ROV y el video simulados (practicar sin hardware) |
| `make piloto` | App con el ROV real (puerto y cámara de `pc/config.json`) |
| `make dispositivos` | Ver puertos seriales y cámaras conectados |
| `make joytest` | Ver ejes y botones del mando |
| `make bringup` | Cargar el programa de pruebas a la ESP32 y abrir el monitor |
| `make rov-usb` | Cargar el firmware de banco (enlace por el USB de la ESP32) |
| `make rov` | Cargar el firmware de **competencia** (enlace RS-485) |
| `make monitor` | Monitor serial de la ESP32 |
| `make fw-build` | Solo compilar todo el firmware |
| `make test` | Todas las pruebas |

Variables opcionales: `make piloto PORT=/dev/ttyACM0 VIDEO=4`, o `ARGS='...'` para cualquier opción de `piloto.py` (por ejemplo `make sim ARGS="--video pc/sesiones/.../camara_01.mp4"`).

Para usar `python` o `pio` directamente en una terminal: `source .venv/bin/activate`.

### Configuración del portátil (`pc/config.json`)

- **Cámara:** `"video": {"dispositivo": 4}` (la capturadora MS210x es `/dev/video4`; la webcam integrada es la 0). Si cambia de puerto USB, verifique con `make dispositivos`.
- **Puerto del ROV:** use la ruta de `/dev/serial/by-id/...` que muestra `make dispositivos` para el adaptador USB-RS485. No cambia al reconectar, a diferencia de `/dev/ttyUSB0`/`ttyACM0`, que dependen del orden en que se conecten la ESP32 y el adaptador.
- **PAL/NTSC:** `"alto": 576` (PAL) o `480` (NTSC), igual que en el OSD de la Caddx.

### Firmware

`make bringup`, `make rov-usb`, `make rov` compilan y cargan por el USB de la ESP32 (PlatformIO detecta el puerto). Desde VS Code se puede usar también la barra de PlatformIO eligiendo el entorno (`env:bringup`, `env:rov`…).

Verificado: compila sin advertencias con el core Arduino-ESP32 2.0.17 (24 % de flash, 6 % de RAM).

**No conectar el USB y el 5 V del XL4015 a la vez** (retroalimentación): quite el jumper XL4015 → VIN para programar por USB.

## 3. Bring-up (integración, en este orden)

Con `make bringup` (carga el programa y abre el monitor a 115200). **Hélices libres o ROV sujeto.** Si la carga se queda en "Connecting...", mantenga presionado BOOT en la ESP32.

1. **`i` — escaneo I2C.** Deben aparecer 0x68 (MPU6050) y 0x76 (MS5837). El arranque también informa si el CRC de la PROM del MS5837 es correcto.
2. **`s` — sensores en vivo.**
   - Presión ~1013 mbar en el aire; sumerja el sensor en un vaso y debe subir.
   - Temperatura del DS18B20 coherente.
   - Microswitches: LLENO=0 y VACÍO=0 en reposo; =1 al presionarlos. Si dan 1 en reposo, revise COM/NC.
   - Voltaje de batería: compárelo con el multímetro. El factor se corrige luego con VBAT_CAL desde la app.
3. **Orientación de la IMU** (con `s` activo):
   - ROV nivelado → acc(cuerpo) ≈ 0, 0, +1.
   - Nariz arriba → pitch positivo.
   - Lado derecho abajo → roll positivo.
   - Si no coincide, edite `IMU_MAP_*` en `config.h`.
4. **`1`…`6` — cada motor.** Con duty POSITIVO:
   - M1–M4 deben empujar hacia **adelante**.
   - M5 debe **levantar la nariz**.
   - M6 debe **llenar** el lastre.
   - Si alguno va al revés, cambie su signo en `MOTOR_DIR` (`config.h`).
   - Si un thruster no arranca con poco duty, anote el mínimo para MOTOR_MIN_DUTY.
5. **`f` / `e` — lastre a pasos.** Verifique que el switch LLENO detiene el llenado y el VACÍO el vaciado. Si están cruzados, intercambie `PIN_LIM_FULL` y `PIN_LIM_EMPTY`.
6. **`b` — caracterización del lastre** a 8.4 V.
   - Hace 3 carreras completas en cada sentido y sugiere BALLAST_STROKE y BALLAST_TIMEOUT.
   - Ajústelos luego desde la app (F2) y guárdelos (Ctrl+S).

Después: `make rov-usb` y `make piloto PORT=<puerto de la ESP32>` para probar la app con el ROV real por USB. Luego `make rov` y `make piloto` (adaptador RS-485) para probar el enlace de competencia.

## 4. App de piloteo

```bash
make piloto                                   # ROV real (puerto y cámara de pc/config.json)
make sim                                      # ROV simulado
make sim ARGS="--video ruta/camara_01.mp4"    # calibrar la detección con un video grabado
```

| Gamepad | Teclado | Función |
|---|---|---|
| Stick izq. Y / X | W-S / A-D | Avance / giro |
| Stick der. Y | R-F | Vertical: lastre (manual) o setpoint (hold) |
| RT − LT | T-G | Cabeceo (ángulo con hold, empuje directo sin hold) |
| **A** | **Espacio** | **Confirmar detección** |
| X | C | Marca de temperatura |
| B | H | Hold de profundidad on/off |
| Y | P | Hold de cabeceo on/off |
| START | Enter | Armar / desarmar |
| BACK (1 s) | Z | Cero de profundidad (en superficie) |
| LB / RB | 1–4 | Potencia −/+ (25–100 %) |
| — | L | Nivelar IMU (desarmado, ROV quieto y horizontal) |
| — | F2 | Parámetros del ROV (↑↓ ←→, Shift ×10, Ctrl+S guarda en flash) |
| — | F3 | Ajuste de visión (Tab cambia máscara, Ctrl+S guarda en config.json) |
| — | F5 | Grabar / detener |
| — | F11 | Pantalla completa / ventana |
| — | ESC ×2 | Salir (desarma) |

La ventana mide 1600×960 (es lo que se graba). Si la pantalla es más pequeña se escala sola; F11 la pone en pantalla completa. La tipografía (Barlow, licencia OFL) va incluida en `pc/aquabot_pc/assets/fonts/`.

**Marca de detección (#28).** Al pulsar A, la marca queda registrada de tres formas a la vez:
- un banner rojo con número, clase y hora en milisegundos, por 3 s;
- el contorno en rojo y el ROI congelado;
- un pitido, una foto del cuadro y del ROI, y una línea en `eventos.csv`.

Todo queda en el video, porque se graba la ventana completa con el reloj en milisegundos. Si no hay candidato, se registra una "MARCA MANUAL" con un recorte central.

**Archivos de cada sesión** (`pc/sesiones/AAAAMMDD_HHMMSS/`):
- `telemetria.csv` (20 Hz, con el comando enviado);
- `eventos.csv`;
- `video_NN.mp4` (la ventana completa, la evidencia);
- `camara_NN.mp4` (la cámara sin interfaz, para calibrar la detección);
- `capturas/`.

## 5. Primera sesión en el agua (orden sugerido)

1. **Antes de sellar la cápsula:** configure PAL/NTSC en el OSD de la Caddx, y en `config.json` (`video.alto`: 576 PAL / 480 NTSC).
2. **En superficie, desarmado:**
   - Z (cero de profundidad).
   - L (nivelar IMU) con el ROV horizontal.
3. **Trimado:**
   - Lleve el lastre a un extremo para referenciar la posición.
   - Busque el punto neutro con el stick vertical (manual).
   - Anote en qué % queda.
4. **Cabeceo:**
   - Arme con potencia baja y active el hold de cabeceo.
   - Si oscila, baje PITCH_KP o suba PITCH_KD.
   - Si queda inclinado de forma sostenida, suba PITCH_KI.
5. **Hold de profundidad:**
   - Actívelo a ~1 m y mueva el setpoint con el stick.
   - Si sobrepasa mucho, baje DEPTH_KP o DEPTH_VMAX.
   - Si reacciona lento, suba DEPTH_KV.
6. **Guardar:** Ctrl+S en F2 para guardar en la flash del ROV.
7. **Video para la detección:**
   - Grabe pasadas sobre el cubo y las monedas.
   - Luego, en tierra: `make sim ARGS="--video pc/sesiones/…/camara_01.mp4"` y F3 para ajustar umbrales.
   - `camara_NN.mp4` es la cámara sin la interfaz; `video_NN.mp4` es la ventana completa (la evidencia).

**Prueba A-2 (flotabilidad neutra):** desarme (corte inmediato de los 5 thrusters) y deje el lastre en el punto neutro. El hold de profundidad usa solo el lastre, no propulsión; confirme con el comité si se permite mover el lastre durante esos 20 s.

## 6. Pruebas

```bash
make test      # firmware en el PC (protocolo, control) + app (protocolo C++ <-> Python, detección)
make robust    # barrido de robustez del hold de profundidad
```

`make test-fw` genera tramas reales del firmware que `make test-pc` decodifica, para verificar que C++ y Python hablan exactamente el mismo protocolo.
