# Protocolo de comunicación EIAquatic (RS-485)

Protocolo binario entre la **Estación de pilotaje** (PC) y el **firmware del ROV** (ESP32), sobre un enlace RS-485 half-duplex de un solo par trenzado a lo largo del tether (~35–40 m).

| | |
|---|---|
| Implementación en el ROV | [`firmware/src/protocol.h`](../firmware/src/protocol.h), [`protocol.cpp`](../firmware/src/protocol.cpp), [`link.cpp`](../firmware/src/link.cpp) |
| Implementación en el PC | [`pc/aquabot_pc/protocol.py`](../pc/aquabot_pc/protocol.py), [`link.py`](../pc/aquabot_pc/link.py) |
| Pruebas de compatibilidad | [`firmware/test_host/`](../firmware/test_host/) genera tramas con el código C++ y [`pc/tests/test_protocol.py`](../pc/tests/test_protocol.py) las decodifica con Python |

## 1. Capa física

| Parámetro | Valor |
|---|---|
| Medio | RS-485 half-duplex, un par trenzado del cable Cat5e del tether |
| Velocidad | **38 400 baud**, 8N1 (un byte = 10 bits ≈ 260 µs) |
| Transceptor del ROV | Waveshare RS485 Board 3.3 V (SP3485). RO → GPIO16, DI → GPIO17, dirección (RSE) → GPIO4 |
| Adaptador del PC | USB-RS485 |
| Terminación | 120 Ω entre A y B en **cada** extremo (A–B ≈ 60 Ω medido con todo apagado) |
| Polarización (bias) | 330 Ω de A a 3.3 V y de B a GND, solo en el extremo del ROV |

La velocidad debe ser la misma en `LINK_BAUD` ([`firmware/src/config.h`](../firmware/src/config.h)) y en `"baudios"` (`pc/config.json`).

## 2. Formato de trama

```
 AA 55 | TIPO | LEN | DATOS (LEN bytes) | CRC16 (LSB, MSB)
```

- **Sincronismo:** `0xAA 0x55`.
- **CRC16-CCITT-FALSE** (polinomio `0x1021`, valor inicial `0xFFFF`), calculado sobre TIPO, LEN y DATOS.
- **Little-endian** en todos los campos de más de un byte.
- **Datos:** máximo 64 bytes. Una trama dañada se descarta completa y nunca se ejecuta.
- **Pausa entre bytes:** si dentro de una trama pasan más de 20 ms entre bytes, el receptor reinicia el análisis.

| Tipo | Código | Sentido | Datos |
|---|---|---|---|
| `CMD` | `0x01` | PC → ROV | 12 bytes |
| `PARAM_SET` | `0x02` | PC → ROV | 6 bytes |
| `PARAM_GET` | `0x03` | PC → ROV | 1 byte |
| `TELEM` | `0x81` | ROV → PC | 36 bytes |
| `PARAM_VALUE` | `0x82` | ROV → PC | 6 bytes |

## 3. Mensajes

### CMD — comando de pilotaje (12 bytes de datos, 18 en total)

| Campo | Tipo | Descripción |
|---|---|---|
| `seq` | uint8 | Número de secuencia |
| `flags` | uint8 | Banderas (tabla siguiente) |
| `surge` | int16 | Avance, −1000…1000 (+ adelante) |
| `yaw` | int16 | Giro, −1000…1000 (+ derecha) |
| `pitch` | int16 | Cabeceo: setpoint con control automático, empuje directo de M5 sin él |
| `vert` | int16 | Vertical: lastre en manual, setpoint en profundidad automática (+ subir) |
| `power` | uint8 | Potencia, 0–100 % |
| `reserved` | uint8 | Reservado |

| Bandera | Bit | Significado |
|---|---|---|
| `CF_ARM` | 0 | Armar los propulsores |
| `CF_PITCH_HOLD` | 1 | Cabeceo automático (IMU) |
| `CF_DEPTH_HOLD` | 2 | Profundidad automática (lastre) |
| `CF_ZERO_DEPTH` | 3 | Cero de profundidad (en el flanco de subida) |
| `CF_LEVEL_IMU` | 4 | Nivelar la IMU (en el flanco de subida, solo desarmado) |
| `CF_REPLY` | 5 | El ROV debe responder con `TELEM` |

### TELEM — telemetría (36 bytes de datos, 42 en total)

| Campo | Tipo | Unidad |
|---|---|---|
| `seq` | uint8 | Copia del `seq` del comando |
| `flags` | uint16 | Estado: armado, modos, sensores OK, topes del lastre, alarmas |
| `depth`, `depth_sp`, `vz` | 3× int16 | mm, mm, mm/s |
| `pressure` | uint16 | mbar |
| `water_temp`, `ms_temp` | 2× int16 | centésimas de °C |
| `pitch`, `roll`, `pitch_sp` | 3× int16 | centésimas de grado |
| `vbat` | uint16 | mV |
| `ballast_pos` | uint8 | % (255 = desconocida) |
| `ballast_dir` | int8 | −1 vaciando, 0 quieto, +1 llenando |
| `motor[5]` | 5× int8 | −100…100 % (M1–M5) |
| `uptime` | uint32 | ms desde el arranque |
| `rx_err` | uint16 | Tramas recibidas con error en el ROV |

### PARAM_SET / PARAM_GET / PARAM_VALUE — parámetros

Sirven para leer y cambiar los **23 parámetros** del ROV en caliente: ganancias, lastre, calibraciones y umbrales, listados en [`firmware/src/params.cpp`](../firmware/src/params.cpp).
- `PARAM_SET = {id: uint8, save: uint8, value: float}`. Si `save = 1`, el valor se guarda en la flash del ESP32.
- `PARAM_GET = {id: uint8}`.
- `PARAM_VALUE = {id: uint8, count: uint8, value: float}`. Es la respuesta a los dos anteriores; `id = 0xFF` indica un parámetro inválido.

## 4. Temporización

El PC es el único que inicia la comunicación; el ROV solo responde. Así nunca transmiten los dos a la vez.

```
 PC   CMD  CMD  CMD  CMD+REPLY ─────────────►
 ROV                          ◄──── TELEM ────
      |←25 ms→|                    (~16 ms el intercambio completo)
```

| | Valor |
|---|---|
| Comandos | **40 por segundo** (cada 25 ms) |
| Telemetría | 1 de cada 4 comandos lleva `CF_REPLY`: **~10 Hz** |
| Espera máxima de respuesta en el PC | 45 ms |
| Parámetros | Como máximo un mensaje por ciclo, después de un intercambio con telemetría |

**Por qué hay redundancia:** el ruido de los motores daña tramas sueltas. Con 40 comandos por segundo, un comando dañado se repone 25 ms después, y el bus queda en silencio la mayor parte del tiempo.

## 5. Seguridad

- **Failsafe:** si el ROV pasa **1 s** sin un `CMD` válido, apaga los propulsores y se desarma. Para volver a armarlo, el PC debe enviar primero un comando sin `CF_ARM`, así que un reinicio o una reconexión nunca rearman el ROV solos.
- **Emersión:** después de **15 s** sin enlace (parámetro `SURFACE_LINKLOSS`), el ROV vacía el lastre para subir a la superficie.
- **Calidad del enlace en la estación:** se muestra el porcentaje de comandos que piden respuesta y no la reciben. En verde si es menor al 5 %.

## 6. Historial

| Versión | Cambio |
|---|---|
| Inicial | 115 200 baud; respuesta a cada comando, 20 Hz; failsafe de 0.5 s |
| 21 oct 2026 | Failsafe a 1 s |
| 22 oct 2026 (v1.0) | 38 400 baud; comandos redundantes a 40 Hz con `CF_REPLY` y telemetría a 10 Hz |
