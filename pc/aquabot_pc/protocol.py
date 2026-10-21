"""Protocolo serial PC <-> ROV. Espejo de firmware/src/protocol.h y params.cpp.

Trama: AA 55 | TIPO | LEN | PAYLOAD | CRC16 (LSB, MSB)
CRC16-CCITT-FALSE (0x1021, init 0xFFFF) sobre TIPO, LEN y PAYLOAD. Little-endian.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

SYNC1, SYNC2 = 0xAA, 0x55
MAX_PAYLOAD = 64

CMD, PARAM_SET, PARAM_GET = 0x01, 0x02, 0x03
TELEM, PARAM_VALUE = 0x81, 0x82

CMD_FMT = "<BBhhhhBB"          # 12 bytes
TELEM_FMT = "<BHhhhHhhhhhHBb5bIH"  # 36 bytes
PARAM_SET_FMT = "<BBf"
PARAM_GET_FMT = "<B"
PARAM_VALUE_FMT = "<BBf"

assert struct.calcsize(CMD_FMT) == 12
assert struct.calcsize(TELEM_FMT) == 36

# Banderas del comando
CF_ARM = 1 << 0
CF_PITCH_HOLD = 1 << 1
CF_DEPTH_HOLD = 1 << 2
CF_ZERO_DEPTH = 1 << 3
CF_LEVEL_IMU = 1 << 4

# Banderas de telemetría
TF_ARMED = 1 << 0
TF_PITCH_HOLD = 1 << 1
TF_DEPTH_HOLD = 1 << 2
TF_LINK_FAILSAFE = 1 << 3
TF_SURFACING = 1 << 4
TF_LOWBAT_WARN = 1 << 5
TF_LOWBAT_CRIT = 1 << 6
TF_LIM_FULL = 1 << 7
TF_LIM_EMPTY = 1 << 8
TF_DEPTH_OK = 1 << 9
TF_IMU_OK = 1 << 10
TF_TEMP_OK = 1 << 11
TF_BALLAST_FAULT = 1 << 12
TF_BALLAST_POS_KNOWN = 1 << 13


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload demasiado largo")
    body = bytes([msg_type, len(payload)]) + payload
    c = crc16(body)
    return bytes([SYNC1, SYNC2]) + body + bytes([c & 0xFF, c >> 8])


class Parser:
    """Acumula bytes y devuelve las tramas válidas como (tipo, payload)."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buf.extend(data)
        out: list[tuple[int, bytes]] = []
        while True:
            i = self.buf.find(bytes([SYNC1, SYNC2]))
            if i < 0:
                # conserva un 0xAA final por si es el inicio de la siguiente trama
                del self.buf[: max(0, len(self.buf) - 1)]
                return out
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 4:
                return out
            length = self.buf[3]
            if length > MAX_PAYLOAD:
                self.errors += 1
                del self.buf[:2]
                continue
            total = length + 6
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2 : 4 + length])
            rx_crc = self.buf[4 + length] | (self.buf[5 + length] << 8)
            if crc16(body) == rx_crc:
                out.append((body[0], body[2:]))
                del self.buf[:total]
            else:
                self.errors += 1
                del self.buf[:2]


@dataclass
class Command:
    seq: int = 0
    flags: int = 0
    surge: float = 0.0  # -1..1
    yaw: float = 0.0
    pitch: float = 0.0
    vert: float = 0.0
    power: int = 50  # %

    def pack(self) -> bytes:
        def q(v: float) -> int:
            return int(round(max(-1.0, min(1.0, v)) * 1000))

        return struct.pack(CMD_FMT, self.seq & 0xFF, self.flags & 0xFF, q(self.surge), q(self.yaw),
                           q(self.pitch), q(self.vert), max(0, min(100, int(self.power))), 0)

    @classmethod
    def unpack(cls, payload: bytes) -> "Command":
        seq, flags, s, y, p, v, power, _ = struct.unpack(CMD_FMT, payload)
        return cls(seq, flags, s / 1000, y / 1000, p / 1000, v / 1000, power)


@dataclass
class Telemetry:
    seq: int
    flags: int
    depth: float  # m, + abajo
    depth_sp: float
    vz: float  # m/s, + bajando
    pressure_mbar: int
    water_temp: float | None  # °C (DS18B20), None si no hay lectura
    ms_temp: float  # °C (MS5837)
    pitch: float  # °, + nariz arriba
    roll: float  # °, + lado derecho abajo
    pitch_sp: float
    vbat: float  # V
    ballast_pos: float | None  # 0 vacío .. 1 lleno, None si desconocida
    ballast_dir: int
    motors: list[float] = field(default_factory=list)  # -1..1, M1..M5
    uptime: float = 0.0  # s
    rx_err: int = 0

    @classmethod
    def unpack(cls, payload: bytes) -> "Telemetry":
        v = struct.unpack(TELEM_FMT, payload)
        (seq, flags, depth, depth_sp, vz, pressure, wtemp, mstemp, pitch, roll, pitch_sp, vbat, bpos,
         bdir) = v[:14]
        motors = [m / 100 for m in v[14:19]]
        uptime, rx_err = v[19], v[20]
        return cls(
            seq=seq, flags=flags, depth=depth / 1000, depth_sp=depth_sp / 1000, vz=vz / 1000,
            pressure_mbar=pressure, water_temp=(wtemp / 100) if flags & TF_TEMP_OK else None,
            ms_temp=mstemp / 100, pitch=pitch / 100, roll=roll / 100, pitch_sp=pitch_sp / 100,
            vbat=vbat / 1000, ballast_pos=None if bpos == 255 else bpos / 100, ballast_dir=bdir,
            motors=motors, uptime=uptime / 1000, rx_err=rx_err,
        )

    def pack(self) -> bytes:  # usado por el simulador
        motors = [int(round(max(-1, min(1, m)) * 100)) for m in (self.motors + [0] * 5)[:5]]

        def i16(x: float) -> int:
            return max(-32768, min(32767, int(round(x))))

        return struct.pack(
            TELEM_FMT, self.seq & 0xFF, self.flags & 0xFFFF, i16(self.depth * 1000), i16(self.depth_sp * 1000),
            i16(self.vz * 1000), max(0, min(65535, int(self.pressure_mbar))),
            i16((self.water_temp or 0) * 100), i16(self.ms_temp * 100), i16(self.pitch * 100),
            i16(self.roll * 100), i16(self.pitch_sp * 100), max(0, min(65535, int(self.vbat * 1000))),
            255 if self.ballast_pos is None else int(round(self.ballast_pos * 100)), self.ballast_dir,
            *motors, int(self.uptime * 1000) & 0xFFFFFFFF, min(65535, self.rx_err),
        )

    def has(self, flag: int) -> bool:
        return bool(self.flags & flag)


@dataclass(frozen=True)
class ParamInfo:
    id: int
    name: str
    default: float
    min: float
    max: float
    step: float
    unit: str
    desc: str


# Mismo orden que firmware/src/params.h
PARAMS: list[ParamInfo] = [
    ParamInfo(0, "PITCH_KP", 0.03, 0.0, 1.0, 0.005, "1/°", "Cabeceo: proporcional"),
    ParamInfo(1, "PITCH_KI", 0.01, 0.0, 1.0, 0.002, "1/(°·s)", "Cabeceo: integral (compensa el lastre)"),
    ParamInfo(2, "PITCH_KD", 0.005, 0.0, 0.5, 0.001, "1/(°/s)", "Cabeceo: derivativo (giroscopio)"),
    ParamInfo(3, "PITCH_MAX_OUT", 0.6, 0.0, 1.0, 0.05, "", "Cabeceo: empuje máximo de M5"),
    ParamInfo(4, "PITCH_SP_MAX", 30.0, 0.0, 60.0, 1.0, "°", "Cabeceo: ángulo con gatillo a fondo"),
    ParamInfo(5, "DEPTH_BAND", 0.05, 0.0, 1.0, 0.01, "m", "Profundidad: banda sin corrección"),
    ParamInfo(6, "DEPTH_VMAX", 0.10, 0.01, 1.0, 0.01, "m/s", "Profundidad: velocidad vertical máx."),
    ParamInfo(7, "DEPTH_KP", 0.15, 0.0, 2.0, 0.01, "1/s", "Profundidad: error -> velocidad"),
    ParamInfo(8, "DEPTH_KV", 0.5, 0.0, 5.0, 0.05, "carr/(m/s)", "Profundidad: velocidad -> lastre"),
    ParamInfo(9, "DEPTH_KI", 0.15, 0.0, 2.0, 0.01, "carr/(m/s·s)", "Profundidad: aprende el neutro"),
    ParamInfo(10, "HOLD_RATE", 0.10, 0.0, 1.0, 0.01, "m/s", "Velocidad del setpoint con el stick"),
    ParamInfo(11, "BALLAST_STROKE", 14.5, 1.0, 120.0, 0.5, "s", "Lastre: tiempo de carrera completa"),
    ParamInfo(12, "BALLAST_TIMEOUT", 27.0, 2.0, 240.0, 1.0, "s", "Lastre: marcha continua máxima"),
    ParamInfo(13, "LOWBAT_WARN", 10.5, 6.0, 13.0, 0.1, "V", "Batería: aviso"),
    ParamInfo(14, "LOWBAT_CRIT", 9.9, 6.0, 13.0, 0.1, "V", "Batería: crítica (empuje al 40 %)"),
    ParamInfo(15, "SURFACE_LINKLOSS", 15.0, 1.0, 300.0, 1.0, "s", "Emerger tras perder el enlace"),
    ParamInfo(16, "SURFACE_LOWBAT", 10.0, 1.0, 300.0, 1.0, "s", "Emerger tras batería crítica"),
    ParamInfo(17, "WATER_DENSITY", 997.0, 990.0, 1030.0, 1.0, "kg/m³", "Densidad del agua"),
    ParamInfo(18, "MOTOR_MIN_DUTY", 0.08, 0.0, 0.5, 0.01, "", "Duty mínimo (zona muerta)"),
    ParamInfo(19, "RAMP_PER_S", 3.0, 0.2, 50.0, 0.1, "1/s", "Rampa de los thrusters"),
    ParamInfo(20, "VBAT_CAL", 0.99, 0.8, 1.2, 0.005, "", "Calibración del voltaje"),
    ParamInfo(21, "IMU_PITCH_OFS", -5.9, -45.0, 45.0, 0.1, "°", "Nivelación IMU: cabeceo"),
    ParamInfo(22, "IMU_ROLL_OFS", -1.0, -45.0, 45.0, 0.1, "°", "Nivelación IMU: alabeo"),
]
PARAM_BY_NAME = {p.name: p for p in PARAMS}


def pack_param_set(pid: int, value: float, save: bool) -> bytes:
    return struct.pack(PARAM_SET_FMT, pid, 1 if save else 0, value)


def pack_param_get(pid: int) -> bytes:
    return struct.pack(PARAM_GET_FMT, pid)


def unpack_param_value(payload: bytes) -> tuple[int, int, float]:
    return struct.unpack(PARAM_VALUE_FMT, payload)


def pack_param_value(pid: int, count: int, value: float) -> bytes:
    return struct.pack(PARAM_VALUE_FMT, pid, count, value)
