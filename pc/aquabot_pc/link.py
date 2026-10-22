"""Enlace con el ROV: el PC es maestro. Cada 50 ms envía un CMD y espera la TELEM.

Los mensajes de parámetros se intercalan en lugar de un CMD cuando hay alguno en cola.
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional

from . import protocol as P

log = logging.getLogger(__name__)

RESPONSE_TIMEOUT_S = 0.045  # el ciclo es de 50 ms: margen para respuestas algo tardías
LINK_OK_S = 1.0  # igual al failsafe del ROV (FAILSAFE_MS): antes de eso, un hueco no desarma


class SerialTransport:
    """Puerto serial con reconexión automática (adaptador USB-RS485 o USB de la ESP32)."""

    def __init__(self, port: str, baud: int = 115200) -> None:
        self.port, self.baud = port, baud
        self.ser = None
        self._last_try = 0.0
        self.error: str | None = None

    def _ensure(self) -> bool:
        if self.ser is not None:
            return True
        if time.monotonic() - self._last_try < 1.0:
            return False
        self._last_try = time.monotonic()
        try:
            import serial  # pyserial

            self.ser = serial.Serial(self.port, self.baud, timeout=0.003, write_timeout=0.1)
            self.ser.reset_input_buffer()
            self.error = None
            log.info("Puerto %s abierto", self.port)
            return True
        except Exception as e:  # noqa: BLE001
            self.error = str(e)
            self.ser = None
            return False

    def _fail(self, e: Exception) -> None:
        self.error = str(e)
        log.warning("Puerto %s: %s", self.port, e)
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        self.ser = None

    def write(self, data: bytes) -> None:
        if not self._ensure():
            return
        try:
            self.ser.write(data)
        except Exception as e:  # noqa: BLE001
            self._fail(e)

    def read(self) -> bytes:
        if not self._ensure():
            time.sleep(0.003)
            return b""
        try:
            n = self.ser.in_waiting
            return self.ser.read(n if n else 1)
        except Exception as e:  # noqa: BLE001
            self._fail(e)
            return b""

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None


@dataclass
class LinkStats:
    tx: int = 0
    rx: int = 0
    timeouts: int = 0
    rtt_ms: float = 0.0
    rate_hz: float = 0.0


class Link(threading.Thread):
    def __init__(self, transport, rate_hz: float = 20.0,
                 on_telemetry: Optional[Callable[[P.Telemetry, P.Command], None]] = None) -> None:
        super().__init__(daemon=True, name="enlace")
        self.transport = transport
        self.period = 1.0 / rate_hz
        self.on_telemetry = on_telemetry
        self.parser = P.Parser()
        self.stats = LinkStats()
        self._lock = threading.Lock()
        self._cmd = P.Command()
        self._pulses: dict[int, float] = {}  # bandera -> instante hasta el que se mantiene en 1
        self._param_q: "queue.Queue[tuple]" = queue.Queue()
        self._stop = threading.Event()
        self.telemetry: P.Telemetry | None = None
        self.last_telemetry_t = 0.0
        self.params: dict[int, float] = {}  # valores confirmados por el ROV
        self.param_count: int | None = None
        self._was_ok = False

    # ------------------------------------------------------------- API para la interfaz
    def set_command(self, **kw) -> None:
        with self._lock:
            self._cmd = replace(self._cmd, **kw)

    def pulse_flag(self, flag: int, seconds: float = 0.3) -> None:
        """Mantiene una bandera en 1 durante un rato (para comandos por flanco)."""
        with self._lock:
            self._pulses[flag] = time.monotonic() + seconds

    def set_param(self, pid: int, value: float, save: bool = False) -> None:
        self._param_q.put(("set", pid, value, save))

    def request_all_params(self) -> None:
        for p in P.PARAMS:
            self._param_q.put(("get", p.id))

    @property
    def ok(self) -> bool:
        return time.monotonic() - self.last_telemetry_t < LINK_OK_S

    def stop(self) -> None:
        self._stop.set()

    # ------------------------------------------------------------- hilo
    def run(self) -> None:
        seq = 0
        t_rate = time.monotonic()
        n_rate = 0
        next_t = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.005, next_t - now))
                continue
            next_t += self.period
            if now - next_t > 0.5:  # se atrasó mucho (p. ej. suspensión): no ráfagas
                next_t = now + self.period

            with self._lock:
                flags = self._cmd.flags
                for f, until in list(self._pulses.items()):
                    if now < until:
                        flags |= f
                    else:
                        del self._pulses[f]
                cmd = replace(self._cmd, seq=seq, flags=flags)
            seq = (seq + 1) & 0xFF
            got = self._exchange(P.encode(P.CMD, cmd.pack()), expect=P.TELEM, cmd=cmd)
            if got:
                n_rate += 1
                # Como mucho un mensaje de parámetros por ciclo, después del CMD: así el CMD
                # nunca deja de salir a 20 Hz y el failsafe del ROV (1 s) no se dispara.
                if not self._param_q.empty():
                    item = self._param_q.get_nowait()
                    if item[0] == "set":
                        frame = P.encode(P.PARAM_SET, P.pack_param_set(item[1], item[2], item[3]))
                    else:
                        frame = P.encode(P.PARAM_GET, P.pack_param_get(item[1]))
                    self._exchange(frame, expect=P.PARAM_VALUE, cmd=None)
            if now - t_rate >= 1.0:
                self.stats.rate_hz = n_rate / (now - t_rate)
                n_rate, t_rate = 0, now

            ok = self.ok
            if ok and not self._was_ok:
                self.request_all_params()  # al (re)conectar, leer los parámetros del ROV
            self._was_ok = ok
        self.transport.close()

    def _exchange(self, frame: bytes, expect: int, cmd: P.Command | None) -> bool:
        t0 = time.monotonic()
        self.transport.write(frame)
        self.stats.tx += 1
        while time.monotonic() - t0 < RESPONSE_TIMEOUT_S:
            data = self.transport.read()
            if not data:
                continue
            for mtype, payload in self.parser.feed(data):
                if mtype == P.TELEM and len(payload) == 36:
                    tel = P.Telemetry.unpack(payload)
                    self.telemetry = tel
                    self.last_telemetry_t = time.monotonic()
                    self.stats.rx += 1
                    self.stats.rtt_ms = (self.last_telemetry_t - t0) * 1000
                    if self.on_telemetry and cmd is not None:
                        try:
                            self.on_telemetry(tel, cmd)
                        except Exception:  # noqa: BLE001
                            log.exception("on_telemetry")
                    if expect == P.TELEM:
                        return True
                elif mtype == P.PARAM_VALUE and len(payload) == 6:
                    pid, count, value = P.unpack_param_value(payload)
                    self.param_count = count
                    if pid != 0xFF:
                        self.params[pid] = value
                    if expect == P.PARAM_VALUE:
                        return True
                # Cualquier otra trama (p. ej. eco del propio adaptador) se ignora.
        self.stats.timeouts += 1
        return False
