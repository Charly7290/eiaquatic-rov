"""ROV simulado. Se conecta a la app como si fuera el puerto serial (write/read/close).

Reproduce la lógica del firmware (armado, failsafe, mezclador, PID de cabeceo, hold de profundidad
en cascada, lastre con fines de carrera) sobre una física sencilla. Sirve para practicar el piloteo
y probar la interfaz sin hardware; los valores físicos son aproximados.
"""
from __future__ import annotations

import math
import random
import struct
import threading
import time

from . import protocol as P


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class _PitchPID:
    def __init__(self) -> None:
        self.integ = 0.0

    def update(self, sp, pitch, rate, dt, kp, ki, kd, max_out):
        e = sp - pitch
        u = kp * e + ki * self.integ - kd * rate
        if abs(u) < max_out or (u > 0) != (e > 0):
            self.integ += e * dt
        if ki > 1e-6:
            lim = max_out / ki
            self.integ = clamp(self.integ, -lim, lim)
        return clamp(kp * e + ki * self.integ - kd * rate, -max_out, max_out)


class _DepthHold:
    """Port de ctl::DepthHold (firmware/src/control.cpp)."""

    def __init__(self) -> None:
        self.setpoint = 0.0
        self.neutral = self.pos_ref = 0.5
        self.dv_f = 0.0
        self.moving = 0
        self.since_stop = 10.0

    def reset(self, sp, pos):
        self.setpoint, self.neutral, self.pos_ref = max(0.0, sp), pos, pos
        self.dv_f, self.moving, self.since_stop = 0.0, 0, 10.0

    def update(self, depth, vz, pos, stick, dt, band, vmax, kp, kv, ki, hold_rate):
        self.setpoint = max(0.0, self.setpoint - stick * hold_rate * dt)
        e = depth - self.setpoint
        eb = e - band if e > band else e + band if e < -band else 0.0
        vref = clamp(-kp * eb, -vmax, vmax)
        dv = vz - vref
        self.dv_f += clamp(dt / 1.0, 0, 1) * (dv - self.dv_f)
        self.neutral = clamp(self.neutral - ki * dv * dt, 0, 1)
        self.pos_ref = clamp(self.neutral - kv * self.dv_f, 0, 1)
        err = self.pos_ref - pos
        self.since_stop += dt
        if self.moving == 0:
            if self.since_stop >= 0.3:
                self.moving = 1 if err > 0.02 else -1 if err < -0.02 else 0
        elif self.moving * err <= 0:
            self.moving, self.since_stop = 0, 0.0
        return self.moving


class SimROV:
    POOL_DEPTH = 2.0
    TRUE_NEUTRAL = 0.55  # posición del lastre con flotabilidad neutra (desconocida para el control)
    STROKE_TRUE = 14.5

    def __init__(self) -> None:
        self.parser = P.Parser()
        self.out = bytearray()
        self.lock = threading.Lock()
        self.params = {p.id: p.default for p in P.PARAMS}
        self.t0 = time.monotonic()
        self.t_last = self.t0
        self.last_cmd_t = 0.0
        self.cmd = P.Command(power=0)
        self.prev_flags = 0
        self.armed = False
        self.arm_blocked = True  # como el firmware: tras arrancar hay que armar explícitamente
        self.link_lost = 0.0
        # estado físico
        self.x = self.y = 0.0
        self.heading = 0.0  # rad
        self.u = 0.0  # avance m/s
        self.r = 0.0  # giro rad/s
        self.depth = 0.05
        self.vz = 0.0
        self.pitch = 0.0  # °
        self.q = 0.0  # °/s
        self.roll = 0.0
        self.ballast = 0.3  # 0 vacío .. 1 lleno (real)
        self.ballast_est = 0.5
        self.ballast_known = False
        self.ballast_dir = 0
        self.ah_used = 0.0
        self.motors = [0.0] * 5
        self.pid = _PitchPID()
        self.hold = _DepthHold()
        self.depth_hold_on = False
        self.pitch_hold_on = False
        self.pitch_sp = 0.0
        self.depth_est = 0.0
        self.vz_est = 0.0
        self.p0 = 1013.25

    # ----------------------------------------------------------------- transporte
    def write(self, data: bytes) -> None:
        for mtype, payload in self.parser.feed(data):
            self._handle(mtype, payload)

    def read(self) -> bytes:
        with self.lock:
            if not self.out:
                time.sleep(0.001)
                return b""
            data = bytes(self.out)
            self.out.clear()
            return data

    def close(self) -> None:
        pass

    def _send(self, mtype: int, payload: bytes) -> None:
        with self.lock:
            self.out.extend(P.encode(mtype, payload))

    # ----------------------------------------------------------------- mensajes
    def _handle(self, mtype: int, payload: bytes) -> None:
        now = time.monotonic()
        if mtype == P.CMD and len(payload) == 12:
            self._step(now)
            self.cmd = P.Command.unpack(payload)
            self.last_cmd_t = now
            rising = self.cmd.flags & ~self.prev_flags
            self.prev_flags = self.cmd.flags
            arm_req = bool(self.cmd.flags & P.CF_ARM)
            if not arm_req:
                self.arm_blocked = False
            self.armed = arm_req and not self.arm_blocked
            if rising & P.CF_ZERO_DEPTH:
                self.p0 = self._pressure_mbar()
            if rising & P.CF_LEVEL_IMU and not self.armed:
                self.params[21], self.params[22] = self.pitch, self.roll
            if self.cmd.flags & P.CF_REPLY:
                self._send(P.TELEM, self._telemetry().pack())
        elif mtype == P.PARAM_SET and len(payload) == 6:
            pid, _save, value = struct.unpack(P.PARAM_SET_FMT, payload)
            if pid < len(P.PARAMS):
                info = P.PARAMS[pid]
                self.params[pid] = clamp(value, info.min, info.max)
                self._send(P.PARAM_VALUE, P.pack_param_value(pid, len(P.PARAMS), self.params[pid]))
            else:
                self._send(P.PARAM_VALUE, P.pack_param_value(0xFF, len(P.PARAMS), 0.0))
        elif mtype == P.PARAM_GET and len(payload) == 1:
            pid = payload[0]
            ok = pid < len(P.PARAMS)
            self._send(P.PARAM_VALUE, P.pack_param_value(pid if ok else 0xFF, len(P.PARAMS),
                                                         self.params.get(pid, 0.0)))

    def _pressure_mbar(self) -> float:
        return 1013.25 + self.depth * 997 * 9.80665 / 100

    # ----------------------------------------------------------------- física + control
    def _step(self, now: float) -> None:
        dt_total = now - self.t_last
        self.t_last = now
        if dt_total <= 0:
            return
        # Sin comandos más de 1 s: failsafe (se evalúa al llegar el siguiente CMD).
        if self.last_cmd_t and now - self.last_cmd_t > 1.0:
            if self.armed:
                self.arm_blocked = True
            self.armed = False
            self.link_lost += now - self.last_cmd_t
        else:
            self.link_lost = 0.0
        steps = max(1, int(dt_total / 0.02))
        dt = min(dt_total / steps, 0.05)
        for _ in range(steps):
            self._control(dt)
            self._physics(dt)

    def _control(self, dt: float) -> None:
        p = self.params
        c = self.cmd
        target = [0.0] * 5
        if self.armed:
            left, right = c.surge + c.yaw, c.surge - c.yaw
            m = max(abs(left), abs(right), 1.0)
            pw = c.power / 100
            target[0] = target[1] = left / m * pw
            target[2] = target[3] = right / m * pw
            self.pitch_hold_on = bool(c.flags & P.CF_PITCH_HOLD)
            if self.pitch_hold_on:
                self.pitch_sp = c.pitch * p[4]
                target[4] = self.pid.update(self.pitch_sp, self.pitch - p[21], self.q, dt, p[0], p[1], p[2], p[3])
            else:
                self.pid.integ, self.pitch_sp = 0.0, 0.0
                target[4] = c.pitch
        else:
            self.pid.integ, self.pitch_sp, self.pitch_hold_on = 0.0, 0.0, False
        step = p[19] * dt
        for i in range(5):
            self.motors[i] = self.motors[i] + clamp(target[i] - self.motors[i], -step, step) if self.armed else 0.0

        # lastre
        hold_req = bool(c.flags & P.CF_DEPTH_HOLD) and self.link_lost == 0
        if hold_req and not self.depth_hold_on:
            self.hold.reset(self.depth_est, self.ballast_est)
        self.depth_hold_on = hold_req
        if self.link_lost >= p[15]:
            req = -1
        elif self.link_lost > 0:
            req = 0
        elif self.depth_hold_on:
            req = self.hold.update(self.depth_est, self.vz_est, self.ballast_est, c.vert, dt,
                                   p[5], p[6], p[7], p[8], p[9], p[10])
        else:
            req = -1 if c.vert > 0.3 else 1 if c.vert < -0.3 else 0
            self.hold.setpoint = self.depth_est
        full, empty = self.ballast >= 1.0, self.ballast <= 0.0
        if (req > 0 and full) or (req < 0 and empty):
            req = 0
        self.ballast_dir = req
        self.ballast = clamp(self.ballast + req * dt / self.STROKE_TRUE, 0, 1)
        self.ballast_est = clamp(self.ballast_est + req * dt / p[11], 0, 1)
        if full or empty:
            self.ballast_est, self.ballast_known = (1.0 if full else 0.0), True

    def _physics(self, dt: float) -> None:
        m = self.motors
        left, right = (m[0] + m[1]) / 2, (m[2] + m[3]) / 2
        self.u += (0.8 * (left + right) / 2 - 1.5 * self.u * abs(self.u) - 0.3 * self.u) * dt
        self.r += (1.2 * (left - right) - 2.0 * self.r) * dt
        self.heading += self.r * dt
        self.x += self.u * math.cos(self.heading) * dt
        self.y += self.u * math.sin(self.heading) * dt
        # cabeceo: M5 levanta la nariz; el lastre trasero lleno levanta la nariz; momento restaurador
        qdot = 60 * m[4] + 25 * (self.ballast - 0.5) - 1.2 * self.pitch - 2.5 * self.q
        self.q += qdot * dt
        self.pitch += self.q * dt
        self.roll += (-0.8 * self.roll + random.gauss(0, 0.5)) * dt
        # profundidad: lastre sobre el neutro hunde; M5 empuja un poco hacia arriba
        acc = 0.6 * (self.ballast - self.TRUE_NEUTRAL) - 0.05 * m[4] - 2.0 * self.vz * abs(self.vz) - 0.3 * self.vz
        if self.depth <= 0.02 and acc < 0:
            acc = 0.0  # flota en la superficie
            self.vz = min(self.vz, 0.0) * 0.5
        self.vz += acc * dt
        self.depth = clamp(self.depth + self.vz * dt, 0.0, self.POOL_DEPTH - 0.1)
        # estimación como en el firmware (sensor con ruido + EMA)
        meas = (self._pressure_mbar() - self.p0) * 100 / (997 * 9.80665) + random.gauss(0, 0.003)
        prev = self.depth_est
        self.depth_est += 0.3 * (meas - self.depth_est)
        self.vz_est += 0.2 * ((self.depth_est - prev) / dt - self.vz_est)
        current = 0.4 + sum(abs(v) for v in m) * 2.8 + (0.35 if self.ballast_dir else 0)
        self.ah_used += current * dt / 3600

    def _telemetry(self) -> P.Telemetry:
        p = self.params
        vbat = 12.5 - 2.4 * self.ah_used / 2.2 - 0.03 * sum(abs(v) for v in self.motors)
        flags = P.TF_DEPTH_OK | P.TF_IMU_OK | P.TF_TEMP_OK
        if self.armed:
            flags |= P.TF_ARMED
        if self.pitch_hold_on:
            flags |= P.TF_PITCH_HOLD
        if self.depth_hold_on:
            flags |= P.TF_DEPTH_HOLD
        if self.link_lost >= p[15]:
            flags |= P.TF_SURFACING
        if vbat < p[13]:
            flags |= P.TF_LOWBAT_WARN
        if vbat < p[14]:
            flags |= P.TF_LOWBAT_CRIT
        if self.ballast >= 1:
            flags |= P.TF_LIM_FULL
        if self.ballast <= 0:
            flags |= P.TF_LIM_EMPTY
        if self.ballast_known:
            flags |= P.TF_BALLAST_POS_KNOWN
        t = time.monotonic() - self.t0
        return P.Telemetry(
            seq=self.cmd.seq, flags=flags, depth=self.depth_est, depth_sp=self.hold.setpoint, vz=self.vz_est,
            pressure_mbar=int(round(self._pressure_mbar())),
            water_temp=24.6 + 0.2 * math.sin(t / 40) + random.gauss(0, 0.02),
            ms_temp=24.9, pitch=self.pitch - p[21], roll=self.roll - p[22], pitch_sp=self.pitch_sp,
            vbat=vbat, ballast_pos=self.ballast_est if self.ballast_known else None,
            ballast_dir=self.ballast_dir, motors=list(self.motors), uptime=t, rx_err=self.parser.errors,
        )

    # Para la escena sintética de video
    def pose(self) -> tuple[float, float, float, float, float]:
        return self.x, self.y, self.heading, self.depth, self.pitch
