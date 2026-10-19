"""Mando (gamepad) y teclado -> ejes de pilotaje + acciones.

Usa la API GameController de SDL (pygame._sdl2.controller), que entrega el mismo mapeo para mandos
Xbox y PlayStation en Linux y Windows. Si el mando no es reconocido, cae a pygame.joystick con los
índices de config.json (use joytest.py para verlos).
"""
from __future__ import annotations

import logging
import time

import pygame

log = logging.getLogger(__name__)

try:
    from pygame._sdl2 import controller as sdl_controller
except Exception:  # noqa: BLE001
    sdl_controller = None

AXES = ("surge", "yaw", "vert", "pitch")

KEY_AXES = {  # tecla positiva, tecla negativa
    "surge": (pygame.K_w, pygame.K_s),
    "yaw": (pygame.K_d, pygame.K_a),
    "vert": (pygame.K_r, pygame.K_f),
    "pitch": (pygame.K_t, pygame.K_g),
}

KEY_ACTIONS = {
    pygame.K_SPACE: "confirmar",
    pygame.K_c: "temperatura",
    pygame.K_h: "hold_prof",
    pygame.K_p: "hold_cab",
    pygame.K_RETURN: "armar",
    pygame.K_KP_ENTER: "armar",
    pygame.K_z: "cero_prof",
    pygame.K_l: "nivelar",
    pygame.K_F2: "panel_params",
    pygame.K_F3: "panel_vision",
    pygame.K_F5: "grabar",
    pygame.K_F11: "pantalla",
    pygame.K_ESCAPE: "salir",
    pygame.K_UP: "arriba",
    pygame.K_DOWN: "abajo",
    pygame.K_LEFT: "izq",
    pygame.K_RIGHT: "der",
    pygame.K_TAB: "siguiente_mascara",
    pygame.K_1: "pot:25",
    pygame.K_2: "pot:50",
    pygame.K_3: "pot:75",
    pygame.K_4: "pot:100",
}

PAD_ACTIONS = {  # botón SDL GameController -> acción
    "A": "confirmar",
    "X": "temperatura",
    "B": "hold_prof",
    "Y": "hold_cab",
    "START": "armar",
    "LEFTSHOULDER": "pot_menos",
    "RIGHTSHOULDER": "pot_mas",
}


def shape(v: float, deadzone: float, expo: float) -> float:
    """Zona muerta + curva exponencial (más fino cerca del centro)."""
    a = abs(v)
    if a < deadzone:
        return 0.0
    a = min(1.0, (a - deadzone) / (1.0 - deadzone))
    return (a ** expo) * (1 if v > 0 else -1)


class Controls:
    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg
        self.deadzone = float(cfg.get("zona_muerta", 0.12))
        self.expo = float(cfg.get("expo", 1.6))
        self.pad = None
        self.pad_kind = None  # "controller" | "joystick"
        self.pad_name = "sin mando"
        self._prev_buttons: dict[str, bool] = {}
        self._back_t0: float | None = None
        self._last_scan = 0.0
        if sdl_controller is not None:
            try:
                sdl_controller.init()
            except Exception:  # noqa: BLE001
                pass
        pygame.joystick.init()

    # ----------------------------------------------------------------- mando
    def _scan(self) -> None:
        self._last_scan = time.monotonic()
        self.pad, self.pad_kind, self.pad_name = None, None, "sin mando"
        n = pygame.joystick.get_count()
        for i in range(n):
            try:
                if sdl_controller is not None and sdl_controller.is_controller(i):
                    self.pad = sdl_controller.Controller(i)
                    self.pad_kind = "controller"
                    self.pad_name = getattr(self.pad, "name", "mando")
                    return
            except Exception:  # noqa: BLE001
                pass
        if n > 0:
            self.pad = pygame.joystick.Joystick(0)
            self.pad.init()
            self.pad_kind = "joystick"
            self.pad_name = self.pad.get_name() + " (genérico)"

    def _pad_ok(self) -> bool:
        if self.pad is None:
            if time.monotonic() - self._last_scan > 1.0:
                self._scan()
            return self.pad is not None
        try:
            if self.pad_kind == "controller" and not self.pad.attached():
                self.pad = None
                return False
        except Exception:  # noqa: BLE001
            self.pad = None
            return False
        return True

    def _read_pad(self) -> tuple[dict[str, float], dict[str, bool]]:
        axes = dict.fromkeys(AXES, 0.0)
        buttons: dict[str, bool] = {}
        if not self._pad_ok():
            return axes, buttons
        try:
            if self.pad_kind == "controller":
                g = self.pad.get_axis
                lx = g(pygame.CONTROLLER_AXIS_LEFTX) / 32767
                ly = g(pygame.CONTROLLER_AXIS_LEFTY) / 32767
                ry = g(pygame.CONTROLLER_AXIS_RIGHTY) / 32767
                lt = g(pygame.CONTROLLER_AXIS_TRIGGERLEFT) / 32767
                rt = g(pygame.CONTROLLER_AXIS_TRIGGERRIGHT) / 32767
                axes["surge"] = shape(-ly, self.deadzone, self.expo)
                axes["yaw"] = shape(lx, self.deadzone, self.expo)
                axes["vert"] = shape(-ry, self.deadzone, self.expo)
                axes["pitch"] = shape(rt - lt, self.deadzone, 1.0)
                for name in list(PAD_ACTIONS) + ["BACK"]:
                    buttons[name] = bool(self.pad.get_button(getattr(pygame, "CONTROLLER_BUTTON_" + name)))
            else:
                m = self.cfg.get("joystick_generico", {})
                ax = m.get("ejes", {})
                for name in AXES:
                    spec = ax.get(name)
                    if spec is None:
                        continue
                    idx, inv = spec
                    if idx < self.pad.get_numaxes():
                        axes[name] = shape(self.pad.get_axis(idx) * (-1 if inv else 1), self.deadzone, self.expo)
                for name, idx in m.get("botones", {}).items():
                    if idx < self.pad.get_numbuttons():
                        buttons[name] = bool(self.pad.get_button(idx))
        except Exception as e:  # noqa: BLE001
            log.warning("mando: %s", e)
            self.pad = None
        return axes, buttons

    # ----------------------------------------------------------------- API
    def update(self, events: list) -> tuple[dict[str, float], list[str]]:
        actions: list[str] = []
        for ev in events:
            if ev.type == pygame.KEYDOWN:
                act = KEY_ACTIONS.get(ev.key)
                if ev.key == pygame.K_s and ev.mod & pygame.KMOD_CTRL:
                    act = "guardar"
                if act in ("izq", "der", "arriba", "abajo") and ev.mod & pygame.KMOD_SHIFT:
                    act += "10"
                if act:
                    actions.append(act)
            elif ev.type in (getattr(pygame, "CONTROLLERDEVICEADDED", -1), pygame.JOYDEVICEADDED,
                             getattr(pygame, "CONTROLLERDEVICEREMOVED", -1), pygame.JOYDEVICEREMOVED):
                self._last_scan = 0.0
                self.pad = None

        axes, buttons = self._read_pad()
        for name, pressed in buttons.items():
            if pressed and not self._prev_buttons.get(name, False) and name in PAD_ACTIONS:
                actions.append(PAD_ACTIONS[name])
        # BACK sostenido 1 s = cero de profundidad (evita tocarlo por accidente)
        if buttons.get("BACK"):
            if self._back_t0 is None:
                self._back_t0 = time.monotonic()
            elif self._back_t0 > 0 and time.monotonic() - self._back_t0 >= 1.0:
                actions.append("cero_prof")
                self._back_t0 = -1.0  # no repetir hasta soltar
        else:
            self._back_t0 = None
        self._prev_buttons = buttons

        # Teclado: suma al mando (sirve de respaldo)
        keys = pygame.key.get_pressed()
        ctrl = pygame.key.get_mods() & pygame.KMOD_CTRL
        if not ctrl:
            for name, (kp, kn) in KEY_AXES.items():
                kv = (1.0 if keys[kp] else 0.0) - (1.0 if keys[kn] else 0.0)
                if kv:
                    axes[name] = max(-1.0, min(1.0, axes[name] + kv))
        return axes, actions
