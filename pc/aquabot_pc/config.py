"""Configuración de la app.

- config.example.json (en git): valores de referencia del equipo.
- config.json (local, fuera de git): la copia de cada portátil. Se crea a partir del ejemplo la
  primera vez. Lo que no esté en el archivo toma el valor por defecto de DEFAULTS.
"""
from __future__ import annotations

import copy
import json
import shutil
import sys
from pathlib import Path

from .detection import DEFAULT_CFG as DETECTION_DEFAULTS

DEFAULTS: dict = {
    "puerto": "COM5" if sys.platform.startswith("win") else "/dev/ttyUSB0",
    "baudios": 115200,
    "frecuencia_cmd_hz": 20,
    "video": {
        "dispositivo": 4,      # índice de la capturadora (/dev/video4 en el portátil de piloteo)
        "ancho": 720,
        "alto": 576,           # 576 = PAL, 480 = NTSC (debe coincidir con el OSD de la Caddx)
        "fps": 25,
        "fourcc": None,        # p. ej. "YUYV" si la capturadora lo necesita
    },
    "carpeta_sesiones": "sesiones",
    "grabar_al_inicio": True,
    "potencia_inicial": 50,
    "cabeceo_auto_inicial": True,
    "mando": {
        "zona_muerta": 0.12,
        "expo": 1.6,
        # Solo para mandos que SDL no reconoce (ver joytest.py): [índice, invertido]
        "joystick_generico": {
            "ejes": {"surge": [1, True], "yaw": [0, False], "vert": [3, True], "pitch": None},
            "botones": {"A": 0, "B": 1, "X": 2, "Y": 3, "LEFTSHOULDER": 4, "RIGHTSHOULDER": 5,
                        "BACK": 6, "START": 7},
        },
    },
    "deteccion": DETECTION_DEFAULTS,
}


def _merge(base: dict, user: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in user.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load(path: str | Path) -> dict:
    path = Path(path)
    if not path.exists():
        example = path.with_name("config.example.json")
        if example.exists():
            shutil.copyfile(example, path)
        else:
            save(path, DEFAULTS)
    with open(path, encoding="utf-8") as f:
        return _merge(DEFAULTS, json.load(f))


def save(path: str | Path, cfg: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
