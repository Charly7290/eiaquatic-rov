#!/usr/bin/env python3
"""AquaBot USB 2026 — app de piloteo.

Ejemplos:
  python piloto.py --sim                    # ROV y video simulados (practicar sin hardware)
  python piloto.py --port /dev/ttyUSB0      # ROV real por el adaptador USB-RS485
  python piloto.py --port /dev/ttyUSB0 --video 0
  python piloto.py --sim --video prueba.mp4 # ajustar la detección con un video grabado
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from aquabot_pc import config as C  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description="AquaBot — piloteo")
    ap.add_argument("--sim", action="store_true", help="usar el ROV simulado")
    ap.add_argument("--port", help="puerto serial (sobrescribe config.json)")
    ap.add_argument("--video", help="índice de la capturadora, ruta de un video, o 'none'")
    ap.add_argument("--config", default=str(HERE / "config.json"), help="archivo de configuración")
    ap.add_argument("--no-rec", action="store_true", help="no grabar video al iniciar (F5 lo activa)")
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--demo", type=float, help=argparse.SUPPRESS)  # prueba automática (segundos)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    cfg_path = Path(args.config)
    cfg = C.load(cfg_path)
    if args.port:
        cfg["puerto"] = args.port
    video = None
    if args.video is not None:
        video = int(args.video) if args.video.isdigit() else args.video

    from aquabot_pc.app import App

    App(cfg, cfg_path, sim=args.sim, video=video, record=False if args.no_rec else None,
        fullscreen=args.fullscreen, demo_seconds=args.demo).run()


if __name__ == "__main__":
    main()
