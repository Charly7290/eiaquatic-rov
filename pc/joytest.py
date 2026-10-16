#!/usr/bin/env python3
"""Muestra los ejes y botones del mando conectado (para configurar un mando no reconocido).

  python joytest.py
"""
import time

import pygame

pygame.init()
pygame.joystick.init()
try:
    from pygame._sdl2 import controller

    controller.init()
except Exception:  # noqa: BLE001
    controller = None

if pygame.joystick.get_count() == 0:
    print("No hay mandos conectados.")
    raise SystemExit(1)

js = pygame.joystick.Joystick(0)
js.init()
known = controller is not None and controller.is_controller(0)
print(f"Mando: {js.get_name()} | ejes {js.get_numaxes()} | botones {js.get_numbuttons()}")
print("Reconocido por SDL (mapeo automático):", "SÍ" if known else "NO -> configure 'joystick_generico' en config.json")
print("Mueva sticks y pulse botones. Ctrl+C para salir.\n")
try:
    while True:
        pygame.event.pump()
        axes = " ".join(f"{i}:{js.get_axis(i):+.2f}" for i in range(js.get_numaxes()))
        btns = " ".join(str(i) for i in range(js.get_numbuttons()) if js.get_button(i))
        print(f"\rejes [{axes}]  botones [{btns:<12}]", end="", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    print()
