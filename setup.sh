#!/usr/bin/env bash
# AquaBot — instalación del entorno de desarrollo (Ubuntu/Debian). Se puede correr varias veces.
#
#   bash setup.sh          todo: paquetes del sistema, permisos, entorno Python, PlatformIO + ESP32
#   bash setup.sh --no-apt    sin sudo (solo entorno Python y PlatformIO)
#   bash setup.sh --no-fw     sin descargar la plataforma ESP32 (~600 MB la primera vez)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$ROOT/.venv"
DO_APT=1
DO_FW=1
for a in "$@"; do
  case "$a" in
    --no-apt) DO_APT=0 ;;
    --no-fw) DO_FW=0 ;;
    -h|--help) sed -n '2,7p' "$0"; exit 0 ;;
    *) echo "Opción desconocida: $a"; exit 1 ;;
  esac
done

# Python del sistema, no el de conda/miniforge; y sin el PYTHONPATH de ROS2 (mezclaría librerías).
PYTHON="${PYTHON:-/usr/bin/python3}"
[[ -x "$PYTHON" ]] || PYTHON="$(command -v python3)"
unset PYTHONPATH PYTHONHOME
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "Nota: hay un entorno conda activo ($CONDA_PREFIX); se usa igual $PYTHON."
fi

step() { printf '\n\033[1;33m==> %s\033[0m\n' "$*"; }
ok()   { printf '    \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '    \033[31m!\033[0m %s\n' "$*"; }

cd "$ROOT"

# ------------------------------------------------------------------ 1. Paquetes del sistema
if [[ $DO_APT -eq 1 ]]; then
  step "Paquetes del sistema (pide la contraseña de sudo)"
  sudo apt-get update -qq
  # python3-venv: entorno virtual · v4l-utils: capturadora · build-essential: pruebas del firmware en el PC
  sudo apt-get install -y -qq python3-venv python3-pip v4l-utils build-essential git
  ok "python3-venv, v4l-utils, build-essential, git"

  step "Permiso para los puertos seriales (ESP32 y adaptador RS-485)"
  if id -nG "$USER" | grep -qw dialout; then
    ok "$USER ya está en el grupo dialout"
  else
    sudo usermod -aG dialout "$USER"
    warn "Agregado al grupo dialout: CIERRE SESIÓN Y VUELVA A ENTRAR para que aplique"
  fi
fi

# ------------------------------------------------------------------ 2. Entorno Python
step "Entorno Python en .venv (app de piloteo, pruebas y PlatformIO)"
PYV=$("$PYTHON" -c 'import sys; print("%d.%d" % sys.version_info[:2])')
"$PYTHON" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' \
  || { warn "Se necesita Python 3.10 o más nuevo (hay $PYV)"; exit 1; }
if [[ -d "$ROOT/pc/.venv" ]]; then
  warn "Hay un entorno viejo en pc/.venv; ya no se usa (el nuevo está en la raíz). Puede borrarlo: rm -rf pc/.venv"
fi
if [[ ! -x "$VENV/bin/python" ]] || ! "$VENV/bin/python" -c 'import sys' 2>/dev/null; then
  rm -rf "$VENV"
  "$PYTHON" -m venv "$VENV"
fi
"$VENV/bin/python" -m pip install -q --upgrade pip
"$VENV/bin/python" -m pip install -q -r pc/requirements.txt -r requirements-dev.txt
ok "Python $PYV, dependencias instaladas"
PYGAME_HIDE_SUPPORT_PROMPT=1 "$VENV/bin/python" -c 'import pygame, cv2, numpy, serial; print("    pygame", pygame.version.ver, "| opencv", cv2.__version__, "| numpy", numpy.__version__, "| pyserial", serial.__version__)' || true

# ------------------------------------------------------------------ 3. Configuración local
step "Configuración local de la app"
if [[ -f pc/config.json ]]; then
  ok "pc/config.json ya existe (no se toca)"
else
  cp pc/config.example.json pc/config.json
  ok "pc/config.json creado a partir de pc/config.example.json"
fi

# ------------------------------------------------------------------ 4. PlatformIO + plataforma ESP32
if [[ $DO_FW -eq 1 ]]; then
  step "PlatformIO: plataforma ESP32 y librerías del firmware (la primera vez tarda)"
  if PATH="$VENV/bin:$PATH" "$VENV/bin/pio" pkg install -d firmware >/dev/null 2>&1; then
    ok "espressif32 + OneWire + DallasTemperature listos"
  else
    warn "No se pudo descargar la plataforma (¿sin internet?). Reintente con: make fw-deps"
  fi
fi

# ------------------------------------------------------------------ 5. Prueba rápida
step "Pruebas"
if make -s test >/dev/null 2>&1; then
  ok "Pruebas del protocolo, control y detección: OK"
else
  warn "Alguna prueba falló: corra 'make test' para ver el detalle"
fi

cat <<EOF

Listo. Comandos útiles (desde $ROOT):
  make sim            app con el ROV simulado
  make piloto         app con el ROV real
  make dispositivos   ver puertos seriales y cámaras conectados
  make help           todos los comandos
Para usar python/pio a mano:  source .venv/bin/activate
EOF
