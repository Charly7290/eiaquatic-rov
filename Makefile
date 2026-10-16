# AquaBot — comandos del proyecto. `make help` para la lista.
# Todo usa el entorno .venv de la raíz (lo crea: bash setup.sh).

VENV   := .venv
PY     := $(CURDIR)/$(VENV)/bin/python
PIO    := $(CURDIR)/$(VENV)/bin/pio
FW     := firmware
PC     := pc
CXX    ?= g++

# Ignorar el entorno de ROS2/conda si la terminal lo tiene cargado (mezclaría librerías de Python).
unexport PYTHONPATH PYTHONHOME

# Variables opcionales:  make piloto PORT=/dev/ttyACM0 VIDEO=4
PORT   ?=
VIDEO  ?=
ARGS   ?=
PILOTO_ARGS := $(if $(PORT),--port $(PORT)) $(if $(VIDEO),--video $(VIDEO)) $(ARGS)

.DEFAULT_GOAL := help
.PHONY: help setup sim piloto joytest dispositivos \
        bringup rov rov-usb fw-build monitor fw-deps \
        test test-pc test-fw robust clean

help: ## Muestra esta ayuda
	@awk 'BEGIN{FS=":.*## "} /^[a-zA-Z_-]+:.*## /{printf "  \033[33m%-14s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""
	@echo "  Variables: PORT=/dev/... VIDEO=N ARGS='...'   (ej.: make piloto PORT=/dev/ttyACM0)"

setup: ## Instala/actualiza todo el entorno (setup.sh)
	bash setup.sh

$(PY):
	@echo "Falta el entorno. Corra primero: bash setup.sh"; exit 1

# ------------------------------------------------------------------ App del PC
sim: | $(PY) ## App con el ROV y el video simulados
	cd $(PC) && $(PY) piloto.py --sim $(PILOTO_ARGS)

piloto: | $(PY) ## App con el ROV real (puerto y video de pc/config.json)
	cd $(PC) && $(PY) piloto.py $(PILOTO_ARGS)

joytest: | $(PY) ## Muestra ejes y botones del mando
	cd $(PC) && $(PY) joytest.py

dispositivos: ## Lista puertos seriales y cámaras conectados
	@echo "Puertos seriales (use la ruta by-id en pc/config.json: no cambia al reconectar):"
	@if [ -d /dev/serial/by-id ]; then ls -l /dev/serial/by-id/ | awk 'NR>1{print "  /dev/serial/by-id/"$$9"  ->  "$$11}'; else echo "  (ninguno)"; fi
	@echo ""
	@echo "Cámaras / capturadoras:"
	@v4l2-ctl --list-devices 2>/dev/null | sed 's/^/  /' || echo "  (v4l2-ctl no instalado: sudo apt install v4l-utils)"

# ------------------------------------------------------------------ Firmware (ESP32 por USB)
bringup: | $(PY) ## Compila y carga el programa de pruebas, y abre el monitor
	$(PIO) run -d $(FW) -e bringup -t upload
	$(PIO) device monitor -d $(FW)

rov: | $(PY) ## Compila y carga el firmware de COMPETENCIA (enlace RS-485)
	$(PIO) run -d $(FW) -e rov -t upload

rov-usb: | $(PY) ## Compila y carga el firmware de banco (enlace por el USB de la ESP32)
	$(PIO) run -d $(FW) -e rov_usb -t upload

fw-build: | $(PY) ## Solo compila todos los programas del firmware (sin cargar)
	$(PIO) run -d $(FW) -e rov -e rov_usb -e bringup

monitor: | $(PY) ## Monitor serial de la ESP32 (115200)
	$(PIO) device monitor -d $(FW)

fw-deps: | $(PY) ## Descarga la plataforma ESP32 y las librerías
	$(PIO) pkg install -d $(FW)

# ------------------------------------------------------------------ Pruebas
test: test-fw test-pc ## Todas las pruebas (firmware en el PC + Python)

test-fw: ## Pruebas del firmware en el PC (protocolo y control)
	@mkdir -p $(FW)/test_host/build
	$(CXX) -std=c++17 -Wall -Wextra -I$(FW)/src $(FW)/test_host/test_host.cpp $(FW)/src/protocol.cpp \
		$(FW)/src/control.cpp -o $(FW)/test_host/build/test_host
	cd $(FW)/test_host && ./build/test_host

test-pc: | $(PY) ## Pruebas de la app (protocolo C++ <-> Python, detección)
	cd $(PC) && $(PY) -m pytest -q tests

robust: ## Barrido de robustez del hold de profundidad
	@mkdir -p $(FW)/test_host/build
	$(CXX) -O2 -std=c++17 -I$(FW)/src $(FW)/test_host/robust.cpp $(FW)/src/control.cpp -o $(FW)/test_host/build/robust
	./$(FW)/test_host/build/robust

clean: ## Borra compilados y cachés (no toca .venv, config.json ni sesiones)
	rm -rf $(FW)/.pio $(FW)/test_host/build $(FW)/test_host/vectors.txt
	find . -path ./$(VENV) -prune -o \( -name __pycache__ -o -name .pytest_cache \) -type d -print -exec rm -rf {} +
