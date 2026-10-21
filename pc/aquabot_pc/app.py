"""Lógica principal de la app de piloteo."""
from __future__ import annotations

import logging
import time
from pathlib import Path

import cv2
import numpy as np
import pygame

from . import config as C
from . import protocol as P
from .controls import Controls
from .detection import TUNABLE, Detector
from .link import Link, SerialTransport
from .session import Session, now_str
from .ui import CYAN, H, RED, UI, VH, VW, W

log = logging.getLogger(__name__)

FPS = 25
BANNER_S = 3.0
ROI_FREEZE_S = 6.0


class App:
    def __init__(self, cfg: dict, cfg_path: Path, sim: bool = False, video=None, record: bool | None = None,
                 fullscreen: bool = False, demo_seconds: float | None = None) -> None:
        self.cfg, self.cfg_path = cfg, cfg_path
        self.demo_seconds = demo_seconds
        pygame.init()
        pygame.display.set_caption("EIAquatic — Piloto ROV")
        # SCALED: la superficie siempre es W×H (lo que se graba), y la ventana se escala si la pantalla es
        # más pequeña. F11 alterna pantalla completa.
        flags = pygame.SCALED | (pygame.FULLSCREEN if fullscreen else 0)
        try:
            self.screen = pygame.display.set_mode((W, H), flags)
        except pygame.error:
            self.screen = pygame.display.set_mode((W, H), pygame.FULLSCREEN if fullscreen else 0)
        self.ui = UI(self.screen)
        self.controls = Controls(cfg["mando"])
        self.detector = Detector(cfg["deteccion"])
        ses_dir = Path(cfg["carpeta_sesiones"])
        if not ses_dir.is_absolute():  # relativa a la carpeta de la app, no al directorio actual
            ses_dir = Path(__file__).resolve().parents[1] / ses_dir
        self.session = Session(ses_dir, fps=FPS)

        # Enlace: ROV real por serial o simulado
        self.sim = None
        if sim:
            from .sim import SimROV

            self.sim = SimROV()
            transport = self.sim
        else:
            transport = SerialTransport(cfg["puerto"], cfg["baudios"])
        self.link = Link(transport, cfg["frecuencia_cmd_hz"], on_telemetry=self.session.log_telemetry)

        # Video: capturadora, archivo o escena sintética del simulador
        from .video import SyntheticScene, VideoSource

        vc = cfg["video"]
        self.video = None
        if video == "none":
            self.video = None
        elif video is None and sim:
            self.video = VideoSource(SyntheticScene(self.sim), fps=vc["fps"])
        else:
            src = vc["dispositivo"] if video is None else video
            self.video = VideoSource(src, vc["ancho"], vc["alto"], vc["fps"], vc["fourcc"],
                                     rotation=vc.get("rotacion", 0), mirror=vc.get("espejo", False))

        # Estado de la interfaz
        self.frame: np.ndarray | None = None
        self.frame_id = 0
        self.tracks = []
        self.arm_req = False
        self.arm_req_t = 0.0
        self.pitch_hold = bool(cfg["cabeceo_auto_inicial"])
        self.depth_hold = False
        self.power = int(cfg["potencia_inicial"])
        self.panel: str | None = None
        self.panel_sel = 0
        self.mask_kind: str | None = "cubo"
        self.banner: tuple[str, tuple, float] | None = None
        self.toast: tuple[str, float] | None = None
        self.confirmed_ids: set[int] = set()
        self.last_confirm: dict | None = None
        self.frozen_roi: np.ndarray | None = None
        self.frozen_until = 0.0
        self.esc_t = 0.0
        self._was_link_ok = False
        self.running = True
        self.record_on_start = cfg["grabar_al_inicio"] if record is None else record
        self._init_sound()

    # ----------------------------------------------------------------- sonido
    def _init_sound(self) -> None:
        self.snd_detect = self.snd_info = None
        try:
            pygame.mixer.init(frequency=44100, size=-16, channels=1)
            sr = 44100

            def tone(freqs, dur):
                t = np.linspace(0, dur, int(sr * dur), False)
                wave = sum(np.sin(2 * np.pi * f * t) for f in freqs) / len(freqs)
                env = np.minimum(1, np.minimum(t, dur - t) * 40)
                pcm = (wave * env * 20000).astype(np.int16)
                channels = (pygame.mixer.get_init() or (0, 0, 1))[2]
                if channels > 1:  # algunos sistemas fuerzan estéreo
                    pcm = np.repeat(pcm[:, None], channels, axis=1)
                return pygame.sndarray.make_sound(np.ascontiguousarray(pcm))

            self.snd_detect = tone([880, 1320], 0.35)
            self.snd_info = tone([660], 0.15)
        except Exception as e:  # noqa: BLE001
            log.info("Sin audio: %s", e)

    def _play(self, snd) -> None:
        if snd is not None:
            try:
                snd.play()
            except Exception:  # noqa: BLE001
                pass

    # ----------------------------------------------------------------- mensajes en pantalla
    def say(self, text: str, seconds: float = 2.5) -> None:
        self.toast = (text, time.monotonic() + seconds)
        log.info(text)

    def roi_display(self) -> tuple[np.ndarray | None, bool]:
        if self.frozen_roi is not None and time.monotonic() < self.frozen_until:
            return self.frozen_roi, True
        best = self.detector.best_candidate()
        if best is None or self.frame is None:
            return None, False
        return self._crop(best.bbox), False

    def _crop(self, bbox) -> np.ndarray:
        x, y, w, h = bbox
        fh, fw = self.frame.shape[:2]
        m = int(0.35 * max(w, h)) + 6
        x0, y0 = max(0, x - m), max(0, y - m)
        x1, y1 = min(fw, x + w + m), min(fh, y + h + m)
        return self.frame[y0:y1, x0:x1].copy()

    # ----------------------------------------------------------------- acciones
    def confirm_detection(self) -> None:
        hora = now_str()
        t = self.link.telemetry
        best = self.detector.best_candidate()
        frame = self.frame.copy() if self.frame is not None else None
        if best is not None and frame is not None:
            label, bbox, roi = best.label, best.bbox, self._crop(best.bbox)
            self.confirmed_ids.add(best.id)
        else:
            label, bbox = "MARCA MANUAL", None
            roi = None
            if frame is not None:  # recorte central como evidencia
                fh, fw = frame.shape[:2]
                roi = frame[fh // 4 : 3 * fh // 4, fw // 4 : 3 * fw // 4].copy()
        self.session.log_event("DETECCION", hora, label, t, bbox, self.frame_id, frame, roi,
                               nota="confirmada por el piloto")
        k = self.session.n_detections
        self.last_confirm = {"k": k, "label": label, "hora": hora}
        self.frozen_roi, self.frozen_until = roi, time.monotonic() + ROI_FREEZE_S
        self.banner = (f"OBJETO DETECTADO #{k}  ·  {label}  ·  {hora}", RED, time.monotonic() + BANNER_S)
        self._play(self.snd_detect)

    def mark_temperature(self) -> None:
        t = self.link.telemetry
        if t is None or t.water_temp is None or not self.link.ok:
            self.say("Sin lectura de temperatura")
            return
        hora = now_str()
        self.session.log_event("TEMPERATURA", hora, "", t, None, self.frame_id, self.frame,
                               nota=f"{t.water_temp:.2f} C")
        self.banner = (f"TEMPERATURA {t.water_temp:.2f} °C  ·  prof. {t.depth:.2f} m  ·  {hora}", CYAN,
                       time.monotonic() + BANNER_S)
        self._play(self.snd_info)

    def toggle_arm(self) -> None:
        if self.arm_req:
            self.arm_req = False
            self.say("Desarmado")
            self.session.log_event("DESARMADO", now_str())
        elif not self.link.ok:
            self.say("No se puede armar: sin enlace con el ROV")
        else:
            self.arm_req, self.arm_req_t = True, time.monotonic()
            self.say("ARMADO — motores activos")
            self.session.log_event("ARMADO", now_str())

    def _panel_rows(self) -> int:
        return len(P.PARAMS) if self.panel == "params" else len(TUNABLE)

    def _adjust(self, direction: int, mult: int) -> None:
        if self.panel == "params":
            p = P.PARAMS[self.panel_sel]
            cur = self.link.params.get(p.id, p.default)
            new = round(min(p.max, max(p.min, cur + direction * mult * p.step)), 6)
            self.link.params[p.id] = new  # se muestra ya; el ROV confirma con PARAM_VALUE
            self.link.set_param(p.id, new, save=False)
        elif self.panel == "vision":
            sec, key, step, lo, hi = TUNABLE[self.panel_sel]
            d = self.detector.cfg[sec] if sec else self.detector.cfg
            v = d[key] + direction * mult * step
            v = min(hi, max(lo, v))
            d[key] = round(v, 4) if isinstance(step, float) else int(v)
            if sec in ("cubo", "generico"):
                self.mask_kind = sec

    def handle(self, action: str) -> None:
        if action != "salir":
            self.esc_t = 0.0
        if self.panel and action in ("arriba", "abajo", "arriba10", "abajo10", "izq", "der", "izq10", "der10",
                                     "guardar", "siguiente_mascara"):
            n = self._panel_rows()
            mult = 10 if action.endswith("10") else 1
            if action.startswith("arriba"):
                self.panel_sel = (self.panel_sel - mult) % n
            elif action.startswith("abajo"):
                self.panel_sel = (self.panel_sel + mult) % n
            elif action.startswith("izq"):
                self._adjust(-1, mult)
            elif action.startswith("der"):
                self._adjust(+1, mult)
            elif action == "siguiente_mascara":
                kinds = [None, "cubo", "generico"]
                self.mask_kind = kinds[(kinds.index(self.mask_kind) + 1) % len(kinds)]
            elif action == "guardar":
                if self.panel == "params":
                    p = P.PARAMS[self.panel_sel]
                    self.link.set_param(p.id, self.link.params.get(p.id, p.default), save=True)
                    self.say("Parámetros guardados en la flash del ROV")
                else:
                    self.cfg["deteccion"] = self.detector.cfg
                    C.save(self.cfg_path, self.cfg)
                    self.say(f"Ajuste de visión guardado en {self.cfg_path.name}")
            return

        if action == "confirmar":
            self.confirm_detection()
        elif action == "temperatura":
            self.mark_temperature()
        elif action == "armar":
            self.toggle_arm()
        elif action == "hold_cab":
            self.pitch_hold = not self.pitch_hold
            self.say("Cabeceo AUTOMÁTICO (IMU)" if self.pitch_hold else "Cabeceo MANUAL")
        elif action == "hold_prof":
            t = self.link.telemetry
            if not self.depth_hold and (t is None or not t.has(P.TF_DEPTH_OK)):
                self.say("Hold de profundidad no disponible: sin sensor de presión")
            else:
                self.depth_hold = not self.depth_hold
                self.say("Profundidad AUTOMÁTICA (lastre)" if self.depth_hold else "Profundidad MANUAL (lastre)")
        elif action == "cero_prof":
            self.link.pulse_flag(P.CF_ZERO_DEPTH)
            self.session.log_event("CERO_PROFUNDIDAD", now_str())
            self.say("Cero de profundidad: el ROV debe estar en la superficie")
        elif action == "nivelar":
            t = self.link.telemetry
            if t is not None and t.has(P.TF_ARMED):
                self.say("Desarme el ROV antes de nivelar la IMU")
            else:
                self.link.pulse_flag(P.CF_LEVEL_IMU)
                self.say("IMU nivelada (el ROV debe estar quieto y horizontal)")
        elif action == "pot_mas":
            self.power = min(100, self.power + 25)
            self.say(f"Potencia {self.power} %", 1.2)
        elif action == "pot_menos":
            self.power = max(25, self.power - 25)
            self.say(f"Potencia {self.power} %", 1.2)
        elif action.startswith("pot:"):
            self.power = int(action[4:])
            self.say(f"Potencia {self.power} %", 1.2)
        elif action in ("panel_params", "panel_vision"):
            want = "params" if action == "panel_params" else "vision"
            self.panel = None if self.panel == want else want
            self.panel_sel = 0
        elif action == "grabar":
            if self.session.recording:
                self.session.stop_recording()
                self.say("Grabación detenida")
            elif self.session.start_recording((W, H)):
                self.say(f"Grabando en {self.session.video_path}")
        elif action == "pantalla":
            try:
                pygame.display.toggle_fullscreen()
            except pygame.error as e:
                self.say(f"No se pudo cambiar a pantalla completa: {e}")
        elif action == "salir":
            if time.monotonic() - self.esc_t < 2.0:
                self.running = False
            else:
                self.esc_t = time.monotonic()
                self.say("Pulse ESC otra vez para salir (el ROV se desarma)")

    # ----------------------------------------------------------------- ciclo principal
    def _safety(self) -> None:
        ok = self.link.ok
        t = self.link.telemetry
        if self._was_link_ok and not ok and self.arm_req:
            self.arm_req = False
            self.say("ENLACE PERDIDO — ROV desarmado. Pulse START para re-armar.", 5)
            self.session.log_event("ENLACE_PERDIDO", now_str())
        self._was_link_ok = ok
        # El ROV desarma solo tras un failsafe y exige soltar y volver a armar.
        if self.arm_req and ok and t is not None and not t.has(P.TF_ARMED) \
                and time.monotonic() - self.arm_req_t > 1.0:
            self.arm_req = False
            self.say("El ROV se desarmó (failsafe). Pulse START para re-armar.", 5)
        if self.depth_hold and t is not None and ok and not t.has(P.TF_DEPTH_OK):
            self.depth_hold = False
            self.say("Hold de profundidad desactivado: falla del sensor de presión", 4)

    def run(self) -> None:
        self.link.start()
        if self.video:
            self.video.start()
        if self.record_on_start:
            self.session.start_recording((W, H))
        clock = pygame.time.Clock()
        t_start = time.monotonic()
        demo_done: set[str] = set()
        try:
            while self.running:
                events = pygame.event.get()
                for ev in events:
                    if ev.type == pygame.QUIT:
                        self.running = False
                axes, actions = self.controls.update(events)
                if self.demo_seconds is not None:  # prueba automática sin pantalla
                    el = time.monotonic() - t_start
                    for at, act in ((1.0, "armar"), (2.5, "confirmar"), (3.0, "temperatura"), (3.5, "hold_prof"),
                                    (4.0, "panel_params"), (4.2, "der"), (4.4, "panel_vision")):
                        if el >= at and act not in demo_done:
                            demo_done.add(act)
                            actions.append(act)
                    axes["surge"] = 0.6 if 1.2 < el < 3.0 else 0.0
                    if el >= self.demo_seconds:
                        self.running = False
                for a in actions:
                    self.handle(a)
                self._safety()

                flags = (P.CF_ARM if self.arm_req else 0) | (P.CF_PITCH_HOLD if self.pitch_hold else 0) \
                    | (P.CF_DEPTH_HOLD if self.depth_hold else 0)
                self.link.set_command(flags=flags, surge=axes["surge"], yaw=axes["yaw"], pitch=axes["pitch"],
                                      vert=axes["vert"], power=self.power)

                if self.video:
                    frame, fid, _ = self.video.latest()
                    self.video_status = self.video.status
                    if frame is not None and fid != self.frame_id:
                        self.frame, self.frame_id = frame, fid
                        self.tracks = self.detector.process(frame)
                        self.session.write_raw(frame)
                else:
                    self.video_status = "desactivado"

                self.ui.draw(self)
                pygame.display.flip()
                if self.session.recording:
                    self.session.write_frame(pygame.image.tobytes(self.screen, "RGB"), self.screen.get_size())
                clock.tick(FPS)
        finally:
            self.shutdown()

    video_status = "—"

    def shutdown(self) -> None:
        # Desarmar antes de cerrar y dar tiempo a que salga el comando.
        self.link.set_command(flags=0, surge=0, yaw=0, pitch=0, vert=0)
        time.sleep(0.2)
        self.link.stop()
        if self.video:
            self.video.stop()
        self.session.close()
        pygame.quit()
        log.info("Sesión guardada en %s", self.session.dir)
