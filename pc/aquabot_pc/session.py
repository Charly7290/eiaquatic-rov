"""Registro de la sesión: telemetría, eventos (detecciones, temperatura), video de la ventana y capturas."""
from __future__ import annotations

import csv
import logging
import queue
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from . import protocol as P

log = logging.getLogger(__name__)

TELEM_COLS = [
    "hora_pc", "uptime_s", "flags", "armado", "prof_m", "setpoint_m", "vz_ms", "presion_mbar", "temp_agua_c",
    "temp_ms5837_c", "cabeceo_deg", "alabeo_deg", "cabeceo_sp_deg", "vbat_v", "lastre_pos", "lastre_dir",
    "m1", "m2", "m3", "m4", "m5", "rx_err",
    "cmd_flags", "cmd_surge", "cmd_yaw", "cmd_pitch", "cmd_vert", "cmd_potencia",
]
EVENT_COLS = ["n", "hora", "tipo", "clase", "prof_m", "temp_agua_c", "cabeceo_deg", "bbox", "cuadro",
              "captura", "roi", "nota"]


def now_str() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class Session:
    def __init__(self, base_dir: str | Path, fps: float = 25.0) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = Path(base_dir) / stamp
        (self.dir / "capturas").mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self._lock = threading.Lock()
        self._tf = open(self.dir / "telemetria.csv", "w", newline="", encoding="utf-8")
        self._tw = csv.writer(self._tf)
        self._tw.writerow(TELEM_COLS)
        self._ef = open(self.dir / "eventos.csv", "w", newline="", encoding="utf-8")
        self._ew = csv.writer(self._ef)
        self._ew.writerow(EVENT_COLS)
        self._ef.flush()
        self.writer: cv2.VideoWriter | None = None
        self.raw_writer: cv2.VideoWriter | None = None  # cámara sin interfaz (para calibrar la visión)
        self.video_path: Path | None = None
        self._idx = 0
        self.recording = False
        self.dropped = 0
        self._q: "queue.Queue" = queue.Queue(maxsize=8)
        self._wt = threading.Thread(target=self._writer_loop, daemon=True, name="grabacion")
        self._wt.start()
        self.n_events = 0
        self.n_detections = 0
        self._rows = 0
        log.info("Sesión en %s", self.dir)

    # ----------------------------------------------------------------- telemetría (hilo del enlace)
    def log_telemetry(self, t: P.Telemetry, cmd: P.Command) -> None:
        with self._lock:
            if self._tf.closed:
                return
            self._tw.writerow([
                now_str(), f"{t.uptime:.3f}", t.flags, int(t.has(P.TF_ARMED)), f"{t.depth:.3f}",
                f"{t.depth_sp:.3f}", f"{t.vz:.3f}", t.pressure_mbar,
                "" if t.water_temp is None else f"{t.water_temp:.2f}", f"{t.ms_temp:.2f}", f"{t.pitch:.2f}",
                f"{t.roll:.2f}", f"{t.pitch_sp:.2f}", f"{t.vbat:.3f}",
                "" if t.ballast_pos is None else f"{t.ballast_pos:.2f}", t.ballast_dir,
                *[f"{m:.2f}" for m in t.motors], t.rx_err,
                cmd.flags, f"{cmd.surge:.3f}", f"{cmd.yaw:.3f}", f"{cmd.pitch:.3f}", f"{cmd.vert:.3f}", cmd.power,
            ])
            self._rows += 1
            if self._rows % 20 == 0:
                self._tf.flush()

    # ----------------------------------------------------------------- eventos
    def log_event(self, tipo: str, hora: str, clase: str = "", t: P.Telemetry | None = None, bbox=None,
                  frame_id: int = 0, frame: np.ndarray | None = None, roi: np.ndarray | None = None,
                  nota: str = "") -> int:
        with self._lock:
            self.n_events += 1
            n = self.n_events
            if tipo == "DETECCION":
                self.n_detections += 1
            cap_name = roi_name = ""
            if frame is not None:
                cap_name = f"capturas/{n:03d}_{tipo.lower()}.jpg"
                cv2.imwrite(str(self.dir / cap_name), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if roi is not None and roi.size:
                roi_name = f"capturas/{n:03d}_roi.jpg"
                cv2.imwrite(str(self.dir / roi_name), roi, [cv2.IMWRITE_JPEG_QUALITY, 95])
            self._ew.writerow([
                n, hora, tipo, clase,
                "" if t is None else f"{t.depth:.3f}",
                "" if t is None or t.water_temp is None else f"{t.water_temp:.2f}",
                "" if t is None else f"{t.pitch:.1f}",
                "" if bbox is None else " ".join(str(int(v)) for v in bbox), frame_id, cap_name, roi_name, nota,
            ])
            self._ef.flush()
            return n

    # ----------------------------------------------------------------- video de la ventana
    def start_recording(self, size: tuple[int, int]) -> bool:
        if self.recording:
            return True
        self._idx += 1
        w, path = self._open_writer(f"video_{self._idx:02d}", size)
        if w is None:
            log.error("No se pudo abrir el grabador de video")
            return False
        self.writer, self.video_path, self.recording = w, path, True
        log.info("Grabando %s", path)
        return True

    def _open_writer(self, stem: str, size: tuple[int, int]):
        for ext, fourcc in (("mp4", "mp4v"), ("avi", "MJPG")):
            path = self.dir / f"{stem}.{ext}"
            w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*fourcc), self.fps, size)
            if w.isOpened():
                return w, path
        return None, None

    # La codificación del video va en un hilo aparte para no frenar la interfaz. El hilo principal solo
    # copia los bytes de la pantalla; si el disco se atrasa, se descartan cuadros en vez de bloquear.
    def _writer_loop(self) -> None:
        while True:
            item = self._q.get()
            if item is None:
                return
            kind, writer, data = item
            try:
                if kind == "ui":
                    raw, (w, h) = data
                    rgb = np.frombuffer(raw, np.uint8).reshape(h, w, 3)
                    writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                elif kind == "raw":
                    writer.write(data)
                elif kind == "release":
                    for wr in writer:
                        if wr is not None:
                            wr.release()
            except Exception:  # noqa: BLE001
                log.exception("grabación")

    def _put(self, item) -> None:
        try:
            self._q.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def stop_recording(self) -> None:
        if self.writer is not None or self.raw_writer is not None:
            self._q.put(("release", (self.writer, self.raw_writer), None))  # después de los cuadros pendientes
        self.writer, self.raw_writer, self.recording = None, None, False

    def write_frame(self, rgb_bytes: bytes, size: tuple[int, int]) -> None:
        """Cuadro de la ventana completa como bytes RGB (pygame.image.tobytes)."""
        if self.writer is not None:
            self._put(("ui", self.writer, (rgb_bytes, size)))

    def write_raw(self, bgr: np.ndarray) -> None:
        """Cuadro de la cámara tal cual (sin interfaz), para calibrar la detección después."""
        if not self.recording:
            return
        if self.raw_writer is None:
            h, w = bgr.shape[:2]
            self.raw_writer, _ = self._open_writer(f"camara_{self._idx:02d}", (w, h))
        if self.raw_writer is not None:
            self._put(("raw", self.raw_writer, bgr))

    def close(self) -> None:
        self.stop_recording()
        self._q.put(None)
        self._wt.join(timeout=10)
        if self.dropped:
            log.warning("Grabación: %d cuadros descartados (disco o CPU lentos)", self.dropped)
        with self._lock:
            self._tf.close()
            self._ef.close()
