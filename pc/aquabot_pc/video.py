"""Fuentes de video: capturadora (EasyCap), archivo de video o escena sintética del simulador."""
from __future__ import annotations

import logging
import math
import sys
import threading
import time

import cv2
import numpy as np

log = logging.getLogger(__name__)


class VideoSource(threading.Thread):
    """Lee cuadros en un hilo y guarda siempre el último (la interfaz nunca espera a la cámara).

    source: índice de dispositivo (int), ruta de archivo (str) o un objeto con render() (escena sintética).
    """

    _ROT = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}

    def __init__(self, source, width: int | None = None, height: int | None = None, fps: float = 25.0,
                 fourcc: str | None = None, rotation: int = 0, mirror: bool = False) -> None:
        super().__init__(daemon=True, name="video")
        self.source = source
        self.width, self.height, self.fps, self.fourcc = width, height, fps, fourcc
        # Orientación de la cámara montada en el ROV: se corrige solo en la capturadora (un video grabado
        # con camara_NN.mp4 ya viene corregido).
        self.rotation = int(rotation) % 360
        self.mirror = bool(mirror)
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._id = 0
        self._t = 0.0
        self._stop = threading.Event()
        self.status = "iniciando"
        self.measured_fps = 0.0

    def latest(self) -> tuple[np.ndarray | None, int, float]:
        with self._lock:
            return self._frame, self._id, self._t

    def stop(self) -> None:
        self._stop.set()

    def _publish(self, frame: np.ndarray) -> None:
        with self._lock:
            self._frame = frame
            self._id += 1
            self._t = time.time()

    def run(self) -> None:
        if hasattr(self.source, "render"):
            self._run_synthetic()
        else:
            self._run_capture()

    def _run_synthetic(self) -> None:
        self.status = "simulado"
        period = 1.0 / self.fps
        nxt = time.monotonic()
        while not self._stop.is_set():
            self._publish(self.source.render())
            self.measured_fps = self.fps
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _open(self):
        if isinstance(self.source, int):
            if sys.platform.startswith("linux"):
                cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)
            elif sys.platform.startswith("win"):
                cap = cv2.VideoCapture(self.source, cv2.CAP_DSHOW)
            else:
                cap = cv2.VideoCapture(self.source)
            if self.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            if self.width:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            if self.height:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # menor latencia
        else:
            cap = cv2.VideoCapture(str(self.source))
        return cap

    def _run_capture(self) -> None:
        is_file = not isinstance(self.source, int)
        cap = None
        n, t_rate = 0, time.monotonic()
        while not self._stop.is_set():
            if cap is None or not cap.isOpened():
                self.status = "conectando"
                cap = self._open()
                if not cap.isOpened():
                    self.status = f"sin video ({self.source})"
                    time.sleep(2.0)
                    continue
                file_fps = cap.get(cv2.CAP_PROP_FPS) if is_file else 0
                period = 1.0 / (file_fps if file_fps and file_fps < 120 else self.fps)
                nxt = time.monotonic()
                self.status = "ok"
            ok, frame = cap.read()
            if not ok:
                if is_file:  # repetir el archivo en bucle
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                self.status = "señal perdida"
                cap.release()
                cap = None
                time.sleep(0.5)
                continue
            if not is_file:
                if self.rotation in self._ROT:
                    frame = cv2.rotate(frame, self._ROT[self.rotation])
                if self.mirror:
                    frame = cv2.flip(frame, 1)
            self._publish(frame)
            n += 1
            now = time.monotonic()
            if now - t_rate >= 1.0:
                self.measured_fps = n / (now - t_rate)
                n, t_rate = 0, now
            if is_file:
                nxt += period
                time.sleep(max(0.0, nxt - time.monotonic()))
        if cap is not None:
            cap.release()


class SyntheticScene:
    """Fondo de piscina visto desde el ROV simulado, con cubos naranja, monedas y un objeto extra.

    Vista aproximada (cenital) para probar el flujo de detección y la interfaz; no pretende ser realista.
    """

    W, H = 720, 576
    PPM = 100  # píxeles por metro de la textura del fondo
    XMIN, XMAX, YMIN, YMAX = -4.0, 16.0, -6.0, 6.0
    POOL_DEPTH = 2.0

    def __init__(self, sim) -> None:
        self.sim = sim
        rng = np.random.default_rng(3)
        w = int((self.XMAX - self.XMIN) * self.PPM)
        h = int((self.YMAX - self.YMIN) * self.PPM)
        tex = np.zeros((h, w, 3), np.uint8)
        tex[:] = (205, 175, 95)  # azul claro de baldosa (BGR)
        tex = (tex.astype(np.int16) + rng.integers(-8, 8, (h, w, 1))).clip(0, 255).astype(np.uint8)
        step = int(0.25 * self.PPM)
        tex[::step, :] = (225, 205, 150)  # juntas
        tex[:, ::step] = (225, 205, 150)
        for yl in np.arange(-5.0, 6.0, 2.5):  # líneas de carril oscuras
            r0 = int((self.YMAX - yl - 0.12) * self.PPM)
            tex[r0 : r0 + int(0.25 * self.PPM), :] = (95, 45, 25)
        self.tex = tex
        self.objects = [
            ("cube", 3.0, 0.8, 0.15, 20), ("pyramid", 8.0, -1.8, 0.15, 55), ("cube", 12.5, 2.7, 0.15, 10),
            ("coin", 2.0, -0.5, 0.0125, 0), ("coin", 2.2, -0.4, 0.0125, 0), ("coin", 6.0, 1.4, 0.012, 0),
            ("coin", 10.0, -0.2, 0.0125, 0), ("rock", 5.0, -1.0, 0.10, 0),
        ]
        self.noise_rng = np.random.default_rng()

    def render(self) -> np.ndarray:
        x, y, hdg, depth, _pitch = self.sim.pose()
        hf = max(0.3, self.POOL_DEPTH - depth)  # altura sobre el fondo
        s = self.W / (2.0 * hf)  # px/m (campo de visión ~90°)
        cx, cy = self.W / 2, self.H / 2
        ch, sh = math.cos(hdg), math.sin(hdg)
        k = self.PPM / s
        M = np.array([
            [k * sh, -k * ch, self.PPM * (x - self.XMIN + cy / s * ch - cx / s * sh)],
            [k * ch, k * sh, self.PPM * (self.YMAX - y - cy / s * sh - cx / s * ch)],
        ], np.float32)
        img = cv2.warpAffine(self.tex, M, (self.W, self.H), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                             borderValue=(150, 120, 60))

        def to_img(wx, wy):
            dx, dy = wx - x, wy - y
            f = dx * ch + dy * sh
            r = dx * sh - dy * ch
            return cx + r * s, cy - f * s

        for kind, ox, oy, size, ang in self.objects:
            u, v = to_img(ox, oy)
            rad = size * s
            if not (-rad < u < self.W + rad and -rad < v < self.H + rad):
                continue
            if kind == "pyramid":  # pirámide vista desde arriba: cuadrado con 4 caras de distinto brillo
                box = cv2.boxPoints(((u, v), (2 * rad, 2 * rad), ang - math.degrees(hdg))).astype(np.int32)
                apex = (int(u), int(v))
                for i, shade in enumerate(((20, 125, 250), (15, 100, 215), (10, 80, 180), (18, 112, 232))):
                    tri = np.array([box[i], box[(i + 1) % 4], apex], np.int32)
                    cv2.fillConvexPoly(img, tri, shade, cv2.LINE_AA)
            elif kind == "cube":
                box = cv2.boxPoints(((u, v), (2 * rad, 2 * rad), ang - math.degrees(hdg)))
                cv2.fillConvexPoly(img, box.astype(np.int32), (20, 115, 245), cv2.LINE_AA)
                cv2.polylines(img, [box.astype(np.int32)], True, (10, 70, 170), max(1, int(rad / 8)), cv2.LINE_AA)
            elif kind == "coin":
                r = max(2, int(rad))
                cv2.circle(img, (int(u), int(v)), r, (150, 170, 175), -1, cv2.LINE_AA)
                cv2.circle(img, (int(u), int(v)), r, (90, 105, 110), max(1, r // 5), cv2.LINE_AA)
                cv2.circle(img, (int(u - r / 3), int(v - r / 3)), max(1, r // 3), (235, 240, 240), -1, cv2.LINE_AA)
            else:  # roca gris irregular (objeto "genérico")
                pts = [(u + rad * (1 + 0.3 * math.sin(3 * a)) * math.cos(a),
                        v + rad * (1 + 0.3 * math.sin(3 * a)) * 0.7 * math.sin(a))
                       for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
                cv2.fillPoly(img, [np.array(pts, np.int32)], (70, 80, 85), cv2.LINE_AA)

        # agua: neblina azul según la distancia, desenfoque y ruido analógico
        haze = min(0.55, 0.2 * hf)
        img = cv2.addWeighted(img, 1 - haze, np.full_like(img, (160, 120, 40)), haze, 0)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        noise = self.noise_rng.normal(0, 6, img.shape).astype(np.int16)
        return (img.astype(np.int16) + noise).clip(0, 255).astype(np.uint8)
