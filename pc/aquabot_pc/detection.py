"""Detección de objetos con visión clásica (OpenCV).

Objetos de la competencia (confirmado 22 oct): CUBOS y PIRÁMIDES NARANJAS. Desde arriba ambos se ven como
un cuadrado naranja (la pirámide con caras de distinto brillo), así que el detector principal es por COLOR
NARANJA con filtros de forma suaves (no depende del tamaño ni de la perspectiva). Respaldo: GENÉRICO,
cualquier mancha que se separa del fondo (por si el color cambia mucho bajo el agua). Círculos tipo moneda:
apagado.

La visión PROPONE candidatos; el piloto CONFIRMA (botón A / barra espaciadora). Los umbrales se
ajustan en vivo (pestaña Visión) porque el color bajo el agua cambia con la luz y la profundidad.
"""
from __future__ import annotations

import copy
import itertools
from dataclasses import dataclass, field

import cv2
import numpy as np

DEFAULT_CFG: dict = {
    "ancho_trabajo": 640,
    "cuadros_estable": 6,  # cuadros seguidos para pasar de "visto" a "candidato"
    # naranja: rango amplio porque bajo el agua el naranja tira a amarillo/café y pierde saturación
    "cubo": {"activo": True, "h_min": 0, "h_max": 30, "s_min": 90, "v_min": 50, "area_min": 80,
             "solidez_min": 0.6, "aspecto_max": 3.0},
    "moneda": {"activo": False, "radio_min": 3, "radio_max": 40, "param2": 18, "contraste_min": 22},
    "generico": {"activo": True, "umbral": 25, "area_min": 60, "alargamiento_max": 4.0},
}

# Parámetros que se ajustan desde la pestaña Visión: (sección, clave, paso, mínimo, máximo).
# paso None = interruptor sí/no.
TUNABLE = [
    ("generico", "activo", None, 0, 1), ("generico", "umbral", 1, 5, 150), ("generico", "area_min", 10, 10, 50000),
    ("generico", "alargamiento_max", 0.2, 1.0, 20.0),
    ("cubo", "activo", None, 0, 1), ("cubo", "h_min", 1, 0, 179), ("cubo", "h_max", 1, 0, 179), ("cubo", "s_min", 5, 0, 255),
    ("cubo", "v_min", 5, 0, 255), ("cubo", "area_min", 20, 10, 20000), ("cubo", "solidez_min", 0.05, 0.3, 1.0),
    ("cubo", "aspecto_max", 0.1, 1.0, 6.0),
    ("moneda", "activo", None, 0, 1), ("moneda", "radio_min", 1, 2, 100), ("moneda", "radio_max", 1, 3, 200),
    ("moneda", "param2", 1, 5, 80), ("moneda", "contraste_min", 1, 0, 120),
    (None, "cuadros_estable", 1, 1, 50),
]

CLASSES = {
    # clase: (etiqueta, color BGR, prioridad)
    # Todas se muestran como "OBJETO": la clase solo indica qué detector lo encontró.
    "cubo": ("OBJETO NARANJA", (0, 140, 255), 3),
    "moneda": ("OBJETO", (0, 215, 255), 2),
    "generico": ("OBJETO", (255, 80, 255), 1),
}


def merge_cfg(user: dict | None) -> dict:
    cfg = copy.deepcopy(DEFAULT_CFG)
    for k, v in (user or {}).items():
        if isinstance(v, dict) and isinstance(cfg.get(k), dict):
            cfg[k].update(v)
        else:
            cfg[k] = v
    return cfg


@dataclass
class Detection:
    cls: str
    bbox: tuple[int, int, int, int]  # x, y, w, h en coordenadas del cuadro original
    contour: np.ndarray
    score: float


@dataclass
class Track:
    id: int
    cls: str
    bbox: tuple[int, int, int, int]
    contour: np.ndarray
    hits: int = 1
    misses: int = 0
    stable: bool = False
    extra: dict = field(default_factory=dict)

    @property
    def label(self) -> str:
        return CLASSES[self.cls][0]

    @property
    def color(self) -> tuple[int, int, int]:
        return CLASSES[self.cls][1]

    @property
    def priority(self) -> int:
        return CLASSES[self.cls][2]


def _iou(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


class Detector:
    def __init__(self, cfg: dict | None = None) -> None:
        self.cfg = merge_cfg(cfg)
        self.tracks: list[Track] = []
        self._ids = itertools.count(1)
        self.masks: dict[str, np.ndarray] = {}
        self.scale = 1.0

    # ----------------------------------------------------------------- detectores
    def _cube(self, hsv: np.ndarray) -> list[Detection]:
        c = self.cfg["cubo"]
        lo, hi = (c["h_min"], c["s_min"], c["v_min"]), (c["h_max"], 255, 255)
        if c["h_min"] <= c["h_max"]:
            mask = cv2.inRange(hsv, lo, hi)
        else:  # rango que cruza el rojo (p. ej. 170..10)
            mask = cv2.inRange(hsv, lo, (179, 255, 255)) | cv2.inRange(hsv, (0, c["s_min"], c["v_min"]), hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        self.masks["cubo"] = mask
        out = []
        for cnt in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            area = cv2.contourArea(cnt)
            if area < c["area_min"]:
                continue
            hull_area = cv2.contourArea(cv2.convexHull(cnt))
            if hull_area <= 0 or area / hull_area < c["solidez_min"]:
                continue
            (_, _), (w, h), _ = cv2.minAreaRect(cnt)
            if min(w, h) <= 0 or max(w, h) / min(w, h) > c["aspecto_max"]:
                continue
            out.append(self._make("cubo", cnt, area))
        return out

    def _coin(self, bgr: np.ndarray, lab: np.ndarray, taken: np.ndarray) -> list[Detection]:
        c = self.cfg["moneda"]
        gray = cv2.medianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), 5)
        rmin, rmax = int(c["radio_min"] * self.scale) or 2, int(c["radio_max"] * self.scale) or 3
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=2 * rmin + 4, param1=100,
                                   param2=c["param2"], minRadius=rmin, maxRadius=max(rmin + 1, rmax))
        out = []
        if circles is None:
            return out
        h, w = gray.shape
        for x, y, r in circles[0][:30]:
            xi, yi, ri = int(x), int(y), max(2, int(r))
            if xi - 2 * ri < 0 or yi - 2 * ri < 0 or xi + 2 * ri >= w or yi + 2 * ri >= h:
                continue
            if taken[yi, xi]:
                continue
            # contraste de color entre el disco y un anillo alrededor (en un recorte local)
            x0, y0 = xi - 2 * ri, yi - 2 * ri
            patch = lab[y0 : yi + 2 * ri + 1, x0 : xi + 2 * ri + 1]
            c0 = (xi - x0, yi - y0)
            inner = np.zeros(patch.shape[:2], np.uint8)
            cv2.circle(inner, c0, max(1, int(ri * 0.8)), 255, -1)
            ring = np.zeros(patch.shape[:2], np.uint8)
            cv2.circle(ring, c0, int(ri * 1.8), 255, -1)
            cv2.circle(ring, c0, int(ri * 1.3), 0, -1)
            m_in = np.array(cv2.mean(patch, inner)[:3])
            m_out = np.array(cv2.mean(patch, ring)[:3])
            contrast = float(np.linalg.norm(m_in - m_out))
            if contrast < c["contraste_min"]:
                continue
            cnt = cv2.ellipse2Poly((xi, yi), (ri, ri), 0, 0, 360, 20).reshape(-1, 1, 2)
            out.append(self._make("moneda", cnt, contrast))
        return out

    def _generic(self, lab: np.ndarray, taken: np.ndarray) -> list[Detection]:
        c = self.cfg["generico"]
        small = cv2.resize(lab, (64, 48), interpolation=cv2.INTER_AREA)
        bg = np.median(small.reshape(-1, 3), axis=0)  # color dominante del fondo
        d = lab.astype(np.float32) - bg.astype(np.float32)
        dist = np.sqrt(0.5 * d[..., 0] ** 2 + d[..., 1] ** 2 + d[..., 2] ** 2)  # L pesa menos (sombras)
        mask = (dist > c["umbral"]).astype(np.uint8) * 255
        mask[taken > 0] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        self.masks["generico"] = mask
        h, w = mask.shape
        out = []
        for cnt in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            area = cv2.contourArea(cnt)
            if area < c["area_min"]:
                continue
            (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
            if min(rw, rh) <= 0 or max(rw, rh) / min(rw, rh) > c["alargamiento_max"]:
                continue  # líneas del carril, bordes
            x, y, bw, bh = cv2.boundingRect(cnt)
            borders = (x <= 1) + (y <= 1) + (x + bw >= w - 1) + (y + bh >= h - 1)
            if borders >= 2:
                continue  # manchas que tocan dos bordes: viñeteo/paredes
            out.append(self._make("generico", cnt, area))
        return out

    def _make(self, cls: str, cnt: np.ndarray, score: float) -> Detection:
        inv = 1.0 / self.scale
        cnt_o = (cnt.astype(np.float32) * inv).astype(np.int32)
        return Detection(cls, cv2.boundingRect(cnt_o), cnt_o, float(score))

    # ----------------------------------------------------------------- API
    def process(self, frame: np.ndarray) -> list[Track]:
        h, w = frame.shape[:2]
        tw = int(self.cfg["ancho_trabajo"])
        self.scale = min(1.0, tw / w)
        small = cv2.resize(frame, (int(w * self.scale), int(h * self.scale))) if self.scale < 1 else frame
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)
        taken = np.zeros(small.shape[:2], np.uint8)

        dets: list[Detection] = []
        if self.cfg["cubo"]["activo"]:
            cubes = self._cube(hsv)
            dets += cubes
            for d in cubes:
                cv2.drawContours(taken, [(d.contour * self.scale).astype(np.int32)], -1, 255, -1)
                cv2.drawContours(taken, [(d.contour * self.scale).astype(np.int32)], -1, 255, 9)
        if self.cfg["moneda"]["activo"]:
            coins = self._coin(small, lab, taken)
            dets += coins
            for d in coins:
                cv2.drawContours(taken, [(d.contour * self.scale).astype(np.int32)], -1, 255, -1)
                cv2.drawContours(taken, [(d.contour * self.scale).astype(np.int32)], -1, 255, 9)
        if self.cfg["generico"]["activo"]:
            dets += self._generic(lab, taken)
        self._track(dets)
        return self.visible()

    def _track(self, dets: list[Detection]) -> None:
        used = set()
        for t in self.tracks:
            best, best_score = None, 0.0
            tx, ty, tw, th = t.bbox
            tcx, tcy = tx + tw / 2, ty + th / 2
            for i, d in enumerate(dets):
                if i in used:  # se asocia aunque cambie el detector (naranja <-> genérico)
                    continue
                x, y, w, h = d.bbox
                dist = np.hypot(x + w / 2 - tcx, y + h / 2 - tcy)
                score = max(_iou(t.bbox, d.bbox), 1.0 - dist / (0.6 * max(tw, th, w, h) + 25))
                if score > best_score:
                    best, best_score = i, score
            if best is not None and best_score > 0.2:
                d = dets[best]
                used.add(best)
                t.bbox, t.contour = d.bbox, d.contour
                if CLASSES[d.cls][2] > CLASSES[t.cls][2]:
                    t.cls = d.cls
                t.hits += 1
                t.misses = 0
            else:
                t.misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= 8]
        for i, d in enumerate(dets):
            if i not in used:
                self.tracks.append(Track(next(self._ids), d.cls, d.bbox, d.contour))
        need = int(self.cfg["cuadros_estable"])
        for t in self.tracks:
            if t.hits >= need and t.misses == 0:
                t.stable = True
            elif t.misses > 2:
                t.stable = False

    def visible(self) -> list[Track]:
        vis = [t for t in self.tracks if t.misses <= 2]
        return sorted(vis, key=lambda t: (t.stable, t.priority, t.hits), reverse=True)

    def best_candidate(self) -> Track | None:
        vis = self.visible()
        return vis[0] if vis else None

    def debug_mask(self, kind: str, shape: tuple[int, int]) -> np.ndarray | None:
        m = self.masks.get(kind)
        if m is None:
            return None
        return cv2.resize(m, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
