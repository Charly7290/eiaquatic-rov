"""Interfaz de piloteo EIAquatic (1600×960, se graba tal cual).

┌ EIAquatic | Piloto ROV ────────────────────────────────────────────────────────┐
│ ┌────────────── video 4:3 ──────────────┐  ┌ Detección ──────────────────────┐ │
│ │ banner de hallazgo / temperatura       │  │ Profundidad                     │ │
│ │ contornos · mira · mensajes            │  │ Temp. agua │ Batería            │ │
│ │                                        │  │ Actitud · cabeceo               │ │
│ └────────────────────────────────────────┘  └ Lastre ─────────────────────────┘ │
│ reloj ms │ REC │ ARMADO │ POT │ ENLACE │ avisos                MOTORES M1…M5     │
└─────────────────────────────────────────────────────────────────────────────────┘
Contornos: visto = línea delgada blanca + etiqueta pequeña · candidato = amarillo · confirmado = rojo.
"""
from __future__ import annotations

import math
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pygame

from . import protocol as P
from .detection import TUNABLE

# ------------------------------------------------------------------ geometría
W, H = 1600, 960
HDR, FTR, M, G = 56, 68, 16, 10          # alto cabecera, alto barra inferior, margen, separación
RPW = 440                                 # ancho del panel derecho
RPX = W - M - RPW                         # x del panel derecho
TOP, BOT = HDR + 12, H - FTR - 12         # zona útil vertical
_avail_w = RPX - 2 * M
VH = BOT - TOP
VW = min(_avail_w, VH * 4 // 3)
VX = M + (_avail_w - VW) // 2
VY = TOP
VIDEO = pygame.Rect(VX, VY, VW, VH)

# ------------------------------------------------------------------ colores
BG = (9, 19, 30)
BAR = (8, 16, 26)
CARD = (14, 29, 44)
CARD_BORDER = (29, 49, 68)
TRACK = (37, 55, 74)
LINE = (40, 62, 84)
TEXT = (232, 240, 247)
MUTED = (134, 155, 176)
CYAN = (34, 197, 230)
GREEN = (34, 197, 94)
YELLOW = (234, 179, 8)
RED = (239, 68, 68)
BLUE = (59, 130, 246)
ORANGE = (249, 115, 22)
SKY = (47, 128, 214)
GROUND = (168, 105, 46)

FONT_DIR = Path(__file__).resolve().parent / "assets" / "fonts"
_NO_GLYPH = set("✓▲▼■↑↓←→")


def mix(a, b, t: float):
    """Mezcla de colores: t=0 -> a, t=1 -> b."""
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def clock_str() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class UI:
    def __init__(self, screen: pygame.Surface) -> None:
        self.screen = screen
        self._fonts: dict = {}
        self._icons: dict = {}
        self._mask = self._video_mask()
        self._lq: list[tuple[float, int, int]] = []  # (t, tx, timeouts) para la pérdida del enlace
        self.hits: list[tuple[pygame.Rect, str]] = []  # zonas clicables del último cuadro -> acción

    def hit(self, pos) -> str | None:
        """Acción del botón/pestaña bajo el puntero (la última dibujada gana)."""
        for r, act in reversed(self.hits):
            if r.collidepoint(pos):
                return act
        return None

    def button(self, label: str, rect: pygame.Rect, action: str, active: bool = False,
               hint: str = "", size: int = 15) -> pygame.Rect:
        hover = rect.collidepoint(pygame.mouse.get_pos())
        if active:
            self.rrect(rect, CYAN, rect.h // 2)
            col = (8, 16, 26)
        else:
            self.rrect(rect, mix(CARD, CYAN, 0.18) if hover else CARD, rect.h // 2)
            self.rrect(rect, CYAN if hover else LINE, rect.h // 2, 1)
            col = TEXT if hover else MUTED
        if hint:
            f, fh = self.font(size, "SemiBold"), self.font(12, "Medium")
            tw, hw = f.size(label)[0], fh.size(hint)[0]
            x = rect.centerx - (tw + 8 + hw) // 2
            self.text(label, (x, rect.centery), size, "SemiBold", col, "midleft")
            self.text(hint, (x + tw + 8, rect.centery + 1), 12, "Medium",
                      (40, 60, 80) if active else mix(col, CARD, 0.35), "midleft")
        else:
            self.text(label, rect.center, size, "SemiBold", col, "center")
        self.hits.append((rect, action))
        return rect

    def tab_width(self, label: str, hint: str, size: int = 15) -> int:
        return self.font(size, "SemiBold").size(label)[0] + 8 + self.font(12, "Medium").size(hint)[0] + 32

    # ================================================================== utilidades
    def font(self, size: int, weight: str = "Regular") -> pygame.font.Font:
        key = (size, weight)
        if key not in self._fonts:
            path = FONT_DIR / f"Barlow-{weight}.ttf"
            if path.exists():
                self._fonts[key] = pygame.font.Font(str(path), size)
            else:  # respaldo si faltan los archivos
                self._fonts[key] = pygame.font.SysFont("dejavusans,arial", size,
                                                       bold=weight in ("SemiBold", "Bold"))
        return self._fonts[key]

    def text(self, s: str, pos, size: int = 16, weight: str = "Regular", color=TEXT,
             anchor: str = "topleft") -> pygame.Rect:
        f = self.font(size, weight)
        if any(ch in _NO_GLYPH for ch in s):  # Barlow no trae flechas ni ✓: se usa DejaVu
            key = ("fallback", size, weight)
            if key not in self._fonts:
                self._fonts[key] = pygame.font.SysFont("dejavusans,arial", size, bold=weight in ("SemiBold", "Bold"))
            f = self._fonts[key]
        surf = f.render(s, True, color)
        r = surf.get_rect(**{anchor: pos})
        self.screen.blit(surf, r)
        return r

    def mono(self, s: str, pos, size: int, weight: str, color) -> pygame.Rect:
        """Texto con dígitos de ancho fijo (el reloj no 'baila')."""
        f = self.font(size, weight)
        cw = f.size("0")[0]
        x, y = pos
        x0 = x
        for ch in s:
            g = f.render(ch, True, color)
            w = cw if ch.isdigit() else g.get_width()
            self.screen.blit(g, (x + (w - g.get_width()) // 2, y))
            x += w
        return pygame.Rect(x0, y, x - x0, f.get_height())

    def rrect(self, rect, color, radius: int = 10, width: int = 0) -> None:
        pygame.draw.rect(self.screen, color, rect, width, border_radius=radius)

    def chip(self, s: str, pos: tuple, color, align: str = "left", filled: bool = False) -> pygame.Rect:
        f = self.font(14, "SemiBold")
        tw = f.size(s)[0]
        w, h = tw + 24, 28
        x, y = pos
        r = pygame.Rect(x - w if align == "right" else x, y, w, h)
        if filled:
            self.rrect(r, color, h // 2)
            self.text(s, r.center, 14, "SemiBold", (10, 14, 20), "center")
        else:
            self.rrect(r, mix(CARD, color, 0.16), h // 2)
            self.rrect(r, mix(CARD, color, 0.7), h // 2, 1)
            self.text(s, r.center, 14, "SemiBold", color, "center")
        return r

    def card(self, rect: pygame.Rect, icon: str, title: str) -> int:
        self.rrect(rect, CARD, 12)
        self.rrect(rect, CARD_BORDER, 12, 1)
        self.icon(icon, (rect.x + 16, rect.y + 15), 22, CYAN)
        self.text(title, (rect.x + 48, rect.y + 26), 16, "SemiBold", TEXT, "midleft")
        return rect.y + 48

    def _video_mask(self) -> pygame.Surface:
        """Capa que redondea las esquinas del video (fondo opaco con hueco transparente)."""
        m = pygame.Surface(VIDEO.size, pygame.SRCALPHA)
        m.fill((*BG, 255))
        pygame.draw.rect(m, (0, 0, 0, 0), m.get_rect(), border_radius=12)
        return m

    # ================================================================== iconos
    def icon(self, name: str, pos, size: int, color) -> None:
        key = (name, size, color)
        if key not in self._icons:
            s = 4  # se dibuja a 4x y se reduce: bordes suaves
            n = size * s
            surf = pygame.Surface((n, n), pygame.SRCALPHA)
            getattr(self, "_ic_" + name)(surf, n, color, max(2, int(n * 0.085)))
            self._icons[key] = pygame.transform.smoothscale(surf, (size, size))
        self.screen.blit(self._icons[key], pos)

    @staticmethod
    def _wave(surf, n, color, lw, y, amp, periods=1.5, x0=0.08, x1=0.92):
        pts = [(n * (x0 + (x1 - x0) * i / 40), y + amp * math.sin(2 * math.pi * periods * i / 40)) for i in range(41)]
        pygame.draw.lines(surf, color, False, pts, lw)

    def _ic_waves(self, surf, n, c, lw):
        for yy in (0.3, 0.5, 0.7):
            self._wave(surf, n, c, lw, n * yy, n * 0.06)

    def _ic_logo(self, surf, n, c, lw):
        for yy in (0.32, 0.52, 0.72):
            self._wave(surf, n, c, int(lw * 1.6), n * yy, n * 0.08, 1.25)

    def _ic_target(self, surf, n, c, lw):
        cx = cy = n // 2
        pygame.draw.circle(surf, c, (cx, cy), int(n * 0.32), lw)
        pygame.draw.circle(surf, c, (cx, cy), int(n * 0.08))
        for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
            pygame.draw.line(surf, c, (cx + dx * n * 0.36, cy + dy * n * 0.36),
                             (cx + dx * n * 0.48, cy + dy * n * 0.48), lw)

    def _ic_thermo(self, surf, n, c, lw):
        pygame.draw.rect(surf, c, (n * 0.4, n * 0.06, n * 0.2, n * 0.6), lw, border_radius=int(n * 0.1))
        pygame.draw.circle(surf, c, (n // 2, int(n * 0.76)), int(n * 0.16), lw)
        pygame.draw.circle(surf, c, (n // 2, int(n * 0.76)), int(n * 0.08))
        pygame.draw.line(surf, c, (n * 0.5, n * 0.35), (n * 0.5, n * 0.68), lw)

    def _ic_battery(self, surf, n, c, lw):
        pygame.draw.rect(surf, c, (n * 0.06, n * 0.28, n * 0.78, n * 0.44), lw, border_radius=int(n * 0.06))
        pygame.draw.rect(surf, c, (n * 0.86, n * 0.4, n * 0.08, n * 0.2))
        pygame.draw.rect(surf, c, (n * 0.16, n * 0.38, n * 0.3, n * 0.24))

    def _ic_attitude(self, surf, n, c, lw):
        cx = cy = n // 2
        pygame.draw.circle(surf, c, (cx, cy), int(n * 0.4), lw)
        pygame.draw.line(surf, c, (n * 0.18, n * 0.56), (n * 0.82, n * 0.44), lw)
        pygame.draw.circle(surf, c, (cx, cy), int(n * 0.07))

    def _ic_drop(self, surf, n, c, lw):
        cx = n / 2
        arc = [(cx + n * 0.27 * math.cos(math.radians(i)), n * 0.62 + n * 0.27 * math.sin(math.radians(i)))
               for i in range(0, 181, 10)]
        pygame.draw.polygon(surf, c, [(cx, n * 0.08)] + arc, lw)

    def _ic_gauge(self, surf, n, c, lw):
        r = pygame.Rect(n * 0.08, n * 0.18, n * 0.84, n * 0.84)
        pygame.draw.arc(surf, c, r, 0, math.pi, lw)
        cx, cy = r.centerx, r.centery
        pygame.draw.line(surf, c, (cx, cy), (cx + n * 0.24, cy - n * 0.22), lw)
        pygame.draw.circle(surf, c, (cx, cy), int(n * 0.07))

    def _ic_clock(self, surf, n, c, lw):
        cx = cy = n // 2
        pygame.draw.circle(surf, c, (cx, cy), int(n * 0.42), lw)
        pygame.draw.line(surf, c, (cx, cy), (cx, cy - n * 0.26), lw)
        pygame.draw.line(surf, c, (cx, cy), (cx + n * 0.18, cy + n * 0.08), lw)

    def _ic_shield(self, surf, n, c, lw):
        pts = [(n * 0.5, n * 0.06), (n * 0.86, n * 0.2), (n * 0.82, n * 0.56), (n * 0.5, n * 0.94),
               (n * 0.18, n * 0.56), (n * 0.14, n * 0.2)]
        pygame.draw.polygon(surf, c, pts, lw)
        pygame.draw.lines(surf, c, False, [(n * 0.34, n * 0.5), (n * 0.46, n * 0.62), (n * 0.68, n * 0.38)], lw)

    def _ic_bolt(self, surf, n, c, lw):
        pts = [(n * 0.58, n * 0.04), (n * 0.2, n * 0.56), (n * 0.48, n * 0.56), (n * 0.4, n * 0.96),
               (n * 0.8, n * 0.42), (n * 0.52, n * 0.42)]
        pygame.draw.polygon(surf, c, pts, lw)

    def _ic_signal(self, surf, n, c, lw):
        for i, hh in enumerate((0.3, 0.55, 0.8)):
            x = n * (0.14 + i * 0.28)
            pygame.draw.rect(surf, c, (x, n * (0.9 - hh), n * 0.18, n * hh), border_radius=int(n * 0.04))

    def _ic_image(self, surf, n, c, lw):
        pygame.draw.rect(surf, c, (n * 0.08, n * 0.16, n * 0.84, n * 0.68), lw, border_radius=int(n * 0.08))
        pygame.draw.lines(surf, c, False, [(n * 0.14, n * 0.76), (n * 0.38, n * 0.48), (n * 0.56, n * 0.66),
                                           (n * 0.68, n * 0.54), (n * 0.86, n * 0.74)], lw)
        pygame.draw.circle(surf, c, (int(n * 0.66), int(n * 0.36)), int(n * 0.07), lw)

    def _ic_check(self, surf, n, c, lw):
        pygame.draw.lines(surf, c, False, [(n * 0.14, n * 0.52), (n * 0.4, n * 0.78), (n * 0.88, n * 0.24)],
                          int(lw * 1.5))

    def _ic_up(self, surf, n, c, lw):
        pygame.draw.line(surf, c, (n / 2, n * 0.15), (n / 2, n * 0.88), lw)
        pygame.draw.lines(surf, c, False, [(n * 0.22, n * 0.42), (n / 2, n * 0.15), (n * 0.78, n * 0.42)], lw)

    def _ic_down(self, surf, n, c, lw):
        pygame.draw.line(surf, c, (n / 2, n * 0.12), (n / 2, n * 0.85), lw)
        pygame.draw.lines(surf, c, False, [(n * 0.22, n * 0.58), (n / 2, n * 0.85), (n * 0.78, n * 0.58)], lw)

    # ================================================================== dibujo
    def draw(self, app) -> None:
        self.hits = []
        self.screen.fill(BG)
        self._header(app)
        self._video(app)
        self._right_panel(app)
        self._footer(app)
        if app.panel == "params":
            self._params_panel(app)
        elif app.panel == "vision":
            self._vision_panel(app)
        elif app.panel == "ayuda":
            self._help_panel(app)
        self._banner_and_toast(app)

    # ------------------------------------------------------------------ cabecera
    def _header(self, app) -> None:
        pygame.draw.rect(self.screen, BAR, (0, 0, W, HDR))
        pygame.draw.line(self.screen, CARD_BORDER, (0, HDR - 1), (W, HDR - 1))
        self.icon("logo", (M + 4, 12), 32, CYAN)
        r = self.text("EIAquatic", (M + 48, HDR // 2), 26, "Bold", TEXT, "midleft")
        pygame.draw.line(self.screen, LINE, (r.right + 18, 16), (r.right + 18, HDR - 16), 1)
        self.text("Piloto ROV", (r.right + 36, HDR // 2), 18, "Regular", MUTED, "midleft")
        on = app.vision_on
        lbl, hint = ("Detección ON", "V") if on else ("Detección OFF", "V")
        bw = self.tab_width(lbl, hint)
        self.button(lbl, pygame.Rect(r.right + 170, 11, bw, HDR - 22), "vision", on, hint)
        x = W - M
        if app.sim is not None:
            x = self.chip("SIMULADOR", (W - M, 14), YELLOW, "right").x - 20
        # Pestañas (clic con el mouse o la tecla indicada). "Piloto" cierra los paneles.
        tabs = [("Piloto", "", "panel_cerrar", app.panel is None),
                ("Parámetros ROV", "F2", "panel_params", app.panel == "params"),
                ("Visión", "F3", "panel_vision", app.panel == "vision"),
                ("Controles", "F1", "panel_ayuda", app.panel == "ayuda")]
        widths = [self.tab_width(lbl, hint) for lbl, hint, _, _ in tabs]
        x -= sum(widths) + 8 * (len(tabs) - 1)
        for (lbl, hint, act, on), w in zip(tabs, widths):
            self.button(lbl, pygame.Rect(x, 11, w, HDR - 22), act, on, hint)
            x += w + 8

    # ------------------------------------------------------------------ video
    def _video(self, app) -> None:
        frame = app.frame
        if frame is None:
            self.rrect(VIDEO, (5, 10, 16), 12)
            self.icon("image", (VIDEO.centerx - 32, VIDEO.centery - 70), 64, MUTED)
            self.text("SIN VIDEO", (VIDEO.centerx, VIDEO.centery + 14), 34, "SemiBold", MUTED, "center")
            self.text(app.video_status, (VIDEO.centerx, VIDEO.centery + 50), 18, "Regular", MUTED, "center")
            self.rrect(VIDEO, CARD_BORDER, 12, 1)
            return
        fh, fw = frame.shape[:2]
        sx, sy = VW / fw, VH / fh
        vis = cv2.resize(frame, (VW, VH), interpolation=cv2.INTER_LINEAR)

        if app.panel == "vision" and app.mask_kind:
            m = app.detector.debug_mask(app.mask_kind, (VH, VW))
            if m is not None:
                tint = np.zeros_like(vis)
                tint[:] = (255, 0, 255) if app.mask_kind == "generico" else (0, 0, 255)
                vis = np.where(m[..., None] > 0, cv2.addWeighted(vis, 0.4, tint, 0.6, 0), vis)

        labels = []
        for t in app.tracks:
            cnt = (t.contour.astype(np.float32) * (sx, sy)).astype(np.int32)
            confirmed = t.id in app.confirmed_ids
            x, y, w, h = cv2.boundingRect(cnt)
            if confirmed:
                color, thick = (68, 68, 239), 4            # rojo (BGR)
            elif t.stable:
                color, thick = (8, 179, 234), 3            # amarillo = candidato
            else:
                color, thick = (235, 235, 235), 1          # visto: blanco delgado, igual para todo
            cv2.drawContours(vis, [cnt], -1, color, thick, cv2.LINE_AA)
            if t.stable or confirmed:
                cv2.rectangle(vis, (x - 5, y - 5), (x + w + 5, y + h + 5), color, 1, cv2.LINE_AA)
            labels.append((t, x, y, confirmed))

        cx, cy = VW // 2, VH // 2  # mira
        for a, b in (((-22, 0), (-7, 0)), ((7, 0), (22, 0)), ((0, -22), (0, -7)), ((0, 7), (0, 22))):
            cv2.line(vis, (cx + a[0], cy + a[1]), (cx + b[0], cy + b[1]), (240, 240, 240), 2, cv2.LINE_AA)

        rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        self.screen.blit(pygame.image.frombuffer(rgb.tobytes(), (VW, VH), "RGB"), VIDEO.topleft)
        for t, x, y, confirmed in labels:
            x += VX
            y += VY
            if confirmed or t.stable:
                col = RED if confirmed else YELLOW
                fg = (255, 255, 255) if confirmed else (10, 14, 20)
                surf = self.font(15, "Bold").render(t.label, True, fg)
                iw = 20 if confirmed else 0
                r = surf.get_rect(bottomleft=(x - 5 + iw, max(VY + 24, y - 8)))
                bg = pygame.Rect(r.x - iw, r.y, r.w + iw, r.h).inflate(12, 6)
                self.rrect(bg, col, 4)
                if confirmed:
                    self.icon("check", (r.x - iw, r.centery - 8), 16, fg)
                self.screen.blit(surf, r)
            else:  # etiqueta pequeña y discreta
                surf = self.font(13, "SemiBold").render(t.label.lower(), True, (240, 240, 240))
                r = surf.get_rect(bottomleft=(x, max(VY + 18, y - 5)))
                bg = pygame.Surface(r.inflate(10, 4).size, pygame.SRCALPHA)
                bg.fill((0, 0, 0, 150))
                self.screen.blit(bg, r.inflate(10, 4))
                self.screen.blit(surf, r)
        self.screen.blit(self._mask, VIDEO.topleft)
        self.rrect(VIDEO, CARD_BORDER, 12, 1)

    # ------------------------------------------------------------------ panel derecho
    @staticmethod
    def _mode(requested: bool, active: bool, waiting_for: str):
        """Modo pedido por el piloto vs. confirmado por el ROV."""
        if requested and active:
            return "AUTO", GREEN
        if requested:
            return f"AUTO · espera {waiting_for}", YELLOW
        return "MANUAL", MUTED

    def _right_panel(self, app) -> None:
        x, w = RPX, RPW
        t: P.Telemetry | None = app.link.telemetry
        stale = t is None or not app.link.ok

        def dim(c):
            return MUTED if stale else c

        heights = [216, 128, 112, 158]
        heights.append(BOT - TOP - sum(heights) - G * 4)
        ys = [TOP]
        for hh in heights[:-1]:
            ys.append(ys[-1] + hh + G)

        # ---------------------------------------------------------- DETECCIÓN
        rect = pygame.Rect(x, ys[0], w, heights[0])
        cy = self.card(rect, "target", "DETECCIÓN")
        n = app.session.n_detections
        self.text(f"{n} hallazgo{'s' if n != 1 else ''}", (rect.right - 16, rect.y + 26), 16, "Medium",
                  RED if n else MUTED, "midright")
        roi, frozen = app.roi_display()
        best = app.detector.best_candidate()
        if frozen:
            state, scol = "DETECTADO", RED
        elif not app.vision_on:
            state, scol = "APAGADA", MUTED
        elif best is not None and best.stable:
            state, scol = "CANDIDATO", YELLOW
        else:
            state, scol = "BUSCANDO", CYAN
        self.chip(state, (rect.x + 172, rect.y + 12), scol)

        box = pygame.Rect(rect.x + 14, cy, w - 28, heights[0] - 48 - 40)
        self.rrect(box, (7, 16, 25), 8)
        if roi is not None and roi.size:
            rh, rw = roi.shape[:2]
            k = min((box.w - 6) / rw, (box.h - 6) / rh)
            roi_r = cv2.resize(roi, (max(1, int(rw * k)), max(1, int(rh * k))),
                               interpolation=cv2.INTER_NEAREST if k > 3 else cv2.INTER_LINEAR)
            rgb = cv2.cvtColor(roi_r, cv2.COLOR_BGR2RGB)
            surf = pygame.image.frombuffer(rgb.tobytes(), (rgb.shape[1], rgb.shape[0]), "RGB")
            self.screen.blit(surf, surf.get_rect(center=box.center))
        else:
            self.icon("image", (box.centerx - 18, box.centery - 30), 36, MUTED)
            msg = "Sin objeto a la vista" if app.vision_on else "Detección apagada · solo video"
            self.text(msg, (box.centerx, box.centery + 20), 16, "Regular", MUTED, "center")
        self.rrect(box, RED if frozen else CARD_BORDER, 8, 3 if frozen else 1)
        ly = box.bottom + 20
        if frozen:
            info = app.last_confirm
            self.text(f"#{info['k']}  {info['label']}", (rect.x + 16, ly), 16, "SemiBold", RED, "midleft")
            self.text(info["hora"], (rect.right - 16, ly), 16, "Medium", TEXT, "midright")
        elif best is not None and best.stable:
            self.text(best.label, (rect.x + 16, ly), 16, "SemiBold", YELLOW, "midleft")
            self.text("pulse A para confirmar", (rect.right - 16, ly), 15, "Regular", TEXT, "midright")
        elif not app.vision_on:
            self.text("A marca un hallazgo manual · V reactiva", (rect.x + 16, ly), 15, "Regular", MUTED, "midleft")
        else:
            self.text("la visión no ve nada estable", (rect.x + 16, ly), 15, "Regular", MUTED, "midleft")

        if t is None:
            r = pygame.Rect(x, ys[1], w, BOT - ys[1])
            self.rrect(r, CARD, 12)
            self.rrect(r, CARD_BORDER, 12, 1)
            self.text("Esperando al ROV…", r.center, 26, "SemiBold", MUTED, "center")
            self.text("sin telemetría", (r.centerx, r.centery + 30), 16, "Regular", MUTED, "center")
            return

        # ---------------------------------------------------------- PROFUNDIDAD
        rect = pygame.Rect(x, ys[1], w, heights[1])
        self.card(rect, "waves", "PROFUNDIDAD")
        split = rect.x + 262
        pygame.draw.line(self.screen, LINE, (split, rect.y + 18), (split, rect.bottom - 18))
        if t.has(P.TF_DEPTH_OK):
            r = self.text(f"{t.depth:.2f}", (rect.x + 14, rect.y + 28), 58, "SemiBold", dim(TEXT))
            self.text("m", (r.right + 6, r.bottom - 16), 28, "Regular", dim(MUTED), "bottomleft")
            self.text(f"presión  {t.pressure_mbar} mbar", (rect.x + 16, rect.bottom - 16), 15, "Medium",
                      MUTED, "bottomleft")
        else:
            self.text("sin sensor", (rect.x + 16, rect.y + 60), 30, "SemiBold", RED)
        hold = t.has(P.TF_DEPTH_HOLD)
        lbl, col = self._mode(app.depth_hold, hold, "sensor")
        mcol = TEXT if col == MUTED else col
        rx = split + 18
        self.icon("gauge", (rx, rect.y + 17), 22, mcol)
        self.text("AUTO…" if len(lbl) > 12 else lbl, (rx + 30, rect.y + 28), 16, "SemiBold", mcol, "midleft")
        vx0 = rx
        if abs(t.vz) > 0.02:  # flecha: bajando / subiendo
            self.icon("down" if t.vz > 0 else "up", (rx, rect.y + 60), 20, dim(CYAN))
            vx0 += 24
        r = self.text(f"{abs(t.vz):.2f}", (vx0, rect.y + 50), 32, "Medium", dim(TEXT))
        self.text("m/s", (r.right + 6, r.bottom - 7), 18, "Regular", dim(MUTED), "bottomleft")
        if app.depth_target is not None:
            sp, scol = f"obj  {app.depth_target:.2f} m", dim(GREEN) if hold else YELLOW
        elif hold:
            sp, scol = f"obj  {t.depth_sp:.2f} m", dim(GREEN)
        else:
            sp, scol = "obj  —", MUTED
        self.text(sp, (rx, rect.bottom - 16), 15, "Medium", scol, "bottomleft")
        # botones de objetivo fijo: subir / bajar 10 cm (también + / − o la cruceta)
        bw, bh = 26, 22
        for i, (ic, act) in enumerate((("up", "objetivo_menos"), ("down", "objetivo_mas"))):
            br = pygame.Rect(rect.right - 14 - (2 - i) * (bw + 4) + 4, rect.bottom - 14 - bh, bw, bh)
            self.button("", br, act)
            hov = br.collidepoint(pygame.mouse.get_pos())
            self.icon(ic, (br.centerx - 7, br.centery - 7), 14, TEXT if hov else MUTED)

        # ---------------------------------------------------------- TEMP + BATERÍA
        hw = (w - G) // 2
        rect = pygame.Rect(x, ys[2], hw, heights[2])
        self.card(rect, "thermo", "TEMP. AGUA")
        if t.water_temp is not None:
            r = self.text(f"{t.water_temp:.2f}", (rect.x + 16, rect.y + 42), 44, "SemiBold", dim(CYAN))
            self.text("°C", (r.right + 6, r.bottom - 9), 20, "Regular", dim(MUTED), "bottomleft")
        else:
            self.text("sin sensor", (rect.x + 16, rect.y + 58), 24, "SemiBold", RED)
        rect = pygame.Rect(x + hw + G, ys[2], hw, heights[2])
        self.card(rect, "battery", "BATERÍA")
        vcol = RED if t.has(P.TF_LOWBAT_CRIT) else YELLOW if t.has(P.TF_LOWBAT_WARN) else GREEN
        r = self.text(f"{t.vbat:.2f}", (rect.x + 16, rect.y + 36), 44, "SemiBold", dim(vcol))
        self.text("V", (r.right + 6, r.bottom - 9), 20, "Regular", dim(MUTED), "bottomleft")
        bar = pygame.Rect(rect.x + 16, rect.bottom - 16, rect.w - 32, 6)
        self.rrect(bar, TRACK, 4)
        frac = max(0.0, min(1.0, (t.vbat - 10.5) / (12.6 - 10.5)))
        if frac > 0:
            self.rrect(pygame.Rect(bar.x, bar.y, int(bar.w * frac), bar.h), dim(vcol), 4)

        # ---------------------------------------------------------- ACTITUD
        rect = pygame.Rect(x, ys[3], w, heights[3])
        self.card(rect, "attitude", "ACTITUD · CABECEO")
        phold = t.has(P.TF_PITCH_HOLD)
        lbl, col = self._mode(app.pitch_hold, phold, "armar")
        self.chip(lbl, (rect.right - 14, rect.y + 12), col, "right")
        r_h = 50
        hc = (rect.x + 16 + r_h, rect.y + 48 + r_h)
        self._horizon(hc, r_h, t.pitch, t.roll, stale or not t.has(P.TF_IMU_OK))
        split = rect.x + 16 + 2 * r_h + 22
        pygame.draw.line(self.screen, LINE, (split, rect.y + 52), (split, rect.bottom - 14))
        vx = split + 20
        self.text("CABECEO", (vx, rect.y + 50), 14, "SemiBold", MUTED)
        r = self.text(f"{t.pitch:+.1f}°", (vx, rect.y + 64), 34, "SemiBold", dim(TEXT))
        if phold:
            self.text(f"objetivo {t.pitch_sp:+.0f}°", (r.right + 14, r.centery), 15, "Medium", dim(GREEN), "midleft")
        self.text("ALABEO", (vx, rect.y + 104), 14, "SemiBold", MUTED)
        self.text(f"{t.roll:+.1f}°", (vx, rect.y + 118), 34, "SemiBold", dim(TEXT))

        # ---------------------------------------------------------- LASTRE
        rect = pygame.Rect(x, ys[4], w, heights[4])
        self.card(rect, "drop", "LASTRE")
        if t.has(P.TF_BALLAST_FAULT):
            state, scol = "FALLA", RED
        else:
            state, scol = {1: ("LLENANDO", CYAN), -1: ("VACIANDO", CYAN), 0: ("QUIETO", MUTED)}[
                max(-1, min(1, t.ballast_dir))]
        self.chip(state, (rect.right - 14, rect.y + 12), scol, "right")
        bx, by, bw = rect.x + 16, rect.y + 58, rect.w - 32
        nseg, gap = 12, 4
        sw = (bw - gap * (nseg - 1)) / nseg
        pos = t.ballast_pos
        for i in range(nseg):
            on = pos is not None and (i + 0.5) / nseg <= pos
            self.rrect(pygame.Rect(int(bx + i * (sw + gap)), by, int(sw), 8), dim(CYAN) if on else TRACK, 3)
        if pos is not None:
            mx = bx + int(bw * pos)
            pygame.draw.rect(self.screen, TEXT, (mx - 2, by - 5, 4, 18), border_radius=2)
        ly = by + 26
        self.text("Vacío · sube", (bx, ly), 15, "Medium", MUTED, "midleft")
        self.text("Lleno · baja", (bx + bw, ly), 15, "Medium", MUTED, "midright")
        if pos is not None:
            self.text(f"{pos * 100:.0f} %", (bx + bw // 2, ly), 17, "SemiBold", dim(CYAN), "center")
        else:
            self.text("posición ? · toque un tope", (bx + bw // 2, ly), 15, "Medium", YELLOW, "center")
        bw2 = (bw - G) // 2
        for i, (name, flag, ic) in enumerate((("TOPE VACÍO", P.TF_LIM_EMPTY, "up"),
                                              ("TOPE LLENO", P.TF_LIM_FULL, "down"))):
            on = t.has(flag)
            b = pygame.Rect(bx + i * (bw2 + G), rect.bottom - 54, bw2, 40)
            if on:
                self.rrect(b, CYAN, 8)
                fg = (8, 20, 30)
            else:
                self.rrect(b, (11, 24, 37), 8)
                self.rrect(b, CARD_BORDER, 8, 1)
                fg = MUTED
            tw = self.font(15, "SemiBold").size(name)[0]
            ix = b.centerx - (tw + 26) // 2
            self.icon(ic, (ix, b.centery - 9), 18, fg)
            self.text(name, (ix + 26, b.centery), 15, "SemiBold", fg, "midleft")

    def _horizon(self, c, r: int, pitch: float, roll: float, dim: bool) -> None:
        """Actitud vista desde ATRÁS del ROV: el horizonte queda fijo y el símbolo amarillo del ROV se
        inclina con él (lado derecho abajo -> el símbolo baja a la derecha; nariz arriba -> sube)."""
        s = 3  # supersampling para bordes suaves
        n = 2 * r * s
        surf = pygame.Surface((n, n), pygame.SRCALPHA)
        sky, ground = ((60, 72, 84), (82, 82, 82)) if dim else (SKY, GROUND)
        surf.fill(sky)
        pygame.draw.rect(surf, ground, (0, n // 2, n, n // 2))
        ppd = r * s / 40.0  # píxeles por grado
        white = (235, 240, 245)
        pygame.draw.line(surf, white, (0, n // 2), (n, n // 2), s * 2)
        for deg in (-20, -10, 10, 20):  # escala de cabeceo (fija)
            yy = n / 2 - deg * ppd
            hw = (0.16 if abs(deg) == 10 else 0.26) * r * s
            pygame.draw.line(surf, white, (n / 2 - hw, yy), (n / 2 + hw, yy), s)
        # símbolo del ROV: alas + punto, rotado con el alabeo y desplazado con el cabeceo
        off = max(-0.8 * r * s, min(0.8 * r * s, pitch * ppd))
        a = math.radians(roll)
        ca, sa = math.cos(a), math.sin(a)
        cx, cy = n / 2, n / 2 - off

        def rot(px, py):
            return (cx + px * ca - py * sa, cy + px * sa + py * ca)

        col = (150, 150, 150) if dim else YELLOW
        w1, w0 = 0.62 * r * s, 0.12 * r * s
        pygame.draw.line(surf, col, rot(-w1, 0), rot(-w0, 0), s * 3)
        pygame.draw.line(surf, col, rot(w0, 0), rot(w1, 0), s * 3)
        pygame.draw.line(surf, col, rot(-w0, 0), rot(0, w0 * 0.8), s * 3)
        pygame.draw.line(surf, col, rot(w0, 0), rot(0, w0 * 0.8), s * 3)
        pygame.draw.line(surf, col, rot(0, 0), rot(0, -0.22 * r * s), s * 2)  # "techo" del ROV
        mask = pygame.Surface((n, n), pygame.SRCALPHA)
        pygame.draw.circle(mask, (255, 255, 255, 255), (n // 2, n // 2), n // 2)
        surf.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        surf = pygame.transform.smoothscale(surf, (2 * r, 2 * r))
        self.screen.blit(surf, (c[0] - r, c[1] - r))
        pygame.draw.circle(self.screen, CARD_BORDER, c, r + 1, 2)

    # ------------------------------------------------------------------ barra inferior
    def pill(self, s: str, x: int, y: int, color, icon: str | None = None, filled: bool = False) -> int:
        tw = self.font(16, "SemiBold").size(s)[0]
        iw = 26 if icon else 0
        r = pygame.Rect(x, y, tw + iw + 32, 40)
        if filled:
            self.rrect(r, color, 20)
        else:
            self.rrect(r, mix(BAR, color, 0.22), 20)
            self.rrect(r, mix(BAR, color, 0.75), 20, 1)
        if icon:
            self.icon(icon, (x + 16, r.centery - 10), 20, TEXT)
        self.text(s, (x + 16 + iw, r.centery), 16, "SemiBold", TEXT, "midleft")
        return r.right + 10

    def _footer(self, app) -> None:
        y0 = H - FTR
        pygame.draw.rect(self.screen, BAR, (0, y0, W, FTR))
        pygame.draw.line(self.screen, CARD_BORDER, (0, y0), (W, y0))
        cy = y0 + FTR // 2
        self.icon("clock", (M + 4, cy - 12), 24, MUTED)
        self.mono(clock_str(), (M + 40, cy - 17), 28, "Medium", TEXT)
        x = M + 40 + 190
        pygame.draw.line(self.screen, LINE, (x, cy - 16), (x, cy + 16))
        x += 20
        if app.session.recording:
            if int(time.time() * 2) % 2:
                pygame.draw.circle(self.screen, RED, (x + 8, cy), 8)
            self.text("REC", (x + 24, cy), 18, "Bold", RED, "midleft")
        else:
            self.text("NO GRABA", (x, cy), 15, "SemiBold", MUTED, "midleft")
        x += 92
        t = app.link.telemetry
        armed = bool(t and t.has(P.TF_ARMED) and app.link.ok)
        if armed:
            x = self.pill("ARMADO", x, cy - 20, RED, "shield", filled=True)
        else:
            x = self.pill("DESARMADO", x, cy - 20, MUTED, "shield")
        x = self.pill(f"POT {app.power}%", x, cy - 20, BLUE, "bolt")
        if app.link.ok:
            # Pérdida real del enlace en los últimos ~5 s: comandos enviados que no tuvieron respuesta.
            # Una trama dañada suelta se descarta por CRC y no afecta (la siguiente llega 50 ms después),
            # así que solo se avisa si se pierde más del 5 %.
            now = time.monotonic()
            st = app.link.stats
            self._lq.append((now, st.tx, st.timeouts))
            while len(self._lq) > 2 and now - self._lq[0][0] > 5.0:
                self._lq.pop(0)
            t0, tx0, to0 = self._lq[0]
            loss = (st.timeouts - to0) / max(1, st.tx - tx0) * 100 if now - t0 > 1.0 else 0.0
            hz = st.rate_hz
            txt = f"ENLACE {hz:.0f} Hz" if hz >= 1 else "ENLACE OK"
            col = GREEN
            if loss >= 5:
                txt += f"  ·  pierde {loss:.0f}%"
                col = RED if loss >= 25 else YELLOW
            x = self.pill(txt, x, cy - 20, col, "signal")
        warns = []
        if not app.link.ok:
            warns.append("SIN ENLACE")
        if t and app.link.ok:
            if t.has(P.TF_SURFACING):
                warns.append("EMERGIENDO")
            if t.has(P.TF_LOWBAT_CRIT):
                warns.append("BATERÍA CRÍTICA")
            elif t.has(P.TF_LOWBAT_WARN):
                warns.append("BATERÍA BAJA")
            if t.has(P.TF_BALLAST_FAULT):
                warns.append("FALLA LASTRE")
        blink = int(time.time() * 3) % 3 != 0
        for wtxt in warns:
            x = self.pill(wtxt, x, cy - 20, RED if blink else (110, 30, 30), filled=True)

        # motores
        mw, mg = 46, 8
        mx = W - M - 5 * mw - 4 * mg
        self.text("MOTORES", (mx - 18, cy), 15, "SemiBold", MUTED, "midright")
        sep = mx - 18 - self.font(15, "SemiBold").size("MOTORES")[0] - 18
        pygame.draw.line(self.screen, LINE, (sep, cy - 16), (sep, cy + 16))
        if app.controls.pad is None:
            self.text("sin mando: teclado", (sep - 16, cy), 14, "Medium", MUTED, "midright")
        motors = t.motors if t else [0.0] * 5
        for i, m in enumerate(motors):
            b = pygame.Rect(mx + i * (mw + mg), cy - 22, mw, 44)
            self.rrect(b, (11, 24, 37), 8)
            self.rrect(b, CARD_BORDER, 8, 1)
            self.text(f"M{i + 1}", (b.centerx, b.y + 16), 15, "SemiBold", TEXT, "center")
            # barra desde el centro: derecha = positivo, izquierda = negativo
            tr = pygame.Rect(b.x + 6, b.bottom - 11, b.w - 12, 5)
            self.rrect(tr, TRACK, 2)
            if abs(m) > 0.01:
                half = tr.w // 2
                ln = max(2, int(abs(m) * half))
                fr = pygame.Rect(tr.centerx if m > 0 else tr.centerx - ln, tr.y, ln, tr.h)
                self.rrect(fr, ORANGE if i < 4 else CYAN, 2)
            pygame.draw.line(self.screen, MUTED, (tr.centerx, tr.y - 1), (tr.centerx, tr.bottom), 1)

    # ------------------------------------------------------------------ paneles F2 / F3
    def _panel_frame(self, title: str, width: int, alpha: int = 235) -> tuple[int, int, int]:
        x0, y0 = VX + 16, VY + 16
        hgt = VH - 32
        panel = pygame.Surface((width, hgt), pygame.SRCALPHA)
        pygame.draw.rect(panel, (*CARD, alpha), panel.get_rect(), border_radius=12)
        self.screen.blit(panel, (x0, y0))
        self.rrect(pygame.Rect(x0, y0, width, hgt), CARD_BORDER, 12, 1)
        # el panel tapa el video: los clics no deben atravesarlo
        self.hits.append((pygame.Rect(x0, y0, width, hgt), "nada"))
        self.text(title, (x0 + 20, y0 + 16), 24, "Bold", CYAN)
        self.button("Cerrar", pygame.Rect(x0 + width - 106, y0 + 14, 90, 32), "panel_cerrar")
        return x0, y0, hgt

    def _list_panel(self, title: str, help_: list[str], rows, sel: int, footer: str,
                    width: int, alpha: int = 235, extra: list[tuple[str, str]] = ()) -> None:
        x0, y0, hgt = self._panel_frame(title, width, alpha)
        y = y0 + 58
        for line in help_:
            self.text(line, (x0 + 20, y), 14, "Regular", MUTED)
            y += 20
        y += 10
        rowh = 30
        list_bottom = y0 + hgt - 96
        visible = max(1, (list_bottom - y) // rowh)
        top = max(0, min(sel - visible // 2, len(rows) - visible))
        vx = x0 + int(width * (0.50 if width >= 600 else 0.56))
        bw = 30
        for i in range(top, min(len(rows), top + visible)):
            name, value, unit = rows[i]
            row = pygame.Rect(x0 + 10, y - 4, width - 20, rowh - 2)
            hover = row.collidepoint(pygame.mouse.get_pos())
            if i == sel:
                self.rrect(row, TRACK, 6)
            elif hover:
                self.rrect(row, mix(CARD, TRACK, 0.5), 6)
            self.hits.append((row, f"sel:{i}"))
            self.text(name, (x0 + 22, y), 16, "Medium", TEXT if i == sel else MUTED)
            self.text(value, (vx, y), 16, "SemiBold", TEXT)
            if unit:
                self.text(unit, (vx + 90, y), 14, "Regular", MUTED)
            if i == sel:  # botones − / + de la fila elegida
                rp = pygame.Rect(row.right - bw - 6, row.y + 2, bw, row.h - 4)
                rm = pygame.Rect(rp.x - bw - 6, rp.y, bw, rp.h)
                for r, act, plus in ((rm, "izq", False), (rp, "der", True)):
                    self.button("", r, act)
                    c = TEXT if r.collidepoint(pygame.mouse.get_pos()) else MUTED
                    pygame.draw.line(self.screen, c, (r.centerx - 6, r.centery), (r.centerx + 6, r.centery), 2)
                    if plus:
                        pygame.draw.line(self.screen, c, (r.centerx, r.centery - 6), (r.centerx, r.centery + 6), 2)
            y += rowh
        if len(rows) > visible:  # indicador de desplazamiento
            bar_h = (list_bottom - (y0 + 92)) * visible // len(rows)
            by = y0 + 92 + (list_bottom - (y0 + 92) - bar_h) * top // max(1, len(rows) - visible)
            self.rrect(pygame.Rect(x0 + width - 6, by, 3, bar_h), LINE, 2)
        self.text(footer, (x0 + 20, y0 + hgt - 80), 15, "Medium", TEXT)
        # botones inferiores
        bx = x0 + width - 20
        for label, act in [("Guardar", "guardar"), *extra]:
            w = self.font(15, "SemiBold").size(label)[0] + 40
            bx -= w
            self.button(label, pygame.Rect(bx, y0 + hgt - 50, w, 34), act, active=(act == "guardar"))
            bx -= 10

    def _params_panel(self, app) -> None:
        rows = []
        for p in P.PARAMS:
            v = app.link.params.get(p.id)
            rows.append((p.name, "—" if v is None else f"{v:.4g}", p.unit))
        sel = P.PARAMS[app.panel_sel]
        self._list_panel(
            "PARÁMETROS DEL ROV",
            ["Clic en una fila para elegirla y − / + para ajustar (o ↑↓ y ←→; Shift ×10, rueda del mouse para moverse).",
             "Los cambios se aplican al instante; sin «Guardar» se pierden al apagar el ROV."],
            rows, app.panel_sel, f"{sel.name}: {sel.desc}", width=VW - 32)

    def _vision_panel(self, app) -> None:
        rows = []
        names = {"generico": "genérico", "cubo": "naranja", "moneda": "moneda"}
        for sec, key, _step, _lo, _hi in TUNABLE:
            v = app.detector.cfg[sec][key] if sec else app.detector.cfg[key]
            if isinstance(v, bool):
                val = "SÍ" if v else "no"
            else:
                val = f"{v:.3g}" if isinstance(v, float) else str(v)
            rows.append((f"{names[sec]}.{key}" if sec else key, val, ""))
        self._list_panel(
            "AJUSTE DE VISIÓN",
            ["Clic en una fila y − / + para ajustar.", "En «activo»: − apaga, + enciende.",
             "Más umbral = menos falsos positivos."],
            rows, app.panel_sel, f"Máscara: {({'cubo': 'naranja', 'generico': 'genérico'}).get(app.mask_kind, 'ninguna')}"
                                 " (rojo naranja, morado genérico)",
            width=440, alpha=215, extra=[("Máscara", "siguiente_mascara")])

    def _help_panel(self, app) -> None:
        width = VW - 32
        x0, y0, hgt = self._panel_frame("CONTROLES", width)
        cols = [
            ("MANDO", [
                ("Stick izquierdo", "avanzar / retroceder y girar"),
                ("Stick derecho (vertical)", "subir / bajar (lastre o setpoint)"),
                ("Gatillos RT / LT", "cabeceo manual (nariz arriba / abajo)"),
                ("START", "armar / desarmar"),
                ("A", "confirmar objeto detectado"),
                ("X", "marcar temperatura"),
                ("B", "profundidad automática on/off"),
                ("Y", "cabeceo automático on/off"),
                ("LB / RB", "potencia − / +"),
                ("BACK (1 s)", "cero de profundidad"),
                ("Cruceta arriba / abajo", "objetivo de profundidad −/+ 10 cm"),
                ("Cruceta izquierda", "detección on/off (solo video)"),
            ]),
            ("TECLADO", [
                ("W / S · A / D", "avanzar / retroceder · girar"),
                ("R / F · T / G", "subir / bajar · cabeceo"),
                ("Enter", "armar / desarmar"),
                ("Espacio · C", "confirmar objeto · temperatura"),
                ("H · P", "profundidad auto · cabeceo auto"),
                ("1 2 3 4", "potencia 25 / 50 / 75 / 100 %"),
                ("Z", "cero de profundidad (en superficie)"),
                ("L", "nivelar IMU (desarmado, quieto)"),
                ("V", "detección on/off (solo video)"),
                ("− / +", "objetivo de profundidad −/+ 10 cm"),
                ("F1 F2 F3", "controles · parámetros · visión"),
                ("F5 · F11", "grabar · pantalla completa"),
                ("Esc ×2", "salir (desarma el ROV)"),
            ]),
        ]
        cw = (width - 60) // 2
        for k, (title, items) in enumerate(cols):
            x = x0 + 20 + k * (cw + 20)
            y = y0 + 70
            self.text(title, (x, y), 15, "Bold", CYAN)
            y += 32
            for key, desc in items:
                self.text(key, (x, y), 16, "SemiBold", TEXT)
                self.text(desc, (x + int(cw * 0.42), y), 16, "Regular", MUTED)
                y += 30
        self.text("Con el panel de parámetros o de visión abierto, el pilotaje con el mando sigue activo.",
                  (x0 + 20, y0 + hgt - 40), 14, "Regular", MUTED)

    # ------------------------------------------------------------------ banner y mensajes
    def _banner_and_toast(self, app) -> None:
        now = time.monotonic()
        if app.banner and now < app.banner[2]:
            text, color, _ = app.banner
            r = pygame.Rect(VX, VY, VW, 64)
            pygame.draw.rect(self.screen, color, r, border_top_left_radius=12, border_top_right_radius=12)
            self.text(text, r.center, 30, "Bold", (255, 255, 255), "center")
            if color == RED:  # marco rojo alrededor del video mientras dura el banner
                self.rrect(VIDEO, RED, 12, 6)
        if app.toast and now < app.toast[1]:
            surf = self.font(17, "Medium").render(app.toast[0], True, TEXT)
            r = surf.get_rect(midbottom=(VIDEO.centerx, VIDEO.bottom - 20))
            self.rrect(r.inflate(32, 16), (8, 16, 26), 10)
            self.rrect(r.inflate(32, 16), CARD_BORDER, 10, 1)
            self.screen.blit(surf, r)
