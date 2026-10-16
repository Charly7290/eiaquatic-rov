"""Dibujo de la ventana de piloteo (1280×768).

┌──────────────────────── video 960×720 ────────────┬──── panel 320 ────┐
│ contornos de detección, mira, banner de hallazgo   │ ROI + estado      │
│                                                    │ telemetría        │
├────────────────────────────────────────────────────┴───────────────────┤
│ reloj ms · REC · armado · modos · potencia · avisos · motores         │
└─────────────────────────────────────────────────────────────────────────┘
"""
from __future__ import annotations

import math
import time
from datetime import datetime

import cv2
import numpy as np
import pygame

from . import protocol as P
from .detection import TUNABLE

W, H = 1280, 768
VW, VH = 960, 720
PX = VW  # x del panel derecho
PW = W - VW

BG = (14, 17, 22)
PANEL = (22, 27, 34)
LINE = (48, 54, 61)
TEXT = (230, 237, 243)
MUTED = (139, 148, 158)
ORANGE = (242, 140, 40)
GREEN = (63, 185, 80)
YELLOW = (210, 153, 34)
RED = (248, 81, 73)
CYAN = (57, 197, 207)


def _font(size: int, mono: bool = False, bold: bool = False) -> pygame.font.Font:
    names = "dejavusansmono,consolas,menlo,monospace" if mono else "dejavusans,segoeui,arial,helvetica"
    return pygame.font.SysFont(names, size, bold=bold)


def clock_str() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class UI:
    def __init__(self, screen: pygame.Surface) -> None:
        self.screen = screen
        self.f_small = _font(13)
        self.f_norm = _font(16)
        self.f_bold = _font(16, bold=True)
        self.f_big = _font(26, bold=True)
        self.f_huge = _font(34, bold=True)
        self.f_mono = _font(22, mono=True, bold=True)
        self.f_mono_s = _font(14, mono=True)
        self.f_banner = _font(28, bold=True)

    # ----------------------------------------------------------------- utilidades
    def text(self, s: str, pos, font=None, color=TEXT, anchor: str = "topleft") -> pygame.Rect:
        surf = (font or self.f_norm).render(s, True, color)
        r = surf.get_rect(**{anchor: pos})
        self.screen.blit(surf, r)
        return r

    def pill(self, s: str, x: int, y: int, bg, fg=TEXT) -> int:
        surf = self.f_bold.render(s, True, fg)
        r = pygame.Rect(x, y, surf.get_width() + 16, 26)
        pygame.draw.rect(self.screen, bg, r, border_radius=13)
        self.screen.blit(surf, surf.get_rect(center=r.center))
        return r.right + 8

    def bar(self, rect: pygame.Rect, frac: float, color, center: bool = False) -> None:
        pygame.draw.rect(self.screen, (40, 46, 54), rect, border_radius=3)
        if center:  # barra bipolar -1..1
            mid = rect.x + rect.w // 2
            w = int(rect.w / 2 * max(-1.0, min(1.0, frac)))
            r = pygame.Rect(min(mid, mid + w), rect.y, abs(w), rect.h)
            pygame.draw.line(self.screen, LINE, (mid, rect.y), (mid, rect.bottom - 1))
        else:
            r = pygame.Rect(rect.x, rect.y, int(rect.w * max(0.0, min(1.0, frac))), rect.h)
        if r.w > 0:
            pygame.draw.rect(self.screen, color, r, border_radius=3)

    # ----------------------------------------------------------------- dibujo
    def draw(self, app) -> None:
        self.screen.fill(BG)
        self._video(app)
        self._right_panel(app)
        self._bottom_bar(app)
        if app.panel == "params":
            self._params_panel(app)
        elif app.panel == "vision":
            self._vision_panel(app)
        self._banner_and_toast(app)

    def _video(self, app) -> None:
        frame = app.frame
        if frame is None:
            pygame.draw.rect(self.screen, (8, 10, 13), (0, 0, VW, VH))
            self.text("SIN VIDEO", (VW // 2, VH // 2 - 20), self.f_huge, MUTED, "center")
            self.text(app.video_status, (VW // 2, VH // 2 + 20), self.f_norm, MUTED, "center")
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
            if confirmed:
                color, thick = (60, 60, 255), 4
            elif t.stable:
                color, thick = (0, 220, 255), 3  # amarillo = candidato
            else:
                color, thick = t.color, 1
            cv2.drawContours(vis, [cnt], -1, color, thick, cv2.LINE_AA)
            if t.stable or confirmed:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(vis, (x - 4, y - 4), (x + w + 4, y + h + 4), color, 1, cv2.LINE_AA)
                labels.append((t, x, y, confirmed))

        cx, cy = VW // 2, VH // 2  # mira
        cv2.line(vis, (cx - 18, cy), (cx - 6, cy), (230, 230, 230), 1, cv2.LINE_AA)
        cv2.line(vis, (cx + 6, cy), (cx + 18, cy), (230, 230, 230), 1, cv2.LINE_AA)
        cv2.line(vis, (cx, cy - 18), (cx, cy - 6), (230, 230, 230), 1, cv2.LINE_AA)
        cv2.line(vis, (cx, cy + 6), (cx, cy + 18), (230, 230, 230), 1, cv2.LINE_AA)

        rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        self.screen.blit(pygame.image.frombuffer(rgb.tobytes(), (VW, VH), "RGB"), (0, 0))
        for t, x, y, confirmed in labels:
            txt = ("✓ " if confirmed else "") + t.label
            col = RED if confirmed else YELLOW
            surf = self.f_bold.render(txt, True, (0, 0, 0))
            r = surf.get_rect(bottomleft=(x - 4, max(22, y - 6)))
            pygame.draw.rect(self.screen, col, r.inflate(8, 4), border_radius=3)
            self.screen.blit(surf, r)

    def _section(self, title: str, y: int) -> int:
        self.text(title, (PX + 14, y), self.f_small, MUTED)
        pygame.draw.line(self.screen, LINE, (PX + 14, y + 18), (W - 14, y + 18))
        return y + 26

    def _right_panel(self, app) -> None:
        pygame.draw.rect(self.screen, PANEL, (PX, 0, PW, VH))
        y = self._section("DETECCIÓN", 8)

        # ROI
        box = pygame.Rect(PX + 14, y, PW - 28, 200)
        pygame.draw.rect(self.screen, (8, 10, 13), box)
        roi, frozen = app.roi_display()
        if roi is not None and roi.size:
            rh, rw = roi.shape[:2]
            k = min(box.w / rw, box.h / rh)
            roi_r = cv2.resize(roi, (max(1, int(rw * k)), max(1, int(rh * k))), interpolation=cv2.INTER_NEAREST
                               if k > 3 else cv2.INTER_LINEAR)
            rgb = cv2.cvtColor(roi_r, cv2.COLOR_BGR2RGB)
            surf = pygame.image.frombuffer(rgb.tobytes(), (rgb.shape[1], rgb.shape[0]), "RGB")
            self.screen.blit(surf, surf.get_rect(center=box.center))
        else:
            self.text("sin objeto a la vista", box.center, self.f_small, MUTED, "center")
        pygame.draw.rect(self.screen, RED if frozen else LINE, box, 3 if frozen else 1)
        y = box.bottom + 8

        if frozen:
            info = app.last_confirm
            self.text(f"● DETECTADO #{info['k']}", (PX + 14, y), self.f_big, RED)
            self.text(f"{info['label']}  ·  {info['hora']}", (PX + 14, y + 32), self.f_norm, TEXT)
        else:
            best = app.detector.best_candidate()
            if best is not None and best.stable:
                self.text("CANDIDATO", (PX + 14, y), self.f_big, YELLOW)
                self.text(f"{best.label}  ·  pulse A para confirmar", (PX + 14, y + 32), self.f_small, TEXT)
            else:
                self.text("Buscando…", (PX + 14, y), self.f_big, MUTED)
        self.text(f"Hallazgos confirmados: {app.session.n_detections}", (PX + 14, y + 54), self.f_small, MUTED)
        y += 80

        # Telemetría
        y = self._section("TELEMETRÍA", y)
        t: P.Telemetry | None = app.link.telemetry
        stale = not app.link.ok

        def val(s, x, yy, font=None, color=TEXT):
            self.text(s, (x, yy), font or self.f_bold, MUTED if stale else color)

        if t is None:
            self.text("esperando al ROV…", (PX + 14, y + 4), self.f_norm, MUTED)
            return

        # Batería
        vcol = RED if t.has(P.TF_LOWBAT_CRIT) else YELLOW if t.has(P.TF_LOWBAT_WARN) else GREEN
        self.text("Batería", (PX + 14, y), self.f_small, MUTED)
        val(f"{t.vbat:5.2f} V", (PX + 120), y - 2, self.f_bold, vcol)
        self.bar(pygame.Rect(PX + 200, y + 3, PW - 214, 10), (t.vbat - 10.5) / (12.6 - 10.5), vcol)
        y += 26

        # Profundidad
        self.text("Profundidad", (PX + 14, y), self.f_small, MUTED)
        if t.has(P.TF_DEPTH_OK):
            val(f"{t.depth:5.2f} m", PX + 120, y - 6, self.f_big)
            arrow = "↓" if t.vz > 0.02 else "↑" if t.vz < -0.02 else "·"
            val(f"{arrow} {abs(t.vz):.2f} m/s", PX + 232, y, self.f_small)
            sp = f"setpoint {t.depth_sp:.2f} m" if t.has(P.TF_DEPTH_HOLD) else "manual"
            self.text(f"{sp}  ·  {t.pressure_mbar} mbar", (PX + 120, y + 24), self.f_small, MUTED)
        else:
            val("sin sensor", PX + 120, y - 2, self.f_bold, RED)
        y += 48

        # Temperatura
        self.text("Temp. agua", (PX + 14, y), self.f_small, MUTED)
        if t.water_temp is not None:
            val(f"{t.water_temp:5.2f} °C", PX + 120, y - 6, self.f_big, CYAN)
        else:
            val("sin sensor", PX + 120, y - 2, self.f_bold, RED)
        y += 38

        # Actitud + horizonte
        self.text("Cabeceo", (PX + 14, y), self.f_small, MUTED)
        val(f"{t.pitch:+6.1f}°", PX + 90, y - 2)
        if t.has(P.TF_PITCH_HOLD):
            self.text(f"sp {t.pitch_sp:+.0f}°", (PX + 90, y + 18), self.f_small, MUTED)
        self.text("Alabeo", (PX + 14, y + 40), self.f_small, MUTED)
        val(f"{t.roll:+6.1f}°", PX + 90, y + 38)
        self._horizon(PX + PW - 60, y + 30, 40, t.pitch, t.roll, stale or not t.has(P.TF_IMU_OK))
        y += 76

        # Lastre
        self.text("Lastre", (PX + 14, y), self.f_small, MUTED)
        r = pygame.Rect(PX + 90, y + 2, PW - 104, 14)
        if t.ballast_pos is not None:
            self.bar(r, t.ballast_pos, ORANGE)
            pos = f"{t.ballast_pos * 100:3.0f} %"
        else:
            pygame.draw.rect(self.screen, (40, 46, 54), r, border_radius=3)
            pos = "posición ? (toque un fin de carrera)"
        state = {1: "llenando ▼", -1: "vaciando ▲", 0: ""}[max(-1, min(1, t.ballast_dir))]
        lims = ("LLENO " if t.has(P.TF_LIM_FULL) else "") + ("VACÍO" if t.has(P.TF_LIM_EMPTY) else "")
        fault = "  FALLA" if t.has(P.TF_BALLAST_FAULT) else ""
        self.text(f"{pos}  {state}  {lims}{fault}", (PX + 90, y + 20), self.f_small,
                  RED if fault else MUTED)
        y += 44

        # Enlace
        self.text("Enlace", (PX + 14, y), self.f_small, MUTED)
        st = app.link.stats
        if app.link.ok:
            self.text(f"OK  {st.rate_hz:4.1f} Hz  {st.rtt_ms:4.1f} ms  err {t.rx_err}/{app.link.parser.errors}",
                      (PX + 90, y), self.f_small, GREEN)
        else:
            err = getattr(app.link.transport, "error", None)
            self.text("SIN ENLACE" + (f": {err[:28]}" if err else ""), (PX + 90, y), self.f_small, RED)
        y += 20
        self.text(f"Video: {app.video_status}  ·  Mando: {app.controls.pad_name[:22]}", (PX + 14, y),
                  self.f_small, MUTED)

    def _horizon(self, cx: int, cy: int, r: int, pitch: float, roll: float, dim: bool) -> None:
        size = 2 * r
        surf = pygame.Surface((size, size), pygame.SRCALPHA)
        surf.fill((60, 120, 200) if not dim else (60, 70, 80))
        off = max(-r, min(r, pitch * 1.5))
        ang = math.radians(roll)
        c, s = math.cos(ang), math.sin(ang)
        hx, hy = r, r + off
        L = 3 * r
        pts = [(hx - L * c, hy - L * s), (hx + L * c, hy + L * s), (hx + L * c - L * s, hy + L * s + L * c),
               (hx - L * c - L * s, hy - L * s + L * c)]
        pygame.draw.polygon(surf, (150, 100, 50) if not dim else (90, 90, 90), pts)
        mask = pygame.Surface((size, size), pygame.SRCALPHA)
        mask.fill((0, 0, 0, 0))
        pygame.draw.circle(mask, (255, 255, 255, 255), (r, r), r)
        surf.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        self.screen.blit(surf, (cx - r, cy - r))
        pygame.draw.circle(self.screen, LINE, (cx, cy), r, 2)
        pygame.draw.line(self.screen, YELLOW, (cx - r // 2, cy), (cx - 6, cy), 3)
        pygame.draw.line(self.screen, YELLOW, (cx + 6, cy), (cx + r // 2, cy), 3)

    def _bottom_bar(self, app) -> None:
        y0 = VH
        pygame.draw.rect(self.screen, (10, 12, 16), (0, y0, W, H - VH))
        pygame.draw.line(self.screen, LINE, (0, y0), (W, y0))
        self.text(clock_str(), (12, y0 + 11), self.f_mono, TEXT)
        x = 190
        if app.session.recording:
            if int(time.time() * 2) % 2:
                pygame.draw.circle(self.screen, RED, (x + 8, y0 + 24), 7)
            self.text("REC", (x + 20, y0 + 15), self.f_bold, RED)
        else:
            self.text("no graba", (x, y0 + 16), self.f_small, MUTED)
        x = 270
        t = app.link.telemetry
        armed = bool(t and t.has(P.TF_ARMED) and app.link.ok)
        x = self.pill("ARMADO" if armed else "DESARMADO", x, y0 + 11, RED if armed else (60, 66, 74))
        x = self.pill("CAB AUTO" if app.pitch_hold else "CAB MANUAL", x, y0 + 11,
                      GREEN if app.pitch_hold else (60, 66, 74))
        x = self.pill("PROF AUTO" if app.depth_hold else "PROF MANUAL", x, y0 + 11,
                      GREEN if app.depth_hold else (60, 66, 74))
        x = self.pill(f"POT {app.power}%", x, y0 + 11, (50, 70, 110))
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
        for wtxt in warns:
            if int(time.time() * 3) % 3:
                x = self.pill(wtxt, x, y0 + 11, RED)
            else:
                x = self.pill(wtxt, x, y0 + 11, (90, 30, 30))
        # motores
        if t:
            mx = W - 5 * 34 - 10
            for i, m in enumerate(t.motors):
                r = pygame.Rect(mx + i * 34, y0 + 8, 26, 32)
                pygame.draw.rect(self.screen, (40, 46, 54), r, border_radius=3)
                h = int(abs(m) * 15)
                mid = r.y + 16
                rr = pygame.Rect(r.x + 3, mid - h if m > 0 else mid, 20, h)
                pygame.draw.rect(self.screen, ORANGE if i < 4 else CYAN, rr)
                self.text(f"M{i + 1}", (r.centerx, r.bottom - 1), _font(10), MUTED, "midbottom")

    def _list_panel(self, title: str, help_: list[str], rows: list[tuple[str, str, str]], sel: int, footer: str,
                    width: int = VW - 80, alpha: int = 230) -> None:
        x0, y0 = 20, 66  # debajo del banner
        panel = pygame.Surface((width, VH - y0 - 20), pygame.SRCALPHA)
        panel.fill((10, 12, 16, alpha))
        self.screen.blit(panel, (x0, y0))
        self.text(title, (x0 + 16, y0 + 10), self.f_big, ORANGE)
        y = y0 + 46
        for line in help_:
            self.text(line, (x0 + 16, y), self.f_small, MUTED)
            y += 18
        y += 8
        visible = (VH - 60 - y) // 24
        top = max(0, min(sel - visible // 2, len(rows) - visible))
        vx = x0 + int(width * 0.55)
        for i in range(top, min(len(rows), top + visible)):
            name, value, unit = rows[i]
            if i == sel:
                pygame.draw.rect(self.screen, (50, 60, 75), (x0 + 8, y - 2, width - 16, 24), border_radius=4)
            self.text(name, (x0 + 16, y), self.f_mono_s, TEXT if i == sel else MUTED)
            self.text(value, (vx, y), self.f_mono_s, TEXT)
            self.text(unit, (vx + 110, y), self.f_small, MUTED)
            y += 24
        self.text(footer, (x0 + 16, VH - 44), self.f_small if width < 600 else self.f_norm, TEXT)

    def _params_panel(self, app) -> None:
        rows = []
        for p in P.PARAMS:
            v = app.link.params.get(p.id)
            rows.append((p.name, "—" if v is None else f"{v:.4g}", p.unit))
        sel = P.PARAMS[app.panel_sel]
        self._list_panel(
            "PARÁMETROS DEL ROV",
            ["↑↓ elegir   ←→ ajustar (Shift ×10)   Ctrl+S guardar en la flash del ROV   F2 cerrar",
             "Los cambios se aplican al instante; sin Ctrl+S se pierden al apagar el ROV."],
            rows, app.panel_sel, f"{sel.name}: {sel.desc}")

    def _vision_panel(self, app) -> None:
        rows = []
        for sec, key, _step, _lo, _hi in TUNABLE:
            v = app.detector.cfg[sec][key] if sec else app.detector.cfg[key]
            rows.append((f"{sec}.{key}" if sec else key, f"{v:.3g}" if isinstance(v, float) else str(v), ""))
        self._list_panel(
            "AJUSTE DE VISIÓN",
            ["↑↓ elegir   ←→ ajustar (Shift ×10)", "Tab máscara   Ctrl+S guardar   F3 cerrar"],
            rows, app.panel_sel, f"Máscara: {app.mask_kind or 'ninguna'} (rojo cubo, morado genérico)",
            width=360, alpha=200)

    def _banner_and_toast(self, app) -> None:
        now = time.monotonic()
        if app.banner and now < app.banner[2]:
            text, color, _ = app.banner
            surf = self.f_banner.render(text, True, (255, 255, 255))
            r = pygame.Rect(0, 0, VW, 56)
            pygame.draw.rect(self.screen, color, r)
            self.screen.blit(surf, surf.get_rect(center=r.center))
            if color == RED:  # marco rojo alrededor del video mientras dura el banner
                pygame.draw.rect(self.screen, RED, (0, 0, VW, VH), 8)
        if app.toast and now < app.toast[1]:
            surf = self.f_norm.render(app.toast[0], True, TEXT)
            r = surf.get_rect(midbottom=(VW // 2, VH - 16))
            pygame.draw.rect(self.screen, (10, 12, 16), r.inflate(24, 12), border_radius=8)
            self.screen.blit(surf, r)
