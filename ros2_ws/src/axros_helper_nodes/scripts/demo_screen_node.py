#!/usr/bin/env python3
# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
Demo screen viewer. Wraps the live inference feed in the Axelera partner-
demo template and shows it through cv::imshow.

Builds the template chrome (panels, headers, footer, metrics) once into a
numpy background, then per-frame copies the background, blits the live
video into the canvas, paints detection overlays + dynamic metrics, and
calls cv::imshow. Lowest-latency path on the demo machine — no browser,
no JPEG encode, no MJPEG buffering.

Subscribes to image_raw, /detections, /color_detections, /pick_roi_polygon
and draws the same overlays the C++ live_inference node would have drawn.
The node is read-only on the rest of the stack — camera_frame_annotated,
pick_and_place, etc. are untouched.

Set MAIN_CAMERA_DISPLAY=0 (default on Pi) so the C++ cv::imshow window
doesn't compete with this viewer.

Footer assets (chip icon, QR code, Axelera logo) are loaded from the paths
defined at module level — no runtime overrides.
"""

import os
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PolygonStamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


# Overlay caches are dropped if older than this. Matches DET_OVERLAY_STALE_S
# in live_inference_node.cpp so the two viewers behave identically.
DET_OVERLAY_STALE_S = 0.5

# ── Axelera brand palette (BGR for OpenCV) ──────────────────────────────────
BG_DEEP        = (40,  18,  10)    # night-blue-deep #0A1228
BG_PANEL       = (50,  35,  25)    # subtle panel surface, slightly above BG
PANEL_BORDER   = (90,  70,  55)    # ~14% white over BG, but flat
TEXT_DIM       = (140, 140, 140)
TEXT_MID       = (190, 190, 190)
TEXT_BRIGHT    = (255, 255, 255)
AXELERA_YELLOW = (24,  190, 244)   # #F4BE18
DARK_INK       = (61,  29,  18)    # night-blue text on yellow

# Candidate TTF paths for cv2.freetype. First hit wins; if none load we
# fall back to Hershey (engineering-blueprint look but always works).
SANS_TTF_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf',
    '/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf',
]
SANS_BOLD_TTF_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
]
MONO_TTF_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
    '/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf',
]


def _load_ft(candidates):
    if not hasattr(cv2, 'freetype'):
        return None
    for p in candidates:
        if not os.path.exists(p):
            continue
        try:
            ft = cv2.freetype.createFreeType2()
            ft.loadFontData(p, 0)
            return ft
        except cv2.error:
            continue
    return None


def _alpha_blit(bg: np.ndarray, fg: np.ndarray, x: int, y: int) -> None:
    """Alpha-composite `fg` onto `bg` at (x, y). fg may be BGR or BGRA;
    bg is BGR. Out-of-bounds regions are clipped, not wrapped."""
    if fg is None:
        return
    h, w = fg.shape[:2]
    bh, bw = bg.shape[:2]
    if x >= bw or y >= bh or x + w <= 0 or y + h <= 0:
        return
    # Clip
    fx0 = max(0, -x)
    fy0 = max(0, -y)
    fx1 = min(w, bw - x)
    fy1 = min(h, bh - y)
    if fx1 <= fx0 or fy1 <= fy0:
        return
    bx0, by0 = x + fx0, y + fy0
    bx1, by1 = x + fx1, y + fy1
    fg_roi = fg[fy0:fy1, fx0:fx1]
    bg_roi = bg[by0:by1, bx0:bx1]
    if fg_roi.ndim == 3 and fg_roi.shape[2] == 4:
        alpha = fg_roi[..., 3:4].astype(np.float32) / 255.0
        rgb = fg_roi[..., :3].astype(np.float32)
        out = rgb * alpha + bg_roi.astype(np.float32) * (1.0 - alpha)
        bg[by0:by1, bx0:bx1] = np.clip(out, 0, 255).astype(np.uint8)
    else:
        bg[by0:by1, bx0:bx1] = fg_roi


# ────────────────────────────────────────────────────────────────────────────
# Asset paths — hardcoded. The demo screen ships with the matching PNGs in
# this package's resource/ dir, and the Axelera logo lives in the voyager-sdk
# install on every demo machine. Not exposed as ROS parameters or env vars.
# ────────────────────────────────────────────────────────────────────────────

try:
    _PKG_SHARE = get_package_share_directory('axros_helper_nodes')
except Exception:
    _PKG_SHARE = ''
QR_CODE_PATH      = (os.path.join(_PKG_SHARE, 'resource', 'QR_code.png')
                     if _PKG_SHARE else '')
CHIP_ICON_PATH    = (os.path.join(_PKG_SHARE, 'resource', 'chip.png')
                     if _PKG_SHARE else '')
AXELERA_LOGO_PATH = os.path.expanduser(
    '~/voyager-sdk/axelera/app/axelera-ai-logo.png')


def _load_image(path: str, logger=None) -> np.ndarray | None:
    """Load a PNG with alpha if present. Returns None on missing/bad files."""
    if not path or not os.path.exists(path):
        if logger is not None:
            logger.warn(f'image not found at {path!r}; using fallback.')
        return None
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None and logger is not None:
        logger.warn(f'cv2.imread failed for {path!r}; using fallback.')
    return img


# ────────────────────────────────────────────────────────────────────────────
# Template renderer (cv::imshow mode)
# ────────────────────────────────────────────────────────────────────────────

class TemplateRenderer:
    """Builds and refreshes the Axelera demo screen for cv::imshow.

    The static chrome (panels, labels, footer, corner markers) is composited
    once into self.bg. render() copies it, blits the live video, and overlays
    dynamic regions (detection boxes + jitter metrics)."""

    def __init__(self, width: int = 1920, height: int = 1080,
                 axelera_logo: np.ndarray | None = None,
                 qr_image: np.ndarray | None = None,
                 chip_icon: np.ndarray | None = None):
        self.w = width
        self.h = height
        self.axelera_logo = axelera_logo  # BGRA preferred
        self.qr_image     = qr_image       # BGR/BGRA, square-ish preferred
        self.chip_icon    = chip_icon      # BGRA preferred, square-ish

        # Padding + grid (mirrors the HTML's `.screen` grid).
        self.pad = max(16, int(width * 24 / 1920))
        right_col_w = int(width * 520 / 1920)
        footer_h = int(height * 180 / 1080)

        # Inference window region (the live video frame).
        self.win = self._rect(
            self.pad, self.pad,
            width - right_col_w - 3 * self.pad, height - footer_h - 3 * self.pad)
        # Inside the inference window, the header bar and the canvas region.
        header_h = max(48, int(height * 60 / 1080))
        self.header = self._rect(
            self.win.x, self.win.y, self.win.w, header_h)
        self.canvas = self._rect(
            self.win.x, self.win.y + header_h,
            self.win.w, self.win.h - header_h)

        # Right column: task panel fills the full column (perf panel removed).
        right_x = self.win.x + self.win.w + self.pad
        self.task = self._rect(right_x, self.win.y, right_col_w, self.win.h)

        # Footer panel.
        self.footer = self._rect(
            self.pad, self.win.y + self.win.h + self.pad,
            width - 2 * self.pad, footer_h)

        # Fonts.
        self.ft_sans = _load_ft(SANS_TTF_CANDIDATES)
        self.ft_bold = _load_ft(SANS_BOLD_TTF_CANDIDATES) or self.ft_sans
        self.ft_mono = _load_ft(MONO_TTF_CANDIDATES) or self.ft_sans

        # Cached pre-rendered background.
        self.bg = self._build_background()

    # ── tiny geometry helper ────────────────────────────────────────────────

    class _Rect:
        __slots__ = ('x', 'y', 'w', 'h')

        def __init__(self, x, y, w, h):
            self.x = int(x)
            self.y = int(y)
            self.w = int(w)
            self.h = int(h)

        @property
        def x2(self): return self.x + self.w

        @property
        def y2(self): return self.y + self.h

    def _rect(self, x, y, w, h):
        return TemplateRenderer._Rect(x, y, w, h)

    # ── text helpers ────────────────────────────────────────────────────────

    def _text(self, img, s: str, x: int, y: int, size: int,
              color, font='sans', baseline='bottom'):
        """Draw text. `y` is the baseline for cv2.putText; for freetype we
        adjust upward so callers can treat y as the baseline consistently."""
        ft = {'sans': self.ft_sans, 'bold': self.ft_bold,
              'mono': self.ft_mono}.get(font, self.ft_sans)
        if ft is not None:
            # freetype org is top-left of the text bounding box.
            org = (x, y - size if baseline == 'bottom' else y)
            ft.putText(img, s, org, size, color, thickness=-1,
                       line_type=cv2.LINE_AA, bottomLeftOrigin=False)
        else:
            scale = max(0.3, size / 38.0)
            thickness = 2 if font == 'bold' else 1
            cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                        color, thickness, cv2.LINE_AA)

    def _text_size(self, s: str, size: int, font='sans'):
        ft = {'sans': self.ft_sans, 'bold': self.ft_bold,
              'mono': self.ft_mono}.get(font, self.ft_sans)
        if ft is not None:
            (w, h), _ = ft.getTextSize(s, size, -1)
            return w, h
        scale = max(0.3, size / 38.0)
        thickness = 2 if font == 'bold' else 1
        (w, h), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX,
                                     scale, thickness)
        return w, h

    # ── panel primitives ────────────────────────────────────────────────────

    def _panel(self, img, r, fill=BG_PANEL, border=PANEL_BORDER):
        cv2.rectangle(img, (r.x, r.y), (r.x2, r.y2), fill, cv2.FILLED)
        cv2.rectangle(img, (r.x, r.y), (r.x2, r.y2), border, 1, cv2.LINE_AA)

    def _hline(self, img, x1, y, x2, color=PANEL_BORDER):
        cv2.line(img, (x1, y), (x2, y), color, 1, cv2.LINE_AA)

    def _control_chip(self, img, label: str, x: int, y: int,
                      pad_x: int = 10, pad_y: int = 4) -> int:
        size = 14
        tw, th = self._text_size(label, size, font='mono')
        x2 = x + tw + 2 * pad_x
        y2 = y + th + 2 * pad_y
        cv2.rectangle(img, (x, y), (x2, y2), PANEL_BORDER, 1, cv2.LINE_AA)
        self._text(img, label, x + pad_x, y + pad_y + th,
                   size, TEXT_DIM, font='mono')
        return x2

    def _corner_markers(self, img, r, length=24, thickness=2, margin=16):
        c = AXELERA_YELLOW
        x1, y1, x2, y2 = r.x + margin, r.y + margin, r.x2 - margin, r.y2 - margin
        # top-left
        cv2.line(img, (x1, y1), (x1 + length, y1), c, thickness, cv2.LINE_AA)
        cv2.line(img, (x1, y1), (x1, y1 + length), c, thickness, cv2.LINE_AA)
        # top-right
        cv2.line(img, (x2, y1), (x2 - length, y1), c, thickness, cv2.LINE_AA)
        cv2.line(img, (x2, y1), (x2, y1 + length), c, thickness, cv2.LINE_AA)
        # bottom-left
        cv2.line(img, (x1, y2), (x1 + length, y2), c, thickness, cv2.LINE_AA)
        cv2.line(img, (x1, y2), (x1, y2 - length), c, thickness, cv2.LINE_AA)
        # bottom-right
        cv2.line(img, (x2, y2), (x2 - length, y2), c, thickness, cv2.LINE_AA)
        cv2.line(img, (x2, y2), (x2, y2 - length), c, thickness, cv2.LINE_AA)

    # ── static background ──────────────────────────────────────────────────

    def _build_background(self) -> np.ndarray:
        bg = np.full((self.h, self.w, 3), BG_DEEP, dtype=np.uint8)

        # Inference window outline (header + canvas borders).
        self._panel(bg, self.win, fill=BG_DEEP)
        # Header strip — slightly darker than canvas so it reads as a bar.
        header_strip = self.header
        cv2.rectangle(bg,
                      (header_strip.x, header_strip.y),
                      (header_strip.x2, header_strip.y2),
                      (28, 12, 6), cv2.FILLED)
        self._hline(bg, header_strip.x, header_strip.y2,
                    header_strip.x2)
        # Canvas background (pure black behind the eventual video blit).
        cv2.rectangle(bg,
                      (self.canvas.x, self.canvas.y),
                      (self.canvas.x2, self.canvas.y2),
                      (0, 0, 0), cv2.FILLED)
        # Header text — left side: LIVE INFERENCE // STREAM 01
        # Live dot rendered each frame (no animation, plain yellow dot).
        hy = self.header.y + self.header.h // 2 + 6
        self._text(bg, 'LIVE INFERENCE', self.header.x + 60, hy,
                   13, TEXT_DIM, font='mono')
        sep_w, _ = self._text_size('LIVE INFERENCE', 13, font='mono')
        sep_x = self.header.x + 60 + sep_w + 14
        self._text(bg, '//', sep_x, hy, 13, PANEL_BORDER, font='mono')
        self._text(bg, 'STREAM 01', sep_x + 28, hy, 13, TEXT_DIM, font='mono')

        # Header chips on the right.
        chip_x = self.header.x2 - 16
        for label in reversed(['RGB', '1920×1080']):
            tw, _ = self._text_size(label, 14, font='mono')
            cw = tw + 20
            chip_left = chip_x - cw
            self._control_chip(bg, label, chip_left, self.header.y + 12)
            chip_x = chip_left - 8

        # Corner markers inside the canvas region.
        self._corner_markers(bg, self.canvas)

        # ── Right column: task panel (fills the column) ─────────────────────
        self._panel(bg, self.task)
        # "LIVE DEMO" yellow tag.
        tag_x = self.task.x + 20
        tag_y = self.task.y + 24
        tag_label = 'LIVE DEMO'
        tw, th = self._text_size(tag_label, 12, font='mono')
        cv2.rectangle(bg, (tag_x, tag_y - 4),
                      (tag_x + tw + 16, tag_y + th + 6),
                      AXELERA_YELLOW, cv2.FILLED)
        self._text(bg, tag_label, tag_x + 8, tag_y + th + 2,
                   12, DARK_INK, font='mono')

        # Task title (manually wrapped — 26 px bold doesn't fit a single
        # line of "Low-Cost, High-Performance AI Robotics" in the panel).
        title_lines = ['Low-Cost, High-Performance', 'AI Robotics']
        y = tag_y + th + 36
        for line in title_lines:
            self._text(bg, line, self.task.x + 20, y, 26,
                       TEXT_BRIGHT, font='bold')
            y += 30

        # Description (manually wrapped to ~52 chars so the 6 lines fit
        # between the title and the meta block).
        desc_lines = [
            "AI-powered robotics shouldn't require an enterprise",
            'budget. A Raspberry Pi inside the Ned2 handles robot',
            'control, while Metis on an Orange Pi accelerates the',
            'AI workload externally. Edge AI meets physical',
            'automation: fast, affordable, and ready to deploy',
            'outside the lab.',
        ]
        y += 8
        for line in desc_lines:
            self._text(bg, line, self.task.x + 20, y, 15,
                       TEXT_MID, font='sans')
            y += 22

        # Meta rows at the bottom. The Model row spans two lines, so the
        # block is ~110 px tall — anchor accordingly.
        meta_y = self.task.y2 - 116
        self._hline(bg, self.task.x + 20, meta_y, self.task.x2 - 20)
        # `value` may be a single string (one line) or a tuple of strings
        # (rendered on consecutive lines, right-aligned). The Model row uses
        # two lines because the per-pipeline names don't fit on one.
        rows = [
            ('MODEL',   ('YOLOv5l · hand detection',
                         'YOLO26l · kitkat detection')),
            ('STREAMS', '1 × 1080p'),
            ('SDK',     'Voyager v1.5.3'),
        ]
        ry = meta_y + 18
        for key, val in rows:
            self._text(bg, key, self.task.x + 20, ry, 11,
                       TEXT_DIM, font='mono')
            lines = val if isinstance(val, tuple) else (val,)
            for i, line in enumerate(lines):
                lw, _ = self._text_size(line, 12, font='mono')
                self._text(bg, line, self.task.x2 - 20 - lw,
                           ry + i * 18, 12, TEXT_BRIGHT, font='mono')
            ry += 22 + (len(lines) - 1) * 18

        # ── Footer (3-column: hardware | QR | logos) ────────────────────────
        self._panel(bg, self.footer)
        self._draw_footer_hardware(bg)
        self._draw_footer_qr(bg)
        self._draw_footer_logos(bg)

        return bg

    # ── footer sub-sections ────────────────────────────────────────────────

    def _draw_footer_hardware(self, bg: np.ndarray) -> None:
        """Left third of the footer: AIPU icon + 'Powered by Metis AIPU'."""
        icon_size = 72
        ix = self.footer.x + 24
        iy = self.footer.y + (self.footer.h - icon_size) // 2
        if self.chip_icon is not None:
            ch, cw = self.chip_icon.shape[:2]
            s = min(icon_size / cw, icon_size / ch)
            new_w = max(1, int(round(cw * s)))
            new_h = max(1, int(round(ch * s)))
            chip_resized = cv2.resize(self.chip_icon, (new_w, new_h),
                                       interpolation=cv2.INTER_AREA)
            ox = ix + (icon_size - new_w) // 2
            oy = iy + (icon_size - new_h) // 2
            _alpha_blit(bg, chip_resized, ox, oy)
        else:
            # Fallback: outlined square + yellow grid lines.
            cv2.rectangle(bg, (ix, iy), (ix + icon_size, iy + icon_size),
                          PANEL_BORDER, 1, cv2.LINE_AA)
            for i in range(1, 4):
                cv2.line(bg, (ix + i * icon_size // 4, iy + 8),
                         (ix + i * icon_size // 4, iy + icon_size - 8),
                         AXELERA_YELLOW, 1, cv2.LINE_AA)
                cv2.line(bg, (ix + 8, iy + i * icon_size // 4),
                         (ix + icon_size - 8, iy + i * icon_size // 4),
                         AXELERA_YELLOW, 1, cv2.LINE_AA)

        text_x = ix + icon_size + 20
        text_y = iy + 4
        self._text(bg, 'POWERED BY', text_x, text_y + 12, 11,
                   AXELERA_YELLOW, font='mono')
        self._text(bg, 'Metis AIPU · M.2 Module', text_x, text_y + 36, 18,
                   TEXT_BRIGHT, font='bold')
        self._text(bg, '214 TOPS · INT8 · 0.5W idle', text_x, text_y + 60,
                   12, TEXT_DIM, font='mono')

    def _draw_footer_qr(self, bg: np.ndarray) -> None:
        """Center column: QR square + caption. Caption mirrors the footer
        hardware block (POWERED BY / Metis AIPU): mono yellow label over a
        bold bright title, both on a single line. If self.qr_image is set
        we blit it; otherwise a stylised placeholder pattern."""
        qr_size = min(70, self.footer.h - 32)
        # Measure the caption so the QR + text block can be centered exactly.
        caption_text = 'Demo details & technical specs'
        caption_w, _ = self._text_size(caption_text, 18, font='bold')
        gap = 16
        block_w = qr_size + gap + caption_w
        qx = self.footer.x + (self.footer.w - block_w) // 2
        qy = self.footer.y + (self.footer.h - qr_size) // 2

        # No white backing card: the QR PNG is expected to carry its own
        # alpha (quiet zone alpha=0), so the footer panel shows through.
        if self.qr_image is not None:
            qr_resized = cv2.resize(self.qr_image, (qr_size, qr_size),
                                     interpolation=cv2.INTER_AREA)
            _alpha_blit(bg, qr_resized, qx, qy)
        else:
            self._draw_qr_placeholder(bg, qx, qy, qr_size)

        # Caption text — same styling as POWERED BY / Metis AIPU.
        cx = qx + qr_size + gap
        self._text(bg, 'SCAN FOR', cx, qy + 22, 11,
                   AXELERA_YELLOW, font='mono')
        self._text(bg, caption_text, cx, qy + 52, 18,
                   TEXT_BRIGHT, font='bold')

    def _draw_qr_placeholder(self, bg: np.ndarray,
                              x: int, y: int, size: int) -> None:
        """Stylised QR-ish square: 3 finder patterns + a few data squares."""
        # White background already drawn by caller.
        finder = max(6, size // 5)
        ink = (29, 29, 18)  # DARK_INK (night-blue)

        def draw_finder(cx, cy):
            cv2.rectangle(bg, (cx, cy), (cx + finder, cy + finder),
                          ink, cv2.FILLED)
            inset = max(2, finder // 6)
            cv2.rectangle(bg,
                          (cx + inset, cy + inset),
                          (cx + finder - inset, cy + finder - inset),
                          (255, 255, 255), cv2.FILLED)
            mid = max(2, finder // 3)
            cv2.rectangle(bg,
                          (cx + (finder - mid) // 2,
                           cy + (finder - mid) // 2),
                          (cx + (finder + mid) // 2,
                           cy + (finder + mid) // 2),
                          ink, cv2.FILLED)

        margin = max(2, size // 16)
        draw_finder(x + margin,                          y + margin)
        draw_finder(x + size - margin - finder,          y + margin)
        draw_finder(x + margin,                          y + size - margin - finder)

        # Sparse data cells (deterministic pattern so it looks the same each run).
        rng = np.random.default_rng(seed=0xAE)
        cell = max(2, size // 16)
        for cy in range(y + margin, y + size - margin, cell):
            for cx in range(x + margin, x + size - margin, cell):
                # Skip the finder regions.
                if ((cx - x) < margin + finder + 2 and
                    (cy - y) < margin + finder + 2):
                    continue
                if ((x + size - cx) < margin + finder + 2 and
                    (cy - y) < margin + finder + 2):
                    continue
                if ((cx - x) < margin + finder + 2 and
                    (y + size - cy) < margin + finder + 2):
                    continue
                if rng.random() < 0.5:
                    cv2.rectangle(bg, (cx, cy),
                                  (cx + cell - 1, cy + cell - 1),
                                  ink, cv2.FILLED)

    def _draw_footer_logos(self, bg: np.ndarray) -> None:
        """Right column: Axelera logo (PNG) right-aligned to the footer."""
        # Axelera logo target box. If we have the PNG, fit it preserving aspect.
        ax_box_w, ax_box_h = 260, 80
        center_y = self.footer.y + self.footer.h // 2

        # Right-align the logo block to the footer's right edge.
        ax_x = self.footer.x2 - 24 - ax_box_w
        ax_y = center_y - ax_box_h // 2
        if self.axelera_logo is not None:
            lh, lw = self.axelera_logo.shape[:2]
            # Fit inside (ax_box_w, ax_box_h) preserving aspect.
            s = min(ax_box_w / lw, ax_box_h / lh)
            new_w = max(1, int(round(lw * s)))
            new_h = max(1, int(round(lh * s)))
            logo_resized = cv2.resize(self.axelera_logo, (new_w, new_h),
                                       interpolation=cv2.INTER_AREA)
            ox = ax_x + (ax_box_w - new_w) // 2
            oy = ax_y + (ax_box_h - new_h) // 2
            _alpha_blit(bg, logo_resized, ox, oy)
        else:
            # Fallback: draw the wordmark in text.
            mark_size = 40
            mx = ax_x
            my = center_y - mark_size // 2
            cv2.rectangle(bg, (mx, my), (mx + mark_size, my + mark_size),
                          AXELERA_YELLOW, cv2.FILLED)
            aw, ah = self._text_size('A', 28, font='bold')
            self._text(bg, 'A', mx + (mark_size - aw) // 2,
                       my + (mark_size + ah) // 2,
                       28, DARK_INK, font='bold')
            self._text(bg, 'Axelera', mx + mark_size + 12, my + mark_size - 8,
                       22, TEXT_BRIGHT, font='bold')
            ax_w_text, _ = self._text_size('Axelera', 22, font='bold')
            self._text(bg, '.AI',
                       mx + mark_size + 12 + ax_w_text,
                       my + mark_size - 8,
                       22, AXELERA_YELLOW, font='bold')

    # ── per-frame render ───────────────────────────────────────────────────

    def render(self, video_bgr: np.ndarray,
               live_dot_on: bool = True) -> np.ndarray:
        out = self.bg.copy()
        self._blit_video(out, video_bgr)
        if live_dot_on:
            cv2.circle(out,
                       (self.header.x + 38,
                        self.header.y + self.header.h // 2),
                       6, AXELERA_YELLOW, cv2.FILLED)
        return out

    def _blit_video(self, out: np.ndarray, video_bgr: np.ndarray) -> None:
        """Letterbox-fit the video into the canvas region."""
        if video_bgr is None or video_bgr.size == 0:
            return
        ih, iw = video_bgr.shape[:2]
        cw, ch = self.canvas.w, self.canvas.h
        if iw == 0 or ih == 0:
            return
        scale = min(cw / iw, ch / ih)
        new_w = max(1, int(round(iw * scale)))
        new_h = max(1, int(round(ih * scale)))
        if new_w != iw or new_h != ih:
            resized = cv2.resize(video_bgr, (new_w, new_h),
                                  interpolation=cv2.INTER_LINEAR)
        else:
            resized = video_bgr
        ox = self.canvas.x + (cw - new_w) // 2
        oy = self.canvas.y + (ch - new_h) // 2
        out[oy:oy + new_h, ox:ox + new_w] = resized


# ────────────────────────────────────────────────────────────────────────────
# Node
# ────────────────────────────────────────────────────────────────────────────

class DemoScreenNode(Node):
    def __init__(self):
        super().__init__('demo_screen')

        # ── Parameters (window sizing + topics only) ────────────────────────
        self.declare_parameter('window_width',  1920)
        self.declare_parameter('window_height', 1080)
        self.declare_parameter('window_title',  'Axelera AI · Live Inference')
        self.declare_parameter('window_fullscreen', False)
        self.declare_parameter('image_topic',            'image_raw')
        self.declare_parameter('detections_topic',       'detections')
        self.declare_parameter('color_detections_topic', '/color_detections')
        self.declare_parameter('pick_roi_topic',         '/pick_roi_polygon')

        image_topic = str(self.get_parameter('image_topic').value)
        det_topic   = str(self.get_parameter('detections_topic').value)
        color_topic = str(self.get_parameter('color_detections_topic').value)
        roi_topic   = str(self.get_parameter('pick_roi_topic').value)

        # ── State ───────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self._det_lock = threading.Lock()
        self._latest_dets: Detection2DArray | None = None
        self._latest_dets_time = self.get_clock().now()
        self._latest_color_dets: Detection2DArray | None = None
        self._latest_color_dets_time = self.get_clock().now()
        self._latest_pick_roi: PolygonStamped | None = None

        # Frame counter for the header live-dot blink cadence.
        self._frame_count = 0

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(
            Detection2DArray, det_topic, self._on_detections, 10)
        self.create_subscription(
            Detection2DArray, color_topic, self._on_color_detections, 10)

        roi_qos = QoSProfile(depth=1)
        roi_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            PolygonStamped, roi_topic, self._on_pick_roi, roi_qos)

        # ── Window + renderer ───────────────────────────────────────────────
        w = int(self.get_parameter('window_width').value)
        h = int(self.get_parameter('window_height').value)
        self._window_name = str(self.get_parameter('window_title').value)
        self._window_opened = False

        log = self.get_logger()
        logo = _load_image(AXELERA_LOGO_PATH, log)
        qr   = _load_image(QR_CODE_PATH,      log)
        chip = _load_image(CHIP_ICON_PATH,    log)
        self.renderer = TemplateRenderer(
            width=w, height=h, axelera_logo=logo,
            qr_image=qr, chip_icon=chip)

        font_status = ('TTF' if self.renderer.ft_sans is not None
                       else 'Hershey (no freetype)')
        log.info(
            f'Demo screen up: {w}×{h}, font={font_status}, '
            f'axelera_logo={"PNG" if logo is not None else "text fallback"}, '
            f'qr={"PNG" if qr is not None else "placeholder"}, '
            f'chip={"PNG" if chip is not None else "grid fallback"}.')

    # ── ROS callbacks ───────────────────────────────────────────────────────

    def _on_detections(self, msg: Detection2DArray):
        with self._det_lock:
            self._latest_dets = msg
            self._latest_dets_time = self.get_clock().now()

    def _on_color_detections(self, msg: Detection2DArray):
        with self._det_lock:
            self._latest_color_dets = msg
            self._latest_color_dets_time = self.get_clock().now()

    def _on_pick_roi(self, msg: PolygonStamped):
        with self._det_lock:
            self._latest_pick_roi = msg

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        annotated = frame.copy()
        now = self.get_clock().now()
        with self._det_lock:
            dets = self._latest_dets
            dets_age = (now - self._latest_dets_time).nanoseconds / 1e9
            color_dets = self._latest_color_dets
            color_age = (now - self._latest_color_dets_time).nanoseconds / 1e9
            pick_roi = self._latest_pick_roi

        if dets is not None and dets_age <= DET_OVERLAY_STALE_S:
            self._draw_yolo(annotated, dets)
        if pick_roi is not None and len(pick_roi.polygon.points) >= 2:
            self._draw_pick_roi(annotated, pick_roi)
        if color_dets is not None and color_age <= DET_OVERLAY_STALE_S:
            self._draw_color(annotated, color_dets)

        self._render_window(annotated)

    # ── Window dispatch ─────────────────────────────────────────────────────

    def _render_window(self, annotated: np.ndarray) -> None:
        self._frame_count += 1
        live_on = (self._frame_count // 24) % 2 == 0  # blink ~1.6s cadence
        frame = self.renderer.render(annotated, live_dot_on=live_on)

        if not self._window_opened:
            # The reliable pattern across GTK / Qt / Wayland-Xwayland: create
            # the window, do one imshow so the WM actually tracks it, THEN
            # toggle the FULLSCREEN hint. Setting the property before the
            # first imshow is a no-op on several backends.
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.imshow(self._window_name, frame)
            cv2.waitKey(1)
            if bool(self.get_parameter('window_fullscreen').value):
                cv2.setWindowProperty(
                    self._window_name, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN)
                # Some WMs only honour fullscreen after a second hint cycle.
                cv2.waitKey(1)
                cv2.setWindowProperty(
                    self._window_name, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN)
            else:
                cv2.resizeWindow(self._window_name,
                                  self.renderer.w, self.renderer.h)
            self._window_opened = True
        cv2.imshow(self._window_name, frame)
        cv2.waitKey(1)

    # ── Overlay drawing (mirrors live_inference_node.cpp) ───────────────────

    @staticmethod
    def _color_for_score(score: float):
        if score >= 0.7:
            return (0, 255, 0)
        if score >= 0.4:
            return (0, 165, 255)
        return (0, 0, 255)

    def _draw_yolo(self, img: np.ndarray, dets: Detection2DArray) -> None:
        for det in dets.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            x1 = int(np.clip(cx - w / 2.0, 0, img.shape[1] - 1))
            y1 = int(np.clip(cy - h / 2.0, 0, img.shape[0] - 1))
            x2 = int(np.clip(cx + w / 2.0, 0, img.shape[1] - 1))
            y2 = int(np.clip(cy + h / 2.0, 0, img.shape[0] - 1))
            color = self._color_for_score(float(hyp.score))
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            label = f'{hyp.class_id} {float(hyp.score):.2f}'
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw, y1),
                          color, cv2.FILLED)
            cv2.putText(img, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0),
                        1, cv2.LINE_AA)
            cv2.circle(img, ((x1 + x2) // 2, (y1 + y2) // 2),
                       3, color, cv2.FILLED)

    def _draw_pick_roi(self, img: np.ndarray, roi: PolygonStamped) -> None:
        pts = np.array(
            [[int(p.x), int(p.y)] for p in roi.polygon.points], dtype=np.int32)
        color = (0, 255, 255)
        cv2.polylines(img, [pts], isClosed=True, color=color,
                      thickness=2, lineType=cv2.LINE_AA)
        x0, y0 = int(pts[0][0]) + 6, max(0, int(pts[0][1]) - 8)
        cv2.putText(img, 'ROI', (x0, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

    def _draw_color(self, img: np.ndarray, dets: Detection2DArray) -> None:
        for det in dets.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)
            sz = 30
            color = (0, 0, 255)
            cv2.line(img, (cx - sz, cy), (cx + sz, cy), color, 2)
            cv2.line(img, (cx, cy - sz), (cx, cy + sz), color, 2)
            cv2.circle(img, (cx, cy), 6, color, -1)
            cv2.putText(img, hyp.class_id, (cx + 20, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

    # ── Shutdown ────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        if self._window_opened:
            try:
                cv2.destroyWindow(self._window_name)
            except cv2.error:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = DemoScreenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
