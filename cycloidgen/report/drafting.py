"""Engineering drawing sheets: views, dimensions and a title block, on A3.

A sheet is put together as data first - geometry in the part's millimetres,
annotations carrying the text they print - and drawn to PDF second.  The order
matters to the tests rather than to the drawing: a number on a drawing has to
be the number the part is made to, and a test cannot read it back out of a PDF.

Everything is drawn as vectors in the PDF's base font.  The tolerancing symbols
that font does not have - position, the circled M, a datum's triangle - are
drawn as lines, so the sheet needs no font of its own and reads the same in
every viewer.  Dimensions are in millimetres and the projection is ISO first
angle: a section seen from the left is drawn on the right.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

from reportlab.lib.units import mm
from reportlab.pdfgen import canvas as pdfcanvas

from .. import __version__

__all__ = ["PAPER", "Callout", "Circle", "CuttingPlane", "Datum", "Frame",
           "Hatch", "Linear", "Polyline", "Segment", "Sheet", "View",
           "write_sheet_pdf"]

#: A3 landscape, millimetres.
PAPER = (420.0, 297.0)
#: ISO 5457: a 20 mm filing margin on the left, 10 mm elsewhere.
BORDER = (20.0, 10.0, 10.0, 10.0)            # left, bottom, right, top
#: The title block, bottom right inside the border: ISO 7200's 180 mm width.
TITLE = (230.0, 10.0, 180.0, 46.0)            # x, y, width, height

_TEXT = 3.5             # dimension and note text, mm
_ARROW = 3.0            # arrowhead length, mm
_WIDE, _THIN = 0.5, 0.25
_PT = 72.0 / 25.4       # points per millimetre

VISIBLE, THIN, HIDDEN, CENTRE = "visible", "thin", "hidden", "centre"


# ---------------------------------------------------------------- geometry

Point = tuple[float, float]


@dataclass(frozen=True)
class Circle:
    centre: Point
    radius: float
    kind: str = VISIBLE


@dataclass(frozen=True)
class Segment:
    a: Point
    b: Point
    kind: str = VISIBLE


@dataclass(frozen=True)
class Polyline:
    points: tuple[Point, ...]
    closed: bool = True
    kind: str = VISIBLE


@dataclass(frozen=True)
class Hatch:
    """Cut material in a section: rectangles ``(u0, v0, u1, v1)``, hatched at
    45 degrees.  Its outline is drawn separately, as segments, so that two cells
    side by side read as one piece of metal and not as two."""

    cells: tuple[tuple[float, float, float, float], ...]


# ------------------------------------------------------------- annotations


@dataclass(frozen=True)
class Frame:
    """A position tolerance, ISO 1101: the zone's diameter, whether it applies
    at maximum material, and the datums it is measured from."""

    tolerance: float
    mmc: bool = False
    datums: tuple[str, ...] = ()


@dataclass(frozen=True)
class Linear:
    """A dimension between two points along one paper axis.

    ``axis`` is ``"h"`` for a horizontal dimension line, ``"v"`` for a
    vertical one; ``offset`` is where the line runs, in paper millimetres from
    ``a`` across the axis - positive is right of, or above, ``a``.
    """

    a: Point
    b: Point
    axis: str
    offset: float
    text: str
    #: Where the text goes when it does not fit between the arrows: past the
    #: ``b`` end or past the ``a`` end.
    spill: str = "b"


@dataclass(frozen=True)
class Callout:
    """A leader from a feature to its text, and the tolerance frame under it.

    ``angle`` is the direction the leader leaves ``target`` in, degrees on the
    paper; ``reach`` is how far it goes, paper millimetres, before the shoulder
    that the text sits on.
    """

    target: Point
    lines: tuple[str, ...]
    angle: float = 45.0
    reach: float = 18.0
    frame: Frame | None = None
    #: Whether the arrow stops on the outline (a hole's edge) or on the centre.
    dot: bool = False
    #: Where the shoulder starts, paper millimetres from the view's centre -
    #: given instead of ``angle`` and ``reach`` when the text has a place to go.
    knee: Point | None = None


@dataclass(frozen=True)
class Datum:
    """A datum feature: a filled triangle on the feature, a box with the letter."""

    target: Point
    angle: float
    letter: str


@dataclass(frozen=True)
class CuttingPlane:
    """Where a section is cut: chain line between ``a`` and ``b`` with thick
    ends, and arrows at each end pointing the way it is looked at."""

    a: Point
    b: Point
    letter: str
    toward: float           # paper direction of the arrows, degrees


@dataclass
class View:
    """One view on the sheet, drawn at ``scale`` about ``centre`` (paper mm)."""

    title: str
    centre: Point
    scale: float
    items: list = field(default_factory=list)
    #: Where the title goes, paper millimetres; above the geometry if None.
    title_at: Point | None = None

    def paper(self, p: Point) -> Point:
        return (self.centre[0] + self.scale * p[0],
                self.centre[1] + self.scale * p[1])


@dataclass
class Sheet:
    """Everything printed on one sheet."""

    part: str
    number: str
    material: str
    process: str
    quantity: int
    mass_g: float
    scale: str
    general: str
    views: list[View] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    design: str = ""
    warning: str = ""


# ----------------------------------------------------------------- drawing


class _Pen:
    """The reportlab canvas, in millimetres."""

    def __init__(self, path: Path) -> None:
        self.c = pdfcanvas.Canvas(str(path), pagesize=(PAPER[0] * mm, PAPER[1] * mm))
        self.c.setTitle("drawing")
        self.c.setCreator(f"cycloidgen {__version__}")

    def style(self, kind: str) -> None:
        c = self.c
        c.setLineWidth((_WIDE if kind == VISIBLE else _THIN) * mm)
        if kind == HIDDEN:
            c.setDash([3.0 * mm, 1.5 * mm])
        elif kind == CENTRE:
            c.setDash([9.0 * mm, 1.5 * mm, 1.0 * mm, 1.5 * mm])
        else:
            c.setDash([])

    def line(self, a: Point, b: Point, kind: str = THIN) -> None:
        self.style(kind)
        self.c.line(a[0] * mm, a[1] * mm, b[0] * mm, b[1] * mm)

    def circle(self, c: Point, r: float, kind: str = VISIBLE) -> None:
        self.style(kind)
        self.c.circle(c[0] * mm, c[1] * mm, r * mm, stroke=1, fill=0)

    def poly(self, pts: Sequence[Point], closed: bool, kind: str) -> None:
        self.style(kind)
        path = self.c.beginPath()
        path.moveTo(pts[0][0] * mm, pts[0][1] * mm)
        for x, y in pts[1:]:
            path.lineTo(x * mm, y * mm)
        if closed:
            path.close()
        self.c.drawPath(path, stroke=1, fill=0)

    def arrow(self, tip: Point, angle: float) -> None:
        """A filled arrowhead with its tip at ``tip``, pointing along ``angle``."""
        a = math.radians(angle)
        back = (tip[0] - _ARROW * math.cos(a), tip[1] - _ARROW * math.sin(a))
        nx, ny = -math.sin(a) * _ARROW / 6.0, math.cos(a) * _ARROW / 6.0
        path = self.c.beginPath()
        path.moveTo(tip[0] * mm, tip[1] * mm)
        path.lineTo((back[0] + nx) * mm, (back[1] + ny) * mm)
        path.lineTo((back[0] - nx) * mm, (back[1] - ny) * mm)
        path.close()
        self.style(THIN)
        self.c.drawPath(path, stroke=0, fill=1)

    def text(self, p: Point, s: str, size: float = _TEXT, anchor: str = "left",
             angle: float = 0.0, bold: bool = False) -> None:
        c = self.c
        c.saveState()
        c.translate(p[0] * mm, p[1] * mm)
        c.rotate(angle)
        c.setFont("Helvetica-Bold" if bold else "Helvetica", size * _PT)
        {"left": c.drawString, "centre": c.drawCentredString,
         "right": c.drawRightString}[anchor](0, 0, s)
        c.restoreState()

    def width(self, s: str, size: float = _TEXT, bold: bool = False) -> float:
        return self.c.stringWidth(s, "Helvetica-Bold" if bold else "Helvetica",
                                  size * _PT) / _PT

    def rect(self, x: float, y: float, w: float, h: float,
             kind: str = THIN) -> None:
        self.style(kind)
        self.c.rect(x * mm, y * mm, w * mm, h * mm, stroke=1, fill=0)


def _frame(pen: _Pen, x: float, y: float, f: Frame) -> float:
    """Draw a position frame with its top-left at ``(x, y)``; its width back."""
    h = 6.0
    cells: list[tuple[str, float]] = [("", h)]
    tol = f"Ø{f.tolerance:.2f}".rstrip("0").rstrip(".") if f.tolerance else "Ø0"
    tol_w = pen.width(tol) + (5.0 if f.mmc else 0.0) + 3.0
    cells.append((tol, tol_w))
    cells += [(d, h) for d in f.datums]
    cx = x
    for i, (s, w) in enumerate(cells):
        pen.rect(cx, y - h, w, h)
        if i == 0:
            # Position: a circle and a cross, drawn rather than set in a font
            # that does not have the character.
            m = (cx + w / 2.0, y - h / 2.0)
            pen.circle(m, 1.6, THIN)
            pen.line((m[0] - 2.4, m[1]), (m[0] + 2.4, m[1]))
            pen.line((m[0], m[1] - 2.4), (m[0], m[1] + 2.4))
        elif i == 1:
            pen.text((cx + 1.5, y - h + 1.6), s)
            if f.mmc:
                mx = cx + 1.5 + pen.width(s) + 2.4
                pen.circle((mx, y - h / 2.0), 1.9, THIN)
                pen.text((mx, y - h + 1.95), "M", size=2.5, anchor="centre")
        else:
            pen.text((cx + w / 2.0, y - h + 1.6), s, anchor="centre")
        cx += w
    return cx - x


def _draw_linear(pen: _Pen, view: View, d: Linear) -> None:
    a, b = view.paper(d.a), view.paper(d.b)
    if d.axis == "v":
        x = a[0] + d.offset
        for p in (a, b):
            sign = 1.0 if x > p[0] else -1.0
            pen.line((p[0] + sign * 1.0, p[1]), (x + sign * 2.0, p[1]))
        lo, hi = sorted((a[1], b[1]))
        pen.line((x, lo), (x, hi))
        if hi - lo > 2.5 * _ARROW:
            pen.arrow((x, lo), -90.0)
            pen.arrow((x, hi), 90.0)
        else:
            pen.line((x, lo - 2 * _ARROW), (x, hi + 2 * _ARROW))
            pen.arrow((x, lo), 90.0)
            pen.arrow((x, hi), -90.0)
        pen.text((x - 1.0, (lo + hi) / 2.0), d.text, anchor="centre", angle=90.0)
    else:
        y = a[1] + d.offset
        for p in (a, b):
            sign = 1.0 if y > p[1] else -1.0
            pen.line((p[0], p[1] + sign * 1.0), (p[0], y + sign * 2.0))
        lo, hi = sorted((a[0], b[0]))
        pen.line((lo, y), (hi, y))
        if hi - lo > 2.5 * _ARROW + pen.width(d.text):
            pen.arrow((lo, y), 180.0)
            pen.arrow((hi, y), 0.0)
            pen.text(((lo + hi) / 2.0, y + 1.0), d.text, anchor="centre")
        else:
            pen.line((lo - 2 * _ARROW, y), (hi + 2 * _ARROW, y))
            pen.arrow((lo, y), 0.0)
            pen.arrow((hi, y), 180.0)
            past_b = (b[0] >= a[0]) == (d.spill == "b")
            if past_b:
                pen.text((hi + 2 * _ARROW + 1.0, y + 1.0), d.text)
            else:
                pen.text((lo - 2 * _ARROW - 1.0, y + 1.0), d.text, anchor="right")


def _draw_callout(pen: _Pen, view: View, c: Callout) -> None:
    tip = view.paper(c.target)
    if c.knee is not None:
        knee = (view.centre[0] + c.knee[0], view.centre[1] + c.knee[1])
    else:
        a = math.radians(c.angle)
        knee = (tip[0] + c.reach * math.cos(a), tip[1] + c.reach * math.sin(a))
    right = knee[0] >= tip[0]
    width = max(pen.width(s) for s in c.lines) + 2.0
    if c.frame is not None:
        width = max(width, 12.0 + 6.0 * len(c.frame.datums) + pen.width("Ø0.00") + 6.0)
    # Kept on the paper: a block that would run off the drawing area slides
    # along its shoulder until it fits.
    lo, hi = BORDER[0] + 2.0, PAPER[0] - BORDER[2] - 2.0
    if right and knee[0] + width > hi:
        knee = (hi - width, knee[1])
    if not right and knee[0] - width < lo:
        knee = (lo + width, knee[1])
    end = (knee[0] + (width if right else -width), knee[1])
    pen.line(tip, knee)
    pen.line(knee, end)
    if c.dot:
        pen.c.circle(tip[0] * mm, tip[1] * mm, 0.6 * mm, stroke=0, fill=1)
    else:
        pen.arrow(tip, math.degrees(math.atan2(tip[1] - knee[1], tip[0] - knee[0])))
    x0 = min(knee[0], end[0]) + 1.0
    step = _TEXT + 1.0
    if knee[1] > tip[1]:
        # Above the feature: the block stands on the shoulder, away from the
        # part, frame lowest and the first line at the top.
        y = knee[1] + 1.0
        if c.frame is not None:
            y += 7.5
            _frame(pen, x0, y - 1.0, c.frame)
        for s in reversed(c.lines):
            pen.text((x0, y), s)
            y += step
        return
    # Below it: the first line on the shoulder, the rest and the frame beneath.
    pen.text((x0, knee[1] + 1.0), c.lines[0])
    y = knee[1] - 1.0
    for s in c.lines[1:]:
        y -= step
        pen.text((x0, y), s)
    if c.frame is not None:
        _frame(pen, x0, y - 1.5, c.frame)


def _draw_datum(pen: _Pen, view: View, d: Datum) -> None:
    tip = view.paper(d.target)
    a = math.radians(d.angle)
    ux, uy = math.cos(a), math.sin(a)
    # The triangle, base on the feature, then a stem to the box.
    nx, ny = -uy, ux
    base = 1.8
    path = pen.c.beginPath()
    path.moveTo((tip[0] + nx * base) * mm, (tip[1] + ny * base) * mm)
    path.lineTo((tip[0] - nx * base) * mm, (tip[1] - ny * base) * mm)
    path.lineTo((tip[0] + ux * 3.0) * mm, (tip[1] + uy * 3.0) * mm)
    path.close()
    pen.style(THIN)
    pen.c.drawPath(path, stroke=1, fill=1)
    stem = (tip[0] + ux * 9.0, tip[1] + uy * 9.0)
    pen.line((tip[0] + ux * 3.0, tip[1] + uy * 3.0), stem)
    side = 6.0
    bx, by = stem[0] - side / 2.0 + ux * side / 2.0, stem[1] - side / 2.0 + uy * side / 2.0
    pen.rect(bx, by, side, side)
    pen.text((bx + side / 2.0, by + 1.6), d.letter, anchor="centre")


def _draw_cut(pen: _Pen, view: View, k: CuttingPlane) -> None:
    a, b = view.paper(k.a), view.paper(k.b)
    pen.line(a, b, CENTRE)
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    ux, uy = (b[0] - a[0]) / d, (b[1] - a[1]) / d
    t = math.radians(k.toward)
    for end, inward in ((a, 1.0), (b, -1.0)):
        stub = (end[0] + inward * ux * 6.0, end[1] + inward * uy * 6.0)
        pen.line(end, stub, VISIBLE)
        tail = (end[0] + 9.0 * math.cos(t), end[1] + 9.0 * math.sin(t))
        pen.line(end, tail)
        pen.arrow(tail, k.toward)
        pen.text((tail[0] + 2.0 * math.cos(t), tail[1] - 1.5), k.letter, size=5.0,
                 anchor="left" if math.cos(t) >= 0 else "right", bold=True)


def _draw_hatch(pen: _Pen, view: View, h: Hatch) -> None:
    c = pen.c
    c.saveState()
    path = c.beginPath()
    xs, ys = [], []
    for u0, v0, u1, v1 in h.cells:
        (x0, y0), (x1, y1) = view.paper((u0, v0)), view.paper((u1, v1))
        x0, x1 = sorted((x0, x1))
        y0, y1 = sorted((y0, y1))
        path.rect(x0 * mm, y0 * mm, (x1 - x0) * mm, (y1 - y0) * mm)
        xs += [x0, x1]
        ys += [y0, y1]
    c.clipPath(path, stroke=0, fill=0)
    pen.style(THIN)
    c.setLineWidth(0.18 * mm)
    lo_x, hi_x, lo_y, hi_y = min(xs), max(xs), min(ys), max(ys)
    step = 2.0
    k = lo_x - (hi_y - lo_y)
    while k < hi_x:
        c.line(k * mm, lo_y * mm, (k + hi_y - lo_y) * mm, hi_y * mm)
        k += step
    c.restoreState()


def _draw_view(pen: _Pen, view: View) -> None:
    for item in view.items:
        if isinstance(item, Hatch):
            _draw_hatch(pen, view, item)
    for item in view.items:
        if isinstance(item, Circle):
            pen.circle(view.paper(item.centre), item.radius * view.scale, item.kind)
        elif isinstance(item, Segment):
            pen.line(view.paper(item.a), view.paper(item.b), item.kind)
        elif isinstance(item, Polyline):
            pen.poly([view.paper(p) for p in item.points], item.closed, item.kind)
    for item in view.items:
        if isinstance(item, Linear):
            _draw_linear(pen, view, item)
        elif isinstance(item, Callout):
            _draw_callout(pen, view, item)
        elif isinstance(item, Datum):
            _draw_datum(pen, view, item)
        elif isinstance(item, CuttingPlane):
            _draw_cut(pen, view, item)


def _projection_symbol(pen: _Pen, x: float, y: float) -> None:
    """ISO 5456-2's first-angle symbol: a truncated cone's side view, small end
    away from its end view, which sits on the right."""
    pen.poly([(x, y + 3.0), (x + 10.0, y + 1.0), (x + 10.0, y + 9.0), (x, y + 7.0)],
             True, THIN)
    pen.line((x - 1.5, y + 5.0), (x + 11.5, y + 5.0), CENTRE)
    pen.circle((x + 18.0, y + 5.0), 4.0, THIN)
    pen.circle((x + 18.0, y + 5.0), 2.0, THIN)


def _title_block(pen: _Pen, s: Sheet) -> None:
    x, y, w, h = TITLE
    pen.rect(x, y, w, h, VISIBLE)
    # Top band: what the part is and its number.
    pen.line((x, y + h - 14.0), (x + w, y + h - 14.0))
    pen.text((x + 3.0, y + h - 9.0), s.part, size=6.0, bold=True)
    pen.text((x + w - 3.0, y + h - 6.0), s.number, size=3.5, anchor="right")
    pen.text((x + w - 3.0, y + h - 11.5), s.design, size=2.5, anchor="right")
    cells = [
        ("Material", s.material), ("Process", s.process),
        ("Quantity", f"{s.quantity}"), ("Mass", f"{s.mass_g:.1f} g"),
        ("Scale", s.scale), ("Sheet", "A3, mm"),
        ("Tolerancing", "ISO 8015"), ("General", s.general),
        ("Drawn by", f"cycloidgen {__version__}"), ("Date", date.today().isoformat()),
    ]
    cols, row_h = 2, 6.4
    col_w = (w - 34.0) / cols
    for i, (label, value) in enumerate(cells):
        cx = x + (i % cols) * col_w
        cy = y + h - 14.0 - (i // cols + 1) * row_h
        pen.rect(cx, cy, col_w, row_h)
        pen.text((cx + 1.2, cy + row_h - 2.2), label, size=1.8)
        pen.text((cx + 1.2, cy + 1.1), value, size=2.8)
    px = x + cols * col_w
    pen.rect(px, y, w - cols * col_w, h - 14.0)
    _projection_symbol(pen, px + 5.0, y + 12.0)
    pen.text((px + 17.0, y + 6.0), "ISO first angle", size=2.2, anchor="centre")


def _notes(pen: _Pen, s: Sheet) -> None:
    x, y = BORDER[0] + 6.0, TITLE[1] + TITLE[3]
    pen.text((x, y), "NOTES", size=3.5, bold=True)
    width = TITLE[0] - x - 8.0
    for i, note in enumerate(s.notes, start=1):
        words, line, first = note.split(), "", True
        lines: list[str] = []
        for word in words:
            trial = f"{line} {word}".strip()
            if pen.width(trial, 2.8) > width - 6.0 and line:
                lines.append(line)
                line = word
            else:
                line = trial
        lines.append(line)
        for text in lines:
            y -= 4.0
            pen.text((x + 6.0, y), text, size=2.8)
            if first:
                pen.text((x, y), f"{i}.", size=2.8)
                first = False


def write_sheet_pdf(sheet: Sheet, path: str | Path) -> Path:
    """Draw ``sheet`` to a one-page PDF at ``path``."""
    path = Path(path)
    pen = _Pen(path)
    pen.c.setTitle(f"{sheet.part} - {sheet.number}")
    left, bottom, right, top = BORDER
    pen.rect(left, bottom, PAPER[0] - left - right, PAPER[1] - bottom - top, VISIBLE)
    for view in sheet.views:
        _draw_view(pen, view)
        at = view.title_at or (view.centre[0], view.centre[1] + view_extent(view)[3] + 6.0)
        pen.text(at, view.title, size=4.0, anchor="centre", bold=True)
    _title_block(pen, sheet)
    _notes(pen, sheet)
    if sheet.warning:
        pen.text((PAPER[0] / 2.0, PAPER[1] - top - 6.0), sheet.warning, size=2.6,
                 anchor="centre")
    pen.c.showPage()
    pen.c.save()
    return path


def view_extent(view: View) -> tuple[float, float, float, float]:
    """``(left, bottom, right, top)`` of a view's geometry, paper millimetres
    from its centre - what its title is placed above."""
    xs, ys = [0.0], [0.0]
    for item in view.items:
        if isinstance(item, Circle):
            for sx in (-1, 1):
                xs.append(view.scale * (item.centre[0] + sx * item.radius))
                ys.append(view.scale * (item.centre[1] + sx * item.radius))
        elif isinstance(item, Segment):
            for p in (item.a, item.b):
                xs.append(view.scale * p[0])
                ys.append(view.scale * p[1])
        elif isinstance(item, Polyline):
            for p in item.points:
                xs.append(view.scale * p[0])
                ys.append(view.scale * p[1])
    return min(xs), min(ys), max(xs), max(ys)
