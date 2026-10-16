"""Dimensioned drawings: one A3 sheet per part, for the shop that makes it.

The cutting DXFs are geometry - an outline and its holes, which is what a laser
wants - and the STEP file is a model.  Neither says how deep a seat is bored,
which face it is bored from, how close a hole has to be to where it is drawn,
or what the bore a bearing is pressed into is held to.  A shop quotes from a
drawing, and those are the things a drawing is for.

The end plates come first because their whole job is holes: two bearing-seat
bores and the patterns the drive is bolted together and onto a motor by.

Every number on a sheet is read from the same properties the solid is built
from, and gathered into a :class:`PlateFeatures` before anything is laid out,
so that the tests can hold the drawing to the part without reading a PDF.
What the drawing adds that the model does not have is tolerances, and each one
is derived rather than chosen:

- a bearing seat's fit from whether its load turns round the outer ring -
  see :mod:`cycloidgen.core.fits`;
- a register's from its job, which is to locate a spigot: H7;
- a hole pattern's position tolerance from the hole and the screw through it -
  the clearance a screw has is the error the holes may have between them, all
  of it where the screw floats through clearance holes in every part, half of
  it where it is fixed in a thread at one end.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

from .. import notice
from ..analysis.bearings import bearing_placements, bearing_schedule, seat_bores
from ..core.fasteners import size_for_hole
from ..core.fits import SeatFit, hole_limits, housing_fit
from ..core.spec import GearSpec, OutputMember, Process
from ..report.drafting import (
    CENTRE,
    HIDDEN,
    THIN,
    Callout,
    Circle,
    CuttingPlane,
    Datum,
    Frame,
    Hatch,
    Linear,
    Polyline,
    Segment,
    Sheet,
    View,
    write_sheet_pdf,
)
from .manifest import sheet_names

__all__ = ["Pattern", "PlateFeatures", "Seat", "plate_features", "plate_sheet",
           "write_sheets"]

#: ISO 2768 general tolerance class each machining process works to.
_GENERAL = {Process.CNC: "ISO 2768-mK", Process.EDM: "ISO 2768-fH"}

#: The plates this module draws.
_PLATES = ("input_end_plate", "output_end_plate")
_TITLES = {"input_end_plate": "Input end plate",
           "output_end_plate": "Output end plate"}

#: The corner left where a seat meets its lip, as the solid rounds it.
_INSIDE_RADIUS = 0.2


# --------------------------------------------------------------- the numbers


@dataclass(frozen=True)
class Seat:
    """A bearing seat: bored from the inside face, to the bearing."""

    bearing: str
    diameter: float
    #: From the inside face; the plate's thickness when it goes right through.
    depth: float
    fit: SeatFit | None


@dataclass(frozen=True)
class Pattern:
    """A set of holes drilled for one kind of screw."""

    label: str
    centres: tuple[tuple[float, float], ...]
    hole: float
    #: How the pattern is stated: a bolt circle's diameter, or a square's side.
    circle: float | None
    square: float | None
    screw: str
    #: Position tolerance at maximum material, the zone's diameter - None where
    #: no screw size is known to derive it from.
    position: float | None
    #: Whether the screw is held in a thread at its far end.
    fixed: bool
    datums: tuple[str, ...]
    #: What is cut round each hole in the inside face: ``(kind, size, depth)``
    #: with kind ``"counterbore"`` or ``"hex pocket"``.
    recess: tuple[str, float, float] | None = None
    #: Degrees from the tie-bolt pattern, for a pattern turned against it.
    phase: float = 0.0
    #: For a pattern that is not equally spaced: each hole's angle, degrees.
    angles: tuple[float, ...] = ()
    #: The fit the holes are reamed to, for a pattern that locates.
    fit: str = ""


@dataclass(frozen=True)
class Register:
    """A recess a spigot locates in."""

    diameter: float
    depth: float
    fit: tuple[int, int]


@dataclass(frozen=True)
class Mount:
    """The motor's mount, standing off the outside face: tube, then flange."""

    tube_bore: float
    tube_outer: float
    height: float
    flange: float
    width: float
    corner: float
    hole: float
    register: Register | None
    #: A round motor's flange is a disc; a square frame's an octagon.
    round: bool
    #: Whether the motor has a spigot for the flange's hole to locate.
    spigot: bool
    window_width: float
    #: Where each window runs, from the plate's outside face.
    window: tuple[float, float]


@dataclass(frozen=True)
class PlateFeatures:
    """Everything a plate's sheet states, in millimetres, before layout.

    The plate's own drawing frame: ``z`` from the inside face (datum A), which
    bolts to the ring housing and runs against the discs; ``x, y`` as the
    solid's, seen from the outside.
    """

    name: str
    title: str
    outside_diameter: float
    thickness: float
    seat: Seat | None
    #: The hole that goes all the way through: the lip's, or the bore's.
    through: float
    #: The lip that holds the bearing in: ``(z0, z1)`` from the inside face.
    lip: tuple[float, float] | None
    register: Register | None
    patterns: tuple[Pattern, ...]
    mount: Mount | None
    edge_break: float
    printed: bool


def _turning_rpm(spec: GearSpec) -> float:
    """How fast the crank's load turns relative to the housing's plates.

    Every load inside the drive - the crank reaction on the shaft supports, the
    output pins' resultant on the carrier - points where the crank points, so
    it turns with the crank: relative to the plates that is the crank against
    the ring, whichever member is grounded.
    """
    return abs(spec.shaft_spin - spec.ring_spin) * spec.crank_rate * spec.input_rpm


def _position(hole: float, screw_d: float | None, fixed: bool) -> float | None:
    """The position tolerance a hole pattern can have and still take its screws.

    Two parts with holes ``H`` and a screw ``F`` through both line up however
    far apart the holes' axes are, up to ``H - F``: the screw needs a circle of
    ``F`` common to both holes, and that is what is left of two circles of
    ``H`` that far apart.  Each hole may be off by half of that in any
    direction, which is a zone of diameter ``H - F`` - the floating fastener.
    Where the far end is a thread there is only one clearance to share between
    the two parts, so each gets half - the fixed fastener.
    """
    if screw_d is None or hole <= screw_d:
        return None
    t = hole - screw_d
    return round(t / 2.0 if fixed else t, 3)


def plate_features(spec: GearSpec, name: str) -> PlateFeatures:
    """The numbers on ``name``'s sheet, read from what the solid is built of."""
    if name not in _PLATES:
        raise ValueError(f"no sheet for {name!r}")
    choices = bearing_schedule(spec)
    bores = seat_bores(spec, choices)
    placements = bearing_placements(spec, choices)
    printed = spec.printed
    t = spec.plate_thickness
    inside = spec.barrel_top if name == "input_end_plate" else spec.barrel_bottom
    toward_outside = 1.0 if name == "input_end_plate" else -1.0
    bore = bores.input_plate if name == "input_end_plate" else bores.output_plate

    # ---- the register: input plate, a motor bolted flush on it --------------
    register = None
    direct_motor = (name == "input_end_plate" and spec.has_motor_face
                    and not spec.coupled and not spec.ground_frame_fitted)
    if direct_motor and spec.motor_face.pilot_diameter > bore:
        frame = spec.motor_face
        register = Register(frame.pilot_diameter, frame.pilot_depth,
                            hole_limits(frame.pilot_diameter, "H7"))

    # ---- the seat and its lip -----------------------------------------------
    placement = next((p for p in placements if p.host == name
                      and p.name in ("bearing_shaft_input", "bearing_output_input",
                                     "bearing_output_main")), None)
    lip = next((s for s in bores.shoulders if s.host == name), None)
    lip_z = None
    seat = None
    through = bore
    if lip is not None:
        lo, hi = sorted(((lip.z0 - inside) * toward_outside,
                         (lip.z1 - inside) * toward_outside))
        lip_z = (lo, hi)
        through = lip.bore
    if placement is not None:
        choice = next(c for c in choices if c.role == placement.role)
        fit = None
        if not printed:
            fit = housing_fit(placement.outer, _turning_rpm(spec), choice.load_N,
                              1000.0 * choice.bearing.C_kN if choice.bearing else 0.0)
        depth = lip_z[0] if lip_z is not None else t
        seat = Seat(placement.catalogue, placement.outer, depth, fit)

    # ---- hole patterns --------------------------------------------------------
    patterns: list[Pattern] = []
    if spec.housing_bolt_count:
        size = spec.tie_bolt_size
        n = spec.housing_bolt_count
        r = spec.housing_bolt_radius
        patterns.append(Pattern(
            label="tie bolts",
            centres=tuple((r * math.cos(2 * math.pi * k / n),
                           r * math.sin(2 * math.pi * k / n)) for k in range(n)),
            hole=spec.housing_bolt_diameter, circle=2.0 * r, square=None,
            screw=size.name if size else "",
            position=_position(spec.housing_bolt_diameter,
                               size.d if size else None, fixed=False),
            fixed=False, datums=("A", "B")))
    if (name == "input_end_plate" and spec.ground_frame_fitted
            and spec.output_bolt_count):
        size = size_for_hole(spec.output_bolt_diameter)
        n = spec.output_bolt_count
        r = spec.output_face_bolt_radius
        phase = spec.output_bolt_phase
        patterns.append(Pattern(
            label="output bolts",
            centres=tuple((r * math.cos(phase + 2 * math.pi * k / n),
                           r * math.sin(phase + 2 * math.pi * k / n))
                          for k in range(n)),
            hole=spec.output_bolt_diameter, circle=2.0 * r, square=None,
            screw=size.name if size else "",
            # Into the driven machine's tapped holes: a fixed fastener.
            position=_position(spec.output_bolt_diameter,
                               size.d if size else None, fixed=True),
            fixed=True, datums=("A", "B"), phase=math.degrees(phase)))
    if direct_motor:
        patterns.append(_motor_pattern(spec, register is not None, recessed=True))
    if spec.plate_dowel_centres:
        patterns.append(Pattern(
            label="plate dowels", centres=tuple(spec.plate_dowel_centres),
            hole=spec.plate_dowel_hole, circle=2.0 * spec.housing_bolt_radius,
            square=None,
            screw=(f"Ø{spec.plate_dowel_diameter:g} m6 x "
                   f"{spec.plate_dowel_length:g} dowel"),
            position=None, fixed=False, datums=(),
            angles=tuple(math.degrees(a) for a in spec.plate_dowel_angles),
            fit="" if printed else "H7"))

    # ---- the motor's mount, through a coupling ------------------------------
    mount = None
    if name == "input_end_plate" and spec.coupled and not spec.ground_frame_fitted:
        frame = spec.motor_face
        tube_bore, tube_outer, height = spec.motor_mount
        hole = spec.motor_mount_hole
        flange_register = None
        if frame.pilot_diameter > hole + 1e-9:
            flange_register = Register(frame.pilot_diameter, frame.pilot_depth,
                                       hole_limits(frame.pilot_diameter, "H7"))
        w0, w1 = (abs(z - spec.plate_face_z) for z in spec.window_span)
        mount = Mount(tube_bore=2.0 * tube_bore, tube_outer=2.0 * tube_outer,
                      height=height, flange=spec.motor_mount_flange,
                      width=frame.body_width, corner=0.11 * frame.body_width,
                      hole=hole, register=flange_register,
                      round=frame.round_body, spigot=frame.spigot,
                      window_width=2.0 * spec.window_half_width,
                      window=tuple(sorted((w0, w1))))
        patterns.append(_motor_pattern(spec, frame.spigot, recessed=False))

    return PlateFeatures(
        name=name, title=_TITLES[name],
        outside_diameter=2.0 * spec.housing_outer_radius, thickness=t,
        seat=seat, through=through, lip=lip_z, register=register,
        patterns=tuple(patterns), mount=mount, edge_break=spec.edge_break,
        printed=printed)


def _motor_pattern(spec: GearSpec, located: bool, recessed: bool) -> Pattern:
    """The motor's screw holes: in the plate, or in the mount's flange."""
    frame = spec.motor_face
    fix = spec.motor_fixing
    # A threaded motor face holds the screw's far end: a fixed fastener.  A
    # flanged motor takes nuts, so the screw floats through both.
    fixed = bool(fix and fix.from_drive)
    recess = None
    if recessed and fix is not None:
        recess = ("counterbore" if fix.from_drive else "hex pocket",
                  fix.recess, fix.recess_depth)
    return Pattern(
        label=f"{frame.name} motor",
        centres=tuple(spec.motor_bolt_centres), hole=frame.bolt_diameter,
        circle=None if frame.square else frame.bolt_span,
        square=frame.bolt_span if frame.square else None,
        screw=fix.size.name if fix else "",
        position=_position(frame.bolt_diameter, fix.size.d if fix else None, fixed),
        fixed=fixed, datums=("A", "C") if located else ("A", "B"), recess=recess)


# ----------------------------------------------------------------- the section


def _material(f: PlateFeatures, z: float, r: float, side: float) -> bool:
    """Whether the cut through the axis along ``x`` has metal at height ``z``
    from the inside face and radius ``r``, on the ``+x`` (side 1) or ``-x``
    (side -1) half."""
    t = f.thickness
    if 0.0 <= z <= t:
        if r > f.outside_diameter / 2.0:
            return False
        inner = f.through / 2.0
        if f.seat is not None and z < f.seat.depth:
            inner = f.seat.diameter / 2.0
        if f.register is not None and z > t - f.register.depth:
            inner = max(inner, f.register.diameter / 2.0)
        if r < inner:
            return False
        for p in f.patterns:
            for cx, cy in p.centres:
                if abs(cy) > 1e-6 or cx * side <= 0.0:
                    continue
                if abs(r - abs(cx)) < p.hole / 2.0:
                    return False
                if (p.recess is not None and z < p.recess[2]
                        and abs(r - abs(cx)) < p.recess[1] / 2.0):
                    return False
        return True
    m = f.mount
    if m is None or z < t:
        return False
    top = t + m.height + m.flange
    if z <= t + m.height:
        return m.tube_bore / 2.0 <= r <= m.tube_outer / 2.0
    if z <= top:
        if r > m.width / 2.0 or r < m.hole / 2.0:
            return False
        if m.register is not None and z > top - m.register.depth:
            return r >= m.register.diameter / 2.0
        return True
    return False


def _breaks(f: PlateFeatures) -> tuple[list[float], list[float]]:
    """Every height and radius at which the section can change."""
    t = f.thickness
    zs = {0.0, t}
    rs = {0.0, f.outside_diameter / 2.0, f.through / 2.0}
    if f.seat is not None:
        zs.add(f.seat.depth)
        rs.add(f.seat.diameter / 2.0)
    if f.register is not None:
        zs.add(t - f.register.depth)
        rs.add(f.register.diameter / 2.0)
    for p in f.patterns:
        for cx, cy in p.centres:
            if abs(cy) < 1e-6:
                rs.update({abs(cx) - p.hole / 2.0, abs(cx) + p.hole / 2.0})
                if p.recess is not None:
                    rs.update({abs(cx) - p.recess[1] / 2.0,
                               abs(cx) + p.recess[1] / 2.0})
                    zs.add(p.recess[2])
    m = f.mount
    if m is not None:
        top = t + m.height + m.flange
        zs.update({t + m.height, top})
        rs.update({m.tube_bore / 2.0, m.tube_outer / 2.0, m.width / 2.0,
                   m.hole / 2.0})
        if m.register is not None:
            zs.add(top - m.register.depth)
            rs.add(m.register.diameter / 2.0)
    return sorted(zs), sorted(r for r in rs if r >= 0.0)


def _section(f: PlateFeatures) -> tuple[Hatch, list[Segment]]:
    """The cut material as cells, and its outline as segments.

    In view coordinates: ``u`` is the height from the inside face, ``v`` the
    radius - up for ``+x``, down for ``-x``.  Cells on a grid of every break,
    so the outline is exact and falls out of which neighbours differ.
    """
    zs, rs = _breaks(f)
    cells: list[tuple[float, float, float, float]] = []
    edges: list[Segment] = []
    for side in (1.0, -1.0):
        grid = [[_material(f, (z0 + z1) / 2.0, (r0 + r1) / 2.0, side)
                 for r0, r1 in pairwise(rs)]
                for z0, z1 in pairwise(zs)]

        def filled(i: int, j: int, grid=grid) -> bool:
            return 0 <= i < len(grid) and 0 <= j < len(grid[0]) and grid[i][j]

        for i, (z0, z1) in enumerate(pairwise(zs)):
            for j, (r0, r1) in enumerate(pairwise(rs)):
                if not grid[i][j]:
                    continue
                cells.append((z0, side * r0, z1, side * r1))
                if not filled(i - 1, j):
                    edges.append(Segment((z0, side * r0), (z0, side * r1)))
                if not filled(i + 1, j):
                    edges.append(Segment((z1, side * r0), (z1, side * r1)))
                if not filled(i, j - 1):
                    edges.append(Segment((z0, side * r0), (z1, side * r0)))
                if not filled(i, j + 1):
                    edges.append(Segment((z0, side * r1), (z1, side * r1)))
    return Hatch(tuple(cells)), edges


# ------------------------------------------------------------------- layout


def _fmt(x: float) -> str:
    return f"{x:.3f}".rstrip("0").rstrip(".")


def _limits(nominal: float, fit: SeatFit) -> str:
    hi, lo = fit.limits_mm(nominal)
    return f"{fit.tolerance_class}, {lo:.3f} to {hi:.3f}"


def _scale(diameter: float) -> float:
    """The largest standard scale the front view fits at, ISO 5455."""
    for s in (5.0, 2.0, 1.0, 0.5, 0.2, 0.1):
        if diameter * s <= 165.0:
            return s
    return 0.1


def _scale_text(s: float) -> str:
    return f"{s:g}:1" if s >= 1.0 else f"1:{1.0 / s:g}"


def _front(f: PlateFeatures, mirror: float) -> list:
    """The view from outside, with the part's ``x`` pointing up the sheet.

    Up rather than across so that the cut along ``x`` is a vertical line and
    its section stands beside the view, at the same heights.  The output plate
    is seen from the other end of the drive, which turns its ``y`` over.
    """
    def uv(p: tuple[float, float]) -> tuple[float, float]:
        return (-mirror * p[1], p[0])

    R = f.outside_diameter / 2.0
    items: list = [Circle((0.0, 0.0), R)]
    items += [Segment((-R - 6.0, 0.0), (R + 6.0, 0.0), CENTRE)]
    m = f.mount
    if m is not None:
        w, c = m.width / 2.0, m.corner
        if m.round:
            items.append(Circle((0.0, 0.0), w))
        else:
            items.append(Polyline(((w - c, w), (-w + c, w), (-w, w - c),
                                   (-w, -w + c), (-w + c, -w), (w - c, -w),
                                   (w, -w + c), (w, w - c))))
        items.append(Circle((0.0, 0.0), m.hole / 2.0))
        items.append(Circle((0.0, 0.0), m.tube_outer / 2.0, HIDDEN))
        if m.register is not None:
            items.append(Circle((0.0, 0.0), m.register.diameter / 2.0))
    else:
        items.append(Circle((0.0, 0.0), f.through / 2.0))
        if f.register is not None:
            items.append(Circle((0.0, 0.0), f.register.diameter / 2.0))
        if f.seat is not None and f.seat.depth < f.thickness - 1e-9:
            items.append(Circle((0.0, 0.0), f.seat.diameter / 2.0, HIDDEN))
    for p in f.patterns:
        if p.circle is not None:
            items.append(Circle((0.0, 0.0), p.circle / 2.0, CENTRE))
        for centre in p.centres:
            c = uv(centre)
            items.append(Circle(c, p.hole / 2.0))
            arm = p.hole / 2.0 + 1.5
            items.append(Segment((c[0] - arm, c[1]), (c[0] + arm, c[1]), THIN))
            items.append(Segment((c[0], c[1] - arm), (c[0], c[1] + arm), THIN))
            if p.recess is not None:
                kind, size, _ = p.recess
                if kind == "counterbore":
                    items.append(Circle(c, size / 2.0, HIDDEN))
                else:
                    # Across corners on the cut axis, as the solid's hexagon.
                    rr = size / math.sqrt(3.0)
                    items.append(Polyline(tuple(
                        (c[0] + rr * math.cos(math.radians(30 + 60 * k)),
                         c[1] + rr * math.sin(math.radians(30 + 60 * k)))
                        for k in range(6)), kind=HIDDEN))
    return items


def _pattern_lines(p: Pattern, f: PlateFeatures) -> tuple[str, ...]:
    n = len(p.centres)
    if p.angles:
        where = (f"ON Ø{_fmt(p.circle)} AT "
                 + " AND ".join(f"{a:g}°" for a in p.angles))
    elif p.circle is not None:
        where = f"ON Ø{_fmt(p.circle)} EQ SP"
    else:
        where = f"ON {_fmt(p.square)} x {_fmt(p.square)} SQUARE"
    size = f"Ø{_fmt(p.hole)} {p.fit}".strip()
    lines = [f"{n}x {size} THRU {where}"]
    if p.phase:
        lines.append(f"{p.phase:.1f}° FROM THE TIE BOLTS")
    if p.recess is not None:
        kind, size, depth = p.recess
        if kind == "counterbore":
            lines.append(f"C'BORE Ø{_fmt(size)} x {_fmt(depth)} DEEP FROM A")
        else:
            lines.append(f"HEX POCKET {_fmt(size)} A/F x {_fmt(depth)} DEEP FROM A")
    if p.fit:
        lines.append("REAM ON ASSEMBLY WITH THE BARREL")
    lines.append(f"for {p.screw}, {p.label}" if p.screw else f"for the {p.label}")
    return tuple(lines)


def _slot(view_scale: float, radius: float, angle: float) -> tuple[float, float]:
    """Where a callout's shoulder starts: diagonally out from the view, clear of
    the part, in paper millimetres from the view's centre."""
    rho = radius * view_scale + 16.0
    return rho * math.cos(math.radians(angle)), rho * math.sin(math.radians(angle))


def _towards(centre: tuple[float, float], radius: float,
             knee_view: tuple[float, float]) -> tuple[float, float]:
    """The point on a circle's edge facing ``knee_view``."""
    dx, dy = knee_view[0] - centre[0], knee_view[1] - centre[1]
    d = math.hypot(dx, dy) or 1.0
    return centre[0] + radius * dx / d, centre[1] + radius * dy / d


#: The front view's centre, and the inside face in the section, paper mm.
_FRONT = (138.0, 163.0)
_SECTION_X = 302.0


def plate_sheet(spec: GearSpec, name: str) -> Sheet:
    """``name``'s sheet, laid out and ready to draw."""
    f = plate_features(spec, name)
    s = _scale(max(f.outside_diameter,
                   f.mount.width * math.sqrt(2.0) if f.mount else 0.0))
    mirror = 1.0 if name == "input_end_plate" else -1.0
    R = f.outside_diameter / 2.0
    title_y = _FRONT[1] + s * R + 22.0

    # ---- the front view -------------------------------------------------------
    front = View("VIEW FROM OUTSIDE", _FRONT, s, _front(f, mirror),
                 title_at=(_FRONT[0], title_y))
    front.items.append(CuttingPlane((0.0, -R - 8.0 / s), (0.0, R + 8.0 / s), "A",
                                    toward=0.0))
    slots = iter((135.0, -135.0, 45.0, -45.0, 160.0, -160.0))
    for p in f.patterns:
        a = next(slots)
        knee = _slot(s, R, a)
        knee_view = (knee[0] / s, knee[1] / s)
        holes = [(-mirror * c[1], c[0]) for c in p.centres]
        hole = max(holes, key=lambda q, a=a: q[0] * math.cos(math.radians(a))
                   + q[1] * math.sin(math.radians(a)))
        frame = (Frame(p.position, mmc=True, datums=p.datums)
                 if p.position else None)
        front.items.append(Callout(_towards(hole, p.hole / 2.0, knee_view),
                                   _pattern_lines(p, f), frame=frame, knee=knee))
    if f.register is not None:
        knee = _slot(s, R, next(slots))
        front.items.append(Callout(
            _towards((0.0, 0.0), f.register.diameter / 2.0, (knee[0] / s, knee[1] / s)),
            (f"Ø{_fmt(f.register.diameter)} H7 x {_fmt(f.register.depth)} DEEP",
             "register: the motor's spigot locates here"), knee=knee))

    # ---- the section ------------------------------------------------------------
    hatch, outline = _section(f)
    height = f.thickness + (f.mount.height + f.mount.flange if f.mount else 0.0)
    section = View("SECTION A-A", (_SECTION_X, _FRONT[1]), s,
                   [hatch, *outline,
                    Segment((-4.0, 0.0), (height + 4.0, 0.0), CENTRE)],
                   title_at=(_SECTION_X + s * height / 2.0, title_y))
    items = section.items
    # Thickness along the top, outside diameter down the right.
    items.append(Linear((0.0, R), (f.thickness, R), "h", 10.0, _fmt(f.thickness)))
    right = s * height + 10.0
    items.append(Linear((f.thickness, -R), (f.thickness, R), "v",
                        right - s * f.thickness, f"Ø{_fmt(f.outside_diameter)}"))
    right += 10.0
    if f.seat is not None:
        d = f.seat.diameter / 2.0
        text = (f"Ø{_fmt(f.seat.diameter)} {f.seat.fit.tolerance_class}"
                if f.seat.fit else f"Ø{_fmt(f.seat.diameter)}")
        left = -12.0
        items.append(Linear((0.0, -d), (0.0, d), "v", left, text))
        items.append(Datum((left / s, -d), -90.0, "B"))
        if f.seat.depth < f.thickness - 1e-9:
            # In the bore, just above the axis: the extension lines run down
            # through the empty seat instead of across the cut metal.
            items.append(Linear((0.0, d), (f.seat.depth, d), "h", 3.0 - s * d,
                                _fmt(f.seat.depth), spill="a"))
    if f.lip is not None or f.seat is None:
        # On the left, from the lip's inside face: out through the seat, which
        # is empty, rather than across whatever stands on the outside face.
        z = f.lip[0] if f.lip is not None else 0.0
        items.append(Linear((z, -f.through / 2.0), (z, f.through / 2.0), "v",
                            -24.0 - s * z, f"Ø{_fmt(f.through)} THRU"))
    if f.register is not None:
        rr = f.register.diameter / 2.0
        items.append(Linear((f.thickness - f.register.depth, rr),
                            (f.thickness, rr), "h", 8.0,
                            _fmt(f.register.depth)))
        items.append(Datum((f.thickness - f.register.depth / 2.0, rr), -90.0, "C"))
    face_v = (R + max(f.through, f.seat.diameter if f.seat else 0.0) / 2.0) / 2.0
    items.append(Datum((0.0, face_v), 180.0, "A"))
    m = f.mount
    if m is not None:
        base = f.thickness
        z_top = base + m.height + m.flange
        items.append(Linear((base, m.width / 2.0), (base + m.height, m.width / 2.0),
                            "h", 10.0, _fmt(m.height)))
        items.append(Linear((base + m.height, m.width / 2.0),
                            (z_top, m.width / 2.0), "h", 10.0, _fmt(m.flange)))
        for dia, text in ((m.width, f"Ø{_fmt(m.width)}" if m.round
                           else f"{_fmt(m.width)} A/F"),
                          (m.tube_outer, f"Ø{_fmt(m.tube_outer)}"),
                          (m.tube_bore, f"Ø{_fmt(m.tube_bore)}")):
            items.append(Linear((z_top, -dia / 2.0), (z_top, dia / 2.0), "v",
                                right - s * z_top, text))
            right += 10.0
        hole_text = (f"Ø{_fmt(m.hole)} THRU" if m.register is not None
                     or not m.spigot
                     else f"Ø{_fmt(m.hole)} H7 THRU - the spigot locates here")
        items.append(Callout((z_top - m.flange / 2.0, -m.hole / 2.0), (hole_text,),
                             knee=(s * z_top + 12.0, -s * R - 4.0)))
        if m.register is not None:
            items.append(Callout(
                (z_top, -m.register.diameter / 2.0),
                (f"Ø{_fmt(m.register.diameter)} H7 x {_fmt(m.register.depth)} DEEP",
                 "the spigot locates here"),
                knee=(s * z_top + 12.0, -s * R - 14.0)))
            items.append(Datum((z_top - m.register.depth / 2.0,
                                m.register.diameter / 2.0), -90.0, "C"))
        elif m.spigot:
            items.append(Datum((z_top - m.flange / 2.0, m.hole / 2.0), -90.0, "C"))

    # ---- notes ------------------------------------------------------------------
    notes = []
    if f.printed:
        notes.append(
            f"Printed ({spec.process.value}) in {spec.housing_material}. No ISO "
            f"general tolerance applies to a printed part: holes are drawn "
            f"{_fmt(spec.hole_clearance)} mm over the size they take, which is the "
            f"clearance this design assumes for the process.")
    else:
        notes.append(f"Dimensions in millimetres. Tolerancing ISO 8015; untoleranced "
                     f"sizes to {_GENERAL[spec.process]}.")
    notes.append("A is the inside face: it bolts to the ring housing and runs "
                 "against the discs. B is the axis of the bearing seat"
                 + (", C the axis of the bore the motor's spigot locates in."
                    if any("C" in p.datums for p in f.patterns) else "."))
    if f.seat is not None:
        if f.seat.fit is not None:
            notes.append(
                f"Bearing seat for a {f.seat.bearing}, {_fmt(f.seat.diameter)} mm "
                f"outside, bored from A to {_limits(f.seat.diameter, f.seat.fit)}: "
                f"{f.seat.fit.why}. Fit per the rolling-bearing makers' housing "
                f"tables, for a steel or cast-iron housing.")
        else:
            notes.append(
                f"Bearing seat printed at the {f.seat.bearing}'s "
                f"{_fmt(f.seat.diameter)} mm outside and the bearing pressed in; "
                f"a printed bore holds no ISO fit, so check the press on a test "
                f"print.")
    for p in f.patterns:
        if p.position is None:
            continue
        how = ("each screw fixed in a thread at its far end, so the plate has "
               "half the clearance" if p.fixed else
               "each screw floats through clearance in every part, so the "
               "whole clearance")
        notes.append(f"{p.label[0].upper()}{p.label[1:]}: position "
                     f"Ø{_fmt(p.position)} at maximum material - an {p.screw} in "
                     f"a Ø{_fmt(p.hole)} hole, {how}.")
    dowels = next((p for p in f.patterns if p.label == "plate dowels"), None)
    if dowels is not None:
        n = len(dowels.centres)
        if dowels.fit:
            notes.append(
                f"Plate dowels: {n}x ISO 8734 {dowels.screw}s, pressed into the "
                f"barrel (N7) and a tight slide here (H7) - they, not the tie "
                f"bolts, put this plate on the ring's axis. Ream these holes on "
                f"assembly with the barrel, its pin circle and B concentric "
                f"within Ø{_fmt(spec.profile_clearance)}: the crank turns on B, "
                f"and an axis off the ring's by more than half the profile "
                f"clearance takes it from one side of the mesh.")
        else:
            notes.append(
                f"Plate dowels: {n}x ISO 8734 {dowels.screw}s, pressed into the "
                f"barrel's printed holes; these are printed "
                f"{_fmt(spec.hole_clearance)} mm over the pin so it slides, "
                f"which is what this design assumes of a printed hole.")
    if f.edge_break > 0.0:
        notes.append(f"Break outside edges and the mouths of the bores "
                     f"{_fmt(f.edge_break)} x 45°; inside corners at the seat's "
                     f"lip R{_fmt(_INSIDE_RADIUS)} max. Not the holes.")
    if f.mount is not None:
        w0, w1 = f.mount.window
        notes.append(f"Two windows through the tube, opposite each other across "
                     f"the y axis: {_fmt(f.mount.window_width)} wide, from "
                     f"{_fmt(w0)} to {_fmt(w1)} above the outside face - for a key "
                     f"to the coupling's screws.")

    general = "as printed" if f.printed else _GENERAL[spec.process]
    member = "carrier" if spec.output_member is OutputMember.CARRIER else "ring"
    return Sheet(
        part=f.title, number=name.upper(), material=spec.housing_material,
        process=spec.process.value, quantity=1, mass_g=_plate_mass(spec, name),
        scale=_scale_text(s), general=general, views=[front, section],
        notes=notes,
        design=(f"{spec.ratio}:1, {f.outside_diameter:.0f} mm OD, "
                f"{member} output"),
        warning=f"{notice.HEADLINE.upper()} - {notice.SHORT}")


def _plate_mass(spec: GearSpec, name: str) -> float:
    """What the bill of materials says this plate weighs."""
    from ..analysis.mass import analyse_mass

    m = analyse_mass(spec)
    if spec.coupled and not spec.ground_frame_fitted:
        plain = (m.plates_mass_g - m.mount_mass_g) / 2.0
        return plain + m.mount_mass_g if name == "input_end_plate" else plain
    return m.plates_mass_g / 2.0


def write_sheets(spec: GearSpec, directory: str | Path,
                 names: Sequence[str] | None = None) -> list[Path]:
    """Write a sheet per part into ``directory``."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    written = []
    for name in names or sheet_names(spec):
        written.append(write_sheet_pdf(plate_sheet(spec, name),
                                       directory / f"{name}.pdf"))
    return written

