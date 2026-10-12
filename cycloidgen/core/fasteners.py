"""Metric fasteners: the ISO dimensions the drive's bolts are drawn and ordered by.

Nominal figures from the standards' own tables - ISO 273 clearance holes in the
medium series, ISO 4762 socket head cap screws, ISO 4032 hexagon nuts and the
coarse thread pitch - for the sizes a drive of this kind uses.  Like the bearing
catalogue they are first-pass values: confirm them against the supplier's sheet
before ordering.

The design states a *hole*, not a bolt: ``housing_bolt_diameter`` is what the
plates are drilled to.  The bolt is read back off it as the largest size that
hole is the medium clearance for, because that is the question a shop asks of a
drawing - "what goes through this" - and a 4.5 mm hole is an M4 whatever it was
meant to be.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["SCREW_LENGTHS", "SIZES", "MetricSize", "hexagon", "screw_length",
           "size_for_hole"]


@dataclass(frozen=True)
class MetricSize:
    """One metric coarse size, millimetres throughout."""

    name: str
    #: Nominal diameter.
    d: float
    #: Coarse thread pitch.
    pitch: float
    #: ISO 273 clearance hole, medium series.
    clearance: float
    #: ISO 4762 head diameter and height, both at their maximum.
    head_d: float
    head_h: float
    #: ISO 4762 hexagon socket: the key size across flats, and its least depth.
    socket: float
    socket_depth: float
    #: ISO 4032 nut: across flats, and height at its maximum.
    nut_s: float
    nut_m: float


SIZES: tuple[MetricSize, ...] = (
    MetricSize("M2.5", 2.5, 0.45, 2.9, 4.5, 2.5, 2.0, 1.1, 5.0, 2.0),
    MetricSize("M3", 3.0, 0.5, 3.4, 5.5, 3.0, 2.5, 1.3, 5.5, 2.4),
    MetricSize("M4", 4.0, 0.7, 4.5, 7.0, 4.0, 3.0, 2.0, 7.0, 3.2),
    MetricSize("M5", 5.0, 0.8, 5.5, 8.5, 5.0, 4.0, 2.5, 8.0, 4.7),
    MetricSize("M6", 6.0, 1.0, 6.6, 10.0, 6.0, 5.0, 3.0, 10.0, 5.2),
    MetricSize("M8", 8.0, 1.25, 9.0, 13.0, 8.0, 6.0, 4.0, 13.0, 6.8),
    MetricSize("M10", 10.0, 1.5, 11.0, 16.0, 10.0, 8.0, 5.0, 16.0, 8.4),
    MetricSize("M12", 12.0, 1.75, 13.5, 18.0, 12.0, 10.0, 6.0, 18.0, 10.8),
    MetricSize("M16", 16.0, 2.0, 17.5, 24.0, 16.0, 14.0, 8.0, 24.0, 14.8),
)

#: ISO 4762's preferred lengths under the head.
SCREW_LENGTHS: tuple[float, ...] = (
    6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80, 90,
    100, 110, 120, 130, 140, 150, 160, 180, 200)


def size_for_hole(hole: float) -> MetricSize | None:
    """The largest size ``hole`` is the medium clearance for, or ``None``.

    ``None`` below an M2.5's 2.9 mm: a hole that small is not for a bolt this
    table knows, and naming one would be ordering a part that does not go in.
    """
    fitting = [s for s in SIZES if s.clearance <= hole + 1e-9]
    return fitting[-1] if fitting else None


def hexagon(cx: float, cy: float, across_flats: float,
            flat_towards: float) -> list[tuple[float, float]]:
    """A hexagon's corners, one flat facing the angle ``flat_towards``.

    For a nut or a socket.  Here rather than in either renderer because both
    draw the same nut, and a nut turned thirty degrees between the STEP file
    and the 3D view is a nut that overhangs the rim in one and not the other.
    """
    r = across_flats / math.sqrt(3.0)
    return [(cx + r * math.cos(flat_towards + math.pi / 6.0 + k * math.pi / 3.0),
             cy + r * math.sin(flat_towards + math.pi / 6.0 + k * math.pi / 3.0))
            for k in range(6)]


def screw_length(grip: float, size: MetricSize) -> float | None:
    """The shortest preferred length that clamps ``grip`` with a nut on it.

    The grip, the nut, and two pitches of thread standing past the nut - the
    usual rule for a nut that is fully engaged.  ``None`` past the end of the
    table, where the answer is a stud and not a screw.
    """
    need = grip + size.nut_m + 2.0 * size.pitch
    return next((n for n in SCREW_LENGTHS if n >= need - 1e-9), None)
