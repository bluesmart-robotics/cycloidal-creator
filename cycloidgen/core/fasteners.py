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

__all__ = ["M2", "SCREW_LENGTHS", "SIZES", "THREAD_DEPTH", "MetricSize", "hexagon",
           "nut_volume", "screw_length", "screw_volume", "size_for_hole",
           "size_named", "thread_length", "thread_mean_diameter"]


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


#: Smaller than any hole the drive's own bolts are read off, so not in
#: :data:`SIZES` - a 2.6 mm tie-bolt hole is still drawn by the hole - but a
#: NEMA 8's face is threaded for it, and that screw has to be named.
M2 = MetricSize("M2", 2.0, 0.4, 2.4, 3.8, 2.0, 1.5, 1.0, 4.0, 1.6)


def size_named(name: str) -> MetricSize | None:
    """A size by its metric name, ``M2`` included; ``None`` for one not here."""
    return next((s for s in (M2, *SIZES) if s.name == name), None)


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


#: An external thread's depth, in pitches: crest to root, ISO 68-1's h3.
THREAD_DEPTH = 0.6134


def thread_length(size: MetricSize, length: float) -> float:
    """How much of a screw ``length`` under the head is threaded: ISO 4762's
    reference length ``2d + 12``, or all of it when it is shorter."""
    return min(length, 2.0 * size.d + 12.0)


def thread_mean_diameter(size: MetricSize) -> float:
    """The plain cylinder with a V-thread's volume per length.

    The drawn thread is a groove every pitch, its radius running straight from
    crest to root and back, so the mean of r squared along it is the mean of a
    linear ramp's square - which is what a drawing that cannot afford the
    grooves draws instead, and still encloses the same steel.
    """
    crest = size.d / 2.0
    root = crest - THREAD_DEPTH * size.pitch
    return 2.0 * math.sqrt((crest ** 2 + crest * root + root ** 2) / 3.0)


def screw_length(grip: float, size: MetricSize) -> float | None:
    """The shortest preferred length that clamps ``grip`` with a nut on it.

    The grip, the nut, and two pitches of thread standing past the nut - the
    usual rule for a nut that is fully engaged.  ``None`` past the end of the
    table, where the answer is a stud and not a screw.
    """
    need = grip + size.nut_m + 2.0 * size.pitch
    return next((n for n in SCREW_LENGTHS if n >= need - 1e-9), None)


# ------------------------------------------------------------------- weight --
#
# What a bought fastener weighs, for the assembled mass.  Off the dimensions
# above rather than from a weight table of its own, so the screw that is drawn,
# ordered and weighed is one screw - and measured against the published ones:
# ISO 4762's own mass table (at 7.85 g/cm3) has M3x30 1.86 g, M4x40 4.65 g,
# M5x50 8.63 g, M6x40 11.0 g and M8x50 24.9 g, and these come within 5% of all
# five; steel ISO 4032 nuts are sold at 0.8, 1.2, 2.5 and 5.2 g from M4 to M8,
# and the nut here within 11% of each.  `tests/test_fasteners.py` holds both.


def screw_volume(size: MetricSize, length: float) -> float:
    """The steel in an ISO 4762 screw ``length`` under the head, mm^3.

    The head as a cylinder less its socket, the shank at the nominal diameter,
    and the threaded end - ISO 4762's reference thread length, ``2d + 12``, or
    the whole shank when it is shorter - at the stress-area diameter, since a
    thread is a groove and not a sleeve.
    """
    head = math.pi / 4.0 * size.head_d ** 2 * size.head_h
    socket = math.sqrt(3.0) / 2.0 * size.socket ** 2 * size.socket_depth
    shank = math.pi / 4.0 * size.d ** 2 * length
    threaded = min(length, 2.0 * size.d + 12.0)
    stress_d = size.d - 0.9382 * size.pitch
    groove = threaded * math.pi / 4.0 * (size.d ** 2 - stress_d ** 2)
    return head - socket + shank - groove


def nut_volume(size: MetricSize) -> float:
    """The steel in an ISO 4032 nut, mm^3.

    A hexagonal prism bored to the nominal diameter: the thread's flanks that
    leaves out and the corner chamfers it leaves in come to about the same.
    """
    return ((math.sqrt(3.0) / 2.0 * size.nut_s ** 2 - math.pi / 4.0 * size.d ** 2)
            * size.nut_m)
