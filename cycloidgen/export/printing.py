"""Parts made ready to print: the right face down, and its bed edge broken back.

Every printed part here is a prism along the drive's axis, so it is printed
flat, one face or the other on the bed, and which one has a measurable answer:
the one that leaves the least to hold up.  For each face down, the area facing
downward at more than 45 degrees from vertical and not on the bed is what the
printer would have to support; less of it wins, and with nothing either way,
more on the bed does.  On a carrier with its pins up one side and its boss down
the other neither is clean, and the better one is taken.

Then the edges on that face are broken back by the process's first-layer
chamfer, so the squashed first layer has somewhere to spread that is not past
the part's outline - see :data:`~cycloidgen.core.spec.FIRST_LAYER_CHAMFER`.
Only edges where a wall meets the bed square: a corner already chamfered, or a
hole's lead-in, is not broken twice.

A part made of something a printer does not lay down is left as it is.
"""
from __future__ import annotations

import math

import cadquery as cq
import numpy as np

from ..core.spec import PRINT_MATERIALS, GearSpec

__all__ = ["OVERHANG_DEG", "face_down", "is_printed", "print_ready", "with_first_layer"]

#: Steeper than this from vertical, a downward face needs holding up - the usual
#: figure for an unsupported overhang.
OVERHANG_DEG = 45.0

#: Between two copies of a part laid out side by side on the bed.
_GAP_MM = 4.0


def is_printed(spec: GearSpec, part: str) -> bool:
    """Whether the part of this name is made of something a printer lays down."""
    from .threemf import _material_of

    return _material_of(spec, part) in PRINT_MATERIALS


def _down_areas(v: np.ndarray, t: np.ndarray, flipped: bool) -> tuple[float, float]:
    """``(on the bed, needing support)`` in mm^2 with the given face down."""
    if flipped:
        v = v * np.array([1.0, -1.0, -1.0])
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    cross = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cross, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        nz = np.where(area > 0.0, cross[:, 2] / (2.0 * area), 0.0)
    floor = v[:, 2].min()
    on_bed = (np.abs(np.stack([a[:, 2], b[:, 2], c[:, 2]]) - floor) < 1e-3).all(0)
    # A hair past 45: a 45 degree chamfer holds itself up, and in floating
    # point its normal lands either side of the line.
    down = nz < -math.cos(math.radians(OVERHANG_DEG)) - 1e-6
    return float(area[on_bed & down].sum()), float(area[down & ~on_bed].sum())


def face_down(shape: cq.Shape, tolerance: float = 0.1) -> bool:
    """True when the part prints best turned over - its +Z face on the bed.

    Less to hold up wins; within a square millimetre of each other, more on the
    bed; and as modelled before turned over.
    """
    from .threemf import _weld

    vertices, triangles = _weld(shape, tolerance)
    v = np.asarray(vertices, dtype=float)
    t = np.asarray(triangles, dtype=int)
    ranked = []
    for flipped in (False, True):
        contact, support = _down_areas(v, t, flipped)
        ranked.append((round(support), -round(contact), flipped))
    return min(ranked)[2]


def _bed_edges(shape: cq.Shape, flipped: bool) -> list[cq.Edge]:
    """The edges on the face that goes down where a wall meets it square.

    Every one of them is an outside corner, without asking: the bed face is the
    lowest the part goes, so there is no metal below it, and a wall meeting it
    square has the part on one side and air on the other.  Which is as well,
    because asking - a point-in-solid test per edge - is most of a second per
    edge on a spline disc.
    """
    plane = -_floor(shape.mirror("XY")) if flipped else _floor(shape)
    faces_of: dict = {}
    for face in shape.Faces():
        for edge in face.Edges():
            faces_of.setdefault(edge, []).append(face)
    found = []
    for edge, faces in faces_of.items():
        if len(faces) != 2:
            continue
        if max(abs(edge.positionAt(u).z - plane) for u in (0.0, 0.5, 1.0)) > 1e-3:
            continue
        p = edge.positionAt(0.5)
        if abs(faces[0].normalAt(p).dot(faces[1].normalAt(p))) > 0.1:
            continue                      # already broken, or a lead-in
        found.append(edge)
    return found


def with_first_layer(spec: GearSpec, part: str, shape: cq.Shape,
                     flipped: bool | None = None) -> cq.Shape:
    """``shape`` with its bed face's edges broken back, where it is printed.

    In the part's own frame: the 3MF keeps it where it assembles and the STL
    then lays it down, and both carry the same break.
    """
    size = spec.first_layer_chamfer
    if size <= 0.0 or not is_printed(spec, part):
        return shape
    if flipped is None:
        flipped = face_down(shape)
    out = [_break(solid, size, _bed_edges(solid, flipped))
           for solid in shape.Solids()]
    return out[0] if len(out) == 1 else cq.Compound.makeCompound(out)


def _break(solid: cq.Shape, size: float, edges: list[cq.Edge]) -> cq.Shape:
    """Chamfer ``edges``, or as many of them as the kernel will take.

    All of them first.  Where that fails it is at a cusp: ring pins formed with
    the housing meet its bore in sharp points, and OCCT cannot run a chamfer
    into one.  So the second try leaves out every edge that meets another bed
    edge at a corner rather than smoothly - the cusps go and the rim, the holes
    and a disc's flank, whose lobes run into each other tangent, keep their
    break.  If even that fails the part is printed unbroken rather than not
    written at all.
    """
    if not edges:
        return solid
    try:
        return solid.chamfer(size, None, edges)
    except Exception:                       # OCCT says "command not done"
        pass
    smooth = [e for e in edges if not _meets_at_a_corner(e, edges)]
    if smooth:
        try:
            return solid.chamfer(size, None, smooth)
        except Exception:
            pass
    return solid


def _meets_at_a_corner(edge: cq.Edge, edges: list[cq.Edge]) -> bool:
    """Whether ``edge`` meets another of ``edges`` end to end, not tangent."""
    for end, at in ((edge.startPoint(), 0.0), (edge.endPoint(), 1.0)):
        mine = edge.tangentAt(at)
        for other in edges:
            if other.isSame(edge):
                continue
            for theirs_end, theirs_at in ((other.startPoint(), 0.0),
                                          (other.endPoint(), 1.0)):
                if (theirs_end - end).Length > 1e-6:
                    continue
                theirs = other.tangentAt(theirs_at)
                if abs(abs(mine.dot(theirs)) - 1.0) > 1e-3:
                    return True
    return False


def print_ready(spec: GearSpec, part: str, shape: cq.Shape) -> cq.Shape:
    """The part as it goes on the bed: face down, broken back, standing at z=0.

    A part that is several solids at different heights - the washers, one per
    gap in the stack - has each one put down on the bed, side by side, rather
    than printed floating where it assembles.  Its axis is at the origin.
    """
    if not is_printed(spec, part):
        return shape
    flipped = face_down(shape)
    shape = with_first_layer(spec, part, shape, flipped)
    if flipped:
        shape = shape.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    solids = shape.Solids()
    floors = {round(s.BoundingBox().zmin, 6) for s in solids}
    if len(solids) > 1 and len(floors) > 1:
        placed, x = [], 0.0
        for s in solids:
            bb = s.BoundingBox()
            placed.append(s.translate(cq.Vector(x - bb.center.x, -bb.center.y,
                                                -_floor(s))))
            x += bb.xlen + _GAP_MM
        shape = cq.Compound.makeCompound(placed)
    bb = shape.BoundingBox()
    return shape.translate(cq.Vector(-bb.center.x, -bb.center.y, -_floor(shape)))


def _floor(shape: cq.Shape) -> float:
    """The lowest point of the part's surface, exactly.

    Off its tessellation rather than its bounding box, which the kernel inflates
    by its own tolerance - by most of a tenth of a millimetre on a spline disc,
    which would print the part floating above the bed.
    """
    vertices, _ = shape.tessellate(0.5, 0.5)
    return min(v.z for v in vertices)
