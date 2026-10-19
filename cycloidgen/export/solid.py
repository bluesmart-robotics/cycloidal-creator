"""3D solids and the assembled gearbox, via CadQuery/OCCT.

Parts are modelled in their own local frames and placed by the assembly, so each
one can also be exported on its own for printing.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.Geom import Geom_BSplineCurve
from OCP.GeomConvert import GeomConvert
from OCP.gp import gp_Pnt
from OCP.TColgp import TColgp_Array1OfPnt
from OCP.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal

from ..analysis.bearings import (
    BY_NAME,
    BearingPlacement,
    SeatBores,
    Shoulder,
    Spacer,
    bearing_placements,
    bearing_schedule,
    pin_shank_diameter,
    placements_for_spec,
    plate_edge_break,
    seat_bores,
)
from ..core import profile as prof
from ..core.fasteners import THREAD_DEPTH, MetricSize, hexagon, thread_length
from ..core.spec import CARRIER_DROP, GearSpec
from ..viz.mesh import PART_COLOURS, TIE_BOLT_NOMINAL
from .manifest import disc_names

__all__ = [
    "bearing_solids",
    "build_assembly",
    "disc_solid",
    "eccentric_shaft",
    "end_cap",
    "output_flange",
    "parts",
    "ring_housing",
    "spacer_parts",
    "tie_bolts",
    "tie_nuts",
    "write_part_steps",
    "write_step",
    "write_stls",
]

def _colour(group: str) -> cq.Color:
    """Part colour, from the same table the 3D viewer paints with.

    The viewer and the STEP file are meant to be recognisably the same gearbox.
    Two hand-written colour lists would agree on the day they were written and
    not for very long after that.
    """
    r, g, b = PART_COLOURS[group]
    return cq.Color(r / 255.0, g / 255.0, b / 255.0)


#: Which colour group each exported part belongs to.  One table rather than a
#: list of groups written out at the assembly and a second one at the per-part
#: export: they are the same gearbox, and somebody who opens both files should
#: not have to wonder whether they are.  Discs and bearings are matched by
#: prefix because there are as many of them as the design says.
_PART_GROUP = {
    "housing": "housing",
    "ring_pins": "ring_pins",
    "eccentric_shaft": "shaft",
    "output_flange": "carrier",
    "end_cap": "carrier",
    "input_end_plate": "end_plates",
    "output_end_plate": "end_plates",
    "tie_bolts": "fasteners",
    "tie_nuts": "fasteners",
    "plate_dowels": "fasteners",
    "rotor_hub": "shaft",
    "shaft_spacer_input": "shaft",
    "shaft_spacer_output": "shaft",
    "washers": "shaft",
}


def _part_colour(name: str) -> cq.Color:
    """The colour of one part, by the name it is exported under."""
    if name.startswith("disc_"):
        return _colour("discs")
    if name.startswith("bearing_"):
        return _colour("bearings")
    return _colour(_PART_GROUP[name])


def _convex(body: cq.Shape, edge: cq.Edge, step: float = 1e-3) -> bool:
    """Whether ``edge`` is an outside corner of ``body`` rather than an inside one.

    Step off the edge along one face's outward normal and against the other's.
    At an outside corner - a rim, or the mouth of a bore - both points land in
    air; at an inside one, where a boss stands on a plate or the shaft runs into
    a cam, both land in metal.  A chamfer only belongs on the first kind: on the
    second OCCT *adds* a wedge, which is a fillet nobody drew, and it fills the
    corner a bearing's inner ring is meant to sit against.
    """
    faces = [f for f in body.Faces() if any(edge.isSame(e) for e in f.Edges())]
    if len(faces) != 2:
        return False
    p = edge.positionAt(0.5)
    a, b = faces[0].normalAt(p), faces[1].normalAt(p)
    return not (body.isInside(p + a * step - b * step)
                or body.isInside(p + b * step - a * step))


#: The radius an inside corner is turned or milled to, millimetres - what a
#: cutter's own nose leaves rather than a sharp corner nobody can cut.  Under the
#: 0.3 mm that the smallest bearing ring here is chamfered to at least, so a ring
#: still sits flat on the face beside it: the rule a bearing catalogue gives for
#: the corner a ring is fitted against.
INSIDE_RADIUS = 0.2


def _round_inside(wp: cq.Workplane, size: float = INSIDE_RADIUS) -> cq.Workplane:
    """Radius the inside corners of ``wp`` that run round the drive's axis.

    The seat-to-lip corner a bearing's outer ring sits in, and the root of the
    carrier's boss its inner ring stands against.  Only round the axis: a
    corner round anything else here is where a bought dowel goes into a made
    part, and that has no radius.  Not the shaft, either: every face of every
    cam has a sleeve or a disc washer standing on it, a millimetre-thick washer
    can spare nothing to a chamfer that would clear one, and a sharp corner is
    what the shaft had.  A kernel that will not fillet a corner leaves the part
    as it was rather than failing the export.
    """
    body = wp.val()

    def on_axis(edge: cq.Edge) -> bool:
        centre = edge._geomAdaptor().Circle().Location()
        return abs(centre.X()) < 1e-6 and abs(centre.Y()) < 1e-6

    edges = [e for e in body.Edges()
             if e.geomType() == "CIRCLE" and on_axis(e) and _concave(body, e)]
    if not edges:
        return wp
    try:
        rounded = body.fillet(size, edges)
    except Exception:
        return wp
    return cq.Workplane("XY").add(rounded) if rounded.isValid() else wp


def _concave(body: cq.Shape, edge: cq.Edge, step: float = 1e-3) -> bool:
    """The inside corner :func:`_convex` refuses: both probes land in metal."""
    faces = [f for f in body.Faces() if any(edge.isSame(e) for e in f.Edges())]
    if len(faces) != 2:
        return False
    p = edge.positionAt(0.5)
    a, b = faces[0].normalAt(p), faces[1].normalAt(p)
    return (body.isInside(p + a * step - b * step)
            and body.isInside(p + b * step - a * step))


def _outline_prism(spec: GearSpec, z0: float, height: float) -> cq.Workplane:
    """The housing's outline from ``z0``, ``height`` tall: a cylinder, or a
    square prism with its upright edges rounded to the corner radius - true
    arcs, so the solid encloses what the mass model's area says."""
    r = spec.housing_outer_radius
    wp = cq.Workplane("XY").workplane(offset=z0)
    if not spec.square_housing:
        return wp.circle(r).extrude(height)
    return (wp.rect(2.0 * r, 2.0 * r).extrude(height)
            .edges("|Z").fillet(spec.housing_corner_radius))


def _on_outline(spec: GearSpec, p: cq.Vector) -> bool:
    """Whether a point lies on a square housing's outline, seen along the axis."""
    r, rc = spec.housing_outer_radius, spec.housing_corner_radius
    c = r - rc
    x, y = abs(p.x), abs(p.y)
    if x > c + 1e-6 and y > c + 1e-6:
        return abs((x - c) ** 2 + (y - c) ** 2 - rc * rc) < 1e-4
    return abs(x - r) < 1e-6 or abs(y - r) < 1e-6


def _break_edges(wp: cq.Workplane, radii: Sequence[float],
                 size: float, outline: GearSpec | None = None) -> cq.Workplane:
    """Chamfer the outside circular edges of ``wp`` that lie on ``radii``.

    Chosen by radius rather than by face, because a radius is what names a
    feature here - the rim, a bearing seat, a shaft end - whichever face of the
    part it turns up on, and the inside corners at the same radius are dropped
    by :func:`_convex` rather than listed.  A square housing's rim is not a
    radius: ``outline`` breaks every edge that lies on it instead.
    """
    if size <= 0.0:
        return wp
    body = wp.val()

    def on_rim(e: cq.Edge) -> bool:
        if outline is None or not outline.square_housing:
            return False
        # In a face's plane: the seams down the corners, where a flat side runs
        # into its arc, are on the outline too but are no edge at all.
        ends = (e.positionAt(0.0), e.positionAt(1.0))
        return (abs(ends[0].z - ends[1].z) < 1e-6
                and all(_on_outline(outline, e.positionAt(t)) for t in (0.0, 0.5, 1.0)))

    edges = [e for e in body.Edges()
             if ((e.geomType() == "CIRCLE"
                  and any(abs(e.radius() - r) < 1e-6 for r in radii))
                 or on_rim(e))
             and _convex(body, e)]
    if not edges:
        return wp
    return cq.Workplane("XY").add(body.chamfer(size, None, edges))


def _host_offset(spec: GearSpec, host: str) -> float:
    """Where a part's own frame sits along the axis once it is assembled."""
    return {"input_end_plate": spec.barrel_top,
            "output_end_plate": spec.barrel_bottom - spec.plate_thickness,
            "output_flange": -CARRIER_DROP,
            "end_cap": 0.0}[host]


def _add_shoulders(wp: cq.Workplane, spec: GearSpec,
                   shoulders: Sequence[Shoulder], host: str) -> cq.Workplane:
    """Leave a lip at the outside end of each seat ``host`` carries.

    The part is built with its seat bored right through, and the lip is put
    back as a ring rather than the seat being counterbored from inside: the
    same three lines whichever part it is and whichever end it faces, and the
    ring's outside is the seat's own surface, which the union merges.
    """
    offset = _host_offset(spec, host)
    for s in shoulders:
        if s.host != host:
            continue
        lip = (cq.Workplane("XY").workplane(offset=s.z0 - offset)
               .circle(s.seat / 2.0).circle(s.bore / 2.0)
               .extrude(s.thickness))
        wp = wp.union(lip)
    return wp


def _lips(shoulders: Sequence[Shoulder], host: str) -> list[float]:
    """The radii of ``host``'s lips, for the break on their mouths."""
    return [s.bore / 2.0 for s in shoulders if s.host == host]


def _rim_land(spec: GearSpec) -> float:
    """Metal between the tie-bolt holes and the outside of the housing."""
    return spec.rim_land


def _profile_edges(spec: GearSpec) -> list[cq.Edge]:
    """The disc outline as the curve itself: one cubic B-spline edge per flank.

    The flank of the solid used to be a polygon, sampled to the *mesh*
    tolerance and capped at 1,400 vertices, so a STEP disc was a prism of up to
    1,400 flat faces that left the true profile by 41 um on the 21:1 and 55 um on
    the 59:1 - ten times what the cutting files are held to, governed by a
    setting documented as being for STL.  A CAM package machining from the STEP
    file was cutting the facets.  It is the sketch's spline now, the same
    control points the DXF in ``sketch/`` carries, so the disc you build on and
    the disc you cut from are one curve.

    Split into one edge per flank - root to crest, crest to root - for two
    reasons, and the first would be enough on its own.  It puts a vertex on
    every lobe root and every crest, which are the two places anybody
    dimensioning a cycloidal disc reaches for, and a single closed curve gives
    them nothing to click.  And OCCT integrates a face's mass properties with a
    fixed number of Gauss points across the whole face rather than per knot
    span: one face carrying every span of the profile reports a prism up to
    2.6% off across the presets, high on some and low on others.  One face per
    lobe still leaves 5e-4 at the tightest tolerance; one per flank stays under
    5e-5, which is inside the fit's own deviation.  Every check here that
    compares a mesh against its solid reads that volume.

    The splits are exact rather than approximate.  The fit samples whole lobes
    starting from the root at ``t = 0``, so the root of lobe ``j`` is the knot
    at ``j*n/lobes`` and its crest, by the profile's own mirror symmetry, is
    halfway between two roots.  Splitting a B-spline only inserts knots, which
    changes how the curve is written and not where it goes.
    """
    fit = prof.profile_spline_from_spec(spec)
    n = len(fit.control_points)
    # OCCT starts a periodic spline one interpolated point later than
    # profile_spline's own evaluator does.  Rolling the poles by one puts
    # parameter zero on the first point the fit passed through - the root.
    poles = TColgp_Array1OfPnt(1, n)
    for i, (x, y) in enumerate(np.roll(fit.control_points, 1, axis=0)):
        poles.SetValue(i + 1, gp_Pnt(float(x), float(y), 0.0))
    knots = TColStd_Array1OfReal(1, n + 1)
    mults = TColStd_Array1OfInteger(1, n + 1)
    for i in range(n + 1):
        knots.SetValue(i + 1, float(i))
        mults.SetValue(i + 1, 1)
    curve = Geom_BSplineCurve(poles, knots, mults, 3, True)
    curve.SetNotPeriodic()

    flanks = 2 * spec.lobes
    return [cq.Edge(BRepBuilderAPI_MakeEdge(GeomConvert.SplitBSplineCurve_s(
                curve, n * j / flanks, n * (j + 1) / flanks, 1e-9)).Edge())
            for j in range(flanks)]


def disc_solid(spec: GearSpec, hole_phase: float = 0.0,
               bore: float | None = None) -> cq.Workplane:
    """One cycloidal disc: profile, central bore, output pin holes.

    ``hole_phase`` (radians) rotates the output-hole pattern against the lobes.
    Every disc past the first needs its own value - see
    :attr:`~cycloidgen.core.spec.GearSpec.disc_hole_phases` - because the discs
    are meshed at different angles but driven by one shared carrier.

    ``bore`` is the central hole as bored - the cam bearing's outside wherever
    there is one, see :class:`~cycloidgen.analysis.bearings.SeatBores` - and is
    worked out from the spec when the caller does not already have it.
    """
    outline = cq.Face.makeFromWires(cq.Wire.assembleEdges(_profile_edges(spec)))
    disc = cq.Workplane("XY").add(cq.Solid.extrudeLinear(
        outline, cq.Vector(0, 0, spec.disc_thickness)))
    if bore is None:
        bore = seat_bores(spec).disc
    disc = disc.faces(">Z").workplane().hole(bore)
    disc = (disc.faces(">Z").workplane()
            .polarArray(spec.output_bolt_circle_radius, math.degrees(hole_phase),
                        360, spec.output_pin_count)
            .hole(spec.output_hole_diameter))
    # The bore is a seat - a cam bearing presses into it, or it runs on the cam
    # - so it gets a lead-in.  The flank and the output holes get nothing: they
    # are where the pins roll, and a chamfer there is contact length the
    # analysis is still counting.
    return _break_edges(disc, [bore / 2.0],
                        min(spec.edge_chamfer, spec.disc_thickness / 4.0))


def ring_housing(spec: GearSpec) -> cq.Workplane:
    """Fixed ring: pins sit half-embedded in pockets on the bore.

    It runs from the output end plate's face up to the input one's, which is
    lower than the disc stack at the output end - the carrier hangs under the
    discs and the barrel has to cover it.  Sized to the stack alone it left a
    slot round the gearbox where the carrier was.

    With ``ring_pins_integral`` the same circles are added instead of removed.
    A pocket and the pin that fills it are one shape read from either side, so
    the whole difference between a housing that takes dowels and one that is
    printed with its pins on is ``cut`` against ``union``.
    """
    z0, h = spec.barrel_bottom, spec.barrel_height
    body = _outline_prism(spec, z0, h).cut(
        cq.Workplane("XY").workplane(offset=z0 - 1.0)
        .circle(spec.pin_circle_radius).extrude(h + 2.0))
    pins = (cq.Workplane("XY").workplane(offset=z0)
            .polarArray(spec.pin_circle_radius, 0, 360, spec.pin_count)
            .circle(spec.pin_radius)
            .extrude(h))
    body = body.union(pins) if spec.ring_pins_integral else body.cut(pins)

    # The bolts that hold the plates on have to pass through the thing they are
    # clamping.  Both plates were drilled for them and the bill of materials
    # orders them; the barrel was not, so the exported assembly was a gearbox
    # whose six tie bolts land on solid wall.
    if spec.tie_bolt_centres:
        bolts = (cq.Workplane("XY").workplane(offset=z0 - 1.0)
                 .pushPoints(spec.tie_bolt_centres)
                 .circle(spec.housing_bolt_diameter / 2.0)
                 .extrude(h + 2.0))
        body = body.cut(bolts)
    # The dowels that locate the plates, pressed in from each end: one hole the
    # whole length, so a dowel can be driven out from the other end.
    body = _cut_dowel_holes(body, spec, z0, h, spec.plate_dowel_diameter)
    # The rim only.  The bore and its pockets are where the pins sit and the
    # disc runs; and the rim's break stops at half the metal between it and the
    # tie-bolt holes, which on the default wall is under a millimetre.
    return _break_edges(body, [spec.housing_outer_radius],
                        min(spec.edge_chamfer, _rim_land(spec) / 2.0), outline=spec)


def ring_pins(spec: GearSpec, placements: Sequence[BearingPlacement] = ()) -> cq.Workplane:
    """The pins themselves, as a single multi-solid part.

    A pin carrying a roller shrinks to that roller's bore: the sleeve's OD is the
    surface the profile was cut against, so the pin cannot also have it.

    Barrel length, from the output plate's face to the input plate's, because
    that is what holds them in - see :attr:`GearSpec.ring_pin_length`.
    """
    shank = pin_shank_diameter(placements, "bearing_ring_pins", 2.0 * spec.pin_radius)
    return (cq.Workplane("XY")
            .workplane(offset=spec.barrel_bottom)
            .polarArray(spec.pin_circle_radius, 0, 360, spec.pin_count)
            .circle(shank / 2.0)
            .extrude(spec.ring_pin_length))


def eccentric_shaft(spec: GearSpec) -> cq.Workplane:
    """Input shaft with one eccentric cam per disc, phased around the axis.

    Driven directly, it stops short of the motor and is bored from that end
    for the motor's own shaft - a D where the motor's is a D.
    """
    lo, hi = spec.shaft_span
    shaft = (cq.Workplane("XY")
             .workplane(offset=lo)
             .circle(spec.input_shaft_diameter / 2.0)
             .extrude(hi - lo))
    z = 0.0
    for phase in spec.disc_phases:
        cx = spec.eccentricity * math.cos(phase)
        cy = -spec.eccentricity * math.sin(phase)
        cam = (cq.Workplane("XY").workplane(offset=z)
               .center(cx, cy)
               .circle(spec.cam_diameter / 2.0)
               .extrude(spec.disc_thickness))
        shaft = shaft.union(cam)
        z += spec.disc_thickness + spec.disc_gap
    bore = spec.motor_bore
    if bore is not None:
        z0, z1, d, flat = bore
        hole = cq.Workplane("XY").workplane(offset=z0).circle(d / 2.0).extrude(z1 - z0)
        if flat > 0.0:
            # The motor shaft's flat faces +X at crank zero, as the drawn one
            # does; the bore keeps the metal there, so it drives through it.
            hole = hole.cut(cq.Workplane("XY").workplane(offset=z0 - 1.0)
                            .center(d / 2.0 - flat + d / 2.0, 0.0)
                            .rect(d, 2.0 * d).extrude(z1 - z0 + 2.0))
        shaft = shaft.cut(hole)
    # Both ends, and both ends of every cam - which is a bearing seat or, with
    # no cam bearing, the journal the disc runs on, and either way wants a
    # lead-in.  A cam is thinnest on the side it is offset towards the axis,
    # so that is what its break is held to; where the shaft runs into a cam is
    # an inside corner and is left alone.  A bored end is a ring, and its
    # break is held to half its wall - a thin one is a warning, not a shaft
    # the modeller cannot chamfer.
    c = spec.edge_chamfer
    radii = [spec.input_shaft_diameter / 2.0]
    end = min(c, spec.input_shaft_diameter / 8.0)
    if bore is not None:
        end = min(end, spec.motor_hub_wall / 2.0)
    shaft = _break_edges(shaft, radii, end)
    land = (spec.cam_diameter - spec.input_shaft_diameter) / 2.0 - spec.eccentricity
    shaft = _break_edges(shaft, [spec.cam_diameter / 2.0], min(c, land / 2.0))
    # The flat a coupling's set screw bears on, cut after the breaks so the end
    # chamfer is a circle when it is made and the flat simply runs through it.
    flat = spec.input_flat
    if flat is not None:
        z0, z1, depth = flat
        r = spec.input_shaft_diameter / 2.0
        # Past whichever end is the shaft's own, so the flat runs out of it.
        z0 -= 1.0 if z0 <= lo + 1e-6 else 0.0
        z1 += 1.0 if z1 >= hi - 1e-6 else 0.0
        shaft = shaft.cut(cq.Workplane("XY").box(
            depth + 1.0, 2.0 * r + 2.0, z1 - z0, centered=False)
            .translate((r - depth, -r - 1.0, z0)))
    return shaft


def _motor_mount(spec: GearSpec, host: str, side: float) -> cq.Workplane:
    """The mount a motor stands on through a coupling, in ``host``'s own frame.

    A tube round the coupling with a window either side for its screws, and on
    the end of it a flange the motor's size with the motor's pattern through
    it and its spigot's register in it.  Part of the plate it stands on rather
    than bolted to it: one fewer joint, and nowhere on that face for screws.
    """
    from ..viz.mesh import octagon

    frame = spec.motor_face
    thick = spec.motor_mount_flange
    shift = _host_offset(spec, host)
    face = spec.plate_face_z - shift
    bore, outer, height = spec.motor_mount
    near = face + side * height
    tube = (cq.Workplane("XY").workplane(offset=min(face, near))
            .circle(outer).circle(bore).extrude(height))
    far = near + side * thick
    width = frame.body_width
    base = cq.Workplane("XY").workplane(offset=min(near, far))
    if frame.round_body:
        flange = base.circle(width / 2.0).extrude(thick)
    else:
        flange = (base.polyline([tuple(p) for p in octagon(width, 0.11 * width)])
                  .close().extrude(thick))
    flange = flange.cut(cq.Workplane("XY").workplane(offset=min(near, far) - 1.0)
                        .circle(spec.motor_mount_hole / 2.0)
                        .extrude(thick + 2.0))
    for cx, cy in spec.motor_bolt_centres:
        flange = flange.cut(cq.Workplane("XY").workplane(offset=min(near, far) - 1.0)
                            .center(cx, cy).circle(frame.bolt_diameter / 2.0)
                            .extrude(thick + 2.0))
    if frame.pilot_diameter > spec.motor_mount_hole + 1e-9:
        # The spigot is wider than the tube: a recess for it in the face.
        z = far - frame.pilot_depth if side > 0 else far
        flange = flange.cut(cq.Workplane("XY").workplane(offset=z)
                            .circle(frame.pilot_diameter / 2.0)
                            .extrude(frame.pilot_depth))
    body = tube.union(flange)
    w0, w1 = (z - shift for z in spec.window_span)
    half = spec.window_half_width
    for sy in (1.0, -1.0):
        body = body.cut(cq.Workplane("XY")
                        .box(2.0 * half, outer + 1.0, w1 - w0, centered=False)
                        .translate((-half, 0.0 if sy > 0 else -outer - 1.0, w0)))
    return body


def rotor_hub(spec: GearSpec) -> cq.Workplane | None:
    """The hub an outrunner's can drives the input shaft through, or None.

    A disc on the shaft's end, bored for it and held by a set screw on its
    flat, with the can's screws through it.  Built at its assembled height:
    it turns with the shaft and needs no placement of its own.
    """
    hub = spec.rotor_hub
    if hub is None:
        return None
    z0, z1, outer, bore = hub
    screw = spec.rotor_hub_screw
    hole = screw[0].clearance if screw else 0.0
    body = (cq.Workplane("XY").workplane(offset=z0)
            .circle(outer / 2.0).circle(bore / 2.0).extrude(z1 - z0))
    if hole and spec.rotor_hub_bolt_centres:
        body = body.cut(cq.Workplane("XY").workplane(offset=z0 - 1.0)
                        .pushPoints(spec.rotor_hub_bolt_centres)
                        .circle(hole / 2.0).extrude(z1 - z0 + 2.0))
    return body


def coupling_solid(spec: GearSpec) -> cq.Workplane | None:
    """The coupling, where it goes: bored for the input shaft on the drive's
    half and the motor's on the other, a set screw into each half facing its
    shaft's flat, and the beam's slots as rings round its middle."""
    c = spec.coupling
    if c is None:
        return None
    lo, hi = spec.coupling_span
    mid = (lo + hi) / 2.0
    drive, motor = ((lo, mid), (mid, hi)) if spec.motor_side > 0 else ((mid, hi), (lo, mid))
    body = (cq.Workplane("XY").workplane(offset=lo).circle(c.outer / 2.0)
            .extrude(hi - lo))
    for (a, b), d in ((drive, spec.input_shaft_diameter),
                      (motor, spec.motor_face.shaft_diameter)):
        body = body.cut(cq.Workplane("XY").workplane(offset=a).circle(d / 2.0)
                        .extrude(b - a))
        body = body.cut(cq.Workplane("YZ").center(0.0, (a + b) / 2.0)
                        .circle(min(1.5, (b - a) / 6.0)).extrude(c.outer / 2.0 + 1.0))
    if c.catalogue:
        from ..core.motor import COUPLING_SLOT_DEPTH, COUPLING_SLOT_WIDTH, COUPLING_SLOTS

        for k in range(COUPLING_SLOTS):
            z = mid + (k - (COUPLING_SLOTS - 1) / 2.0) * c.length / 8.0
            body = body.cut(cq.Workplane("XY")
                            .workplane(offset=z - COUPLING_SLOT_WIDTH / 2.0)
                            .circle(c.outer / 2.0 + 1.0)
                            .circle(c.outer / 2.0 - COUPLING_SLOT_DEPTH)
                            .extrude(COUPLING_SLOT_WIDTH))
    return body


def _motor_recesses(spec: GearSpec, inside: float, side: float) -> cq.Workplane | None:
    """What the motor's screws need cut into the face the discs run on: a
    counterbore for a head, or a hexagonal pocket for a nut, from ``inside``
    towards the motor.  None with no motor, or a screw this build cannot name.
    """
    fix = spec.motor_fixing
    if fix is None:
        return None
    z0 = inside if side > 0 else inside - fix.recess_depth
    shapes = []
    for cx, cy in spec.motor_bolt_centres:
        sketch = cq.Workplane("XY").workplane(offset=z0)
        if fix.from_drive:
            sketch = sketch.center(cx, cy).circle(fix.recess / 2.0)
        else:
            sketch = sketch.polyline(hexagon(cx, cy, fix.recess, 0.0)).close()
        shapes.append(sketch.extrude(fix.recess_depth).val())
    return cq.Workplane("XY").add(cq.Compound.makeCompound(shapes))


def motor_screws(spec: GearSpec) -> dict[str, cq.Workplane]:
    """The motor's screws, and their nuts where it takes them, where they go."""
    from ..viz.mesh import motor_screw_layout

    fix = spec.motor_fixing
    if fix is None:
        return {}
    size = fix.size
    layout = motor_screw_layout(spec)
    screws, nuts = [], []
    for cx, cy in spec.motor_bolt_centres:
        head = next((z0, z1) for what, z0, z1 in layout if what == "head")
        shank = next((z0, z1) for what, z0, z1 in layout if what == "shank")
        try:
            screw = _cap_screw(size, shank[1] - shank[0], 0.0)
            # Built head-down at the origin, shank up; turned over when the
            # head is on the upper end, then put where its head goes.
            if (head[0] + head[1]) / 2.0 > (shank[0] + shank[1]) / 2.0:
                screw = screw.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180.0)
                screw = screw.moved(cq.Location(cq.Vector(cx, cy, head[0])))
            else:
                screw = screw.moved(cq.Location(cq.Vector(cx, cy, head[1])))
        except Exception:                   # a kernel that will not cut it
            screw = (cq.Workplane("XY").workplane(offset=shank[0]).center(cx, cy)
                     .circle(size.d / 2.0).extrude(shank[1] - shank[0])
                     .union(cq.Workplane("XY").workplane(offset=head[0])
                            .center(cx, cy).circle(size.head_d / 2.0)
                            .extrude(head[1] - head[0])).val())
        screws.append(screw)
        for what, z0, _z1 in layout:
            if what == "nut":
                try:
                    nut = _hex_nut(size, 0.0)
                except Exception:
                    nut = (cq.Workplane("XY").polyline(hexagon(0.0, 0.0, size.nut_s, 0.0))
                           .close().extrude(size.nut_m).val())
                nuts.append(nut.moved(cq.Location(cq.Vector(cx, cy, z0))))
    out = {"motor_screws": cq.Workplane("XY").add(cq.Compound.makeCompound(screws))}
    if nuts:
        out["motor_nuts"] = cq.Workplane("XY").add(cq.Compound.makeCompound(nuts))
    return out


def _motor_pattern(wp: cq.Workplane, spec: GearSpec) -> cq.Workplane:
    """Drill a motor's bolt pattern from whichever face ``wp`` is on.

    NEMA frames put four bolts on a *square*; metric flanges use a circle.
    Which of the two it is comes off the frame rather than being assumed here,
    for the reason the table exists at all - four holes drawn on a circle of the
    same size land where a NEMA motor has nothing.
    """
    frame = spec.motor_face
    if frame.square:
        return wp.rarray(frame.bolt_span, frame.bolt_span, 2, 2).hole(
            frame.bolt_diameter)
    return wp.polarArray(frame.bolt_span / 2.0, 0, 360, frame.bolt_count).hole(
        frame.bolt_diameter)


def output_flange(spec: GearSpec,
                  placements: Sequence[BearingPlacement] = (),
                  bores: SeatBores | None = None) -> cq.Workplane:
    """Carrier plate, its output pins, the boss, and - on a ring-output drive -
    the base the whole gearbox is bolted down by.

    The boss is what makes the output bearing a real part rather than a line on
    a schedule: it stands out through the output end plate, carries that bearing
    on its outside, and holds the outboard shaft support on its inside.  Without
    it the flange was a plate floating on six pins.

    What is on the *end* of that boss depends on which member turns.  Off the
    carrier, nothing: the boss is the output shaft end and a coupling grips it.
    Off the ring, this part is the one that stands still, so it is what the
    drive is mounted by - the boss ends in a plate the size of the housing with
    the motor's pattern in it, and the protrusion that used to be grip length
    becomes the standoff that keeps that plate clear of the turning one.

    One part rather than a base bolted to a carrier, because it is one rigid
    body: everything here is grounded together, and a joint drawn across it
    would be a joint with no load to carry and a fastener pattern to invent.
    """
    plate_r = spec.output_bolt_circle_radius + spec.output_pin_diameter
    t = spec.output_flange_thickness
    shank = pin_shank_diameter(placements, "bearing_output_pins",
                               spec.output_pin_diameter)
    # Bored to the outboard shaft support, which sits in it.
    bores = bores or seat_bores(spec)
    bore = bores.hub
    plate = cq.Workplane("XY").circle(plate_r).extrude(-t)
    hub = (cq.Workplane("XY").workplane(offset=-t)
           .circle(spec.hub_diameter / 2.0)
           .extrude(-(spec.plate_thickness + spec.boss_protrusion)))
    pins = (cq.Workplane("XY")
            .polarArray(spec.output_bolt_circle_radius, 0, 360, spec.output_pin_count)
            .circle(shank / 2.0)
            .extrude(spec.output_pin_length))
    body = plate.union(hub).union(pins)

    if spec.ground_frame_fitted:
        # This part is modelled in its *own* frame, with the carrier's top face
        # at zero - the assembly then drops it by the carrier drop.  So an
        # assembled height has to be lifted by that drop to be used here, and
        # the base was not: it came out one millimetre below the boss it stands
        # on, which is a part in two pieces.  The volume was right, which is why
        # nothing caught it, and a printer would have made two.
        bottom = spec.base_plate_bottom + CARRIER_DROP
        top = spec.boss_bottom + CARRIER_DROP
        if spec.base_has_flange:
            # A flange the housing's outline, thin, and the pad under the motor
            # the plate's whole thickness - see GearSpec.base_pad.
            flange = spec.base_flange_thickness
            kind, size = spec.base_pad
            pad = cq.Workplane("XY").workplane(offset=bottom)
            pad = (pad.circle(size / 2.0) if kind == "round" else
                   pad.polyline(spec.base_pad_outline()).close())
            base = (_outline_prism(spec, top - flange, flange)
                    .union(pad.extrude(top - bottom)))
        else:
            base = _outline_prism(spec, bottom, spec.plate_thickness)
        if spec.coupled:
            base = base.union(_motor_mount(spec, "output_flange", -1.0))
        elif spec.has_motor_face:
            frame = spec.motor_face
            # The register first, and only where the spigot is wider than the
            # bore already through here - a motor that pilots on 22 mm into a
            # 22 mm bore is already located by it.
            if frame.pilot_diameter > bore:
                base = (base.faces("<Z").workplane()
                        .circle(frame.pilot_diameter / 2.0)
                        .cutBlind(-min(frame.pilot_depth,
                                       spec.plate_thickness / 2.0)))
            base = _motor_pattern(base.faces("<Z").workplane(), spec)
            recesses = _motor_recesses(
                spec, spec.base_plate_bottom + CARRIER_DROP + spec.plate_thickness,
                -1.0)
            if recesses is not None:
                base = base.cut(recesses)
        # Tapped up into the flange from the machine's side, the side it is
        # bolted down from: the housing turns over the other.
        if spec.base_mount_centres:
            base = base.cut(cq.Workplane("XY")
                            .workplane(offset=top - spec.base_flange_thickness)
                            .pushPoints(spec.base_mount_centres)
                            .circle(spec.base_mount_hole / 2.0)
                            .extrude(spec.base_mount_depth))
        body = body.union(base)

    # Bored through the lot in one go, from the far face: the shaft passes
    # through all of it and a two-diameter bore here would be a fit this app has
    # no reason to claim.
    body = (body.faces("<Z").workplane()
            .circle(bore / 2.0)
            .cutThruAll())
    # The plate's rim, the end of the boss and both mouths of the bore - the
    # output bearing goes on over one and a shaft support into the other.  Not
    # the pins: they are what the disc's holes and the rollers run on.
    # And the lip past the support, which holds its outer ring in against the
    # crank's thrust - down the rest of the boss, and on a frame through the
    # base as well, short of the motor's register.
    body = _add_shoulders(body, spec, bores.shoulders, "output_flange")
    # The boss's own end only off the carrier: on a frame both of its ends are
    # in the base and the carrier plate, so it has no edge of its own to break,
    # and asking for its radius found the circle a motor's screws on that
    # radius cut into the base instead - which the kernel would not chamfer.
    radii = [plate_r, bore / 2.0, *_lips(bores.shoulders, "output_flange")]
    if spec.ground_frame_fitted:
        radii.append(spec.housing_outer_radius)
    else:
        radii.append(spec.hub_diameter / 2.0)
    return _round_inside(_break_edges(body, radii, min(spec.edge_chamfer, t / 4.0),
                                      outline=spec if spec.ground_frame_fitted
                                      else None))


def end_cap(spec: GearSpec, placements: Sequence[BearingPlacement] = (),
            bores: SeatBores | None = None) -> cq.Workplane:
    """The carrier's mirror image at the far end of the output pins.

    It exists only on a ring-output drive, and it is what makes that drive a
    frame rather than a cantilever.  Three things come out of putting a second
    plate on the far end of the pins:

    * the pins are supported at both ends, so they carry ``F*L/4`` instead of
      ``F*L`` - the single biggest structural change in this app;
    * the housing gets a second bearing, so it can take a moment instead of
      hanging off one;
    * the pins become the frame's own fasteners, threaded into the carrier and
      headed on the outside of this plate, so there is nothing else to hold the
      two grounded plates together.

    Same plate radius as the carrier, because it is the same plate seen from the
    other end - and that radius is comfortably inside the barrel bore, which it
    has to be: this part lives *inside* the housing that turns around it.
    """
    plate_r = spec.output_bolt_circle_radius + spec.output_pin_diameter
    shank = pin_shank_diameter(placements, "bearing_output_pins",
                               spec.output_pin_diameter)
    plate = (cq.Workplane("XY").workplane(offset=spec.end_cap_bottom)
             .circle(plate_r)
             .extrude(spec.output_flange_thickness))
    # The boss the second output bearing rides on, and the outboard shaft
    # support sits in.  It reaches exactly through the input end plate and stops
    # flush with it: nothing grips this one, unlike the carrier's.
    boss = (cq.Workplane("XY").workplane(offset=spec.end_cap_top)
            .circle(spec.hub_diameter / 2.0)
            .extrude(spec.cap_boss_top - spec.end_cap_top))
    # Clearance for the pins, not a press fit: they are located by the carrier
    # at the other end, and a plate pressed onto six pins cannot be assembled.
    # Cut as explicit cylinders rather than drilled from a selected face - the
    # highest face of this part is the top of the boss, and a pattern drilled
    # from there at the bolt circle radius starts outside the metal.
    pins = (cq.Workplane("XY").workplane(offset=spec.end_cap_bottom)
            .polarArray(spec.output_bolt_circle_radius, 0, 360,
                        spec.output_pin_count)
            .circle((shank + spec.hole_clearance) / 2.0)
            .extrude(spec.output_flange_thickness))
    # Bored to the inboard shaft support, which sits in it, with the lip that
    # holds it in past it.
    bores = bores or seat_bores(spec)
    seat = bores.cap
    bore = (cq.Workplane("XY").workplane(offset=spec.end_cap_bottom)
            .circle(seat / 2.0)
            .extrude(spec.cap_boss_top - spec.end_cap_bottom))
    body = _add_shoulders(plate.union(boss).cut(pins).cut(bore), spec,
                          bores.shoulders, "end_cap")
    return _round_inside(_break_edges(
        body, [plate_r, spec.hub_diameter / 2.0, seat / 2.0,
               *_lips(bores.shoulders, "end_cap")],
        min(spec.edge_chamfer, spec.output_flange_thickness / 4.0)))


def housing_end_plate(spec: GearSpec, bore: float, motor_face: bool = False,
                      shoulders: Sequence[Shoulder] = ()) -> cq.Workplane:
    """One of the two plates that close the housing.

    Same outside as the housing, because they bolt to it face to face.  What
    differs between them is the hole - the input side is bored for the shaft
    support, the output side for the bearing the whole drive turns on - and what
    is cut into the outside of it.

    ``motor_face`` marks the *input-side* plate rather than promising a motor:
    what that face is for depends on which member turns.  With the housing
    grounded it is where the motor bolts on.  With the housing as the output it
    is the face that turns, so the motor has gone to the carrier's base and this
    is what the driven machine bolts to instead - the same plate, the opposite
    end of the power flow.
    """
    plate = _outline_prism(spec, 0.0, spec.plate_thickness).cut(
        cq.Workplane("XY").workplane(offset=-1.0)
        .circle(max(bore, 1e-3) / 2.0).extrude(spec.plate_thickness + 2.0))

    if spec.tie_bolt_centres:
        # The input plate of a ring drive is tapped for them: drawn at the tap
        # drill, which is the metal the thread is cut in - or at the hole a
        # heat-set insert is pressed into, where it takes one.
        tapped = motor_face and spec.tie_screws_tapped
        hole = spec.tie_thread_hole if tapped else spec.housing_bolt_diameter
        plate = plate.cut(cq.Workplane("XY").workplane(offset=-1.0)
                          .pushPoints(spec.tie_bolt_centres)
                          .circle(hole / 2.0)
                          .extrude(spec.plate_thickness + 2.0))
    plate = _cut_dowel_holes(plate, spec, 0.0, spec.plate_thickness,
                             spec.plate_dowel_hole)

    if motor_face and spec.ground_frame_fitted:
        if spec.output_bolt_count:
            plate = (plate.faces(">Z").workplane()
                     .polarArray(spec.output_face_bolt_radius,
                                 math.degrees(spec.output_bolt_phase), 360,
                                 spec.output_bolt_count)
                     .hole(spec.output_bolt_diameter))
    elif motor_face and spec.coupled:
        plate = plate.union(_motor_mount(spec, "input_end_plate", 1.0))
    elif motor_face and spec.has_motor_face:
        frame = spec.motor_face
        # The register first: a shallow recess on the outside face that the
        # motor's spigot drops into, and the only thing that actually centres it
        # - four clearance holes on their own leave it free to sit anywhere
        # inside them.
        if frame.pilot_diameter > bore:
            plate = (plate.faces(">Z").workplane()
                     .circle(frame.pilot_diameter / 2.0)
                     .cutBlind(-frame.pilot_depth))
        plate = _motor_pattern(plate.faces(">Z").workplane(), spec)
        # The inside face is local zero, and the discs run on it.
        recesses = _motor_recesses(spec, 0.0, 1.0)
        if recesses is not None:
            plate = plate.cut(recesses)
    # The lip on the outside face that holds the bearing's outer ring in.
    # ``shoulders`` is every seat's; this plate takes the ones cut in it.
    host = "input_end_plate" if motor_face else "output_end_plate"
    plate = _add_shoulders(plate, spec, shoulders, host)
    # The rim, held clear of the tie-bolt holes as the barrel's is, and both
    # mouths of the bore, which is a bearing seat in either plate.
    return _round_inside(_break_edges(
        plate, [spec.housing_outer_radius, max(bore, 1e-3) / 2.0,
                *_lips(shoulders, host)],
        plate_edge_break(spec, shoulders, host), outline=spec))


#: A deep-groove ball's diameter as a share of the bearing's radial section,
#: ``(D - d) / 2``.  Catalogues do not print it, and it is close to the same
#: number across a series: it puts a 6004's ball at 6.6 mm against the 6.35 mm
#: it has.
BALL_SHARE = 0.60
#: How far each ring's shoulder reaches towards the pitch circle, as a share of
#: the ball - leaving a gap between the rings that the balls show through.
SHOULDER_SHARE = 0.36
#: A drawn cup's wall, as a share of the radial section; the rest is needle.
CUP_SHARE = 0.25
#: Needle centres apart, in needle diameters: the cage between them.  Sets the
#: cup at about two thirds of its envelope, where the published masses of HK
#: cups put it - see :data:`~cycloidgen.analysis.bearings.BEARING_FILL`.
CAGE_PITCH = 1.45


def _ball_bearing(d: float, D: float, B: float) -> list[cq.Shape]:
    """A deep-groove ball bearing on the Z axis, from z = 0 to ``B``.

    Two rings with their corners broken and a raceway in each, and a full
    pitch circle of balls showing between them.  No cage: it is what holds the
    balls apart, and the balls held apart is what it would show.
    """
    section = (D - d) / 2.0
    ball = BALL_SHARE * section
    pitch = (D + d) / 4.0                                  # pitch radius
    inner_out = pitch - SHOULDER_SHARE * ball
    outer_in = pitch + SHOULDER_SHARE * ball
    corner = min(0.3, B / 10.0, section / 8.0)

    def ring(r_out: float, r_in: float) -> cq.Shape:
        body = cq.Workplane("XY").circle(r_out).circle(r_in).extrude(B)
        body = body.edges("%CIRCLE").chamfer(corner).val()
        groove = cq.Solid.makeTorus(pitch, 0.52 * ball,
                                    pnt=cq.Vector(0, 0, B / 2.0))
        return body.cut(groove)

    count = max(int(2.0 * math.pi * pitch / (1.25 * ball)), 3)
    balls = [cq.Solid.makeSphere(ball / 2.0, cq.Vector(
        pitch * math.cos(2.0 * math.pi * k / count),
        pitch * math.sin(2.0 * math.pi * k / count), B / 2.0),
        angleDegrees1=-90, angleDegrees2=90)
        for k in range(count)]
    return [ring(D / 2.0, outer_in), ring(inner_out, d / 2.0), *balls]


def _drawn_cup(fw: float, D: float, C: float) -> list[cq.Shape]:
    """A drawn-cup needle roller bearing on the Z axis, from z = 0 to ``C``.

    A thin pressed cup with a lip turned in at each end, and needles riding
    directly on whatever goes through it - which is why its bore is ``Fw``, the
    needles' own inscribed circle, and not a ring.
    """
    section = (D - fw) / 2.0
    wall = CUP_SHARE * section
    needle = section - wall
    lip = fw / 2.0 + 0.55 * needle
    profile = [(D / 2.0, 0.0), (D / 2.0, C), (lip, C), (lip, C - wall),
               (D / 2.0 - wall, C - wall), (D / 2.0 - wall, wall), (lip, wall),
               (lip, 0.0)]
    cup = (cq.Workplane("XZ").polyline(profile).close()
           .revolve(360.0, (0, 0, 0), (0, 1, 0)).val())
    length = C - 2.0 * wall - 0.2
    centre = fw / 2.0 + needle / 2.0
    count = max(int(2.0 * math.pi * centre / (CAGE_PITCH * needle)), 3)
    needles = [cq.Solid.makeCylinder(needle / 2.0, length, cq.Vector(
        centre * math.cos(2.0 * math.pi * k / count),
        centre * math.sin(2.0 * math.pi * k / count), wall + 0.1))
        for k in range(count)]
    return [cup, *needles]


def _bearing_shapes(placement: BearingPlacement, width: float) -> list[cq.Shape]:
    """One bearing of ``placement``, at the origin: races and rolling elements
    where the catalogue says what kind it is, a plain ring where it does not."""
    part = BY_NAME.get(placement.catalogue)
    d, D = placement.bore, placement.outer
    try:
        if part is not None and part.kind == "needle":
            return _drawn_cup(d, D, width)
        if part is not None:
            return _ball_bearing(d, D, width)
    except Exception:                       # a kernel that will not cut it
        pass
    return [cq.Workplane("XY").circle(D / 2.0).circle(d / 2.0)
            .extrude(width).val()]


def bearing_solids(spec: GearSpec,
                   placements: Sequence[BearingPlacement] | None = None
                   ) -> dict[str, cq.Workplane]:
    """The bearings, keyed by part name: rings, races and rolling elements.

    Drawn as the part rather than as the room it takes, because a CAD model
    with a grey annulus where a bearing goes is a sketch, and anyone opening
    the STEP file sees the bearing before they see anything else.  Each one is
    a compound of separate solids - rings, balls, needles, a cup - and not a
    union: they are separate pieces of steel, and fusing a few hundred needles
    into one body is a boolean that costs seconds to say nothing.  One bearing
    is built per placement and copied to every position it has.

    Built at their **assembled** height rather than in a part-local frame,
    because that is the one thing they share with the mesh: the two differ by a
    pure axial shift on some parts, and a ring built where it ends up needs only
    the host's turn and offset in the plane.
    """
    if placements is None:
        placements = placements_for_spec(spec)
    out: dict[str, cq.Workplane] = {}
    for placement in placements:
        made: dict[float, list[cq.Shape]] = {}
        shapes: list[cq.Shape] = []
        for r in placement.rings:
            width = round(r.z1 - r.z0, 9)
            if width not in made:
                made[width] = _bearing_shapes(placement, width)
            shapes += [s.moved(cq.Location(cq.Vector(r.cx, r.cy, r.z0)))
                       for s in made[width]]
        if shapes:
            out[placement.name] = cq.Workplane("XY").add(
                cq.Compound.makeCompound(shapes))
    return out


def _ring(s: Spacer) -> cq.Workplane:
    """One spacer, where it goes."""
    return (cq.Workplane("XY").workplane(offset=s.z0)
            .circle(s.outer / 2.0).circle(s.bore / 2.0).extrude(s.length))


def spacer_parts(spec: GearSpec,
                 bores: SeatBores | None = None) -> dict[str, cq.Workplane]:
    """The shaft's two locating sleeves and the washers between the discs.

    Built where they assemble, as the end cap is: they have no other frame worth
    giving them, and a printer lays a part flat whatever height it came at.
    The sleeves are a part each because they are different lengths; the
    washers are one part however many there are, as the ring pins are, because
    every one is the same washer.  No edge break - a millimetre-thick washer
    would give most of itself to one.
    """
    bores = bores or seat_bores(spec)
    out: dict[str, cq.Workplane] = {}
    washers: cq.Workplane | None = None
    for s in bores.spacers:
        if s.name.startswith("washer"):
            washers = _ring(s) if washers is None else washers.union(_ring(s))
        else:
            out[s.name] = _ring(s)
    if washers is not None:
        out["washers"] = washers
    return out


def _cut_dowel_holes(body: cq.Workplane, spec: GearSpec, z0: float,
                     length: float, diameter: float) -> cq.Workplane:
    """Cut the plate dowels' holes through ``body`` from ``z0`` for ``length``.

    ``diameter`` is the dowel's in the barrel, a press, and
    :attr:`~cycloidgen.core.spec.GearSpec.plate_dowel_hole` in a plate - the
    dowel's again when machined, since the fit is a drawing note and not a
    hundredth of a millimetre a solid could show.
    """
    centres = spec.plate_dowel_centres
    if not centres:
        return body
    holes = (cq.Workplane("XY").workplane(offset=z0 - 1.0)
             .pushPoints(centres).circle(diameter / 2.0)
             .extrude(length + 2.0))
    return body.cut(holes)


def plate_dowels(spec: GearSpec) -> cq.Workplane | None:
    """The dowels themselves, as one multi-solid part; None with none.

    Bought, like the bolts, so in the assembly and not the per-part export.
    Plain ground cylinders at the nominal diameter: an ISO 8734 pin's end
    chamfers are a few hundredths of a gram.
    """
    spans = spec.plate_dowel_spans
    if not spans:
        return None
    r = spec.plate_dowel_diameter / 2.0
    shapes = [cq.Solid.makeCylinder(r, z1 - z0, cq.Vector(cx, cy, z0))
              for z0, z1 in spans for cx, cy in spec.plate_dowel_centres]
    return cq.Workplane("XY").add(cq.Compound.makeCompound(shapes))


def _bolt_centres(spec: GearSpec) -> list[tuple[float, float, float]]:
    """Where each tie bolt goes, and the angle it sits at round the axis."""
    return [(x, y, math.atan2(y, x)) for x, y in spec.tie_bolt_centres]


def tie_bolts(spec: GearSpec) -> cq.Workplane:
    """The bolts themselves, as one multi-solid part.

    Bought, like the bearings and the pins, so they are in the assembly and not
    in the per-part export: a STEP file of a cap screw is a thing to order, not
    a thing to make.  Drawn at the nominal size rather than at the clearance
    hole, because that is the bolt - the gap round it is the fit.

    Socket head cap screws, heads on the output end plate, with a nut on the
    input one - which is where the head used to have nowhere to go: that face
    is where a motor bolts on, and a counterbore will not fit the wall.  The
    bolt circle runs down the middle of the housing wall, so on the default
    6 mm wall an M4's 8 mm counterbore would break out through the rim; the
    head stands on the face instead, and its 7 mm overhangs the rim by half a
    millimetre.  The length is ISO 4762's next preferred one past the stack, the
    nut and two threads, and the screw is drawn at that length.

    A hole too small for any size in :mod:`~cycloidgen.core.fasteners` is drawn
    as it was before there was a table: a shank at the hole, a little under.
    """
    size = spec.tie_bolt_size
    z0 = spec.tie_bolt_bottom
    if size is None:
        shank = TIE_BOLT_NOMINAL * spec.housing_bolt_diameter
        return (cq.Workplane("XY").workplane(offset=z0)
                .pushPoints(spec.tie_bolt_centres)
                .circle(shank / 2.0)
                .extrude(spec.tie_bolt_length))
    length = spec.tie_screw_length or spec.tie_bolt_length
    shapes = []
    for cx, cy, a in _bolt_centres(spec):
        try:
            bolt = _cap_screw(size, length, a)
        except Exception:                   # a kernel that will not cut it
            bolt = (cq.Workplane("XY").circle(size.d / 2.0).extrude(length)
                    .union(cq.Workplane("XY").workplane(offset=-size.head_h)
                           .circle(size.head_d / 2.0).extrude(size.head_h))
                    .val())
        shapes.append(bolt.moved(cq.Location(cq.Vector(cx, cy, z0))))
    return cq.Workplane("XY").add(cq.Compound.makeCompound(shapes))


def _thread_profile(size: MetricSize, z_start: float, z_end: float) -> list:
    """Half a threaded shank, as the (r, z) outline a revolve turns into one.

    Rings rather than a helix: a V-groove every pitch, as deep as an external
    thread's - which is a thread to the eye and to the volume, and a few dozen
    conical faces to the kernel where a true helix is a swept surface it has to
    trim against everything it touches.  Ends in the usual 45-degree point,
    down to the root.
    """
    crest, root = size.d / 2.0, size.d / 2.0 - THREAD_DEPTH * size.pitch
    points = [(0.0, z_start), (crest, z_start)]
    z = z_start
    while z + size.pitch <= z_end - (crest - root) + 1e-9:
        points += [(root, z + size.pitch / 2.0), (crest, z + size.pitch)]
        z += size.pitch
    if z_end - (crest - root) > z + 1e-6:
        points.append((crest, z_end - (crest - root)))
    points += [(root, z_end), (0.0, z_end)]
    return points


def _cap_screw(size: MetricSize, length: float, flat: float) -> cq.Shape:
    """An ISO 4762 socket head cap screw, its head under ``z = 0``.

    The head's free end is broken, the socket is a hexagon ending in the cone
    its drill left, and the shank is plain to where ISO 4762's reference thread
    length, ``2d + 12``, begins - threaded from there to the point.
    """
    threaded = thread_length(size, length)
    plain = (cq.Workplane("XY").circle(size.d / 2.0)
             .extrude(length - threaded).val()) if threaded < length else None
    thread = (cq.Workplane("XZ")
              .polyline(_thread_profile(size, length - threaded, length)).close()
              .revolve(360.0, (0, 0, 0), (0, 1, 0)).val())
    head = (cq.Workplane("XY").workplane(offset=-size.head_h)
            .circle(size.head_d / 2.0).extrude(size.head_h)
            .faces("<Z").edges().chamfer(0.08 * size.head_d).val())
    corner = size.socket / math.sqrt(3.0)
    socket = (cq.Workplane("XY").workplane(offset=-size.head_h)
              .polyline(hexagon(0.0, 0.0, size.socket, flat)).close()
              .extrude(size.socket_depth).val())
    drill = cq.Solid.makeCone(corner, 0.0, 0.6 * corner, cq.Vector(
        0, 0, -size.head_h + size.socket_depth))
    body = head.fuse(thread) if plain is None else head.fuse(plain, thread)
    return body.cut(socket.fuse(drill)).clean()


def _hex_nut(size: MetricSize, flat: float) -> cq.Shape:
    """An ISO 4032 nut from ``z = 0`` to its height, bore at the nominal size.

    Both faces chamfered at thirty degrees from just inside the flats, which is
    what puts the ring on a nut's face: a hexagonal prism cut by a double cone.
    """
    corners = size.nut_s / math.sqrt(3.0)
    face = 0.475 * size.nut_s
    drop = (corners - face) * math.tan(math.radians(30.0))
    crown = (cq.Workplane("XZ")
             .polyline([(0.0, 0.0), (face, 0.0), (corners + 0.01, drop + 0.01),
                        (corners + 0.01, size.nut_m - drop - 0.01),
                        (face, size.nut_m), (0.0, size.nut_m)]).close()
             .revolve(360.0, (0, 0, 0), (0, 1, 0)).val())
    prism = (cq.Workplane("XY").polyline(hexagon(0.0, 0.0, size.nut_s, flat))
             .close().extrude(size.nut_m).val())
    bore = cq.Workplane("XY").circle(size.d / 2.0).extrude(size.nut_m).val()
    return prism.intersect(crown).cut(bore).clean()


def tie_nuts(spec: GearSpec) -> cq.Workplane | None:
    """The nuts on the input end plate, one per tie bolt, or ``None``.

    ISO 4032, a flat towards the rim so the nut overhangs it no further than
    the head does at the other end.  None when the bolt is not one the table
    knows - there is then no nut to name either.
    """
    size = spec.tie_bolt_size
    if size is None or spec.tie_screws_tapped:
        return None
    face = spec.barrel_top + spec.plate_thickness
    shapes = []
    for cx, cy, a in _bolt_centres(spec):
        try:
            nut = _hex_nut(size, a)
        except Exception:                   # a kernel that will not cut it
            nut = (cq.Workplane("XY").polyline(hexagon(0.0, 0.0, size.nut_s, a))
                   .close().extrude(size.nut_m)
                   .cut(cq.Workplane("XY").circle(size.d / 2.0)
                        .extrude(size.nut_m)).val())
        shapes.append(nut.moved(cq.Location(cq.Vector(cx, cy, face))))
    return cq.Workplane("XY").add(cq.Compound.makeCompound(shapes))


def build_assembly(spec: GearSpec) -> cq.Assembly:
    """Full gearbox at crank angle zero, each disc on its own phase."""
    assy = cq.Assembly(name=f"cycloidal_{spec.ratio}to1")
    # One schedule for both: where the bearings go, and what their seats are
    # bored to.  Two lookups could not disagree, but they would size every
    # bearing twice.
    choices = bearing_schedule(spec)
    placements = tuple(bearing_placements(spec, choices))
    bores = seat_bores(spec, choices)

    #: Where each part was put, so that a bearing can be placed against its host
    #: rather than have the same pose worked out a second time.  Only the planar
    #: part of it: the rings are already built at their assembled height.
    planar: dict[str, cq.Location] = {}
    identity = cq.Location(cq.Vector(0, 0, 0))

    assy.add(ring_housing(spec), name="housing",
             color=_part_colour("housing"))
    planar["housing"] = identity
    if not spec.ring_pins_integral:
        assy.add(ring_pins(spec, placements), name="ring_pins",
                 color=_part_colour("ring_pins"))
        planar["ring_pins"] = identity

    z = 0.0
    for i, (phase, hole_phase) in enumerate(zip(spec.disc_phases,
                                                spec.disc_hole_phases,
                                                strict=True)):
        cx = spec.eccentricity * math.cos(phase)
        cy = -spec.eccentricity * math.sin(phase)
        rot = math.degrees(phase / spec.lobes)
        assy.add(disc_solid(spec, hole_phase, bores.disc), name=f"disc_{i + 1}",
                 loc=cq.Location(cq.Vector(cx, cy, z), cq.Vector(0, 0, 1), rot),
                 color=_part_colour(f"disc_{i + 1}"))
        planar[f"disc_{i + 1}"] = cq.Location(cq.Vector(cx, cy, 0.0),
                                              cq.Vector(0, 0, 1), rot)
        z += spec.disc_thickness + spec.disc_gap

    assy.add(eccentric_shaft(spec), name="eccentric_shaft",
             color=_part_colour("eccentric_shaft"))
    for name, body in spacer_parts(spec, bores).items():
        assy.add(body, name=name, color=_part_colour(name))
    assy.add(output_flange(spec, placements, bores), name="output_flange",
             loc=cq.Location(cq.Vector(0, 0, -CARRIER_DROP)),
             color=_part_colour("output_flange"))
    planar["eccentric_shaft"] = planar["output_flange"] = identity

    # The end cap, on a ring-output drive: the far end of the output pins, and
    # the second thing the housing turns on.  Built at its assembled height, so
    # it needs no placement of its own.
    if spec.ground_frame_fitted:
        assy.add(end_cap(spec, placements, bores), name="end_cap",
                 color=_part_colour("end_cap"))
        planar["end_cap"] = identity

    # The plates close the housing, one on each face.  The output one sits
    # outboard of the carrier, so the boss passes through it and the pins stay
    # inside; the input one sits straight on top of the barrel - which reaches
    # past the discs to cover the end cap when there is one.
    assy.add(housing_end_plate(spec, bores.input_plate, motor_face=True,
                               shoulders=bores.shoulders),
             name="input_end_plate",
             loc=cq.Location(cq.Vector(0, 0, spec.barrel_top)),
             color=_part_colour("input_end_plate"))
    assy.add(housing_end_plate(spec, bores.output_plate,
                               shoulders=bores.shoulders),
             name="output_end_plate",
             loc=cq.Location(cq.Vector(
                 0, 0, -CARRIER_DROP - spec.output_flange_thickness
                 - spec.plate_thickness)),
             color=_part_colour("output_end_plate"))
    planar["input_end_plate"] = planar["output_end_plate"] = identity

    for name, body in bearing_solids(spec, placements).items():
        host = next(p.host for p in placements if p.name == name)
        assy.add(body, name=name, loc=planar[host], color=_part_colour(name))

    for name, body in motor_screws(spec).items():
        assy.add(body, name=name, color=_part_colour("tie_bolts"))
    coupling = coupling_solid(spec)
    if coupling is not None:
        assy.add(coupling, name="coupling", color=_part_colour("eccentric_shaft"))
    hub = rotor_hub(spec)
    if hub is not None:
        assy.add(hub, name="rotor_hub", color=_part_colour("rotor_hub"))

    dowels = plate_dowels(spec)
    if dowels is not None:
        assy.add(dowels, name="plate_dowels", color=_part_colour("plate_dowels"))
    if spec.housing_bolt_count:
        assy.add(tie_bolts(spec), name="tie_bolts", color=_part_colour("tie_bolts"))
        nuts = tie_nuts(spec)
        if nuts is not None:
            assy.add(nuts, name="tie_nuts", color=_part_colour("tie_nuts"))
    return assy


def write_step(spec: GearSpec, path: str | Path) -> Path:
    """Export the whole gearbox as one STEP assembly."""
    path = Path(path)
    build_assembly(spec).export(str(path))
    return path


def parts(spec: GearSpec) -> dict[str, cq.Workplane]:
    """Every distinct part, each in its own frame, keyed by file name.

    One entry per *distinct* disc: the hole pattern differs between them unless
    ``output_pin_count`` happens to be a multiple of ``2*lobes``, and shipping
    one file for a stack of different parts is how a drive gets built wrong.

    Made parts only.  A bearing is bought, and an STL of one is a fit check at
    best and a part someone tries to print at worst; they are in the assembly,
    which is where fit is checked, and in the BOM, which is where you order them.
    """
    choices = bearing_schedule(spec)
    placements = tuple(bearing_placements(spec, choices))
    bores = seat_bores(spec, choices)
    out = {
        "housing": ring_housing(spec),
        "eccentric_shaft": eccentric_shaft(spec),
        **spacer_parts(spec, bores),
        "output_flange": output_flange(spec, placements, bores),
        "input_end_plate": housing_end_plate(spec, bores.input_plate,
                                             motor_face=True,
                                             shoulders=bores.shoulders),
        "output_end_plate": housing_end_plate(spec, bores.output_plate,
                                              shoulders=bores.shoulders),
    }
    if spec.ground_frame_fitted:
        out["end_cap"] = end_cap(spec, placements, bores)
    hub = rotor_hub(spec)
    if hub is not None:
        out["rotor_hub"] = hub
    # Integral pins are not a part, so there is no file for them.  The bundle
    # is declared from this dict, so leaving an empty one in would put a solid
    # in the manifest that the housing already contains.
    if not spec.ring_pins_integral:
        out["ring_pins"] = ring_pins(spec, placements)
    phases = (spec.disc_hole_phases[:1] if spec.discs_are_identical
              else spec.disc_hole_phases)
    for name, hole_phase in zip(disc_names(spec), phases, strict=True):
        out[name] = disc_solid(spec, hole_phase, bores.disc)
    return out


def write_part_steps(spec: GearSpec, directory: str | Path) -> list[Path]:
    """Export every part as its own STEP solid.

    The assembly file is for looking at and for checking fit.  These are for
    handing one part to a machine shop, or to a CAM package that would otherwise
    make you fish the body out of an assembly first.

    Written through the assembly writer rather than as a bare solid, and the
    reason is the *name*.  A solid exported on its own carries whatever OCCT
    calls it, which is ``Open CASCADE STEP translator 7.9 2`` - so every file in
    this folder opened in a CAD package as that, eight parts told apart by a
    trailing number that has nothing to do with the part.  The file name was the
    only thing carrying the identity, and the file name is the first thing lost:
    importing a part into an assembly keeps the product name and drops the path.
    A part somebody means to build on has to say what it is from the inside.

    So each one goes out as a single named part under the design it came from,
    in the colour the assembly and the 3D view already give it.  It costs about
    1% of the file, and what arrives is ``cycloidal_21to1 > housing`` rather
    than a numbered translator string.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for name, wp in parts(spec).items():
        out = directory / f"{name}.step"
        one = cq.Assembly(name=f"cycloidal_{spec.ratio}to1")
        one.add(wp, name=name, color=_part_colour(name))
        one.export(str(out))
        written.append(out)
    return written


def write_stls(spec: GearSpec, directory: str | Path, prefix: str = "") -> list[Path]:
    """Export every part as its own STL - STL has no assembly structure.

    A printed part goes out ready to print: on the face that needs the least
    holding up, its bed edge broken back for the first layer, standing on z=0
    - see :mod:`cycloidgen.export.printing`.  The rest are as they are modelled.
    """
    from .printing import print_ready

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    tol = spec.stl_linear_tolerance
    written: list[Path] = []
    for name, wp in parts(spec).items():
        out = directory / f"{prefix}{name}.stl"
        cq.exporters.export(print_ready(spec, name, wp.val()), str(out),
                            tolerance=tol, angularTolerance=0.1)
        written.append(out)
    return written
