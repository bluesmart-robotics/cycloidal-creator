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
    BearingPlacement,
    SeatBores,
    Shoulder,
    bearing_placements,
    bearing_schedule,
    pin_shank_diameter,
    placements_for_spec,
    seat_bores,
)
from ..core import profile as prof
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
    "tie_bolts",
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


def _break_edges(wp: cq.Workplane, radii: Sequence[float],
                 size: float) -> cq.Workplane:
    """Chamfer the outside circular edges of ``wp`` that lie on ``radii``.

    Chosen by radius rather than by face, because a radius is what names a
    feature here - the rim, a bearing seat, a shaft end - whichever face of the
    part it turns up on, and the inside corners at the same radius are dropped
    by :func:`_convex` rather than listed.
    """
    if size <= 0.0:
        return wp
    body = wp.val()
    edges = [e for e in body.Edges()
             if e.geomType() == "CIRCLE"
             and any(abs(e.radius() - r) < 1e-6 for r in radii)
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
    if not spec.housing_bolt_count:
        return spec.housing_wall
    return (spec.housing_outer_radius - spec.housing_bolt_radius
            - spec.housing_bolt_diameter / 2.0)


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
    body = (cq.Workplane("XY").workplane(offset=z0)
            .circle(spec.housing_outer_radius)
            .circle(spec.pin_circle_radius)
            .extrude(h))
    pins = (cq.Workplane("XY").workplane(offset=z0)
            .polarArray(spec.pin_circle_radius, 0, 360, spec.pin_count)
            .circle(spec.pin_radius)
            .extrude(h))
    body = body.union(pins) if spec.ring_pins_integral else body.cut(pins)

    # The bolts that hold the plates on have to pass through the thing they are
    # clamping.  Both plates were drilled for them and the bill of materials
    # orders them; the barrel was not, so the exported assembly was a gearbox
    # whose six tie bolts land on solid wall.
    if spec.housing_bolt_count:
        bolts = (cq.Workplane("XY").workplane(offset=z0)
                 .polarArray(spec.housing_bolt_radius, 0, 360,
                             spec.housing_bolt_count)
                 .circle(spec.housing_bolt_diameter / 2.0)
                 .extrude(h))
        body = body.cut(bolts)
    # The rim only.  The bore and its pockets are where the pins sit and the
    # disc runs; and the rim's break stops at half the metal between it and the
    # tie-bolt holes, which on the default wall is under a millimetre.
    return _break_edges(body, [spec.housing_outer_radius],
                        min(spec.edge_chamfer, _rim_land(spec) / 2.0))


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
    """Input shaft with one eccentric cam per disc, phased around the axis."""
    overhang = spec.shaft_overhang
    shaft = (cq.Workplane("XY")
             .workplane(offset=-overhang)
             .circle(spec.input_shaft_diameter / 2.0)
             .extrude(spec.stack_height + 2 * overhang))
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
    # Both ends, and both ends of every cam - which is a bearing seat or, with
    # no cam bearing, the journal the disc runs on, and either way wants a
    # lead-in.  A cam is thinnest on the side it is offset towards the axis,
    # so that is what its break is held to; where the shaft runs into a cam is
    # an inside corner and is left alone.
    c = spec.edge_chamfer
    shaft = _break_edges(shaft, [spec.input_shaft_diameter / 2.0],
                         min(c, spec.input_shaft_diameter / 8.0))
    land = (spec.cam_diameter - spec.input_shaft_diameter) / 2.0 - spec.eccentricity
    return _break_edges(shaft, [spec.cam_diameter / 2.0], min(c, land / 2.0))


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
           .extrude(-(spec.plate_thickness + spec.output_boss_protrusion)))
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
        base = (cq.Workplane("XY")
                .workplane(offset=spec.base_plate_bottom + CARRIER_DROP)
                .circle(spec.housing_outer_radius)
                .extrude(spec.plate_thickness))
        if spec.has_motor_face:
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
    radii = [plate_r, spec.hub_diameter / 2.0, bore / 2.0,
             *_lips(bores.shoulders, "output_flange")]
    if spec.ground_frame_fitted:
        radii.append(spec.housing_outer_radius)
    return _break_edges(body, radii, min(spec.edge_chamfer, t / 4.0))


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
    return _break_edges(body,
                        [plate_r, spec.hub_diameter / 2.0, seat / 2.0,
                         *_lips(bores.shoulders, "end_cap")],
                        min(spec.edge_chamfer, spec.output_flange_thickness / 4.0))


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
    plate = (cq.Workplane("XY")
             .circle(spec.housing_outer_radius)
             .circle(max(bore, 1e-3) / 2.0)
             .extrude(spec.plate_thickness))

    if spec.housing_bolt_count:
        plate = (plate.faces(">Z").workplane()
                 .polarArray(spec.housing_bolt_radius, 0, 360,
                             spec.housing_bolt_count)
                 .hole(spec.housing_bolt_diameter))

    if motor_face and spec.ground_frame_fitted:
        if spec.output_bolt_count:
            plate = (plate.faces(">Z").workplane()
                     .polarArray(spec.output_face_bolt_radius,
                                 math.degrees(spec.output_bolt_phase), 360,
                                 spec.output_bolt_count)
                     .hole(spec.output_bolt_diameter))
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
    # The lip on the outside face that holds the bearing's outer ring in.
    # ``shoulders`` is every seat's; this plate takes the ones cut in it.
    host = "input_end_plate" if motor_face else "output_end_plate"
    plate = _add_shoulders(plate, spec, shoulders, host)
    # The rim, held clear of the tie-bolt holes as the barrel's is, and both
    # mouths of the bore, which is a bearing seat in either plate.
    return _break_edges(plate, [spec.housing_outer_radius, max(bore, 1e-3) / 2.0,
                                *_lips(shoulders, host)],
                        min(spec.edge_chamfer, spec.plate_thickness / 4.0,
                            _rim_land(spec) / 2.0))


def bearing_solids(spec: GearSpec,
                   placements: Sequence[BearingPlacement] | None = None
                   ) -> dict[str, cq.Workplane]:
    """The bearings as plain rings, keyed by part name.

    Rings, not races and rolling elements.  What the assembly has to answer is
    where a bearing goes and how much room it takes, and a modelled cage would
    add a few thousand faces to say nothing more about either.

    Built at their **assembled** height rather than in a part-local frame,
    because that is the one thing they share with the mesh: the two differ by a
    pure axial shift on some parts, and a ring built where it ends up needs only
    the host's turn and offset in the plane.
    """
    if placements is None:
        placements = placements_for_spec(spec)
    out: dict[str, cq.Workplane] = {}
    for placement in placements:
        body: cq.Workplane | None = None
        for r in placement.rings:
            ring = (cq.Workplane("XY").workplane(offset=r.z0)
                    .center(r.cx, r.cy)
                    .circle(placement.outer / 2.0)
                    .circle(placement.bore / 2.0)
                    .extrude(r.z1 - r.z0))
            body = ring if body is None else body.union(ring)
        if body is not None:
            out[placement.name] = body
    return out


def tie_bolts(spec: GearSpec) -> cq.Workplane:
    """The bolts themselves, as one multi-solid part.

    Bought, like the bearings and the pins, so they are in the assembly and not
    in the per-part export: a STEP file of a cap screw is a thing to order, not
    a thing to make.  Drawn at the nominal size rather than at the clearance
    hole, because that is the bolt - the gap round it is the fit.

    Shank only, and deliberately.  Where the head sits is a design decision this
    app has not been given - proud, counterbored, or threaded into a blind hole
    - and the one face it would land on is the input plate's outside, which is
    where a motor bolts on.  A head standing proud there is an interference; a
    counterbore is a change to a part this app dimensions for a shop to make.
    Drawing a head we cannot place correctly is worse than drawing the shank we
    can, and the bill of materials is what you order from.
    """
    shank = TIE_BOLT_NOMINAL * spec.housing_bolt_diameter
    return (cq.Workplane("XY").workplane(offset=spec.tie_bolt_bottom)
            .polarArray(spec.housing_bolt_radius, 0, 360, spec.housing_bolt_count)
            .circle(shank / 2.0)
            .extrude(spec.tie_bolt_length))


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

    if spec.housing_bolt_count:
        assy.add(tie_bolts(spec), name="tie_bolts", color=_part_colour("tie_bolts"))
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
        "output_flange": output_flange(spec, placements, bores),
        "input_end_plate": housing_end_plate(spec, bores.input_plate,
                                             motor_face=True,
                                             shoulders=bores.shoulders),
        "output_end_plate": housing_end_plate(spec, bores.output_plate,
                                              shoulders=bores.shoulders),
    }
    if spec.ground_frame_fitted:
        out["end_cap"] = end_cap(spec, placements, bores)
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
    """Export every part as its own STL - STL has no assembly structure."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    tol = spec.stl_linear_tolerance
    written: list[Path] = []
    for name, wp in parts(spec).items():
        out = directory / f"{prefix}{name}.stl"
        cq.exporters.export(wp.val(), str(out), tolerance=tol, angularTolerance=0.1)
        written.append(out)
    return written
