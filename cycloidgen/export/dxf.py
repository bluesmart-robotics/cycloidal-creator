"""DXF output, laid out for CAM rather than for looking at.

The profile goes out as a closed LWPOLYLINE sampled to the chord tolerance in the
spec.  Splines are more compact but far less portable between CAM packages.
"""
from __future__ import annotations

from pathlib import Path

import ezdxf
import numpy as np

from .. import __version__
from ..analysis.bearings import (
    bearing_placements,
    bearing_schedule,
    pin_shank_diameter,
    seat_bores,
)
from ..core import profile as prof
from ..core.spec import GearSpec
from .manifest import disc_names

__all__ = ["write_dxf", "write_part_dxfs", "write_sketch_dxfs"]

LAYERS = {
    "DISC_PROFILE": 3,
    "DISC_BORE": 1,
    "OUTPUT_HOLES": 5,
    "RING_PINS": 2,
    "HOUSING": 7,
    "PITCH": 8,
    # The plates carry two hole patterns that answer to different drawings - the
    # tie bolts are ours, the motor pattern is the motor maker's - and a shop
    # drilling one wants to be able to switch the other off.
    "BOLTS": 6,
    "MOTOR": 4,
    # The lip past a bearing seat: the through hole, where the seat is bored
    # from one face only.
    "SHOULDER": 30,
}


def _new_doc() -> tuple:
    doc = ezdxf.new("R2010", setup=True)
    doc.header["$INSUNITS"] = 4  # millimetres
    # Which build cut this file.  A DXF gets mailed to a shop and comes back as
    # metal six weeks later, by which time nothing else in the room remembers
    # what produced it.  Custom header vars are the standard place for it and
    # no CAM package cares what is in them.
    doc.header.custom_vars.append("GENERATOR", "cycloidgen")
    doc.header.custom_vars.append("CYCLOIDGEN_VERSION", __version__)
    for name, color in LAYERS.items():
        doc.layers.add(name=name, color=color)
    return doc, doc.modelspace()


def _title(msp, spec: GearSpec, text: str, radius: float) -> None:
    msp.add_text(text, height=max(radius / 30.0, 1.2),
                 dxfattribs={"layer": "HOUSING"}
                 ).set_placement((-radius, -radius - radius / 12.0))


def _cross(msp, c: tuple[float, float], size: float = 2.0) -> None:
    """Centre mark, which is what a hole is drilled from.

    A circle says where the metal is not; the crosshair says where the point
    goes.  Cheap to add and the difference between a file you can cut and a file
    you can also set up by hand from.
    """
    msp.add_line((c[0] - size, c[1]), (c[0] + size, c[1]), dxfattribs={"layer": "PITCH"})
    msp.add_line((c[0], c[1] - size), (c[0], c[1] + size), dxfattribs={"layer": "PITCH"})


def _polar(radius: float, count: int,
           phase: float = 0.0) -> list[tuple[float, float]]:
    """``count`` points on ``radius``, starting at ``phase`` - zero the way
    every array in this app is built, so a DXF cannot land the pattern the solid
    does not.  The one pattern that is not at zero is the output face's bolts,
    which are half a tie-bolt pitch round so the two circles interleave."""
    return [(radius * np.cos(phase + 2.0 * np.pi * k / count),
             radius * np.sin(phase + 2.0 * np.pi * k / count))
            for k in range(count)]


def _end_plate(spec: GearSpec, directory: Path, name: str, bore: float,
               motor_face: bool = False, shoulders=()) -> Path:
    """One of the two plates that close the housing, as a drilling drawing.

    Both are the housing outside diameter on the same tie-bolt circle; what
    differs is the hole in the middle - shaft support one end, the bearing the
    whole drive turns on at the other - and whether a motor bolts to the face.
    Everything here is read from the same properties
    :func:`~cycloidgen.export.solid.housing_end_plate` extrudes, including the
    rule that a register smaller than the bore is not a feature but a bore that
    has already swallowed it.
    """
    doc, msp = _new_doc()
    r = spec.housing_outer_radius
    msp.add_circle((0, 0), r, dxfattribs={"layer": "HOUSING"})
    msp.add_circle((0, 0), max(bore, 1e-3) / 2.0, dxfattribs={"layer": "DISC_BORE"})
    _cross(msp, (0.0, 0.0), size=max(bore / 2.0, 1.0) + 3.0)

    note = f"{name.replace('_', ' ')}  OD {2 * r:g}  bore {bore:g}"

    # The lip that holds the bearing in: the seat is bored from the inside to
    # a depth and the hole carries on through at the lip's size.  On its own
    # layer, because a laser cutting the plate cuts the through hole and the
    # seat is a second operation - and saying which face it is bored from is
    # the half of it a circle cannot.
    for lip in (s for s in shoulders if s.host == name):
        msp.add_circle((0, 0), lip.bore / 2.0, dxfattribs={"layer": "SHOULDER"})
        depth = spec.plate_thickness - lip.thickness
        note += (f" x {depth:g} deep from the inside, through {lip.bore:g}")

    if spec.housing_bolt_count:
        msp.add_circle((0, 0), spec.housing_bolt_radius,
                       dxfattribs={"layer": "PITCH", "linetype": "DASHED"})
        for c in _polar(spec.housing_bolt_radius, spec.housing_bolt_count):
            msp.add_circle(c, spec.housing_bolt_diameter / 2.0,
                           dxfattribs={"layer": "BOLTS"})
            _cross(msp, c)
        note += (f"  |  {spec.housing_bolt_count} x {spec.housing_bolt_diameter:g} "
                 f"tie on BC {2 * spec.housing_bolt_radius:g}")

    # ``motor_face`` marks the input-side plate rather than promising a motor.
    # On a ring-output drive that face turns, so the motor has moved to the
    # carrier's base and this is what the driven machine bolts to instead.
    if motor_face and spec.ground_frame_fitted and spec.output_bolt_count:
        for c in _polar(spec.output_face_bolt_radius, spec.output_bolt_count,
                        spec.output_bolt_phase):
            msp.add_circle(c, spec.output_bolt_diameter / 2.0,
                           dxfattribs={"layer": "MOTOR"})
            _cross(msp, c)
        note += (f"  |  {spec.output_bolt_count} x {spec.output_bolt_diameter:g} "
                 f"output on BC {2 * spec.output_face_bolt_radius:g}, "
                 f"{np.degrees(spec.output_bolt_phase):.1f} deg off the tie bolts")
    elif motor_face and spec.has_motor_face:
        frame = spec.motor_face
        if frame.pilot_diameter > bore:
            msp.add_circle((0, 0), frame.pilot_diameter / 2.0,
                           dxfattribs={"layer": "MOTOR"})
            note += f"  |  register {frame.pilot_diameter:g} x {frame.pilot_depth:g} deep"
        # A NEMA pattern is a *square*, so its bolts do not sit on the bolt
        # circle a polar array would put them on - stating one as the other is
        # the mistake this table exists to stop anybody making.
        if frame.square:
            h = frame.bolt_span / 2.0
            holes = [(sx * h, sy * h) for sx in (-1, 1) for sy in (-1, 1)]
            pattern = f"{frame.bolt_span:g} square"
        else:
            holes = _polar(frame.bolt_span / 2.0, frame.bolt_count)
            pattern = f"BC {frame.bolt_span:g}"
        for c in holes:
            msp.add_circle(c, frame.bolt_diameter / 2.0, dxfattribs={"layer": "MOTOR"})
            _cross(msp, c)
        note += (f"  |  {frame.name}: {len(holes)} x {frame.bolt_diameter:g} "
                 f"on {pattern}")

    _title(msp, spec, note, r)
    out = directory / f"{name}.dxf"
    doc.saveas(out)
    return out


def write_dxf(spec: GearSpec, path: str | Path) -> Path:
    """Write disc, ring and housing geometry to a single DXF."""
    path = Path(path)
    doc, msp = _new_doc()

    p = prof.profile_from_spec(spec)
    msp.add_lwpolyline(p.points, close=True, dxfattribs={"layer": "DISC_PROFILE"})

    # As bored: to the cam bearing's outside wherever there is one.
    bore_r = seat_bores(spec).disc / 2.0
    msp.add_circle((0, 0), bore_r, dxfattribs={"layer": "DISC_BORE"})

    # Each disc in a stack carries its hole pattern at a different angle, so a
    # multi-disc drive needs one layer per disc - they are different parts.
    hole_r = spec.output_hole_diameter / 2.0
    for disc_index, hole_phase in enumerate(spec.disc_hole_phases):
        layer = ("OUTPUT_HOLES" if spec.discs_are_identical
                 else f"OUTPUT_HOLES_DISC{disc_index + 1}")
        if layer not in doc.layers:
            doc.layers.add(name=layer, color=LAYERS["OUTPUT_HOLES"] + disc_index)
        for k in range(spec.output_pin_count):
            a = 2.0 * np.pi * k / spec.output_pin_count + hole_phase
            c = (spec.output_bolt_circle_radius * np.cos(a),
                 spec.output_bolt_circle_radius * np.sin(a))
            msp.add_circle(c, hole_r, dxfattribs={"layer": layer})
        if spec.discs_are_identical:
            break

    for k in range(spec.pin_count):
        a = 2.0 * np.pi * k / spec.pin_count
        c = (spec.pin_circle_radius * np.cos(a), spec.pin_circle_radius * np.sin(a))
        msp.add_circle(c, spec.pin_radius, dxfattribs={"layer": "RING_PINS"})

    msp.add_circle((0, 0), spec.pin_circle_radius,
                   dxfattribs={"layer": "PITCH", "linetype": "DASHED"})
    msp.add_circle((0, 0), spec.housing_outer_radius, dxfattribs={"layer": "HOUSING"})

    msp.add_text(
        f"cycloidal drive  i={spec.ratio}:1  N={spec.lobes} lobes / {spec.pin_count} pins  "
        f"R={spec.pin_circle_radius} Rr={spec.pin_radius} E={spec.eccentricity}  "
        f"clearance={spec.profile_clearance} ({spec.offset_mode.value})",
        height=2.0,
        dxfattribs={"layer": "HOUSING"},
    ).set_placement((-spec.housing_outer_radius, -spec.housing_outer_radius - 8))

    doc.saveas(path)
    return path


def write_part_dxfs(spec: GearSpec, directory: str | Path) -> list[Path]:
    """One DXF per part, each on its own origin and nothing else in the file.

    ``disc.dxf`` is a drawing: every part of the drive on separate layers, for
    reading.  These are for cutting - a laser, waterjet or CAM job wants one
    closed outline and its holes, not the assembly it belongs to.  Each disc in
    a stack gets its own file, because their hole patterns differ.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    p = prof.profile_from_spec(spec)
    # One schedule for every file in the folder: the disc bore, the carrier's
    # and the plates' are each cut to the bearing that goes in them, and a
    # cutting file with a millimetre of air round a pressed cup is a disc the
    # bearing falls out of.
    choices = bearing_schedule(spec)
    bores = seat_bores(spec, choices)
    bore_r = bores.disc / 2.0
    hole_r = spec.output_hole_diameter / 2.0

    identical = spec.discs_are_identical
    phases = spec.disc_hole_phases[:1] if identical else spec.disc_hole_phases
    for name, hole_phase in zip(disc_names(spec), phases, strict=True):
        doc, msp = _new_doc()
        msp.add_lwpolyline(p.points, close=True, dxfattribs={"layer": "DISC_PROFILE"})
        msp.add_circle((0, 0), bore_r, dxfattribs={"layer": "DISC_BORE"})
        for k in range(spec.output_pin_count):
            a = 2.0 * np.pi * k / spec.output_pin_count + hole_phase
            msp.add_circle((spec.output_bolt_circle_radius * np.cos(a),
                            spec.output_bolt_circle_radius * np.sin(a)),
                           hole_r, dxfattribs={"layer": "OUTPUT_HOLES"})
        _title(msp, spec, f"{name}  i={spec.ratio}:1  holes at "
                          f"{np.degrees(hole_phase):+.3f} deg", p.outer_radius)
        out = directory / f"{name}.dxf"
        doc.saveas(out)
        written.append(out)

    # ---- ring plate: the housing bore and its pin pockets -------------------
    doc, msp = _new_doc()
    msp.add_circle((0, 0), spec.housing_outer_radius, dxfattribs={"layer": "HOUSING"})
    msp.add_circle((0, 0), spec.pin_circle_radius, dxfattribs={"layer": "DISC_BORE"})
    for k in range(spec.pin_count):
        a = 2.0 * np.pi * k / spec.pin_count
        msp.add_circle((spec.pin_circle_radius * np.cos(a),
                        spec.pin_circle_radius * np.sin(a)),
                       spec.pin_radius, dxfattribs={"layer": "RING_PINS"})
    _title(msp, spec, f"ring plate  {spec.pin_count} pins  "
                      f"BC {2 * spec.pin_circle_radius:g}",
           spec.housing_outer_radius)
    out = directory / "ring_plate.dxf"
    doc.saveas(out)
    written.append(out)

    # ---- carrier drilling template -----------------------------------------
    doc, msp = _new_doc()
    plate_r = spec.output_bolt_circle_radius + spec.output_pin_diameter
    # A pin carrying a roller is pressed in by its shank, not by the diameter the
    # disc runs on - drilling the working size would leave nothing to press.
    shank = pin_shank_diameter(bearing_placements(spec, choices),
                               "bearing_output_pins", spec.output_pin_diameter)
    msp.add_circle((0, 0), plate_r, dxfattribs={"layer": "HOUSING"})
    # The bore the solid has, which is the outboard shaft support's seat.  This
    # was a shaft clearance hole, a millimetre over the shaft, while the STEP
    # carrier was bored to the seat - two answers to one hole.
    msp.add_circle((0, 0), bores.hub / 2.0, dxfattribs={"layer": "DISC_BORE"})
    msp.add_circle((0, 0), spec.output_bolt_circle_radius,
                   dxfattribs={"layer": "PITCH", "linetype": "DASHED"})
    for c in _polar(spec.output_bolt_circle_radius, spec.output_pin_count):
        # the pin is a press fit in the carrier, so this is the pin size, not
        # the running hole in the disc
        msp.add_circle(c, shank / 2.0, dxfattribs={"layer": "OUTPUT_HOLES"})
        _cross(msp, c)
    _title(msp, spec, f"output carrier  {spec.output_pin_count} x "
                      f"{shank:g} press fit  "
                      f"BC {2 * spec.output_bolt_circle_radius:g}", plate_r)
    out = directory / "output_carrier.dxf"
    doc.saveas(out)
    written.append(out)

    # ---- the two plates that close the housing ------------------------------
    # New made parts as of the end plates, and the only ones whose whole job is
    # a hole pattern: a bolt circle and a motor face are things you drill, and
    # a STEP file is not something you can drill from.
    # The input plate's bore is not always the shaft support's: with a frame
    # the second main bearing sits in it, and this used to drill that plate to
    # the 22 mm support seat while the STEP plate had the 42 mm bearing seat.
    written.append(_end_plate(spec, directory, "input_end_plate",
                              bores.input_plate, motor_face=True,
                              shoulders=bores.shoulders))
    written.append(_end_plate(spec, directory, "output_end_plate",
                              bores.output_plate, shoulders=bores.shoulders))
    return written


#: Degree of the fitted profile curve.  Cubic is what every modelling package
#: reads without thinking about it, and it is what the fit in
#: :mod:`cycloidgen.core.profile` solves for.
_SPLINE_DEGREE = 3


def _add_profile_spline(msp, control: np.ndarray) -> None:
    """Write the fitted profile as a closed, periodic SPLINE.

    Through ``set_closed`` rather than by hand.  A closed B-spline is written by
    wrapping the first ``degree`` control points onto the end of the list over a
    uniform knot vector - which is what this did first, and it produces the
    identical curve, checked to 7e-15 against evaluating the closed form
    directly.  What the library's own path adds is the pair of flags that says
    so, and they are the difference between a reader knowing the curve is one
    loop and a reader deciding for itself: without them the valid parameter
    range is something each one works out, and the ones that take the whole knot
    vector trace about three per cent of the loop twice.  An outline that
    overlaps itself is one that will not close into a face, which is most of
    what this file is for.
    """
    # Three components, because ``set_closed`` wraps by extending the stored
    # control points and that store is strict about it.
    spline = msp.add_spline(dxfattribs={"layer": "DISC_PROFILE"})
    spline.set_closed([(float(x), float(y), 0.0) for x, y in control],
                      degree=_SPLINE_DEGREE)


def write_sketch_dxfs(spec: GearSpec, directory: str | Path) -> list[Path]:
    """One DXF per distinct disc, with the profile as a single curve.

    The cutting files carry the profile as a polyline of about eighteen hundred
    vertices.  That is the right answer for CAM and the wrong one for a
    modelling package, where the profile is something to build *on*: eighteen
    hundred vertices is not geometry you can offset, dimension or hang a feature
    off, it is a cloud in the shape of a disc.  So the same outline goes out
    again as one closed cubic B-spline - a hundred to a couple of hundred
    control points, fitted to the tolerance the polyline is held to and stamped
    with the deviation it actually reached.

    A folder of its own rather than another layer in the cutting file, and that
    is the point rather than an accident.  These files already carry geometry
    that is not for cutting - pitch circles, the motor's pattern - but none of
    it *looks* like the outline, and two outlines in a file somebody sends to a
    laser is a part cut twice.

    The construction geometry the disc never had is here too.  Every other part
    in the cutting folder gets centre marks and a pitch circle; the disc, which
    is the one part somebody is going to build on, got neither - so there was
    nothing in it to place a feature against but the holes themselves.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    fitted = prof.profile_spline_from_spec(spec)
    outer = float(np.hypot(*prof.profile_from_spec(spec).points.T).max())
    bore_r = seat_bores(spec).disc / 2.0
    hole_r = spec.output_hole_diameter / 2.0
    bolt_r = spec.output_bolt_circle_radius

    phases = (spec.disc_hole_phases[:1] if spec.discs_are_identical
              else spec.disc_hole_phases)
    for name, hole_phase in zip(disc_names(spec), phases, strict=True):
        doc, msp = _new_doc()
        _add_profile_spline(msp, fitted.control_points)
        msp.add_circle((0, 0), bore_r, dxfattribs={"layer": "DISC_BORE"})
        _cross(msp, (0.0, 0.0), size=max(bore_r / 3.0, 2.0))
        # The bolt circle the holes sit on, as construction: a hole pattern you
        # can see the circle of is one you can add to.
        msp.add_circle((0, 0), bolt_r, dxfattribs={"layer": "PITCH"})
        for k in range(spec.output_pin_count):
            a = 2.0 * np.pi * k / spec.output_pin_count + hole_phase
            centre = (bolt_r * np.cos(a), bolt_r * np.sin(a))
            msp.add_circle(centre, hole_r, dxfattribs={"layer": "OUTPUT_HOLES"})
            _cross(msp, centre, size=max(hole_r / 2.0, 1.0))
        _title(msp, spec,
               f"{name} sketch  i={spec.ratio}:1  holes at "
               f"{np.degrees(hole_phase):+.3f} deg  "
               f"profile: one cubic spline, {fitted.samples} control points, "
               f"within {1000 * fitted.deviation:.1f} um of the exact curve  "
               f"- NOT a cutting file, see dxf/",
               outer)
        out = directory / f"{name}.dxf"
        doc.saveas(out)
        written.append(out)
    return written
