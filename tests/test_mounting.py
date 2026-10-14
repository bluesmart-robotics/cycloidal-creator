"""How the gearbox bolts to the world, at both ends.

The app talked about the motor in four places and had no motor interface at
all: no bolt pattern, no register, no shaft.  ``shaft_bearings_fitted = False``
said in so many words that the drive "hangs on the motor face", and there was
no face.  Nothing on this gearbox could be attached to anything.
"""
from __future__ import annotations

import numpy as np
import pytest

from cycloidgen.analysis import analyse
from cycloidgen.core.spec import MOTOR_FRAMES, NO_MOTOR, preset


def _spec(frame: str = "NEMA 17", shaft: float | None = None):
    spec = preset(21)
    spec.motor_frame = frame
    if shaft is not None:
        spec.input_shaft_diameter = shaft
    return spec


def _codes(spec):
    return {f.code for f in analyse(spec).report.findings}


# ------------------------------------------------------------------ the frames


def test_a_nema_pattern_is_a_square_and_not_a_bolt_circle():
    """Four holes on a circle of the same size land where the motor has nothing.

    It is the kind of wrong that draws perfectly and does not fit, so the
    distinction is in the data rather than in whoever writes the geometry.
    """
    frame = MOTOR_FRAMES["NEMA 17"]
    assert frame.square
    assert frame.bolt_circle_diameter == pytest.approx(31.0 * np.sqrt(2.0))
    assert frame.bolt_circle_diameter > frame.bolt_span


def test_every_frame_puts_its_bolts_outside_its_own_register():
    """A hole inside the spigot is a hole in the motor, not in the plate."""
    for name, frame in MOTOR_FRAMES.items():
        if name == NO_MOTOR:
            continue
        reach = frame.bolt_circle_diameter / 2.0 - frame.bolt_diameter / 2.0
        assert reach > frame.pilot_diameter / 2.0, name


def test_no_motor_is_the_default_because_none_of_them_fits_the_presets():
    """Presuming a motor that cannot drive the preset it ships with would put a
    finding on every design out of the box."""
    assert preset(21).motor_frame == NO_MOTOR
    assert not [c for c in _codes(preset(21)) if c.startswith("MOTOR")]


# ------------------------------------------------------------------- the plate


def test_the_motor_face_is_cut_into_the_input_plate_only():
    """The output end plate carries the bearing the drive turns on; a motor
    pattern in it would be four holes into the machine's own mounting face."""
    from cycloidgen.export import solid

    plain = solid.parts(preset(21))
    with_motor = solid.parts(_spec())
    assert (with_motor["input_end_plate"].val().Volume()
            < plain["input_end_plate"].val().Volume())
    assert (with_motor["output_end_plate"].val().Volume()
            == pytest.approx(plain["output_end_plate"].val().Volume()))


def test_the_bolt_holes_are_drawn_where_the_motor_has_bolts():
    """The picture and the part have to agree about a pattern you drill."""
    from cycloidgen.viz.mesh import build_mesh

    spec = _spec()
    mesh = build_mesh(spec)
    plate = next(p for p in mesh.parts if p.name == "input_end_plate")
    xy = mesh.vertices[plate.vertices][:, :2]

    half = spec.motor_face.bolt_span / 2.0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            near = np.hypot(xy[:, 0] - sx * half, xy[:, 1] - sy * half)
            assert near.min() == pytest.approx(spec.motor_face.bolt_diameter / 2.0,
                                               rel=0.05), (sx, sy)


def test_the_tie_bolts_miss_the_pins_they_are_holding_the_plate_over():
    spec = preset(21)
    assert spec.housing_bolt_count > 0
    inner = spec.pin_circle_radius + spec.pin_radius
    assert inner < spec.housing_bolt_radius - spec.housing_bolt_diameter / 2.0
    assert (spec.housing_bolt_radius + spec.housing_bolt_diameter / 2.0
            < spec.housing_outer_radius)


# ------------------------------------------------------------------- the checks


def test_driven_directly_the_input_shaft_is_bored_for_the_motor_s():
    """The drive's shaft slides onto the motor's - a D on its D - and stops
    short of the motor, which sits flush on the plate.  It used to run on out
    of the plate for a coupling to grip, into the space the motor is in."""
    spec = _spec()
    frame = spec.motor_face
    _lo, hi = spec.shaft_span
    assert hi < spec.motor_face_z - frame.pilot_depth            # clear of it
    _z0, z1, d, flat = spec.motor_bore
    assert z1 == hi and d > frame.shaft_diameter and flat == frame.shaft_flat
    assert spec.motor_engagement >= 1.5 * frame.shaft_diameter
    assert spec.input_flat is None                  # nothing left to grip
    codes = _codes(spec)
    assert not {"MOTOR_SHAFT_MISMATCH", "MOTOR_SHAFT_SHORT"} & codes

    # ...and through a coupling there is nothing to bore.
    spec.motor_drives_the_shaft = False
    assert spec.motor_bore is None
    assert spec.shaft_span[1] > spec.motor_face_z


def test_an_input_shaft_too_thin_to_bore_is_caught_and_does_not_block():
    """The old advice was to make them the same shaft; bored for the motor's,
    that leaves no wall at all.  A warning: every file is still drawn."""
    findings = {f.code: f for f in analyse(_spec(shaft=5.0)).report.findings}
    assert findings["MOTOR_SHAFT_MISMATCH"].severity.value == "warning"


def test_a_motor_shaft_that_barely_reaches_is_caught(monkeypatch):
    """The input shaft runs right up to the spigot, so what decides how much
    of the motor's is inside it is the motor's shaft - a stub on a short one."""
    from cycloidgen.core import motor

    stub = motor.MOTOR_FRAMES["NEMA 17"].model_copy(update={"shaft_length": 8.0})
    monkeypatch.setitem(motor.MOTOR_FRAMES, "NEMA 17", stub)
    spec = _spec()
    assert spec.motor_engagement < 1.5 * stub.shaft_diameter
    assert "MOTOR_SHAFT_SHORT" in _codes(spec)


@pytest.mark.parametrize("frame", ["NEMA 17", "NEMA 23"])
def test_the_motor_s_screws_are_let_into_the_face_the_discs_run_on(frame):
    """The inside of the plate runs against the discs with nothing to spare, so
    a head or a nut there has to be below its surface: a NEMA 17's threaded
    face takes screws from the drive's side, heads in counterbores; a NEMA
    23's flange takes them from its own side, into nuts in pockets."""
    from cycloidgen.viz.mesh import motor_screw_layout

    spec = _spec(frame)
    fix = spec.motor_fixing
    assert fix.from_drive == (frame == "NEMA 17")
    assert fix.fits and "MOTOR_SCREW_SEAT" not in _codes(spec)
    inside = spec.motor_face_z - spec.motor_side * fix.plate
    for _what, z0, _z1 in motor_screw_layout(spec):
        assert z0 >= inside - 1e-9                  # nothing stands into the discs
    assert spec.motor_face_z + fix.engaged > spec.motor_face_z


def test_a_plate_too_thin_for_its_screws_is_caught():
    spec = _spec("NEMA 23")
    spec.end_plate_thickness = 5.0
    assert "MOTOR_SCREW_SEAT" in _codes(spec)


def test_a_pattern_that_runs_off_the_plate_is_an_error():
    """A small drive can be narrower across than the motor bolted to it, and
    then the holes are in fresh air - which does make the exported plate wrong."""
    spec = preset(10)
    spec.motor_frame = "NEMA 34"
    spec.motor_drives_the_shaft = False
    spec.pin_circle_radius = 20.0
    spec.housing_wall = 2.0
    assert "MOTOR_FACE_CLASH" in _codes(spec)
    assert not analyse(spec).report.ok


def test_hanging_the_drive_on_the_motor_is_measured_against_the_motor():
    """'Check its rating' was the best this could say before there was a frame
    to check against."""
    spec = _spec()
    spec.shaft_bearings_fitted = False
    spec.output_torque_Nm = 15.0
    assert "MOTOR_RADIAL_LOAD" in _codes(spec)

    spec.shaft_bearings_fitted = True
    assert "MOTOR_RADIAL_LOAD" not in _codes(spec)


def test_the_bolts_reach_the_bill_of_materials():
    from cycloidgen.export.bom import bom_items

    parts = {i.part: i for i in bom_items(analyse(_spec()))}
    assert parts["Tie bolt"].quantity == preset(21).housing_bolt_count
    assert parts["Motor bolt"].quantity == 4
    assert "NEMA 17" in parts["Motor bolt"].note

    assert "Motor bolt" not in {i.part for i in bom_items(analyse(preset(21)))}


# -------------------------------------------------------------- the output end


def test_the_output_boss_stands_out_far_enough_to_grip():
    """It came out flush with the end plate, which is nothing for a coupling to
    hold - the output of this topology is a shaft, not a bolt face."""
    spec = preset(21)
    assert spec.output_boss_protrusion > 0

    from cycloidgen.viz.mesh import build_mesh
    mesh = build_mesh(spec)
    carrier = next(p for p in mesh.parts if p.name == "output_flange")
    plate = next(p for p in mesh.parts if p.name == "output_end_plate")
    assert (mesh.vertices[carrier.vertices][:, 2].min()
            < mesh.vertices[plate.vertices][:, 2].min() - 1e-9)


# ------------------------------------------------------------ the motor itself


@pytest.mark.parametrize("name", [n for n in MOTOR_FRAMES if n != NO_MOTOR])
def test_a_frame_says_enough_about_its_motor_to_fit_one(name):
    """Body, shaft and fixing - what decides whether the motor can be put where
    the drive says, and which side its screws go in from."""
    from cycloidgen.core.fasteners import size_named

    frame = MOTOR_FRAMES[name]
    assert frame.bolt_span < frame.body_width            # the holes are in it
    assert frame.pilot_diameter < frame.body_width
    assert frame.shaft_length > frame.pilot_depth
    assert 0.0 <= frame.shaft_flat < frame.shaft_diameter / 4.0
    assert frame.fixing in ("tapped", "through")
    screw = size_named(frame.screw)
    assert screw is not None, frame.screw
    assert screw.d < frame.bolt_diameter                 # and goes through them
    assert frame.fixing_depth > 0.0


@pytest.mark.parametrize("ring", [False, True])
def test_the_motor_is_drawn_standing_off_its_face(ring):
    """Up off a carrier drive's input plate, down off a ring drive's base, its
    spigot in the register and its shaft pointing into the drive."""
    from cycloidgen.core.spec import OutputMember
    from cycloidgen.viz.mesh import mesh_for_spec

    spec = _spec()
    if ring:
        spec.output_member = OutputMember.RING
    frame, zf, side = spec.motor_face, spec.motor_face_z, spec.motor_side
    assert side == (-1.0 if ring else 1.0)
    parts = {p.name: p for p in mesh_for_spec(spec).parts}
    body = mesh_for_spec(spec).vertices[parts["motor"].vertices]
    shaft = mesh_for_spec(spec).vertices[parts["motor_shaft"].vertices]
    far = (body[:, 2] * side).max() * side
    assert far == pytest.approx(zf + side * frame.body_length)
    spigot = (body[:, 2] * -side).max() * -side
    assert spigot == pytest.approx(zf - side * frame.pilot_depth)
    assert (shaft[:, 2] * -side).max() * -side == pytest.approx(
        zf - side * frame.shaft_length)
    assert parts["motor_shaft"].spin == -1.0          # it is what turns the input


def test_the_motor_is_on_the_bill_and_weighs_nothing_in_the_drive():
    from cycloidgen.export.bom import bom_items
    from cycloidgen.viz.mesh import mesh_for_spec

    spec = _spec()
    a = analyse(spec)
    items = bom_items(a)
    motor = next(i for i in items if i.part == "Motor")
    assert motor.mass_each_g == 0.0 and motor.source == "buy"
    covered = [n for item in items for n in item.assembly]
    for part in mesh_for_spec(spec).parts:
        assert covered.count(part.name) == 1, part.name
