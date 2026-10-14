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

    # ...and through a coupling there is nothing to bore: the shaft ends inside
    # the coupling, short of its middle, and the motor stands off on a mount.
    spec.motor_drives_the_shaft = False
    assert spec.motor_bore is None
    c0, c1 = spec.coupling_span
    assert c0 < spec.shaft_span[1] < (c0 + c1) / 2.0
    assert spec.motor_face_z > c1


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
    spec.pin_circle_radius = 20.0
    spec.housing_wall = 2.0
    assert "MOTOR_FACE_CLASH" in _codes(spec)
    assert not analyse(spec).report.ok

    # Through a coupling the pattern is on the mount's flange, which is the
    # motor's own size however small the drive under it.
    spec.motor_drives_the_shaft = False
    assert "MOTOR_FACE_CLASH" not in _codes(spec)


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


# ---------------------------------------------------------- through a coupling


def _coupled(frame: str = "NEMA 17", ring: bool = False):
    from cycloidgen.core.spec import OutputMember

    spec = _spec(frame)
    spec.motor_drives_the_shaft = False
    if ring:
        spec.output_member = OutputMember.RING
    return spec


def test_the_coupling_is_the_smallest_in_the_catalogue_that_takes_both_shafts():
    from cycloidgen.core.motor import COUPLINGS, coupling_for

    assert coupling_for(5.0, 8.0).name == "D19 x L25"
    assert coupling_for(8.0, 5.0) == coupling_for(5.0, 8.0)     # either way round
    assert coupling_for(5.0, 10.0).name == "D25 x L30"
    for c in COUPLINGS:
        assert c.min_bore < c.max_bore < c.outer
    # Too far apart for any of them: a plain sleeve, and it says it is one.
    sleeve = coupling_for(4.0, 20.0)
    assert not sleeve.catalogue and sleeve.outer > 20.0


@pytest.mark.parametrize("ring", [False, True])
def test_through_a_coupling_the_motor_stands_on_a_mount(ring):
    """Off the plate by the coupling and the flange it lands on: the coupling
    clear of both, and the two shafts meeting in its middle a gap apart."""
    from cycloidgen.core.motor import COUPLING_GAP

    spec = _coupled(ring=ring)
    side, face, plate = spec.motor_side, spec.motor_face_z, spec.plate_face_z
    c0, c1 = spec.coupling_span
    near, far = (c0, c1) if side > 0 else (c1, c0)
    assert side * (near - plate) == pytest.approx(COUPLING_GAP)
    bore, _outer, height = spec.motor_mount
    assert height + spec.motor_mount_flange == pytest.approx(side * (face - plate))
    assert side * (plate + side * height - far) >= COUPLING_GAP - 1e-9
    assert bore > spec.coupling.outer / 2.0                 # room to turn

    end = spec.shaft_span[1] if side > 0 else spec.shaft_span[0]
    tip = face - side * spec.motor_face.shaft_length
    assert side * (tip - end) == pytest.approx(COUPLING_GAP)
    assert spec.motor_engagement == pytest.approx(
        (spec.coupling.length - COUPLING_GAP) / 2.0)
    # The input shaft's flat is under the coupling's screw, on its own half.
    z0, z1, _depth = spec.input_flat
    assert (z0, z1) == ((c0, end) if side > 0 else (end, c1))

    # Nothing of driving it directly is left.
    assert spec.motor_bore is None and spec.motor_fixing.recess == 0.0
    assert not {"MOTOR_SHAFT_SHORT", "MOTOR_SCREW_SEAT",
                "COUPLING_DOES_NOT_FIT"} & _codes(spec)


@pytest.mark.parametrize("name", [n for n in MOTOR_FRAMES if n != NO_MOTOR])
def test_the_mount_s_flange_is_as_thick_as_the_motor_s_screws_want(name):
    """Up through a flange into a threaded face, a preferred length fills the
    motor's threads or falls short of them by as much as two millimetres -
    most of an M3's - so the flange is the thickness that makes one fill them."""
    from cycloidgen.core.motor import MOTOR_MOUNT_FLANGE

    spec = _coupled(name)
    fix = spec.motor_fixing
    assert spec.motor_mount_flange >= MOTOR_MOUNT_FLANGE
    assert fix.fits and fix.recess == 0.0
    if fix.from_drive:
        assert fix.engaged == pytest.approx(spec.motor_face.fixing_depth - 0.3)
    assert "MOTOR_SCREW_SEAT" not in _codes(spec)


def test_a_coupling_that_does_not_fit_is_a_warning():
    """A NEMA 11's screws leave less room inside them than the tube round the
    coupling a 10 mm shaft takes; and some shafts no catalogue coupling joins."""
    findings = {f.code: f for f in analyse(_coupled("NEMA 11")).report.findings}
    assert findings["COUPLING_DOES_NOT_FIT"].severity.value == "warning"

    spec = _coupled()
    spec.input_shaft_diameter = 14.0
    assert not spec.coupling.catalogue
    assert "COUPLING_DOES_NOT_FIT" in _codes(spec)


def test_a_motor_shaft_the_flange_takes_most_of_is_caught():
    """A NEMA 8's shaft is ten millimetres; the flange it passes is eight."""
    spec = _coupled("NEMA 8")
    assert spec.motor_engagement < 1.5 * spec.motor_face.shaft_diameter
    assert "MOTOR_SHAFT_SHORT" in _codes(spec)


def test_the_coupling_and_its_mount_are_weighed_and_on_the_bill():
    """The mount is part of the input plate, so the two plates stop being one
    part twice; the coupling is bought, and turns with the input."""
    from cycloidgen.export.bom import bom_items
    from cycloidgen.viz.mesh import mesh_for_spec

    spec = _coupled()
    a = analyse(spec)
    m = a.mass
    assert m.coupling_mass_g > 0.0 and m.mount_mass_g > 0.0
    plain = analyse(_spec()).mass
    assert plain.coupling_mass_g == plain.mount_mass_g == 0.0

    items = bom_items(a)
    parts = {i.part: i for i in items}
    assert "Housing end plate" not in parts
    inlet = parts["Input end plate, with motor mount"]
    outlet = parts["Output end plate"]
    assert inlet.mass_each_g + outlet.mass_each_g == pytest.approx(m.plates_mass_g)
    assert inlet.mass_each_g - outlet.mass_each_g == pytest.approx(m.mount_mass_g)
    coupling = parts["Shaft coupling"]
    assert coupling.source == "buy" and spec.coupling.name in coupling.size
    assert coupling.mass_each_g == pytest.approx(m.coupling_mass_g)
    covered = [n for item in items for n in item.assembly]
    for part in mesh_for_spec(spec).parts:
        assert covered.count(part.name) == 1, part.name


def test_a_coupling_s_half_is_all_a_motor_shaft_is_asked_to_fill():
    """One and a half times a NEMA 34's 14 mm shaft is 21 mm, and the coupling
    that takes it has 19.5 mm a side: its maker sized that for the bore."""
    from cycloidgen.core.motor import COUPLING_GAP

    spec = _coupled("NEMA 34")
    assert spec.motor_engagement < 1.5 * spec.motor_face.shaft_diameter
    assert spec.motor_engagement == pytest.approx(
        (spec.coupling.length - COUPLING_GAP) / 2.0)
    assert "MOTOR_SHAFT_SHORT" not in _codes(spec)
