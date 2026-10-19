"""Does the assembly actually fit together?

The 2D meshing tests prove the disc rolls on the ring.  These prove the *rest*
of the machine agrees with it - which is a separate claim, and one that was
wrong the first time: every disc past the first had its output holes at the
wrong angle, so the carrier pins could not enter them.

The headline check is the one you can see by eye in the drawing: an output pin
must stay inside its hole at every crank angle.
"""
from __future__ import annotations

import numpy as np
import pytest

from cycloidgen.analysis import analyse
from cycloidgen.core.spec import CARRIER_DROP, GearSpec, preset
from cycloidgen.core.validate import Severity

RATIOS = [10, 15, 21, 29, 59]
STACKS = [1, 2, 3]


def _pin_and_hole_centres(spec: GearSpec, phi: float, disc: int):
    """Carrier pin centres and disc hole centres in the housing frame."""
    n = spec.output_pin_count
    base = 2.0 * np.pi * np.arange(n) / n
    # the carrier rotates with the discs
    pins = spec.output_bolt_circle_radius * np.column_stack(
        [np.cos(base + phi / spec.lobes), np.sin(base + phi / spec.lobes)])

    phase = spec.disc_phases[disc]
    hole_phase = spec.disc_hole_phases[disc]
    centre = np.array([spec.eccentricity * np.cos(phi + phase),
                       -spec.eccentricity * np.sin(phi + phase)])
    d = (phi + phase) / spec.lobes
    ang = base + d + hole_phase
    holes = centre + spec.output_bolt_circle_radius * np.column_stack(
        [np.cos(ang), np.sin(ang)])
    return pins, holes


@pytest.mark.parametrize("ratio", RATIOS)
@pytest.mark.parametrize("discs", STACKS)
def test_output_pin_never_leaves_its_hole(ratio, discs):
    """The visible one: green circle stays inside blue circle, always.

    The pin centre orbits the hole centre at radius exactly E, and the hole is
    the pin plus 2E across, so the pin is permanently inside - by zero margin in
    theory and by the hole clearance in practice.
    """
    spec = preset(ratio)
    spec.disc_count = discs
    slack = spec.output_hole_diameter / 2 - spec.output_pin_diameter / 2
    for disc in range(discs):
        for phi in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            pins, holes = _pin_and_hole_centres(spec, float(phi), disc)
            offset = np.hypot(*(pins - holes).T)
            assert offset.max() <= slack + 1e-9, (
                f"disc {disc + 1} at {np.degrees(phi):.0f} deg: pin centre "
                f"{offset.max():.4f} mm from the hole centre, only {slack:.4f} mm "
                f"of room - the pin is outside its hole")


@pytest.mark.parametrize("ratio", RATIOS)
@pytest.mark.parametrize("discs", [2, 3])
def test_pin_orbit_radius_is_exactly_the_eccentricity(ratio, discs):
    """Ignoring clearance, the offset traces a circle of radius E, not less."""
    spec = preset(ratio)
    spec.disc_count = discs
    spec.hole_clearance = 0.0
    for disc in range(discs):
        offsets = []
        for phi in np.linspace(0, 2 * np.pi, 180, endpoint=False):
            pins, holes = _pin_and_hole_centres(spec, float(phi), disc)
            offsets.append(np.hypot(*(pins - holes).T))
        offsets = np.array(offsets)
        assert np.allclose(offsets, spec.eccentricity, atol=1e-9)


@pytest.mark.parametrize("ratio", RATIOS)
def test_disc_clears_the_housing_bore(ratio):
    """Disc reach is (R - Rr + E) from its own centre, which is E off axis."""
    spec = preset(ratio)
    reach = spec.pin_circle_radius - spec.effective_Rr + 2 * spec.eccentricity
    assert reach < spec.pin_circle_radius, (
        f"disc reaches {reach:.3f} mm, housing bore is "
        f"{spec.pin_circle_radius:.3f} mm - needs 2E <= Rr")


def test_discs_are_identical_only_when_pins_allow_it():
    spec = preset(15)
    spec.disc_count = 2
    assert not spec.discs_are_identical
    spec.output_pin_count = 2 * spec.lobes if 2 * spec.lobes <= 24 else 24
    if spec.output_pin_count == 2 * spec.lobes:
        assert spec.discs_are_identical

    single = preset(15)
    single.disc_count = 1
    assert single.discs_are_identical


def test_stl_export_emits_one_file_per_distinct_disc(tmp_path):
    from cycloidgen.export import solid
    spec = preset(15)
    spec.disc_count = 2
    names = {p.stem for p in solid.write_stls(spec, tmp_path)}
    assert {"disc_1", "disc_2"} <= names
    assert "disc" not in names


def test_stl_export_emits_one_disc_when_they_are_identical(tmp_path):
    from cycloidgen.export import solid
    spec = preset(15)
    spec.disc_count = 1
    names = {p.stem for p in solid.write_stls(spec, tmp_path)}
    assert "disc" in names


def test_assembly_reports_that_the_discs_differ():
    from cycloidgen.core.validate import validate
    spec = preset(15)
    spec.disc_count = 2
    assert "DISCS_DIFFER" in {f.code for f in validate(spec).findings}


# ------------------------------------------------------- what holds it together


@pytest.mark.parametrize("ratio", [15, 21, 33])
def test_the_tie_bolts_pass_through_the_thing_they_clamp(ratio):
    """The bill of materials orders six of them and both end plates are
    drilled for them.  The barrel they clamp was not, so the exported gearbox
    was a stack of parts whose bolts landed on solid wall - which nothing in
    the app noticed, because nothing was looking at the barrel.
    """
    import math

    from cycloidgen.export.solid import ring_housing

    # Without the rim's break, which is held clear of the bolt holes and so is
    # not the same break on a barrel that has none.
    spec = preset(ratio).model_copy(update={"edge_chamfer": 0.0})
    assert spec.housing_bolt_count, "this preset has no tie bolts to check"

    drilled = ring_housing(spec).val().Volume()
    plain = ring_housing(
        spec.model_copy(update={"housing_bolt_count": 0})).val().Volume()

    hole = (math.pi * (spec.housing_bolt_diameter / 2.0) ** 2
            * spec.barrel_height * spec.housing_bolt_count)
    assert plain - drilled == pytest.approx(hole, rel=0.02)


@pytest.mark.parametrize("ratio", [10, 15, 21, 29, 39, 59])
def test_no_preset_puts_its_bolts_where_there_is_no_wall(ratio):
    spec = preset(ratio)
    codes = [f.code for f in analyse(spec).report.findings]
    assert "HOUSING_BOLT_CLASH" not in codes


def test_a_bolt_that_breaks_into_the_pockets_is_an_error():
    """Inward: the bolt reaches the bore the discs run against."""
    spec = preset(21).model_copy(update={"housing_bolt_diameter": 9.0})
    finding = next(f for f in analyse(spec).report.findings
                   if f.code == "HOUSING_BOLT_CLASH")
    assert finding.severity is Severity.ERROR
    assert "pockets" in finding.message


def test_a_bolt_too_big_for_its_wall_is_an_error():
    """It breaks into the pockets and out through the rim at the same moment,
    because the circle is derived as the middle of the wall - which is why this
    is one question rather than an inner test and an outer one."""
    spec = preset(21)
    fat = spec.model_copy(update={"housing_bolt_diameter": spec.housing_wall})
    finding = next(f for f in analyse(fat).report.findings
                   if f.code == "HOUSING_BOLT_CLASH")
    assert finding.severity is Severity.ERROR


@pytest.mark.parametrize("ratio", [15, 21, 33])
def test_the_bolt_length_is_the_distance_it_has_to_span(ratio):
    """It used to be quoted off the disc stack.  When the barrel was lengthened
    to reach the plates it bolts to, the bill of materials did not follow - so
    the drive was ordering bolts seven millimetres short of the thing they pass
    through, and nothing compared the two.
    """
    from cycloidgen.export.bom import bom_items
    from cycloidgen.viz.mesh import mesh_for_spec

    spec = preset(ratio)
    mesh = mesh_for_spec(spec)

    faces = []
    for part in mesh.parts:
        if part.group == "end_plates":
            z = mesh.vertices[part.vertices][:, 2]
            faces += [float(z.min()), float(z.max())]
    assert max(faces) - min(faces) == pytest.approx(spec.tie_bolt_length)

    # The screw is ordered at a preferred length past that, and the grip it has
    # to clamp is stated beside it.
    stated = next(i for i in bom_items(analyse(spec)) if i.part == "Tie bolt")
    assert f"{spec.tie_bolt_length:.0f} mm of grip" in stated.note
    assert f" x {spec.tie_screw_length:g} " in stated.size
    assert spec.tie_screw_length > spec.tie_bolt_length


@pytest.mark.parametrize("ratio", [15, 21, 33])
def test_the_bolts_are_drawn_where_the_holes_are(ratio):
    """Bought, so they are in the picture and the STEP assembly and not in the
    per-part export - the same treatment the bearings get."""
    from cycloidgen.export.solid import build_assembly, parts, tie_bolts

    spec = preset(ratio)
    assert "tie_bolts" not in parts(spec)
    assert "tie_bolts" in {c.name for c in build_assembly(spec).children}

    # Socket head cap screws, heads on the output plate, at the preferred
    # length that clamps the stack with a nut and two threads past it.
    size = spec.tie_bolt_size
    assert size is not None and size.name == "M4"      # a 4.5 mm hole
    bounds = tie_bolts(spec).val().BoundingBox()
    assert bounds.zmin == pytest.approx(spec.tie_bolt_bottom - size.head_h)
    assert bounds.zmax == pytest.approx(spec.tie_bolt_bottom + spec.tie_screw_length)
    assert (spec.tie_screw_length
            >= spec.tie_bolt_length + size.nut_m + 2.0 * size.pitch)
    # the shank runs in the clearance hole, not filling it
    assert size.d < spec.housing_bolt_diameter


@pytest.mark.parametrize("ratio", [15, 21, 33])
def test_the_mass_model_knows_about_every_hole_in_the_barrel(ratio):
    """It already subtracted the ring-pin pockets.  The tie-bolt holes went in
    later, and a mass model that knows about one set of holes and not the other
    is describing a different part from the one the exporter writes.

    Barrel and plates are weighed apart, and that is the point of doing it here
    rather than on their sum: while they were one number the two could be wrong
    in opposite directions and still add up.  They were - the barrel's line on
    the bill of materials carried the plates' mass and the plates' line carried
    nothing.
    """
    from cycloidgen.export.solid import housing_end_plate, ring_housing

    spec = preset(ratio)
    ms = analyse(spec).mass
    rho = spec.housing_mat.density_g_cm3

    barrel = ring_housing(spec).val().Volume()
    plates = (housing_end_plate(spec, spec.hub_bore, motor_face=True).val().Volume()
              + housing_end_plate(
                  spec, spec.output_bearing_seat_diameter).val().Volume())

    assert ms.housing_mass_g / rho * 1000.0 == pytest.approx(barrel, rel=0.02)
    assert ms.plates_mass_g / rho * 1000.0 == pytest.approx(plates, rel=0.02)


@pytest.mark.parametrize("ratio", [15, 21, 33])
@pytest.mark.parametrize("discs", STACKS)
def test_a_pin_is_as_long_as_the_space_it_has_to_fill(ratio, discs):
    """Both pin lengths were quoted off the disc stack, and neither of them is
    the disc stack.

    The barrel was lengthened to reach the end plates it bolts to, and three
    things that measure themselves against it were never told.  The tie bolt was
    one and got fixed; these are the other two.

    * A ring pin sits in a pocket broached the barrel's whole length.  Cut to the
      disc stack it has seven millimetres of empty groove beneath it and nothing
      holding it up, so it slides down out of the mesh - and leaves an open
      pocket at the top for a lobe to drop into.
    * An output pin leaves the carrier face a *drop* below the first disc, so a
      stack-high pin arrives a drop short of the last one.  It was: seven of the
      top disc's eight millimetres, while the bearing stress this app reports
      for that hole is computed over all eight.
    """
    from cycloidgen.export.solid import output_flange, ring_housing, ring_pins

    spec = preset(ratio)
    spec.disc_count = discs

    barrel = ring_housing(spec).val().BoundingBox()
    pins = ring_pins(spec).val().BoundingBox()
    assert pins.zlen == pytest.approx(spec.ring_pin_length)
    # Trapped: it starts where the barrel starts and ends where it ends, which
    # is where the two end plates are.
    assert pins.zmin == pytest.approx(barrel.zmin)
    assert pins.zmax == pytest.approx(barrel.zmax)

    # The flange is built in its own frame with the plate below zero and the
    # pins above it, so the top of the box *is* the pin.
    flange = output_flange(spec).val().BoundingBox()
    assert flange.zmax == pytest.approx(spec.output_pin_length)
    # And in the assembly that has to land on the far face of the last disc.
    assert spec.output_pin_length - CARRIER_DROP == pytest.approx(spec.stack_height)


@pytest.mark.parametrize("ratio", [15, 21, 33])
def test_the_bill_of_materials_orders_the_part_that_gets_exported(ratio):
    """Every length on the bill measured against the solid it names.

    This is the general form of the tie-bolt check above, and it exists because
    that bug was not one bug: the barrel, both pins and the bolt all quoted the
    disc stack, and each was found separately by reading the code rather than by
    anything failing.  A dowel ordered short is not a fit you adjust at assembly,
    it is the wrong part - so the number on the bill and the number in the STEP
    file are held together here instead.
    """
    import re

    from cycloidgen.export.bom import bom_items
    from cycloidgen.export.solid import (
        housing_end_plate,
        output_flange,
        ring_housing,
        ring_pins,
    )

    spec = preset(ratio)
    drawn = {
        "Ring housing": ring_housing(spec),
        "Housing end plate": housing_end_plate(spec, spec.hub_bore,
                                               motor_face=True),
        "Ring pin (dowel)": ring_pins(spec),
        # The output pins are cut into the carrier, so the part that carries
        # them is where their length is measured.
        "Output pin (dowel)": output_flange(spec),
    }
    stated = {i.part: i.size for i in bom_items(analyse(spec))}

    for part, solid in drawn.items():
        match = re.fullmatch(r"([\d.]+)(?: mm)? dia x ([\d.]+) mm", stated[part])
        assert match, f"{part}: cannot read a length out of {stated[part]!r}"
        box = solid.val().BoundingBox()
        length = box.zmax if part == "Output pin (dowel)" else box.zlen
        assert float(match.group(2)) == pytest.approx(length, rel=1e-3), part


@pytest.mark.parametrize("ratio", [15, 21, 33])
def test_nothing_you_have_to_make_weighs_nothing(ratio):
    """A made part quoting zero grams is a part the mass model has not been told
    about, and the bill of materials is the one place that shows.

    The end plates did exactly this: their mass was real and computed, but it was
    being added to the barrel's line, so the plates - a third of the gearbox -
    went out as a blank cell next to the words "make".
    """
    from cycloidgen.export.bom import bom_items

    made = [i for i in bom_items(analyse(preset(ratio))) if i.source == "make"]
    assert made
    assert not [i.part for i in made if i.mass_each_g <= 0.0]


# ------------------------------------------------------------ broken edges


def _radius(face) -> float:
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    return BRepAdaptor_Surface(face.wrapped).Cylinder().Radius()


def _made_parts(spec: GearSpec) -> dict:
    from cycloidgen.export import solid
    parts = solid.parts(spec)
    parts.pop("ring_pins", None)          # dowels: bought, and their own maker's edges
    # Spacers carry no break: a millimetre-thick washer would give most of
    # itself to one.
    for name in solid.spacer_parts(spec):
        parts.pop(name)
    return parts


@pytest.mark.parametrize("output", ["output pin carrier", "ring housing"])
def test_a_break_only_ever_takes_metal_away(output):
    """A chamfer on an inside corner is a wedge OCCT *adds*.

    That is a fillet nobody drew, standing in the corner a bearing's inner ring
    is meant to sit against - where the boss meets the carrier plate, or where
    the shaft runs into a cam.  So every broken part has to fit wholly inside
    the same part unbroken, which is a sharper statement than its volume going
    down: a wedge added in one place hides behind a chamfer taken off another.
    """
    spec = preset(21)
    spec.output_member = output
    broken = _made_parts(spec)
    sharp = _made_parts(spec.model_copy(update={"edge_chamfer": 0.0}))
    for name, wp in broken.items():
        added = wp.val().cut(sharp[name].val()).Volume()
        taken = sharp[name].val().Volume() - wp.val().Volume()
        assert added < 1e-6, f"{name} gained {added:.4f} mm^3"
        assert taken > 0.0, f"{name} has no break on it at all"


def test_no_break_touches_a_running_surface():
    """The flank, the output holes and the pins are contact length.

    The analysis counts the whole disc thickness as the length of every line
    contact in the drive, so a chamfer on any of them would be a drawing that
    disagrees with its own numbers.  Each of those faces is checked to still
    run the full height it ran before the edges were broken.
    """
    from cycloidgen.export import solid

    spec = preset(21)
    disc = solid.disc_solid(spec).val()
    t = spec.disc_thickness
    flank = [f for f in disc.Faces() if f.geomType() == "EXTRUSION"]
    holes = [f for f in disc.Faces() if f.geomType() == "CYLINDER"
             and abs(_radius(f) - spec.output_hole_diameter / 2.0) < 1e-6]
    assert len(flank) == 2 * spec.lobes and len(holes) == spec.output_pin_count
    for face in flank + holes:
        assert face.BoundingBox().zlen == pytest.approx(t, abs=1e-6)

    flange = solid.output_flange(spec).val()
    pins = [f for f in flange.Faces() if f.geomType() == "CYLINDER"
            and abs(_radius(f) - spec.output_pin_diameter / 2.0) < 1e-6]
    assert len(pins) == spec.output_pin_count
    for face in pins:
        assert face.BoundingBox().zlen == pytest.approx(spec.output_pin_length,
                                                        abs=1e-6)


def test_no_edge_chamfer_is_the_sharp_part():
    """Zero means none: no cone anywhere, and the volume the part had before."""
    spec = preset(15).model_copy(update={"edge_chamfer": 0.0})
    for name, wp in _made_parts(spec).items():
        assert not any(f.geomType() == "CONE" for f in wp.val().Faces()), name


@pytest.mark.parametrize("ratio", RATIOS)
def test_a_large_break_is_trimmed_rather_than_refused(ratio):
    """The field goes to 3 mm and the metal beside a tie-bolt hole is under one.

    A break is held to a quarter of its plate and half the land it stands on,
    so the largest value the panel allows still builds every part, valid.
    """
    spec = preset(ratio).model_copy(update={"edge_chamfer": 3.0})
    for name, wp in _made_parts(spec).items():
        assert wp.val().isValid(), name


# ------------------------------------------------------- what locates the shaft


@pytest.mark.parametrize("output", ["output pin carrier", "ring housing"])
def test_the_shaft_is_located_from_cam_to_shoulder(output):
    """Thrust either way goes cam, sleeve, inner ring, balls, outer ring, lip.

    So each sleeve has to run from a cam's end face exactly to a support's inner
    ring, and bear on that ring and nothing else: its outside between the
    bearing's bore and its pitch circle, which is where the shoulder's hole is.
    Without them there was a millimetre and a half of air on one side of the
    shaft and eight and a half on the other.
    """
    from cycloidgen.analysis.bearings import placements_for_spec, seat_bores

    spec = preset(21)
    spec.output_member = output
    bores = seat_bores(spec)
    sleeves = {s.name: s for s in bores.spacers if s.name.startswith("shaft_spacer")}
    placed = {p.name: p for p in placements_for_spec(spec)}

    top = sleeves["shaft_spacer_input"]
    support = placed["bearing_shaft_input"]
    assert top.z0 == pytest.approx(spec.stack_height)          # the last cam's face
    assert top.z1 == pytest.approx(support.rings[0].z0)

    bottom = sleeves["shaft_spacer_output"]
    support = placed["bearing_shaft_output"]
    assert bottom.z0 == pytest.approx(support.rings[0].z1)
    assert bottom.z1 == pytest.approx(0.0)                      # the first cam's face

    for s in (top, bottom):
        assert spec.input_shaft_diameter < s.bore
        assert support.bore < s.outer < (support.bore + support.outer) / 2.0


def test_a_washer_holds_every_gap_and_bears_only_on_disc():
    """Whichever way the two discs either side have orbited, the washer reaches
    past the disc bore and stops short of the output holes."""
    from cycloidgen.analysis.bearings import seat_bores

    spec = preset(15)
    spec.disc_count = 3
    bores = seat_bores(spec)
    washers = [s for s in bores.spacers if s.name.startswith("washer")]
    assert len(washers) == spec.disc_count - 1
    e = spec.eccentricity
    pitch = spec.disc_thickness + spec.disc_gap
    for i, w in enumerate(washers):
        assert w.z0 == pytest.approx(i * pitch + spec.disc_thickness)
        assert w.length == pytest.approx(spec.disc_gap)
        assert bores.disc / 2.0 + e < w.outer / 2.0
        assert w.outer / 2.0 < (spec.output_bolt_circle_radius
                                - spec.output_hole_diameter / 2.0 - e)

    single = preset(15)
    single.disc_count = 1
    assert not [s for s in seat_bores(single).spacers if s.name.startswith("washer")]


@pytest.mark.parametrize("output", ["output pin carrier", "ring housing"])
def test_no_spacer_shares_space_with_another_part(output):
    """Each one sits in a gap the drive already had - on the shaft, through the
    bores, between the discs - so it touches its neighbours and goes into
    none of them."""
    from cycloidgen.export import solid

    spec = preset(21)
    spec.output_member = output
    assy = solid.build_assembly(spec)
    spacers = set(solid.spacer_parts(spec))
    placed = {c.name: c.obj.val().moved(c.loc) for c in assy.children}
    for name in spacers:
        for other, shape in placed.items():
            if other == name:
                continue
            overlap = placed[name].intersect(shape).Volume()
            assert overlap < 1e-6, f"{name} goes {overlap:.4f} mm^3 into {other}"


@pytest.mark.parametrize("output", ["output pin carrier", "ring housing"])
def test_the_screws_and_nuts_clamp_the_stack_and_go_into_none_of_it(output):
    """Heads on the output plate's outside, nuts on the input plate's, shanks
    through the clearance holes - touching every plate, inside none of them."""
    from cycloidgen.export import solid

    spec = preset(21)
    spec.output_member = output
    size = spec.tie_bolt_size
    assy = solid.build_assembly(spec)
    placed = {c.name: c.obj.val().moved(c.loc) for c in assy.children}
    nuts = placed["tie_nuts"].BoundingBox()
    assert nuts.zmin == pytest.approx(spec.barrel_top + spec.plate_thickness)
    assert nuts.zlen == pytest.approx(size.nut_m)
    for fastener in ("tie_bolts", "tie_nuts"):
        for other in ("housing", "input_end_plate", "output_end_plate"):
            overlap = placed[fastener].intersect(placed[other]).Volume()
            assert overlap < 1e-6, f"{fastener} goes {overlap:.3f} mm^3 into {other}"


def test_the_bill_names_the_screw_and_the_nut():
    from cycloidgen.export.bom import bom_items

    spec = preset(21)
    items = {i.part: i for i in bom_items(analyse(spec))}
    assert items["Tie bolt"].size == "M4 x 45 socket head cap screw, ISO 4762"
    assert items["Tie bolt nut"].size == "M4 hexagon nut, ISO 4032"
    assert items["Tie bolt"].quantity == items["Tie bolt nut"].quantity == \
        spec.housing_bolt_count


def test_nuts_on_a_turning_output_face_are_reported():
    """On a ring drive the input plate is what the load bolts to."""
    def raised(spec):
        return [f for f in analyse(spec).report.findings
                if f.code == "TIE_NUTS_ON_OUTPUT_FACE"]

    assert not raised(preset(21))
    ring = preset(21)
    ring.output_member = "ring housing"
    (found,) = raised(ring)
    assert found.value == pytest.approx(ring.tie_screw_length - ring.tie_bolt_length)


@pytest.mark.parametrize("output", ["output pin carrier", "ring housing"])
def test_the_input_end_has_a_flat_for_a_set_screw_where_one_can_reach(output):
    """A set screw biting a round shaft walks round it under load.  The flat
    runs from clear of the input plate to the end - outside everything the
    shaft passes through - a tenth of the diameter deep.  Not on a ring drive,
    whose driven end is inside the carrier's boss."""
    from cycloidgen.export import solid
    from cycloidgen.viz.mesh import build_mesh

    spec = preset(21)
    spec.output_member = output
    shaft = solid.eccentric_shaft(spec).val()
    r = spec.input_shaft_diameter / 2.0
    flats = [f for f in shaft.Faces() if f.geomType() == "PLANE"
             and abs(f.normalAt().x - 1.0) < 1e-9]
    if spec.ground_frame_fitted:
        assert spec.input_flat is None and not flats
        return
    z0, z1, depth = spec.input_flat
    assert depth == pytest.approx(0.1 * spec.input_shaft_diameter)
    assert z0 > spec.barrel_top + spec.plate_thickness      # clear of the plate
    (face,) = flats
    box = face.BoundingBox()
    assert box.xmin == pytest.approx(r - depth)
    assert box.zmin == pytest.approx(z0) and box.zmax == pytest.approx(z1)

    mesh = build_mesh(spec)
    part = next(p for p in mesh.parts if p.name == "eccentric_shaft")
    v = mesh.vertices[part.vertices]
    on_end = v[v[:, 2] > z0 + 1e-6]
    assert on_end[:, 0].max() == pytest.approx(r - depth)


# ------------------------------------------------- weighed as it is exported


@pytest.mark.parametrize("ring_output,motor,direct", [
    (False, "None", True), (True, "None", True), (False, "NEMA 17", True),
    (True, "NEMA 23", True), (False, "NEMA 17", False), (True, "NEMA 23", False),
    (False, "8108 outrunner", True)])
def test_every_made_part_weighs_what_its_solid_encloses(ring_output, motor, direct):
    """The mass model and the STEP export are two descriptions of one gearbox.

    Held part by part, because a total can agree while two parts are wrong in
    opposite directions.  The carrier's boss stood proud of its plate in the
    solid for a release before the mass model heard about it.  The solid
    carrier carries its pins as one body; the mass model weighs them as the
    dowels they are bought as, so they are added back here at their own size.
    Two percent is the edge breaks and the shaft's flat, which only the solid
    has.
    """
    from cycloidgen.analysis.bearings import pin_diameters
    from cycloidgen.core.spec import OutputMember
    from cycloidgen.export import solid

    spec = preset(21)
    spec.motor_frame = motor              # driven directly: a bored shaft
    spec.motor_drives_the_shaft = direct  # or a mount on the plate, coupled
    if ring_output:
        spec.output_member = OutputMember.RING
    m = analyse(spec).mass
    parts = solid.parts(spec)

    def volume(grams, material):          # mm^3
        return grams / material.density_g_cm3 * 1000.0

    _, out_d = pin_diameters(spec)
    out_pins = (spec.output_pin_count * np.pi * (out_d / 2.0) ** 2
                * spec.output_pin_length)
    house = spec.housing_mat
    expected = {
        ("housing",): volume(m.housing_mass_g, house),
        ("output_flange",): volume(m.flange_mass_g, house) + out_pins,
        ("input_end_plate", "output_end_plate"): volume(m.plates_mass_g, house),
        ("eccentric_shaft",): volume(m.shaft_mass_g, spec.shaft_mat),
        ("disc_1",): volume(m.disc_mass_g, spec.disc_mat),
    }
    if spec.ground_frame_fitted:
        expected[("end_cap",)] = volume(m.end_cap_mass_g, house)
    if spec.rotor_hub is not None:
        expected[("rotor_hub",)] = volume(m.rotor_hub_mass_g, spec.shaft_mat)
    for names, analytic in expected.items():
        enclosed = sum(parts[n].val().Volume() for n in names)
        assert analytic == pytest.approx(enclosed, rel=0.02), names


@pytest.mark.parametrize("ring_output,coupled", [
    (False, False), (True, False), (False, True), (True, True)])
def test_the_inertia_is_the_inertia_of_the_exported_solids(ring_output, coupled):
    """The shaft and the turning output member, against what OCCT integrates
    over the solids the STEP file is made of.

    The pins are given the housing's material for this, because the solid
    carrier carries its pins as one body and one body has one density.  The
    tie bolts are counted as their mass on their circle, which leaves out
    their own spin - a part in a thousand of the housing they turn with.
    Through a coupling, the coupling turns with the shaft, in aluminium.
    """
    import cadquery as cq

    from cycloidgen.core.spec import OutputMember
    from cycloidgen.export import solid

    spec = preset(21)
    spec.pin_material = spec.housing_material
    if ring_output:
        spec.output_member = OutputMember.RING
    if coupled:
        spec.motor_frame = "NEMA 17"
        spec.motor_drives_the_shaft = False
    m = analyse(spec).mass
    parts = {**solid.parts(spec), "tie_bolts": solid.tie_bolts(spec),
             "tie_nuts": solid.tie_nuts(spec)}

    def inertia(names, material):          # kg mm^2, about the drive's axis
        return sum(cq.Shape.matrixOfInertia(parts[n].val())[2][2] for n in names) \
            * material.density_g_cm3 * 1e-6

    shaft = inertia(["eccentric_shaft", "shaft_spacer_input", "shaft_spacer_output"],
                    spec.shaft_mat)
    if coupled:
        from cycloidgen.core.spec import MATERIALS

        aluminium = MATERIALS["Aluminium 7075-T6"]
        parts["coupling"] = solid.coupling_solid(spec)
        shaft += inertia(["coupling"], aluminium)
        assert m.coupling_mass_g == pytest.approx(
            parts["coupling"].val().Volume() * aluminium.density_g_cm3 / 1000.0,
            rel=0.02)
    assert m.shaft_inertia_kg_mm2 == pytest.approx(shaft, rel=0.02)
    turning = (["housing", "input_end_plate", "output_end_plate", "ring_pins",
                "tie_bolts", "tie_nuts"] if ring_output else ["output_flange"])
    assert m.output_inertia_kg_mm2 == pytest.approx(
        inertia(turning, spec.housing_mat), rel=0.02)
    assert m.input_inertia_kg_mm2 == pytest.approx(
        m.shaft_inertia_kg_mm2 + m.reflected_inertia_kg_mm2
        + m.output_inertia_kg_mm2 / spec.ratio ** 2)


@pytest.mark.parametrize("bolt", [4.5, 3.4])
def test_a_plate_s_edge_breaks_leave_its_lip_standing(bolt):
    """A lip's bore is broken on both faces, and two breaks that meet are a
    knife edge the CAD kernel will not cut.  An M3 tie bolt leaves a millimetre
    of rim land in the default wall, which let the break grow to half a
    millimetre on a one-millimetre lip, and the plate - with every solid
    exported alongside it - failed.  Read off the solid: what is left standing
    of each lip's bore.  At the default bolt the break is what it always was."""
    from cycloidgen.analysis.bearings import (
        LIP_LAND,
        bearing_schedule,
        plate_edge_break,
        seat_bores,
    )
    from cycloidgen.export import solid

    spec = preset(21)
    spec.housing_bolt_diameter = bolt
    bores = seat_bores(spec, bearing_schedule(spec))
    parts = solid.parts(spec)
    for name in ("input_end_plate", "output_end_plate"):
        body = parts[name].val()
        assert body.isValid(), name
        lips = [s for s in bores.shoulders if s.host == name]
        assert lips, name
        for lip in lips:
            standing = [f.BoundingBox().zlen for f in body.Faces()
                        if f.geomType() == "CYLINDER"
                        and abs(f.BoundingBox().xlen - lip.bore) < 1e-6]
            assert standing and max(standing) >= LIP_LAND - 1e-6, name
        if bolt == 4.5:
            assert plate_edge_break(spec, bores.shoulders, name) == spec.edge_break


def test_a_bearing_s_corner_is_radiused_and_a_washer_s_is_not():
    """Inside corners where a bearing ring sits get the radius a cutter leaves -
    smaller than any ring's own chamfer, so the ring still sits flat.  The
    shaft's do not: a sleeve or a millimetre washer stands on every cam face,
    and neither can carry the chamfer that would clear one."""
    from cycloidgen.export import solid

    assert solid.INSIDE_RADIUS < 0.3          # the least ring chamfer here
    parts = solid.parts(preset(21))

    def radii(name):
        return [f for f in parts[name].val().Faces() if f.geomType() == "TORUS"]

    for name in ("output_flange", "input_end_plate", "output_end_plate"):
        assert radii(name), name
        assert parts[name].val().isValid(), name
    assert not radii("eccentric_shaft")


def _presets():
    from cycloidgen.core.presets import PRESETS

    return [p.key for p in PRESETS]


@pytest.mark.parametrize("key", _presets())
@pytest.mark.parametrize("output", ["output pin carrier", "ring housing"])
def test_the_carrier_turns_clear_of_the_ring_pins(key, output):
    """The carrier sits inside the barrel among the ring pins, and so does the
    end cap on a ring drive: read off the solids, each one shares no metal with
    the pins - the 29:1 NEMA 23's 12 mm output pins put its carrier half a
    millimetre into them - and the radius it was held to is the room there
    is, worked out by hand."""
    import cadquery as cq

    from cycloidgen.core.presets import by_key
    from cycloidgen.core.spec import CARRIER_RIM_GAP, OutputMember
    from cycloidgen.export import solid

    spec = by_key(key).spec()
    spec.output_member = OutputMember(output)
    room = spec.pin_circle_radius - spec.pin_radius - CARRIER_RIM_GAP
    assert spec.carrier_radius <= room + 1e-9
    pins = solid.ring_pins(spec).val()
    carrier = solid.output_flange(spec).val().moved(
        cq.Location(cq.Vector(0.0, 0.0, -CARRIER_DROP)))
    assert pins.intersect(carrier).Volume() == pytest.approx(0.0, abs=1e-6)
    if spec.ground_frame_fitted:
        assert pins.intersect(solid.end_cap(spec).val()).Volume() == pytest.approx(
            0.0, abs=1e-6)


def test_a_carrier_held_inside_the_ring_pins_says_when_its_rim_is_thin():
    spec = preset(21)
    spec.output_pin_diameter = 10.0
    spec.output_bolt_circle_radius = spec.pin_circle_radius - spec.pin_radius - 7.0
    rim = (spec.carrier_radius - spec.output_bolt_circle_radius
           - spec.output_pin_diameter / 2.0)
    assert rim == pytest.approx(7.0 - 0.5 - 5.0)
    found = next(f for f in analyse(spec).report.findings
                 if f.code == "THIN_CARRIER_RIM")
    assert found.severity is Severity.WARNING
    assert found.value == pytest.approx(rim)
    assert found.limit == pytest.approx(2.5)
