"""Which member the output comes off, and everything that follows from it.

A cycloidal drive is a three-shaft machine.  The crank is always the input; the
ring and the carrier are interchangeable, and grounding one rather than the
other is a real design choice that changes the reduction, the direction, the
speed of every rubbing contact, which part turns on screen and which end of the
gearbox the motor bolts to.  The app used to have the first of the two wired in
as an assumption spread across a dozen modules.

The tests here fall into two groups.  The first re-derives the motion from the
meshing constraint and checks the rate model against it - that is the half that
found a sign error the analysis had been carrying in three places.  The second
follows the choice out into the geometry, because a ratio that changes while
the motor stays bolted to a plate that now turns is not a feature, it is a
gearbox nobody can build.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from cycloidgen.analysis import analyse
from cycloidgen.core import kinematics as kin
from cycloidgen.core.spec import CARRIER_DROP, OutputMember, preset

RATIOS = [10, 15, 21, 29, 39, 59]


def _spec(ratio: int = 15, member: OutputMember = OutputMember.RING, **kw):
    spec = preset(ratio)
    spec.output_member = member
    for name, value in kw.items():
        setattr(spec, name, value)
    return spec


# ------------------------------------------------------------------- the ratio


@pytest.mark.parametrize("ratio", RATIOS)
def test_grounding_the_other_member_is_worth_exactly_one_tooth(ratio):
    """The two members are one tooth apart, so the two reductions are too.

    Not an approximation and not a coincidence: it is the whole content of a
    single-tooth-difference drive, and it is the reason a ring-output build is
    free reduction - the same parts, bolted down at the other end.
    """
    carrier = _spec(ratio, OutputMember.CARRIER)
    ring = _spec(ratio, OutputMember.RING)
    assert carrier.ratio == carrier.lobes
    assert ring.ratio == ring.pin_count
    assert ring.ratio == carrier.ratio + 1


@pytest.mark.parametrize("ratio", RATIOS)
def test_the_ratio_is_the_quotient_of_the_two_spins_it_is_meant_to_be(ratio):
    """``ratio`` is declared as a tooth count and used as a speed ratio.

    Those are two different statements about the drive and only one of them is
    in the code, so this is where they are held together: whatever the spins
    say the reduction is, the integer has to agree - and it has to agree in
    both configurations, which is what stops the field being right by accident
    in the one it was written for.
    """
    for member in OutputMember:
        spec = _spec(ratio, member)
        assert abs(spec.shaft_spin / spec.output_spin) == pytest.approx(spec.ratio)
        assert spec.output_rpm == pytest.approx(spec.input_rpm / spec.ratio)


@pytest.mark.parametrize("ratio", RATIOS)
def test_only_the_carrier_reverses(ratio):
    """Off the carrier the output turns against the input; off the ring it turns
    with it.  Read off the signs rather than declared, so the flag cannot
    disagree with the picture drawn from the same numbers."""
    carrier, ring = _spec(ratio, OutputMember.CARRIER), _spec(ratio, OutputMember.RING)
    assert carrier.output_reverses
    assert not ring.output_reverses
    assert (carrier.output_spin < 0) != (carrier.shaft_spin < 0)
    assert (ring.output_spin < 0) == (ring.shaft_spin < 0)


# -------------------------------------------------------------- the rate model


@pytest.mark.parametrize("ratio", RATIOS)
def test_a_relative_speed_does_not_care_which_member_is_bolted_down(ratio):
    """Grounding a member adds one rigid rotation to the whole drive.

    So anything measured *between* two bodies has to come out the same in both
    configurations when it is stated per unit crank angle - and anything
    measured against the ground has to differ.  Both halves matter: a model
    that got the first wrong would report a mesh that changes when you turn the
    gearbox round, and one that got the second wrong would report an output
    that does not.
    """
    carrier, ring = _spec(ratio, OutputMember.CARRIER), _spec(ratio, OutputMember.RING)
    for a, b in ((carrier, ring),):
        assert (a.disc_spin - a.shaft_spin) == pytest.approx(b.disc_spin - b.shaft_spin)
        assert (a.carrier_spin - a.ring_spin) == pytest.approx(
            b.carrier_spin - b.ring_spin)
    # ...and against the ground they differ, which is the point of the choice
    assert carrier.ring_spin == 0.0
    assert ring.carrier_spin == pytest.approx(0.0)


@pytest.mark.parametrize("ratio", RATIOS)
def test_the_disc_runs_against_the_crank_so_their_rates_add(ratio):
    """``1 + 1/N`` per crank radian, and the app read ``1 - 1/N`` for a long time.

    The disc turns the opposite way from the crank, so the bearing between them
    sees the sum.  With the sign the other way the cam bearing - the fastest in
    the drive - was understated by 14% at fifteen lobes, and understated worst
    exactly where the ratio is low and the speed is highest.
    """
    for member in OutputMember:
        spec = _spec(ratio, member)
        per_crank = abs(spec.disc_spin - spec.shaft_spin)
        assert per_crank == pytest.approx(1.0 + 1.0 / ratio)
        assert spec.crank_relative_rate == pytest.approx(per_crank * spec.crank_rate)

    # And in each configuration it lands on a number worth recognising.
    assert _spec(ratio, OutputMember.CARRIER).crank_relative_rate == pytest.approx(
        1.0 + 1.0 / ratio)
    # With the carrier grounded the disc does not rotate at all, so the cam
    # bearing simply turns at the input speed.
    assert _spec(ratio, OutputMember.RING).crank_relative_rate == pytest.approx(1.0)


@pytest.mark.parametrize("ratio", RATIOS)
def test_the_carrier_rotation_is_the_one_the_pins_can_actually_follow(ratio):
    """Solved from the coupling rather than assumed.

    The output pins sit in holes on the same bolt circle, and the disc
    translates on a circle of radius E relative to the carrier - so at every
    crank angle each pin must be exactly E from its hole centre.  Only one
    carrier rotation satisfies that, and it is the disc's own.  This is the
    constraint that caught the sign: with the rate turned round the pins walk
    out of their holes within a few degrees.
    """
    spec = _spec(ratio, OutputMember.CARRIER)
    n, bc, E = spec.output_pin_count, spec.output_bolt_circle_radius, spec.eccentricity
    gamma = 2.0 * np.pi * np.arange(n) / n
    for phi in (0.0, 0.37, 1.9, 4.4):
        holes = kin.to_world(bc * np.column_stack([np.cos(gamma), np.sin(gamma)]),
                             phi, E, ratio)
        psi = phi / ratio                      # the carrier's rotation
        pins = bc * np.column_stack([np.cos(gamma + psi), np.sin(gamma + psi)])
        assert np.allclose(np.linalg.norm(holes - pins, axis=1), E, atol=1e-9)


@pytest.mark.parametrize("ratio", RATIOS)
def test_the_two_stage_periods_agree_once_the_sign_is_right(ratio):
    """The ring pattern repeats over a turn; the output pattern repeats ``n``
    times in it.

    The two were derived independently - one from the profile parameter at each
    pin, the other from the eccentricity direction seen from the carrier - and
    they only meet on the correct rate.  They did not, and the output stage was
    being swept over a window 14% too wide that was not a period of anything.
    """
    for pins in (4, 6, 9, 12):
        assert (kin.ring_stage_period(ratio) / pins
                == pytest.approx(kin.output_stage_period(ratio, pins)))


@pytest.mark.parametrize("ratio", [15, 29])
def test_the_output_load_pattern_really_does_repeat_over_that_period(ratio):
    """The period claimed, measured on the thing it is a period of.

    The set of loaded pins comes back rotated by one pin pitch rather than
    identical, so it is the force *spectrum* that has to match - which is
    exactly what every statistic taken over the sweep depends on.
    """
    spec = preset(ratio)
    period = kin.output_stage_period(ratio, spec.output_pin_count)
    for phi in (0.0, 0.29, 1.13):
        here = np.sort(kin.output_loads(spec, phi, 1000.0).forces)
        later = np.sort(kin.output_loads(spec, phi + period, 1000.0).forces)
        assert np.allclose(here, later, atol=1e-9)


def test_a_ring_output_drive_reflects_less_inertia_back_to_the_motor():
    """The disc does not rotate in the ground frame when the carrier is held, so
    only its orbiting mass is reflected - a consequence of the rate model that
    is worth pinning down because nothing else in the app would notice it."""
    carrier, ring = _spec(21, OutputMember.CARRIER), _spec(21, OutputMember.RING)
    assert carrier.disc_speed_ratio == pytest.approx(1.0 / 21)
    assert ring.disc_speed_ratio == pytest.approx(0.0)
    assert (analyse(ring).mass.reflected_inertia_kg_mm2
            < analyse(carrier).mass.reflected_inertia_kg_mm2)


# ------------------------------------------------------------------ the bearings


def test_the_cam_bearing_speed_follows_the_member_that_is_grounded():
    """The one number the sign error moved most, checked against the model that
    replaced it rather than against a constant."""
    for member in OutputMember:
        spec = _spec(21, member, input_rpm=1200.0)
        cam = next(c for c in analyse(spec).bearings
                   if c.role == "Eccentric cam bearing")
        assert cam.speed_rpm == pytest.approx(
            spec.input_rpm * spec.crank_relative_rate)
    # It is above the input speed off the carrier and exactly it off the ring.
    assert _spec(21, OutputMember.CARRIER).crank_relative_rate > 1.0


def test_the_main_output_bearing_turns_at_the_output_speed_either_way():
    """It separates the carrier from the ring, and that difference *is* the
    output speed however the drive is mounted - which is why the one seat whose
    geometry does not move between the two configurations is this one."""
    for member in OutputMember:
        spec = _spec(21, member)
        rate = abs(spec.carrier_spin - spec.ring_spin) * spec.crank_rate
        assert rate == pytest.approx(1.0 / spec.ratio)
        main = next(c for c in analyse(spec).bearings
                    if c.role == "Main output bearing")
        assert main.speed_rpm == pytest.approx(spec.output_rpm)


# ------------------------------------------------------------------ the geometry


def test_the_motor_bolts_to_the_member_that_stands_still():
    """A motor on a plate that turns is a gearbox nobody can build.

    So the pattern moves off the input end plate and onto the carrier's base
    when the housing becomes the output - and the plate it left has to come out
    heavier than it was, because the holes are no longer in it.
    """
    from cycloidgen.export import solid

    carrier = _spec(21, OutputMember.CARRIER, motor_frame="NEMA 17")
    ring = _spec(21, OutputMember.RING, motor_frame="NEMA 17")
    assert carrier.motor_mounts_on_carrier is False
    assert ring.motor_mounts_on_carrier is True

    plain = solid.housing_end_plate(carrier, carrier.hub_bore).val().Volume()
    with_motor = solid.housing_end_plate(
        carrier, carrier.hub_bore, motor_face=True).val().Volume()
    assert with_motor < plain                       # four holes taken out of it

    # And on the ring-output drive the motor's holes are in the carrier instead.
    # The carrier is modelled in its own frame, with its top face at zero, so an
    # assembled height reads a carrier drop higher here.
    # It wraps the motor, down to its back.
    flange = solid.output_flange(ring).val()
    box = flange.BoundingBox()
    assert ring.base_wraps_motor
    assert box.zmin == pytest.approx(ring.sleeve_bottom + CARRIER_DROP)
    half = ring.motor_face.bolt_span / 2.0
    reach = math.hypot(half, half) + ring.motor_face.bolt_diameter / 2.0
    assert reach < ring.sleeve_half_width           # the pattern lands on it


def test_the_base_is_only_there_when_the_carrier_is_the_ground():
    """Off the carrier the boss is a shaft end and a coupling grips it; a plate
    across the end of it would be a plate across the output."""
    from cycloidgen.export import solid

    carrier, ring = _spec(21, OutputMember.CARRIER), _spec(21, OutputMember.RING)
    assert not carrier.ground_frame_fitted and ring.ground_frame_fitted
    assert (solid.output_flange(ring).val().Volume()
            > solid.output_flange(carrier).val().Volume())
    # The envelope grows by the base and the standoff it needs, by the
    # barrel's own growth, because the end cap lives inside it, and by the
    # housing's closed top over its bearing.  The plates are the same
    # thickness both ways on this design, a wall and 2 mm.
    from cycloidgen.core.spec import CUP_CEILING

    assert ring.housing_plate_thickness == carrier.plate_thickness
    assert (ring.envelope_length - carrier.envelope_length
            == pytest.approx(ring.plate_thickness + ring.boss_protrusion
                             + CARRIER_DROP + ring.output_flange_thickness
                             + CUP_CEILING))


def test_the_mass_model_weighs_the_base_it_exported():
    """A made part the mass model has not been told about is a gearbox that
    weighs less on paper than in your hand.

    Compared as the *difference* between the two configurations rather than as
    an absolute, because the carrier's solid also carries the output pins and
    those are weighed as bought dowels rather than as part of it.  The
    difference is the base and nothing else, which is exactly the thing being
    checked.
    """
    from cycloidgen.export import solid

    carrier, ring = _spec(21, OutputMember.CARRIER), _spec(21, OutputMember.RING)
    rho = ring.housing_mat.density_g_cm3
    drawn = (solid.output_flange(ring).val().Volume()
             - solid.output_flange(carrier).val().Volume())
    weighed = (analyse(ring).mass.flange_mass_g
               - analyse(carrier).mass.flange_mass_g) / rho * 1000.0
    assert drawn > 0
    assert weighed == pytest.approx(drawn, rel=0.02)


def test_the_turning_housing_is_closed_and_the_load_screws_into_it():
    """A barrel has nowhere to grip, so off the ring the interface is a face:
    the housing's own closed top, the driven machine's screw holes blind in it.
    Probed in the solid - closed on the axis, each hole open to its depth and
    shut past it - and there is no input end plate to make."""
    import cadquery as cq

    from cycloidgen.export import solid

    ring = _spec(21, OutputMember.RING)
    housing = solid.ring_housing(ring).val()
    top = ring.cup_top
    assert housing.isInside(cq.Vector(0.0, 1.0, top - 0.5))
    depth = ring.output_face_hole_depth
    assert len(ring.output_face_bolt_centres) == ring.output_bolt_count
    for x, y in ring.output_face_bolt_centres:
        assert not housing.isInside(cq.Vector(x, y, top - depth + 0.2))
        assert housing.isInside(cq.Vector(x, y, top - depth - 0.2))
    assert "input_end_plate" not in solid.parts(ring)

    none = _spec(21, OutputMember.RING, output_bolt_count=0)
    assert not none.output_face_bolt_centres
    assert solid.ring_housing(none).val().Volume() > housing.Volume()


def test_the_face_s_screws_go_in_the_solid_top_not_down_the_wall():
    """The top is solid from the second main bearing's seat to the rim, so the
    holes go in the middle of that, where an insert has the room a 6 mm wall
    does not give it - unless they are deeper than the top, when only the
    middle of the barrel's wall clears the pin pockets under it."""
    from cycloidgen.core.spec import CAP_CLEARANCE, CUP_CEILING

    ring = _spec(21, OutputMember.RING)
    assert ring.output_face_bolt_radius == pytest.approx(
        (ring.output_bearing_seat_diameter / 2.0 + ring.housing_outer_radius) / 2.0)
    assert ring.output_face_hole_depth <= (ring.housing_plate_thickness + CUP_CEILING
                                           - CAP_CLEARANCE)
    deep = _spec(21, OutputMember.RING, output_bolt_diameter=13.5)    # an M12
    assert deep.output_face_hole_depth > (deep.housing_plate_thickness + CUP_CEILING
                                          - CAP_CLEARANCE)
    assert deep.output_face_bolt_radius == deep.housing_bolt_radius


# ---------------------------------------------------------------- the frame


def test_the_output_pins_land_in_something_at_the_far_end():
    """The reference drive's whole structure, in one assertion.

    A carrier-output drive extrudes its pins from one plate and nothing catches
    them; a ring-output drive lands them in an end cap and the two grounded
    plates are held together *by the pins*.  Everything structural about this
    configuration follows from that, so it is checked on the geometry rather
    than on the flag.
    """
    from cycloidgen.export import solid

    carrier, ring = _spec(21, OutputMember.CARRIER), _spec(21, OutputMember.RING)
    assert not carrier.output_pins_are_supported_at_both_ends
    assert ring.output_pins_are_supported_at_both_ends

    # The pin reaches through the cap, because it is also the bolt holding it on.
    reach = -CARRIER_DROP + ring.output_pin_length
    assert reach == pytest.approx(ring.end_cap_top)
    # ...and the cap is a drop clear of the last disc, mirroring the carrier.
    assert ring.end_cap_bottom == pytest.approx(ring.stack_height + CARRIER_DROP)

    cap = solid.end_cap(ring).val()
    box = cap.BoundingBox()
    assert box.zmin == pytest.approx(ring.end_cap_bottom)
    assert box.zmax == pytest.approx(ring.cap_boss_top)
    # It lives inside the barrel it is turning inside, so it has to clear the
    # ring pins - which is the constraint that decides it can exist at all.
    assert box.xlen / 2.0 < ring.pin_circle_radius - ring.pin_radius


def test_the_barrel_covers_the_cap_it_turns_around():
    """A barrel that stopped at the discs would leave the cap standing out of
    the housing - the same fault the carrier had before the barrel was
    lengthened for it, at the other end."""
    ring = _spec(21, OutputMember.RING)
    assert ring.barrel_top == pytest.approx(ring.end_cap_top)
    assert ring.barrel_bottom < -CARRIER_DROP
    # And the ring pins run the barrel's whole length, so they grew with it.
    assert ring.ring_pin_length == pytest.approx(ring.barrel_height)
    carrier = _spec(21, OutputMember.CARRIER)
    assert carrier.barrel_top == pytest.approx(carrier.stack_height)


def test_a_beam_is_not_a_cantilever():
    """The factor the end cap buys, measured rather than asserted.

    Both the bending moment and the pin's own stiffness change, and they change
    in *opposite* directions - the moment falls by more than half, the stiffness
    falls too because the span is now the whole pin rather than half a stack.
    Both are real and the app should report both, so both are pinned here.
    """
    from cycloidgen.analysis.fatigue import output_pin_bending_moment_Nmm

    carrier, ring = _spec(21, OutputMember.CARRIER), _spec(21, OutputMember.RING)
    for spec in (carrier, ring):
        spec.disc_count = 2
    cantilever = output_pin_bending_moment_Nmm(carrier, 100.0)
    beam = output_pin_bending_moment_Nmm(ring, 100.0)
    assert beam < cantilever / 2.0

    pin_a = next(p for p in analyse(carrier).fatigue.parts
                 if p.part == "output pin")
    pin_b = next(p for p in analyse(ring).fatigue.parts
                 if p.part == "output pin")
    assert pin_b.alternating_MPa < pin_a.alternating_MPa / 2.0


def test_a_single_disc_beam_is_the_textbook_case():
    """One central load on two supports is ``F*L/4``.

    The general expression this uses has to collapse to that, or the stack case
    it was written for is being trusted on nothing.
    """
    from cycloidgen.analysis.fatigue import output_pin_bending_moment_Nmm

    spec = _spec(21, OutputMember.RING, disc_count=1)
    span = spec.output_pin_length
    a = CARRIER_DROP + spec.stack_height / 2.0
    b = span - a
    assert output_pin_bending_moment_Nmm(spec, 100.0) == pytest.approx(
        100.0 * a * b / span)


def test_the_housing_is_carried_at_both_ends():
    """One bearing locates a housing; it does not hold one against a moment.

    Two, one on each of the frame's bosses, is what the end cap makes possible -
    and it is the difference between a drive you can hang a wheel on and one you
    can only couple to.
    """
    from cycloidgen.analysis.bearings import placements_for_spec

    carrier, ring = _spec(21, OutputMember.CARRIER), _spec(21, OutputMember.RING)
    main = next(c for c in analyse(ring).bearings
                if c.role == "Main output bearing")
    assert main.count == 2
    assert next(c for c in analyse(carrier).bearings
                if c.role == "Main output bearing").count == 1

    placed = {p.name: p for p in placements_for_spec(ring)}
    assert {"bearing_output_main", "bearing_output_input"} <= set(placed)
    # One in the plate under the housing and one in the housing's own top, so
    # each stays with the part holding its outer ring.
    assert placed["bearing_output_main"].host == "output_end_plate"
    assert placed["bearing_output_input"].host == "housing"
    # And they are at opposite ends of the drive, which is the point.
    assert placed["bearing_output_main"].rings[0].z0 < 0.0
    assert placed["bearing_output_input"].rings[0].z0 > ring.stack_height


def test_the_shaft_supports_move_into_the_grounded_bosses():
    """A support whose outer ring turns is not a shaft support.

    With a frame the input end plate turns, so the seat that used to be in it
    moves into the end cap's boss - which mirrors the carrier's boss at the
    other end and is the same part number in both.
    """
    from cycloidgen.analysis.bearings import placements_for_spec

    placed = {p.name: p.host for p in placements_for_spec(_spec(21, OutputMember.RING))}
    assert placed["bearing_shaft_input"] == "end_cap"
    assert placed["bearing_shaft_output"] == "output_flange"

    plain = {p.name: p.host
             for p in placements_for_spec(_spec(21, OutputMember.CARRIER))}
    assert plain["bearing_shaft_input"] == "input_end_plate"


def test_the_end_cap_is_a_part_you_can_order_and_make():
    """It reaches the manifest, the per-part exports and the bill of materials -
    and it weighs something, which is the check that catches a made part the
    mass model has never been told about."""
    from cycloidgen.export import solid
    from cycloidgen.export.bom import bom_items
    from cycloidgen.export.manifest import part_names

    ring = _spec(21, OutputMember.RING)
    assert "end_cap" in part_names(ring)
    assert "end_cap" in solid.parts(ring)
    assert "end_cap" not in part_names(_spec(21, OutputMember.CARRIER))
    assert "end_cap" not in solid.parts(_spec(21, OutputMember.CARRIER))

    line = next(i for i in bom_items(analyse(ring)) if i.part == "End cap")
    assert line.source == "make" and line.mass_each_g > 0.0

    rho = ring.housing_mat.density_g_cm3
    assert analyse(ring).mass.end_cap_mass_g / rho * 1000.0 == pytest.approx(
        solid.end_cap(ring).val().Volume(), rel=0.02)


def test_the_carrier_is_one_solid_and_not_two():
    """The base is built in assembled coordinates inside a part-local frame, so
    it has to be lifted by the carrier drop to land on the boss.  It was not,
    and the part came out in two pieces - with the right total volume, which is
    why nothing caught it and a printer would have made both."""
    from cycloidgen.export import solid

    for member in OutputMember:
        flange = solid.output_flange(_spec(21, member))
        assert len(flange.solids().vals()) == 1, member


@pytest.mark.parametrize("counts", [(6, 6), (8, 8), (6, 12), (6, 7)])
def test_the_face_s_screws_and_the_plate_s_never_meet(counts):
    """They used to share one plate and one circle, so the counts decided
    whether a hole went through a hole.  The face is the housing's closed top
    now and the plate's screws come up from under the housing, so no pair of
    counts collides."""
    ties, outs = counts
    spec = _spec(21, OutputMember.RING, housing_bolt_count=ties,
                 output_bolt_count=outs)
    assert not [f for f in analyse(spec).report.findings
                if f.code == "OUTPUT_BOLT_CLASH"]


def test_the_check_does_not_fire_on_a_carrier_output_drive():
    """There is no output bolt circle there at all - the interface is the boss."""
    spec = _spec(21, OutputMember.CARRIER, output_bolt_count=12)
    assert not [f for f in analyse(spec).report.findings
                if f.code == "OUTPUT_BOLT_CLASH"]


# -------------------------------------------------------------- what turns


@pytest.mark.parametrize("ratio", [15, 21])
def test_the_grounded_part_does_not_move_and_the_output_part_does(ratio):
    """The 3D view has to draw the mechanism the analysis describes.

    Measured on the vertices rather than on the declared spin, because the
    frame rotation is applied at placement time and a part could carry the
    right number and still be drawn in the wrong frame.
    """
    from cycloidgen.viz.mesh import build_mesh

    for member in OutputMember:
        spec = _spec(ratio, member)
        mesh = build_mesh(spec)
        at_zero = mesh.world_vertices(0.0)
        turned = mesh.world_vertices(1.1)
        moved = {}
        for part in mesh.parts:
            delta = np.abs(turned[part.vertices] - at_zero[part.vertices]).max()
            moved[part.name] = float(delta)
        assert moved[spec.grounded_part] < 1e-9, (member, moved)
        assert moved[spec.output_part] > 1e-6, (member, moved)


@pytest.mark.parametrize("ratio", [15, 21])
def test_the_output_part_turns_by_exactly_one_over_the_ratio(ratio):
    """Not merely "it moves": the angle it moves by is the reduction, and the
    reduction is what the whole choice is about."""
    from cycloidgen.viz.mesh import build_mesh

    for member in OutputMember:
        spec = _spec(ratio, member)
        mesh = build_mesh(spec)
        part = next(p for p in mesh.parts if p.name == spec.output_part)
        # A point well off the axis, so the angle is readable.
        base = mesh.world_vertices(0.0)[part.vertices]
        j = int(np.argmax(np.hypot(base[:, 0], base[:, 1])))
        phi = 0.9
        after = mesh.world_vertices(phi)[part.vertices][j]
        turn = math.atan2(after[1], after[0]) - math.atan2(base[j][1], base[j][0])
        turn = (turn + math.pi) % (2.0 * math.pi) - math.pi
        expected = abs(spec.shaft_spin) * phi / spec.ratio
        assert abs(turn) == pytest.approx(expected, rel=1e-6)


@pytest.mark.parametrize("ratio", [15, 21])
def test_both_renderers_place_a_ring_output_part_the_same_way(ratio):
    """The software view transforms vertices; the hardware view hands VTK a
    rotation and a translation.

    They already agreed on the per-part motion, and the frame rotation is a
    second place the same law now has to be written down - so it is a second
    place it can be left out.  Leaving it out of the pose is exactly the bug
    that would draw the housing turning in one view and standing still in the
    other, from one mesh, with nothing failing.
    """
    from cycloidgen.viz.mesh import build_mesh
    from cycloidgen.viz.vtkbridge import pose_matrix

    mesh = build_mesh(_spec(ratio, OutputMember.RING))
    assert mesh.frame_spin != 0.0
    phi = 1.3
    world = mesh.world_vertices(phi, explode=0.4)
    for part in mesh.parts:
        angle, dx, dy, dz = pose_matrix(mesh, part, phi, explode=0.4)
        c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
        local = mesh.vertices[part.vertices]
        placed = local[:, :2] @ np.array([[c, -s], [s, c]]).T + [dx, dy]
        assert np.allclose(placed, world[part.vertices, :2], atol=1e-9), part.name
        assert np.allclose(local[:, 2] + dz, world[part.vertices, 2], atol=1e-9)


def test_the_mesh_cache_can_tell_the_two_configurations_apart():
    """They differ in the base, the motor face, the bolt pattern and the frame
    the whole thing is drawn in - a shared fingerprint would serve one for the
    other."""
    from cycloidgen.viz.mesh import mesh_fingerprint, mesh_for_spec

    carrier, ring = _spec(21, OutputMember.CARRIER), _spec(21, OutputMember.RING)
    assert mesh_fingerprint(carrier) != mesh_fingerprint(ring)
    assert mesh_for_spec(carrier) is not mesh_for_spec(ring)


@pytest.mark.parametrize("ratio", RATIOS)
def test_every_part_of_a_ring_output_drive_is_still_watertight(ratio):
    """The base is a new prism stacked on the boss and pierced by the motor's
    bolts, and every one of those is a chance to leave a face open.  The section
    plane can only cap a closed surface, so this is the test that says the new
    geometry is geometry and not a heap of panels."""
    from vtkmodules.vtkFiltersCore import vtkFeatureEdges, vtkTriangleFilter

    from cycloidgen.viz.mesh import build_mesh
    from cycloidgen.viz.vtkbridge import closed_polydata

    spec = _spec(ratio, OutputMember.RING, motor_frame="NEMA 23")
    mesh = build_mesh(spec)
    faults = []
    for part in mesh.parts:
        triangles = vtkTriangleFilter()
        triangles.SetInputData(closed_polydata(mesh, part))
        triangles.Update()
        for boundary, nonmanifold, label in ((True, False, "holes"),
                                             (False, True, "non-manifold")):
            edges = vtkFeatureEdges()
            edges.SetInputConnection(triangles.GetOutputPort())
            edges.SetBoundaryEdges(boundary)
            edges.SetNonManifoldEdges(nonmanifold)
            edges.FeatureEdgesOff()
            edges.ManifoldEdgesOff()
            edges.Update()
            count = edges.GetOutput().GetNumberOfCells()
            if count:
                faults.append(f"{part.name}: {count} {label}")
    assert not faults, "; ".join(faults)


def test_a_saved_ring_output_design_comes_back_as_one():
    """The choice has to survive the round trip, or a saved design reopens as a
    different gearbox with a different ratio."""
    from cycloidgen.core.designfile import design_dict, spec_from_dict

    spec = _spec(29, OutputMember.RING, output_bolt_count=8, housing_bolt_count=8)
    back = spec_from_dict(design_dict(spec))
    assert back.output_member is OutputMember.RING
    assert back.ratio == spec.ratio == 30
    assert back.output_bolt_count == 8


# ------------------------------------------------ the plate under the housing


def test_a_ring_drive_s_plate_screws_come_up_into_the_housing_heads_let_in():
    """Probed in the solids: each screw's hole is open up into the housing's
    underside to its depth and shut past it, its counterbore in the plate is
    open with the land left outside it, the heads are inside the plate so
    nothing stands under it, and each screw has its thread in the housing."""
    import cadquery as cq

    from cycloidgen.core.spec import COUNTERBORE_LAND
    from cycloidgen.export import solid

    spec = _ring("nema17-printed-15")
    # An M4's counterbore would break out of the 6 mm wall.
    assert spec.bolt_size_asked.name == "M4" and spec.tie_bolt_size.name == "M3"
    width, depth = spec.tie_counterbore
    made = solid.parts(spec)
    housing, plate = made["housing"].val(), made["output_end_plate"].val()
    z0 = spec.barrel_bottom
    for x, y in spec.tie_bolt_centres:
        r = math.hypot(x, y)
        ux, uy = x / r, y / r
        assert not housing.isInside(cq.Vector(x, y, z0 + spec.tie_thread_depth - 0.2))
        assert housing.isInside(cq.Vector(x, y, z0 + spec.tie_thread_depth + 0.2))
        # the plate in its own frame, the outside face at zero
        inside, land = width / 2.0 - 0.1, width / 2.0 + COUNTERBORE_LAND - 0.1
        assert not plate.isInside(cq.Vector(x + ux * inside, y + uy * inside,
                                            depth - 0.2))
        assert plate.isInside(cq.Vector(x + ux * land, y + uy * land, depth - 0.2))
    screws = solid.tie_bolts(spec).val().BoundingBox()
    assert screws.zmin >= z0 - spec.housing_plate_thickness - 1e-6
    assert screws.zmax - z0 >= spec.tie_thread_engagement - 1e-6
    assert screws.zmax <= z0 + spec.tie_thread_depth + 1e-6
    assert solid.tie_nuts(spec) is None
    # a wall with the room takes the size the hole asks for
    assert _ring("nema17-printed-15", housing_wall=9.0).tie_bolt_size.name == "M4"


def test_no_screw_s_counterbore_fitting_is_an_error():
    from cycloidgen.core.validate import Severity

    spec = _ring(21, housing_wall=2.5)
    assert spec.tie_bolt_size is None
    found = [f for f in analyse(spec).report.findings
             if f.code == "HOUSING_BOLT_CLASH"]
    assert found and found[0].severity is Severity.ERROR


def test_a_carrier_drive_keeps_its_nuts():
    """Its input plate is the motor's face and stands still: nothing to tap for."""
    spec = preset(21)
    spec.end_plate_thickness = 12.0
    assert not spec.tie_screws_tapped


def test_the_ring_drive_is_the_length_its_parts_add_up_to():
    """The sleeve's foot to the housing's closed face, worked out by hand -
    the motor's length in the sleeve, then the base's plate and up - against
    the assembled solids' own extent."""
    from cycloidgen.core.spec import BASE_RUNNING_GAP, CUP_CEILING
    from cycloidgen.export import solid

    spec = _ring("nema17-printed-15")
    by_hand = (spec.motor_face.body_length
               + spec.plate_thickness + BASE_RUNNING_GAP + spec.housing_plate_thickness
               + spec.barrel_height + spec.housing_plate_thickness + CUP_CEILING)
    assert spec.envelope_length == pytest.approx(by_hand)
    assy = solid.build_assembly(spec)
    boxes = [child.obj.val().moved(child.loc).BoundingBox()
             for child in assy.children
             if child.name in ("housing", "output_flange", "output_end_plate")]
    z_lo = min(b.zmin for b in boxes)
    z_hi = max(b.zmax for b in boxes)
    assert z_hi - z_lo == pytest.approx(spec.envelope_length, abs=1e-6)


# --------------------------------------------------- the gap above the base


def test_a_ring_drive_s_base_sits_a_running_gap_under_the_plate_that_turns():
    """Read off the assembled preview: the lowest point of anything that turns -
    the plate under the housing, the screws let into it - against the top of
    the base, which does not."""
    from cycloidgen.core.spec import BASE_RUNNING_GAP, COUPLING_GRIP
    from cycloidgen.viz import build_mesh

    spec = preset(21)
    spec.output_member = OutputMember.RING
    mesh = build_mesh(spec)
    world = mesh.world_vertices(0.0)
    parts = {p.name: p for p in mesh.parts}
    turning = min(world[parts[n].vertices][:, 2].min()
                  for n in ("output_end_plate", "tie_bolts"))
    flange = world[parts["output_flange"].vertices]
    base = flange[np.hypot(flange[:, 0], flange[:, 1]) > spec.hub_diameter / 2.0 + 1.0]
    plate_bottom = spec.barrel_bottom - spec.housing_plate_thickness
    base_top = base[base[:, 2] < plate_bottom][:, 2].max()
    assert turning - base_top == pytest.approx(BASE_RUNNING_GAP, abs=1e-6)
    # Off the carrier it is still the grip, and a stated length is kept.
    assert preset(21).boss_protrusion == COUPLING_GRIP
    spec.output_boss_protrusion = 12.0
    assert spec.boss_protrusion == 12.0


# ------------------------------------------- the base: a flange and a pad


def _ring(key_or_ratio, **over):
    from cycloidgen.core.presets import by_key

    spec = (by_key(key_or_ratio).spec() if isinstance(key_or_ratio, str)
            else preset(key_or_ratio))
    spec.output_member = OutputMember.RING
    for name, value in over.items():
        setattr(spec, name, value)
    return spec


def _inserted(**over):
    """The printed ring drive with heat-set inserts, square, so the turning
    plate has the room for them as well as the base."""
    from cycloidgen.core.spec import HousingShape

    return _ring("nema17-printed-15", **{"heat_set_inserts": True,
                                         "housing_shape": HousingShape.SQUARE, **over})


BASES = {
    "printed NEMA 17": lambda: _ring("nema17-printed-15"),
    "printed, heat-set inserts": _inserted,
    "machined NEMA 23": lambda: _ring("nema23-machined-21"),
    "NEMA 34": lambda: _ring("nema34-machined-29"),
    "no motor, a whole plate": lambda: _ring(21),
}


@pytest.mark.parametrize("name", BASES)
def test_the_base_weighs_what_its_solid_encloses(name):
    """The flange, the pad, the screw holes tapped up into it and the motor's
    register and seats: the mass model and the STEP solid are one part, to a
    percent - the preview is held to the same in ``test_viz.py``."""
    from cycloidgen.analysis.bearings import pin_diameters
    from cycloidgen.export import solid

    spec = BASES[name]()
    body = solid.output_flange(spec).val()
    assert body.isValid() and len(body.Solids()) == 1
    _, out_d = pin_diameters(spec)
    pins = spec.output_pin_count * np.pi * (out_d / 2.0) ** 2 * spec.output_pin_length
    weighed = analyse(spec).mass.flange_mass_g / spec.housing_mat.density_g_cm3 * 1000.0
    assert weighed + pins == pytest.approx(body.Volume(), rel=0.01)


@pytest.mark.parametrize("name", BASES)
def test_the_base_is_bolted_down_from_below_into_blind_threads(name):
    """Probed in the solid: inside each hole there is no metal, and over its
    end there is - the screws come up from the machine's side and stop under
    the face the turning housing runs over."""
    import cadquery as cq

    from cycloidgen.core.spec import TAPPED_COVER
    from cycloidgen.export import solid

    spec = BASES[name]()
    assert len(spec.base_mount_centres) == 4
    body = solid.output_flange(spec).val()
    if spec.base_wraps_motor:
        _probe_sleeve_mounts(spec, body)
        return
    top = spec.boss_bottom + CARRIER_DROP
    floor = top - spec.base_flange_thickness
    for x, y in spec.base_mount_centres:
        assert not body.isInside(cq.Vector(x, y, floor + spec.base_mount_depth / 2.0))
        assert body.isInside(cq.Vector(x, y, top - TAPPED_COVER / 2.0))
        # the hole as wide as the drill or the insert, and a wall either side
        # of it, inside the outline and clear of the pad
        hole = spec.base_mount_hole / 2.0
        assert not body.isInside(cq.Vector(x + hole - 0.2, y, floor + 0.5))
        assert body.isInside(cq.Vector(x + hole + 0.5, y, floor + 0.5))


def _probe_sleeve_mounts(spec, body):
    """Up into the sleeve's corners from its foot: no metal in each hole, metal
    over its end, and the wall either side of it along its diagonal."""
    import cadquery as cq

    from cycloidgen.core.spec import SLEEVE_HOLE_WALL

    foot = spec.sleeve_bottom + CARRIER_DROP
    hole = spec.base_mount_hole / 2.0
    for x, y in spec.base_mount_centres:
        r = math.hypot(x, y)
        ux, uy = x / r, y / r
        assert not body.isInside(cq.Vector(x, y, foot + spec.base_mount_depth / 2.0))
        assert body.isInside(cq.Vector(x, y, foot + spec.base_mount_depth + 0.5))
        for side in (1.0, -1.0):
            d = side * (hole - 0.2)
            assert not body.isInside(cq.Vector(x + ux * d, y + uy * d, foot + 1.0))
            d = side * (hole + SLEEVE_HOLE_WALL - 0.2)
            assert body.isInside(cq.Vector(x + ux * d, y + uy * d, foot + 1.0))


def test_the_flange_is_thinner_than_the_pad_under_it():
    """With no motor to wrap, the base is still a flange the housing's size on
    a pad the boss's - the flange only as thick as its screws need."""
    spec = _ring(21, end_plate_thickness=12.0)
    assert spec.base_has_flange and not spec.base_wraps_motor
    assert spec.base_flange_thickness < spec.plate_thickness
    assert spec.base_pad == ("round", spec.hub_diameter)


@pytest.mark.parametrize("key", ["nema17-printed-15", "nema23-machined-21",
                                 "nema34-machined-29"])
def test_a_ring_drive_s_base_wraps_its_motor(key):
    """The base comes down round the motor's body to its back, with air round
    the body and a wall past it, a slot through its +x flat for the cable, and
    the drive stands on its foot - probed in the solid; and the motor the view
    draws is inside it, clear of it."""
    import cadquery as cq

    from cycloidgen.core.spec import SLEEVE_CLEARANCE
    from cycloidgen.export import solid
    from cycloidgen.export.bom import bom_items
    from cycloidgen.viz import build_mesh

    spec = _ring(key)
    assert spec.base_wraps_motor and not spec.base_has_flange
    body = solid.output_flange(spec).val()
    assert body.isValid() and len(body.Solids()) == 1
    frame = spec.motor_face
    h = frame.body_width / 2.0
    foot = spec.sleeve_bottom + CARRIER_DROP
    mid = foot + frame.body_length / 2.0
    wall = spec.sleeve_half_width - h - SLEEVE_CLEARANCE
    for x, y in ((-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
        assert not body.isInside(cq.Vector(x * (h + SLEEVE_CLEARANCE / 2.0),
                                           y * (h + SLEEVE_CLEARANCE / 2.0), mid))
        assert body.isInside(cq.Vector(x * (h + SLEEVE_CLEARANCE + wall / 2.0),
                                       y * (h + SLEEVE_CLEARANCE + wall / 2.0), mid))
    width, height = spec.cable_slot
    x = h + SLEEVE_CLEARANCE + wall / 2.0
    assert not body.isInside(cq.Vector(x, 0.0, foot + height / 2.0))
    assert body.isInside(cq.Vector(x, 0.0, foot + height + 0.5))
    assert body.isInside(cq.Vector(x, width / 2.0 + 1.0, foot + height / 2.0))
    _probe_sleeve_mounts(spec, body)

    mesh = build_mesh(spec)
    motor = next(p for p in mesh.parts if p.name == "motor")
    v = mesh.vertices[motor.vertices]
    assert v[:, 2].min() == pytest.approx(spec.sleeve_bottom)
    assert np.abs(v[:, :2]).max() <= h + 1e-6
    a = analyse(spec)
    assert (sum(i.mass_total_g for i in bom_items(a))
            == pytest.approx(a.mass.total_mass_g, rel=1e-6))


def test_through_a_coupling_the_base_stays_a_whole_plate():
    spec = _ring(21, motor_frame="NEMA 17", motor_drives_the_shaft=False)
    assert not spec.base_has_flange and not spec.base_mount_centres
    assert spec.base_flange_thickness == spec.plate_thickness


# ---- heat-set inserts ------------------------------------------------------------


def test_heat_set_inserts_go_wherever_a_screw_threads_into_the_print():
    """Up into the housing's underside for the plate's screws, down into its
    face for the driven machine's, up into the base for its own - probed in the
    solids at the insert's hole, with the wall round it, on a round 6 mm
    housing."""
    import cadquery as cq

    from cycloidgen.core.spec import INSERT_WALL
    from cycloidgen.export import solid

    spec = _ring("nema17-printed-15", heat_set_inserts=True)
    assert spec.tie_insert.thread == "M3" and spec.output_insert.thread == "M4"
    assert spec.base_insert.thread == "M4"
    assert spec.tie_thread_engagement == 1.5 * spec.tie_bolt_size.d
    housing = solid.ring_housing(spec).val()

    def probe(centres, hole, z):
        for x, y in centres:
            r = math.hypot(x, y)
            ux, uy = x / r, y / r
            assert not housing.isInside(cq.Vector(x + ux * (hole / 2.0 - 0.2),
                                                  y + uy * (hole / 2.0 - 0.2), z))
            assert housing.isInside(cq.Vector(x + ux * (hole / 2.0 + INSERT_WALL - 0.2),
                                              y + uy * (hole / 2.0 + INSERT_WALL - 0.2),
                                              z))

    probe(spec.tie_bolt_centres, spec.tie_insert.hole, spec.barrel_bottom + 1.0)
    probe(spec.output_face_bolt_centres, spec.output_insert.hole, spec.cup_top - 1.0)
    assert not any(f.code == "INSERTS_DO_NOT_FIT" for f in analyse(spec).report.findings)


def test_the_inserts_holes_weigh_what_the_solids_lose_to_them():
    """The same drive with and without inserts: what the housing, the plate and
    the base lose to the wider holes and gain in the thicker flange, in the
    mass model and in the solids - the difference, so a percent of the part
    cannot hide it."""
    from cycloidgen.export import solid

    with_ = _ring("nema17-printed-15", heat_set_inserts=True)
    without = _ring("nema17-printed-15")
    # The sleeve's corners grow for the inserts' wider holes.
    assert with_.sleeve_half_width > without.sleeve_half_width

    def weighed(spec):
        m = analyse(spec).mass
        return {"housing": m.housing_mass_g, "plates": m.plates_mass_g,
                "base": m.flange_mass_g}

    def enclosed(spec):
        made = solid.parts(spec)
        return {"housing": made["housing"].val().Volume(),
                "plates": made["output_end_plate"].val().Volume(),
                "base": solid.output_flange(spec).val().Volume()}

    rho = with_.housing_mat.density_g_cm3 / 1000.0
    w, wo = weighed(with_), weighed(without)
    e, eo = enclosed(with_), enclosed(without)
    for part in ("housing", "base"):
        assert (w[part] - wo[part]) / rho == pytest.approx(e[part] - eo[part],
                                                           rel=1e-3), part
    # the plate's holes are the screws' clearance either way
    assert w["plates"] == pytest.approx(wo["plates"])


def test_the_bill_orders_the_inserts_the_mass_model_weighs():
    """A line per insert size, each insert a brass tube by the CAD kernel's own
    volume; and the bill still adds up to the assembled mass."""
    import cadquery as cq

    from cycloidgen.core.fasteners import BRASS_DENSITY, size_named
    from cycloidgen.export.bom import bom_items

    spec = _ring("nema17-printed-15", heat_set_inserts=True)
    a = analyse(spec)
    items = bom_items(a)
    lines = {i.part: i for i in items if i.part.startswith("Heat-set insert")}
    small, large = lines["Heat-set insert, M3"], lines["Heat-set insert, M4"]
    assert small.quantity == len(spec.tie_bolt_centres)
    assert large.quantity == (len(spec.output_face_bolt_centres)
                              + len(spec.base_mount_centres))
    for line, insert in ((small, spec.tie_insert), (large, spec.output_insert)):
        tube = (cq.Workplane("XY").circle(insert.outer / 2.0)
                .circle(size_named(insert.thread).d / 2.0)
                .extrude(insert.length).val())
        assert line.mass_each_g == pytest.approx(
            tube.Volume() * BRASS_DENSITY / 1000.0, rel=1e-6)
    assert "heat-set inserts" in next(i for i in items if i.part == "Plate screw").note
    assert (sum(i.mass_total_g for i in items)
            == pytest.approx(a.mass.total_mass_g, rel=1e-6))


def test_face_inserts_too_deep_for_the_top_say_what_wall_would_take_them():
    """An M6 insert is longer than the top, so its holes go down the middle of
    the wall, which a 6 mm one has no room for there: the face is tapped as
    before, and the wall the check names is then built and holds."""
    import re

    spec = _ring("nema17-printed-15", heat_set_inserts=True,
                 output_bolt_diameter=6.6)
    assert spec.output_bolt_size.name == "M6" and spec.output_insert is None
    found = next(f for f in analyse(spec).report.findings
                 if f.code == "INSERTS_DO_NOT_FIT")
    assert "face the driven machine bolts to" in found.message
    wall = float(re.search(r"a ([\d.]+) mm housing wall", found.message).group(1))
    assert _ring("nema17-printed-15", heat_set_inserts=True, output_bolt_diameter=6.6,
                 housing_wall=wall).output_insert is not None
    assert _ring("nema17-printed-15", heat_set_inserts=True, output_bolt_diameter=6.6,
                 housing_wall=wall - 0.5).output_insert is None


def test_a_metal_drive_ignores_the_inserts():
    """Brass in aluminium is not what the option is for: a metal part is tapped,
    and nothing it weighs or says moves."""
    plain = _ring("nema23-machined-21")
    asked = _ring("nema23-machined-21", heat_set_inserts=True)
    assert asked.tie_insert_wanted is None and asked.base_insert is None
    assert asked.output_insert is None
    a, b = analyse(plain), analyse(asked)
    assert b.mass.total_mass_g == a.mass.total_mass_g
    assert [f.code for f in b.report.findings] == [f.code for f in a.report.findings]


def test_the_carrier_output_drive_has_no_thread_in_plastic_to_insert():
    """Off the carrier the tie screws take nuts and nothing is threaded into
    the print, so asking for inserts changes nothing."""
    plain = by_key_spec("nema17-printed-15")
    asked = by_key_spec("nema17-printed-15", heat_set_inserts=True)
    assert asked.tie_insert is None and asked.base_insert is None
    assert asked.output_insert is None
    assert analyse(asked).mass.total_mass_g == analyse(plain).mass.total_mass_g


def by_key_spec(key, **over):
    from cycloidgen.core.presets import by_key

    spec = by_key(key).spec()
    for name, value in over.items():
        setattr(spec, name, value)
    return spec


def test_the_plate_s_sheet_and_cutting_file_state_the_counterbores_the_solid_has():
    """The plate under a ring drive's housing: its sheet calls for the screws'
    counterbores from the outside, its DXF states them, and the size they give
    is the one in the solid - probed a hair inside and outside at every centre
    the sheet gives, at the counterbore's depth and past it."""
    import tempfile
    from pathlib import Path

    import cadquery as cq
    import ezdxf

    from cycloidgen.export import sheets, solid
    from cycloidgen.export.dxf import write_part_dxfs
    from cycloidgen.export.manifest import sheet_names

    spec = _ring("nema17-printed-15")
    assert sheet_names(spec) == ["output_end_plate"]
    f = sheets.plate_features(spec, "output_end_plate")
    screws = next(p for p in f.patterns if p.label == "plate screws")
    kind, width, depth = screws.recess
    assert (kind, width, depth) == ("counterbore", *spec.tie_counterbore)
    assert screws.recess_outside
    lines = sheets._pattern_lines(screws, f)
    assert f"C'BORE Ø{sheets._fmt(width)} x {sheets._fmt(depth)} DEEP FROM THE OUTSIDE" \
        in lines
    plate = solid.parts(spec)["output_end_plate"].val()
    for x, y in screws.centres:
        r = math.hypot(x, y)
        for radius, z, metal in ((width / 2.0 - 0.1, depth - 0.2, False),
                                 (width / 2.0 + 0.1, depth - 0.2, True),
                                 (screws.hole / 2.0 - 0.1, depth + 0.2, False),
                                 (screws.hole / 2.0 + 0.1, depth + 0.2, True)):
            point = cq.Vector(x + x / r * radius, y + y / r * radius, z)
            assert plate.isInside(point) is metal
    with tempfile.TemporaryDirectory() as tmp:
        written = {p.name for p in write_part_dxfs(spec, tmp)}
        assert "input_end_plate.dxf" not in written
        doc = ezdxf.readfile(Path(tmp) / "output_end_plate.dxf")
        note = next(t.dxf.text for t in doc.modelspace().query("TEXT"))
    assert f"c'bore {width:g} x {depth:g} deep from the outside" in note
