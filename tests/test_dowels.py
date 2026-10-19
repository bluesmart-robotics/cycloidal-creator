"""The dowels that put the end plates on the ring's axis."""
from __future__ import annotations

import math

import pytest

from cycloidgen.analysis import analyse
from cycloidgen.analysis.mass import plate_dowel_mass_g
from cycloidgen.core.spec import OutputMember, Process, preset
from cycloidgen.export.bom import bom_items


def _codes(spec):
    return {f.code: f for f in analyse(spec).report.findings}


@pytest.mark.parametrize("count", range(1, 13))
def test_a_plate_goes_on_one_way_only(count):
    """The plate and the barrel are symmetric under a turn of a tie-bolt pitch
    and under a flip across a bolt or between two; the dowels have to break
    every one of those, or a plate can go on turned or upside down with its
    seats somewhere the barrel's pin circle is not."""
    spec = preset(21)
    spec.housing_bolt_count = count
    angles = [math.degrees(a) for a in spec.plate_dowel_angles]
    assert len(angles) == 2
    pitch = 360.0 / count

    def same(a, b):
        key = sorted(round(x % 360.0, 6) % 360.0 for x in a)
        return key == sorted(round(x % 360.0, 6) % 360.0 for x in b)

    for k in range(1, count):
        assert not same(angles, [a + k * pitch for a in angles]), ("turn", k)
    for m in range(2 * count):
        axis = m * pitch / 2.0
        assert not same(angles, [2.0 * axis - a for a in angles]), ("flip", axis)


@pytest.mark.parametrize("ratio", [10, 15, 21, 29, 39, 59])
def test_a_dowel_hole_leaves_metal_all_round(ratio):
    """On the middle of the wall, a quarter of the wall either side of it at
    least - clear of the pin pockets inside and the rim outside - and a
    millimetre clear of every bolt hole in the plate."""
    spec = preset(ratio)
    d = spec.plate_dowel_diameter
    assert d > 0.0
    inside = spec.housing_bolt_radius - d / 2.0 - (spec.pin_circle_radius
                                                     + spec.pin_radius)
    outside = spec.housing_outer_radius - (spec.housing_bolt_radius + d / 2.0)
    assert min(inside, outside) >= spec.housing_wall / 4.0 - 1e-9
    for a in spec.plate_dowel_angles:
        assert spec.plate_dowel_metal(a)[0] >= 1.0


def test_the_dowels_keep_off_the_motor_pattern():
    """A NEMA 34's corner sits at 135 degrees, where the second dowel would
    have gone on a 10:1 - so it goes somewhere else."""
    spec = preset(10)
    spec.motor_frame = "NEMA 34"
    spec.input_shaft_diameter = 17.0
    assert "PLATE_DOWEL_CLASH" not in _codes(spec)
    for a in spec.plate_dowel_angles:
        assert spec.plate_dowel_metal(a)[0] >= 1.0


def test_without_dowels_nothing_locates_the_plates():
    spec = preset(21)
    assert "PLATES_NOT_LOCATED" not in _codes(spec)
    spec.plate_dowels = False
    found = _codes(spec)["PLATES_NOT_LOCATED"]
    # An M4 through 4.5 mm in the plate and in the barrel: the whole clearance.
    assert found.value == pytest.approx(0.5)
    assert found.limit == pytest.approx(spec.profile_clearance)
    assert not spec.plate_dowel_centres and not spec.plate_dowel_spans


def test_each_dowel_reaches_into_its_plate_and_no_further():
    spec = preset(21)
    into_plate, into_barrel = spec.plate_dowel_engagement
    (lo_in, hi_in), (lo_out, hi_out) = spec.plate_dowel_spans
    length = spec.plate_dowel_length
    assert hi_in - lo_in == hi_out - lo_out == length
    # Short of the outside faces, and both ends' dowels clear of each other.
    assert hi_in == pytest.approx(spec.barrel_top + spec.plate_thickness - 0.5)
    assert lo_out == pytest.approx(spec.barrel_bottom - spec.plate_thickness + 0.5)
    assert lo_in - spec.barrel_top < 0.0 < hi_out - spec.barrel_bottom
    assert lo_in > hi_out
    assert length >= into_plate + into_barrel


def test_a_printed_plate_lets_its_dowel_slide():
    spec = preset(21)
    assert spec.printed and spec.plate_dowel_hole == pytest.approx(
        spec.plate_dowel_diameter + spec.hole_clearance)
    spec.process = Process.CNC
    spec.apply_process_defaults()
    assert spec.plate_dowel_hole == spec.plate_dowel_diameter


@pytest.mark.parametrize("member", [OutputMember.CARRIER, OutputMember.RING])
def test_the_bill_orders_the_dowels_it_weighs(member):
    spec = preset(21)
    spec.output_member = member
    a = analyse(spec)
    line = next(i for i in bom_items(a) if i.part == "Plate dowel")
    # Two at each end, or under a ring drive's housing only: its top is the
    # housing itself.
    assert line.quantity == (2 if spec.ground_frame_fitted else 4)
    assert line.assembly == ("plate_dowels",)
    assert "ISO 8734" in line.size and f"x {spec.plate_dowel_length:g}" in line.size
    assert line.mass_each_g == pytest.approx(plate_dowel_mass_g(spec))
    spec.plate_dowels = False
    assert not any(i.part == "Plate dowel" for i in bom_items(analyse(spec)))


def test_the_solids_have_the_holes_and_the_dowels_fill_them():
    pytest.importorskip("cadquery")
    import cadquery as cq

    from cycloidgen.analysis.bearings import seat_bores
    from cycloidgen.core.spec import MATERIALS
    from cycloidgen.export.solid import housing_end_plate, plate_dowels, ring_housing

    spec = preset(21)
    spec.process = Process.CNC
    spec.apply_process_defaults()
    bores = seat_bores(spec)
    barrel = ring_housing(spec).val()
    plates = {
        "input": housing_end_plate(spec, bores.input_plate, motor_face=True,
                                   shoulders=bores.shoulders).val(),
        "output": housing_end_plate(spec, bores.output_plate,
                                    shoulders=bores.shoulders).val(),
    }
    z_mid = (spec.barrel_bottom + spec.barrel_top) / 2.0
    for x, y in spec.plate_dowel_centres:
        assert not barrel.isInside(cq.Vector(x, y, z_mid))
        for plate in plates.values():
            assert not plate.isInside(cq.Vector(x, y, spec.plate_thickness / 2.0))
        # and the metal right beside the hole is there
        r = spec.plate_dowel_diameter / 2.0 + 0.5
        assert barrel.isInside(cq.Vector(x * (1 + r / math.hypot(x, y)),
                                          y * (1 + r / math.hypot(x, y)), z_mid))
    dowels = plate_dowels(spec).val()
    steel = MATERIALS["Bearing steel 100Cr6"].density_g_cm3
    assert dowels.Volume() * steel / 1000.0 == pytest.approx(
        4 * plate_dowel_mass_g(spec), rel=1e-6)
