"""A square housing: its outline, its corner bolts, and what reads them.

Every closed form here is checked against the outline as drawn - a polygon of
the points the mesh and the 2D view are built from - and every hole against
that same outline, so the formulas and the parts cannot drift apart.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from cycloidgen.analysis import analyse
from cycloidgen.core.presets import PRESETS
from cycloidgen.core.spec import HousingShape, OutputMember, preset


def square(ratio: int = 21, **over):
    spec = preset(ratio)
    spec.housing_shape = HousingShape.SQUARE
    for name, value in over.items():
        setattr(spec, name, value)
    return spec


def _polygon(points) -> tuple[float, float, float]:
    """Area, polar second moment about the origin and perimeter of a closed
    polygon, by the shoelace sums."""
    p = np.asarray(points, float)
    q = np.roll(p, -1, axis=0)
    cross = p[:, 0] * q[:, 1] - q[:, 0] * p[:, 1]
    ix = (cross * (p[:, 1] ** 2 + p[:, 1] * q[:, 1] + q[:, 1] ** 2)).sum() / 12.0
    iy = (cross * (p[:, 0] ** 2 + p[:, 0] * q[:, 0] + q[:, 0] ** 2)).sum() / 12.0
    return cross.sum() / 2.0, ix + iy, float(np.hypot(*(q - p).T).sum())


def _to_outline(spec, x: float, y: float) -> float:
    """How far a point inside the outline is from it, off the drawn points."""
    p = np.asarray(spec.housing_outline(4000), float)
    q = np.roll(p, -1, axis=0)
    d = q - p
    t = np.clip(((x - p[:, 0]) * d[:, 0] + (y - p[:, 1]) * d[:, 1])
                / (d ** 2).sum(axis=1), 0.0, 1.0)
    return float(np.hypot(p[:, 0] + t * d[:, 0] - x, p[:, 1] + t * d[:, 1] - y).min())


# ------------------------------------------------------------------- outline


@pytest.mark.parametrize("shape", list(HousingShape))
@pytest.mark.parametrize("ratio,wall", [(21, 6.0), (59, 6.0), (15, 11.0)])
def test_the_outline_s_area_moment_and_perimeter_are_the_drawn_outline_s(shape, ratio,
                                                                         wall):
    spec = preset(ratio)
    spec.housing_shape = shape
    spec.housing_wall = wall
    area, polar, perimeter = _polygon(spec.housing_outline(40000))
    assert spec.housing_section_area == pytest.approx(area, rel=1e-6)
    assert spec.housing_polar_area_moment == pytest.approx(polar, rel=1e-6)
    assert spec.housing_perimeter == pytest.approx(perimeter, rel=1e-6)


def test_a_square_is_the_round_one_s_size_across_its_flats():
    """Same metal past the pin pockets at the thinnest place: the flats."""
    rnd, sq = preset(21), square()
    assert sq.housing_outer_radius == rnd.housing_outer_radius
    points = np.asarray(sq.housing_outline(400))
    assert np.abs(points).max() == pytest.approx(sq.housing_outer_radius)
    assert sq.housing_corner_radius == sq.housing_wall
    assert sq.housing_section_area > rnd.housing_section_area
    assert sq.cooling_area_mm2 == pytest.approx(
        sq.housing_perimeter * sq.envelope_length + 2.0 * sq.housing_section_area)
    assert sq.cooling_area_mm2 > rnd.cooling_area_mm2


# ---------------------------------------------------------------- tie bolts


def test_the_tie_bolts_go_in_the_corners_whatever_the_count_says():
    for count in (3, 6, 8):
        spec = square(housing_bolt_count=count)
        ties = spec.tie_bolt_centres
        assert len(ties) == 4
        c = spec.housing_outer_radius - spec.housing_corner_radius
        assert sorted(ties) == sorted((sx * c, sy * c) for sx in (-1, 1) for sy in (-1, 1))
    assert square(housing_bolt_count=0).tie_bolt_centres == []


@pytest.mark.parametrize("bolt", [3.4, 4.5, 5.5, 9.0])
def test_a_corner_bolt_has_the_rim_land_all_round_it(bolt):
    """Off the drawn outline rather than the formula: the arc and both flats
    are each the rim land from the hole, and the pin pockets further."""
    spec = square(housing_bolt_diameter=bolt)
    x, y = spec.tie_bolt_centres[0]
    assert _to_outline(spec, x, y) - bolt / 2.0 == pytest.approx(spec.rim_land, abs=1e-4)
    pockets = spec.pin_circle_radius + spec.pin_radius
    assert math.hypot(x, y) - pockets - bolt / 2.0 > spec.rim_land


def test_a_square_wall_takes_a_bolt_a_round_one_cannot():
    """The round wall's bolt has half the wall either side of it; the corner's
    has a whole wall, so the clash moves from one wall to two."""
    def clash(spec):
        return any(f.code == "HOUSING_BOLT_CLASH" for f in analyse(spec).report.findings)

    big = 8.0                                    # over a 6 mm wall, under 12
    rnd = preset(21)
    rnd.housing_bolt_diameter = big
    assert clash(rnd)
    assert not clash(square(housing_bolt_diameter=big))
    assert clash(square(housing_bolt_diameter=12.5))


@pytest.mark.parametrize("square_housing", [False, True])
@pytest.mark.parametrize("count", [4, 5, 6, 7, 8, 12])
def test_the_output_bolts_meet_no_screw_whatever_the_count(count, square_housing):
    """They used to share a plate with the tie bolts, so a count could put one
    through the other.  They are in the housing's closed top now, and the
    screws are under it."""
    spec = square() if square_housing else preset(21)
    spec.output_member = OutputMember.RING
    spec.output_bolt_count = count
    assert not [f for f in analyse(spec).report.findings
                if f.code == "OUTPUT_BOLT_CLASH"]


@pytest.mark.parametrize("ring", [False, True])
@pytest.mark.parametrize("key", [p.key for p in PRESETS if not p.key.startswith("ratio-")])
def test_every_hole_in_a_square_input_plate_has_metal_round_it(key, ring):
    """Tie bolts, dowels, output bolts and motor screws, each against all the
    others and against the drawn outline.  The outline is held to what the
    round drive gives a hole on its wall's middle circle - the rest of the wall
    either side - and a corner bolt to its rim land."""
    spec = next(p for p in PRESETS if p.key == key).spec()
    spec.housing_shape = HousingShape.SQUARE
    if ring:
        spec.output_member = OutputMember.RING
    holes = [(x, y, d) for x, y, d, _ in spec.input_plate_holes]
    holes += [(x, y, spec.plate_dowel_hole) for x, y in spec.plate_dowel_centres]
    for i, (x, y, d) in enumerate(holes):
        least = (spec.rim_land if (x, y) in spec.tie_bolt_centres
                 else (spec.housing_wall - d) / 2.0)
        assert _to_outline(spec, x, y) - d / 2.0 >= least - 1e-4, (x, y)
        for u, v, e in holes[i + 1:]:
            assert math.hypot(x - u, y - v) - (d + e) / 2.0 >= 1.0, ((x, y), (u, v))


@pytest.mark.parametrize("ring", [False, True])
@pytest.mark.parametrize("motor", ["None", "NEMA 17", "NEMA 23"])
def test_a_square_plate_goes_on_one_way_only(ring, motor):
    """Every way of turning or flipping the plate that puts its bolts back on
    the barrel's must move a dowel off its hole."""
    spec = square(motor_frame=motor)
    if ring:
        spec.output_member = OutputMember.RING
    ties = np.asarray(spec.tie_bolt_centres)
    dowels = np.asarray(spec.plate_dowel_centres)
    assert len(dowels) == 2

    def same(a, b):
        return all(np.hypot(*(b - p).T).min() < 1e-6 for p in a)

    moves = []
    for k in range(4):
        c, s = math.cos(k * math.pi / 2.0), math.sin(k * math.pi / 2.0)
        moves.append(np.array([[c, -s], [s, c]]))
        c2, s2 = math.cos(k * math.pi / 2.0), math.sin(k * math.pi / 2.0)
        moves.append(np.array([[c2, s2], [s2, -c2]]))       # mirror at k * 45 deg
    for m in moves[1:]:                                      # all but leaving it be
        assert same(ties @ m.T, ties)
        moved = dowels @ m.T
        # Off by more than the dowel: it would not go in.
        assert np.hypot(*(moved[:, None, :] - dowels[None, :, :]).T).min(axis=0).max() \
            > spec.plate_dowel_diameter


def test_a_square_ring_drive_starts_its_output_bolts_on_a_flat():
    spec = square(output_member=OutputMember.RING)
    assert spec.output_bolt_phase == 0.0
    x, y, _, what = next(h for h in spec.input_plate_holes if h[3] == "an output bolt")
    assert (x, y) == pytest.approx((spec.output_face_bolt_radius, 0.0))
    assert what == "an output bolt"


# ------------------------------------------------------------- drawings


def test_the_dxf_outline_is_the_square_with_true_arcs(tmp_path):
    import ezdxf
    from ezdxf import path as dxfpath

    from cycloidgen.export import dxf

    spec = square()
    files = {f.stem: f for f in dxf.write_part_dxfs(spec, tmp_path)}
    for name in ("input_end_plate", "output_end_plate", "ring_plate"):
        msp = ezdxf.readfile(files[name]).modelspace()
        outline = [e for e in msp if e.dxf.layer == "HOUSING"
                   and e.dxftype() != "TEXT" and e.dxftype() != "MTEXT"]
        assert [e.dxftype() for e in outline] == ["LWPOLYLINE"], name
        assert outline[0].closed and outline[0].has_arc
        points = [(v.x, v.y) for v in dxfpath.make_path(outline[0]).flattening(0.0005)]
        assert _polygon(points)[0] == pytest.approx(spec.housing_section_area, rel=1e-5)
    bolts = ezdxf.readfile(files["input_end_plate"]).modelspace().query(
        'CIRCLE[layer=="BOLTS"]')
    assert sorted((round(c.dxf.center.x, 6), round(c.dxf.center.y, 6)) for c in bolts) \
        == sorted((round(x, 6), round(y, 6)) for x, y in spec.tie_bolt_centres)


def test_the_svg_draws_the_square():
    from cycloidgen.export.svg import svg_document

    spec = square()
    text = svg_document(spec)
    r = spec.housing_outer_radius
    assert f'<rect x="{-r:.4f}" y="{-r:.4f}" width="{2 * r:.4f}"' in text
    assert f'rx="{spec.housing_corner_radius:.4f}"' in text


@pytest.mark.parametrize("name", ["input_end_plate", "output_end_plate"])
def test_a_square_plate_s_sheet_states_the_square(name, tmp_path):
    from cycloidgen.export.sheets import plate_features, plate_sheet

    spec = square(motor_frame="NEMA 17")
    f = plate_features(spec, name)
    assert f.corner_radius == spec.housing_corner_radius
    assert f.outside_text == f"{spec.housing_outer_radius * 2:g} A/F"
    ties = next(p for p in f.patterns if p.label == "tie bolts")
    c = spec.housing_outer_radius - spec.housing_corner_radius
    assert ties.circle is None and ties.square == pytest.approx(2.0 * c)
    assert len(ties.centres) == 4
    sheet = plate_sheet(spec, name)
    assert any("corners R" in n for n in sheet.notes)
    assert "square" in sheet.design


def test_the_bill_names_the_square_and_counts_four_tie_bolts():
    from cycloidgen.export.bom import bom_items

    spec = square()
    items = {i.part: i for i in bom_items(analyse(spec))}
    assert items["Ring housing"].size.startswith(f"{2 * spec.housing_outer_radius:.1f} square")
    assert items["Tie bolt"].quantity == 4
    assert "corners of a" in items["Tie bolt"].note


def test_the_2d_view_turns_a_square_housing_with_the_ring():
    """A circle reads the same turned; a square does not, so on a ring-output
    drive it goes round with the pins."""
    from matplotlib.figure import Figure

    from cycloidgen.report import plots

    spec = square(output_member=OutputMember.RING)
    view = plots.ProfileView(Figure())
    view.set_design(spec)
    housing, outline, _ = view._squares[0]
    view.set_crank(40.0)
    a = spec.frame_spin * math.radians(40.0)
    turned = outline @ np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
    assert np.allclose(housing.get_xy(), turned)
    assert not np.allclose(housing.get_xy(), outline)


# ---------------------------------------------------------- against solids


@pytest.mark.parametrize("ring_output,motor,direct", [
    (False, "None", True), (True, "None", True), (False, "NEMA 17", True),
    (True, "NEMA 23", False)])
def test_a_square_part_weighs_what_its_solid_encloses(ring_output, motor, direct):
    """The same two percent as the round drive's parts, from the corners'
    true arcs in the solid to the closed forms in the mass model."""
    from cycloidgen.export import solid

    spec = square(motor_frame=motor, motor_drives_the_shaft=direct)
    if ring_output:
        spec.output_member = OutputMember.RING
    m = analyse(spec).mass
    parts = solid.parts(spec)
    density = spec.housing_mat.density_g_cm3
    expected = {("housing",): m.housing_mass_g,
                ("input_end_plate", "output_end_plate"): m.plates_mass_g}
    if ring_output:
        expected[("end_cap",)] = m.end_cap_mass_g
    for names, grams in expected.items():
        names = tuple(n for n in names if n in parts)
        enclosed = sum(parts[n].val().Volume() for n in names)
        assert grams / density * 1000.0 == pytest.approx(enclosed, rel=0.02), names
        assert all(parts[n].val().isValid() for n in names), names
    # The base the ring drive stands on is the square too.
    if ring_output:
        from cycloidgen.analysis.bearings import pin_diameters

        _, out_d = pin_diameters(spec)
        pins = spec.output_pin_count * np.pi * (out_d / 2.0) ** 2 * spec.output_pin_length
        assert m.flange_mass_g / density * 1000.0 + pins == pytest.approx(
            parts["output_flange"].val().Volume(), rel=0.02)


def test_a_square_ring_turns_with_the_inertia_of_its_solids():
    import cadquery as cq

    from cycloidgen.export import solid

    spec = square(output_member=OutputMember.RING)
    spec.pin_material = spec.housing_material
    m = analyse(spec).mass
    parts = {**solid.parts(spec), "tie_bolts": solid.tie_bolts(spec)}
    assert solid.tie_nuts(spec) is None and "input_end_plate" not in parts
    turning = ["housing", "output_end_plate", "ring_pins", "tie_bolts"]
    occt = sum(cq.Shape.matrixOfInertia(parts[n].val())[2][2] for n in turning) \
        * spec.housing_mat.density_g_cm3 * 1e-6
    assert m.output_inertia_kg_mm2 == pytest.approx(occt, rel=0.02)
