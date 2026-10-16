"""The dimensioned drawings: what they state, held to the part they state it of."""
from __future__ import annotations

import itertools
import math
from itertools import pairwise

import ezdxf
import numpy as np
import pytest

from cycloidgen.analysis import analyse
from cycloidgen.analysis.bearings import seat_bores
from cycloidgen.core.spec import OutputMember, Process, preset
from cycloidgen.export import manifest, sheets
from cycloidgen.export.bom import bom_items
from cycloidgen.export.dxf import write_part_dxfs

PLATES = ("input_end_plate", "output_end_plate")


def _design(kind: str):
    spec = preset(21)
    if kind != "printed":
        spec.process = Process.CNC
        spec.apply_process_defaults()
        spec.housing_material = "Aluminium 7075-T6"
    if kind == "ring":
        spec.output_member = OutputMember.RING
    if kind.startswith("nema17"):
        spec.motor_frame = "NEMA 17"
    if kind == "nema17_coupled":
        spec.motor_drives_the_shaft = False
    if kind == "nema17_thick":
        # Thick enough for a register and a lip in the same plate.
        spec.end_plate_thickness = 11.0
    if kind == "nema23":
        spec.motor_frame = "NEMA 23"
    return spec


KINDS = ("printed", "machined", "ring", "nema17", "nema17_coupled", "nema17_thick",
         "nema23")


# ------------------------------------------------------------- against the model


@pytest.mark.parametrize("kind", KINDS)
def test_every_seat_is_the_bearing_the_solid_is_bored_to(kind):
    spec = _design(kind)
    bores = seat_bores(spec)
    for name, bore in zip(PLATES, (bores.input_plate, bores.output_plate), strict=True):
        f = sheets.plate_features(spec, name)
        assert f.seat is not None
        assert f.seat.diameter == pytest.approx(bore)
        lip = next((s for s in bores.shoulders if s.host == name), None)
        assert (f.lip is None) == (lip is None)
        if lip is not None:
            assert f.through == pytest.approx(lip.bore)
            assert f.lip[1] - f.lip[0] == pytest.approx(lip.thickness)
            assert f.seat.depth == pytest.approx(f.lip[0])


@pytest.mark.parametrize("kind", ["machined", "ring", "nema17", "nema23"])
def test_a_machined_seat_in_a_plate_is_held_by_interference(kind):
    """Every load inside the drive turns with the crank, so the outer ring in
    either plate always has its load turning round it: an M or N fit, never the
    sliding H that a stationary load would take."""
    spec = _design(kind)
    for name in PLATES:
        fit = sheets.plate_features(spec, name).seat.fit
        assert fit is not None and fit.tolerance_class in ("M7", "N7")
        assert fit.upper_um <= 0


def test_a_printed_seat_claims_no_fit():
    for name in PLATES:
        f = sheets.plate_features(_design("printed"), name)
        assert f.printed and f.seat.fit is None


def test_the_patterns_are_the_ones_the_plate_is_drilled_with():
    spec = _design("nema17")
    f = sheets.plate_features(spec, "input_end_plate")
    by = {p.label: p for p in f.patterns}
    tie = by["tie bolts"]
    assert len(tie.centres) == spec.housing_bolt_count
    assert tie.circle == pytest.approx(2.0 * spec.housing_bolt_radius)
    assert tie.hole == spec.housing_bolt_diameter
    motor = by["NEMA 17 motor"]
    assert sorted(motor.centres) == sorted(spec.motor_bolt_centres)
    assert motor.square == spec.motor_face.bolt_span
    assert motor.recess[0] == "counterbore" and motor.datums == ("A", "C")
    assert f.register.diameter == spec.motor_face.pilot_diameter
    # Nothing motor-shaped on the far plate.
    out = sheets.plate_features(spec, "output_end_plate")
    assert [p.label for p in out.patterns] == ["tie bolts"]
    assert out.register is None


# --------------------------------------------------------- position tolerances


def _common_circle_fits(hole: float, offset: float, screw: float) -> bool:
    """Whether a screw of ``screw`` passes two holes of ``hole`` whose axes are
    ``offset`` apart - by sampling the screw's outline at the best place for it,
    halfway between, rather than by the formula under test."""
    a = np.linspace(0.0, 2.0 * math.pi, 721)
    x, y = offset / 2.0 + screw / 2.0 * np.cos(a), screw / 2.0 * np.sin(a)
    inside_one = np.hypot(x, y) <= hole / 2.0 + 1e-9
    inside_two = np.hypot(x - offset, y) <= hole / 2.0 + 1e-9
    return bool(np.all(inside_one & inside_two))


@pytest.mark.parametrize("kind", ["machined", "ring", "nema17", "nema23"])
def test_a_position_tolerance_still_lets_every_screw_through(kind):
    """Each hole anywhere in its zone, the worst pair a whole zone apart: the
    screw still goes through both - and a hundredth more and it would not.  For
    a screw held in a thread at one end the thread's axis is off by as much
    again, so the plate's own hole has to clear it from twice its share."""
    from cycloidgen.core.fasteners import size_for_hole

    spec = _design(kind)
    for name in PLATES:
        for p in sheets.plate_features(spec, name).patterns:
            screw = size_for_hole(p.hole).d
            offset = 2.0 * p.position if p.fixed else p.position
            assert _common_circle_fits(p.hole, offset, screw), p.label
            assert not _common_circle_fits(p.hole, offset + 0.01, screw), p.label


# ---------------------------------------------------------------- against the solid


def _solid(spec, name):
    pytest.importorskip("cadquery")
    from cycloidgen.export.solid import housing_end_plate

    bores = seat_bores(spec)
    bore = bores.input_plate if name == "input_end_plate" else bores.output_plate
    return housing_end_plate(spec, bore, motor_face=name == "input_end_plate",
                             shoulders=bores.shoulders).val()


def _probes(f):
    """Points to ask about, ``(z, r)``: inside every cell of the drawn section,
    and a hair either side of every line it draws - so a line drawn a quarter
    of a millimetre from where the solid's surface is has a point between them.

    Edge breaks and inside radii are stated in a note, not drawn, and they only
    change a corner: so no point goes nearer a corner than the break.
    """
    zs, rs = sheets._breaks(f)
    corner = f.edge_break + 0.1
    zm = [(a + b) / 2.0 for a, b in pairwise(zs)]
    rm = [(a + b) / 2.0 for a, b in pairwise(rs)]
    points = {(z0 + fz * (z1 - z0), r0 + fr * (r1 - r0))
              for z0, z1 in pairwise(zs) for r0, r1 in pairwise(rs)
              for fz, fr in itertools.product((0.1, 0.5, 0.9), repeat=2)}
    points |= {(z, b + d) for z in zm for b in rs for d in (-0.15, 0.15)}
    points |= {(b + d, r) for r in rm for b in zs for d in (-0.15, 0.15)}
    for z, r in sorted(points):
        if r < 0.0:
            continue
        dz = min(abs(z - b) for b in zs)
        dr = min(abs(r - b) for b in rs)
        if min(dz, dr) < 0.05 or max(dz, dr) < corner:
            continue
        yield z, r


def _disagreements(f, solid, name):
    import cadquery as cq

    out = []
    for z, r in _probes(f):
        local = z if name == "input_end_plate" else f.thickness - z
        for side in (1.0, -1.0):
            drawn = sheets._material(f, z, r, side)
            made = solid.isInside(cq.Vector(side * r, 0.0, local), 1e-6)
            if drawn != made:
                out.append((z, r, side, drawn))
    return out


@pytest.mark.parametrize("kind", KINDS)
@pytest.mark.parametrize("name", PLATES)
def test_the_section_drawn_is_the_section_of_the_solid(kind, name):
    """The drawing's section is built from the features it dimensions; the
    solid from the parts' own construction.  Asked point by point whether there
    is metal there, the two have to agree - which is what says a seat drawn 6.5
    deep is bored 6.5 deep, and that a register, a lip, a counterbore or a mount
    the solid has is not missing from the sheet."""
    spec = _design(kind)
    f = sheets.plate_features(spec, name)
    assert _disagreements(f, _solid(spec, name), name) == []


@pytest.mark.parametrize("change", ["seat deeper", "seat wider", "lip wider",
                                    "plate thicker"])
def test_the_section_check_sees_a_quarter_millimetre(change):
    """The check above, pointed at a drawing that is wrong on purpose: every
    one of these moves a line by a quarter of a millimetre, and each has to
    show up as metal where the solid has none or the other way round."""
    import dataclasses

    spec = _design("nema17_thick")
    name = "input_end_plate"
    f = sheets.plate_features(spec, name)
    wrong = {
        "seat deeper": dataclasses.replace(
            f, seat=dataclasses.replace(f.seat, depth=f.seat.depth + 0.25)),
        "seat wider": dataclasses.replace(
            f, seat=dataclasses.replace(f.seat, diameter=f.seat.diameter + 0.5)),
        "lip wider": dataclasses.replace(f, through=f.through + 0.5),
        "plate thicker": dataclasses.replace(f, thickness=f.thickness + 0.25),
    }[change]
    assert _disagreements(wrong, _solid(spec, name), name)


def test_the_cutting_file_states_the_depth_the_sheet_does():
    """A register in the outside face takes its depth off the far end of the
    seat's room. The plate's DXF used to quote the seat as the plate less the
    lip - 10 deep in an 11 mm plate where it is bored 8, which goes through the
    lip it is meant to stop short of."""
    spec = _design("nema17_thick")
    f = sheets.plate_features(spec, "input_end_plate")
    assert f.register is not None and f.lip is not None
    import tempfile
    from pathlib import Path

    with tempfile.TemporaryDirectory() as tmp:
        write_part_dxfs(spec, tmp)
        doc = ezdxf.readfile(Path(tmp) / "input_end_plate.dxf")
        note = next(t.dxf.text for t in doc.modelspace().query("TEXT"))
    assert f"x {f.seat.depth:g} deep from the inside" in note


# -------------------------------------------------------------------- the sheet


@pytest.mark.parametrize("kind", ["machined", "nema17_coupled", "ring"])
def test_a_sheet_weighs_the_plate_the_bill_does(kind):
    spec = _design(kind)
    lines = {item.part: item for item in bom_items(analyse(spec))}
    for name in PLATES:
        sheet = sheets.plate_sheet(spec, name)
        line = next(item for item in lines.values() if name in item.assembly)
        assert sheet.mass_g == pytest.approx(line.mass_each_g)


def test_the_sheets_are_written_where_the_manifest_says(tmp_path):
    spec = _design("nema17_coupled")
    written = sheets.write_sheets(spec, tmp_path)
    assert [p.name for p in written] == [f"{n}.pdf" for n in manifest.sheet_names(spec)]
    for p in written:
        assert p.read_bytes().startswith(b"%PDF")
