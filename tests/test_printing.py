"""Printed parts come out ready to print: face down, standing, broken back."""
from __future__ import annotations

import struct

import numpy as np
import pytest

from cycloidgen.analysis import analyse
from cycloidgen.core.spec import FIRST_LAYER_CHAMFER, Process, preset
from cycloidgen.export import printing, solid


def _stl_points(path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    rows = np.frombuffer(data[84:84 + 50 * count], dtype=np.dtype(
        [("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rows["v"].reshape(-1, 3)


@pytest.fixture(scope="module")
def stls(tmp_path_factory):
    spec = preset(21)
    out = tmp_path_factory.mktemp("stl")
    return spec, {p.stem: p for p in solid.write_stls(spec, out)}


def test_a_printed_part_stands_on_the_bed(stls):
    """Every printed STL has its lowest point at z = 0 and its axis at the
    origin - including the washers, which assemble one above the other and are
    put down side by side instead of printed floating."""
    spec, paths = stls
    for name, path in paths.items():
        if not printing.is_printed(spec, name):
            continue
        pts = _stl_points(path)
        assert pts[:, 2].min() == pytest.approx(0.0, abs=1e-4), name
        assert abs(pts[:, 0].max() + pts[:, 0].min()) < 1e-3, name
    washers = _stl_points(paths["washers"])
    assert washers[:, 2].max() == pytest.approx(spec.disc_gap, abs=1e-3)


def test_a_part_made_some_other_way_is_left_as_it_is(stls):
    """A steel shaft is turned, not printed, so its file is the part as drawn."""
    spec, paths = stls
    assert not printing.is_printed(spec, "eccentric_shaft")
    pts = _stl_points(paths["eccentric_shaft"])
    assert pts[:, 2].min() == pytest.approx(-spec.shaft_overhang, abs=1e-3)


def test_a_part_goes_down_on_the_face_that_needs_no_support():
    """The end cap's boss stands up off its plate: plate down needs nothing
    held up; boss down would hang the whole plate in the air.  And the input
    plate goes down on its outside, where the shoulder ring is, because that
    face has more on the bed.  A ring drive's housing goes down on its closed
    top, which is a whole face; open end down it would print its top over the
    bore as a bridge."""
    spec = preset(21)
    assert printing.face_down(solid.parts(spec)["input_end_plate"].val())
    spec.output_member = "ring housing"
    assert not printing.face_down(solid.end_cap(spec).val())
    assert printing.face_down(solid.parts(spec)["housing"].val())


def test_fdm_breaks_the_bed_edge_and_nothing_else_does():
    """The first layer is squashed on FDM alone; and only a printed part is
    printed."""
    fdm = preset(21)
    assert fdm.process is Process.FDM
    assert fdm.first_layer_chamfer == FIRST_LAYER_CHAMFER[Process.FDM] > 0.0
    disc = solid.parts(fdm)["disc_1"].val()
    broken = printing.with_first_layer(fdm, "disc_1", disc)
    assert broken.isValid() and broken.Volume() < disc.Volume() - 1.0

    sla = preset(21)
    sla.process = Process.SLA
    assert sla.first_layer_chamfer == 0.0
    assert printing.with_first_layer(sla, "disc_1", disc) is disc

    steel = preset(21)
    steel.disc_material = "Steel 4140 (hardened)"
    assert printing.with_first_layer(steel, "disc_1", disc) is disc


def test_the_break_is_on_the_face_that_goes_down_only():
    """The top of a printed disc keeps its full-height flank; the bottom loses
    the chamfer, so every flank face now stops short of the bed face."""
    spec = preset(21)
    disc = solid.parts(spec)["disc_1"].val()
    broken = printing.with_first_layer(spec, "disc_1", disc)
    flanks = [f for f in broken.Faces() if f.geomType() == "EXTRUSION"]
    c = spec.first_layer_chamfer
    for face in flanks:
        verts, _ = face.tessellate(0.05, 0.2)
        zs = [v.z for v in verts]
        assert min(zs) == pytest.approx(c, abs=1e-3)
        assert max(zs) == pytest.approx(spec.disc_thickness, abs=1e-3)


def test_the_contact_is_shorter_by_the_break_on_a_printed_disc():
    """The break takes the bottom of every line contact on the flank and in the
    output holes, so the analysis counts that much less of it - and the
    contact stress rises as the root of the lengths, which is the whole of the
    change between two drives that differ only in it."""
    fdm = preset(21)
    assert fdm.contact_length == pytest.approx(fdm.disc_thickness - 0.3)
    same = preset(21)
    same.process = Process.SLA            # clearances left where FDM put them
    assert same.contact_length == same.disc_thickness

    p_fdm = analyse(fdm).contact.max_pin_pressure_MPa
    p_same = analyse(same).contact.max_pin_pressure_MPa
    ratio = (fdm.disc_thickness / fdm.contact_length) ** 0.5
    assert p_fdm == pytest.approx(p_same * ratio, rel=1e-6)


def test_a_cusp_the_kernel_cannot_chamfer_costs_its_edges_not_the_part():
    """Ring pins formed with the housing meet its bore in sharp points, and
    OCCT cannot run a chamfer into one.  The rim still gets its break, the
    cusped edges go without, and the part is written."""
    spec = preset(21)
    spec.ring_pins_integral = True
    housing = solid.parts(spec)["housing"].val()
    broken = printing.with_first_layer(spec, "housing", housing)
    assert broken.isValid()
    assert broken.Volume() < housing.Volume() - 1.0
