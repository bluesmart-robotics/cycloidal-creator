"""An outrunner: a drum on a stator base, driving through its can."""
from __future__ import annotations

import numpy as np
import pytest

from cycloidgen.analysis import analyse
from cycloidgen.core.motor import MOTOR_FRAMES, ROTOR_HUB_RIM, ROTOR_HUB_THICKNESS
from cycloidgen.core.presets import by_key
from cycloidgen.export.bom import bom_items

KEY = "outrunner-8108-machined-15"


def test_the_can_drives_a_hub_the_shaft_runs_through_to():
    spec = by_key(KEY).spec()
    frame = spec.motor_face
    assert frame.rotor_driven and not frame.spigot and frame.round_body
    z0, z1, outer, bore = spec.rotor_hub
    # On the can's face, the far end of the motor from its base.
    assert z0 == pytest.approx(spec.motor_face_z + frame.body_length)
    assert z1 - z0 == pytest.approx(ROTOR_HUB_THICKNESS)
    screw, _length = spec.rotor_hub_screw
    assert outer == pytest.approx(frame.rotor_bolt_span + screw.clearance
                                  + 2.0 * ROTOR_HUB_RIM)
    assert bore == pytest.approx(spec.input_shaft_diameter + spec.hole_clearance)
    for x, y in spec.rotor_hub_bolt_centres:
        assert np.hypot(x, y) == pytest.approx(frame.rotor_bolt_span / 2.0)
    # The shaft reaches through the motor to the hub's far face, is not bored
    # for a shaft the motor does not have, and carries the hub's flat.
    assert spec.shaft_span[1] == pytest.approx(z1)
    assert spec.motor_bore is None
    assert spec.input_flat[:2] == pytest.approx((z0, z1))
    # And no register is cut for a spigot that is not there.
    assert spec.motor_face.pilot_diameter <= spec.input_plate_bore


def test_nothing_asks_how_far_a_shaft_it_does_not_have_goes_in():
    a = analyse(by_key(KEY).spec())
    codes = {f.code for f in a.report.findings}
    assert not codes & {"MOTOR_SHAFT_SHORT", "MOTOR_SHAFT_MISMATCH",
                        "MOTOR_SHAFT_TOO_BIG", "MOTOR_HOLLOW_TOO_SMALL"}


def test_a_shaft_too_thick_for_the_hollow_centre_cannot_be_built():
    spec = by_key(KEY).spec()
    spec.input_shaft_diameter = MOTOR_FRAMES["8108 outrunner"].through_bore
    found = next(f for f in analyse(spec).report.findings
                 if f.code == "MOTOR_HOLLOW_TOO_SMALL")
    assert found.severity.value == "error"
    assert found.value > found.limit


def test_the_bill_orders_the_hub_and_its_screws():
    spec = by_key(KEY).spec()
    a = analyse(spec)
    lines = {i.part: i for i in bom_items(a)}
    assert lines["Rotor hub"].mass_each_g == pytest.approx(a.mass.rotor_hub_mass_g)
    assert lines["Rotor hub screw"].quantity == len(spec.rotor_hub_bolt_centres)
    total = sum(i.quantity * i.mass_each_g for i in lines.values())
    assert total == pytest.approx(a.mass.total_mass_g, rel=1e-9)


def test_the_hub_is_drawn_and_weighed_as_the_solid_is():
    """The quick mesh, the solid and the mass model: three descriptions of the
    hub, held to one another, and the shaft that now runs through the motor
    held to its solid the same way."""
    pytest.importorskip("cadquery")
    from cycloidgen.export import solid
    from cycloidgen.viz.mesh import build_mesh

    spec = by_key(KEY).spec()
    hub = solid.rotor_hub(spec).val().Volume()
    m = analyse(spec).mass
    assert m.rotor_hub_mass_g / spec.shaft_mat.density_g_cm3 * 1000.0 == pytest.approx(
        hub, rel=1e-6)
    from tests.test_viz import _part_volumes

    mesh = build_mesh(spec)
    index = next(i for i, p in enumerate(mesh.parts) if p.name == "rotor_hub")
    assert _part_volumes(mesh)[index] == pytest.approx(hub, rel=0.01)
    shaft = solid.parts(spec)["eccentric_shaft"].val().Volume()
    assert m.shaft_mass_g / spec.shaft_mat.density_g_cm3 * 1000.0 == pytest.approx(
        shaft, rel=0.02)
