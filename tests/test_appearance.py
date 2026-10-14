"""What the hardware view shades each part as: its material's finish.

No display needed - the table and the studio are plain data, which is the
point of keeping them out of the widget.
"""
from __future__ import annotations

import numpy as np
import pytest

from cycloidgen.core.spec import MATERIALS, OutputMember, preset
from cycloidgen.viz import appearance
from cycloidgen.viz.mesh import mesh_for_spec


def test_every_material_has_a_finish_and_nothing_else_does():
    assert set(appearance.FINISHES) == set(MATERIALS)


@pytest.mark.parametrize("name", sorted(MATERIALS))
def test_a_metal_has_its_own_colour_and_nothing_else_does(name):
    """Metallic is a property of the material, not a slider: 1 or 0.  A metal
    is drawn in its own colour; a printed or moulded part keeps its part's."""
    surface = appearance.FINISHES[name]
    metal = surface.metallic == 1.0
    assert surface.metallic in (0.0, 1.0)
    assert (surface.colour is not None) is metal
    assert 0.0 < surface.roughness < 1.0


def test_each_part_is_shaded_in_the_material_it_is_ordered_in():
    spec = preset(21)
    spec.disc_material = "Steel 4140 (hardened)"
    spec.pin_material = "Bearing steel 100Cr6"
    spec.housing_material = "Aluminium 7075-T6"
    spec.shaft_material = "Steel 1045"
    expected = {
        "housing": spec.housing_material, "input_end_plate": spec.housing_material,
        "output_flange": spec.housing_material, "ring_pins": spec.pin_material,
        "disc_1": spec.disc_material, "washers": spec.disc_material,
        "eccentric_shaft": spec.shaft_material,
        "bearing_cam_1": appearance.BEARING_MATERIAL,
        "tie_bolts": spec.pin_material,
    }
    parts = {p.name: p for p in mesh_for_spec(spec).parts}
    for name, material in expected.items():
        assert appearance.material_of(spec, parts[name]) == material, name


def test_integral_pins_are_the_housing():
    spec = preset(21)
    spec.ring_pins_integral = True
    spec.housing_material = "PETG"
    part = next(p for p in mesh_for_spec(spec).parts if p.group == "housing")
    assert appearance.material_of(spec, part) == "PETG"


def test_a_printed_part_keeps_its_colour_and_a_metal_one_does_not():
    spec = preset(21)                                   # PLA discs, steel pins
    spec.output_member = OutputMember.CARRIER
    parts = {p.name: p for p in mesh_for_spec(spec).parts}
    disc_rgb, _ = appearance.finish(spec, parts["disc_1"])
    pin_rgb, pin = appearance.finish(spec, parts["ring_pins"])
    assert disc_rgb == parts["disc_1"].colour
    assert pin_rgb == pin.colour != parts["ring_pins"].colour


def test_the_studio_is_grey_lit_from_above_and_has_lights_in_it():
    """Grey, or every metal comes out tinted; brighter above than below, or
    a part looks lit from the floor; and with something brighter than the room,
    or a polished face has nothing to show."""
    probe = appearance.studio(256, 128)
    assert probe.shape == (128, 256, 3) and probe.dtype == np.float32
    assert (probe > 0.0).all()
    assert np.array_equal(probe[..., 0], probe[..., 1])
    assert np.array_equal(probe[..., 1], probe[..., 2])
    ceiling, floor = probe[:16].mean(), probe[-16:].mean()
    assert ceiling > floor
    assert probe.max() > 3.0 * np.median(probe)
