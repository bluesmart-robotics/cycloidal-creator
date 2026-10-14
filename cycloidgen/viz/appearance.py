"""What each part looks like in the 3D view: its material's finish, not a paint.

The quick mesh and the STEP file colour parts by what they *do* - pins orange,
discs blue - which is right for a diagram and for telling parts apart in a CAD
package's tree.  It is not what the gearbox looks like.  A steel pin is steel,
an aluminium housing is aluminium, and the difference between a printed disc
and a ground one is mostly how light comes off it.

So the hardware view shades physically: every material in
:data:`~cycloidgen.core.spec.MATERIALS` has a finish here - metallic or not, and
how rough.  A metal takes its own colour, because that is what a metal is.  A
printed or moulded part keeps its part colour, because filament and resin come
in any colour and the one in the diagram is as good a guess as any; it takes
only its surface from the material - PA12 chalky, PETG glossy, resin smooth.

No Qt and no VTK here, so the table is checked without a display.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..core.spec import GearSpec
from .mesh import Part

__all__ = ["BEARING_MATERIAL", "FINISHES", "MOTOR_FINISH", "Finish", "finish",
           "material_of", "studio"]


@dataclass(frozen=True)
class Finish:
    """How a surface returns light, in the terms a PBR shader takes."""

    #: The metal's own colour, sRGB; ``None`` for a part that keeps its part
    #: colour - anything printed or moulded.
    colour: tuple[int, int, int] | None
    #: 1 for a metal, 0 for everything else.  Nothing in between is a material.
    metallic: float
    #: 0 is a mirror, 1 is chalk.
    roughness: float


#: Metal colours are their reflectance at normal incidence, the number a metal
#: is characterised by in PBR, brought into sRGB.  Roughness is the finish the
#: part is made with here: a turned steel pin, a milled aluminium housing, a
#: ground and hardened disc, a polished bearing race.
FINISHES: dict[str, Finish] = {
    "PLA": Finish(None, 0.0, 0.52),
    "PETG": Finish(None, 0.0, 0.32),
    "ABS": Finish(None, 0.0, 0.48),
    "PA12 (SLS)": Finish(None, 0.0, 0.88),
    "Tough Resin (SLA)": Finish(None, 0.0, 0.28),
    "POM (Delrin)": Finish(None, 0.0, 0.40),
    "Aluminium 7075-T6": Finish((222, 224, 226), 1.0, 0.40),
    "Bronze CuSn12": Finish((214, 168, 112), 1.0, 0.32),
    "Steel 1045": Finish((180, 182, 186), 1.0, 0.38),
    "Steel 4140 (hardened)": Finish((118, 121, 127), 1.0, 0.28),
    "Bearing steel 100Cr6": Finish((200, 202, 206), 1.0, 0.24),
}

#: What a bearing is made of, whatever the drive's own materials are.
BEARING_MATERIAL = "Bearing steel 100Cr6"

#: A stepper's body: black paint over the stack, which is not one of the
#: drive's materials and so is not in the table.  Its shaft is steel.
MOTOR_FINISH = Finish((44, 46, 50), 0.0, 0.42)
MOTOR_SHAFT_MATERIAL = "Steel 1045"


def material_of(spec: GearSpec, part: Part) -> str:
    """The material a part is made of - the same choice the bill orders it in."""
    group = part.group
    if group == "bearings":
        return BEARING_MATERIAL
    if group == "motor":
        return MOTOR_SHAFT_MATERIAL
    if group == "discs" or part.name == "washers":
        return spec.disc_material
    if group == "shaft":
        return spec.shaft_material
    if group == "ring_pins":
        return spec.housing_material if spec.ring_pins_integral else spec.pin_material
    if group == "fasteners":
        return spec.pin_material
    return spec.housing_material            # housing, end plates, carrier


def finish(spec: GearSpec, part: Part) -> tuple[tuple[int, int, int], Finish]:
    """The colour to shade ``part`` in, and its finish."""
    if part.name == "motor":
        return MOTOR_FINISH.colour, MOTOR_FINISH
    surface = FINISHES[material_of(spec, part)]
    return (surface.colour or part.colour), surface


def _soft(x: np.ndarray, edge: float) -> np.ndarray:
    """1 inside ``|x| < 1``, 0 outside, with a smooth step ``edge`` wide."""
    t = np.clip((1.0 + edge - np.abs(x)) / (2.0 * edge), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


#: The lights in the studio, as (azimuth, elevation, half-width, half-height,
#: brightness) - degrees, and multiples of the room's own brightness.  A large
#: key above and to one side, a weaker fill opposite, and a tall narrow strip
#: behind: the three a product photograph is lit with, and what a polished
#: metal needs something to reflect, or it renders as a black hole.
_SOFTBOXES = (
    (35.0, 50.0, 30.0, 16.0, 1.9),
    (215.0, 25.0, 26.0, 14.0, 0.9),
    (305.0, 10.0, 6.0, 30.0, 1.5),
)


def studio(width: int = 512, height: int = 256) -> np.ndarray:
    """An equirectangular light probe of a photographic studio, linear RGB.

    Made here rather than shipped as an HDR file: it is a few gradients and
    four rectangles, a file would be a megabyte in the repository and in every
    bundle, and nothing photographed would be neutral enough.  Columns are
    azimuth from 0 to 360 degrees, rows elevation from +90 to -90, Z up.

    Grey on purpose.  A coloured environment tints every metal it lights, and
    the one thing a steel part must not look is blue.
    """
    az = np.linspace(0.0, 360.0, width, endpoint=False)[None, :]
    el = np.linspace(90.0, -90.0, height)[:, None]
    up = np.sin(np.radians(el))
    # A room: a bright ceiling, walls falling off towards a darker floor.
    room = np.where(up >= 0.0, 0.32 + 0.18 * up, 0.16 + 0.10 * up)
    light = np.broadcast_to(room, (height, width)).copy()
    for a, e, w, h, gain in _SOFTBOXES:
        d_az = (az - a + 180.0) % 360.0 - 180.0
        light += gain * _soft(d_az / w, 0.25) * _soft((el - e) / h, 0.25)
    return np.repeat(light[:, :, None], 3, axis=2).astype(np.float32)
