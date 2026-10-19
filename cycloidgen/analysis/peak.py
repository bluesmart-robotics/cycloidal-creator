"""What the drive survives for a moment.

The torque capacity is a rating: the load the ring contact carries for the
thousands of hours behind a bearing life, against an allowable chosen for that -
under the endurance of steel, and for a polymer well under what it takes, because
plastic creeps.  A drive also meets loads it does not run at: a hard stop at the
end of a move, a crash, the motor stalled against something that will not give.
What matters then is not the life used but whether anything is bent.

So this is the torque at which the first part starts to yield.  Every load path
in the drive is carried by the output torque, so each one is worked out at the
rated torque and scaled to the torque that takes it to its yield; the smallest
of those is the momentary torque, and the part it belongs to is what goes first.

* **The ring contact and the output pins in their holes.**  A line contact first
  yields below its surface, where the shear stress peaks at 0.30 of the peak
  pressure, so it starts when the peak pressure reaches 1.67 times the softer
  body's yield strength (Johnson, *Contact Mechanics*, 1985, sec. 4.2, with
  Tresca).  For a polymer the material table holds the tensile strength where
  a metal has its yield, and that stands in - lower than the compressive yield
  that governs under a contact, so on the safe side.  Pressure goes as the root
  of the load, so the torque as the square of the pressure.  The ring contact is
  read with the pins sharing it as they do at that torque, as the capacity is;
  the output pins on their ideal share, since their holes' clearance is not
  solved for load sharing.
* **The output pins in bending,** to the pin material's yield.
* **The disc web beside the output holes,** in shear, to 0.577 of the disc
  material's yield.
* **Every bearing the torque loads** - the cam bearings, the shaft supports and
  any pin rollers - to its basic static rating C0, past which the rolling
  elements dent their races.  The main output bearing carries whatever the
  machine hangs on the output, not the torque, and is not in it.

Not in it: the input shaft and whatever couples it to the motor, the tie bolts
and the housing, which do not carry the torque in a way this model sizes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..core.spec import GearSpec
from .bearings import BearingChoice
from .efficiency import EfficiencyResult
from .fatigue import output_pin_fatigue
from .mass import MassResult
from .mechanics import ContactResult

__all__ = ["CONTACT_YIELD_ONSET", "PeakLimit", "PeakResult", "analyse_peak",
           "ring_contact_yield_rigid_Nm"]

#: Peak line-contact pressure at the first yield below the surface, as a
#: multiple of the softer body's yield strength (Johnson 1985, sec. 4.2: the
#: subsurface shear peak is 0.300 p0, against Tresca's Y / 2).
CONTACT_YIELD_ONSET = 1.67

#: The bearings the output torque does not load.
_NOT_TORQUE_LOADED = frozenset({"Main output bearing"})


@dataclass(frozen=True)
class PeakLimit:
    """One load path, and the output torque that takes it to its yield."""

    part: str
    torque_Nm: float
    #: What gives, said as the stress against its limit at that torque.
    how: str


@dataclass(frozen=True)
class PeakResult:
    """The momentary torque, every load path behind it, and the motor's."""

    #: Smallest first.
    limits: tuple[PeakLimit, ...]
    #: What the motor puts through the ratio at standstill and full current:
    #: the torque a stall against a hard stop applies.  ``inf`` with no motor
    #: stated, and zero with no curve.
    motor_Nm: float

    @property
    def torque_Nm(self) -> float:
        """The output torque at which the first part starts to yield."""
        return self.limits[0].torque_Nm if self.limits else math.inf

    @property
    def first(self) -> PeakLimit | None:
        """What yields first."""
        return self.limits[0] if self.limits else None

    @property
    def motor_can_bend_it(self) -> bool:
        return math.isfinite(self.motor_Nm) and self.motor_Nm > self.torque_Nm


def _softer_yield(spec: GearSpec) -> tuple[float, str]:
    """The yield strength of the softer of pin and disc, and its name."""
    pin, disc = spec.pin_mat, spec.disc_mat
    softer = pin if pin.sigma_yield_MPa < disc.sigma_yield_MPa else disc
    return softer.sigma_yield_MPa, softer.name


def ring_contact_yield_rigid_Nm(spec: GearSpec, contact: ContactResult) -> float:
    """The output torque at which the ring contact first yields on the
    rigid-disc share-out - before clearance concentrates it, which is what
    :func:`~cycloidgen.analysis.stiffness.analyse_stiffness` takes it on to."""
    if contact.max_pin_pressure_MPa <= 0.0:
        return math.inf
    onset = CONTACT_YIELD_ONSET * _softer_yield(spec)[0]
    return spec.output_torque_Nm * (onset / contact.max_pin_pressure_MPa) ** 2


def analyse_peak(spec: GearSpec, contact: ContactResult, ring_concentration: float,
                 mass: MassResult, bearings: list[BearingChoice],
                 efficiency: EfficiencyResult, temperature_C: float) -> PeakResult:
    """The momentary torque.  ``ring_concentration`` is the ring contact's load
    concentration at the torque it yields at, off the stiffness solve."""
    rated = spec.output_torque_Nm
    limits: list[PeakLimit] = []
    y_contact, softer = _softer_yield(spec)
    onset = CONTACT_YIELD_ONSET * y_contact

    rigid = ring_contact_yield_rigid_Nm(spec, contact)
    if math.isfinite(rigid):
        limits.append(PeakLimit(
            "ring contact", rigid / max(ring_concentration, 1.0),
            f"{onset:.0f} MPa peak pressure, 1.67x the {softer}'s {y_contact:g} MPa"))
    if contact.max_output_pressure_MPa > 0.0:
        limits.append(PeakLimit(
            "output pins in their holes",
            rated * (onset / contact.max_output_pressure_MPa) ** 2,
            f"{onset:.0f} MPa peak pressure, 1.67x the {softer}'s {y_contact:g} MPa"))

    bending = output_pin_fatigue(spec, temperature_C).alternating_MPa
    if bending > 0.0:
        pin = spec.pin_mat
        limits.append(PeakLimit(
            "output pins in bending", rated * pin.sigma_yield_MPa / bending,
            f"{pin.sigma_yield_MPa:g} MPa, the {pin.name}'s yield"))

    if mass.web_shear_MPa > 0.0:
        limits.append(PeakLimit(
            "disc web", rated * mass.web_shear_allow_MPa / mass.web_shear_MPa,
            f"{mass.web_shear_allow_MPa:.0f} MPa in shear, 0.577x the "
            f"{spec.disc_mat.name}'s yield"))

    for choice in bearings:
        b = choice.bearing
        if (b is None or not choice.fitted or choice.carried_elsewhere
                or choice.role in _NOT_TORQUE_LOADED or choice.load_N <= 0.0):
            continue
        limits.append(PeakLimit(
            f"{choice.role.lower()} ({b.designation})",
            rated * 1000.0 * b.C0_kN / choice.load_N,
            f"its static rating, {b.C0_kN:g} kN"))

    curve = spec.motor_curve
    if not curve.modelled:
        motor = math.inf
    else:
        motor = curve.continuous_torque_at(0.0) * spec.ratio * efficiency.efficiency
    return PeakResult(limits=tuple(sorted(limits, key=lambda lim: lim.torque_Nm)),
                      motor_Nm=motor)
