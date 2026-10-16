"""Named starting points: the ratio presets, and whole drives built round a motor.

:func:`~cycloidgen.core.spec.preset` is geometry for a reduction and nothing
else - a drive with no motor named, at the default duty, in the default
material.  That is a place to start drawing, and it is what the six ratio
presets here still are.

A motor preset is a drive.  What turns it, from a datasheet; what it is made
of and how; how hard and how fast it is asked to work - and the parts round the
motor that make it buildable: plates thick enough for the motor's register to
leave the bearing a lip and its screws a full thread, a cam turned to the cam
bearing's bore.  The duty is the drive's, not the motor's: a printed drive is
rated at what its plastic carries and its motor turns it with plenty to spare,
a machined one at what its motor can turn with the margin the motor check asks
for.

``tests/test_presets.py`` holds every motor preset to analysing with no error,
no warning about its motor, its mounting or its bearings, a capacity over its
rated torque and its motor's margin.  The warnings it does carry are the ones
its ratio preset carries - its pressure angle, its load sharing - which are the
geometry's, not the motor's.

The motor figures are typical of the class rather than one maker's part: a
48 mm NEMA 17, a 112 mm NEMA 23, a 114 mm NEMA 34, as their datasheets give
them to within a few percent of each other; the outrunner is one maker's 8108,
and its continuous current is worked out from its rated power.  Check yours.
"""
from __future__ import annotations

from dataclasses import dataclass

from .spec import GearSpec, MotorKind, Process, preset

__all__ = ["PRESETS", "Preset", "by_key", "matching"]


@dataclass(frozen=True)
class Preset:
    """One entry in the preset list."""

    #: Stable, for the command line and the settings file: ``nema17-printed-15``.
    key: str
    #: What the list shows, whole, since a closed list shows one line of it:
    #: ``NEMA 17 - 15:1, printed``.
    title: str
    group: str
    #: One line on what it is for.
    summary: str
    ratio: int
    #: Fields set over :func:`preset`, in order.  The process first, because
    #: its clearances are defaults the rest may override.
    settings: tuple[tuple[str, object], ...] = ()

    def spec(self) -> GearSpec:
        """A fresh design, every time: a preset is a starting point."""
        spec = preset(self.ratio)
        settings = dict(self.settings)
        if "process" in settings:
            spec.process = settings.pop("process")
            spec.apply_process_defaults()
        for name, value in settings.items():
            setattr(spec, name, value)
        return spec


# ---- what each class of motor is, off its datasheets -------------------------------

_NEMA17 = (("motor_frame", "NEMA 17"), ("motor_kind", MotorKind.STEPPER),
           ("motor_holding_torque_Nm", 0.59), ("motor_rated_current_A", 2.0),
           ("motor_resistance_ohm", 1.4), ("motor_inductance_mH", 3.0),
           ("motor_steps_per_rev", 200), ("motor_supply_V", 24.0))
_NEMA23 = (("motor_frame", "NEMA 23"), ("motor_kind", MotorKind.STEPPER),
           ("motor_holding_torque_Nm", 3.0), ("motor_rated_current_A", 4.2),
           ("motor_resistance_ohm", 0.88), ("motor_inductance_mH", 3.4),
           ("motor_steps_per_rev", 200), ("motor_supply_V", 48.0))
_NEMA34 = (("motor_frame", "NEMA 34"), ("motor_kind", MotorKind.STEPPER),
           ("motor_holding_torque_Nm", 12.0), ("motor_rated_current_A", 5.0),
           ("motor_resistance_ohm", 1.1), ("motor_inductance_mH", 7.5),
           ("motor_steps_per_rev", 200), ("motor_supply_V", 70.0))
#: MAD's 8108 KV100 catalogue: 100 rpm/V, 186 mOhm, 350 W for an hour.  The
#: catalogue gives no continuous current, so 15 A is that power on the 24 V
#: this preset runs from.
_OUTRUNNER_8108 = (("motor_frame", "8108 outrunner"), ("motor_kind", MotorKind.DC),
                   ("motor_kv_rpm_per_V", 100.0), ("motor_resistance_ohm", 0.186),
                   ("motor_rated_current_A", 15.0), ("motor_supply_V", 24.0))

# ---- how it is made ---------------------------------------------------------------

_PRINTED = (("process", Process.FDM), ("disc_material", "PETG"),
            ("housing_material", "PETG"))
_MACHINED = (("process", Process.CNC), ("disc_material", "Steel 4140 (hardened)"),
             ("pin_material", "Bearing steel 100Cr6"),
             ("housing_material", "Aluminium 7075-T6"),
             ("shaft_material", "Steel 4140 (hardened)"))

#: A motor flush on the input plate cuts its register into the plate's outside
#: and its screws' recesses into the inside, and the default 8 mm plate has
#: room for neither and the bearing's lip as well.  12 mm has room for all
#: three, for the NEMA 17's screws into a thread and the NEMA 23's into nuts.
_MOTOR_PLATE = (("end_plate_thickness", 12.0),)


def _duty(torque_Nm: float, output_rpm: float, ratio: int) -> tuple:
    return (("output_torque_Nm", torque_Nm), ("input_rpm", output_rpm * ratio))


PRESETS: tuple[Preset, ...] = (
    *(Preset(f"ratio-{r}", f"{r}:1", "By ratio",
             "Geometry for the reduction, at the default duty in PLA; no motor "
             "named.", r) for r in (10, 15, 21, 29, 39, 59)),
    Preset("nema17-printed-15", "NEMA 17 - 15:1, printed", "NEMA 17 stepper",
           "A printed joint for a small arm or a camera head: 1 Nm at 20 rpm, "
           "PETG, the motor straight into the input shaft.", 15,
           (*_PRINTED, *_NEMA17, *_MOTOR_PLATE, *_duty(1.0, 20.0, 15),
            # The cam to the cam bearing's bore, which the 15:1's room leaves
            # two millimetres short of it.
            ("eccentric_cam_diameter", 18.0))),
    Preset("nema17-printed-21", "NEMA 17 - 21:1, printed", "NEMA 17 stepper",
           "More reduction in the same motor: 1.5 Nm at 15 rpm, PETG.", 21,
           (*_PRINTED, *_NEMA17, *_MOTOR_PLATE, *_duty(1.5, 15.0, 21))),
    Preset("nema23-machined-21", "NEMA 23 - 21:1, machined", "NEMA 23 stepper",
           "Steel discs in an aluminium housing: 25 Nm at 15 rpm off a 48 V "
           "driver.", 21,
           (*_MACHINED, *_NEMA23, *_MOTOR_PLATE, *_duty(25.0, 15.0, 21))),
    Preset("nema23-machined-29", "NEMA 23 - 29:1, machined", "NEMA 23 stepper",
           "The same motor geared for torque: 24 Nm at 10 rpm, where the "
           "output pins' fatigue sets the limit rather than the motor.", 29,
           (*_MACHINED, *_NEMA23, *_MOTOR_PLATE, *_duty(24.0, 10.0, 29))),
    Preset("nema34-machined-29", "NEMA 34 - 29:1, machined", "NEMA 34 stepper",
           "A large machined drive: 80 Nm at 8 rpm off a 70 V driver, on a "
           "180 mm housing.", 29,
           (*_MACHINED, *_NEMA34, *_duty(80.0, 8.0, 29),
            # Bigger than any ratio preset: the 29:1's 60 mm pin circle cannot
            # carry what this motor turns.
            ("pin_circle_radius", 80.0), ("pin_radius", 5.0),
            ("eccentricity", 1.6), ("output_bolt_circle_radius", 48.0),
            ("output_pin_diameter", 12.0), ("output_pin_count", 8),
            # The motor's 14 mm shaft goes into the input shaft, so that is
            # 20 - and a 30 mm cam round it, the HK3020 on that, and discs as
            # wide as the cup, since no narrower cup that size is listed.
            ("input_shaft_diameter", 20.0), ("eccentric_cam_diameter", 30.0),
            ("center_bore_diameter", 37.0), ("disc_thickness", 20.0),
            # Room for the register, the lip and the flanged motor's nuts.
            ("end_plate_thickness", 16.0))),
    Preset("outrunner-8108-machined-15", "8108 outrunner - 15:1",
           "Brushless outrunner",
           "A robot joint: 8 Nm at 60 rpm, the outrunner's can bolted to a hub "
           "on the input shaft, which runs through the motor's hollow centre.",
           15,
           (*_MACHINED, *_OUTRUNNER_8108, *_duty(8.0, 60.0, 15),
            ("eccentric_cam_diameter", 18.0))),
)


def by_key(key: str) -> Preset:
    """The preset with this key; ``KeyError`` naming the ones there are."""
    for p in PRESETS:
        if p.key == key:
            return p
    raise KeyError(f"no preset {key!r}; there are " + ", ".join(p.key for p in PRESETS))


def matching(spec: GearSpec) -> Preset | None:
    """The preset this design *is*, field for field, or None once it is not."""
    return next((p for p in PRESETS if p.ratio == spec.lobes and p.spec() == spec), None)
