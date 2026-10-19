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

A machined preset is the size its job needs.  They began on their ratio
preset's geometry, a pin circle of 55 to 80 mm, and stood two to three times
the width of their motors carrying four to nine times their duty.  Each is now
the smallest drive the design search and a pass of the full analysis found
that carries its duty at the search's own safety factor of 1.5 and outlasts
what its motor can put through the ratio - so a stall does not take the
gearbox with it - with ISO 8734 dowels for pins and the cam turned to a
catalogue bearing's bore.

A printed one cannot be sized that way.  On the printed contact model a drive
as wide as a machined one's ratio to its motor carries a fifth of a newton
metre at that safety factor - less than the motor does with no gearbox at all.
So the printed ones are 85 mm across, twice their motor, with 10 mm discs.
On a printed drive the continuous rating goes with the length of the disc -
the clearance leaves one or two pins carrying it whatever the diameter - but
a longer disc is a longer output pin, and past about 12 mm the drive survives
less for a moment, not more.  At 10 mm it is rated at 0.35 and 0.3 Nm, what
it carries at that safety factor, and survives ten and eleven for a moment.
The rating is held for thousands of hours against a contact allowable chosen
because plastic creeps, and it is a long way under what the motor can put
through the ratio: the momentary torque is the number that says it will not
break.

``tests/test_presets.py`` holds every motor preset to analysing with no error,
no warning about its motor, its mounting or its bearings, a capacity over its
rated torque, its motor's margin and that safety factor; a machined one to
outlasting its motor, and each to a width in proportion to its motor's.  The
warnings they do carry are the ones their geometry carries - the load
sharing, the parts round the mesh being softer than the mesh, a printed
drive's lost motion - which are not the motor's.

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
           "A printed joint for a small arm or a camera head: 0.35 Nm at 20 "
           "rpm, PETG, the motor straight into the input shaft, on an 85 mm "
           "housing.", 15,
           (*_PRINTED, *_NEMA17, *_MOTOR_PLATE, *_duty(0.35, 20.0, 15),
            # 5 mm ring pins on a 68 mm circle and two 10 mm discs.  The cam
            # on an HK1808, which is the width of the disc, and six 5 mm
            # output pins round it.
            ("pin_circle_radius", 34.0), ("pin_radius", 2.5),
            ("eccentricity", 1.2), ("disc_thickness", 10.0),
            ("eccentric_cam_diameter", 18.0), ("center_bore_diameter", 24.0),
            ("output_bolt_circle_radius", 21.0), ("output_pin_count", 6),
            ("output_pin_diameter", 5.0))),
    Preset("nema17-printed-21", "NEMA 17 - 21:1, printed", "NEMA 17 stepper",
           "More reduction in the same motor, for finer steps rather than more "
           "torque - the plastic sets that: 0.3 Nm at 15 rpm, PETG, on an 85 mm "
           "housing.", 21,
           (*_PRINTED, *_NEMA17, *_MOTOR_PLATE, *_duty(0.3, 15.0, 21),
            ("pin_circle_radius", 34.0), ("pin_radius", 2.5),
            ("eccentricity", 1.1), ("disc_thickness", 10.0),
            ("eccentric_cam_diameter", 18.0), ("center_bore_diameter", 24.0),
            ("output_bolt_circle_radius", 21.0), ("output_pin_count", 6),
            ("output_pin_diameter", 5.0))),
    Preset("nema23-machined-21", "NEMA 23 - 21:1, machined", "NEMA 23 stepper",
           "Steel discs in an aluminium housing: 25 Nm at 15 rpm off a 48 V "
           "driver, on an 83 mm housing.", 21,
           (*_MACHINED, *_NEMA23, *_MOTOR_PLATE, *_duty(25.0, 15.0, 21),
            # 5 mm ring pins on a 66 mm circle, two 14 mm discs; the cam on an
            # HK1512 and ten 8 mm output pins round it.
            ("pin_circle_radius", 33.0), ("pin_radius", 2.5),
            ("eccentricity", 1.1), ("disc_thickness", 14.0),
            ("eccentric_cam_diameter", 15.0), ("center_bore_diameter", 21.0),
            ("output_bolt_circle_radius", 20.0), ("output_pin_count", 10),
            ("output_pin_diameter", 8.0))),
    Preset("nema23-machined-29", "NEMA 23 - 29:1, machined", "NEMA 23 stepper",
           "The same motor geared for torque: 24 Nm at 10 rpm, on a 90 mm "
           "housing.", 29,
           (*_MACHINED, *_NEMA23, *_MOTOR_PLATE, *_duty(24.0, 10.0, 29),
            # Five 12 mm output pins: at this ratio they carry the most and
            # bend the most, and fewer, thicker ones have the room.
            ("pin_circle_radius", 36.0), ("pin_radius", 3.0),
            ("eccentricity", 0.75), ("disc_thickness", 14.0),
            ("eccentric_cam_diameter", 15.0), ("center_bore_diameter", 21.0),
            ("output_bolt_circle_radius", 21.5), ("output_pin_count", 5),
            ("output_pin_diameter", 12.0),
            # At this ratio the motor stalled at full current dents a 6900's
            # races: a 35 mm boss leaves room for a 6000 on the shaft, and puts
            # a 6807 round the boss exactly.  Named, since a 6900 lasts and the
            # selection takes the smallest that does.  It is 2 mm wider, and
            # the motor's register would take the lip that holds it in: the
            # plates go to 16 mm, the next thickness the motor's screws seat in
            # as well.
            ("output_hub_diameter", 35.0), ("shaft_bearing", "6000"),
            ("end_plate_thickness", 16.0))),
    Preset("nema34-machined-29", "NEMA 34 - 29:1, machined", "NEMA 34 stepper",
           "A large machined drive: 80 Nm at 8 rpm off a 70 V driver, on a "
           "111 mm housing.", 29,
           (*_MACHINED, *_NEMA34, *_duty(80.0, 8.0, 29),
            ("pin_circle_radius", 45.0), ("pin_radius", 3.0),
            ("eccentricity", 1.0), ("output_bolt_circle_radius", 29.5),
            ("output_pin_diameter", 12.0), ("output_pin_count", 8),
            # The motor's 14 mm shaft goes into the input shaft, so that is
            # 20 - and a 30 mm cam round it, the HK3020 on that, and discs as
            # wide as the cup, since no narrower cup that size is listed.
            ("input_shaft_diameter", 20.0), ("eccentric_cam_diameter", 30.0),
            ("center_bore_diameter", 37.0), ("disc_thickness", 20.0),
            # The crank's reaction on a pin circle this size wants more on the
            # shaft than the 32 mm the default boss leaves room for: a 6904
            # lasts, but the motor stalled at full current dents its races,
            # and a 6004 it does not.  That is a 50 mm boss, and the 6810 round
            # it is 65 across, so the wall it is bored through goes to 7.5 mm
            # to make the seat that wide.
            ("output_hub_diameter", 50.0), ("housing_wall", 7.5),
            ("shaft_bearing", "6004"),
            # Room for the register, the lip beyond a 12 mm wide 6004 and the
            # flanged motor's nuts, at a thickness its screws seat in.
            ("end_plate_thickness", 21.0))),
    Preset("outrunner-8108-machined-15", "8108 outrunner - 15:1",
           "Brushless outrunner",
           "A robot joint: 8 Nm at 30 rpm, the outrunner's can bolted to a hub "
           "on the input shaft, which runs through the motor's hollow centre.",
           15,
           (*_MACHINED, *_OUTRUNNER_8108, *_duty(8.0, 30.0, 15),
            # As wide as the motor and short: a joint is a pancake.  Its speed
            # is what such a joint runs at for minutes on end, not the most it
            # can: the 21 W it loses at 60 rpm would hold this housing at 97 C
            # in still air, and shedding that under 70 C takes the 120 mm
            # housing this preset used to have.
            ("pin_circle_radius", 34.0), ("pin_radius", 3.0),
            ("eccentricity", 1.25), ("output_bolt_circle_radius", 21.0),
            ("output_pin_count", 6), ("output_pin_diameter", 5.0),
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
