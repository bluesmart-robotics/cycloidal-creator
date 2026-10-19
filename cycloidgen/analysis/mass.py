"""Mass, inertia, rotating unbalance and the disc's own structure.

Everything here comes off the sampled profile rather than off a cylinder
approximation: the disc is a lobed annulus with a bore and a ring of holes
through it, and treating it as a solid disc overstates its mass by a third.

Three questions get answered:

* **How heavy is it, and what does that buy?**  Torque capacity per kilogram is
  the number that actually compares a cycloidal drive against the planetary it
  is competing with.
* **What does the crank have to throw around?**  A disc orbiting at radius E is
  an unbalanced mass, and the force it develops goes as the *square* of input
  speed - which is why one disc is fine at 200 rpm and shakes itself apart at
  4000.  Two discs at 180 degrees cancel the force but leave a couple.
* **Will the web hold?**  Output pin load has to cross the ligament between the
  hole and the nearest free surface.  That ligament is the thinnest structural
  member in the whole drive and nothing else in the app looks at it.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from ..core import profile as prof
from ..core.fasteners import (
    BRASS_DENSITY,
    HeatSetInsert,
    MetricSize,
    insert_volume,
    nut_volume,
    screw_volume,
)
from ..core.motor import COUPLING_SLOT_DEPTH, COUPLING_SLOT_WIDTH, COUPLING_SLOTS
from ..core.spec import CUP_CEILING, MATERIALS, GearSpec, OutputMember, d_section_area
from .bearings import BearingChoice, SeatBores, bearing_schedule, pin_diameters, seat_bores

__all__ = ["MassResult", "analyse_mass", "insert_mass_g", "plate_dowel_mass_g",
           "rotor_hub_screw_mass_g", "tie_bolt_mass_g"]

_MM3_TO_CM3 = 1e-3


@dataclass
class MassResult:
    """Mass properties and the disc web check."""

    disc_mass_g: float                 # one disc
    disc_volume_cm3: float
    total_mass_g: float
    housing_mass_g: float               # the barrel alone
    plates_mass_g: float                # both end plates together
    pins_mass_g: float
    shaft_mass_g: float
    flange_mass_g: float
    #: The end cap, on a ring-output drive; zero on the other, where there
    #: is no such part.  Its own line rather than folded into the carrier's:
    #: it is a separate thing to make, and a made part quoting zero grams on
    #: the bill of materials is a part the mass model has not been told about.
    end_cap_mass_g: float
    #: The hub an outrunner's can drives the input shaft through; zero for any
    #: other motor.  In the shaft's material, and turning with it.
    rotor_hub_mass_g: float
    #: The shaft's two locating sleeves and the washers between the discs,
    #: together: small, but made, and on the bill of materials.
    spacers_mass_g: float
    #: Every bearing the drive is built with, at the count it needs: the cam
    #: bearings, the shaft supports, the main output bearings and the pin
    #: rollers.  Bought, but in the gearbox, so on the scale.  Estimated from
    #: each part's envelope - see :data:`~.bearings.BEARING_FILL`.
    bearings_mass_g: float
    #: The shaft coupling, when there is one: aluminium, bought.
    coupling_mass_g: float
    #: The motor mount the motor stands on through a coupling: part of the
    #: input end plate on a carrier drive, of the carrier's base on a ring one,
    #: and counted in that part's mass - this is its share of it.
    mount_mass_g: float
    #: The tie bolts and their nuts, the motor's screws where the drive names
    #: them, and the dowels that locate the end plates - in the material the
    #: bill orders each in.  Not the output bolts: they have no length until the
    #: drive knows what they go into, and a guessed length is a guessed mass.
    fasteners_mass_g: float

    disc_inertia_kg_mm2: float         # about its own centre
    #: The disc stage: everything that moves the way a disc moves - the discs
    #: and the cam bearings pressed into them - seen at the input shaft.  One
    #: of the three terms of :attr:`input_inertia_kg_mm2`.
    reflected_inertia_kg_mm2: float
    #: The input shaft with its cams and the sleeves clamped to it, about its
    #: own axis.  The motor turns it directly, so it reaches the input as is.
    shaft_inertia_kg_mm2: float
    #: The member that turns at the output, about the axis: the carrier with
    #: its pins, or on a ring-output drive the housing with its end plates,
    #: ring pins and tie bolts.  At the output - see the next field.
    output_inertia_kg_mm2: float
    #: All of it as the motor sees it: the shaft, the disc stage, and the
    #: output member through the square of the ratio.  What a motor is sized
    #: and a servo loop tuned against, less the load, which the drive does not
    #: know.
    input_inertia_kg_mm2: float

    #: Residual, at the rated input speed, of everything orbiting at E in each
    #: disc's plane: the disc, its cam bearing and the cam it runs on.
    unbalance_force_N: float
    unbalance_couple_Nmm: float
    #: That orbiting mass, one disc's plane of it, grams.
    orbiting_mass_g: float
    balanced: bool

    web_shear_MPa: float
    web_shear_allow_MPa: float
    hole_bearing_MPa: float
    min_web_mm: float

    @property
    def web_safety_factor(self) -> float:
        return (self.web_shear_allow_MPa / self.web_shear_MPa
                if self.web_shear_MPa > 0 else float("inf"))

    def power_density_Nm_per_kg(self, torque_capacity_Nm: float) -> float:
        kg = self.total_mass_g / 1000.0
        return torque_capacity_Nm / kg if kg > 0 else 0.0


def _disc_section(spec: GearSpec, bore: float) -> tuple[float, float]:
    """Cross-sectional area and polar second moment of one disc, mm^2 and mm^4.

    ``bore`` is the hole as bored - see :class:`~.bearings.SeatBores` - which
    is the cam bearing's outside wherever there is one, not the room the
    parameter left for it.
    """
    p = prof.profile_from_spec(spec, n=4000)
    area = p.area()
    polar = p.polar_second_moment()

    bore_r = bore / 2.0
    area -= math.pi * bore_r ** 2
    polar -= 0.5 * math.pi * bore_r ** 4

    hole_r = spec.output_hole_diameter / 2.0
    d = spec.output_bolt_circle_radius
    hole_area = math.pi * hole_r ** 2
    # parallel axis: each hole sits a bolt-circle radius off the disc centre
    hole_polar = 0.5 * math.pi * hole_r ** 4 + hole_area * d * d
    area -= spec.output_pin_count * hole_area
    polar -= spec.output_pin_count * hole_polar

    return max(area, 0.0), max(polar, 0.0)


def _base_motor_cuts(spec: GearSpec, bores: SeatBores) -> float:
    """What the motor takes out of a ring drive's base besides its screws'
    holes, mm^3: the register its spigot drops into, and the counterbores or
    nut pockets let into the base's other face - as the solid cuts them, so
    a base thinned to a flange round its pad weighs what it encloses.  These
    were left out while the base was a whole plate, where they were a
    percent or two of it."""
    if not spec.has_motor_face or spec.coupled:
        return 0.0
    frame = spec.motor_face
    cut = 0.0
    if frame.pilot_diameter > bores.hub:
        depth = min(frame.pilot_depth, spec.plate_thickness / 2.0)
        cut += math.pi * ((frame.pilot_diameter / 2.0) ** 2
                          - (bores.hub / 2.0) ** 2) * depth
    fix = spec.motor_fixing
    if fix is not None and fix.recess_depth > 0.0:
        pocket = (math.pi * (fix.recess / 2.0) ** 2 if fix.from_drive
                  else math.sqrt(3.0) / 2.0 * fix.recess ** 2)
        hole = math.pi * (frame.bolt_diameter / 2.0) ** 2
        cut += len(spec.motor_bolt_centres) * (pocket - hole) * fix.recess_depth
    return cut


def _lip_volume(bores: SeatBores, host: str) -> float:
    """The metal the shoulders leave in ``host``'s seats, mm^3.

    A seat is weighed bored right through to the bearing; the lip past the
    bearing is metal that bore does not take, and it is put back here as the
    solid puts it back.
    """
    return sum(math.pi * ((s.seat / 2.0) ** 2 - (s.bore / 2.0) ** 2) * s.thickness
               for s in bores.shoulders if s.host == host)


def _polar(r_out: float, r_in: float, length: float) -> float:
    """The integral of r^2 dV of a tube about its own axis, mm^5.  ``r_in`` 0 is a rod."""
    return math.pi * length * (r_out ** 4 - r_in ** 4) / 2.0


def _polar_off_axis(radius: float, centre: float, length: float) -> float:
    """The integral of r^2 dV of a rod whose own axis is ``centre`` off the drive's, mm^5."""
    return math.pi * radius ** 2 * length * (centre ** 2 + radius ** 2 / 2.0)


def _lip_polar(bores: SeatBores, host: str) -> float:
    """The integral of r^2 dV of the shoulders in ``host``'s seats - see :func:`_lip_volume`."""
    return sum(_polar(s.seat / 2.0, s.bore / 2.0, s.thickness)
               for s in bores.shoulders if s.host == host)


def _rollers_on_circle(bearings: Sequence[BearingChoice], role: str,
                       circle: float) -> float:
    """Inertia of the pin rollers in ``role``, riding on ``circle``, kg mm^2."""
    choice = next((c for c in bearings if c.role == role and c.fitted and c.fits
                   and c.bearing is not None and c.count), None)
    if choice is None:
        return 0.0
    part = choice.bearing
    kg = choice.count * part.mass_g / 1000.0
    return kg * (circle ** 2 + (part.outer ** 2 + part.bore ** 2) / 8.0)


def motor_screw_mass_g(spec: GearSpec) -> float:
    """One of the motor's screws, with its nut where it takes one, grams."""
    fix = spec.motor_fixing
    if fix is None:
        return 0.0
    volume = screw_volume(fix.size, fix.length)
    if not fix.from_drive:
        volume += nut_volume(fix.size)
    return volume * _MM3_TO_CM3 * spec.pin_mat.density_g_cm3


def rotor_hub_screw_mass_g(spec: GearSpec) -> float:
    """One of the screws an outrunner's can bolts to its hub with, grams."""
    screw = spec.rotor_hub_screw
    if screw is None:
        return 0.0
    return screw_volume(*screw) * _MM3_TO_CM3 * spec.pin_mat.density_g_cm3


def plate_dowel_mass_g(spec: GearSpec) -> float:
    """One plate dowel, grams: a hardened steel cylinder, its ends' small
    chamfers left in; zero with none."""
    length = spec.plate_dowel_length
    if not length:
        return 0.0
    rho = MATERIALS["Bearing steel 100Cr6"].density_g_cm3
    return math.pi * (spec.plate_dowel_diameter / 2.0) ** 2 * length * _MM3_TO_CM3 * rho


def insert_mass_g(spec: GearSpec, insert: HeatSetInsert,
                  size: MetricSize | None = None) -> float:
    """One heat-set insert, grams, in brass: for ``size``, or the tie screws'."""
    size = size or spec.tie_bolt_size
    if size is None:
        return 0.0
    return insert_volume(insert, size) * _MM3_TO_CM3 * BRASS_DENSITY


def _blind(spec: GearSpec) -> list[tuple[list[tuple[float, float]], float, float]]:
    """A ring drive's housing's blind holes: ``(centres, diameter, depth)`` -
    up into its underside for the plate's screws and its dowels, and down into
    its face for the driven machine's screws."""
    return [(spec.tie_bolt_centres, spec.tie_thread_hole, spec.tie_thread_depth),
            (spec.plate_dowel_centres, spec.plate_dowel_diameter,
             spec.housing_dowel_depth),
            (spec.output_face_bolt_centres, spec.output_face_hole,
             spec.output_face_hole_depth)]


def _cup_holes_volume(spec: GearSpec) -> float:
    """What the blind holes in a ring drive's housing take out of it, mm^3."""
    return sum(len(centres) * math.pi * (d / 2.0) ** 2 * depth
               for centres, d, depth in _blind(spec))


def _cup_holes_polar(spec: GearSpec) -> float:
    """The integral of r^2 dV over those holes - see :func:`_cup_holes_volume`."""
    return sum(_polar_off_axis(d / 2.0, math.hypot(x, y), depth)
               for centres, d, depth in _blind(spec) for x, y in centres)


def _turning_plate_volume(spec: GearSpec, bores: SeatBores) -> float:
    """The plate under a ring drive's housing, mm^3: the outline less the main
    bearing's seat, its screws' clearance and their counterbores, and its
    dowels' holes, with the lip put back."""
    t = spec.housing_plate_thickness
    volume = (spec.housing_section_area
              - math.pi * (bores.output_plate / 2.0) ** 2) * t
    volume -= (len(spec.plate_dowel_centres) * math.pi
               * (spec.plate_dowel_hole / 2.0) ** 2 * t)
    counterbore = spec.tie_counterbore
    if counterbore is not None:
        width, depth = counterbore
        clearance = spec.tie_bolt_size.clearance
        n = len(spec.tie_bolt_centres)
        volume -= n * math.pi * ((clearance / 2.0) ** 2 * t
                                 + ((width / 2.0) ** 2 - (clearance / 2.0) ** 2) * depth)
    return volume + _lip_volume(bores, "output_end_plate")


def _turning_plate_polar(spec: GearSpec, bores: SeatBores, outline: float) -> float:
    """The integral of r^2 dV over that plate - see :func:`_turning_plate_volume`."""
    t = spec.housing_plate_thickness
    polar = (outline - _polar(bores.output_plate / 2.0, 0.0, 1.0)) * t
    polar -= sum(_polar_off_axis(spec.plate_dowel_hole / 2.0, math.hypot(x, y), t)
                 for x, y in spec.plate_dowel_centres)
    counterbore = spec.tie_counterbore
    if counterbore is not None:
        width, depth = counterbore
        clearance = spec.tie_bolt_size.clearance
        for x, y in spec.tie_bolt_centres:
            r = math.hypot(x, y)
            polar -= (_polar_off_axis(clearance / 2.0, r, t - depth)
                      + _polar_off_axis(width / 2.0, r, depth))
    return polar + _lip_polar(bores, "output_end_plate")


def tie_bolt_mass_g(spec: GearSpec) -> float:
    """One tie bolt and its nut, grams - the screw alone where it threads into
    a tapped plate, and its insert with it where that is in one; zero where
    the drive cannot name them.

    A hole the fastener table has no size for is ordered by the hole and a
    length, which is not a part anybody can weigh.
    """
    size, length = spec.tie_bolt_size, spec.tie_screw_length
    if size is None or length is None:
        return 0.0
    nut = 0.0 if spec.tie_screws_tapped else nut_volume(size)
    insert = (insert_mass_g(spec, spec.tie_insert)
              if spec.tie_screws_tapped and spec.tie_insert is not None else 0.0)
    return ((screw_volume(size, length) + nut)
            * _MM3_TO_CM3 * spec.pin_mat.density_g_cm3 + insert)


def analyse_mass(spec: GearSpec, bores: SeatBores | None = None,
                 bearings: Sequence[BearingChoice] | None = None) -> MassResult:
    """Mass, inertia, unbalance and the disc web stress.

    Every bore is weighed as it is bored, which is to the bearing in it, and
    every bearing is weighed in its seat.  ``bores`` and ``bearings`` when the
    caller has them already - working the schedule out needs the loads, which
    is most of the cost - and worked out otherwise.
    """
    if bearings is None:
        bearings = bearing_schedule(spec)
    if bores is None:
        bores = seat_bores(spec, bearings)
    area, polar = _disc_section(spec, bores.disc)
    t = spec.disc_thickness
    n = spec.disc_count

    rho_disc = spec.disc_mat.density_g_cm3
    rho_house = spec.housing_mat.density_g_cm3
    rho_pin = spec.pin_mat.density_g_cm3
    rho_shaft = spec.shaft_mat.density_g_cm3

    disc_volume = area * t                                    # mm^3
    disc_mass = disc_volume * _MM3_TO_CM3 * rho_disc          # g

    # ---- the rest of the parts ---------------------------------------------
    # Inside the outline, round or square, less the bore the pins line.
    ring_area = spec.housing_section_area - math.pi * spec.pin_circle_radius ** 2
    pocket_area = spec.pin_count * 0.5 * math.pi * spec.pin_radius ** 2
    # Every hole through the barrel, not just the ones the pins sit in: the tie
    # bolts run its whole length too, and a mass model that knows about one set
    # and not the other is describing a different part from the one the exporter
    # writes.
    bolt_area = (len(spec.tie_bolt_centres) * math.pi
                 * (spec.housing_bolt_diameter / 2.0) ** 2)
    # And the dowels that locate the plates: a hole through the barrel's length
    # and through each plate, at the dowel's own diameter.
    dowel_r = spec.plate_dowel_diameter / 2.0
    dowel_area = len(spec.plate_dowel_centres) * math.pi * dowel_r ** 2
    slip_r = spec.plate_dowel_hole / 2.0
    slip_area = len(spec.plate_dowel_centres) * math.pi * slip_r ** 2
    # The barrel is longer than the disc stack: it reaches down past the carrier
    # to the output end plate it bolts to.  The pins are not - they only have to
    # span the discs - which is why these two lengths are different.
    # Integral pins are the housing, so the barrel gains the half-moons it
    # would otherwise have had cut out of it - and gains them in the *housing's*
    # material, which on a printed drive is the whole point of the option and
    # is lighter than the steel dowels it replaces.
    pins_in_barrel = pocket_area if spec.ring_pins_integral else -pocket_area
    housing_volume = (max(ring_area + pins_in_barrel - bolt_area - dowel_area, 0.0)
                      * spec.barrel_height)
    if spec.ground_frame_fitted:
        # A ring drive's housing: no hole goes through it.  Its screws' and
        # dowels' holes are blind up into its underside, which is all that
        # came off above; its top, bored for the second main bearing with the
        # lip past it, and the ceiling over that, closing it; and the driven
        # machine's screw holes blind down into the face.
        housing_volume = (max(ring_area + pins_in_barrel, 0.0) * spec.barrel_height
                          - _cup_holes_volume(spec))
        housing_volume += ((spec.housing_section_area
                            - math.pi * (bores.input_plate / 2.0) ** 2)
                           * spec.housing_plate_thickness
                           + _lip_volume(bores, "housing")
                           + spec.housing_section_area * CUP_CEILING)
    housing_mass = housing_volume * _MM3_TO_CM3 * rho_house

    # Each pin at its own length and its own diameter, and neither length is the
    # disc stack: a ring pin spans the barrel it is trapped in, an output pin the
    # drop it crosses plus the discs it drives.  Weighing both off
    # ``stack_height`` was the same assumption the bill of materials was making,
    # and it is 41% light on the ring pins of a 21:1.
    ring_pin_d, output_pin_d = pin_diameters(spec)
    pins_volume = (0.0 if spec.ring_pins_integral else
                   spec.pin_count * math.pi * (ring_pin_d / 2.0) ** 2
                   * spec.ring_pin_length)
    pins_volume += (spec.output_pin_count * math.pi * (output_pin_d / 2.0) ** 2
                    * spec.output_pin_length)
    pins_mass = pins_volume * _MM3_TO_CM3 * rho_pin

    span_lo, span_hi = spec.shaft_span
    shaft_volume = math.pi * (spec.input_shaft_diameter / 2.0) ** 2 * (span_hi - span_lo)
    cam_extra = n * math.pi * ((spec.cam_diameter / 2.0) ** 2
                               - (spec.input_shaft_diameter / 2.0) ** 2) * t
    # Less the motor's bore, driven directly: a D through shaft and cams alike.
    bore = spec.motor_bore
    bore_volume = (0.0 if bore is None else
                   d_section_area(bore[2] / 2.0, bore[3]) * (bore[1] - bore[0]))
    # The hub an outrunner's can bolts to: a disc on the shaft's end, less its
    # bore and the can's screw holes.
    hub_volume = hub_polar = 0.0
    if spec.rotor_hub is not None:
        z0, z1, outer, hub_bore = spec.rotor_hub
        screw = spec.rotor_hub_screw
        hole = screw[0].clearance if screw else 0.0
        centres = spec.rotor_hub_bolt_centres
        hub_volume = (math.pi / 4.0 * (outer ** 2 - hub_bore ** 2)
                      - len(centres) * math.pi / 4.0 * hole ** 2) * (z1 - z0)
        hub_polar = (_polar(outer / 2.0, hub_bore / 2.0, z1 - z0)
                     - sum(_polar_off_axis(hole / 2.0, math.hypot(x, y), z1 - z0)
                           for x, y in centres))
    hub_mass = hub_volume * _MM3_TO_CM3 * rho_shaft
    shaft_mass = ((shaft_volume + max(cam_extra, 0.0) - bore_volume)
                  * _MM3_TO_CM3 * rho_shaft)

    plate_r = spec.carrier_radius
    flange_volume = math.pi * plate_r ** 2 * spec.output_flange_thickness
    # Plus the boss the drive turns on, bored through for the shaft, and less
    # that same bore through the plate.  As long as it is drawn: through the
    # end plate *and* standing proud of it, for a coupling to grip or as the
    # frame's standoff.  It was weighed flush, after the solid had stopped
    # being - 3.4 cm3 of boss on every preset that only the STEP file had.
    flange_volume += math.pi * ((spec.hub_diameter / 2.0) ** 2
                                - (bores.hub / 2.0) ** 2) * (
        spec.housing_plate_thickness + spec.boss_protrusion)
    flange_volume -= math.pi * (bores.hub / 2.0) ** 2 * spec.output_flange_thickness
    # And, on a ring-output drive, the base on the end of that boss - the plate
    # the whole gearbox is bolted down by.  It is housing-sized and as thick as
    # an end plate, so on a small drive it is a fifth of the assembled mass;
    # leaving it out would report a gearbox lighter than the one exported.
    # Where the base is a flange and a pad, the flange is the outline at its own
    # thickness and the pad the rest of the way down under the motor.
    if spec.base_wraps_motor:
        # The plate the motor stands on, the sleeve's outline across, and the
        # sleeve under it down to the motor's back, less its cable's slot.
        width, height = spec.cable_slot
        h, _ = spec.sleeve_bore
        flange_volume += (spec.sleeve_area * spec.plate_thickness
                          + (spec.sleeve_area - spec.sleeve_bore_area)
                          * spec.motor_face.body_length
                          - width * height * (spec.sleeve_half_width - h)
                          - math.pi * (bores.hub / 2.0) ** 2 * spec.plate_thickness)
    elif spec.ground_frame_fitted:
        flange = spec.base_flange_thickness
        flange_volume += (spec.housing_section_area * flange
                          + spec.base_pad_area * (spec.plate_thickness - flange)
                          - math.pi * (bores.hub / 2.0) ** 2 * spec.plate_thickness)
    if spec.ground_frame_fitted:
        flange_volume -= (len(spec.base_mount_centres) * math.pi
                          * (spec.base_mount_hole / 2.0) ** 2 * spec.base_mount_depth)
        flange_volume -= _base_motor_cuts(spec, bores)
        if spec.has_motor_face:
            frame = spec.motor_face
            flange_volume -= (frame.bolt_count * math.pi
                              * (frame.bolt_diameter / 2.0) ** 2
                              * spec.plate_thickness)
    flange_volume += _lip_volume(bores, "output_flange")
    if spec.ground_frame_fitted:
        flange_volume += spec.motor_mount_volume      # under the base
    flange_mass = max(flange_volume, 0.0) * _MM3_TO_CM3 * rho_house

    # The end cap at the far end of the pins: the carrier's mirror image, and
    # a part of its own to make.  Plate less its pin clearance holes and its
    # bore, plus the boss the input-side output bearing rides on.
    cap_volume = 0.0
    if spec.ground_frame_fitted:
        pin_hole_r = (output_pin_d + spec.hole_clearance) / 2.0
        cap_volume = ((math.pi * plate_r ** 2
                       - math.pi * (bores.cap / 2.0) ** 2
                       - spec.output_pin_count * math.pi * pin_hole_r ** 2)
                      * spec.output_flange_thickness
                      + math.pi * ((spec.hub_diameter / 2.0) ** 2
                                   - (bores.cap / 2.0) ** 2)
                      * (spec.cap_boss_top - spec.end_cap_top))
    if spec.ground_frame_fitted:
        cap_volume += _lip_volume(bores, "end_cap")
    cap_mass = max(cap_volume, 0.0) * _MM3_TO_CM3 * rho_house

    # The two plates that close the housing.  They are part of the gearbox and
    # were simply not weighed: on the 21:1 preset they are a third of it.
    #
    # Reported beside the barrel rather than folded into it.  They are a separate
    # part with its own line on the bill of materials, and while the two were one
    # number that line had to quote zero - a made part that weighs nothing.
    outer_area = spec.housing_section_area
    # The output bolts go through one of the two, so they come off once - the
    # same accounting the tie bolts get for going through both.
    output_bolt_area = (spec.output_bolt_count * math.pi
                        * (spec.output_bolt_diameter / 2.0) ** 2
                        if spec.ground_frame_fitted else 0.0)
    input_tie_area = bolt_area
    plates_volume = (2.0 * outer_area
                     - math.pi * (bores.input_plate / 2.0) ** 2
                     - math.pi * (bores.output_plate / 2.0) ** 2
                     - bolt_area - input_tie_area   # the tie bolts go through both
                     - 2.0 * slip_area          # and so do the dowels' holes
                     - output_bolt_area
                     ) * spec.plate_thickness
    plates_volume += (_lip_volume(bores, "input_end_plate")
                      + _lip_volume(bores, "output_end_plate"))
    if spec.ground_frame_fitted:
        plates_volume = _turning_plate_volume(spec, bores)
    # Through a coupling, the motor stands on a mount that is part of the
    # plate at its end of the drive: the input plate, or the carrier's base.
    mount_volume = spec.motor_mount_volume
    if not spec.ground_frame_fitted:
        plates_volume += mount_volume
    mount_mass = mount_volume * _MM3_TO_CM3 * rho_house
    plates_mass = max(plates_volume, 0.0) * _MM3_TO_CM3 * rho_house

    # The sleeves in the shaft's material and the washers in the discs', which
    # is what the bill of materials orders them in.
    spacers_mass = sum(
        math.pi * ((s.outer / 2.0) ** 2 - (s.bore / 2.0) ** 2) * s.length
        * _MM3_TO_CM3 * (rho_disc if s.name.startswith("washer") else rho_shaft)
        for s in bores.spacers)

    # The parts the seats are bored to, and only those: a bearing switched off,
    # carried by the motor, or named and too big to go in is not in this
    # gearbox, which is the same rule the bores and the 3D view follow.
    bearings_mass = sum(c.count * c.bearing.mass_g for c in bearings
                        if c.fitted and c.fits and c.bearing is not None)

    tie_mass = tie_bolt_mass_g(spec) * len(spec.tie_bolt_centres)
    hub_screws_mass = rotor_hub_screw_mass_g(spec) * len(spec.rotor_hub_bolt_centres)
    dowels_mass = (plate_dowel_mass_g(spec) * len(spec.plate_dowel_spans)
                   * len(spec.plate_dowel_centres))
    fasteners_mass = (tie_mass + dowels_mass + hub_screws_mass
                      + motor_screw_mass_g(spec) * len(spec.motor_bolt_centres))
    if spec.base_insert is not None:
        fasteners_mass += (insert_mass_g(spec, spec.base_insert,
                                         spec.bolt_size_asked)
                           * len(spec.base_mount_centres))
    if spec.output_insert is not None:
        fasteners_mass += (insert_mass_g(spec, spec.output_insert,
                                         spec.output_bolt_size)
                           * len(spec.output_face_bolt_centres))

    coupling = spec.coupling
    coupling_polar = 0.0
    coupling_mass = 0.0
    if coupling is not None:
        # Aluminium, as beam couplings are, bored half to each shaft.
        rho_al = MATERIALS["Aluminium 7075-T6"].density_g_cm3
        half = coupling.length / 2.0
        r_o = coupling.outer / 2.0
        bores_polar = (_polar(spec.input_shaft_diameter / 2.0, 0.0, half)
                       + _polar(spec.motor_face.shaft_diameter / 2.0, 0.0, half))
        volume = (math.pi * r_o * r_o * coupling.length
                  - math.pi * (spec.input_shaft_diameter ** 2
                               + spec.motor_face.shaft_diameter ** 2) / 4.0 * half)
        coupling_polar = _polar(r_o, 0.0, coupling.length) - bores_polar
        if coupling.catalogue:
            # Less the beam's cuts, as the solid draws them.
            r_s = r_o - COUPLING_SLOT_DEPTH
            volume -= (COUPLING_SLOTS * math.pi * (r_o * r_o - r_s * r_s)
                       * COUPLING_SLOT_WIDTH)
            coupling_polar -= COUPLING_SLOTS * _polar(r_o, r_s, COUPLING_SLOT_WIDTH)
        coupling_mass = volume * _MM3_TO_CM3 * rho_al
        coupling_polar *= rho_al

    total = (n * disc_mass + housing_mass + plates_mass + pins_mass
             + shaft_mass + flange_mass + cap_mass + spacers_mass + bearings_mass
             + fasteners_mass + coupling_mass + hub_mass)

    # ---- inertia ------------------------------------------------------------
    # kg*mm^2:  g/cm^3 * mm^4 * 1e-3 (cm^3/mm^3) / 1e3 (kg/g) = 1e-6
    disc_inertia = polar * t * rho_disc * 1e-6
    m_disc_kg = disc_mass / 1000.0
    # Each disc orbits at radius E - at the input speed exactly, whichever
    # member is grounded, because the orbit is the crank's own motion seen from
    # the axis - and spins at `disc_speed_ratio` of the input.  That second term
    # is 1/N off the carrier and *zero* off the ring, where the disc does not
    # rotate in the ground frame at all: a ring-output drive reflects only the
    # orbiting mass back to the motor.
    #
    # The cam bearing goes with it.  Its cup is pressed into the disc bore and
    # its rollers' centres ride the cam, so it orbits and spins exactly as the
    # disc does - which is also why it is weighed in the same plane below.
    cam = next((c for c in bearings if c.role == "Eccentric cam bearing"
                and c.fitted and c.fits and c.bearing is not None and c.count), None)
    if cam is not None:
        part = cam.bearing
        m_cam_bearing_kg = cam.count * part.mass_g / n / 1000.0
        cam_bearing_inertia = (m_cam_bearing_kg
                               * (part.outer ** 2 + part.bore ** 2) / 8.0)
    else:
        m_cam_bearing_kg = cam_bearing_inertia = 0.0
    reflected = n * ((m_disc_kg + m_cam_bearing_kg) * spec.eccentricity ** 2
                     + (disc_inertia + cam_bearing_inertia)
                     * spec.disc_speed_ratio ** 2)

    # The shaft, off the same shapes it was weighed as: a rod, a cam per disc
    # centred at E less the rod's section it already counted, and the sleeves
    # that turn with it.  Its phases do not matter here; a radius is a radius.
    rs = spec.input_shaft_diameter / 2.0
    shaft_polar = (_polar(rs, 0.0, span_hi - span_lo)
                   + n * (_polar_off_axis(spec.cam_diameter / 2.0,
                                          spec.eccentricity, t)
                          - _polar(rs, 0.0, t)))
    if bore is not None:          # the flat's sliver is a part in a thousand
        shaft_polar -= _polar(bore[2] / 2.0, 0.0, bore[1] - bore[0])
    sleeves_polar = sum(_polar(s.outer / 2.0, s.bore / 2.0, s.length)
                        for s in bores.spacers if not s.name.startswith("washer"))
    shaft_inertia = ((shaft_polar + sleeves_polar + hub_polar) * rho_shaft
                     + coupling_polar) * 1e-6

    # The member that turns at the output, off the same shapes again.  Which
    # one that is is the whole difference between the two topologies, and it
    # is not a small one: a carrier is a plate on the axis, a housing is most
    # of the gearbox's mass at its largest radius.
    if spec.output_member is OutputMember.CARRIER:
        rb = bores.hub / 2.0
        flange_polar = (_polar(plate_r, 0.0, spec.output_flange_thickness)
                        + _polar(spec.hub_diameter / 2.0, rb,
                                 spec.plate_thickness + spec.boss_protrusion)
                        - _polar(rb, 0.0, spec.output_flange_thickness)
                        + _lip_polar(bores, "output_flange"))
        pins_polar = spec.output_pin_count * _polar_off_axis(
            output_pin_d / 2.0, spec.output_bolt_circle_radius, spec.output_pin_length)
        output_inertia = ((flange_polar * rho_house + pins_polar * rho_pin) * 1e-6
                          + _rollers_on_circle(bearings, "Output pin roller",
                                               spec.output_bolt_circle_radius))
    else:
        R = spec.pin_circle_radius
        outline = spec.housing_polar_area_moment       # mm^4 per mm of length
        rp = spec.pin_radius
        # A pocket is the outer half of a pin's circle, cut into the barrel's
        # bore: its area at R, plus its first moment outward, plus its own.
        pocket = (0.5 * math.pi * rp ** 2 * R ** 2 + 4.0 * R * rp ** 3 / 3.0
                  + math.pi * rp ** 4 / 4.0)
        # The housing that turns is the barrel and its closed top in one, and
        # one plate under it: see the volumes above.
        barrel_polar = ((outline - math.pi * R ** 4 / 2.0
                         + (1.0 if spec.ring_pins_integral else -1.0)
                         * spec.pin_count * pocket) * spec.barrel_height
                        - _cup_holes_polar(spec)
                        + (outline - _polar(bores.input_plate / 2.0, 0.0, 1.0))
                        * spec.housing_plate_thickness
                        + _lip_polar(bores, "housing")
                        + outline * CUP_CEILING)
        plates_polar = _turning_plate_polar(spec, bores, outline)
        pins_polar = (0.0 if spec.ring_pins_integral else spec.pin_count
                      * _polar_off_axis(ring_pin_d / 2.0, R, spec.ring_pin_length))
        output_inertia = (((barrel_polar + plates_polar) * rho_house
                           + pins_polar * rho_pin) * 1e-6
                          # the tie bolts and the dowels, as their masses where
                          # they are - and only they: the motor's screws are in
                          # the base, which is what stands still on this drive
                          + tie_bolt_mass_g(spec) / 1000.0
                          * sum(x * x + y * y for x, y in spec.tie_bolt_centres)
                          + dowels_mass / 1000.0 * spec.housing_bolt_radius ** 2
                          + (insert_mass_g(spec, spec.output_insert,
                                           spec.output_bolt_size) / 1000.0
                             * sum(x * x + y * y
                                   for x, y in spec.output_face_bolt_centres)
                             if spec.output_insert is not None else 0.0)
                          + _rollers_on_circle(bearings, "Ring pin roller", R))
    input_inertia = shaft_inertia + reflected + output_inertia / spec.ratio ** 2

    # ---- unbalance ----------------------------------------------------------
    # Everything in a disc's plane whose centre is at E: the disc, its cam
    # bearing, and the cam itself.  The cam is part of the shaft, but a circle
    # centred at E with the shaft's own section inside it puts its *whole*
    # area's first moment at E - the shaft's share sits on the axis and adds
    # nothing - so it throws the same force a loose disc of its size would.
    m_cam_kg = (math.pi * (spec.cam_diameter / 2.0) ** 2 * t
                * _MM3_TO_CM3 * rho_shaft / 1000.0)
    orbiting_kg = m_disc_kg + m_cam_bearing_kg + m_cam_kg
    omega = spec.input_rpm * 2.0 * math.pi / 60.0
    single = orbiting_kg * (spec.eccentricity / 1000.0) * omega ** 2  # N, one plane
    phases = np.asarray(spec.disc_phases)
    resultant = float(np.hypot(np.cos(phases).sum(), np.sin(phases).sum()))
    force = single * resultant
    # Evenly phased discs cancel the force but leave a couple: equal and opposite
    # forces a disc pitch apart.  N in, mm arms, so the sum is already Nmm.
    pitch = t + spec.disc_gap
    offsets = (np.arange(n) - (n - 1) / 2.0) * pitch
    couple = float(np.hypot((single * np.cos(phases) * offsets).sum(),
                            (single * np.sin(phases) * offsets).sum()))

    # ---- disc web -----------------------------------------------------------
    hole_r = spec.output_hole_diameter / 2.0
    bore_r = bores.disc / 2.0
    p = prof.profile_from_spec(spec, n=4000)
    web_inner = spec.output_bolt_circle_radius - hole_r - bore_r
    web_outer = p.root_radius - (spec.output_bolt_circle_radius + hole_r)
    web_side = (2.0 * spec.output_bolt_circle_radius
                * math.sin(math.pi / spec.output_pin_count)) - 2.0 * hole_r
    min_web = min(web_inner, web_outer, web_side)

    # the pin's share of torque has to shear out through the thinnest ligament
    # on both sides of the hole
    from ..core.kinematics import output_loads, output_sweep_angles
    peak = 0.0
    # over the output stage's own period, which is about twice a lobe pitch -
    # the window this used to sweep, and so half the cycle the peak is in
    for phi in output_sweep_angles(spec.lobes, spec.output_pin_count, 48):
        f = output_loads(spec, float(phi), spec.output_torque_Nm * 1000.0 / n).forces
        if f.size:
            peak = max(peak, float(f.max()))
    shear_area = max(2.0 * min_web * t, 1e-9)
    web_shear = peak / shear_area
    hole_bearing = peak / max(spec.output_pin_diameter * t, 1e-9)

    return MassResult(
        disc_mass_g=disc_mass,
        disc_volume_cm3=disc_volume * _MM3_TO_CM3,
        total_mass_g=total,
        housing_mass_g=housing_mass,
        plates_mass_g=plates_mass,
        pins_mass_g=pins_mass,
        shaft_mass_g=shaft_mass,
        flange_mass_g=flange_mass,
        end_cap_mass_g=cap_mass,
        rotor_hub_mass_g=hub_mass,
        spacers_mass_g=spacers_mass,
        bearings_mass_g=bearings_mass,
        fasteners_mass_g=fasteners_mass,
        coupling_mass_g=coupling_mass,
        mount_mass_g=mount_mass,
        disc_inertia_kg_mm2=disc_inertia,
        reflected_inertia_kg_mm2=reflected,
        shaft_inertia_kg_mm2=shaft_inertia,
        output_inertia_kg_mm2=output_inertia,
        input_inertia_kg_mm2=input_inertia,
        unbalance_force_N=force,
        unbalance_couple_Nmm=couple,
        orbiting_mass_g=orbiting_kg * 1000.0,
        balanced=resultant < 1e-9,
        web_shear_MPa=web_shear,
        web_shear_allow_MPa=spec.disc_mat.shear_allow_MPa,
        hole_bearing_MPa=hole_bearing,
        min_web_mm=min_web,
    )
