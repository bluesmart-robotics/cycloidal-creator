"""Bill of materials: what you actually have to make or buy.

The drawings say what the parts are; this says how many, out of what, how heavy,
and - for the bought items - what to order.  It is generated from the same
analysis as everything else, so the bearing designations here are the ones the
sizing study picked, not a guess made later.

The multi-disc trap shows up here too: when the discs are different parts they
get separate lines with their own hole-pattern offsets, because a BOM that says
"cycloidal disc x2" is an invitation to make the same part twice and jam the
drive.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path

from ..analysis import DesignAnalysis
from ..analysis.bearings import pin_diameters, seat_bores
from ..core.fasteners import nut_volume, screw_volume
from ..core.spec import OutputMember

__all__ = ["BomItem", "bom_items", "write_bom_csv"]


@dataclass(frozen=True)
class BomItem:
    """One line of the bill of materials."""

    part: str
    quantity: int
    material: str
    size: str
    mass_each_g: float
    source: str          # "make" or "buy"
    note: str = ""
    #: The assembly parts this line orders, by the names the STEP file and
    #: the 3D view give them - which is what lets a part picked in the view
    #: say what it is.  Empty for a line that is not drawn: a motor bolt, a
    #: dowel that is part of the carrier's body in the model.  Not in the CSV
    #: or the report; it is a cross-reference, not a column.
    assembly: tuple[str, ...] = ()

    @property
    def mass_total_g(self) -> float:
        return self.quantity * self.mass_each_g


def _pin_mass(diameter: float, length: float, density_g_cm3: float) -> float:
    volume_mm3 = math.pi * (diameter / 2.0) ** 2 * length
    return volume_mm3 * 1e-3 * density_g_cm3


def bom_items(a: DesignAnalysis) -> list[BomItem]:
    """Every part in the assembly, made and bought."""
    s, m = a.spec, a.mass
    bores = seat_bores(s, a.bearings)
    items: list[BomItem] = []

    # ---- discs --------------------------------------------------------------
    if s.disc_count == 1 or s.discs_are_identical:
        items.append(BomItem(
            part="Cycloidal disc", quantity=s.disc_count, material=s.disc_material,
            size=f"{2 * s.disc_outer_radius:.1f} dia x {s.disc_thickness:g} mm",
            mass_each_g=m.disc_mass_g, source="make",
            note=("all discs identical" if s.disc_count > 1 else "")))
    else:
        for i, phase in enumerate(s.disc_hole_phases, start=1):
            degrees = math.degrees(phase)
            note = ("reference hole pattern - NOT interchangeable with the others"
                    if abs(degrees) < 1e-9 else
                    f"hole pattern rotated {degrees:+.3f} deg - NOT interchangeable")
            items.append(BomItem(
                part=f"Cycloidal disc {i}", quantity=1, material=s.disc_material,
                size=f"{2 * s.disc_outer_radius:.1f} dia x {s.disc_thickness:g} mm",
                mass_each_g=m.disc_mass_g, source="make", note=note))

    # ---- made parts ---------------------------------------------------------
    items.append(BomItem(
        part="Ring housing", quantity=1, material=s.housing_material,
        size=f"{2 * s.housing_outer_radius:.1f} dia x {s.barrel_height:g} mm",
        mass_each_g=m.housing_mass_g, source="make",
        note=f"{s.pin_count} "
             + ("pins formed with it" if s.ring_pins_integral else "pin pockets")
             + f" on a {2 * s.pin_circle_radius:g} mm circle; "
             + ("bolted down - this is the fixed member"
                if s.output_member is OutputMember.CARRIER else
                "this is the output member and it turns")))

    items.append(BomItem(
        part="Housing end plate", quantity=2, material=s.housing_material,
        size=f"{2 * s.housing_outer_radius:.1f} dia x {s.plate_thickness:g} mm",
        mass_each_g=m.plates_mass_g / 2.0, source="make",
        note=f"input side bored {bores.input_plate:.1f} for the "
             + ("second output bearing" if s.ground_frame_fitted
                else "shaft support")
             + f", output side {bores.output_plate:.1f} for the output bearing "
             f"- they are not interchangeable"
             + (f"; the input side also carries the {s.output_bolt_count} "
                f"output bolts the load hangs on" if s.ground_frame_fitted
                and s.output_bolt_count else "")))

    items.append(BomItem(
        part="Output flange / carrier", quantity=1, material=s.housing_material,
        size=f"{2 * (s.output_bolt_circle_radius + s.output_pin_diameter):.1f} dia "
             f"x {s.output_flange_thickness:g} mm",
        mass_each_g=m.flange_mass_g, source="make",
        note=f"{s.output_pin_count} pin seats on a "
             f"{2 * s.output_bolt_circle_radius:g} mm circle"
             + ("; one part with the base the drive is bolted down by, on the "
                f"end of the boss - {2 * s.housing_outer_radius:.1f} dia x "
                f"{s.plate_thickness:g} mm" if s.ground_frame_fitted else "")))

    if s.ground_frame_fitted:
        items.append(BomItem(
            part="End cap", quantity=1, material=s.housing_material,
            size=f"{2 * (s.output_bolt_circle_radius + s.output_pin_diameter):.1f} "
                 f"dia x {s.output_flange_thickness:g} mm",
            mass_each_g=m.end_cap_mass_g, source="make",
            note=f"the far end of the output pins - clearance holes, not press "
                 f"fits, on the same {2 * s.output_bolt_circle_radius:g} mm "
                 f"circle, plus a {s.hub_diameter:.1f} mm boss for the "
                 f"input-side output bearing. It goes inside the barrel, so it "
                 f"is fitted before the housing"))

    bore = s.motor_bore
    items.append(BomItem(
        part="Eccentric input shaft", quantity=1, material=s.shaft_material,
        size=f"{s.input_shaft_diameter:g} mm dia, {s.disc_count} cam(s) "
             f"{s.cam_diameter:g} mm at {s.eccentricity:g} mm offset",
        mass_each_g=m.shaft_mass_g, source="make",
        note="cams phased "
             + ", ".join(f"{math.degrees(p):.0f}" for p in s.disc_phases) + " deg"
             + (f"; a {s.input_flat[2]:g} mm deep flat on the input end, "
                f"{s.input_flat[1] - s.input_flat[0]:g} mm long, for a "
                f"coupling's set screw" if s.input_flat else "")
             + (f"; bored {bore[2]:g} mm{' D' if bore[3] > 0 else ''}, "
                f"{bore[1] - bore[0]:.1f} mm deep from the motor's end, onto the "
                f"{s.motor_face.name}'s shaft" if bore is not None else "")))

    # ---- what locates the shaft, and keeps the discs apart -------------------
    # Made rather than bought: a sleeve a millimetre and a half long to a given
    # bore and outside is not a catalogue part, and a washer between two discs
    # is cut from what the discs are.
    def _ring_mass(spacer, density: float) -> float:
        return (math.pi * ((spacer.outer / 2.0) ** 2 - (spacer.bore / 2.0) ** 2)
                * spacer.length / 1000.0 * density)

    washers = [w for w in bores.spacers if w.name.startswith("washer")]
    for sp in bores.spacers:
        if sp in washers:
            continue
        items.append(BomItem(
            part=sp.label, quantity=1, material=s.shaft_material,
            size=f"{sp.bore:g} bore x {sp.outer:g} x {sp.length:g} mm",
            mass_each_g=_ring_mass(sp, s.shaft_mat.density_g_cm3), source="make",
            note="slides on the shaft between a cam's face and the support's "
                 "inner ring; it is what locates the shaft along its axis"))
    if washers:
        w = washers[0]
        items.append(BomItem(
            part="Disc washer", quantity=len(washers), material=s.disc_material,
            size=f"{w.bore:g} bore x {w.outer:g} x {w.length:g} mm",
            mass_each_g=_ring_mass(w, s.disc_mat.density_g_cm3), source="make",
            note="between neighbouring discs, holding the gap the stack was "
                 "designed with; a bearing polymer or bronze is the better buy "
                 "than the disc material it is quoted in"))

    # ---- bought pins --------------------------------------------------------
    # Both at their own length and their own diameter, neither of which is what
    # this used to quote.  A dowel ordered wrong is not a fit that can be
    # adjusted at assembly, it is the wrong part - and the diameter was the
    # worse of the two: under a roller the pin *is* the roller's bore, so a
    # rollered 15:1 was ordering 14 mm dowels for an 8 mm hole.
    ring_pin_d, output_pin_d = pin_diameters(s)

    # Nothing to order when the pins are the housing.  A line here for a part
    # that arrives already attached to another line is how a build ends up with
    # twenty-two dowels and nowhere to put them.
    if not s.ring_pins_integral:
        ring_pin_mass = _pin_mass(ring_pin_d, s.ring_pin_length,
                                  s.pin_mat.density_g_cm3)
        items.append(BomItem(
            part="Ring pin (dowel)", quantity=s.pin_count,
            material=s.pin_material,
            size=f"{ring_pin_d:g} mm dia x {s.ring_pin_length:g} mm",
            mass_each_g=ring_pin_mass, source="buy",
            note=("carries a roller" if ring_pin_d < 2 * s.pin_radius else
                  "free to rotate" if s.ring_pins_are_rollers else "fixed")
                 + ", captive between the two end plates"))

    output_pin_mass = _pin_mass(output_pin_d, s.output_pin_length,
                                s.pin_mat.density_g_cm3)
    items.append(BomItem(
        part="Output pin (dowel)", quantity=s.output_pin_count,
        material=s.pin_material,
        size=f"{output_pin_d:g} mm dia x {s.output_pin_length:g} mm",
        mass_each_g=output_pin_mass, source="buy",
        note=(f"carries a roller running in {s.output_hole_diameter:.3f} mm holes"
              if output_pin_d < s.output_pin_diameter else
              f"runs in {s.output_hole_diameter:.3f} mm holes")))

    # ---- fasteners ----------------------------------------------------------
    # Named by size and length when the hole is one the table knows - what a
    # supplier is actually asked for - and by the hole when it is not.
    bolt = s.tie_bolt_size
    if s.housing_bolt_count and bolt is not None and s.tie_screw_length is not None:
        # The two halves of `tie_bolt_mass_g`, which is what the mass model
        # adds up - so the lines and the total cannot disagree.
        density = s.pin_mat.density_g_cm3 / 1000.0
        screw_g = screw_volume(bolt, s.tie_screw_length) * density
        nut_g = nut_volume(bolt) * density
        items.append(BomItem(
            part="Tie bolt", quantity=s.housing_bolt_count, material=s.pin_material,
            size=f"{bolt.name} x {s.tie_screw_length:g} socket head cap screw, "
                 f"ISO 4762",
            mass_each_g=screw_g, source="buy",
            note=f"head on the output end plate, through it, the barrel and the "
                 f"input end plate, on a {2 * s.housing_bolt_radius:.1f} mm "
                 f"circle; {s.tie_bolt_length:.0f} mm of grip"))
        items.append(BomItem(
            part="Tie bolt nut", quantity=s.housing_bolt_count,
            material=s.pin_material, size=f"{bolt.name} hexagon nut, ISO 4032",
            mass_each_g=nut_g, source="buy",
            note="on the input end plate's outside face"))
    elif s.housing_bolt_count:
        items.append(BomItem(
            part="Tie bolt", quantity=s.housing_bolt_count, material=s.pin_material,
            size=f"{s.housing_bolt_diameter:g} mm clearance, about "
                 f"{s.tie_bolt_length:.0f} mm under the head",
            mass_each_g=0.0, source="buy",
            note=f"through both end plates into the barrel, on a "
                 f"{2 * s.housing_bolt_radius:.1f} mm circle"))

    if s.ground_frame_fitted and s.output_bolt_count:
        items.append(BomItem(
            part="Output bolt", quantity=s.output_bolt_count,
            material=s.pin_material,
            size=f"{s.output_bolt_diameter:g} mm clearance",
            mass_each_g=0.0, source="buy",
            note=f"what the driven machine attaches to, through the input end "
                 f"plate on a {2 * s.output_face_bolt_radius:.1f} mm circle - "
                 f"the tie bolts' own circle, half a pitch round from them"))

    if s.has_motor_face:
        frame = s.motor_face
        pattern = (f"{frame.bolt_span:g} mm square" if frame.square
                   else f"{frame.bolt_span:g} mm circle")
        face = ("the carrier's base" if s.motor_mounts_on_carrier
                else "the input end plate")
        fix = s.motor_fixing
        count = len(s.motor_bolt_centres)
        if fix is None:
            items.append(BomItem(
                part="Motor bolt", quantity=count, material=s.pin_material,
                size=f"{frame.bolt_diameter:g} mm clearance",
                mass_each_g=0.0, source="buy",
                note=f"{frame.name} pattern, {pattern}, into {face}"))
        else:
            density = s.pin_mat.density_g_cm3 / 1000.0
            how = (f"in from the drive's side into the motor's threads, heads in "
                   f"{fix.recess:.1f} mm counterbores {fix.recess_depth:.1f} mm deep "
                   f"in the inside of {face}" if fix.from_drive else
                   f"in from the motor's side through its flange, into nuts in "
                   f"hexagonal pockets in the inside of {face}")
            items.append(BomItem(
                part="Motor bolt", quantity=count, material=s.pin_material,
                size=f"{fix.size.name} x {fix.length:g} socket head cap screw, "
                     f"ISO 4762",
                mass_each_g=screw_volume(fix.size, fix.length) * density,
                source="buy", note=f"{frame.name} pattern, {pattern}; {how}"))
            if not fix.from_drive:
                items.append(BomItem(
                    part="Motor bolt nut", quantity=count, material=s.pin_material,
                    size=f"{fix.size.name} hexagon nut, ISO 4032",
                    mass_each_g=nut_volume(fix.size) * density, source="buy",
                    note=f"in its pocket in the inside of {face}"))

    # ---- what drives it -----------------------------------------------------
    # On the bill because it is on the drawing, and because whoever builds the
    # drive has to buy one - but weighed at nothing, since the gearbox's mass
    # is the gearbox's.
    if s.has_motor_face:
        frame = s.motor_face
        items.append(BomItem(
            part="Motor", quantity=1, material="-",
            size=f"{frame.name}, {frame.shaft_diameter:g} mm shaft",
            mass_each_g=0.0, source="buy",
            note="not part of the drive: drawn to check that it fits, chosen by "
                 "whoever runs it"))

    # ---- bearings -----------------------------------------------------------
    for choice in a.bearings:
        if choice.bearing is None or not choice.count:
            continue
        b = choice.bearing
        life = ("-" if choice.life_hours == float("inf")
                else f"L10 {choice.life_hours:,.0f} h")
        # Weighed where the mass model weighs it, which is where it goes in: a
        # named part too big for its seat is still ordered, and still listed so
        # the problem is on the sheet, but it is not in the assembled mass.
        weighed = choice.fitted and choice.fits
        items.append(BomItem(
            part=choice.role, quantity=choice.count, material="bearing steel",
            size=f"{b.designation} ({b.bore:g}x{b.outer:g}x{b.width:g})",
            mass_each_g=b.mass_g if weighed else 0.0, source="buy",
            note=f"{choice.seat}; {choice.load_N:.0f} N at "
                 f"{choice.speed_rpm:.0f} rpm, {life}"))

    return [replace(item, assembly=_assembly(item.part, s)) for item in items]


def _assembly(part: str, spec) -> tuple[str, ...]:
    """The assembly part names a bill-of-materials line stands for.

    By the line's own name, in one table, because that is the one place the
    two vocabularies meet; a test holds every drawn part to exactly one line.
    """
    if part == "Cycloidal disc":
        return tuple(f"disc_{i + 1}" for i in range(spec.disc_count))
    if part.startswith("Cycloidal disc "):
        return (f"disc_{part.rsplit(' ', 1)[1]}",)
    if part == "Eccentric cam bearing":
        return tuple(f"bearing_cam_{i + 1}" for i in range(spec.disc_count))
    return {
        "Ring housing": ("housing",),
        "Housing end plate": ("input_end_plate", "output_end_plate"),
        "Output flange / carrier": ("output_flange",),
        "End cap": ("end_cap",),
        "Eccentric input shaft": ("eccentric_shaft",),
        "Shaft spacer, input side": ("shaft_spacer_input",),
        "Shaft spacer, output side": ("shaft_spacer_output",),
        "Disc washer": ("washers",),
        "Ring pin (dowel)": ("ring_pins",),
        "Tie bolt": ("tie_bolts",),
        "Tie bolt nut": ("tie_nuts",),
        "Input shaft support": ("bearing_shaft_input", "bearing_shaft_output"),
        "Main output bearing": ("bearing_output_main", "bearing_output_input"),
        "Ring pin roller": ("bearing_ring_pins",),
        "Motor": ("motor", "motor_shaft"),
        "Motor bolt": ("motor_screws",),
        "Motor bolt nut": ("motor_nuts",),
        "Output pin roller": ("bearing_output_pins",),
    }.get(part, ())


def write_bom_csv(a: DesignAnalysis, path: str | Path) -> Path:
    """Write the bill of materials as CSV."""
    path = Path(path)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["Part", "Qty", "Material", "Size", "Mass each (g)",
                         "Mass total (g)", "Make/Buy", "Note"])
        for item in bom_items(a):
            writer.writerow([
                item.part, item.quantity, item.material, item.size,
                f"{item.mass_each_g:.2f}" if item.mass_each_g else "",
                f"{item.mass_total_g:.2f}" if item.mass_each_g else "",
                item.source, item.note])
        writer.writerow([])
        writer.writerow(["Assembled mass (g)", f"{a.mass.total_mass_g:.1f}"])
    return path
