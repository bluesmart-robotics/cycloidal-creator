"""Which parts a clash is about, so the 3D view can point at them.

A clash check finds two things that go into each other - a profile into its
pins, a hole into a bore, a bolt into a pocket, a motor's spigot into a
bearing - and says so in words.  The words name the parts; this names them the
way the assembly does, so the view can tint them.

From the checks rather than from the meshes.  Testing the meshes against each
other for overlap would find every bearing in its seat and every pin in its
pocket, because those are meant to touch, and telling a fit from a clash is
exactly what the checks already do with the geometry they were written for.
"""
from __future__ import annotations

__all__ = ["CLASH_CODES", "clash_parts", "finding_parts"]

#: The checks whose failure is one part occupying another's space, or a
#: feature breaking through the part it is cut in.
CLASH_CODES = frozenset({
    "PROFILE_INTERFERENCE", "PROFILE_SELF_INTERSECT", "PIN_OVERLAP",
    "HOLE_HITS_BORE", "HOLE_BREAKS_RIM", "OUTPUT_HOLES_OVERLAP",
    "ECCENTRIC_TIGHT", "HOUSING_BOLT_CLASH", "OUTPUT_BOLT_CLASH",
    "MOTOR_FACE_CLASH", "BEARING_DOES_NOT_FIT", "SEAT_HAS_NO_SHOULDER",
    "PLATE_DOWEL_CLASH",
})

#: Where a bearing that will not go in would have gone, by its role: the parts
#: to look at, since the bearing itself is not drawn.  Both of them - what it
#: sits in and what it sits on - because the check fails either way: an
#: outside too big for its seat, or a bore standing off its journal, which is
#: the cam or the shaft or the boss and not the seat at all.
_SEAT_OF_ROLE = {
    "Eccentric cam bearing": ("discs", "eccentric_shaft"),
    "Input shaft support": ("input_seat", "output_flange", "eccentric_shaft"),
    "Main output bearing": ("output_end_plate", "input_main_seat",
                            "output_flange", "main_journal"),
    "Ring pin roller": ("ring_pins",),
    "Output pin roller": ("output_flange",),
}


def finding_parts(analysis, finding) -> tuple[str, ...]:
    """The assembly parts ``finding`` is about, or nothing for a non-clash."""
    spec = analysis.spec
    discs = tuple(f"disc_{i + 1}" for i in range(spec.disc_count))
    code = finding.code
    if code not in CLASH_CODES:
        return ()
    if code in ("PROFILE_SELF_INTERSECT", "HOLE_HITS_BORE", "HOLE_BREAKS_RIM",
                "OUTPUT_HOLES_OVERLAP"):
        return discs
    if code == "PROFILE_INTERFERENCE":
        return (*discs, "housing" if spec.ring_pins_integral else "ring_pins")
    if code == "PIN_OVERLAP":
        return ("housing",) if spec.ring_pins_integral else ("ring_pins", "housing")
    if code == "ECCENTRIC_TIGHT":
        return (*discs, "eccentric_shaft")
    # On a ring drive the input end plate is the housing's own closed top.
    top = "housing" if spec.ground_frame_fitted else "input_end_plate"
    if code == "HOUSING_BOLT_CLASH":
        return tuple(dict.fromkeys(("housing", top, "output_end_plate", "tie_bolts")))
    if code == "OUTPUT_BOLT_CLASH":
        return (top,)
    if code == "MOTOR_FACE_CLASH":
        return ("output_flange",) if spec.motor_mounts_on_carrier else (
            "input_end_plate",)
    if code == "PLATE_DOWEL_CLASH":
        return ("plate_dowels", "output_end_plate" if spec.ground_frame_fitted
                else "input_end_plate")
    if code == "PLATES_NOT_LOCATED":
        return tuple(dict.fromkeys(("housing", top, "output_end_plate")))
    if code == "SEAT_HAS_NO_SHOULDER":
        from ..analysis.bearings import seat_bores

        found: list[str] = []
        for missing in seat_bores(spec, analysis.bearings).unshouldered:
            found += [missing.host, missing.bearing]
        return tuple(dict.fromkeys(found))
    # BEARING_DOES_NOT_FIT: "<role>: <why>."  The bearing is not drawn, so the
    # seat it would have gone in is what there is to point at.
    role = finding.message.split(":", 1)[0]
    out: list[str] = []
    for seat in _SEAT_OF_ROLE.get(role, ()):
        if seat == "discs":
            out += discs
        elif seat == "input_seat":
            out.append("end_cap" if spec.ground_frame_fitted else "input_end_plate")
        elif seat == "input_main_seat":
            if spec.ground_frame_fitted:
                out.append("housing")
        elif seat == "main_journal":
            if spec.ground_frame_fitted:
                out.append("end_cap")          # the input side's boss
        else:
            out.append(seat)
    return tuple(out)


def clash_parts(analysis) -> dict[str, list]:
    """Every part a clash in ``analysis`` is about, with the findings that say so."""
    out: dict[str, list] = {}
    for finding in analysis.report.findings:
        for name in finding_parts(analysis, finding):
            out.setdefault(name, []).append(finding)
    return out
