"""ISO 286 limits for the holes a drawing puts a fit on, and which fit a seat takes.

A bearing seat is bored to the bearing's outside and nothing more is said in the
geometry - :class:`~cycloidgen.analysis.bearings.SeatBores` leaves the fit to
"a drawing note rather than a millimetre this app invents".  This is that note.

Two halves, and only the second is a judgement:

- **The limits** of a tolerance class are the standard's, and they are worked
  out here the way ISO 286-1 defines them - a grade's width, ``IT``, and a
  fundamental deviation that places it - rather than copied out of the class
  tables in ISO 286-2.  ``tests/test_fits.py`` holds the result to those
  tables, so a mistyped grade shows up as a wrong class and not as a quietly
  wrong drawing.
- **Which class** a seat takes follows the rolling-bearing makers' housing
  tables, and what decides it is a fact the drive already knows: whether the
  load turns relative to the ring.  An outer ring whose load turns round it
  rolls round the bore and walks unless it is held by interference; one whose
  load stands still can be a sliding fit.  In a cycloidal drive every internal
  load turns with the crank, so every seat in the housing has a turning load on
  its outer ring - including the shaft supports, which in most machines are the
  textbook stationary case.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["HOLE_CLASSES", "LIGHT_LOAD", "SeatFit", "hole_limits", "housing_fit",
           "nominal_range", "tolerance_unit"]

#: Upper limits of the ISO 286 nominal size ranges, millimetres.  A diameter
#: belongs to the range it is *over the previous limit and up to and including*
#: this one.  From 3 mm: no seat or register here is smaller.
_LIMITS: tuple[float, ...] = (3, 6, 10, 18, 30, 50, 80, 120, 180, 250, 315, 400)

#: Standard tolerance grades, micrometres, one per range above (ISO 286-1).
#: Not the bare formula: the standard rounds each grade, and the values it
#: publishes are the ones a shop's gauges are made to.  The formula is the test.
IT: dict[int, tuple[int, ...]] = {
    6: (8, 9, 11, 13, 16, 19, 22, 25, 29, 32, 36),
    7: (12, 15, 18, 21, 25, 30, 35, 40, 46, 52, 57),
    8: (18, 22, 27, 33, 39, 46, 54, 63, 72, 81, 89),
}

#: The fundamental deviation of an ``n`` shaft, micrometres: its lower limit,
#: ``5 D^0.34`` rounded.  The ``N`` hole is defined from it.
_EI_N: tuple[int, ...] = (8, 10, 12, 15, 17, 20, 23, 27, 31, 34, 37)

#: The hole classes this module can state.  H for a sliding fit, M and N for
#: the transition fits a turning outer-ring load takes.
HOLE_CLASSES: tuple[str, ...] = ("H7", "H8", "M7", "N7")

#: A turning outer-ring load at or below this share of the bearing's dynamic
#: rating is *light*, and takes M7 rather than N7.  The makers do not agree on
#: where light ends - SKF puts it at 0.05 C, the DIN tables Inventor reprints at
#: C/P above 15, 0.067 C - so this is the lower of the two: a load both call
#: light.  Neither has the drive needing P7, which is for shock loads in a
#: one-piece housing, and nothing here knows about shock.
LIGHT_LOAD = 0.05


def nominal_range(diameter: float) -> int:
    """Which ISO 286 range ``diameter`` falls in, as an index into the tables."""
    if not _LIMITS[0] < diameter <= _LIMITS[-1]:
        raise ValueError(f"{diameter:g} mm is outside the {_LIMITS[0]:g} to "
                         f"{_LIMITS[-1]:g} mm this module has limits for")
    return next(i for i, upper in enumerate(_LIMITS[1:]) if diameter <= upper)


def tolerance_unit(index: int) -> float:
    """ISO 286-1's tolerance unit ``i`` for a range, micrometres.

    ``0.45 D^(1/3) + 0.001 D`` at the geometric mean of the range's limits.  A
    grade is a fixed multiple of it before rounding - 10 for IT6, 16 for IT7, 25
    for IT8 - which is how the table above is checked.
    """
    d = math.sqrt(_LIMITS[index] * _LIMITS[index + 1])
    return 0.45 * d ** (1.0 / 3.0) + 0.001 * d


def hole_limits(diameter: float, tolerance_class: str) -> tuple[int, int]:
    """``(upper, lower)`` deviation of a hole, micrometres, from its nominal.

    ISO 286-1's rules for the four classes here: an H hole starts at the
    nominal; for M and N up to grade 8 the upper deviation is the matching
    shaft's lower one with its sign turned, plus ``delta`` - the step from the
    grade below, which keeps a hole-basis and a shaft-basis fit of the same
    name the same fit.  For M that shaft deviation is itself ``IT7 - IT6``, so
    M7 always ends at the nominal.
    """
    if tolerance_class not in HOLE_CLASSES:
        raise ValueError(f"no limits for {tolerance_class!r}; this module knows "
                         + ", ".join(HOLE_CLASSES))
    i = nominal_range(diameter)
    letter, grade = tolerance_class[0], int(tolerance_class[1:])
    width = IT[grade][i]
    if letter == "H":
        return width, 0
    delta = width - IT[grade - 1][i]
    ei_shaft = IT[7][i] - IT[6][i] if letter == "M" else _EI_N[i]
    upper = -ei_shaft + delta
    return upper, upper - width


@dataclass(frozen=True)
class SeatFit:
    """The tolerance class a bearing seat is bored to, and why that one."""

    tolerance_class: str
    #: Micrometres from the nominal - the bearing's outside diameter.
    upper_um: int
    lower_um: int
    #: One clause for the drawing, saying what decided it.
    why: str

    def limits_mm(self, nominal: float) -> tuple[float, float]:
        """The largest and smallest the bore may be, millimetres."""
        return nominal + self.upper_um / 1000.0, nominal + self.lower_um / 1000.0


def housing_fit(diameter: float, turning_rpm: float, load_N: float,
                rating_N: float) -> SeatFit:
    """The fit for an outer ring seated in a bore, from what its load does.

    ``turning_rpm`` is how fast the load turns relative to the part the bore is
    in; ``load_N`` against ``rating_N`` (the bearing's dynamic rating, ``C``)
    is how hard it is pressed.  A load that turns, however slowly, rolls the
    ring round its bore, so the test is whether it turns at all.
    """
    if turning_rpm <= 0.0:
        cls = "H7"
        why = ("the load stands still on the outer ring, so it may be a "
               "sliding fit")
    elif rating_N > 0.0 and load_N <= LIGHT_LOAD * rating_N:
        cls = "M7"
        why = (f"the load turns round the outer ring at {turning_rpm:.0f} rpm, "
               f"lightly ({load_N / rating_N:.3f} C): a transition fit keeps it "
               f"from creeping")
    else:
        cls = "N7"
        share = f"{load_N / rating_N:.2f} C" if rating_N > 0.0 else "unrated"
        why = (f"the load turns round the outer ring at {turning_rpm:.0f} rpm "
               f"({share}): it has to be held by interference or it creeps")
    upper, lower = hole_limits(diameter, cls)
    return SeatFit(cls, upper, lower, why)
