"""ISO 286 limits, and the fit a bearing seat is given."""
from __future__ import annotations

import math

import pytest

from cycloidgen.core.fits import (
    _EI_N,
    _LIMITS,
    IT,
    LIGHT_LOAD,
    hole_limits,
    housing_fit,
    nominal_range,
    shaft_limits,
    tolerance_unit,
)

# ISO 286-2's class tables, as published: (upper, lower) in micrometres, one
# pair per range from "over 3 to 6" to "over 315 to 400".  Typed from the
# standard's tables, not from the rules the module applies - which is the point:
# a grade or a deviation mistyped in the module comes out as a class that
# disagrees with what the standard prints.
PUBLISHED = {
    "H7": [(12, 0), (15, 0), (18, 0), (21, 0), (25, 0), (30, 0), (35, 0),
           (40, 0), (46, 0), (52, 0), (57, 0)],
    "H8": [(18, 0), (22, 0), (27, 0), (33, 0), (39, 0), (46, 0), (54, 0),
           (63, 0), (72, 0), (81, 0), (89, 0)],
    "M7": [(0, -12), (0, -15), (0, -18), (0, -21), (0, -25), (0, -30), (0, -35),
           (0, -40), (0, -46), (0, -52), (0, -57)],
    "N7": [(-4, -16), (-4, -19), (-5, -23), (-7, -28), (-8, -33), (-9, -39),
           (-10, -45), (-12, -52), (-14, -60), (-14, -66), (-16, -73)],
}


# And ISO 286-2's shaft tables, the same way.
PUBLISHED_SHAFTS = {
    "h6": [(0, -8), (0, -9), (0, -11), (0, -13), (0, -16), (0, -19), (0, -22),
           (0, -25), (0, -29), (0, -32), (0, -36)],
    "m6": [(12, 4), (15, 6), (18, 7), (21, 8), (25, 9), (30, 11), (35, 13),
           (40, 15), (46, 17), (52, 20), (57, 21)],
    "n6": [(16, 8), (19, 10), (23, 12), (28, 15), (33, 17), (39, 20), (45, 23),
           (52, 27), (60, 31), (66, 34), (73, 37)],
}


def _middle(index: int) -> float:
    return (_LIMITS[index] + _LIMITS[index + 1]) / 2.0


@pytest.mark.parametrize("cls", sorted(PUBLISHED))
def test_the_rules_reproduce_the_published_classes(cls):
    worked = [hole_limits(_middle(i), cls) for i in range(len(_LIMITS) - 1)]
    assert worked == PUBLISHED[cls]


@pytest.mark.parametrize("cls", sorted(PUBLISHED_SHAFTS))
def test_the_rules_reproduce_the_published_shafts(cls):
    worked = [shaft_limits(_middle(i), cls) for i in range(len(_LIMITS) - 1)]
    assert worked == PUBLISHED_SHAFTS[cls]


def test_a_dowel_presses_into_n7_and_locates_in_h7():
    """What the plates' dowels are drilled for, read off the limits rather
    than taken on trust: an m6 pin in an N7 hole is tight at every size and
    every pair of limits, and in an H7 hole it is a transition fit whose
    loosest case is one grade 6 of clearance - the play a plate has on it."""
    for i in range(len(_LIMITS) - 1):
        d = _middle(i)
        pin_hi, pin_lo = shaft_limits(d, "m6")
        n_hi, _ = hole_limits(d, "N7")
        assert pin_lo - n_hi > 0                      # always interference
        h_hi, h_lo = hole_limits(d, "H7")
        assert h_hi - pin_lo == IT[6][i] > 0          # can be clear...
        assert pin_hi - h_lo > 0                      # ...or tight


@pytest.mark.parametrize(("grade", "factor"), [(6, 10), (7, 16), (8, 25)])
def test_every_grade_is_its_multiple_of_the_tolerance_unit(grade, factor):
    """The second way to a grade: ``IT = k i``, which the standard then rounds.
    Within a micrometre of it everywhere - a transposed digit is not."""
    for index, value in enumerate(IT[grade]):
        assert abs(value - factor * tolerance_unit(index)) <= 1.0, (grade, index)


def test_the_n_deviation_is_the_standards_formula():
    for index, value in enumerate(_EI_N):
        d = math.sqrt(_LIMITS[index] * _LIMITS[index + 1])
        assert abs(value - 5.0 * d ** 0.34) <= 1.0, index


def test_a_range_includes_its_upper_limit_and_not_its_lower():
    assert nominal_range(30.0) == nominal_range(18.0001)
    assert nominal_range(30.0001) == nominal_range(50.0)
    with pytest.raises(ValueError):
        nominal_range(3.0)
    with pytest.raises(ValueError):
        nominal_range(400.5)


def test_a_turning_load_is_held_by_interference():
    fit = housing_fit(19.0, 1000.0, 200.0, 1730.0)
    assert fit.tolerance_class == "N7"
    assert (fit.upper_um, fit.lower_um) == (-7, -28)
    assert fit.limits_mm(19.0) == pytest.approx((18.993, 18.972))


def test_a_light_turning_load_takes_the_transition_fit_and_no_lighter():
    rating = 10_000.0
    assert housing_fit(32.0, 50.0, LIGHT_LOAD * rating, rating).tolerance_class == "M7"
    assert housing_fit(32.0, 50.0, 1.01 * LIGHT_LOAD * rating,
                       rating).tolerance_class == "N7"


def test_a_load_that_stands_still_on_the_ring_is_a_sliding_fit():
    fit = housing_fit(19.0, 0.0, 5000.0, 1730.0)
    assert fit.tolerance_class == "H7"
    assert fit.lower_um == 0
