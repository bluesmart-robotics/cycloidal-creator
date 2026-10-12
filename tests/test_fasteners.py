"""The metric fastener table, and the two rules that read it."""
from __future__ import annotations

from itertools import pairwise

import pytest

from cycloidgen.core.fasteners import SIZES, hexagon, screw_length, size_for_hole


def test_a_hole_names_the_bolt_it_is_the_clearance_for():
    """The design states the hole; the bolt is read back off it.  A 4.5 mm hole
    is an M4's medium clearance, and anything short of an M5's 5.5 is still an
    M4 - the next size will not go through."""
    assert size_for_hole(4.5).name == "M4"
    assert size_for_hole(5.4).name == "M4"
    assert size_for_hole(5.5).name == "M5"
    assert size_for_hole(4.4).name == "M3"
    assert size_for_hole(2.8) is None


def test_the_table_is_in_order_and_every_size_goes_through_its_own_hole():
    for small, large in pairwise(SIZES):
        assert small.d < large.d and small.clearance < large.clearance
    for s in SIZES:
        assert s.d < s.clearance < s.head_d
        assert s.socket < s.head_d and s.socket_depth < s.head_h
        assert s.d < s.nut_s


def test_a_screw_clamps_the_grip_with_a_nut_and_two_threads_past_it():
    m4 = size_for_hole(4.5)
    assert screw_length(40.0, m4) == 45      # 40 + 3.2 + 1.4 = 44.6
    assert screw_length(47.0, m4) == 55      # 51.6: there is no 52
    assert screw_length(500.0, m4) is None   # a stud, not a screw


def test_a_hexagon_has_a_flat_where_it_is_asked_for():
    """A nut turned thirty degrees overhangs the rim by its corner instead of
    its flat, so the one rule both renderers share is which way it faces."""
    import math

    pts = hexagon(0.0, 0.0, 7.0, 0.0)
    xs = [p[0] for p in pts]
    assert max(xs) == pytest.approx(3.5)                 # half across flats
    assert max(math.hypot(*p) for p in pts) == pytest.approx(7.0 / math.sqrt(3.0))
