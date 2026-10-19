"""The momentary torque: what the drive survives for a moment, and what goes first.

Each limit is the rated-torque stress scaled to its yield, so each is read back
here by a second analysis run *at* that torque - the contact sweep, the load
sharing and the bearing loads all worked out there rather than scaled to it.
"""
from __future__ import annotations

import math

import pytest

from cycloidgen.analysis import analyse
from cycloidgen.analysis.bearings import select_bearings
from cycloidgen.analysis.fatigue import output_pin_fatigue
from cycloidgen.analysis.mechanics import analyse_contacts
from cycloidgen.analysis.peak import CONTACT_YIELD_ONSET
from cycloidgen.analysis.stiffness import analyse_stiffness
from cycloidgen.core.presets import by_key
from cycloidgen.core.spec import preset

DESIGNS = {
    "printed": lambda: by_key("nema17-printed-15").spec(),
    "machined": lambda: by_key("nema23-machined-21").spec(),
    "pla default": lambda: preset(21),
}


def _limit(a, starts):
    return next(lim for lim in a.peak.limits if lim.part.startswith(starts))


def _softer_yield(spec):
    return min(spec.pin_mat.sigma_yield_MPa, spec.disc_mat.sigma_yield_MPa)


@pytest.mark.parametrize("name", DESIGNS)
def test_the_ring_contact_yields_where_it_says(name):
    """At the torque it names, the peak pressure - off a contact sweep and a
    load-sharing solve both run there - is 1.67 times the softer yield."""
    spec = DESIGNS[name]()
    torque = _limit(analyse(spec), "ring contact").torque_Nm
    at = spec.model_copy(update={"output_torque_Nm": torque})
    pressure = analyse_contacts(at).max_pin_pressure_MPa * math.sqrt(
        max(analyse_stiffness(at).load_concentration, 1.0))
    assert pressure == pytest.approx(CONTACT_YIELD_ONSET * _softer_yield(spec), rel=2e-3)


@pytest.mark.parametrize("name", DESIGNS)
def test_the_output_pins_yield_where_they_say(name):
    spec = DESIGNS[name]()
    a = analyse(spec)
    contact = _limit(a, "output pins in their holes").torque_Nm
    at = spec.model_copy(update={"output_torque_Nm": contact})
    assert analyse_contacts(at).max_output_pressure_MPa == pytest.approx(
        CONTACT_YIELD_ONSET * _softer_yield(spec), rel=1e-6)
    bending = _limit(a, "output pins in bending").torque_Nm
    at = spec.model_copy(update={"output_torque_Nm": bending})
    assert output_pin_fatigue(at, a.thermal.temperature_C).alternating_MPa == \
        pytest.approx(spec.pin_mat.sigma_yield_MPa, rel=1e-6)


@pytest.mark.parametrize("name", DESIGNS)
def test_a_bearing_reaches_its_static_rating_where_it_says(name):
    """The bearing loads worked out again at that torque, against the C0 of the
    bearing the limit names."""
    spec = DESIGNS[name]()
    a = analyse(spec)
    for choice in a.bearings:
        lim = next((lim for lim in a.peak.limits
                    if lim.part.startswith(choice.role.lower())), None)
        if lim is None:
            continue
        at = spec.model_copy(update={"output_torque_Nm": lim.torque_Nm})
        contact = analyse_contacts(at)
        again = next(c for c in select_bearings(
            at, contact.eccentric_bearing_load_N, contact.max_output_force_N,
            ring_pin_load_N=contact.max_pin_force_N) if c.role == choice.role)
        assert again.load_N == pytest.approx(1000.0 * choice.bearing.C0_kN, rel=1e-6)


def test_the_main_output_bearing_is_not_a_torque_limit():
    """It carries what the machine hangs on the output, not the torque."""
    a = analyse(by_key("nema23-machined-21").spec())
    assert not any(lim.part.startswith("main output") for lim in a.peak.limits)
    assert a.peak.limits == tuple(sorted(a.peak.limits, key=lambda lim: lim.torque_Nm))
    assert a.peak.torque_Nm == a.peak.limits[0].torque_Nm


def test_a_motor_that_can_bend_the_drive_is_told():
    """A stalled stepper puts its standstill torque through the ratio; past
    the momentary torque the check says so, and names what goes first."""
    spec = by_key("nema17-printed-15").spec()
    a = analyse(spec)
    assert a.peak.motor_Nm == pytest.approx(
        spec.motor_curve.continuous_torque_at(0.0) * spec.ratio
        * a.efficiency.efficiency)
    assert not any(f.code == "MOTOR_OVERLOADS_DRIVE" for f in a.report.findings)
    spec.motor_holding_torque_Nm *= 1.5 * a.peak.torque_Nm / a.peak.motor_Nm
    found = [f for f in analyse(spec).report.findings
             if f.code == "MOTOR_OVERLOADS_DRIVE"]
    assert found and a.peak.first.part in found[0].message


def test_with_no_motor_there_is_nothing_to_overload_it():
    a = analyse(preset(21))
    assert math.isinf(a.peak.motor_Nm) and not a.peak.motor_can_bend_it
    assert math.isfinite(a.peak.torque_Nm) and a.peak.torque_Nm > 0.0
