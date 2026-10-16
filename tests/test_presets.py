"""The named presets: the ratios, and the drives built round a motor."""
from __future__ import annotations

import pytest

from cycloidgen.__main__ import main
from cycloidgen.analysis import analyse
from cycloidgen.analysis.motor import COMFORTABLE_MARGIN
from cycloidgen.core.motor import MOTOR_FRAMES
from cycloidgen.core.presets import PRESETS, by_key, matching
from cycloidgen.core.spec import NO_MOTOR, preset

MOTOR_PRESETS = [p for p in PRESETS if p.group != "By ratio"]

#: What a motor preset may not be told: anything about its motor, how it is
#: mounted or held together, its bearings, or a part that will not last.  The
#: warnings its geometry carries anyway - pressure angle, load sharing,
#: compliance, lost motion on a printed drive - are its ratio preset's.
MUST_NOT = ("MOTOR_", "COUPLING_", "BEARING_DOES_NOT_FIT", "NO_BEARING_FITS",
            "SEAT_HAS_NO_SHOULDER", "PLATE", "HOUSING_BOLT", "OUTPUT_BOLT",
            "HERTZ_STRESS_RING", "HERTZ_STRESS_OUTPUT", "FATIGUE", "SHORT_BEARING_LIFE",
            "OVERTEMP", "RUNNING_HOT", "PV_LIMIT")


def test_every_key_is_unique_and_every_title_says_what_it_is():
    keys = [p.key for p in PRESETS]
    assert len(keys) == len(set(keys))
    titles = [p.title for p in PRESETS]
    assert len(titles) == len(set(titles))
    for p in MOTOR_PRESETS:
        frame = dict(p.settings)["motor_frame"]
        assert p.title.startswith(frame) and f"{p.ratio}:1" in p.title


@pytest.mark.parametrize("ratio", [10, 15, 21, 29, 39, 59])
def test_a_ratio_preset_is_the_geometry_it_always_was(ratio):
    p = by_key(f"ratio-{ratio}")
    assert p.spec() == preset(ratio)
    assert p.spec().motor_frame == NO_MOTOR
    assert matching(preset(ratio)) is p


@pytest.mark.parametrize("p", MOTOR_PRESETS, ids=lambda p: p.key)
def test_a_motor_preset_is_a_drive_its_motor_turns(p):
    spec = p.spec()
    assert spec.ratio == p.ratio
    assert spec.motor_frame in MOTOR_FRAMES and spec.motor_curve is not None
    a = analyse(spec)
    assert a.report.ok, [f.code for f in a.report.findings if f.severity.value == "error"]
    told = [f.code for f in a.report.findings
            if f.severity.value != "info" and f.code.startswith(MUST_NOT)]
    assert not told, told
    assert a.torque_capacity_with_clearance_Nm >= spec.output_torque_Nm
    assert a.motor.modelled and a.motor.margin >= COMFORTABLE_MARGIN


@pytest.mark.parametrize("p", [p for p in MOTOR_PRESETS if not p.spec().printed],
                         ids=lambda p: p.key)
def test_a_machined_preset_is_the_size_of_its_job(p):
    """Strong enough for its duty at the design search's own safety factor,
    stronger than its motor can make it work - a stall does not take the
    gearbox with it - and no wider than its motor needs.  They stood two to
    three times their motors' width once, on their ratio preset's geometry,
    carrying four to nine times their duty."""
    spec = p.spec()
    a = analyse(spec)
    assert a.pin_safety_factor_with_clearance >= 1.5
    assert a.torque_capacity_with_clearance_Nm >= a.motor.output_torque_ceiling_Nm
    assert 2.0 * spec.housing_outer_radius <= 1.65 * spec.motor_face.body_width


@pytest.mark.parametrize("p", MOTOR_PRESETS, ids=lambda p: p.key)
def test_its_summary_states_its_duty(p):
    """The line in the list is what somebody chooses by, so it has to be the
    drive's own numbers and not last week's."""
    spec = p.spec()
    assert f"{spec.output_torque_Nm:g} Nm" in p.summary
    assert f"{spec.output_rpm:g} rpm" in p.summary
    # and the size, where it states one
    if "mm housing" in p.summary:
        assert f" {2.0 * spec.housing_outer_radius:g} mm housing" in p.summary


def test_a_preset_is_a_fresh_design_every_time():
    p = by_key("nema17-printed-15")
    a = p.spec()
    a.output_torque_Nm = 9.0
    assert p.spec().output_torque_Nm != 9.0
    assert matching(a) is None and matching(p.spec()) is p


def test_the_command_line_lists_and_takes_them(capsys, tmp_path):
    assert main(["--list-presets"]) == 0
    listed = capsys.readouterr().out
    for p in PRESETS:
        assert p.key in listed
    assert main(["--preset", "nema17-printed-21", "--list-outputs"]) == 0
    with pytest.raises(SystemExit, match="no preset 'nope'"):
        main(["--preset", "nope", "--list-outputs"])
