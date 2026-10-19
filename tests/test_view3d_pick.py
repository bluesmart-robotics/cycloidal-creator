"""Clicking a part in the 3D tab says what it is.

Headless against the software view, which is the one that renders without a GL
context; the hardware view picks through VTK's own cell picker and is checked
in a real window, where there is one.  What these hold is the part a pixel
lands on, the tint the picked part takes, and the line of the bill of
materials the strip under the viewport shows for it.
"""
from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication

from cycloidgen.analysis import analyse
from cycloidgen.core.spec import preset
from cycloidgen.export.bom import bom_items
from cycloidgen.ui.view3d import Assembly3DTab, AssemblyView
from cycloidgen.viz.mesh import mesh_for_spec

_BUILT: list = []


@pytest.fixture(autouse=True)
def _widgets_do_not_outlive_the_test():
    yield
    from PySide6.QtCore import QEvent

    instance = QApplication.instance()
    while _BUILT:
        _BUILT.pop().deleteLater()
    if instance is not None:
        instance.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        instance.processEvents()


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def _painted_view(app, spec, view_name="top"):
    view = AssemblyView()
    _BUILT.append(view)
    view.resize(640, 480)
    view.set_spec(spec)
    view.set_standard_view(view_name)
    view.fit()
    view.grab()                       # paints, which is what a pick reads
    return view


def test_a_pixel_names_the_part_on_top_of_it(app):
    """Looking down the axis the shaft end is in the middle of the picture and
    nothing is in the corner."""
    view = _painted_view(app, preset(21))
    assert view.pick(320, 240) == "eccentric_shaft"
    assert view.pick(2, 2) is None


def test_a_click_picks_and_a_drag_does_not(app):
    """Both orbit on the left button, so the two are told apart by distance."""
    from PySide6.QtCore import QPoint, QPointF, Qt
    from PySide6.QtGui import QMouseEvent

    view = _painted_view(app, preset(21))
    seen = []
    view.part_picked.connect(seen.append)

    def press_release(start, end):
        for kind, at in ((QMouseEvent.MouseButtonPress, start),
                         (QMouseEvent.MouseButtonRelease, end)):
            event = QMouseEvent(kind, QPointF(*at), QPointF(view.mapToGlobal(
                QPoint(*at))), Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
            QApplication.sendEvent(view, event)

    press_release((320, 240), (321, 241))
    assert seen == ["eccentric_shaft"]
    press_release((320, 240), (360, 240))           # an orbit
    assert seen == ["eccentric_shaft"]


def test_the_picked_part_is_tinted_and_nothing_else_is(app):
    """Towards the accent, and only the part that was picked: the shaft end in
    the middle of the picture changes colour, the end plate round it does not."""
    from cycloidgen.ui import branding

    view = _painted_view(app, preset(21))
    middle = view.pick(320, 240)
    rim = next((x, 240) for x in range(160, 300)
               if view.pick(x, 240) == "input_end_plate")

    def pixel(at):
        return view.grab().toImage().pixelColor(*at)

    plain_middle, plain_rim = pixel((320, 240)), pixel(rim)
    view.set_selected(middle)
    tinted_middle, tinted_rim = pixel((320, 240)), pixel(rim)

    accent = branding.palette("light").accent
    target = [int(accent[i:i + 2], 16) for i in (1, 3, 5)]

    def distance(c):
        return sum((a - b) ** 2 for a, b in zip((c.red(), c.green(), c.blue()),
                                                 target, strict=True))

    assert distance(tinted_middle) < distance(plain_middle)
    assert tinted_rim == plain_rim


def test_the_strip_reads_the_line_on_the_bill_of_materials(app):
    """The picked part's line, in the bill's words: a plate is a "Housing end
    plate", two of them, and the info says which one was clicked."""
    spec = preset(21)
    analysis = analyse(spec)
    tab = Assembly3DTab()
    _BUILT.append(tab)
    tab.set_spec(spec)
    tab.set_analysis(analysis)

    tab.view.part_picked.emit("input_end_plate")
    text = tab._info.text()
    assert "Input end plate" in text and "Housing end plate" in text
    assert spec.housing_material in text
    assert tab.view._selected == "input_end_plate"

    tab.view.part_picked.emit(None)
    assert tab.view._selected is None
    assert "Click a part" in tab._info.text()


@pytest.mark.parametrize("output,rollers", [("output pin carrier", False),
                                            ("ring housing", True),
                                            ("output pin carrier", True)])
def test_every_drawn_part_is_on_exactly_one_line(output, rollers):
    """The cross-reference the strip reads, held for every part the view
    draws: one line each, and never two."""
    spec = preset(15)
    spec.output_member = output
    spec.ring_pins_are_rollers = spec.output_pins_are_rollers = rollers
    covered = [n for item in bom_items(analyse(spec)) for n in item.assembly]
    for part in mesh_for_spec(spec).parts:
        assert covered.count(part.name) == 1, part.name


def test_x_ray_sees_and_clicks_through_the_housing(app):
    """Translucent, and not what a click lands on: with X-ray on, a click on
    the input plate finds whatever is behind it."""
    view = _painted_view(app, preset(21))
    at = next((x, 240) for x in range(160, 300)
              if view.pick(x, 240) == "input_end_plate")
    plain = view.grab().toImage().pixelColor(*at)

    view.set_xray(True)
    see_through = view.grab().toImage().pixelColor(*at)
    behind = view.pick(*at)
    assert behind not in ("input_end_plate", "output_end_plate", "housing")
    assert see_through != plain


def test_x_ray_is_remembered(app, tmp_path, monkeypatch):
    from cycloidgen.ui.settings import ENV_VAR

    monkeypatch.setenv(ENV_VAR, str(tmp_path / "x.ini"))
    tab = Assembly3DTab()
    _BUILT.append(tab)
    tab._xray.setChecked(True)
    tab.save_state()
    again = Assembly3DTab()
    _BUILT.append(again)
    again.restore_state()
    assert again._xray.isChecked()


def test_a_measurement_reads_in_the_unit_the_window_reads_in(app):
    """The strip says what to click next, then the distance and its three
    components - in inches when the window is in inches, because a length read
    off the screen follows the preference like every other one."""
    from cycloidgen.units import unit

    tab = Assembly3DTab()
    _BUILT.append(tab)
    tab._measure.setChecked(True)        # the button exists either way
    tab._measured(None)
    assert "click a point" in tab._info.text()
    tab._measured(((0.0, 0.0, 0.0), None))
    assert "second point" in tab._info.text()

    tab._measured(((0.0, 0.0, 0.0), (30.0, 40.0, 0.0)))
    assert "<b>50.00 mm</b>" in tab._info.text()
    assert "x 30.00 mm" in tab._info.text() and "y 40.00 mm" in tab._info.text()

    tab.set_unit(unit("in"))
    assert "<b>1.9685 in</b>" in tab._info.text()

    tab._measure.setChecked(False)
    assert "Click a part" in tab._info.text()


def _with_motor():
    spec = preset(21)
    spec.motor_frame = "NEMA 17"       # its register lands the spigot on a bearing
    return spec


def test_a_clash_names_the_parts_it_is_about():
    """In the assembly's own names, so the view can tint them - and only for a
    clash: a check about stress or wear points at no part."""
    from cycloidgen.viz.clashes import clash_parts, finding_parts

    motor = analyse(_with_motor())
    parts = clash_parts(motor)
    assert {"input_end_plate", "bearing_shaft_input"} <= set(parts)

    ring = preset(21)
    ring.output_member = "ring housing"
    ring.housing_wall = 2.5                     # no plate screw's head fits
    thin = analyse(ring)
    (finding,) = [f for f in thin.report.findings
                  if f.code == "HOUSING_BOLT_CLASH"]
    assert set(finding_parts(thin, finding)) == {"housing", "output_end_plate",
                                                 "tie_bolts"}

    for f in motor.report.findings:
        if f.code in ("PRESSURE_ANGLE", "MASS", "LOST_MOTION"):
            assert finding_parts(motor, f) == ()


@pytest.mark.parametrize("output", ["output pin carrier", "ring housing"])
def test_every_part_a_clash_names_is_one_the_view_draws(output):
    spec = _with_motor()
    spec.output_member = output
    from cycloidgen.viz.clashes import clash_parts

    drawn = {p.name for p in mesh_for_spec(spec).parts}
    assert set(clash_parts(analyse(spec))) <= drawn


def test_the_clashes_box_tints_them_and_a_picked_part_says_why(app):
    spec = _with_motor()
    tab = Assembly3DTab()
    _BUILT.append(tab)
    tab.set_spec(spec)
    tab.set_analysis(analyse(spec))

    assert tab._clash_box.isEnabled() and "(1)" in tab._clash_box.text()
    assert not tab.view._flagged
    tab._clash_box.setChecked(True)
    assert {"input_end_plate", "bearing_shaft_input"} <= tab.view._flagged

    tab.view.part_picked.emit("input_end_plate")
    assert "SEAT_HAS_NO_SHOULDER" in tab._info.text()

    # Picked in the checks list, the same parts - with the box off.
    tab._clash_box.setChecked(False)
    tab.point_at("SEAT_HAS_NO_SHOULDER")
    assert "bearing_shaft_input" in tab.view._flagged
    tab.point_at("PRESSURE_ANGLE")
    assert not tab.view._flagged


def test_a_design_with_no_clash_has_nothing_to_show(app):
    spec = preset(21)
    tab = Assembly3DTab()
    _BUILT.append(tab)
    tab.set_spec(spec)
    tab.set_analysis(analyse(spec))
    assert not tab._clash_box.isEnabled()
    assert tab._clash_box.text() == "Clashes"
