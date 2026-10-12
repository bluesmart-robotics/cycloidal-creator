"""A Qt call that returns nothing must leave `None` the way it found it.

PySide6 6.12.0 returns `None` from every such call without taking a reference
to it, so each call costs `None` one reference.  Before Python 3.12 that count
is an ordinary one, starting at about five thousand, and when it reaches zero
the interpreter aborts with "Fatal Python error: deallocating None" - no
traceback of ours, no exception, and in a test run nothing in the log but the
exit code.  The 3D view spends a few thousand calls on a frame, so the window
died within seconds; the suite died wherever the count happened to run out,
which moved from run to run.

`pyproject.toml` keeps 6.12 off the interpreters it breaks.  This is the check
that says whether a later release can be let back on: it measures the thing
itself, so it fails on the one call that leaks rather than on whichever
unrelated test happens to be running when the count hits zero.  On 3.12 and
later `None` is immortal, its count does not move, and this passes whatever
PySide6 does - which is also why those interpreters never noticed.
"""
from __future__ import annotations

import os
import sys

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("PySide6")

#: Calls of each kind.  Far below the five thousand that would end the process
#: on a leaking build, far above anything the loop itself could account for.
_CALLS = 500

#: What the loop may move the count by without a leak: whatever the interpreter
#: does with `None` on the side.  A leak moves it by `_CALLS`.
_SLACK = 25


def test_a_qt_call_that_returns_nothing_costs_none_nothing():
    import PySide6
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QColor, QImage, QPainter
    from PySide6.QtWidgets import QApplication, QLabel

    QApplication.instance() or QApplication([])
    label = QLabel()
    image = QImage(8, 8, QImage.Format_ARGB32)
    painter = QPainter(image)
    colour = QColor(10, 20, 30)
    try:
        for name, call in (
            ("QLabel.setText", lambda: label.setText("x")),
            ("QPainter.setPen", lambda: painter.setPen(Qt.NoPen)),
            ("QPainter.setBrush", lambda: painter.setBrush(colour)),
        ):
            before = sys.getrefcount(None)
            for _ in range(_CALLS):
                call()
            drift = sys.getrefcount(None) - before
            assert abs(drift) <= _SLACK, (
                f"PySide6 {PySide6.__version__} on Python "
                f"{sys.version.split()[0]}: {_CALLS} calls of {name} moved "
                f"None's reference count by {drift}")
    finally:
        painter.end()
        label.deleteLater()
