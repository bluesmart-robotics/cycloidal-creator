"""The 3D tab: the assembled drive, turning under the same crank as the drawing.

Painted with ``QPainter`` over the draw list from :mod:`cycloidgen.viz.scene`.
No OpenGL, no Qt 3D, no scene graph.  That is a deliberate trade and worth
stating: a hardware path would render more triangles than this ever needs to,
and would also be the one part of the application that fails on a machine with
no GL driver, over a remote desktop, or in a headless test - which is precisely
where a mechanical tool gets used.  A cycloidal drive is a few thousand polygons;
software is fast enough, and it can be checked without a display.
"""
from __future__ import annotations

import html
import math
import traceback
from contextlib import suppress

import numpy as np
from PySide6.QtCore import QPoint, QPointF, QRect, QSize, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPolygonF
from PySide6.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QLayout,
    QMenu,
    QPushButton,
    QSizePolicy,
    QSlider,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ..core.spec import GearSpec
from ..viz.mesh import EDGE_SHADE, PART_GROUPS, Mesh, mesh_for_spec
from ..viz.scene import Camera, render
from . import branding
from .logpanel import logger
from .settings import app_settings

__all__ = ["Assembly3DTab", "AssemblyView", "FlowLayout"]


class FlowLayout(QLayout):
    """A row that wraps instead of squeezing.

    ``QHBoxLayout`` given less width than its children asked for takes it from
    them anyway, and a ``QCheckBox`` that has been squeezed elides its own text:
    *Housing* becomes *Housin*, *Carrier* becomes *Carrie*. On a wide window the
    row fits and nothing shows; on a narrower one - a laptop, a Mac's default
    window, a user who has dragged the splitter - every label in the visibility
    row loses its last letters at once, which reads as a rendering fault rather
    than as a layout that ran out of room.

    Wrapping is the honest answer for a row of independent toggles: they have no
    order that matters and no alignment to keep, so a second line costs nothing
    but height, and height is what this tab has spare.
    """

    def __init__(self, parent=None, spacing: int = 6) -> None:
        super().__init__(parent)
        self._items: list = []
        self.setSpacing(spacing)

    # ------------------------------------------------------- QLayout plumbing
    def addItem(self, item) -> None:
        self._items.append(item)

    def count(self) -> int:
        return len(self._items)

    def itemAt(self, index: int):
        return self._items[index] if 0 <= index < len(self._items) else None

    def takeAt(self, index: int):
        return self._items.pop(index) if 0 <= index < len(self._items) else None

    def expandingDirections(self):
        return Qt.Orientations(Qt.Orientation(0))

    def hasHeightForWidth(self) -> bool:
        return True

    def heightForWidth(self, width: int) -> int:
        return self._lay_out(QRect(0, 0, width, 0), apply=False)

    def setGeometry(self, rect) -> None:
        super().setGeometry(rect)
        self._lay_out(rect, apply=True)

    def sizeHint(self) -> QSize:
        return self.minimumSize()

    def minimumSize(self) -> QSize:
        size = QSize()
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        margins = self.contentsMargins()
        return size + QSize(margins.left() + margins.right(),
                            margins.top() + margins.bottom())

    def _lay_out(self, rect, *, apply: bool) -> int:
        """Place the items left to right, wrapping; return the height used.

        Two passes, because the items are centred in their row and a row's
        height is not known until it has ended.  Placing everything at the
        row's top - which is what one pass can do - lines the widgets up by
        their boxes rather than by what is drawn in them, and a control a few
        pixels taller than the checkboxes beside it then paints its own
        contents low: the bearings menu hung below the row like dropped
        punctuation, which is exactly what it was mistaken for.
        """
        margins = self.contentsMargins()
        area = rect.adjusted(margins.left(), margins.top(),
                             -margins.right(), -margins.bottom())
        space = self.spacing()

        rows: list[tuple[int, list]] = []
        x, line, line_height = area.x(), [], 0
        for item in self._items:
            hint = item.sizeHint()
            if line and x + hint.width() > area.right():
                rows.append((line_height, line))
                x, line, line_height = area.x(), [], 0
            line.append((item, x, hint))
            x += hint.width() + space
            line_height = max(line_height, hint.height())
        if line:
            rows.append((line_height, line))
        if not rows:
            return margins.top() + margins.bottom()

        y = area.y()
        for height, items in rows:
            if apply:
                for item, item_x, hint in items:
                    top = y + (height - hint.height()) // 2
                    item.setGeometry(QRect(QPoint(item_x, top), hint))
            y += height + space
        return y - space - rect.y() + margins.bottom()


#: Standard viewpoints, as (azimuth, elevation) in degrees.
STANDARD_VIEWS: dict[str, tuple[float, float]] = {
    "iso": (38.0, 26.0),
    "front": (-90.0, 0.0),
    "top": (-90.0, 88.0),
    "side": (0.0, 0.0),
}

_ORBIT_PER_PIXEL = 0.34
_ZOOM_PER_NOTCH = 0.86

#: Groups this tab opens with switched off, on a machine that has never run it.
#:
#: Only the end plates.  With them on, the assembled view is a closed cylinder
#: with a shaft out of one end - which is exactly what the gearbox looks like and
#: exactly no use as the first thing a design tool shows you.  They are one click
#: away and the checkbox is visibly unticked, so nothing is being hidden from
#: anyone; the default just starts where the work is.
#: The tab opens on an open gearbox: with the plates on, the assembled view is
#: a closed cylinder with a shaft out of one end, which is what the gearbox
#: looks like and exactly no use as the first thing a design tool shows you.
#: The bolts go with them - six fasteners floating where the plates they hold
#: on are not is a stranger picture than either.
_HIDDEN_BY_DEFAULT: frozenset[str] = frozenset({"end_plates", "fasteners"})

#: A press and release closer together than this is a click, not a drag.  Both
#: views orbit on the left button, so the one gesture has to be told from the
#: other by how far the pointer went in between.
CLICK_SLOP_PX = 4

#: How far a picked part's colour goes towards the accent, and a flagged one's
#: towards the error red.  Far enough to find at a glance, not so far that the
#: part stops looking like what it is made of.
SELECTED_TINT = 0.45
FLAGGED_TINT = 0.6

#: The groups X-ray makes see-through: what closes the drive, not what works
#: inside it.  And how much of them is left - enough to keep the outline and
#: the bolt circle, little enough to read the stack through.
XRAY_GROUPS = frozenset({"housing", "end_plates"})
XRAY_OPACITY = 0.22


def tinted(rgb, hex_colour: str, amount: float) -> tuple[int, int, int]:
    """``rgb`` (0-255) moved ``amount`` of the way towards ``hex_colour``."""
    target = [int(hex_colour[i:i + 2], 16) for i in (1, 3, 5)]
    return tuple(round(float(c) + (t - float(c)) * amount)
                 for c, t in zip(rgb, target, strict=True))


def _polygon(points: np.ndarray) -> QPolygonF:
    return QPolygonF([QPointF(float(x), float(y)) for x, y in points])


class AssemblyView(QWidget):
    """Orbit, pan and zoom around one design at one crank angle."""

    #: A part was clicked: its name, or ``None`` for empty space.
    part_picked = Signal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(320, 260)
        self.setMouseTracking(False)
        self.setToolTip("Drag to orbit, right-drag to pan, wheel to zoom, "
                        "double-click to re-fit.")

        self._spec: GearSpec | None = None
        self._mesh: Mesh | None = None
        self._camera = Camera()
        self._span = 200.0
        self._crank = 0.0
        self._explode = 0.0
        self._hidden: set[str] = set()
        self._edges = False
        self._mode = "light"
        self._drag: tuple[float, float] | None = None
        self._drag_button = Qt.NoButton
        self._failed = ""
        self._label_font = branding.mono_font(9)
        self._press: tuple[float, float] | None = None
        self._selected: str | None = None
        self._flagged: frozenset[str] = frozenset()
        self._xray = False
        self._last_draw = None

    # ------------------------------------------------------------------ state
    def set_spec(self, spec: GearSpec) -> None:
        """Adopt a design, re-framing only when the drive genuinely resized.

        Re-framing a view the user has just orbited and zoomed to, because they
        nudged a clearance by 0.01 mm, is infuriating.  So the camera is left
        alone unless the assembly has changed size enough that the old framing
        would not hold it - a preset change, or a new design from the optimiser.
        """
        try:
            mesh = mesh_for_spec(spec)
        except Exception:
            self._failed = traceback.format_exc()
            logger.error("3D view could not build the assembly\n%s",
                         self._failed.rstrip())
            self._mesh = None
            self.update()
            return

        was = self._span if self._mesh is not None else 0.0
        self._failed = ""
        self._spec, self._mesh = spec, mesh
        if not 0.9 * was <= self._measure() <= 1.1 * was:
            self._refit()
        self.update()

    def set_crank(self, degrees: float) -> None:
        self._crank = float(degrees)
        self.update()

    def set_explode(self, fraction: float) -> None:
        """Pull the assembly apart, re-framing as it grows.

        Exploding is a change of what you are looking at, not of where you are
        looking from, so it takes the framing with it; the alternative is a
        slider that pushes half the gearbox off the edge of the panel.
        """
        self._explode = float(fraction)
        self._refit()
        self.update()

    def _measure(self) -> float:
        """Diagonal of the assembly's bounding box, for framing and zoom limits."""
        if self._mesh is None:
            return self._span
        lo, hi = self._mesh.bounds(self._explode)
        self._span = max(float(np.linalg.norm(hi - lo)), 1e-6)
        return self._span

    def _refit(self) -> None:
        if self._mesh is not None:
            self._camera = Camera.framing(self._mesh, explode=self._explode,
                                          azimuth=self._camera.azimuth,
                                          elevation=self._camera.elevation)

    def set_group_visible(self, group: str, visible: bool) -> None:
        """Show or hide a part group, or one named part."""
        self._hidden.discard(group) if visible else self._hidden.add(group)
        self.update()

    def hideable_parts(self, group: str) -> list[tuple[str, str]]:
        """(name, label) of the parts in ``group``, for a per-part menu."""
        if self._mesh is None:
            return []
        return [(p.name, p.label) for p in self._mesh.parts if p.group == group]

    def set_edges(self, on: bool) -> None:
        self._edges = bool(on)
        self.update()

    def set_theme(self, mode: str) -> None:
        self._mode = mode
        self.update()

    def set_selected(self, name: str | None) -> None:
        """Tint one part towards the accent: the one the info strip describes."""
        self._selected = name
        self.update()

    def set_flagged(self, names) -> None:
        """Tint these parts towards the error red: the ones a clash names."""
        self._flagged = frozenset(names)
        self.update()

    def set_xray(self, on: bool) -> None:
        """See through the housing and its plates."""
        self._xray = bool(on)
        self.update()

    def _see_through(self, part) -> bool:
        return self._xray and part.group in XRAY_GROUPS

    def pick(self, x: float, y: float) -> str | None:
        """The part under the pixel ``(x, y)``, or ``None``.

        Off the frame last painted, front to back: the draw list is ordered
        back to front, so the last face that contains the point is the one on
        top.  Each face is tested with its holes, so a click down a bore finds
        whatever is at the bottom of it.
        """
        draw, mesh = self._last_draw, self._mesh
        if draw is None or mesh is None:
            return None
        point = QPointF(float(x), float(y))
        for i in range(len(draw.loops) - 1, -1, -1):
            path = QPainterPath()
            path.setFillRule(Qt.OddEvenFill)
            for loop in draw.loops[i]:
                path.addPolygon(_polygon(loop))
                path.closeSubpath()
            part = mesh.parts[int(draw.parts[i])]
            # A part you are seeing through is not what you clicked on.
            if path.contains(point) and not self._see_through(part):
                return part.name
        return None

    def set_standard_view(self, name: str) -> None:
        azimuth, elevation = STANDARD_VIEWS[name]
        self._camera = Camera(target=self._camera.target, azimuth=azimuth,
                              elevation=elevation, distance=self._camera.distance,
                              fov_deg=self._camera.fov_deg)
        self.update()

    def fit(self) -> None:
        self._measure()
        self._refit()
        self.update()

    @property
    def camera(self) -> Camera:
        return self._camera

    def set_camera_angles(self, azimuth: float, elevation: float) -> None:
        """Restore a stored orientation.  Distance stays with the geometry."""
        self._camera = Camera(target=self._camera.target, azimuth=azimuth,
                              elevation=elevation, distance=self._camera.distance,
                              fov_deg=self._camera.fov_deg)
        self.update()

    # --------------------------------------------------------------- painting
    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        p = branding.palette(self._mode)
        painter.fillRect(self.rect(), QColor(p.raised))

        if self._mesh is None:
            painter.setPen(QColor(p.ink_dim))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             "No assembly to show."
                             + ("\nSee the log tab." if self._failed else ""))
            return

        width, height = max(self.width(), 1), max(self.height(), 1)
        draw = render(self._mesh, math.radians(self._crank), self._camera,
                      width, height, explode=self._explode, hidden=self._hidden)
        self._last_draw = draw
        names = [part.name for part in self._mesh.parts]
        clear = [self._see_through(part) for part in self._mesh.parts]

        # One pen and one brush per face.  The pen matters: filling a polygon
        # without outlining it leaves an antialiased hairline of background
        # between it and its neighbour, and a solid part ends up looking like a
        # wireframe of its own seams.
        for loops, rgb, index in zip(draw.loops, draw.colours, draw.parts,
                                     strict=True):
            name = names[int(index)]
            if name == self._selected:
                rgb = tinted(rgb, p.accent, SELECTED_TINT)
            elif name in self._flagged:
                rgb = tinted(rgb, p.error, FLAGGED_TINT)
            fill = QColor(int(rgb[0]), int(rgb[1]), int(rgb[2]))
            if clear[int(index)]:
                # Back to front already, so a translucent face simply goes
                # over whatever is behind it.  No outline: two translucent
                # neighbours would each draw the seam between them, and it
                # would come out twice as dark as either face.
                fill.setAlphaF(XRAY_OPACITY)
                painter.setBrush(fill)
                painter.setPen(Qt.NoPen)
                self._draw_face(painter, loops)
                continue
            painter.setBrush(fill)
            # The same rule the hardware path uses - `EDGE_SHADE` is shared
            # so the two renderers cannot drift about what an edge looks
            # like, which is how one of them ended up drawing them in the
            # theme's ink and the other in the part's.
            painter.setPen(QColor(*[int(c * EDGE_SHADE) for c in rgb])
                           if self._edges else fill)
            self._draw_face(painter, loops)

        self._paint_overlay(painter, p)

    @staticmethod
    def _draw_face(painter: QPainter, loops) -> None:
        """One face, with its holes, in the current pen and brush."""
        if len(loops) == 1:
            painter.drawPolygon(_polygon(loops[0]))
            return
        path = QPainterPath()
        path.setFillRule(Qt.OddEvenFill)
        for loop in loops:
            path.addPolygon(_polygon(loop))
            path.closeSubpath()
        painter.drawPath(path)

    def _paint_overlay(self, painter: QPainter, p) -> None:
        """Readouts and an axis triad, drawn over the model."""
        painter.setFont(self._label_font)
        painter.setPen(QColor(p.ink))
        if self._spec is not None:
            # Modulo a turn: playback runs the crank unwrapped over the
            # mechanism's period, which is `lobes` input revolutions.  And the
            # crank angle is the *input* angle only while the ring is the
            # grounded member - with the carrier grounded the crank runs at
            # (N+1)/N of the input, so what the shafts read is not the
            # parameter the animation is stepping.
            turned = abs(self._spec.shaft_spin) * self._crank
            out = (turned / self._spec.ratio) % 360.0
            painter.drawText(12, 20, f"INPUT {turned % 360.0:6.1f} deg")
            painter.drawText(12, 36, f"OUTPUT{out:7.2f} deg   {self._spec.ratio}:1")

        right, up, _ = self._camera.basis()
        ox, oy, length = 30.0, self.height() - 30.0, 24.0
        for axis, name in zip(np.eye(3), "XYZ", strict=True):
            dx, dy = float(axis @ right) * length, -float(axis @ up) * length
            painter.setPen(QColor(p.ink_dim))
            painter.drawLine(QPointF(ox, oy), QPointF(ox + dx, oy + dy))
            painter.setPen(QColor(p.ink))
            painter.drawText(QPointF(ox + 1.35 * dx - 3, oy + 1.35 * dy + 4), name)

        painter.setPen(QColor(p.ink_dim))
        painter.drawText(self.rect().adjusted(0, 0, -10, -8),
                         Qt.AlignRight | Qt.AlignBottom,
                         "drag orbit  -  right-drag pan  -  wheel zoom")

    # ------------------------------------------------------------ interaction
    def mousePressEvent(self, event) -> None:
        pos = event.position()
        self._drag = (pos.x(), pos.y())
        self._press = (pos.x(), pos.y())
        self._drag_button = event.button()

    def mouseMoveEvent(self, event) -> None:
        if self._drag is None:
            return
        pos = event.position()
        dx, dy = pos.x() - self._drag[0], pos.y() - self._drag[1]
        self._drag = (pos.x(), pos.y())
        if self._drag_button == Qt.LeftButton:
            # Dragging right turns the model right, which means the *camera*
            # goes the other way: increasing azimuth walks the eye anticlockwise
            # and the model appears to slide left.
            self._camera = self._camera.orbited(-dx * _ORBIT_PER_PIXEL,
                                                dy * _ORBIT_PER_PIXEL)
        else:
            self._camera = self._camera.panned(dx, dy, self.height())
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        pos = event.position()
        if (self._press is not None and event.button() == Qt.LeftButton
                and abs(pos.x() - self._press[0]) <= CLICK_SLOP_PX
                and abs(pos.y() - self._press[1]) <= CLICK_SLOP_PX):
            self.part_picked.emit(self.pick(pos.x(), pos.y()))
        self._press = None
        self._drag = None
        self._drag_button = Qt.NoButton

    def mouseDoubleClickEvent(self, event) -> None:
        self.fit()

    def wheelEvent(self, event) -> None:
        notches = event.angleDelta().y() / 120.0
        self._camera = self._camera.zoomed(_ZOOM_PER_NOTCH ** notches,
                                           span=self._span)
        self.update()


def build_view(parent: QWidget | None = None) -> QWidget:
    """The best 3D view this machine can give us.

    VTK when it is there and its render window can actually be created, which
    is the normal case because CadQuery installs it.  The software painter
    otherwise: a build with the CAD kernel stripped out, a machine with no
    OpenGL, a remote session that does not forward it.  Falling back is worth
    more than failing - a flat-shaded gearbox is still a gearbox, and the
    alternative is a tab that shows an error.
    """
    try:
        from . import view3d_vtk
        if view3d_vtk.available():
            view = view3d_vtk.VtkAssemblyView(parent)
            logger.info("3D: hardware renderer (VTK)")
            return view
        logger.info("3D: VTK unavailable, using the software renderer")
    except Exception:
        logger.warning("3D: VTK failed to start, using the software renderer\n%s",
                       traceback.format_exc().rstrip())
    return AssemblyView(parent)


class Assembly3DTab(QWidget):
    """The viewer plus the controls a gearbox actually needs.

    Hiding the housing and pulling the stack apart are the two things anyone
    does first with a 3D assembly, so they are one click and one drag rather
    than a menu.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        self.view = build_view()

        view_row = QHBoxLayout()
        view_row.setSpacing(6)
        # One joined control: four positions of the same camera, not four
        # unrelated buttons that happen to sit side by side.
        views = QHBoxLayout()
        views.setSpacing(0)
        names = (("iso", "Iso"), ("front", "Front"), ("top", "Top"),
                 ("side", "Side"))
        for index, (key, text) in enumerate(names):
            button = QPushButton(text)
            button.setProperty("segment", "first" if index == 0 else
                               "last" if index == len(names) - 1 else "middle")
            button.setToolTip(f"Look at the drive from the {key}"
                              if key != "iso" else "The isometric view")
            button.clicked.connect(lambda _c=False, k=key:
                                   self.view.set_standard_view(k))
            views.addWidget(button)
        view_row.addLayout(views)
        fit = QPushButton("Fit")
        fit.setToolTip("Frame the whole assembly. Double-clicking the view "
                       "does the same.")
        fit.clicked.connect(self.view.fit)
        view_row.addWidget(fit)

        # The two sliders travel together.  They are one kind of control - drag
        # to open the assembly up - and they were split across the two rows,
        # explode alone at the top and section squeezed onto the end of the
        # checkboxes at 150 px, where the row had already run out of width and
        # wrapped.  Same row, same stretch, so they read as the pair they are.
        view_row.addSpacing(10)
        view_row.addWidget(branding.caption("Explode"))
        self._explode = QSlider(Qt.Horizontal)
        self._explode.setRange(0, 100)
        self._explode.setToolTip("Slide the parts apart along the axis, in "
                                 "assembly order.")
        self._explode.valueChanged.connect(
            lambda v: self.view.set_explode(v / 100.0))
        view_row.addWidget(self._explode, 1)

        self._section = QSlider(Qt.Horizontal)
        self._section.setRange(0, 100)
        self._section.setToolTip("Cut the assembly on a plane through the axis, "
                                 "to see the mesh instead of the outside of it.")
        # Only the hardware view can cut: a clipping plane is a per-fragment
        # test, and the software painter works on whole faces.
        if hasattr(self.view, "set_section"):
            view_row.addSpacing(10)
            view_row.addWidget(branding.caption("Section"))
            self._section.valueChanged.connect(
                lambda v: self.view.set_section(v / 100.0))
            view_row.addWidget(self._section, 1)

        view_row.addSpacing(10)
        self._edges = branding.chip(QCheckBox("Edges"))
        self._edges.setToolTip("Outline every facet. Useful for reading the "
                               "shape, noisy on a fine profile.")
        self._edges.toggled.connect(self.view.set_edges)
        view_row.addWidget(self._edges)
        self._xray = branding.chip(QCheckBox("X-ray"))
        self._xray.setToolTip("See through the housing and its end plates. A "
                              "click goes through them to the part behind.")
        self._xray.toggled.connect(self.view.set_xray)
        view_row.addWidget(self._xray)
        self._clash_box = branding.chip(QCheckBox("Clashes"))
        self._clash_box.setToolTip("Tint red every part a clash check is about: "
                                   "a profile into its pins, a bolt into a "
                                   "pocket, a spigot onto a bearing.")
        self._clash_box.setEnabled(False)
        self._clash_box.toggled.connect(lambda _on: self._apply_flags())
        view_row.addWidget(self._clash_box)
        # Measuring needs the point a click lands on in the model, which the
        # hardware view's picker gives and the software painter's draw list
        # does not - the same line the section plane is on, for a kindred
        # reason.
        self._measure = QPushButton("Measure")
        self._measure.setCheckable(True)
        self._measure.setToolTip("Click two points on the model for the distance "
                                 "between them. A third click starts again.")
        self._measure.toggled.connect(self._toggle_measuring)
        if hasattr(self.view, "set_measuring"):
            self.view.measured.connect(self._measured)
            view_row.addWidget(self._measure)
        layout.addLayout(view_row)

        # Visibility, and nothing else.  What this row is for is now answerable
        # from the row itself.
        show_row = FlowLayout()
        show_row.addWidget(branding.caption("Show"))
        # The bearings are the one group where all-or-nothing is not enough.  The
        # cam bearing is down a bore and the shaft supports are out in the open,
        # so seeing one of them usually means putting the others away - and which
        # ones a design even has changes with the design, which is why this is a
        # menu rebuilt per spec rather than a row of boxes built once.
        self._hidden_parts: set[str] = set()
        self._bearing_menu = branding.chip(QToolButton())
        # Says what it opens.  An ellipsis and a menu arrow beside a row of
        # toggles read as stray punctuation, whatever their size.
        self._bearing_menu.setText("Each bearing ▾")
        self._bearing_menu.setPopupMode(QToolButton.InstantPopup)
        self._bearing_menu.setToolTip("Show or hide bearings one at a time.")
        self._bearing_menu.setMenu(QMenu(self._bearing_menu))
        self._bearing_menu.setEnabled(False)

        self._groups: dict[str, QCheckBox] = {}
        for group, label in PART_GROUPS:
            box = branding.chip(QCheckBox(label))
            box.setChecked(group not in _HIDDEN_BY_DEFAULT)
            box.toggled.connect(
                lambda on, g=group: self.view.set_group_visible(g, on))
            self.view.set_group_visible(group, box.isChecked())
            show_row.addWidget(box)
            self._groups[group] = box
            if group == "bearings":
                show_row.addWidget(self._bearing_menu)

        layout.addLayout(show_row)

        layout.addWidget(self.view, 1)

        # What the picked part is.  Under the viewport rather than over it: the
        # hardware viewport is a native window, and a Qt widget laid over one is
        # drawn under it on some platforms and not at all on others.
        self._info = QLabel()
        self._info.setObjectName("BrandTagline")
        self._info.setWordWrap(True)
        self._info.setTextFormat(Qt.RichText)
        layout.addWidget(self._info)
        self._bom: dict = {}
        self._labels: dict[str, str] = {}
        self._selected: str | None = None
        self._unit = None
        self._measurement = None
        self._analysis = None
        self._mode = "light"
        self._clashes: dict[str, list] = {}
        self._pointed: frozenset[str] = frozenset()
        self.view.part_picked.connect(self._picked)
        self._describe()

    # ------------------------------------------------------------------ state
    def set_spec(self, spec: GearSpec) -> None:
        self.view.set_spec(spec)
        self._rebuild_bearing_menu()
        try:
            self._labels = {p.name: p.label for p in mesh_for_spec(spec).parts}
        except Exception:                 # the view has already said why
            self._labels = {}
        # A part this design no longer has cannot stay picked.
        if self._selected not in self._labels:
            self._picked(None)

    def set_analysis(self, analysis) -> None:
        """What each part is, off the bill of materials for this analysis."""
        from ..export.bom import bom_items
        from ..viz.clashes import clash_parts

        self._analysis = analysis
        self._bom = {name: item for item in bom_items(analysis)
                     for name in item.assembly}
        self._clashes = clash_parts(analysis)
        count = len({f.message for found in self._clashes.values() for f in found})
        self._clash_box.setText(f"Clashes ({count})" if count else "Clashes")
        self._clash_box.setEnabled(bool(count))
        self._apply_flags()
        self._describe()

    def point_at(self, code: str | None) -> None:
        """Tint the parts the check ``code`` is about - the one picked in the
        checks list - on top of whatever the Clashes box is showing."""
        from ..viz.clashes import finding_parts

        parts: set[str] = set()
        if code is not None and self._analysis is not None:
            for finding in self._analysis.report.findings:
                if finding.code == code:
                    parts.update(finding_parts(self._analysis, finding))
        self._pointed = frozenset(parts)
        self._apply_flags()

    def _apply_flags(self) -> None:
        shown = set(self._clashes) if self._clash_box.isChecked() else set()
        self.view.set_flagged(shown | self._pointed)

    def set_unit(self, unit) -> None:
        """Write lengths read off the model in ``unit``, as the window does."""
        self._unit = unit
        if hasattr(self.view, "length_text"):
            self.view.length_text = self._length
        self._describe()

    def _length(self, mm: float) -> str:
        return self._unit.text(mm, 2) if self._unit is not None else f"{mm:.2f} mm"

    def _toggle_measuring(self, on: bool) -> None:
        self._measurement = None
        if hasattr(self.view, "set_measuring"):
            self.view.set_measuring(on)
        self._describe()

    def _measured(self, points) -> None:
        self._measurement = points
        self._describe()

    def _picked(self, name: str | None) -> None:
        self._selected = name
        self.view.set_selected(name)
        self._describe()

    def _describe(self) -> None:
        """The info strip: the picked part's line on the bill of materials.

        In the bill's own words and units, because that is what it is - the
        same line the CSV carries, for the part under the pointer.
        """
        if self._measure.isChecked():
            self._info.setText(self._measure_text())
            return
        name = self._selected
        if name is None:
            self._info.setText("Click a part to see what it is, what it is made "
                               "of and what it weighs.")
            return
        label = html.escape(self._labels.get(name, name))
        item = self._bom.get(name)
        if item is None:
            self._info.setText(f"<b>{label}</b>" + self._clash_text(name))
            return
        mass = (f", {item.mass_each_g:.1f} g each" if item.mass_each_g else "")
        # The bill's name only where it says something the label does not -
        # "Housing end plate" for the input plate, not "Ring housing" twice.
        line_name = ("" if item.part == self._labels.get(name)
                     else f"{html.escape(item.part)}: ")
        line = (f"<b>{label}</b> &nbsp;&middot;&nbsp; "
                f"{line_name}{item.quantity} x "
                f"{html.escape(item.material)}, {html.escape(item.size)}{mass}, "
                f"{'made' if item.source == 'make' else 'bought'}")
        if item.note:
            line += f"<br>{html.escape(item.note)}"
        self._info.setText(line + self._clash_text(name))

    def _clash_text(self, name: str) -> str:
        """What the clash checks say about this part, in the error colour."""
        found = self._clashes.get(name)
        if not found:
            return ""
        colour = branding.palette(self._mode).error
        lines = "<br>".join(f"{html.escape(f.code)}: {html.escape(f.message)}"
                            for f in found)
        return f"<br><span style='color:{colour}'>{lines}</span>"

    def _rebuild_bearing_menu(self) -> None:
        """One entry per bearing this design actually has.

        A bearing already hidden stays hidden across a design change: the names
        are stable, so switching preset and back does not quietly put a part you
        put away back on the screen.  A name that no longer exists costs nothing
        - the renderer is being told to hide something that is not there.
        """
        menu = self._bearing_menu.menu()
        menu.clear()
        parts = self.view.hideable_parts("bearings")
        self._bearing_menu.setEnabled(bool(parts))
        for name, label in parts:
            action = menu.addAction(label)
            action.setCheckable(True)
            # Checked before it is connected: setting it after would fire
            # `toggled` and hide whatever the last design had hidden.
            action.setChecked(name not in self._hidden_parts)
            action.toggled.connect(
                lambda on, n=name: self._set_part_visible(n, on))
            self.view.set_group_visible(name, name not in self._hidden_parts)

    def _set_part_visible(self, name: str, visible: bool) -> None:
        self._hidden_parts.discard(name) if visible else self._hidden_parts.add(name)
        self.view.set_group_visible(name, visible)

    def _measure_text(self) -> str:
        """The strip while measuring: what to click next, or what was measured."""
        points = self._measurement
        if points is None:
            return "Measuring: click a point on the model, then a second one."
        a, b = points
        if b is None:
            return "Measuring: click a second point."
        d = [bv - av for av, bv in zip(a, b, strict=True)]
        total = sum(c * c for c in d) ** 0.5
        parts = " &nbsp;".join(f"&Delta;{axis} {self._length(abs(c))}"
                               for axis, c in zip("xyz", d, strict=True))
        return (f"<b>{self._length(total)}</b> between the two points &nbsp;"
                f"&middot;&nbsp; {parts} &nbsp;&middot;&nbsp; a third click starts "
                f"again")

    def set_crank(self, degrees: float) -> None:
        self.view.set_crank(degrees)

    def refresh_theme(self, mode: str) -> None:
        self._mode = mode
        self.view.set_theme(mode)
        self._describe()

    def render_options(self) -> dict:
        """What an exported animation needs to look like this tab does.

        Orientation, explode and which groups are hidden - not the section
        plane or the edge outlines, which are the hardware renderer's own and
        have no equivalent in the polygon list a frame is built from.
        """
        camera = self.view.camera
        return {"azimuth": camera.azimuth, "elevation": camera.elevation,
                "explode": self._explode.value() / 100.0,
                "hidden": frozenset(self._hidden())}

    def _hidden(self) -> list[str]:
        """Everything currently switched off, groups and single parts alike.

        One list, because the renderer takes one set: a name that is both a group
        and a part is a group of exactly one part named after it, so it says the
        same thing read either way.
        """
        return [g for g, box in self._groups.items()
                if not box.isChecked()] + sorted(self._hidden_parts)

    def save_state(self) -> None:
        settings = app_settings()
        camera = self.view.camera
        settings.setValue("view3d_azimuth", camera.azimuth)
        settings.setValue("view3d_elevation", camera.elevation)
        settings.setValue("view3d_explode", self._explode.value())
        settings.setValue("view3d_edges", self._edges.isChecked())
        settings.setValue("view3d_xray", self._xray.isChecked())
        settings.setValue("view3d_section", self._section.value())
        settings.setValue("view3d_hidden", self._hidden())

    def restore_state(self) -> None:
        """Reopen on the viewpoint the last session left.

        Every value is restored on its own and defensively: a stored setting
        from an older build should cost a default, not a tab that will not
        build.
        """
        settings = app_settings()
        try:
            azimuth = float(settings.value("view3d_azimuth", 38.0))
            elevation = float(settings.value("view3d_elevation", 26.0))
        except (TypeError, ValueError):
            azimuth, elevation = STANDARD_VIEWS["iso"]
        self.view.set_camera_angles(azimuth, elevation)

        with suppress(TypeError, ValueError):
            self._explode.setValue(int(settings.value("view3d_explode", 0)))
        with suppress(TypeError, ValueError):
            self._section.setValue(int(settings.value("view3d_section", 0)))
        self._edges.setChecked(bool(settings.value("view3d_edges", False, type=bool)))
        self._xray.setChecked(bool(settings.value("view3d_xray", False, type=bool)))

        # `contains` rather than a falsy check: a stored *empty* list means the
        # last session had everything on, and treating that as "no preference"
        # would put the end plates back every time you took them off.
        if settings.contains("view3d_hidden"):
            hidden = settings.value("view3d_hidden") or []
        else:
            hidden = list(_HIDDEN_BY_DEFAULT)
        if isinstance(hidden, str):
            hidden = [hidden]
        for group, box in self._groups.items():
            box.setChecked(group not in hidden)
        # Whatever is left is a part name.  Kept without checking it against the
        # current design: the tab is restored before a spec ever arrives, so
        # there is nothing yet to check it against.
        self._hidden_parts = {h for h in hidden if h not in self._groups}
        self._rebuild_bearing_menu()
