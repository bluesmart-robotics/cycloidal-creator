"""A heading that opens and closes the group of parameters under it.

The parameter panel holds ten groups and some sixty fields.  Laid out all at
once, each in a bordered card, that is a wall: the drive's geometry - the four
numbers most sessions are spent on - is one card among ten, and the panel reads
as busy before a single value has been read.  Most of those groups are set once
per design, or never.  So each one folds behind its heading, the open ones are
remembered, and the panel opens on the few that are actually being worked on.

Folding never hides anything the window is pointing at: the parameter filter
opens every group it finds a match in, and a check that highlights a field opens
the group that field is in.  See :class:`cycloidgen.ui.main_window.MainWindow`.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QPushButton, QVBoxLayout, QWidget

from . import branding

__all__ = ["Section"]


class Section(QWidget):
    """A clickable heading over a body that it shows and hides."""

    #: The user opened or closed it - not emitted for programmatic changes, so
    #: only a choice somebody made is ever remembered.
    toggled = Signal(bool)

    def __init__(self, title: str, *, first: bool = False,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._title = title
        self._open = True
        self._mode = "light"

        self.header = QPushButton(title)
        self.header.setObjectName("SectionHeader")
        if first:
            # The panel's own edge is already a line above it.
            self.header.setProperty("first", "true")
        self.header.setCursor(Qt.PointingHandCursor)
        self.header.setFocusPolicy(Qt.TabFocus)
        self.header.clicked.connect(self._clicked)

        self.body = QWidget()
        self.body.setObjectName("SectionBody")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.header)
        layout.addWidget(self.body)
        self._refresh_icon()

    @property
    def title(self) -> str:
        return self._title

    def is_expanded(self) -> bool:
        return self._open

    def set_expanded(self, on: bool) -> None:
        """Open or close it without counting as the user's choice."""
        self._open = bool(on)
        self.body.setVisible(self._open)
        self._refresh_icon()

    def set_theme(self, mode: str) -> None:
        self._mode = mode
        self._refresh_icon()

    def _clicked(self) -> None:
        self.set_expanded(not self._open)
        self.toggled.emit(self._open)

    def _refresh_icon(self) -> None:
        ink = branding.palette(self._mode).ink_dim
        self.header.setIcon(branding.chevron_icon(self._open, ink))
        self.header.setToolTip(("Hide" if self._open else "Show")
                               + f" the {self._title.lower()} parameters")
