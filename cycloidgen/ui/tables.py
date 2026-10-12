"""Table behaviour shared between the panels that carry one.

Both the checks list and the outputs list have a column that is a *sentence*
rather than a field, and both of them used to end every row in an ellipsis.
The fix is the same in both places, so it lives in one.
"""
from __future__ import annotations

from PySide6.QtCore import QEvent, QSize, Qt
from PySide6.QtGui import QFontMetrics
from PySide6.QtWidgets import QStyledItemDelegate, QStyleOptionViewItem, QTreeWidget

__all__ = ["WRAP_MARGIN_PX", "WrappingColumn"]

#: Padding either side of a wrapped cell, and under it.  One number for both:
#: text that touches the column edge reads as clipped even when it is not.
WRAP_MARGIN_PX = 8

#: Taken off every row of a list gone to one line.  The stylesheet's item
#: padding is sized for rows that wrap, where it separates one sentence from
#: the next; on a single line two pixels of it are the difference between the
#: fourth row being on screen and being cut by its bottom edge.
_ELIDED_TRIM_PX = 2


class WrappingColumn(QStyledItemDelegate):
    """Give one column of a tree the height its wrapped text actually needs.

    ``QAbstractItemView.setWordWrap`` is half the job and the half that is easy
    to mistake for all of it: it makes the *painter* wrap, so the text is laid
    out correctly and then clipped to a row one line tall.  The row height comes
    from the delegate's size hint, and the default one measures a single line
    because the option it is handed during layout has no width in it yet.

    So the width is taken from the column rather than from the option.  Nothing
    else here changes: painting is still the base class's, which already wraps.

    ``keep_rows`` makes the wrapping conditional.  Wrapped, every row is as tall
    as its sentence, and in a short view that spends the height on reading one
    finding instead of seeing several: the checks list at its floor on a
    1366x768 laptop held two.  Given ``keep_rows``, the column wraps only while
    that many rows still fit wrapped, and otherwise every row is one line and
    elided - the full text is a tooltip away, and in the checks list it is also
    in the explanation panel.  Call :meth:`refit` when rows are hidden or
    shown, which a view does not announce.

    One line per row was not enough on its own.  At the checks list's floor the
    column header took 37 of the 101 pixels the rows had, so even single lines
    came to two and a half.  Elided, the header goes as well and the rows lose a
    little padding - the header is a label for columns somebody is not reading
    as a table at that size, and it comes back with the room.
    """

    def __init__(self, view: QTreeWidget, column: int,
                 keep_rows: int | None = None) -> None:
        super().__init__(view)
        self._view = view
        self._column = column
        self._keep_rows = keep_rows
        self._wrap = True
        # A stretched last section resizes with the window and with the
        # splitter, and a row measured against the old width is a row that
        # clips or a row with a band of empty under it.
        view.header().sectionResized.connect(
            lambda *_: view.scheduleDelayedItemsLayout())
        if keep_rows:
            view.header().sectionResized.connect(lambda *_: self.refit())
            view.viewport().installEventFilter(self)

    @property
    def wrapping(self) -> bool:
        """Whether the column is wrapping right now, or one line and elided."""
        return self._wrap

    def eventFilter(self, watched, event) -> bool:
        if event.type() == QEvent.Resize:
            self.refit()
        return False

    def sizeHint(self, option, index) -> QSize:
        hint = super().sizeHint(option, index)
        if not self._wrap:
            return QSize(hint.width(), hint.height() - _ELIDED_TRIM_PX)
        if index.column() != self._column:
            return hint
        return self._wrapped(hint, option, index, self._view.columnWidth(self._column))

    def _wrapped(self, hint: QSize, option, index, column_width: int) -> QSize:
        width = column_width - WRAP_MARGIN_PX
        if width <= 0:
            return hint
        styled = QStyleOptionViewItem(option)
        self.initStyleOption(styled, index)
        if not styled.text:
            return hint
        needed = QFontMetrics(styled.font).boundingRect(
            0, 0, width, 0, Qt.TextWordWrap, styled.text).height()
        return QSize(hint.width(), max(hint.height(), needed + WRAP_MARGIN_PX))

    def refit(self) -> None:
        """Wrap only while the first ``keep_rows`` shown rows fit wrapped."""
        if not self._keep_rows:
            return
        view, model = self._view, self._view.model()
        # Measured as if the scrollbar were up whether it is or not.  Wrapping
        # is what brings it up, and it takes its width out of the stretched
        # column - so a measurement that followed it could wrap, find the rows
        # taller for the narrower column, unwrap, lose the scrollbar, and go
        # round again.
        column = view.columnWidth(self._column)
        bar = view.verticalScrollBar()
        if not bar.isVisible():
            column -= bar.sizeHint().width()
        option = QStyleOptionViewItem()
        option.initFrom(view)
        shown = [i for i in range(view.topLevelItemCount())
                 if not view.topLevelItem(i).isHidden()][:self._keep_rows]
        needed = 0
        for row in shown:
            height = 0
            for col in range(view.columnCount()):
                index = model.index(row, col)
                hint = super().sizeHint(option, index)
                if col == self._column:
                    hint = self._wrapped(hint, option, index, column)
                height = max(height, hint.height())
            needed += height
        # Against the height the rows would have with the header up, for the
        # same reason as the scrollbar: putting it away is what this decides,
        # and the room that frees must not be what decides it back.  Measured
        # from the base class's hints too, which carry no elided trim.
        header = view.header()
        room = view.viewport().height()
        if header.isHidden():
            room -= header.sizeHint().height()
        wrap = needed <= room
        if wrap != self._wrap:
            self._wrap = wrap
            view.setWordWrap(wrap)
            header.setVisible(wrap)
            view.scheduleDelayedItemsLayout()
