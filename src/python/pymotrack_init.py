"""pymotrack — Python bindings for motrack.

The compiled pybind11 extension lives at ``pymotrack.pymotrack``; this
wrapper re-exports its symbols so ``import pymotrack`` works directly:

    >>> import pymotrack as mt
    >>> tracker = mt.Tracker(mt.TrackerType.ByteTrack)
    >>> tracks = tracker.update(objects)
"""

from .pymotrack import (  # noqa: F401
    Object,
    Rect,
    Track,
    Tracker,
    TrackerConfig,
    TrackerType,
)

__all__ = [
    "Object",
    "Rect",
    "Track",
    "Tracker",
    "TrackerConfig",
    "TrackerType",
]
