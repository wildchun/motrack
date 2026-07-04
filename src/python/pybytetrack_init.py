"""pybytetrack — Python bindings for ByteTrackLib.

The compiled pybind11 extension lives at ``pybytetrack.pybytetrack``; this
wrapper re-exports its symbols so ``import pybytetrack`` continues to work
exactly like it did before the wheel packaging:

    >>> import pybytetrack as pybt
    >>> tracker = pybt.ByteTracker(max_age=30, track_thresh=0.3,
    ...                            heigh_thresh=0.6, match_thresh=0.8)
    >>> tracks = tracker.update([])
"""
from .pybytetrack import ByteTracker, Object, Rect, Track  # noqa: F401

__all__ = ["ByteTracker", "Object", "Rect", "Track"]
