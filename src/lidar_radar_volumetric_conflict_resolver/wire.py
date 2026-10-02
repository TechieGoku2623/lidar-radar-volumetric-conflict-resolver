"""Numeric frames for one merge.

The summary is four uint32 counters: fused, withheld, dropouts, conflicts.
A track frame is a sensor tag plus five float64 fields. Neither layout
carries a bus identifier.
"""

from __future__ import annotations

import struct

from .exceptions import EngineKernelException

SUMMARY_FORMAT = ">IIII"
TRACK_FORMAT = ">Bddddd"
TAG_LIDAR = 1
TAG_RADAR = 2


def pack_summary(fused: int, withheld: int, dropouts: int, conflicts: int) -> bytes:
    """Pack the four merge counters."""
    return struct.pack(
        SUMMARY_FORMAT, int(fused), int(withheld), int(dropouts), int(conflicts)
    )


def unpack_summary(payload: bytes) -> tuple[int, int, int, int]:
    """Inverse of :func:`pack_summary`."""
    expected = struct.calcsize(SUMMARY_FORMAT)
    if len(payload) != expected:
        raise EngineKernelException("fusion summary length is not 16 bytes")
    fused, withheld, dropouts, conflicts = struct.unpack(SUMMARY_FORMAT, payload)
    return int(fused), int(withheld), int(dropouts), int(conflicts)


def pack_track(
    tag: int,
    x: float,
    y: float,
    z: float,
    range_rate: float,
    confidence: float,
) -> bytes:
    """Pack one numeric track. ``tag`` is 1 for lidar and 2 for radar."""
    return struct.pack(
        TRACK_FORMAT,
        int(tag),
        float(x),
        float(y),
        float(z),
        float(range_rate),
        float(confidence),
    )


def unpack_track(payload: bytes) -> tuple[int, float, float, float, float, float]:
    """Inverse of :func:`pack_track`."""
    expected = struct.calcsize(TRACK_FORMAT)
    if len(payload) != expected:
        raise EngineKernelException("fusion track length is not 41 bytes")
    tag, x, y, z, range_rate, confidence = struct.unpack(TRACK_FORMAT, payload)
    return (
        int(tag),
        float(x),
        float(y),
        float(z),
        float(range_rate),
        float(confidence),
    )
