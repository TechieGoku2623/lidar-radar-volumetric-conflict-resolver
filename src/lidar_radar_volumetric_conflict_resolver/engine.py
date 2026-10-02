"""Fuse synthetic lidar clusters with synthetic radar tracks.

Radar positions are predicted with constant radial velocity over ``dt``.
The velocity vector is ``range_rate`` times the position unit vector. A
degenerate position (no direction) skips that prediction. Association is
greedy gated nearest neighbor on Euclidean distance. A pair inside the
gate whose range rates differ by more than the bound is a conflict, as is
any not-yet-counted pair that shares a spatial cell and contradicts.

When both confidences are below the floor the pair is withheld. Otherwise
the published position is confidence-weighted and the range rate comes
from the higher-confidence sensor. A lidar track with no radar partner
inside the gate is a dropout, reported separately and not fused.

Ingress builds two lists. One merge step is the only place they meet,
which is the freedom-from-interference shape from ISO 26262. This module
claims no ASIL and does not open a vehicle bus.
"""

from __future__ import annotations

import asyncio
import logging
import math
import statistics
import struct
from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace

from . import wire
from .exceptions import EngineKernelException

_ARENA_M = 200.0
_ORIGIN_EPS = 1.0e-9
_LOGGER = logging.getLogger("lidar_radar_volumetric_conflict_resolver")
_LOGGER.addHandler(logging.NullHandler())


@dataclass(frozen=True, slots=True)
class _Track:
    sensor: str
    index: int
    x: float
    y: float
    z: float
    range_rate: float
    confidence: float


def _require_positive(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EngineKernelException(f"{name} must be numeric")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise EngineKernelException(f"{name} must be a positive finite value")
    return number


class LidarRadarVolumetricConflictResolver:
    """Separate lidar and radar ingest, then one explicit merge."""

    def __init__(
        self,
        arena_m: float = _ARENA_M,
        gate_m: float = 3.0,
        rate_bound: float = 2.5,
        confidence_floor: float = 0.35,
        dt_s: float = 0.05,
    ) -> None:
        self._arena_m = _require_positive("arena_m", arena_m)
        self._gate_m = _require_positive("gate_m", gate_m)
        self._rate_bound = _require_positive("rate_bound", rate_bound)
        self._floor = _require_positive("confidence_floor", confidence_floor)
        if self._floor > 1.0:
            raise EngineKernelException("confidence_floor must sit in (0, 1]")
        if isinstance(dt_s, bool) or not isinstance(dt_s, (int, float)):
            raise EngineKernelException("dt_s must be numeric")
        if not math.isfinite(float(dt_s)) or float(dt_s) < 0.0:
            raise EngineKernelException("dt_s must be a non-negative finite value")
        self._dt_s = float(dt_s)
        self._lock = asyncio.Lock()
        self._spool: deque[bytes] = deque(maxlen=64)
        self._logger = _LOGGER

    async def run(self, records: Sequence[object]) -> dict[str, object]:
        """Ingest each sensor on its own path, then merge once."""
        async with self._lock:
            self._guard_batch(records)
            lidar, radar = await asyncio.gather(
                self._ingest(records, "lidar"),
                self._ingest(records, "radar"),
            )
            return self._merge(lidar, radar)

    def _guard_batch(self, records: Sequence[object]) -> None:
        if isinstance(records, (str, bytes)) or not isinstance(records, Sequence):
            raise EngineKernelException("tracks must be a sequence of records")
        for record in records:
            if not isinstance(record, Mapping):
                raise EngineKernelException("track record must be a mapping")
            sensor = record.get("sensor")
            if sensor not in {"lidar", "radar"}:
                raise EngineKernelException("sensor tag must be lidar or radar")

    async def _ingest(
        self, records: Sequence[object], sensor: str
    ) -> tuple[_Track, ...]:
        await asyncio.sleep(0)
        selected: list[_Track] = []
        ordinal = 0
        for record in records:
            if not isinstance(record, Mapping) or record.get("sensor") != sensor:
                continue
            track = self._parse(record, sensor, ordinal)
            if sensor == "radar":
                track = self._predict(track)
            selected.append(track)
            ordinal += 1
        return tuple(selected)

    def _parse(self, record: Mapping[str, object], sensor: str, index: int) -> _Track:
        x = self._coord(record.get("x"), sensor)
        y = self._coord(record.get("y"), sensor)
        z = self._coord(record.get("z"), sensor)
        range_rate = self._coord(record.get("range_rate"), sensor)
        confidence = self._coord(record.get("confidence"), sensor)
        if max(abs(x), abs(y), abs(z)) > self._arena_m:
            raise EngineKernelException(f"{sensor} coordinate outside the arena")
        if confidence < 0.0 or confidence > 1.0:
            raise EngineKernelException(
                f"{sensor} confidence outside the unit interval"
            )
        return _Track(sensor, index, x, y, z, range_rate, confidence)

    def _coord(self, value: object, sensor: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise EngineKernelException(f"{sensor} track field is not numeric")
        number = float(value)
        if not math.isfinite(number):
            raise EngineKernelException(f"{sensor} coordinate is not finite")
        return number

    def _predict(self, track: _Track) -> _Track:
        radius = math.hypot(track.x, track.y, track.z)
        if radius < _ORIGIN_EPS:
            return track
        step = track.range_rate * self._dt_s / radius
        predicted = replace(
            track,
            x=track.x + track.x * step,
            y=track.y + track.y * step,
            z=track.z + track.z * step,
        )
        if max(abs(predicted.x), abs(predicted.y), abs(predicted.z)) > self._arena_m:
            raise EngineKernelException("predicted radar coordinate outside the arena")
        return predicted

    def _merge(
        self, lidar: Sequence[_Track], radar: Sequence[_Track]
    ) -> dict[str, object]:
        used: set[int] = set()
        fused = 0
        withheld = 0
        dropouts = 0
        conflicts = 0
        published: list[dict[str, float]] = []
        counted: set[tuple[tuple[str, int], tuple[str, int]]] = set()

        for item in lidar:
            match = self._nearest(item, radar, used)
            if match is None:
                dropouts += 1
                self._logger.warning(
                    "lidar dropout x=%.2f y=%.2f z=%.2f", item.x, item.y, item.z
                )
                continue
            index, partner, _distance = match
            used.add(index)
            if self._note_conflict(item, partner, counted):
                conflicts += 1
            if item.confidence < self._floor and partner.confidence < self._floor:
                withheld += 1
                self._logger.warning(
                    "track withheld x=%.2f y=%.2f z=%.2f", item.x, item.y, item.z
                )
                continue
            published.append(self._blend(item, partner))
            fused += 1

        cells: dict[tuple[int, int, int], list[_Track]] = {}
        for track in (*lidar, *radar):
            cells.setdefault(self._cell(track), []).append(track)
        for members in cells.values():
            for left_i in range(len(members)):
                for right_i in range(left_i + 1, len(members)):
                    if self._note_conflict(members[left_i], members[right_i], counted):
                        conflicts += 1

        frame = struct.pack(wire.SUMMARY_FORMAT, fused, withheld, dropouts, conflicts)
        if wire.unpack_summary(frame) != (fused, withheld, dropouts, conflicts):
            raise EngineKernelException("summary frame failed struct roundtrip")
        self._spool.append(frame)
        for item in published:
            self._spool.append(
                wire.pack_track(
                    wire.TAG_LIDAR,
                    item["x"],
                    item["y"],
                    item["z"],
                    item["range_rate"],
                    item["confidence"],
                )
            )
        return {
            "fused": fused,
            "withheld": withheld,
            "dropouts": dropouts,
            "conflicts": conflicts,
            "published": published,
            "frame": frame,
        }

    def _nearest(
        self,
        lidar: _Track,
        radar: Sequence[_Track],
        used: set[int],
    ) -> tuple[int, _Track, float] | None:
        best_index = -1
        best_distance = math.inf
        best: _Track | None = None
        for index, candidate in enumerate(radar):
            if index in used:
                continue
            distance = math.dist(
                (lidar.x, lidar.y, lidar.z),
                (candidate.x, candidate.y, candidate.z),
            )
            if distance <= self._gate_m and distance < best_distance:
                best_distance = distance
                best_index = index
                best = candidate
        if best is None:
            return None
        return best_index, best, best_distance

    def _contradicts(self, left: _Track, right: _Track) -> bool:
        # pstdev of two samples is half the absolute difference.
        spread = statistics.pstdev((left.range_rate, right.range_rate))
        return spread * 2.0 > self._rate_bound

    def _note_conflict(
        self,
        left: _Track,
        right: _Track,
        counted: set[tuple[tuple[str, int], tuple[str, int]]],
    ) -> bool:
        if not self._contradicts(left, right):
            return False
        key = self._pair_key(left, right)
        if key in counted:
            return False
        counted.add(key)
        spread = statistics.pstdev((left.range_rate, right.range_rate))
        self._logger.warning("range-rate conflict spread=%.4f", spread)
        return True

    def _pair_key(
        self, left: _Track, right: _Track
    ) -> tuple[tuple[str, int], tuple[str, int]]:
        left_key = (left.sensor, left.index)
        right_key = (right.sensor, right.index)
        if left_key <= right_key:
            return left_key, right_key
        return right_key, left_key

    def _cell(self, track: _Track) -> tuple[int, int, int]:
        size = self._gate_m
        return (
            math.floor(track.x / size),
            math.floor(track.y / size),
            math.floor(track.z / size),
        )

    def _blend(self, lidar: _Track, radar: _Track) -> dict[str, float]:
        total = lidar.confidence + radar.confidence
        if total <= 0.0:
            raise EngineKernelException("confidence weight is undefined")
        lidar_weight = lidar.confidence / total
        radar_weight = 1.0 - lidar_weight
        if radar.confidence > lidar.confidence:
            range_rate = radar.range_rate
        else:
            range_rate = lidar.range_rate
        return {
            "x": lidar_weight * lidar.x + radar_weight * radar.x,
            "y": lidar_weight * lidar.y + radar_weight * radar.y,
            "z": lidar_weight * lidar.z + radar_weight * radar.z,
            "range_rate": range_rate,
            "confidence": statistics.fmean((lidar.confidence, radar.confidence)),
        }
