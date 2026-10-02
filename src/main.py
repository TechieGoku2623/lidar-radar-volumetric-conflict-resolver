"""Fuse synthetic lidar clusters with synthetic radar tracks.

Each path is parsed on its own and handed across its own queue. A single merge
step is the only place the two batches meet. Overlapping volumes whose range
rates disagree are resolved by confidence-weighted position, or withheld when
both confidences are low. The split is aligned with the freedom-from-interference
idea in ISO 26262. This module does not claim an ASIL and does not talk to a
vehicle bus. Inputs are numeric tracks the caller already has.
"""

from __future__ import annotations

import asyncio
import logging
import math
import statistics
import struct
import sys
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass

FUSION_TOPIC = "vehicle.perception.fused_track"
DEFAULT_ARENA_M = 200.0
DEFAULT_GATE_M = 3.0
DEFAULT_RATE_BOUND = 2.5
DEFAULT_LOW_CONFIDENCE = 0.35
DEFAULT_MAX_RANGE_RATE = 80.0

_STATE_CODE = {
    "fused": 1,
    "resolved": 2,
    "withheld": 3,
    "dropout": 4,
}

__all__ = [
    "EngineKernelException",
    "FUSION_TOPIC",
    "SensorTrack",
    "VolumetricConflictResolver",
    "configure_logging",
    "main",
]


class EngineKernelException(Exception):
    """A track that cannot enter the arena without corrupting the merge."""


def configure_logging() -> None:
    """Install a timestamped handler once, if the process has none yet."""
    if logging.getLogger().handlers:
        return
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S%z",
    )


@dataclass(frozen=True, slots=True)
class SensorTrack:
    """One cluster or track: meters, meters per second, unit confidence."""

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


def _finite_float(item: object, source: str) -> float:
    if isinstance(item, bool) or not isinstance(item, (int, float)):
        raise EngineKernelException(f"{source} track field is not numeric")
    value = float(item)
    if not math.isfinite(value):
        raise EngineKernelException(f"{source} coordinate is not finite")
    return value


def _five_floats(
    track: Sequence[float], source: str
) -> tuple[float, float, float, float, float]:
    if isinstance(track, (str, bytes)) or not isinstance(track, Sequence):
        raise EngineKernelException(f"{source} track must be a five-field tuple")
    if len(track) != 5:
        raise EngineKernelException(f"{source} track must be a five-field tuple")
    values = [_finite_float(item, source) for item in track]
    return (values[0], values[1], values[2], values[3], values[4])


def _separation(left: SensorTrack, right: SensorTrack) -> float:
    return math.dist((left.x, left.y, left.z), (right.x, right.y, right.z))


def _cell_of(track: SensorTrack, cell_m: float) -> tuple[int, int, int]:
    return (
        math.floor(track.x / cell_m),
        math.floor(track.y / cell_m),
        math.floor(track.z / cell_m),
    )


def _weights(lidar: SensorTrack, radar: SensorTrack) -> tuple[float, float]:
    total = lidar.confidence + radar.confidence
    if total <= 0.0:
        return 0.5, 0.5
    lidar_weight = lidar.confidence / total
    return lidar_weight, 1.0 - lidar_weight


def _blend_position(
    lidar: SensorTrack, radar: SensorTrack, lidar_weight: float, radar_weight: float
) -> tuple[float, float, float]:
    return (
        lidar_weight * lidar.x + radar_weight * radar.x,
        lidar_weight * lidar.y + radar_weight * radar.y,
        lidar_weight * lidar.z + radar_weight * radar.z,
    )


def _select_range_rate(
    lidar: SensorTrack,
    radar: SensorTrack,
    agree: bool,
    lidar_weight: float,
    radar_weight: float,
) -> tuple[float, str]:
    if not agree:
        if lidar.confidence >= radar.confidence:
            return lidar.range_rate, "lidar"
        return radar.range_rate, "radar"
    blended = lidar_weight * lidar.range_rate + radar_weight * radar.range_rate
    return blended, "blend"


def pack_frame(
    state: str,
    x: float,
    y: float,
    z: float,
    range_rate: float,
    confidence: float,
) -> bytes:
    """Pack one fused frame. The record is numeric."""
    return struct.pack(
        ">fffffB",
        x,
        y,
        z,
        range_rate,
        confidence,
        _STATE_CODE[state],
    )


class VolumetricConflictResolver:
    """Isolate lidar and radar ingress, then merge explicitly.

    The two queues are the freedom-from-interference boundary inside this
    process. ``_spool`` holds packed frames that a producer for
    ``vehicle.perception.fused_track`` would drain. No bus client is opened.
    """

    def __init__(
        self,
        arena_m: float = DEFAULT_ARENA_M,
        gate_m: float = DEFAULT_GATE_M,
        rate_bound: float = DEFAULT_RATE_BOUND,
        low_confidence: float = DEFAULT_LOW_CONFIDENCE,
        max_range_rate: float = DEFAULT_MAX_RANGE_RATE,
        ledger_limit: int = 256,
    ) -> None:
        configure_logging()
        self._arena_m = _require_positive("arena_m", arena_m)
        self._gate_m = _require_positive("gate_m", gate_m)
        self._rate_bound = _require_positive("rate_bound", rate_bound)
        self._low_confidence = _require_positive("low_confidence", low_confidence)
        if self._low_confidence > 1.0:
            raise EngineKernelException("low_confidence must sit in (0, 1]")
        self._max_range_rate = _require_positive("max_range_rate", max_range_rate)
        if isinstance(ledger_limit, bool) or not isinstance(ledger_limit, int):
            raise EngineKernelException("ledger_limit must be an integer")
        if ledger_limit < 1:
            raise EngineKernelException("ledger_limit must be at least 1")
        self._ledger: deque[dict[str, object]] = deque(maxlen=ledger_limit)
        self._spool: deque[bytes] = deque(maxlen=ledger_limit)
        self._lock = asyncio.Lock()
        self._lidar_q: asyncio.Queue[tuple[SensorTrack, ...]] = asyncio.Queue()
        self._radar_q: asyncio.Queue[
            tuple[tuple[SensorTrack, ...], asyncio.Future[dict[str, object]]]
        ] = asyncio.Queue()
        self._merger: asyncio.Task[None] | None = None
        self._logger = logging.getLogger("volumetric.kernel")

    async def _ensure(self) -> None:
        async with self._lock:
            if self._merger is None or self._merger.done():
                self._merger = asyncio.create_task(
                    self._merge_loop(), name="volumetric-merge"
                )

    async def close(self) -> None:
        task = self._merger
        self._merger = None
        if task is None or task.done():
            return
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            return

    async def resolve(
        self,
        lidar_tracks: Sequence[Sequence[float]],
        radar_tracks: Sequence[Sequence[float]],
    ) -> dict[str, object]:
        """Parse each sensor path, enqueue both, and wait for the merge."""
        await self._ensure()
        lidar_batch, radar_batch = await asyncio.gather(
            self._ingress_lidar(lidar_tracks),
            self._ingress_radar(radar_tracks),
        )
        future: asyncio.Future[dict[str, object]] = (
            asyncio.get_running_loop().create_future()
        )
        self._lidar_q.put_nowait(lidar_batch)
        self._radar_q.put_nowait((radar_batch, future))
        return await future

    async def _ingress_lidar(
        self, tracks: Sequence[Sequence[float]]
    ) -> tuple[SensorTrack, ...]:
        await asyncio.sleep(0)
        return self._parse_batch(tracks, "lidar")

    async def _ingress_radar(
        self, tracks: Sequence[Sequence[float]]
    ) -> tuple[SensorTrack, ...]:
        await asyncio.sleep(0)
        return self._parse_batch(tracks, "radar")

    def _parse_batch(
        self, tracks: Sequence[Sequence[float]], source: str
    ) -> tuple[SensorTrack, ...]:
        if isinstance(tracks, (str, bytes)) or not isinstance(tracks, Sequence):
            raise EngineKernelException(f"{source} batch must be a sequence of tracks")
        return tuple(self._parse_track(track, source) for track in tracks)

    def _parse_track(self, track: Sequence[float], source: str) -> SensorTrack:
        x, y, z, range_rate, confidence = _five_floats(track, source)
        if max(abs(x), abs(y), abs(z)) > self._arena_m:
            raise EngineKernelException(f"{source} coordinate outside the arena")
        if abs(range_rate) > self._max_range_rate:
            raise EngineKernelException(f"{source} range rate outside the arena bound")
        if confidence < 0.0 or confidence > 1.0:
            raise EngineKernelException(
                f"{source} confidence outside the unit interval"
            )
        return SensorTrack(x, y, z, range_rate, confidence)

    async def _merge_loop(self) -> None:
        while True:
            lidar_batch = await self._lidar_q.get()
            try:
                radar_batch, future = await self._radar_q.get()
            except asyncio.CancelledError:
                self._lidar_q.task_done()
                raise
            try:
                result = self._merge(lidar_batch, radar_batch)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if not future.done():
                    future.set_exception(exc)
            else:
                if not future.done():
                    future.set_result(result)
            finally:
                self._lidar_q.task_done()
                self._radar_q.task_done()

    def _merge(
        self,
        lidar_batch: Sequence[SensorTrack],
        radar_batch: Sequence[SensorTrack],
    ) -> dict[str, object]:
        used: set[int] = set()
        tracks: list[dict[str, object]] = []
        for lidar in lidar_batch:
            match = self._nearest(lidar, radar_batch, used)
            if match is None:
                tracks.append(self._dropout(lidar, "lidar"))
                continue
            index, radar, distance = match
            used.add(index)
            tracks.append(self._combine(lidar, radar, distance))
        for index, radar in enumerate(radar_batch):
            if index not in used:
                tracks.append(self._dropout(radar, "radar"))
        return self._summarize(tracks)

    def _nearest(
        self,
        lidar: SensorTrack,
        radar_batch: Sequence[SensorTrack],
        used: set[int],
    ) -> tuple[int, SensorTrack, float] | None:
        best_index = -1
        best_distance = math.inf
        best_track: SensorTrack | None = None
        for index, radar in enumerate(radar_batch):
            if index in used:
                continue
            distance = _separation(lidar, radar)
            same_cell = _cell_of(lidar, self._gate_m) == _cell_of(radar, self._gate_m)
            if distance <= self._gate_m or same_cell:
                if distance < best_distance:
                    best_distance = distance
                    best_index = index
                    best_track = radar
        if best_track is None:
            return None
        return best_index, best_track, best_distance

    def _combine(
        self, lidar: SensorTrack, radar: SensorTrack, distance: float
    ) -> dict[str, object]:
        rate_delta = abs(lidar.range_rate - radar.range_rate)
        conflict = rate_delta > self._rate_bound
        both_low = (
            lidar.confidence < self._low_confidence
            and radar.confidence < self._low_confidence
        )
        if both_low:
            self._logger.warning(
                "track withheld reason=low_confidence delta=%.2f x=%.2f y=%.2f z=%.2f",
                rate_delta,
                lidar.x,
                lidar.y,
                lidar.z,
            )
            return self._withheld(lidar, radar, distance, rate_delta, conflict)
        if conflict:
            self._logger.warning(
                "range-rate conflict resolved by confidence weight delta=%.2f",
                rate_delta,
            )
            return self._resolved(lidar, radar, distance, rate_delta)
        return self._fused(lidar, radar, distance, rate_delta)

    def _pair_metrics(
        self, lidar: SensorTrack, radar: SensorTrack
    ) -> tuple[float, float, tuple[int, int, int], bool]:
        mean_confidence = statistics.mean((lidar.confidence, radar.confidence))
        spread = statistics.pstdev((lidar.range_rate, radar.range_rate))
        lidar_cell = _cell_of(lidar, self._gate_m)
        same_cell = lidar_cell == _cell_of(radar, self._gate_m)
        return mean_confidence, spread, lidar_cell, same_cell

    def _fused(
        self,
        lidar: SensorTrack,
        radar: SensorTrack,
        distance: float,
        rate_delta: float,
    ) -> dict[str, object]:
        return self._published(
            "fused", lidar, radar, distance, rate_delta, conflict=False
        )

    def _resolved(
        self,
        lidar: SensorTrack,
        radar: SensorTrack,
        distance: float,
        rate_delta: float,
    ) -> dict[str, object]:
        return self._published(
            "resolved", lidar, radar, distance, rate_delta, conflict=True
        )

    def _published(
        self,
        state: str,
        lidar: SensorTrack,
        radar: SensorTrack,
        distance: float,
        rate_delta: float,
        conflict: bool,
    ) -> dict[str, object]:
        lidar_weight, radar_weight = _weights(lidar, radar)
        x, y, z = _blend_position(lidar, radar, lidar_weight, radar_weight)
        range_rate, rate_source = _select_range_rate(
            lidar,
            radar,
            agree=not conflict,
            lidar_weight=lidar_weight,
            radar_weight=radar_weight,
        )
        mean_confidence, spread, cell, same_cell = self._pair_metrics(lidar, radar)
        confidence = mean_confidence
        frame = pack_frame(state, x, y, z, range_rate, confidence)
        self._spool.append(frame)
        return {
            "state": state,
            "source": "pair",
            "x": x,
            "y": y,
            "z": z,
            "range_rate": range_rate,
            "range_rate_source": rate_source,
            "confidence": confidence,
            "distance_m": distance,
            "range_rate_delta": rate_delta,
            "conflict": conflict,
            "publish": True,
            "same_cell": same_cell,
            "cell": list(cell),
            "mean_confidence": mean_confidence,
            "range_rate_spread": spread,
            "frame_hex": frame.hex(),
        }

    def _withheld(
        self,
        lidar: SensorTrack,
        radar: SensorTrack,
        distance: float,
        rate_delta: float,
        conflict: bool,
    ) -> dict[str, object]:
        mean_confidence, spread, cell, same_cell = self._pair_metrics(lidar, radar)
        frame = pack_frame("withheld", 0.0, 0.0, 0.0, 0.0, 0.0)
        return {
            "state": "withheld",
            "source": "pair",
            "x": None,
            "y": None,
            "z": None,
            "range_rate": None,
            "range_rate_source": None,
            "confidence": None,
            "distance_m": distance,
            "range_rate_delta": rate_delta,
            "conflict": conflict,
            "publish": False,
            "same_cell": same_cell,
            "cell": list(cell),
            "mean_confidence": mean_confidence,
            "range_rate_spread": spread,
            "frame_hex": frame.hex(),
        }

    def _dropout(self, track: SensorTrack, source: str) -> dict[str, object]:
        self._logger.warning(
            "sensor dropout source=%s x=%.2f y=%.2f z=%.2f",
            source,
            track.x,
            track.y,
            track.z,
        )
        frame = pack_frame(
            "dropout", track.x, track.y, track.z, track.range_rate, track.confidence
        )
        cell = _cell_of(track, self._gate_m)
        return {
            "state": "dropout",
            "source": source,
            "x": track.x,
            "y": track.y,
            "z": track.z,
            "range_rate": track.range_rate,
            "range_rate_source": source,
            "confidence": track.confidence,
            "distance_m": None,
            "range_rate_delta": None,
            "conflict": False,
            "publish": False,
            "same_cell": False,
            "cell": list(cell),
            "mean_confidence": statistics.mean((track.confidence,)),
            "range_rate_spread": statistics.pstdev((track.range_rate,)),
            "frame_hex": frame.hex(),
        }

    def _summarize(self, tracks: list[dict[str, object]]) -> dict[str, object]:
        conflict_count = 0
        dropout_count = 0
        withheld_count = 0
        for track in tracks:
            if track["conflict"]:
                conflict_count += 1
            if track["state"] == "dropout":
                dropout_count += 1
            if track["state"] == "withheld":
                withheld_count += 1
        result: dict[str, object] = {
            "status": "merged",
            "tracks": tracks,
            "conflict_count": conflict_count,
            "dropout_count": dropout_count,
            "withheld_count": withheld_count,
            "track_count": len(tracks),
            "track_topic": FUSION_TOPIC,
        }
        self._ledger.append(result)
        return {
            "status": "merged",
            "tracks": [dict(track) for track in tracks],
            "conflict_count": conflict_count,
            "dropout_count": dropout_count,
            "withheld_count": withheld_count,
            "track_count": len(tracks),
            "track_topic": FUSION_TOPIC,
        }


async def _demo() -> dict[str, object]:
    resolver = VolumetricConflictResolver(arena_m=200.0)
    lidar = (
        (5.0, 5.0, 1.0, 12.0, 0.92),
        (30.0, -8.0, 0.4, 1.5, 0.80),
    )
    radar = ((5.15, 5.05, 1.02, -6.0, 0.61),)
    try:
        return await resolver.resolve(lidar, radar)
    finally:
        await resolver.close()


def main() -> int:
    configure_logging()
    logger = logging.getLogger("volumetric.kernel")
    try:
        report = asyncio.run(_demo())
    except EngineKernelException:
        logger.exception("demo tracks rejected")
        return 1
    logger.info(
        "demo complete tracks=%s conflicts=%s dropouts=%s",
        report["track_count"],
        report["conflict_count"],
        report["dropout_count"],
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
