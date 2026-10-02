"""Deterministic checks and a latency sample for the volumetric resolver."""

from __future__ import annotations

import asyncio
import importlib
import json
import math
import random
import statistics
import struct
import sys
import time
import tracemalloc
from pathlib import Path

SEED = 26262
ITERATIONS = 5000
WARMUP = 20


def load_module():
    root = str(Path(__file__).resolve().parents[1])
    if root not in sys.path:
        sys.path.insert(0, root)
    return importlib.import_module("src.main")


def expect(condition: bool, detail: object) -> None:
    if not condition:
        raise AssertionError(detail)


def empirical_p99(samples: list[float]) -> float:
    ordered = sorted(samples)
    rank = math.ceil(0.99 * len(ordered))
    index = min(len(ordered), max(1, rank)) - 1
    return ordered[index]


def close_to(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-9, abs_tol=1e-9)


def wire_close(left: float, right: float) -> bool:
    """Binary32 frames round the binary64 coordinates kept on the track."""
    return math.isclose(left, right, rel_tol=1e-5, abs_tol=1e-5)


async def check_fused_agreement(mod, rng: random.Random) -> None:
    resolver = mod.VolumetricConflictResolver(arena_m=200.0)
    jitter = rng.uniform(-0.05, 0.05)
    lidar = [(12.0 + jitter, -4.0, 0.5, 6.0, 0.82)]
    radar = [(12.3 + jitter, -3.9, 0.45, 6.4, 0.70)]
    try:
        report = await resolver.resolve(lidar, radar)
    finally:
        await resolver.close()
    expect(report["status"] == "merged", report)
    expect(report["conflict_count"] == 0, report)
    expect(report["dropout_count"] == 0, report)
    track = report["tracks"][0]
    expect(track["state"] == "fused", track)
    expect(track["publish"] is True, track)
    expect(track["range_rate_source"] == "blend", track)
    expect(track["conflict"] is False, track)
    blob = bytes.fromhex(track["frame_hex"])
    x, y, z, rate, confidence, code = struct.unpack(">fffffB", blob)
    expect(code == 1, code)
    expect(wire_close(x, track["x"]), (x, track["x"]))
    expect(wire_close(y, track["y"]), (y, track["y"]))
    expect(wire_close(z, track["z"]), (z, track["z"]))
    expect(wire_close(rate, track["range_rate"]), (rate, track["range_rate"]))
    expect(confidence > 0.0, confidence)


async def check_contradictory_same_cell(mod) -> None:
    resolver = mod.VolumetricConflictResolver(arena_m=200.0)
    lidar = (5.0, 5.0, 1.0, 12.0, 0.90)
    radar = (5.20, 5.10, 1.05, -8.0, 0.40)
    weight = 0.90 / (0.90 + 0.40)
    other = 1.0 - weight
    expected = (
        weight * lidar[0] + other * radar[0],
        weight * lidar[1] + other * radar[1],
        weight * lidar[2] + other * radar[2],
    )
    try:
        report = await resolver.resolve([lidar], [radar])
    finally:
        await resolver.close()
    expect(report["conflict_count"] == 1, report)
    track = report["tracks"][0]
    expect(track["state"] == "resolved", track)
    expect(track["conflict"] is True, track)
    expect(track["same_cell"] is True, track)
    expect(track["publish"] is True, track)
    expect(track["range_rate_source"] == "lidar", track)
    expect(close_to(track["range_rate"], lidar[3]), track)
    expect(close_to(track["x"], expected[0]), (track["x"], expected[0]))
    expect(close_to(track["y"], expected[1]), (track["y"], expected[1]))
    expect(close_to(track["z"], expected[2]), (track["z"], expected[2]))
    expect(track["range_rate_delta"] > 2.5, track)


async def check_low_confidence_withhold(mod) -> None:
    resolver = mod.VolumetricConflictResolver(arena_m=200.0)
    lidar = (0.0, 0.0, 0.0, 4.0, 0.20)
    radar = (0.20, 0.0, 0.0, -6.0, 0.10)
    try:
        report = await resolver.resolve([lidar], [radar])
    finally:
        await resolver.close()
    track = report["tracks"][0]
    expect(track["state"] == "withheld", track)
    expect(track["publish"] is False, track)
    expect(track["x"] is None, track)
    expect(track["conflict"] is True, track)
    expect(report["withheld_count"] == 1, report)
    blob = bytes.fromhex(track["frame_hex"])
    x, y, z, rate, confidence, code = struct.unpack(">fffffB", blob)
    expect(code == 3, code)
    expect(x == 0.0 and y == 0.0 and z == 0.0, blob)
    expect(rate == 0.0 and confidence == 0.0, blob)


async def check_lidar_dropout(mod) -> None:
    resolver = mod.VolumetricConflictResolver(arena_m=200.0)
    paired_lidar = (0.0, 0.0, 0.0, 2.0, 0.80)
    orphan_lidar = (30.0, -8.0, 0.4, 1.5, 0.77)
    radar = (0.25, 0.10, 0.0, 2.2, 0.74)
    try:
        report = await resolver.resolve([paired_lidar, orphan_lidar], [radar])
    finally:
        await resolver.close()
    expect(report["dropout_count"] == 1, report)
    expect(report["track_count"] == 2, report)
    states = {(track["source"], track["state"]) for track in report["tracks"]}
    expect(("pair", "fused") in states, report["tracks"])
    orphans = [
        track
        for track in report["tracks"]
        if track["source"] == "lidar" and track["state"] == "dropout"
    ]
    expect(len(orphans) == 1, report["tracks"])
    expect(close_to(orphans[0]["x"], 30.0), orphans[0])
    expect(orphans[0]["publish"] is False, orphans[0])


async def check_rejects_bad_tracks(mod) -> None:
    resolver = mod.VolumetricConflictResolver(arena_m=200.0)
    finite = False
    outside = False
    fast = False
    try:
        try:
            await resolver.resolve([(math.nan, 0.0, 0.0, 0.0, 0.5)], [])
        except mod.EngineKernelException:
            finite = True
        try:
            await resolver.resolve([(250.0, 0.0, 0.0, 0.0, 0.5)], [])
        except mod.EngineKernelException:
            outside = True
        try:
            await resolver.resolve([(0.0, 0.0, 0.0, 81.0, 0.5)], [])
        except mod.EngineKernelException:
            fast = True
        edge = await resolver.resolve(
            [(200.0, 0.0, 0.0, 0.0, 0.6)],
            [(199.5, 0.2, 0.0, 0.2, 0.6)],
        )
    finally:
        await resolver.close()
    expect(finite, "non-finite coordinate was fused")
    expect(outside, "track outside the arena was fused")
    expect(fast, "range rate outside the bound was fused")
    expect(edge["tracks"][0]["state"] == "fused", edge)


async def run_benchmark(mod) -> dict[str, float | int]:
    resolver = mod.VolumetricConflictResolver(arena_m=200.0)
    lidar = [(12.0, -4.0, 0.5, 6.0, 0.82)]
    radar = [(12.3, -3.9, 0.45, 6.4, 0.70)]
    latencies: list[float] = []
    try:
        for _ in range(WARMUP):
            await resolver.resolve(lidar, radar)
        tracemalloc.start()
        last = None
        for _ in range(ITERATIONS):
            started = time.perf_counter_ns()
            last = await resolver.resolve(lidar, radar)
            latencies.append((time.perf_counter_ns() - started) / 1000.0)
        _current, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
        await resolver.close()
    expect(last is not None and last["tracks"][0]["state"] == "fused", last)
    return {
        "n": ITERATIONS,
        "avg_us": round(statistics.fmean(latencies), 3),
        "p99_us": round(empirical_p99(latencies), 3),
        "peak_bytes": peak,
    }


async def execute(name: str, func) -> dict[str, object]:
    try:
        await func()
    except Exception as exc:
        return {
            "name": name,
            "passed": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    return {"name": name, "passed": True}


async def amain() -> dict[str, object]:
    mod = load_module()
    rng = random.Random(SEED)
    checks = [
        ("fused_agreement", lambda: check_fused_agreement(mod, rng)),
        ("contradictory_same_cell", lambda: check_contradictory_same_cell(mod)),
        ("low_confidence_withhold", lambda: check_low_confidence_withhold(mod)),
        ("lidar_dropout", lambda: check_lidar_dropout(mod)),
        ("reject_non_finite_and_arena", lambda: check_rejects_bad_tracks(mod)),
    ]
    results = []
    for name, func in checks:
        results.append(await execute(name, func))
    benchmark = await run_benchmark(mod)
    status = "PASS" if all(item["passed"] for item in results) else "FAIL"
    return {
        "status": status,
        "seed": SEED,
        "checks": results,
        "benchmark": benchmark,
    }


def main() -> int:
    summary = asyncio.run(amain())
    benchmark = summary["benchmark"]
    print(
        "benchmark "
        f"n={benchmark['n']} avg_us={benchmark['avg_us']:.3f} "
        f"p99_us={benchmark['p99_us']:.3f} peak_bytes={benchmark['peak_bytes']}"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
