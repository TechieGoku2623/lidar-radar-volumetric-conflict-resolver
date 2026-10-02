"""Fixed-seed latency sample for the volumetric resolver."""

from __future__ import annotations

import asyncio
import math
import random
import statistics
import sys
import time
import tracemalloc

from .engine import LidarRadarVolumetricConflictResolver

SEED = 26262
ITERATIONS = 5000
WARMUP = 20


def _empirical_p99(samples: list[float]) -> float:
    ordered = sorted(samples)
    rank = math.ceil(0.99 * len(ordered))
    index = min(len(ordered), max(1, rank)) - 1
    return ordered[index]


def _records(rng: random.Random) -> list[dict[str, float | str]]:
    base = 12.0 + rng.uniform(-0.05, 0.05)
    return [
        {
            "sensor": "lidar",
            "x": base,
            "y": -4.0,
            "z": 0.5,
            "range_rate": 6.0,
            "confidence": 0.82,
        },
        {
            "sensor": "radar",
            "x": base + 0.25,
            "y": -3.9,
            "z": 0.45,
            "range_rate": 6.2,
            "confidence": 0.70,
        },
    ]


async def _execute() -> dict[str, object]:
    rng = random.Random(SEED)
    records = _records(rng)
    engine = LidarRadarVolumetricConflictResolver(arena_m=200.0, dt_s=0.05)
    for _ in range(WARMUP):
        await engine.run(records)
    latencies: list[float] = []
    tracemalloc.start()
    try:
        last = None
        for _ in range(ITERATIONS):
            started = time.perf_counter_ns()
            last = await engine.run(records)
            latencies.append((time.perf_counter_ns() - started) / 1000.0)
        _current, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    if (
        last is None
        or int(last["fused"]) != 1
        or int(last["withheld"])
        or int(last["dropouts"])
        or int(last["conflicts"])
    ):
        raise RuntimeError("benchmark pair did not fuse cleanly")
    return {
        "status": "ok",
        "seed": SEED,
        "iterations": ITERATIONS,
        "latency_us": round(latencies[-1], 3),
        "memory_peak_bytes": int(peak),
        "benchmark_avg_us": round(statistics.fmean(latencies), 3),
        "benchmark_p99_us": round(_empirical_p99(latencies), 3),
    }


def main() -> int:
    try:
        report = asyncio.run(_execute())
    except Exception as exc:
        print(f"status=error detail={type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(
        "status={status} seed={seed} iterations={iterations} "
        "latency_us={latency_us:.3f} memory_peak_bytes={memory_peak_bytes} "
        "benchmark_avg_us={benchmark_avg_us:.3f} "
        "benchmark_p99_us={benchmark_p99_us:.3f}".format(**report)
    )
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
