# LiDAR Radar Volumetric Conflict Resolver

A high-throughput, low-latency asynchronous engine engineered to resolve contradictory synthetic lidar and radar tracks inside a spatial gate by publishing a confidence-weighted position and the higher-confidence range rate, or by withholding the pair when both confidences sit below the floor.

Website: https://github.com/TechieGoku2623/lidar-radar-volumetric-conflict-resolver

Topics: `python` `asyncio` `autonomous-vehicles` `lidar` `radar` `sensor-fusion`


## 🏗️ Systems Architecture & Event Topology

`LidarRadarVolumetricConflictResolver.run` takes one scan of synthetic tracks. Each record is a mapping: `sensor` (`lidar` or `radar`), `x`, `y`, `z` in meters, `range_rate` in meters per second, and `confidence` in `[0, 1]`. Lidar ingress and radar ingress are separate coroutines joined with `asyncio.gather`. A single `_merge` is the only place the two lists meet. That split is the freedom-from-interference shape described in ISO 26262. This module claims no ASIL and does not open a vehicle bus.

Radar points are predicted before association. The Cartesian step is `range_rate * dt` projected onto the position unit vector. A position whose length is below `1e-9` has no direction, so prediction is skipped and the measured point is kept. Association is greedy gated nearest neighbor: each lidar track, in ingest order, takes the closest unused radar track whose Euclidean distance is at most `gate_m` (default 3 m).

A conflict is a pair whose range rates differ by more than `rate_bound` (default 2.5 m/s). The test is `2 * statistics.pstdev(pair) > rate_bound`, which is the absolute difference for a two-sample population. The same test is applied to every not-yet-counted pair that shares a cell of size `gate_m`. If both confidences are strictly below `confidence_floor` (default 0.35), the pair is withheld and is not published. Otherwise the position is the confidence-weighted mean of the lidar point and the predicted radar point, and the range rate is taken from the higher-confidence sensor. Ties keep the lidar rate.

A lidar track with no radar partner inside the gate is a dropout. It is counted in `dropouts` and is not fused. Unmatched radar tracks are not dropouts. They can still contribute to a same-cell conflict.

Non-finite fields, a confidence outside `[0, 1]`, or a coordinate outside the 200 m arena raise `EngineKernelException` before a frame is published. The summary frame is `struct` format `>IIII`: fused, withheld, dropouts, conflicts.

## 📊 Core Visual Walkthrough & Engine Pipeline Flow

![Terminal walkthrough](docs/assets/terminal-walkthrough.gif)

```
lidar records                         radar records
     |                                      |
     v                                      v
 finite / arena / confidence           same checks, separate coroutine
     |                                      |
     |                                      v
     |                               unit vector of (x, y, z)
     |                               + range_rate * dt   (skip if degenerate)
     \                                      /
      v                                    v
      greedy nearest neighbor, distance <= gate
      |
      +-- lidar with no radar partner --> dropouts += 1, not fused
      +-- both confidences < floor ----> withheld += 1, not published
      +-- otherwise -------------------> fused position, higher-confidence rate
      |
      +-- |range_rate difference| > bound ----> conflicts += 1
      +-- same cell, contradictory rates ----> conflicts += 1 if not counted
      v
 struct summary: fused, withheld, dropouts, conflicts
```

Insert the structural terminal walkthrough recording at docs/assets/terminal-walkthrough.gif before publishing the release notes.

## ⚡ Low-Level OS Mechanics & Network Physics

The two ingest coroutines each yield once, then parse only their own sensor tag. A fault raised while parsing radar fails the `gather` and does not publish a partial lidar fusion. The merge itself is synchronous and runs while the `asyncio.Lock` is held, so two scans cannot interleave their counters. No CAN frame, no automotive Ethernet socket, and no bus write exists in this process.

Separation is `math.dist` on the three coordinates. The cell index is `math.floor` of each axis divided by the gate, so two reports that share a cube can contradict even when the Euclidean gate did not pair them. Confidence weights are `c / (c_lidar + c_radar)`. Published confidence is `statistics.fmean` of the two confidences. The summary uses four big-endian uint32 counters. A published track can also be packed as tag plus five float64 fields (`>Bddddd`); that frame is numeric only.

Prediction adds `range_rate * dt * (position / |position|)` to the radar point. `dt` defaults to 0.05 s and may be zero, which leaves the radar point where it was measured. A predicted point that would leave the arena raises rather than being clamped onto the wall.

## ⚖️ Architecture Trade-offs & Pragmatic Decisions

A multi-hypothesis tracker would carry tracks across scans and would need a motion model and an identifier stable over time. This resolver is one scan. Association is greedy in lidar ingest order, not a global assignment, so a closer later lidar can lose a radar partner that an earlier lidar already took. The conflict rule is explicit: if the range rates disagree, count the pair, keep the position blend when confidence allows, and take the range rate from the sensor with more confidence.

Withholding is the output when both confidences are below the floor. Publishing a low-confidence blend would look like a track. A lidar cluster with no radar partner is reported as a dropout instead of being copied into the fused list, so a planner that reads only `published` does not see an unconfirmed cluster.

The arena bound rejects a coordinate that cannot exist in the configured volume instead of clamping it onto the wall. Clamping would create a false cluster on the boundary. The caller who meant that point can raise the arena; the caller who sent a sentinel cannot. The default arena is 200 m, and the boundary itself is inside.

## 🚀 Local Installation & Benchmarking

```bash
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"
python -m lidar_radar_volumetric_conflict_resolver
python -m lidar_radar_volumetric_conflict_resolver.harness
```

```python
import asyncio

from lidar_radar_volumetric_conflict_resolver import (
    LidarRadarVolumetricConflictResolver,
)


async def demo() -> None:
    resolver = LidarRadarVolumetricConflictResolver(arena_m=200.0, dt_s=0.05)
    await resolver.run(
        [
            {
                "sensor": "lidar",
                "x": 12.0,
                "y": -4.0,
                "z": 0.5,
                "range_rate": 6.0,
                "confidence": 0.82,
            },
            {
                "sensor": "radar",
                "x": 12.3,
                "y": -3.9,
                "z": 0.45,
                "range_rate": 6.2,
                "confidence": 0.70,
            },
        ]
    )


asyncio.run(demo())
```

The runtime is the Python 3.12 standard library. `pip install -r requirements.txt` succeeds with no third-party pins. The tracks above are synthetic numbers, not bus frames. `black==24.8.0` and `flake8==7.1.1` live in the `dev` extra.

## 🖥️ Terminal Diagnostic Output Preview

```
2026-10-02T02:58:28+0000 WARNING [lidar_radar_volumetric_conflict_resolver] range-rate conflict spread=9.0000
2026-10-02T02:58:28+0000 WARNING [lidar_radar_volumetric_conflict_resolver] lidar dropout x=30.00 y=-8.00 z=0.40
2026-10-02T02:58:28+0000 INFO [lidar_radar_volumetric_conflict_resolver] demo complete fused=1 withheld=0 dropouts=1 conflicts=1
```

`python -m lidar_radar_volumetric_conflict_resolver` exits 0. The orphan lidar point at `(30, -8, 0.4)` is the dropout. The paired reports with range rates 12 m/s and -6 m/s are the conflict. Population spread of that pair is 9.

## 📊 Empirical Benchmarking Performance Report

Measured by `python -m lidar_radar_volumetric_conflict_resolver.harness` with seed 26262, 5000 iterations after a warmup of 20, `time.perf_counter_ns` latency in microseconds, and `tracemalloc` peak. The scan is one lidar track and one nearby radar track with agreeing range rates, which fuses with zero conflicts, zero dropouts, and zero withholds.

```
status=ok seed=26262 iterations=5000 latency_us=296.838 memory_peak_bytes=176413 benchmark_avg_us=303.049 benchmark_p99_us=468.791
```

| Metric | Measured |
| --- | ---: |
| Status | ok |
| Seed | 26262 |
| Iterations | 5000 |
| latency_us | 296.838 |
| memory_peak_bytes | 176413 |
| benchmark_avg_us | 303.049 |
| benchmark_p99_us | 468.791 |

## 🛡️ Edge-Case Resilience & SOC2/Regulatory Compliance

A lidar track with no radar partner inside the gate increments `dropouts` and is omitted from `published`. A pair inside the gate whose range rates differ by more than the bound increments `conflicts`; if the two tracks merely share a cell and were not already counted, that pair increments `conflicts` as well. When both confidences are below the floor the pair increments `withheld` and is not published. Non-finite coordinates, a confidence outside the unit interval, and points outside the 200 m arena raise `EngineKernelException` before a frame is fused. The arena boundary (exactly 200 m) is accepted.

ISO 26262 freedom-from-interference is the structural reference for the separate ingest lists and the single merge step. This module claims no ASIL. It does not open a CAN or automotive Ethernet socket and it does not transmit a frame. SOC 2 processing integrity is the withhold rule: a low-confidence pair is counted and is not presented as a fused track. Inputs are synthetic numeric tracks the caller already has.
