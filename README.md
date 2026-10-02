# LiDAR Radar Volumetric Conflict Resolver

A high-throughput, low-latency asynchronous engine engineered to resolve contradictory lidar clusters and radar tracks that occupy one spatial cell, publishing a confidence-weighted fusion or withholding the track when both confidences are too low.

## 🏗️ Systems Architecture & Event Topology

`VolumetricConflictResolver` parses each sensor on its own path and meets them in one merge. `resolve` is the coroutine. A lidar track and a radar track are five-tuples: x, y, z, range rate, confidence. `pack_frame` writes the published numbers as IEEE-754 binary32 plus a state code. The topic name is `vehicle.perception.fused_track`.

Association uses a gate in meters (`gate_m`, default 3). Tracks in the same cell with a range-rate disagreement above `rate_bound` (default 2.5 m/s) are a conflict. The position is blended by confidence. The range rate is blended when the sensors agree and taken from the higher-confidence sensor when they do not. When both confidences are at or below `low_confidence` (default 0.35), the track is withheld: coordinates are null, `publish` is false, and the frame carries zeros.

An `asyncio.Lock` covers the ledger and the spool. `configure_logging` calls `logging.basicConfig` with timestamps. A non-finite coordinate, a point outside the arena, or a range rate past `max_range_rate` raises `EngineKernelException`. The split between the two ingress paths follows the freedom-from-interference idea in ISO 26262. This module does not claim an ASIL and does not talk to a vehicle bus.

## 📊 Core Visual Walkthrough & Engine Pipeline Flow

```
lidar tuples                         radar tuples
     |                                    |
     v                                    v
 finite / arena / rate-rate gate     same gate, separate queue
     \                                  /
      v                                v
      cell = floor(coord / gate_m)
      |
      +-- lidar with no radar partner --> dropout, publish false
      +-- both confidences low --------> withheld, null coordinates
      +-- same cell, |rate delta| high -> resolved, rate from higher confidence
      +-- agreement --------------------> fused, blended rate
      v
 pack_frame (binary32) on vehicle.perception.fused_track
```

Insert the structural terminal walkthrough recording at docs/assets/terminal-walkthrough.gif before publishing the release notes.

## ⚡ Low-Level OS Mechanics & Network Physics

Separation is `math.dist` on the three coordinates. The cell index is `math.floor` of each axis divided by the gate, so two reports 20 cm apart share a cell and two reports on opposite sides of a boundary do not. Confidence weights are normalized by the sum of the two confidences. The published frame uses `struct` format `>fffffB`. Binary32 cannot hold every binary64 blend; the dict keeps the full-precision blend, and the frame is the value a downstream consumer will decode. The harness compares those two with a tolerance that covers one binary32 rounding.

`statistics.mean` is the pair confidence. `statistics.pstdev` is the range-rate spread. The ledger is a bounded `deque`. No CAN frame, no Ethernet AVB socket, and no bus write exists in this process. The queues are the interference boundary: a fault raised while parsing radar does not mutate a lidar slot.

## ⚖️ Architecture Trade-offs & Pragmatic Decisions

A multi-hypothesis tracker would carry tracks across scans and would need a motion model and an identifier stable over time. This resolver is one scan. Association is geometric, and the conflict rule is explicit: if the range rates disagree inside one cell, keep the position blend, take the range rate from the sensor with more confidence, and mark `conflict`. If neither sensor is confident, withhold. Withholding is the safe output. Publishing a low-confidence blend would look like a track to a planner.

The arena bound rejects a coordinate that cannot exist in the configured volume instead of clamping it onto the wall. Clamping would create a false cluster on the boundary. The caller who meant that point can raise the arena; the caller who sent a sentinel cannot.

## 🚀 Local Installation & Benchmarking

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python src/main.py
python src/test_harness.py
```

```python
import asyncio

from src.main import VolumetricConflictResolver


async def demo() -> None:
    resolver = VolumetricConflictResolver(arena_m=200.0)
    lidar = [(12.0, -4.0, 0.5, 6.0, 0.82)]
    radar = [(12.3, -3.9, 0.45, 6.4, 0.70)]
    try:
        await resolver.resolve(lidar, radar)
    finally:
        await resolver.close()


asyncio.run(demo())
```

The runtime is the Python 3.12 standard library. `pip install -r requirements.txt` succeeds with no third-party packages.

## 🖥️ Terminal Diagnostic Output Preview

```
WARNING [volumetric.kernel] range-rate conflict resolved by confidence weight delta=18.00
WARNING [volumetric.kernel] sensor dropout source=lidar x=30.00 y=-8.00 z=0.40
INFO [volumetric.kernel] demo complete tracks=2 conflicts=1 dropouts=1
```

`python src/main.py` exits 0. The orphan lidar point is the dropout. The paired cell with opposing range rates is the conflict.

## 📊 Empirical Benchmarking Performance Report

Measured by `python src/test_harness.py` with seed 26262, 5000 iterations, `time.perf_counter_ns` latency in microseconds, and `tracemalloc` peak.

| Metric | Measured |
| --- | ---: |
| Status | PASS |
| Iterations | 5000 |
| Average latency | 275.998 µs |
| Empirical P99 | 525.056 µs |
| tracemalloc peak | 586582 bytes |

## 🛡️ Edge-Case Resilience & SOC2/Regulatory Compliance

A lidar cluster with no radar partner is a dropout: the lidar coordinates are reported, `publish` is false, and `dropout_count` increments. Contradictory range rates inside one cell are marked `conflict`, the position is confidence-weighted, and the range rate comes from the higher-confidence sensor. When both confidences are low the track is withheld and the published coordinates are null. Non-finite coordinates and points outside the arena raise `EngineKernelException` before a frame is fused.

ISO 26262 freedom-from-interference is the structural reference for the separate ingress queues. This module claims no ASIL. It does not open a CAN or automotive Ethernet socket and it does not transmit a frame. SOC 2 processing integrity is the withhold rule: a low-confidence pair is counted and is not presented as a fused track.
