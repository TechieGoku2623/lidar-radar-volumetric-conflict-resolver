"""Roundtrip, fusion, conflict/withhold, and dropout/arena coverage."""

from __future__ import annotations

import asyncio
import math
import unittest

from lidar_radar_volumetric_conflict_resolver import (
    EngineKernelException,
    LidarRadarVolumetricConflictResolver,
)
from lidar_radar_volumetric_conflict_resolver.wire import (
    pack_summary,
    pack_track,
    unpack_summary,
    unpack_track,
)


def _track(
    sensor: str,
    x: float,
    y: float,
    z: float,
    range_rate: float,
    confidence: float,
) -> dict[str, float | str]:
    return {
        "sensor": sensor,
        "x": x,
        "y": y,
        "z": z,
        "range_rate": range_rate,
        "confidence": confidence,
    }


class VolumetricEngineTest(unittest.TestCase):
    def test_summary_and_track_roundtrip(self) -> None:
        summary = pack_summary(2, 1, 3, 4)
        self.assertEqual(len(summary), 16)
        self.assertEqual(unpack_summary(summary), (2, 1, 3, 4))
        track = pack_track(1, 11.5, -0.25, 0.5, 4.0, 0.75)
        self.assertEqual(len(track), 41)
        self.assertEqual(unpack_track(track), (1, 11.5, -0.25, 0.5, 4.0, 0.75))

    def test_happy_path_predicts_and_skips_degenerate_radar(self) -> None:
        engine = LidarRadarVolumetricConflictResolver(
            arena_m=200.0, gate_m=3.0, rate_bound=2.5, dt_s=0.5
        )
        fused = asyncio.run(
            engine.run(
                [
                    _track("lidar", 12.0, 0.0, 0.0, 4.0, 0.75),
                    _track("radar", 8.0, 0.0, 0.0, 4.0, 0.25),
                ]
            )
        )
        self.assertEqual(fused["fused"], 1)
        self.assertEqual(fused["withheld"], 0)
        self.assertEqual(fused["dropouts"], 0)
        self.assertEqual(fused["conflicts"], 0)
        published = fused["published"][0]
        self.assertAlmostEqual(published["x"], 11.5, places=9)
        self.assertAlmostEqual(published["y"], 0.0, places=9)
        self.assertAlmostEqual(published["z"], 0.0, places=9)
        self.assertAlmostEqual(published["range_rate"], 4.0, places=9)
        self.assertAlmostEqual(published["confidence"], 0.5, places=9)
        self.assertEqual(unpack_summary(fused["frame"]), (1, 0, 0, 0))
        origin = asyncio.run(
            engine.run(
                [
                    _track("lidar", 0.4, 0.0, 0.0, 1.0, 0.9),
                    _track("radar", 0.0, 0.0, 0.0, 1.0, 0.9),
                ]
            )
        )
        self.assertEqual(origin["fused"], 1)
        self.assertEqual(origin["conflicts"], 0)
        self.assertAlmostEqual(origin["published"][0]["x"], 0.2, places=9)

    def test_conflict_withhold_and_same_cell(self) -> None:
        engine = LidarRadarVolumetricConflictResolver(
            arena_m=200.0, gate_m=3.0, rate_bound=2.5, dt_s=0.0
        )
        conflict = asyncio.run(
            engine.run(
                [
                    _track("lidar", 5.0, 5.0, 1.0, 12.0, 0.9),
                    _track("radar", 5.0, 5.0, 1.0, -8.0, 0.4),
                ]
            )
        )
        self.assertEqual(conflict["fused"], 1)
        self.assertEqual(conflict["withheld"], 0)
        self.assertEqual(conflict["dropouts"], 0)
        self.assertEqual(conflict["conflicts"], 1)
        self.assertAlmostEqual(conflict["published"][0]["range_rate"], 12.0, places=9)
        self.assertAlmostEqual(conflict["published"][0]["x"], 5.0, places=9)
        withheld = asyncio.run(
            engine.run(
                [
                    _track("lidar", 1.0, 1.0, 1.0, 1.0, 0.20),
                    _track("radar", 1.2, 1.0, 1.0, 1.2, 0.10),
                ]
            )
        )
        self.assertEqual(withheld["withheld"], 1)
        self.assertEqual(withheld["fused"], 0)
        self.assertEqual(withheld["conflicts"], 0)
        self.assertEqual(withheld["published"], [])
        cell = asyncio.run(
            engine.run(
                [
                    _track("lidar", 0.0, 0.0, 0.0, 0.0, 0.9),
                    _track("lidar", 2.5, 0.0, 0.0, 10.0, 0.9),
                    _track("radar", 0.2, 0.0, 0.0, 0.0, 0.8),
                ]
            )
        )
        self.assertEqual(cell["fused"], 1)
        self.assertEqual(cell["dropouts"], 1)
        self.assertEqual(cell["conflicts"], 2)

    def test_lidar_dropout_and_arena_guard(self) -> None:
        engine = LidarRadarVolumetricConflictResolver(
            arena_m=200.0, gate_m=3.0, rate_bound=2.5, dt_s=0.0
        )
        report = asyncio.run(
            engine.run(
                [
                    _track("lidar", 0.0, 0.0, 0.0, 2.0, 0.8),
                    _track("lidar", 40.0, 0.0, 0.0, 1.0, 0.8),
                    _track("radar", 0.3, 0.0, 0.0, 2.1, 0.7),
                ]
            )
        )
        self.assertEqual(report["fused"], 1)
        self.assertEqual(report["dropouts"], 1)
        self.assertEqual(report["withheld"], 0)
        self.assertEqual(report["conflicts"], 0)
        self.assertEqual(len(report["published"]), 1)
        with self.assertRaises(EngineKernelException):
            asyncio.run(engine.run([_track("lidar", math.nan, 0.0, 0.0, 0.0, 0.5)]))
        with self.assertRaises(EngineKernelException):
            asyncio.run(engine.run([_track("radar", 250.0, 0.0, 0.0, 0.0, 0.5)]))
        with self.assertRaises(EngineKernelException):
            asyncio.run(engine.run([_track("lidar", 0.0, 0.0, 0.0, 0.0, 1.5)]))
        edge = asyncio.run(
            engine.run(
                [
                    _track("lidar", 200.0, 0.0, 0.0, 0.0, 0.8),
                    _track("radar", 199.0, 0.0, 0.0, 0.0, 0.8),
                ]
            )
        )
        self.assertEqual(edge["fused"], 1)
        self.assertEqual(edge["dropouts"], 0)


if __name__ == "__main__":
    unittest.main()
