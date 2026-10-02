"""Fuse one contradictory pair and report one lidar dropout."""

from __future__ import annotations

import asyncio
import logging
import sys

from .engine import LidarRadarVolumetricConflictResolver
from .exceptions import EngineKernelException


async def _demo() -> dict[str, object]:
    engine = LidarRadarVolumetricConflictResolver(
        arena_m=200.0, gate_m=3.0, rate_bound=2.5, dt_s=0.05
    )
    return await engine.run(
        [
            {
                "sensor": "lidar",
                "x": 5.0,
                "y": 5.0,
                "z": 1.0,
                "range_rate": 12.0,
                "confidence": 0.90,
            },
            {
                "sensor": "lidar",
                "x": 30.0,
                "y": -8.0,
                "z": 0.4,
                "range_rate": 1.5,
                "confidence": 0.80,
            },
            {
                "sensor": "radar",
                "x": 5.2,
                "y": 5.1,
                "z": 1.05,
                "range_rate": -6.0,
                "confidence": 0.60,
            },
        ]
    )


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S%z",
    )
    logger = logging.getLogger("lidar_radar_volumetric_conflict_resolver")
    try:
        report = asyncio.run(_demo())
    except EngineKernelException:
        logger.exception("demo tracks rejected")
        return 1
    if (
        int(report["fused"]) != 1
        or int(report["dropouts"]) != 1
        or int(report["conflicts"]) < 1
        or int(report["withheld"]) != 0
    ):
        logger.error(
            "demo merge mismatched fused=%s withheld=%s dropouts=%s conflicts=%s",
            report["fused"],
            report["withheld"],
            report["dropouts"],
            report["conflicts"],
        )
        return 1
    logger.info(
        "demo complete fused=%s withheld=%s dropouts=%s conflicts=%s",
        report["fused"],
        report["withheld"],
        report["dropouts"],
        report["conflicts"],
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
