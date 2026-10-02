"""LiDAR and radar volumetric conflict resolver.

Tracks are synthetic numeric reports the caller already has. The package
keeps lidar and radar ingress apart until one merge step. It does not open
a CAN, Ethernet, or other vehicle bus, and it claims no ASIL.
"""

from __future__ import annotations

from .engine import LidarRadarVolumetricConflictResolver
from .exceptions import EngineKernelException

__all__ = [
    EngineKernelException.__name__,
    LidarRadarVolumetricConflictResolver.__name__,
]
__version__ = "1.0.0"
