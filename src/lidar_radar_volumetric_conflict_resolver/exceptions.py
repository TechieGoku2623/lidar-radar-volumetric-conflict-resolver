"""Kernel faults for the volumetric resolver.

A non-finite coordinate or a point outside the arena raises
``EngineKernelException`` before the merge publishes a track. Inputs are
synthetic numeric tracks. This package does not open a vehicle bus and
does not claim an ASIL.
"""

from __future__ import annotations


class EngineKernelException(Exception):
    """A track that cannot enter the arena without corrupting the merge."""
