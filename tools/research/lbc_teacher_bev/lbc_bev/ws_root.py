"""Resolve ASMC code and data roots on the host or in Docker."""
from __future__ import annotations

import os
from pathlib import Path


def asmc_ws_root() -> Path:
    for key in ("ASMC", "ASMC_WS_ROOT"):
        val = os.environ.get(key)
        if val:
            root = Path(val).expanduser().resolve()
            if (root / "src").is_dir():
                return root
    docker_ws = Path("/root/ws")
    if (docker_ws / "src").is_dir():
        return docker_ws

    for parent in Path(__file__).resolve().parents:
        if (parent / "src").is_dir() and (parent / "tools").is_dir():
            return parent
    raise RuntimeError("could not locate the ASMC repository root")


def asmc_data_root() -> Path:
    """Return the external artifact root shared by repository tooling."""

    configured = os.environ.get("ASMC_DATA")
    if configured:
        return Path(configured).expanduser().resolve()
    return asmc_ws_root().parent / "data"


def lbc_data_root() -> Path:
    """Return the LBC teacher-BEV artifact root (never inside the source tree)."""

    return asmc_data_root() / "lbc_teacher_bev"
