"""Read-only adapters for the MGeo JSON variants in this workspace."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Iterable

import numpy as np

from .geometry import resample_polyline


from .asset_paths import current_source_path, recorded_source_path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resolve_source(repo_root: Path, raw: str) -> Path:
    expanded = Path(os.path.expandvars(os.path.expanduser(current_source_path(raw))))
    return expanded if expanded.is_absolute() else (repo_root / expanded).resolve()


def first_existing_source(repo_root: Path, candidates: Iterable[str]) -> tuple[Path | None, int | None]:
    for index, raw in enumerate(candidates):
        path = resolve_source(repo_root, raw)
        if path.is_file():
            return path, index
    return None, None


def feature_cloud(
    features: list[dict],
    step_m: float,
    translation_xyz: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    all_points: list[np.ndarray] = []
    all_tangents: list[np.ndarray] = []
    owners: list[str] = []
    translation = None if translation_xyz is None else np.asarray(translation_xyz, dtype=np.float64)
    for feature in features:
        points, tangents = resample_polyline(feature.get("points", ()), step_m)
        if translation is not None and len(points):
            points = points + translation
        all_points.extend(points)
        all_tangents.extend(tangents)
        owners.extend([str(feature.get("idx", ""))] * len(points))
    if not all_points:
        return (
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 2), dtype=np.float64),
            np.zeros(0, dtype=object),
        )
    return np.asarray(all_points), np.asarray(all_tangents), np.asarray(owners, dtype=object)


def source_record(repo_root: Path, configured_path: str, path: Path) -> dict:
    try:
        display = str(path.relative_to(repo_root))
    except ValueError:
        # Keep reports portable: paths in the workspace data sibling are named,
        # but a developer-specific absolute home directory is never serialized.
        display = f"<external-data>/{path.name}"
    return {
        "configured_path": recorded_source_path(configured_path),
        "resolved_display": recorded_source_path(display),
        "sha256": sha256_file(path),
        "bytes": path.stat().st_size,
    }
