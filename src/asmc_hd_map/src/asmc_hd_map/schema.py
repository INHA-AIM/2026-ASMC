"""Canonical HD-map document helpers.

MGeo fields are preserved under ``attributes.mgeo`` when they are useful, but
MGeo IDs never become canonical foreign keys without a reviewed correspondence.
"""

from __future__ import annotations

from typing import Iterable


SCHEMA_VERSION = "asmc.hd_map.canonical/v1"
ALLOWED_STATUS = {"candidate", "reviewed", "operational", "rejected"}
ALLOWED_SOURCE_KIND = {
    "current_mgeo",
    "current_lidar_semantic",
    "current_gps_direct",
    "legacy_katri",
    "link_width_proxy",
    "manual_review",
}


def make_candidate_feature(
    *,
    feature_type: str,
    canonical_id: str,
    points: Iterable[Iterable[float]],
    attributes: dict,
    source_id: str,
    source_sha256: str,
    target_link_ids: list[str],
    validation: dict,
) -> dict:
    return {
        "id": canonical_id,
        "feature_type": feature_type,
        "frame_id": "map",
        "geometry": {
            "type": "LineString",
            "coordinates": [[float(v) for v in row] for row in points],
        },
        "attributes": dict(attributes),
        "topology": {"current_link_ids": list(target_link_ids)},
        "provenance": {
            "source_kind": "legacy_katri",
            "source_id": source_id,
            "source_sha256": source_sha256,
            "transform": "declared_origin_translation",
        },
        "validation": dict(validation),
        "status": "candidate",
    }


def validate_canonical_document(document: dict) -> None:
    if document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported canonical HD-map schema_version")
    frame = document.get("frame") or {}
    if frame.get("frame_id") != "map":
        raise ValueError("canonical frame_id must be map")
    origin = frame.get("local_origin_in_global")
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError("canonical frame requires a three-element origin")
    for feature in document.get("features", []):
        if not str(feature.get("id", "")).startswith(str(document.get("map_id")) + ":"):
            raise ValueError("feature id must belong to the canonical map namespace")
        if feature.get("status") not in ALLOWED_STATUS:
            raise ValueError("invalid feature status")
        provenance = feature.get("provenance") or {}
        if provenance.get("source_kind") not in ALLOWED_SOURCE_KIND:
            raise ValueError("unknown provenance.source_kind")
        if not provenance.get("source_sha256"):
            raise ValueError("source hash is mandatory")
        geometry = feature.get("geometry") or {}
        if geometry.get("type") not in {"Point", "LineString", "Polygon"}:
            raise ValueError("unsupported geometry type")
        if not geometry.get("coordinates"):
            raise ValueError("feature geometry is empty")
