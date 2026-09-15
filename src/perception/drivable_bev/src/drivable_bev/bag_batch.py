"""Selection-manifest validation shared by shadow rosbag batch tooling."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import itertools
from pathlib import Path
from typing import Iterable, Mapping, Optional, Tuple

import yaml


@dataclass(frozen=True)
class SelectedBagRun:
    run_id: str
    raw_path: Path
    scenario: str
    weather: str
    sim_hour: int
    recorded_controller: str
    effective_controller: str
    duration_s: float


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_bag_selection(
    selection_path,
    rosbag_root,
    scenarios: Optional[Iterable[str]] = None,
) -> Tuple[dict, Tuple[SelectedBagRun, ...]]:
    selection_path = Path(selection_path).expanduser().resolve()
    rosbag_root = Path(rosbag_root).expanduser().resolve()
    with selection_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if document.get("schema_version") != "asmc-rosbag-selection-1.0.0":
        raise ValueError("unsupported rosbag selection schema")
    if document.get("missing_combinations"):
        raise ValueError("selection still contains missing combinations")
    requested = set(str(value) for value in (scenarios or []))
    seen_ids = set()
    seen_conditions = set()
    result = []
    for row in document.get("selected_runs") or []:
        scenario = str(row["scenario"])
        if requested and scenario not in requested:
            continue
        run_id = str(row["run_id"])
        if run_id in seen_ids:
            raise ValueError("duplicate selected run_id: {}".format(run_id))
        condition = (scenario, str(row["weather"]), int(row["sim_hour"]))
        if condition in seen_conditions:
            raise ValueError("duplicate selected condition: {}".format(condition))
        raw_path = rosbag_root / "raw" / run_id
        metadata_path = raw_path / "metadata.yaml"
        bag_files = sorted((raw_path / "bags").glob("*.bag"))
        if not metadata_path.is_file() or not bag_files:
            raise ValueError("selected raw run is incomplete: {}".format(raw_path))
        with metadata_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream) or {}
        if metadata.get("status") != "complete":
            raise ValueError("selected raw run is not complete: {}".format(run_id))
        if str(metadata.get("scenario")) != scenario:
            raise ValueError("scenario metadata mismatch: {}".format(run_id))
        environment = metadata.get("environment_initial") or {}
        if str(environment.get("weather")) != condition[1]:
            raise ValueError("weather metadata mismatch: {}".format(run_id))
        if int(environment.get("sim_hour")) != condition[2]:
            raise ValueError("simulation hour metadata mismatch: {}".format(run_id))
        seen_ids.add(run_id)
        seen_conditions.add(condition)
        result.append(
            SelectedBagRun(
                run_id=run_id,
                raw_path=raw_path,
                scenario=scenario,
                weather=condition[1],
                sim_hour=condition[2],
                recorded_controller=str(row.get("recorded_controller", "unknown")),
                effective_controller=str(row.get("effective_controller", "unknown")),
                duration_s=float(row.get("duration_s", 0.0)),
            )
        )
    if not result:
        raise ValueError("selection contains no matching runs")
    scope = document.get("scope") or {}
    allowed_weather = [str(value) for value in scope.get("allowed_weather") or []]
    allowed_hours = [int(value) for value in scope.get("allowed_sim_hours") or []]
    scoped_scenarios = [str(value) for value in scope.get("scenarios") or []]
    matrix_scenarios = sorted(requested) if requested else scoped_scenarios
    if matrix_scenarios and allowed_weather and allowed_hours:
        expected = set(
            itertools.product(matrix_scenarios, allowed_weather, allowed_hours)
        )
        actual = {
            (value.scenario, value.weather, value.sim_hour) for value in result
        }
        if actual != expected:
            raise ValueError(
                "selection matrix differs from scope: missing={} unexpected={}".format(
                    sorted(expected - actual), sorted(actual - expected)
                )
            )
    return document, tuple(
        sorted(result, key=lambda value: (value.scenario, value.weather, value.sim_hour))
    )


def diagnostic_values(metadata: dict, status_name: str) -> dict:
    diagnostics = metadata.get("diagnostics_final") or {}
    for by_name in diagnostics.values():
        if status_name in (by_name or {}):
            return dict((by_name[status_name] or {}).get("values") or {})
    return {}


def topic_message_count(metadata: Mapping, topic: str) -> int:
    """Return a recorder topic count without trusting YAML scalar types."""
    statistics = (metadata.get("topic_statistics") or {}).get(topic) or {}
    try:
        return int(statistics.get("messages", 0))
    except (TypeError, ValueError):
        return 0


def diagnostic_int(values: Mapping, key: str, default: int = 0) -> int:
    """Parse a DiagnosticArray KeyValue integer captured in result metadata."""
    try:
        return int(float(values.get(key, default)))
    except (TypeError, ValueError):
        return int(default)


def result_quality_gates(
    metadata: Mapping, require_boundary_family: bool = False
) -> dict:
    """Build technical, non-accuracy gates for a shadow lane result bag.

    Boundary messages must be present, but an individual message may contain
    zero candidates. Candidate completeness is an evaluation outcome and must
    not be confused with a pipeline or recording failure.
    """
    lane = diagnostic_values(dict(metadata), "drivable_bev/lane_vectorizer")
    typed_boundary_count = topic_message_count(
        metadata, "/perception/lane/boundary_observations"
    )
    family_boundary_count = topic_message_count(
        metadata, "/perception/lane/boundary_family"
    )
    legacy_boundary_count = topic_message_count(
        metadata, "/perception/lane/boundaries"
    )
    result = {
        "result_complete": metadata.get("status") == "complete",
        "camera_diagnostics_recorded": topic_message_count(
            metadata, "/perception/camera/diagnostics"
        ) > 0,
        "bev_diagnostics_recorded": topic_message_count(
            metadata, "/perception/bev/diagnostics"
        ) > 0,
        "lane_diagnostics_recorded": topic_message_count(
            metadata, "/perception/lane/diagnostics"
        ) > 0,
        "lane_boundaries_recorded": (
            typed_boundary_count > 0
            or family_boundary_count > 0
            or legacy_boundary_count > 0
        ),
        "lane_outputs_positive": diagnostic_int(lane, "outputs") > 0,
        "lane_errors_zero": diagnostic_int(lane, "errors") == 0,
    }
    if require_boundary_family:
        result["boundary_family_recorded"] = family_boundary_count > 0
    return result
