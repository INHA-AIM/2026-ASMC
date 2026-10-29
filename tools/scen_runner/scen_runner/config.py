"""설정 파일과 저장소 경로."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import yaml

TOOL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = TOOL_ROOT.parents[1]
DEFAULT_CONFIG = TOOL_ROOT / "config" / "static_obstacle.yaml"
DEFAULT_CATALOG = TOOL_ROOT / "config" / "obstacle_catalog.yaml"
LOCAL_CONFIG = TOOL_ROOT / "config" / "local.yaml"
LINK_MAP = "src/asmc_planner/config/behavior/link_set.json"
_LEGACY_PLANNER_ASSETS = "src/behavior_planner/src/config/"
_PLANNER_ASSETS = "src/asmc_planner/config/behavior/"


def repo_path(rel: str) -> Path:
    path = Path(rel)
    if path.is_absolute():
        try:
            relative = path.relative_to(REPO_ROOT).as_posix()
        except ValueError:
            return path
    else:
        relative = path.as_posix()
    # Match main's planner asset ownership migration. Saved replay records
    # retain their original source names; geometry/hash checks use the live file.
    if relative.startswith(_LEGACY_PLANNER_ASSETS):
        relative = _PLANNER_ASSETS + relative[len(_LEGACY_PLANNER_ASSETS):]
    return REPO_ROOT / relative


def load_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_config(path: Path | None = None) -> dict:
    config = load_yaml(Path(path) if path else DEFAULT_CONFIG)
    # Local settings only choose the endpoint; never override scene physics,
    # object generation, random seeds, map assets, or replay definitions.
    config.setdefault("morai", {}).update(load_local_config().get("morai", {}))
    return config


def load_local_config() -> dict:
    """PC-specific values are optional for offline tools and absent in Git."""
    if not LOCAL_CONFIG.exists():
        return {}
    try:
        with LOCAL_CONFIG.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise ValueError(f"개인 설정 YAML 오류: {LOCAL_CONFIG} · {error}") from error
    if data is None:
        data = {}
    fields = {"morai": {"host", "port"}, "dll": {"managed_path"}, "bridge": {"script"}}
    if not isinstance(data, dict) or data.keys() - fields.keys():
        raise ValueError(f"{LOCAL_CONFIG}: morai, dll, bridge 항목만 설정할 수 있습니다.")
    for section, values in data.items():
        if not isinstance(values, dict) or values.keys() - fields[section]:
            allowed = ", ".join(sorted(fields[section]))
            raise ValueError(f"{LOCAL_CONFIG}: {section} 항목은 {allowed} 키를 가진 매핑이어야 합니다.")
    return data


def require_morai_endpoint(settings: dict) -> tuple[str, int]:
    """Validate before GUI/process startup or any connection attempt."""
    host, port = settings.get("host"), settings.get("port")
    if not isinstance(host, str) or not host.strip():
        raise ValueError(
            f"MORAI 접속 주소가 없습니다. {LOCAL_CONFIG.parent / 'local.example.yaml'}을 "
            f"{LOCAL_CONFIG}로 복사하고 morai.host에 이 PC의 MORAI 주소를 입력하세요.")
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError(f"{LOCAL_CONFIG}: morai.port는 1~65535 사이의 정수여야 합니다.")
    return host.strip(), port


def section_config(base: dict, section: str) -> dict:
    """A213 검증 범위는 최신 HD맵의 링크 station에서 읽는다."""
    cfg = copy.deepcopy(base)
    if section == "intersection":
        return cfg
    if section != "A2256W000213":
        raise ValueError(f"지원하지 않는 검증 구간: {section}")
    doc = json.loads(repo_path(cfg["map"]["drivable_map"]).read_text(encoding="utf-8"))
    lane = next(l for l in doc["lanes"] if l.get("link_id") == section)
    s0, s1 = float(lane["route_station_start_m"]), float(lane["route_station_end_m"])
    s0 = (s0 + s1) / 2.0
    cfg["zone"].update(link_id=section, evaluation_s_range=[s0, s1],
                       view_s_range=[s0 - 5.0, s1 + 5.0])
    for case in cfg["speed_cases"]:
        case["start_station_m"] = s0
    cfg["search"]["goal_after_checkpoint_m"] = 0.0
    if base.get("single_scene_runtime"):
        cfg["search"]["goal_after_checkpoint_m"] = -float(base["single_scene_runtime"]["handoff_before_stopline_m"])
    # 좁은 차로에서는 0.25 m 후보 사이에 가능한 회피 폭이 있을 수 있다.
    cfg["search"]["lateral_offsets_m"]["fine"]["step"] = 0.05
    cfg["clearance"]["checkpoint_m"] = 0.0  # 링크 끝까지 중심 경로로 복귀
    return cfg


def load_catalog(path: Path | None = None) -> dict:
    data = load_yaml(Path(path) if path else DEFAULT_CATALOG)
    data.setdefault("models", {})
    data.setdefault("exclude", [])
    return data


def save_catalog(catalog: dict, path: Path | None = None) -> None:
    with open(Path(path) if path else DEFAULT_CATALOG, "w", encoding="utf-8") as f:
        yaml.safe_dump(catalog, f, allow_unicode=True, sort_keys=False)


def merged(base: dict, override: dict | None) -> dict:
    """중첩 dict 를 덮어쓴 사본. GUI 에서 바꾼 판정 값을 반영할 때 쓴다."""
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merged(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out
