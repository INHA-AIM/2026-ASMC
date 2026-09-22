#!/usr/bin/env python3
"""Validate the P3-B bridge baseline, transport profiles, and MORAI presets."""

from __future__ import annotations

import json
import hashlib
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools/bridge"))
from render_morai_network_profiles import (  # noqa: E402
    ProfileError,
    check_all,
    load_contract,
)


class ContractError(RuntimeError):
    """Raised when the committed P3-B contract and implementation diverge."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractError(message)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _launch_arg(path: Path, name: str) -> str:
    root = ET.parse(path).getroot()
    for arg in root.findall("arg"):
        if arg.get("name") == name:
            return str(arg.get("default"))
    raise ContractError(f"{path.relative_to(REPO_ROOT)} lacks launch arg {name}")


def _network_entries(document: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
    for ego in document.get("egoNetworkData", []):
        yield from ego.get("listEgoNetworkInfo", [])


def _ctrl_entry(document: Dict[str, Any]) -> Dict[str, Any]:
    matches = [entry for entry in _network_entries(document) if entry.get("Division_TYPE") == 3]
    _require(len(matches) == 1, "network JSON must contain exactly one Ego Ctrl Cmd entry")
    return matches[0]


def _validate_generated_ips(
    document: Dict[str, Any], release: str, transport: str, contract: Dict[str, Any]
) -> None:
    profile = contract["transport_profiles"][transport]
    field = transport == "lan2lan"
    allowed_pubsub = {
        contract["channels"]["competition_vehicle_status"]["morai_pubsub_type"],
        contract["channels"]["collision_data"]["morai_pubsub_type"],
        contract["channels"]["ego_vehicle_status"]["morai_pubsub_type_by_release"][release],
    }
    for entry in _network_entries(document):
        config_udp = entry.get("config", {}).get("udpConfig", {})
        ctrl_udp = entry.get("EgoCtrlConfig", {}).get("udpConfig", {})
        expected_lan = field and entry.get("PUBSUB_TYPE") in allowed_pubsub
        expected_ctrl_lan = field and entry.get("Division_TYPE") == 3
        expected_config_ips = (
            (profile["morai_ip"], profile["autonav_ip"])
            if expected_lan or not field
            else ("127.0.0.1", "127.0.0.1")
        )
        expected_ctrl_ips = (
            (profile["morai_ip"], profile["autonav_ip"])
            if expected_ctrl_lan or not field
            else ("127.0.0.1", "127.0.0.1")
        )
        _require(
            (config_udp.get("hostIP"), config_udp.get("destinationIP"))
            == expected_config_ips,
            f"{transport} contains a non-allowlisted network channel",
        )
        _require(
            (ctrl_udp.get("hostIP"), ctrl_udp.get("destinationIP"))
            == expected_ctrl_ips,
            f"{transport} contains a non-control EgoCtrlConfig LAN endpoint",
        )


def _sensor_config(item: Dict[str, Any]) -> Dict[str, Any]:
    for key in ("cc", "lc", "gc", "ic", "gtc"):
        value = item.get(key)
        if isinstance(value, dict):
            return value
    raise ContractError("sensor entry lacks its typed configuration object")


def _sensor_summary(path: Path) -> Dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    lidar = _sensor_config(data["Lidar3DList"][0])
    return {
        "cameras": len(data.get("cameraList", [])),
        "lidars": len(data.get("Lidar3DList", [])),
        "camera4_pitch_deg": float(data["cameraList"][3]["rot"]["pitch"]),
        "lidar_period_s": float(lidar["sensorPeriod"]),
        "lidar_frequency_index": int(lidar["frequencyIndex"]),
        "lidar_ports": [
            int(lidar["udpConfig"]["hostPort"]),
            int(lidar["udpConfig"]["destinationPort"]),
        ],
    }


def validate(contract_path: Path) -> Dict[str, Any]:
    contract = load_contract(contract_path)
    _require(contract["schema_version"] == 1, "unsupported bridge contract schema")
    runtime = contract["runtime_protocol"]
    _require(runtime["id"] == "morai_udp", "runtime protocol must be morai_udp")
    _require(not runtime["release_behavior_split"], "simulator releases must share one bridge")
    _require(
        set(runtime["supported_simulator_releases"]) == {"26r1", "molitcomp03"},
        "both simulator releases must be covered",
    )
    _require(
        contract["defaults"] == {"transport_profile": "local", "sensor_set": "v2"},
        "runtime defaults must remain release-independent local + v2",
    )
    implementation = contract["implementation_baseline"]
    _require(
        implementation["packages"]
        == ["integration_launch", "morai_udp_bridge", "morai_sensor_bridge", "vehicle_state"],
        "bridge package baseline changed",
    )
    for name, spec in implementation["artifacts"].items():
        _require(
            _sha256(REPO_ROOT / spec["path"]) == spec["sha256"],
            f"{name} implementation artifact SHA changed",
        )
    _require(
        implementation["published_topics"]["/vehicle/state"] == "asmc_msgs/VehicleState",
        "normalized vehicle state topic changed",
    )
    _require(
        implementation["published_topics"]["/Competition_topic"]
        == "morai_msgs/EgoVehicleStatus",
        "Competition Vehicle Status topic contract changed",
    )
    _require(
        implementation["subscribed_topics"]["/ctrl_cmd"] == "morai_msgs/CtrlCmd",
        "Ctrl topic contract changed",
    )

    ctrl = contract["channels"]["ego_ctrl_cmd"]
    _require(
        (ctrl["host_port"], ctrl["destination_port"], ctrl["ctrl_mode"])
        == (9093, 9094, 2),
        "Ego Ctrl Cmd must be 9093/9094 mode 2",
    )
    ego_status = contract["channels"]["ego_vehicle_status"]
    _require(
        (ego_status["host_port"], ego_status["destination_port"], ego_status["runtime_bridge_bind_port"])
        == (9088, 9099, 9099),
        "Ego Vehicle Status must be normalized to 9088/9099",
    )
    _require(
        ego_status["morai_pubsub_type_by_release"] == {"26r1": 770, "molitcomp03": 295},
        "Ego Vehicle Status PUBSUB mapping changed",
    )
    competition_status = contract["channels"]["competition_vehicle_status"]
    _require(
        (
            competition_status["host_port"],
            competition_status["destination_port"],
            competition_status["runtime_bridge_bind_port"],
        )
        == (7603, 7502, 7502),
        "Competition Vehicle Status must use 7603/7502",
    )
    _require(
        competition_status["state"]
        == "static_decoder_active_live_capture_pending",
        "Competition decoder acceptance state changed",
    )
    packet_contract = competition_status["packet_contract"]
    _require(
        (
            packet_contract["header_ascii"],
            packet_contract["payload_bytes"],
            packet_contract["datagram_bytes"],
        )
        == ("#MoraiInfo$", 152, 181),
        "Competition packet contract changed",
    )
    _require(
        _sha256(REPO_ROOT / packet_contract["fixture_path"])
        == packet_contract["fixture_sha256"],
        "Competition packet fixture SHA changed",
    )
    _require(
        contract["transport_profiles"]["local"]
        == {
            "morai_ip": "127.0.0.1",
            "autonav_ip": "127.0.0.1",
            "deployment_state": "active_default",
        },
        "local IP contract changed",
    )
    field = contract["transport_profiles"]["lan2lan"]
    _require(
        (field["morai_ip"], field["autonav_ip"])
        == ("192.168.0.1", "192.168.0.10"),
        "lan2lan IP contract changed",
    )

    for release, spec in contract["network_templates"].items():
        template_path = REPO_ROOT / spec["template"]
        _require(
            _sha256(template_path) == spec["template_sha256"],
            f"{release} network template SHA changed",
        )
        document = json.loads(template_path.read_text(encoding="utf-8"))
        ctrl_config = _ctrl_entry(document)["EgoCtrlConfig"]
        udp = ctrl_config["udpConfig"]
        _require(ctrl_config["egoCtrlMode"] == 2, f"{release} template is not mode 2")
        _require(
            (udp["hostPort"], udp["destinationPort"]) == (9093, 9094),
            f"{release} template does not use 9093/9094",
        )
        for transport, output_rel in spec["generated"].items():
            output_path = REPO_ROOT / output_rel
            _require(
                _sha256(output_path) == spec["generated_sha256"][transport],
                f"{release}/{transport} generated network SHA changed",
            )
            generated_document = json.loads(output_path.read_text(encoding="utf-8"))
            _validate_generated_ips(generated_document, release, transport, contract)
            ego_entries = [
                entry
                for entry in _network_entries(generated_document)
                if entry.get("PUBSUB_TYPE")
                == ego_status["morai_pubsub_type_by_release"][release]
            ]
            _require(len(ego_entries) == 1, f"{release} must contain one Ego Vehicle Status")
            ego_udp = ego_entries[0]["config"]["udpConfig"]
            _require(
                (ego_udp["hostPort"], ego_udp["destinationPort"]) == (9088, 9099),
                f"{release}/{transport} Ego Vehicle Status ports changed",
            )
            competition_entries = [
                entry
                for entry in _network_entries(generated_document)
                if entry.get("PUBSUB_TYPE") == competition_status["morai_pubsub_type"]
            ]
            _require(
                len(competition_entries) == 1,
                f"{release} must contain one Competition Vehicle Status",
            )
            competition_udp = competition_entries[0]["config"]["udpConfig"]
            _require(
                (competition_udp["hostPort"], competition_udp["destinationPort"])
                == (7603, 7502),
                f"{release}/{transport} Competition Vehicle Status ports changed",
            )
    try:
        check_all(contract)
    except ProfileError as exc:
        raise ContractError(str(exc)) from exc

    integration = REPO_ROOT / "src/integration_launch/launch/morai_udp_bridges.launch"
    udp_launch = REPO_ROOT / "src/morai_udp_bridge/launch/morai_udp_bridge.launch"
    sensor_launch = REPO_ROOT / "src/morai_sensor_bridge/launch/morai_sensor_bridge.launch"
    _require(_launch_arg(integration, "runtime_profile") == "morai_udp", "integration runtime profile split")
    _require(_launch_arg(udp_launch, "runtime_profile") == "morai_udp", "UDP runtime profile split")
    _require(_launch_arg(sensor_launch, "runtime_profile") == "morai_udp", "sensor runtime profile split")
    _require(_launch_arg(integration, "transport_profile") == "local", "integration transport default changed")
    _require(_launch_arg(udp_launch, "transport_profile") == "local", "UDP transport default changed")
    _require(_launch_arg(sensor_launch, "transport_profile") == "local", "sensor transport default changed")
    _require(_launch_arg(integration, "cmd_port") == "9093", "integration Ctrl default is not 9093")
    _require(_launch_arg(udp_launch, "cmd_port") == "9093", "UDP Ctrl default is not 9093")
    _require(_launch_arg(udp_launch, "ego_port") == "9099", "Ego status bind port is not 9099")
    _require(
        _launch_arg(integration, "competition_port") == "7502",
        "integration Competition status bind port is not 7502",
    )
    _require(
        _launch_arg(udp_launch, "competition_port") == "7502",
        "UDP Competition status bind port is not 7502",
    )
    entrypoint_text = (REPO_ROOT / "scripts/udp_bridge.sh").read_text(encoding="utf-8")
    _require(
        "Usage: ./scripts/udp_bridge.sh [--transport local|lan2lan]" in entrypoint_text,
        "bridge CLI must expose transport without a simulator selector",
    )
    _require(
        'SIMULATOR="${ASMC_SIMULATOR' not in entrypoint_text,
        "bridge CLI must not select a simulator release",
    )

    summaries = {
        name: _sensor_summary(REPO_ROOT / spec["path"])
        for name, spec in contract["sensor_sets"].items()
    }
    for name, spec in contract["sensor_sets"].items():
        _require(
            _sha256(REPO_ROOT / spec["path"]) == spec["sha256"],
            f"{name} sensor set SHA changed",
        )
    for name, spec in contract["legacy_network_artifacts"].items():
        _require(
            _sha256(REPO_ROOT / spec["path"]) == spec["sha256"],
            f"{name} legacy network artifact SHA changed",
        )
    _require(summaries["v1"]["camera4_pitch_deg"] == 340.0, "v1 camera-4 pitch changed")
    _require(summaries["v2"]["camera4_pitch_deg"] == 328.0, "v2 camera-4 pitch changed")
    _require(summaries["v3"]["camera4_pitch_deg"] == 328.0, "v3 camera-4 pitch changed")
    _require(math.isclose(summaries["v2"]["lidar_period_s"], 0.1, abs_tol=1e-6), "v2 is not 10 Hz")
    _require(summaries["v2"]["lidar_frequency_index"] == 1, "v2 frequencyIndex changed")
    _require(math.isclose(summaries["v3"]["lidar_period_s"], 1.0 / 15.0, abs_tol=1e-6), "v3 is not 15 Hz")
    _require(summaries["v3"]["lidar_frequency_index"] == 2, "v3 frequencyIndex changed")
    _require(summaries["v2"]["lidar_ports"] == [9298, 9299], "v2 LiDAR ports changed")
    _require(summaries["v3"]["lidar_ports"] == [9298, 9299], "v3 LiDAR ports changed")
    _require((summaries["GT"]["cameras"], summaries["GT"]["lidars"]) == (7, 2), "GT sensor inventory changed")

    return {
        "baseline_id": contract["baseline_id"],
        "runtime_protocol": runtime["id"],
        "transport_profiles": sorted(contract["transport_profiles"]),
        "network_profiles": sum(
            len(spec["generated"]) for spec in contract["network_templates"].values()
        ),
        "packages": implementation["packages"],
        "published_topic_count": len(implementation["published_topics"]),
        "ctrl": "9093/9094 mode 2",
        "ego_status": "9088/9099",
        "competition_status": "7603/7502 static decoder; live capture pending",
        "sensor_sets": summaries,
    }


def main() -> None:
    contract_path = REPO_ROOT / "config/bridge/bridge_profiles_v1.json"
    result = validate(contract_path)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    print("P3-B bridge profile contract: PASS")


if __name__ == "__main__":
    main()
