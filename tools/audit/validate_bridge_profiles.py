#!/usr/bin/env python3
"""Validate the P3-F consolidated bridge contract and retained live gates."""

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
from render_morai_sensor_profiles import (  # noqa: E402
    SensorProfileError,
    check_all as check_all_sensor_profiles,
    iter_udp_configs,
)


class ContractError(RuntimeError):
    """Raised when the committed bridge contract and implementation diverge."""


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
    competition_pubsub = contract["channels"]["competition_vehicle_status"].get(
        "morai_pubsub_type_by_release", {}
    ).get(release)
    allowed_pubsub = {
        contract["channels"]["collision_data"]["morai_pubsub_type"],
        contract["channels"]["ego_vehicle_status"]["morai_pubsub_type_by_release"][release],
    }
    if competition_pubsub is not None:
        allowed_pubsub.add(competition_pubsub)
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
        contract["defaults"]
        == {
            "transport_profile": "local",
            "stream_profile": "competition",
            "sensor_set": "v3",
        },
        "runtime defaults must remain release-independent local + competition + v3",
    )
    stream_profiles = contract["stream_profiles"]
    _require(
        set(stream_profiles) == {"competition", "development"},
        "stream profiles must be competition and development",
    )
    _require(
        stream_profiles["competition"]["deployment_safe"],
        "competition stream profile must be deployment-safe",
    )
    _require(
        stream_profiles["development"]["allowed_transports"] == ["local"],
        "development streams must remain local-only",
    )
    expected_forbidden_topics = {
        "/Object_topic",
        "/sem_front/image_jpeg/compressed",
        "/sem_left/image_jpeg/compressed",
        "/sem_right/image_jpeg/compressed",
        "/velodyne_points_instance",
    }
    _require(
        set(stream_profiles["competition"]["forbidden_topics"])
        == expected_forbidden_topics,
        "competition privileged-topic denylist changed",
    )
    implementation = contract["implementation_baseline"]
    _require(
        implementation["packages"] == ["asmc_bridge"],
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
    _require(
        set(implementation["optional_development_topics"]) == expected_forbidden_topics,
        "development-only topic inventory changed",
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
        competition_status["morai_pubsub_type_by_release"] == {"molitcomp03": 787},
        "Competition Vehicle Status release mapping changed",
    )
    _require(
        (
            competition_status["host_port"],
            competition_status["destination_port"],
            competition_status["runtime_bridge_bind_port"],
        )
        == (9797, 9798, 9798),
        "Competition Vehicle Status must use 9797/9798",
    )
    _require(
        competition_status["state"]
        == "static_decoder_active_live_stream_not_emitted",
        "Competition decoder live-block state changed",
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
        packet_contract["live_acceptance"]
        == "pending_molitcomp03_competition_mode_packet_capture",
        "Competition live acceptance state changed",
    )
    traffic_light = contract["channels"]["traffic_light_status_oracle"]
    _require(
        (
            traffic_light["morai_pubsub_type"],
            traffic_light["host_port"],
            traffic_light["destination_port"],
        )
        == (258, 7603, 7502),
        "TrafficLight status evidence mapping changed",
    )
    traffic_light_query = contract["channels"]["traffic_light_query_oracle"]
    _require(
        traffic_light_query
        == {
            "morai_pubsub_type": 778,
            "required": False,
            "source_export_presence": {
                "26r1": False,
                "molitcomp03": False,
                "organizer_molitcomp03": False,
            },
            "state": "absent_from_inspected_exports_not_a_runtime_contract",
        },
        "GetTrafficLight/MoraiTLServiceProvider must remain optional and absent",
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
        expected_source = {
            "26r1": (
                23,
                False,
                "5b57d2c95b57548f6fda2d574f8db77e1748239cb39056c705516dd79fe93006",
            ),
            "molitcomp03": (
                12,
                True,
                "182fdaa802d59a41a343fbff7344d233031153b0435402ccbaa8ea1fbd7f4b51",
            ),
        }[release]
        _require(
            (
                spec["source_entry_count"],
                spec["competition_vehicle_status_available"],
                spec["source_export_sha256"],
            )
            == expected_source,
            f"{release} source export provenance changed",
        )
        expected_primary_types = {
            "26r1": [
                786, 258, 261, 265, 263, 267, 271, 770, 772, 779, 259,
                262, 264, 266, 776, 783, 269, 270, 274, 275, 780, 782,
            ],
            "molitcomp03": [
                771, 261, 267, 295, 770, 772, 258, 787, 259, 266, 783, 780,
            ],
        }[release]
        expected_auxiliary_types = {"26r1": [4865], "molitcomp03": []}[release]
        _require(
            spec["source_primary_ego_pubsub_types"] == expected_primary_types
            and spec["source_auxiliary_pubsub_types"] == expected_auxiliary_types,
            f"{release} source PUBSUB inventory changed",
        )
        if release == "molitcomp03":
            organizer = spec["organizer_reference"]
            _require(
                organizer["source_export_sha256"]
                == "68b5bd089b38449819ec064a67e9f821e3280068c1171e751c77e13282b57577"
                and organizer["source_entry_count"] == 10
                and organizer["source_primary_ego_pubsub_types"]
                == [771, 258, 261, 267, 295, 787, 770, 772, 259, 780],
                "organizer MolitComp03 network provenance changed",
            )
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
            status_pubsub = competition_status["morai_pubsub_type_by_release"].get(release)
            competition_entries = [
                entry
                for entry in _network_entries(generated_document)
                if status_pubsub is not None and entry.get("PUBSUB_TYPE") == status_pubsub
            ]
            if status_pubsub is None:
                _require(
                    not competition_entries,
                    f"{release} unexpectedly gained a Competition Vehicle Status mapping",
                )
            else:
                _require(
                    len(competition_entries) == 1,
                    f"{release} must contain one Competition Vehicle Status",
                )
                competition_udp = competition_entries[0]["config"]["udpConfig"]
                _require(
                    (competition_udp["hostPort"], competition_udp["destinationPort"])
                    == (9797, 9798),
                    f"{release}/{transport} Competition Vehicle Status ports changed",
                )
    try:
        check_all(contract)
    except ProfileError as exc:
        raise ContractError(str(exc)) from exc
    try:
        check_all_sensor_profiles(contract)
    except SensorProfileError as exc:
        raise ContractError(str(exc)) from exc

    facade = REPO_ROOT / "src/asmc_bridge/launch/bridge.launch"
    components = REPO_ROOT / "src/asmc_bridge/launch/morai_udp_bridges.launch"
    udp_launch = REPO_ROOT / "src/asmc_bridge/launch/morai_udp_bridge.launch"
    sensor_launch = REPO_ROOT / "src/asmc_bridge/launch/morai_sensor_bridge.launch"
    _require(_launch_arg(facade, "runtime_profile") == "morai_udp", "facade runtime profile split")
    _require(_launch_arg(facade, "transport_profile") == "local", "facade transport default changed")
    _require(_launch_arg(facade, "stream_profile") == "competition", "facade stream default changed")
    _require(_launch_arg(facade, "enable_vehicle_state") == "true", "facade vehicle state default changed")
    _require(_launch_arg(components, "runtime_profile") == "morai_udp", "component runtime profile split")
    _require(_launch_arg(udp_launch, "runtime_profile") == "morai_udp", "UDP runtime profile split")
    _require(_launch_arg(sensor_launch, "runtime_profile") == "morai_udp", "sensor runtime profile split")
    _require(_launch_arg(components, "transport_profile") == "local", "component transport default changed")
    _require(_launch_arg(udp_launch, "transport_profile") == "local", "UDP transport default changed")
    _require(_launch_arg(sensor_launch, "transport_profile") == "local", "sensor transport default changed")
    _require(_launch_arg(components, "stream_profile") == "competition", "component stream default changed")
    _require(_launch_arg(udp_launch, "stream_profile") == "competition", "UDP stream default changed")
    _require(_launch_arg(sensor_launch, "stream_profile") == "competition", "sensor stream default changed")
    for launch_path in (facade, components, sensor_launch):
        _require(
            _launch_arg(launch_path, "lidar_frame_mode") == "azimuth",
            f"{launch_path.relative_to(REPO_ROOT)} LiDAR framing is not azimuth-based",
        )
    expected_development_eval = "$(eval arg('stream_profile') == 'development')"
    _require(
        _launch_arg(components, "enable_object") == expected_development_eval,
        "component Object allowlist is not stream-profile controlled",
    )
    _require(
        _launch_arg(components, "enable_semantic_camera")
        == expected_development_eval,
        "component Semantic allowlist is not stream-profile controlled",
    )
    _require(
        _launch_arg(components, "enable_instance_lidar")
        == expected_development_eval,
        "component Instance allowlist is not stream-profile controlled",
    )
    _require(_launch_arg(components, "cmd_port") == "9093", "component Ctrl default is not 9093")
    _require(_launch_arg(udp_launch, "cmd_port") == "9093", "UDP Ctrl default is not 9093")
    _require(_launch_arg(udp_launch, "ego_port") == "9099", "Ego status bind port is not 9099")
    _require(
        _launch_arg(components, "competition_port") == "9798",
        "component Competition status bind port is not 9798",
    )
    _require(
        _launch_arg(udp_launch, "competition_port") == "9798",
        "UDP Competition status bind port is not 9798",
    )
    entrypoint_text = (REPO_ROOT / "scripts/udp_bridge.sh").read_text(encoding="utf-8")
    _require(
        "[--transport local|lan2lan] [--streams competition|development]"
        in entrypoint_text,
        "bridge CLI must expose transport and streams without a simulator selector",
    )
    _require(
        'SIMULATOR="${ASMC_SIMULATOR' not in entrypoint_text,
        "bridge CLI must not select a simulator release",
    )
    _require(
        "development streams are local-only" in entrypoint_text,
        "bridge CLI must reject LAN2LAN privileged streams",
    )
    _require(
        "exec roslaunch asmc_bridge bridge.launch" in entrypoint_text,
        "bridge CLI must use the canonical asmc_bridge facade",
    )
    _require(
        'CMD_IP="$TRANSPORT_CMD_IP"' in entrypoint_text,
        "bridge CLI must derive Ctrl destination from the selected transport",
    )
    compose_text = (REPO_ROOT / "docker/ros-noetic/docker-compose.yaml").read_text(
        encoding="utf-8"
    )
    _require(
        "MORAI_CMD_IP=" not in compose_text,
        "compose must not override the selected transport Ctrl destination",
    )
    composition = contract["composition"]
    _require(
        composition
        == {
            "canonical_package": "asmc_bridge",
            "canonical_launch": "bridge.launch",
            "vehicle_state_default": True,
            "vehicle_state_source": "gps_imu",
            "lidar_frame_mode": "azimuth",
            "compatibility_launch": "asmc_bridge/morai_udp_bridges.launch",
            "legacy_packages_retained_for_rollback": False,
        },
        "bridge composition contract changed",
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
        for transport, generated_rel in spec.get("transport_generated", {}).items():
            generated_path = REPO_ROOT / generated_rel
            _require(
                _sha256(generated_path)
                == spec["transport_generated_sha256"][transport],
                f"{name}/{transport} generated sensor SHA changed",
            )
            document = json.loads(generated_path.read_text(encoding="utf-8"))
            expected_ips = (
                contract["transport_profiles"][transport]["morai_ip"],
                contract["transport_profiles"][transport]["autonav_ip"],
            )
            udp_configs = list(iter_udp_configs(document))
            _require(
                len(udp_configs) == spec["expected_udp_channels"],
                f"{name}/{transport} sensor UDP channel count changed",
            )
            _require(
                all(
                    (udp.get("hostIP"), udp.get("destinationIP")) == expected_ips
                    for udp in udp_configs
                ),
                f"{name}/{transport} sensor IP mapping changed",
            )
    for name in ("v2", "v3"):
        _require(
            set(contract["sensor_sets"][name]["transport_generated"])
            == {"local", "lan2lan"},
            f"{name} must provide local and LAN2LAN sensor imports",
        )
    _require(
        "transport_generated" not in contract["sensor_sets"]["GT"],
        "GT sensor set must not have a deployment transport derivative",
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
    _require(
        contract["sensor_sets"]["v3"]["live_acceptance"]
        == "accepted_configured_15hz_observed_complete_scans_about_12hz",
        "v3 configured 15 Hz acceptance semantics changed",
    )
    _require(summaries["v2"]["lidar_ports"] == [9298, 9299], "v2 LiDAR ports changed")
    _require(summaries["v3"]["lidar_ports"] == [9298, 9299], "v3 LiDAR ports changed")
    _require((summaries["GT"]["cameras"], summaries["GT"]["lidars"]) == (7, 2), "GT sensor inventory changed")

    live = contract["live_observations"]
    _require(live["date"] == "2026-09-23", "P3-E live observation date changed")
    _require(live["simulator_release"] == "molitcomp03", "P3-E simulator changed")
    _require(live["transport_profile"] == "local", "P3-E transport changed")
    _require(live["sensor_set"] == "v3", "P3-E sensor set changed")
    lidar_live = live["lidar"]
    _require(lidar_live["packets_per_revolution"] == 50, "live LiDAR packet grouping changed")
    _require(
        (
            lidar_live["configured_sensor_hz"],
            lidar_live["consumer_poll_hz"],
            lidar_live["observed_complete_scan_hz_approx"],
            lidar_live["duplicate_resampling_enabled"],
            lidar_live["accepted"],
        ) == (15.0, 15.0, 12.0, False, True),
        "configured-15/observed-12 LiDAR requirement changed",
    )
    _require(
        live["vehicle_state"]["valid_after_initial_sensor_window"]
        and live["ctrl_closed_loop"]["accepted"]
        and live["bag_replay"]["accepted"],
        "local vehicle-state, Ctrl, or bag-replay gate regressed",
    )
    competition_live = live["competition_udp_9798"]
    _require(
        competition_live["configured_pubsub_type"] == 787
        and competition_live["observed_packets_in_8_seconds"] == 0
        and not competition_live["accepted"],
        "UDP 9798 live Competition stream must remain pending",
    )
    traffic_light_live = live["traffic_light_udp_7502"]
    _require(
        traffic_light_live["configured_pubsub_type"] == 258
        and traffic_light_live["observed_header_ascii"] == "#TrafficLight$"
        and not traffic_light_live["consumed_by_bridge"],
        "UDP 7502 TrafficLight evidence changed",
    )

    return {
        "baseline_id": contract["baseline_id"],
        "runtime_protocol": runtime["id"],
        "transport_profiles": sorted(contract["transport_profiles"]),
        "network_profiles": sum(
            len(spec["generated"]) for spec in contract["network_templates"].values()
        ),
        "sensor_transport_profiles": sum(
            len(spec.get("transport_generated", {}))
            for spec in contract["sensor_sets"].values()
        ),
        "default_stream_profile": contract["defaults"]["stream_profile"],
        "canonical_launch": "asmc_bridge/bridge.launch",
        "packages": implementation["packages"],
        "published_topic_count": len(implementation["published_topics"]),
        "ctrl": "9093/9094 mode 2",
        "ego_status": "9088/9099",
        "competition_status": "9797/9798 type 787; live emission pending",
        "lidar_frame_mode": composition["lidar_frame_mode"],
        "v3_configured_cycle_accepted": lidar_live["accepted"],
        "sensor_sets": summaries,
    }


def main() -> None:
    contract_path = REPO_ROOT / "config/bridge/bridge_profiles_v1.json"
    result = validate(contract_path)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    print("P3-F consolidated bridge profile contract: PASS")


if __name__ == "__main__":
    main()
