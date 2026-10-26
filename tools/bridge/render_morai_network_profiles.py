#!/usr/bin/env python3
"""Render deterministic MORAI network imports from release and transport profiles."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = REPO_ROOT / "config/bridge/bridge_profiles_v1.json"


class ProfileError(RuntimeError):
    """Raised when the bridge profile cannot be rendered safely."""


def load_contract(path: Path = DEFAULT_CONTRACT) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _entries(document: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
    for ego in document.get("egoNetworkData", []):
        for entry in ego.get("listEgoNetworkInfo", []):
            yield entry


def _rewrite_udp_ips(config: Dict[str, Any], morai_ip: str, autonav_ip: str) -> None:
    udp = config.get("udpConfig")
    if not isinstance(udp, dict):
        return
    udp["hostIP"] = morai_ip
    udp["destinationIP"] = autonav_ip


def render_profile(
    contract: Dict[str, Any], release: str, transport: str
) -> Dict[str, Any]:
    try:
        template_rel = contract["network_templates"][release]["template"]
        transport_data = contract["transport_profiles"][transport]
    except KeyError as exc:
        raise ProfileError(f"unknown profile key: {exc.args[0]}") from exc

    if release == "molitcomp03":
        verified = contract.get("verified_status_bindings", {}).get(release, {})
        status = verified.get("competition", {})
        detail = verified.get("detail", {})
        if (status.get("pubsub_type"), status.get("destination_port"), status.get("lan_allowed"),
                detail.get("pubsub_type"), detail.get("lan_allowed")) != (295, 9099, True, 787, False):
            raise ProfileError("MolitComp03 requires the verified 9099 Competition / local-only Detail mapping")
    template = json.loads((REPO_ROOT / template_rel).read_text(encoding="utf-8"))
    result = copy.deepcopy(template)
    morai_ip = transport_data["morai_ip"]
    autonav_ip = transport_data["autonav_ip"]
    channels = contract["channels"]
    field_transport = transport == "lan2lan"
    default_morai_ip = "127.0.0.1" if field_transport else morai_ip
    default_autonav_ip = "127.0.0.1" if field_transport else autonav_ip
    ctrl_found = collision_found = False
    status_pubsub = channels["competition_vehicle_status"].get(
        "morai_pubsub_type_by_release", {}
    ).get(release)
    status_found = status_pubsub is None
    ego_status_found = False
    ego_status_pubsub = channels["ego_vehicle_status"][
        "morai_pubsub_type_by_release"
    ][release]

    for entry in _entries(result):
        # Field output is fail-closed: all unspecified/privileged channels stay
        # on the MORAI PC loopback. Only the explicit competition allowlist is
        # promoted to LAN IPs below.
        _rewrite_udp_ips(
            entry.get("config", {}), default_morai_ip, default_autonav_ip
        )
        _rewrite_udp_ips(
            entry.get("EgoCtrlConfig", {}), default_morai_ip, default_autonav_ip
        )

        if entry.get("Division_TYPE") == 3:
            ctrl = entry["EgoCtrlConfig"]
            udp = ctrl["udpConfig"]
            _rewrite_udp_ips(ctrl, morai_ip, autonav_ip)
            ctrl["egoCtrlMode"] = channels["ego_ctrl_cmd"]["ctrl_mode"]
            udp["hostPort"] = channels["ego_ctrl_cmd"]["host_port"]
            udp["destinationPort"] = channels["ego_ctrl_cmd"]["destination_port"]
            ctrl_found = True

        pubsub = entry.get("PUBSUB_TYPE")
        if status_pubsub is not None and pubsub == status_pubsub:
            udp = entry["config"]["udpConfig"]
            binding = contract.get("verified_status_bindings", {}).get(release, {})
            if not field_transport or binding.get("detail", {}).get("lan_allowed", True):
                _rewrite_udp_ips(entry["config"], morai_ip, autonav_ip)
            udp["hostPort"] = channels["competition_vehicle_status"]["host_port"]
            udp["destinationPort"] = channels["competition_vehicle_status"]["destination_port"]
            status_found = True
        elif pubsub == ego_status_pubsub:
            udp = entry["config"]["udpConfig"]
            _rewrite_udp_ips(entry["config"], morai_ip, autonav_ip)
            udp["hostPort"] = channels["ego_vehicle_status"]["host_port"]
            udp["destinationPort"] = channels["ego_vehicle_status"]["destination_port"]
            ego_status_found = True
        elif pubsub == channels["collision_data"]["morai_pubsub_type"]:
            udp = entry["config"]["udpConfig"]
            _rewrite_udp_ips(entry["config"], morai_ip, autonav_ip)
            udp["hostPort"] = channels["collision_data"]["host_port"]
            udp["destinationPort"] = channels["collision_data"]["destination_port"]
            collision_found = True

    missing = [
        name
        for name, found in (
            ("Ego Ctrl Cmd", ctrl_found),
            ("Competition Vehicle Status", status_found),
            ("Ego Vehicle Status", ego_status_found),
            ("CollisionData", collision_found),
        )
        if not found
    ]
    if missing:
        raise ProfileError(
            f"{release} template lacks required MORAI entries: {', '.join(missing)}"
        )
    return result


def rendered_bytes(document: Dict[str, Any]) -> bytes:
    return (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


def profile_items(contract: Dict[str, Any]) -> Iterable[Tuple[str, str, Path]]:
    for release, spec in contract["network_templates"].items():
        for transport, output_rel in spec["generated"].items():
            yield release, transport, REPO_ROOT / output_rel


def write_all(contract: Dict[str, Any]) -> None:
    for release, transport, output in profile_items(contract):
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(rendered_bytes(render_profile(contract, release, transport)))
        print(f"wrote {output.relative_to(REPO_ROOT)}")


def check_all(contract: Dict[str, Any]) -> None:
    failures = []
    for release, transport, output in profile_items(contract):
        expected = rendered_bytes(render_profile(contract, release, transport))
        if not output.is_file():
            failures.append(f"missing {output.relative_to(REPO_ROOT)}")
        elif output.read_bytes() != expected:
            failures.append(f"stale {output.relative_to(REPO_ROOT)}")
    if failures:
        raise ProfileError("; ".join(failures))
    print("MORAI generated network profiles are deterministic and current")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--write", action="store_true", help="write every generated profile")
    group.add_argument("--check", action="store_true", help="fail if generated files are stale")
    args = parser.parse_args()
    contract = load_contract(args.contract)
    if args.write:
        write_all(contract)
    else:
        check_all(contract)


if __name__ == "__main__":
    main()
