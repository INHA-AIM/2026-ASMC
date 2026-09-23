#!/usr/bin/env python3
"""Render deterministic MORAI sensor imports for local and LAN2LAN transport."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, Tuple


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = REPO_ROOT / "config/bridge/bridge_profiles_v1.json"


class SensorProfileError(RuntimeError):
    """Raised when a sensor transport profile cannot be rendered safely."""


def load_contract(path: Path = DEFAULT_CONTRACT) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def iter_udp_configs(value: Any) -> Iterator[Dict[str, Any]]:
    """Yield every typed sensor udpConfig without assuming a release schema."""
    if isinstance(value, dict):
        for key, child in value.items():
            if key == "udpConfig" and isinstance(child, dict):
                yield child
            else:
                yield from iter_udp_configs(child)
    elif isinstance(value, list):
        for child in value:
            yield from iter_udp_configs(child)


def _without_udp_ips(value: Any) -> Any:
    result = copy.deepcopy(value)
    for udp in iter_udp_configs(result):
        udp.pop("hostIP", None)
        udp.pop("destinationIP", None)
    return result


def render_sensor_profile(
    contract: Dict[str, Any], sensor_set: str, transport: str
) -> Dict[str, Any]:
    try:
        sensor_spec = contract["sensor_sets"][sensor_set]
        transport_spec = contract["transport_profiles"][transport]
        sensor_spec["transport_generated"][transport]
    except KeyError as exc:
        raise SensorProfileError(f"unknown or non-renderable profile key: {exc.args[0]}") from exc

    if transport == "lan2lan" and sensor_spec.get("role") == "development_ground_truth_only":
        raise SensorProfileError("development GT sensor set must not be rendered for LAN2LAN")

    source_path = REPO_ROOT / sensor_spec["path"]
    source = json.loads(source_path.read_text(encoding="utf-8"))
    result = copy.deepcopy(source)
    udp_configs = list(iter_udp_configs(result))
    expected_count = int(sensor_spec["expected_udp_channels"])
    if len(udp_configs) != expected_count:
        raise SensorProfileError(
            f"{sensor_set} has {len(udp_configs)} UDP channels, expected {expected_count}"
        )

    for udp in udp_configs:
        udp["hostIP"] = transport_spec["morai_ip"]
        udp["destinationIP"] = transport_spec["autonav_ip"]

    if _without_udp_ips(source) != _without_udp_ips(result):
        raise SensorProfileError("sensor renderer changed fields other than UDP IPs")
    return result


def rendered_bytes(document: Dict[str, Any]) -> bytes:
    return (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


def profile_items(contract: Dict[str, Any]) -> Iterable[Tuple[str, str, Path]]:
    for sensor_set, spec in contract["sensor_sets"].items():
        for transport, output_rel in spec.get("transport_generated", {}).items():
            yield sensor_set, transport, REPO_ROOT / output_rel


def write_all(contract: Dict[str, Any]) -> None:
    for sensor_set, transport, output in profile_items(contract):
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(
            rendered_bytes(render_sensor_profile(contract, sensor_set, transport))
        )
        print(f"wrote {output.relative_to(REPO_ROOT)}")


def check_all(contract: Dict[str, Any]) -> None:
    failures = []
    for sensor_set, transport, output in profile_items(contract):
        expected = rendered_bytes(
            render_sensor_profile(contract, sensor_set, transport)
        )
        if not output.is_file():
            failures.append(f"missing {output.relative_to(REPO_ROOT)}")
        elif output.read_bytes() != expected:
            failures.append(f"stale {output.relative_to(REPO_ROOT)}")
    if failures:
        raise SensorProfileError("; ".join(failures))
    print("MORAI generated sensor profiles are deterministic and current")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--write", action="store_true")
    group.add_argument("--check", action="store_true")
    args = parser.parse_args()
    contract = load_contract(args.contract)
    if args.write:
        write_all(contract)
    else:
        check_all(contract)


if __name__ == "__main__":
    main()
