#!/usr/bin/env python3
"""
Probe MORAI gRPC: connectivity + GetAvailableMaps + GetMGeo.

WSL note: do NOT use 127.0.0.1 for a Windows-hosted SIM.
  Use the Windows host as seen from WSL, usually the default gateway:
    ip route show default   → e.g. 172.24.208.1
  Check:  python3 -c "import socket; socket.create_connection(('172.24.208.1',7789),2)"

Usage:
  export MORAI_SIM_HOST=172.24.208.1
  /tmp/mgeo-probe-venv/bin/python tools/probe_get_mgeo.py --out /tmp/mgeo_dump
"""

from __future__ import print_function

import argparse
import json
import os
import subprocess
import sys


def _guess_host():
    env = os.environ.get("MORAI_SIM_HOST")
    if env:
        return env
    try:
        out = subprocess.check_output(["ip", "route", "show", "default"], text=True)
        return out.split()[2]
    except Exception:
        return "127.0.0.1"


def _add_grpc_paths():
    root = os.environ.get("AIM_PROJECT") or os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..")
    )
    grpc_src = os.environ.get(
        "MORAI_GRPC_SRC",
        os.path.join(
            root,
            "external/team/2026-summer-VIP2/grpc_inha_univ/src",
        ),
    )
    for p in (grpc_src, os.path.join(grpc_src, "api"), os.path.join(grpc_src, "proto")):
        if not os.path.isdir(p):
            raise SystemExit("grpc src missing: %s (set MORAI_GRPC_SRC)" % p)
        if p not in sys.path:
            sys.path.insert(0, p)
    return grpc_src


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", action="append", dest="maps", default=None)
    ap.add_argument("--host", default=_guess_host())
    ap.add_argument("--port", type=int, default=int(os.environ.get("MORAI_SIM_PORT", "7789")))
    ap.add_argument("--out", default="")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    _add_grpc_paths()
    try:
        import grpc
    except ImportError:
        print(
            "ERROR: need grpcio. e.g.\n"
            "  uv venv /tmp/mgeo-probe-venv\n"
            "  uv pip install --python /tmp/mgeo-probe-venv/bin/python grpcio protobuf"
        )
        return 1

    import proto.sim_adapter as sa

    sa.MAX_MESSAGE_LENGTH = 512 * 1024 * 1024
    from proto.sim_adapter import SimAdapter
    from proto.morai.common.type_pb2 import StringValue

    try:
        from lib.common.compress_util import decompress
    except ImportError:
        import base64
        import gzip

        def decompress(compressed, base64_encoded=True):
            source = (
                base64.standard_b64decode(compressed) if base64_encoded else compressed
            )
            return gzip.decompress(source).decode("utf-8")

    interesting = (
        "road_mesh",
        "lane_boundary",
        "lane_marking",
        "synced_traffic",
        "traffic_light",
        "link_set",
        "node_set",
        "global_info",
    )

    print("connect %s:%s ..." % (args.host, args.port))
    if args.host in ("127.0.0.1", "localhost"):
        print(
            "NOTE: WSL→Windows SIM이면 127.0.0.1 대신 "
            "`ip route show default`의 gateway IP를 쓰세요."
        )

    adapter = SimAdapter()
    try:
        adapter.connect(args.host, args.port)
        grpc.channel_ready_future(adapter._channel).result(timeout=5.0)
        print("channel READY")
    except Exception as e:
        print("CONNECT FAIL:", e)
        return 2

    try:
        maps = adapter.get_available_maps()
        names = list(maps.values) if maps is not None else []
        print("GetAvailableMaps: %d maps" % len(names))
        for n in names:
            if "2025" in n or "KATRI" in n or "K-City" in n or "K-city" in n:
                print("  *", n)
    except Exception as e:
        print("GetAvailableMaps FAIL:", e)
        names = []

    maps_to_try = args.maps or [
        "R_KR_PR_K-city_2025",
        "R_KR_PG_K-city_2025",
        "R_KR_PG_KATRI",
    ]
    for n in names:
        if n not in maps_to_try and ("2025" in n or n == "R_KR_PG_KATRI"):
            maps_to_try.append(n)

    any_ok = False
    for map_name in maps_to_try:
        print("\n=== GetMGeo(%r) ===" % map_name)
        param = StringValue(value=map_name)
        try:
            resp = adapter._map_stub.GetMGeo(param, timeout=args.timeout)
        except Exception as e:
            print("RPC error:", e)
            continue
        if resp is None or not getattr(resp, "objects", None):
            print("empty")
            continue
        any_ok = True
        fnames = [o.filename for o in resp.objects]
        print("n_files:", len(fnames))
        for n in sorted(fnames):
            flag = "  << interest" if any(k in n.lower() for k in interesting) else ""
            print(" -", n, flag)
        if args.out:
            out_dir = os.path.join(args.out, map_name.replace("/", "_"))
            os.makedirs(out_dir, exist_ok=True)
            meta = []
            for o in resp.objects:
                try:
                    text = decompress(o.compressed_data)
                except Exception as e:
                    print("decompress fail", o.filename, e)
                    continue
                path = os.path.join(out_dir, o.filename)
                os.makedirs(os.path.dirname(path) or out_dir, exist_ok=True)
                with open(path, "w", encoding="utf-8") as f:
                    f.write(text)
                meta.append({"filename": o.filename, "bytes": len(text)})
            with open(os.path.join(out_dir, "_index.json"), "w") as f:
                json.dump({"map": map_name, "files": meta}, f, indent=2)
            print("wrote", out_dir)

    if not any_ok:
        print(
            "\nGetMGeo FAILED for all tried names (often StatusCode.CANCELLED).\n"
            "gRPC 자체는 살아 있음(GetAvailableMaps OK). 이 SIM 빌드/라이선스에서\n"
            "GetMGeo 가 막혔거나 서버가 즉시 거절하는 경우로 보임.\n"
            "→ bake는 공개 link_set 근사 경로로 진행."
        )
        return 4
    print("\nOK — interest 파일에 lane_boundary/road_mesh 있으면 dump 사용 가능.")
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
