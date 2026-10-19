# -*- coding: utf-8 -*-
"""Dataset layout writer: meta.yaml + manifest.jsonl + channel files."""

from __future__ import print_function

import json
import os
import subprocess
from datetime import datetime

import yaml

from data_collection import SCHEMA_VERSION


def default_asmc_data_root():
    from .storage_paths import default_data_root
    return default_data_root()



def _git_sha(cwd=None):
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=cwd,
            stderr=subprocess.DEVNULL,
        )
        return out.decode("utf-8").strip()
    except Exception:
        return "unknown"


class DatasetWriter(object):
    def __init__(self, profile, run_id=None, data_root=None, asmc_cwd=None):
        self.profile = profile
        self.data_root = os.path.expanduser(data_root or default_asmc_data_root())
        self.run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.root = os.path.join(self.data_root, "datasets", self.run_id)
        self._frame_idx = 0
        self._skipped = {"no_ego": 0, "channel_missing": 0}
        self._manifest = None
        self._ego_fp = None
        self.asmc_cwd = asmc_cwd
        from .storage_paths import validate_output
        validate_output(self.data_root, "datasets", self.run_id)
        if os.path.exists(self.root):
            raise RuntimeError("Collection run already exists; choose a new run ID")
        self._prepare_dirs()
        self._write_meta()
        self._manifest = open(
            os.path.join(self.root, "manifest.jsonl"), "a", buffering=1
        )
        if profile["channels"].get("ego") or profile["channels"].get("competition_ego"):
            self._ego_fp = open(
                os.path.join(self.root, "state", "ego.jsonl"), "a", buffering=1
            )

    def _prepare_dirs(self):
        os.makedirs(self.root, exist_ok=False)
        for sub in (
            "sensors/cam_front",
            "sensors/cam_left",
            "sensors/cam_right",
            "sensors/cam_extra",
            "sensors/lidar",
            "state",
            "gt/objects",
            "gt/traffic_lights",
            "gt/collision",
            "cmd",
        ):
            os.makedirs(os.path.join(self.root, sub), exist_ok=True)

    def _write_meta(self):
        meta = {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.run_id,
            "profile": self.profile.get("name"),
            "description": self.profile.get("description"),
            "channels": self.profile.get("channels"),
            "topics": self.profile.get("topics"),
            "collect_hz": self.profile.get("collect_hz"),
            "mode": self.profile.get("mode"),
            "map": self.profile.get("map"),
            "vehicle": self.profile.get("vehicle"),
            "bridge_type": self.profile.get("bridge_type"),
            "git_sha": _git_sha(self.asmc_cwd),
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "notes": self.profile.get("notes"),
            "privileged_gt": True,
            "warning": "gt/ is train/eval only — never import in deploy graph",
        }
        path = os.path.join(self.root, "meta.yaml")
        with open(path, "w") as f:
            yaml.safe_dump(meta, f, default_flow_style=False, allow_unicode=True)

    @property
    def frame_idx(self):
        return self._frame_idx

    def close(self):
        if self._manifest:
            self._manifest.close()
            self._manifest = None
        if self._ego_fp:
            self._ego_fp.close()
            self._ego_fp = None
        summary = {
            "frames_written": self._frame_idx,
            "skipped": self._skipped,
        }
        with open(os.path.join(self.root, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)

    def write_frame(self, payload):
        """
        payload keys (optional):
          sim_time, stamp_ns, ego, competition_ego,
          cams: {name: jpeg_bytes},
          lidar_xyzi: Nx4 float32 bytes or None,
          objects: dict/list,
          tl_status: dict/list,
          collision: dict,
          missing: [channel names skipped]
        """
        idx = self._frame_idx
        stem = "{:06d}".format(idx)
        paths = {}
        keys = {}
        missing = list(payload.get("missing") or [])

        if "ego" in payload and payload["ego"] is not None and self._ego_fp:
            row = {"frame_idx": idx, "sim_time": payload.get("sim_time"), "ego": payload["ego"]}
            if payload.get("competition_ego") is not None:
                row["competition_ego"] = payload["competition_ego"]
            self._ego_fp.write(json.dumps(row, ensure_ascii=False) + "\n")
            keys["ego_row"] = idx

        cams = payload.get("cams") or {}
        for name, blob in cams.items():
            if not blob:
                missing.append(name)
                continue
            rel = "sensors/{}/{}.jpg".format(name, stem)
            with open(os.path.join(self.root, rel), "wb") as f:
                f.write(blob)
            paths[name] = rel

        lidar_bytes = payload.get("lidar_xyzi")
        if lidar_bytes is not None:
            rel = "sensors/lidar/{}.bin".format(stem)
            with open(os.path.join(self.root, rel), "wb") as f:
                f.write(lidar_bytes)
            paths["lidar"] = rel
            keys["lidar_points"] = payload.get("lidar_n", 0)

        if payload.get("objects") is not None:
            rel = "gt/objects/{}.json".format(stem)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["objects"], f, ensure_ascii=False)
            keys["gt_objects"] = rel

        if payload.get("tl_status") is not None:
            rel = "gt/traffic_lights/{}.json".format(stem)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["tl_status"], f, ensure_ascii=False)
            keys["gt_tl"] = rel

        if payload.get("collision") is not None:
            rel = "gt/collision/{}.json".format(stem)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["collision"], f, ensure_ascii=False)
            keys["collision"] = rel

        if payload.get("gps") is not None:
            rel = "state/gps/{}.json".format(stem)
            os.makedirs(os.path.join(self.root, "state", "gps"), exist_ok=True)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["gps"], f, ensure_ascii=False)
            keys["gps"] = rel

        if payload.get("imu") is not None:
            rel = "state/imu/{}.json".format(stem)
            os.makedirs(os.path.join(self.root, "state", "imu"), exist_ok=True)
            with open(os.path.join(self.root, rel), "w") as f:
                json.dump(payload["imu"], f, ensure_ascii=False)
            keys["imu"] = rel

        if missing:
            self._skipped["channel_missing"] += 1

        rec = {
            "frame_idx": idx,
            "sim_time": payload.get("sim_time"),
            "stamp_ns": payload.get("stamp_ns"),
            "paths": paths,
            "keys": keys,
        }
        if missing:
            rec["missing"] = missing
        self._manifest.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._frame_idx += 1
        return idx
