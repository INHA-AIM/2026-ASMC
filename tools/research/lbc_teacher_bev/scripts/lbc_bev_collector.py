#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS collector: MORAI ego/objects -> LBC 7-channel BEV (320x320) NPZ dumps."""
from __future__ import annotations

import os
import sys
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rospy

from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList

_LBC_ROOT = Path(__file__).resolve().parents[1]
if str(_LBC_ROOT) not in sys.path:
    sys.path.insert(0, str(_LBC_ROOT))

from lbc_bev import LBCRenderer
from lbc_bev.morai_adapters import (
    TrafficLightStateCache,
    TurnSignalTracker,
    load_intersection_traffic_groups,
    load_intscn_traffic_light_map,
    load_traffic_light_catalog,
    object_status_list_to_states,
    setup_morai_event_cmd_turn_signal_polling,
)
from lbc_bev.static_layers import compute_bev_affine_matrix
from lbc_bev.ws_root import asmc_ws_root, lbc_data_root

PUBLISH_HZ = 10.0
SAVE_INTERVAL_S = 1.0


class LBCBEVCollector:
    def __init__(self):
        rospy.init_node("lbc_bev_collector", anonymous=False)
        rospy.loginfo(
            "[LBCBEV] ROS_MASTER_URI=%s | data_root=%s",
            os.environ.get("ROS_MASTER_URI", "(unset)"),
            lbc_data_root(),
        )
        ws_root = os.path.expanduser(
            rospy.get_param("~aim_ws_root", str(asmc_ws_root()))
        )
        self.renderer = LBCRenderer(ws_root)
        ws = Path(ws_root)
        self._tl_catalog = load_traffic_light_catalog(self.renderer.tl_json, vehicle_only=True)
        self._intscn_catalog = load_intscn_traffic_light_map(
            ws / "R_KR_PG_KATRI/synced_traffic_light_set.json",
            vehicle_signal_ids=set(self._tl_catalog.keys()),
        )

        self.ego_pose = None
        self.latest_obj = None
        self._tl_cache = TrafficLightStateCache()
        synced = ws / "R_KR_PG_KATRI/synced_traffic_light_set.json"
        self._tl_cache.set_traffic_groups(
            load_intersection_traffic_groups(synced, set(self._tl_catalog.keys()))
        )
        self._turn_signal = TurnSignalTracker()
        self.lock = threading.Lock()
        self.last_save_time = 0.0
        self.bev_save_dir = self._init_bev_save_dir()
        self.save_visualization = bool(rospy.get_param("~save_visualization", False))

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self._ego_cb, queue_size=10)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self._obj_cb, queue_size=1)
        self._subscribe_traffic_light_optional()
        self._setup_morai_event_cmd_lamps()

        rospy.Timer(rospy.Duration(1.0 / PUBLISH_HZ), self._timer_cb)
        rospy.loginfo(
            "[LBCBEV] started at %.0f Hz | save_dir=%s | tl_catalog=%d",
            PUBLISH_HZ,
            self.bev_save_dir,
            len(self._tl_catalog),
        )
        rospy.spin()

    def _subscribe_traffic_light_optional(self):
        param_topic = rospy.get_param("~traffic_light_topic", "")
        topics = (
            [param_topic]
            if param_topic
            else ["/GetTrafficLightStatus", "/IntscnTL_topic", "/TrafficLight_status"]
        )
        try:
            from morai_msgs.msg import (
                GetTrafficLightStatus,
                IntscnTL,
                MoraiTLInfo,
                TrafficLight,
            )
            tl_msg_types = (GetTrafficLightStatus, IntscnTL, MoraiTLInfo, TrafficLight)
            subscribed = []
            for topic in topics:
                if not topic:
                    continue
                for msg_type in tl_msg_types:
                    try:
                        rospy.Subscriber(topic, msg_type, self._tl_cb, queue_size=10)
                        subscribed.append(f"{topic} ({msg_type.__name__})")
                    except Exception:
                        continue
            if subscribed:
                rospy.loginfo("[LBCBEV] traffic subscribers: %s", ", ".join(subscribed))
                for topic in topics:
                    if not topic:
                        continue
                    for msg_type in (GetTrafficLightStatus, IntscnTL):
                        try:
                            boot = rospy.wait_for_message(topic, msg_type, timeout=1.0)
                            self._tl_cache.ingest(boot)
                            break
                        except rospy.ROSException:
                            continue
                return
            rospy.logwarn("[LBCBEV] could not subscribe traffic topics %s", topics)
        except Exception as exc:
            rospy.logwarn("[LBCBEV] traffic light subscribe skipped: %s", exc)

    def _init_bev_save_dir(self):
        base_dir = Path(
            os.path.expanduser(
                rospy.get_param("~data_root", str(lbc_data_root()))
            )
        ) / "bev_map"
        common_timestamp = os.environ.get("COMMON_TIMESTAMP")
        if common_timestamp:
            timestamp_str = common_timestamp
            rospy.loginfo("[LBCBEV] COMMON_TIMESTAMP=%s", timestamp_str)
        else:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            rospy.logwarn("[LBCBEV] COMMON_TIMESTAMP unset, using %s", timestamp_str)
        scenario_dir = base_dir / f"LBC_{timestamp_str}"
        scenario_dir.mkdir(parents=True, exist_ok=True)
        return scenario_dir

    def _ego_cb(self, msg):
        with self.lock:
            self.ego_pose = {
                "x": float(msg.position.x),
                "y": float(msg.position.y),
                "yaw_deg": float(msg.heading),
            }

    def _obj_cb(self, msg):
        with self.lock:
            self.latest_obj = msg

    def _tl_cb(self, msg):
        with self.lock:
            self._tl_cache.ingest(msg)


    def _setup_morai_event_cmd_lamps(self) -> None:
        """Poll /Service_MoraiEventCmd only — do not use /Lamps_topic (empty on KATRI)."""
        setup_morai_event_cmd_turn_signal_polling(
            self._turn_signal, self.lock, log_tag="[LBCBEV]"
        )

    def _timer_cb(self, _event):
        with self.lock:
            if self.ego_pose is None:
                return
            ego = dict(self.ego_pose)
            obj = self.latest_obj
            turn_mode = self._turn_signal.mode
            tl_cache = self._tl_cache

        baked = self.renderer.baked_maps
        bev_affine = compute_bev_affine_matrix(
            baked, ego["x"], ego["y"], ego["yaw_deg"]
        )
        vehicles, pedestrians = object_status_list_to_states(obj)
        traffic_lights = tl_cache.build_traffic_lights(
            self._tl_catalog,
            self._intscn_catalog,
            turn_mode=turn_mode,
            bev_affine=bev_affine,
            baked=baked,
            filter_to_bev=True,
            ego_xy=(ego["x"], ego["y"]),
            bev_margin_px=48,
        )

        out = self.renderer.render(
            ego["x"],
            ego["y"],
            ego["yaw_deg"],
            vehicles=vehicles,
            pedestrians=pedestrians,
            traffic_lights=traffic_lights,
        )
        birdview = out["birdview"].astype(np.uint8)
        cropped = out["cropped"].astype(np.uint8)
        stamp = rospy.Time.now()
        self._save_bev(birdview, cropped, out.get("visualization"), stamp)

    def _save_bev(self, birdview, cropped, visualization, stamp):
        now = rospy.get_time()
        if now - self.last_save_time < SAVE_INTERVAL_S:
            return
        self.last_save_time = now

        try:
            ts = stamp.secs + stamp.nsecs / 1e9
            path = self.bev_save_dir / f"{ts:.6f}.npz"
            np.savez_compressed(
                path,
                bev_map=birdview,
                birdview_cropped=cropped,
            )
            if self.save_visualization and visualization is not None:
                vis_path = self.bev_save_dir / f"{ts:.6f}_vis.png"
                cv2.imwrite(str(vis_path), cv2.cvtColor(visualization, cv2.COLOR_RGB2BGR))
            rospy.logdebug("[LBCBEV] saved %s shape=%s", path, birdview.shape)
        except Exception as exc:
            rospy.logerr("[LBCBEV] save failed: %s", exc)


def main():
    try:
        LBCBEVCollector()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
