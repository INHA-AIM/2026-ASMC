#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish LBC 7-channel BEV as ROS images; optional OpenCV imshow window."""
from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from sensor_msgs.msg import Image

_LBC_ROOT = Path(__file__).resolve().parents[1]
if str(_LBC_ROOT) not in sys.path:
    sys.path.insert(0, str(_LBC_ROOT))

from lbc_bev import LBCRenderer
from lbc_bev.morai_adapters import (
    TrafficLightStateCache,
    TurnSignalTracker,
    load_intscn_traffic_light_map,
    load_intersection_traffic_groups,
    load_traffic_light_catalog,
    object_status_list_to_states,
    setup_morai_event_cmd_turn_signal_polling,
)
from lbc_bev.morai_vehicle import (
    IONIQ5,
    LBC_TRAINING_PIXELS_AHEAD,
    MORAI_EGO_PIXELS_AHEAD,
)
from lbc_bev.spec import (
    BEV_OBJECT_FILTER_RADIUS_M,
    CROP_SIZE,
    EGO_PIXEL_COL,
    EGO_PIXEL_ROW,
    EGO_VIS_COLOR,
    MAP_SIZE,
    OBJECT_BEV_MARGIN_PX,
    PIXELS_AHEAD_VEHICLE,
)
from lbc_bev.static_layers import (
    compute_bev_affine_matrix,
    filter_dynamic_states_to_bev,
    world_points_to_bev_pixels,
)
from lbc_bev.ws_root import asmc_ws_root, lbc_data_root

PUBLISH_HZ = 10.0
DEFAULT_TL_TOPICS = (
    "/GetTrafficLightStatus",
    "/IntscnTL_topic",
    "/TrafficLight_status",
)


class LBCBEVVisualizer:
    def __init__(self):
        rospy.init_node("lbc_bev_visualizer", anonymous=False)
        self._debug = bool(rospy.get_param("~debug", True))
        self._save_snap = bool(rospy.get_param("~save_snapshots", False))
        self._use_imshow = bool(rospy.get_param("~use_imshow", True))
        self._draw_ego_imshow = bool(rospy.get_param("~draw_ego_imshow", True))
        self._object_bev_margin_px = int(
            rospy.get_param("~object_bev_margin_px", OBJECT_BEV_MARGIN_PX)
        )
        self._publish_images = bool(rospy.get_param("~publish_images", True))
        self._skip_overrun = bool(rospy.get_param("~skip_overrun_frames", True))
        self._filter_radius_m = float(
            rospy.get_param("~filter_objects_radius_m", BEV_OBJECT_FILTER_RADIUS_M)
        )
        self._max_vehicles = int(rospy.get_param("~max_vehicles", 120))
        self._max_pedestrians = int(rospy.get_param("~max_pedestrians", 80))
        self._pixels_ahead = int(rospy.get_param("~pixels_ahead", MORAI_EGO_PIXELS_AHEAD))
        self._frame_id = rospy.get_param("~frame_id", "map")
        self._pub_count = 0
        self._tl_topic_used = "(none)"
        self._snap_dir = Path(
            os.path.expanduser(
                rospy.get_param(
                    "~snapshot_dir",
                    str(lbc_data_root() / "diag" / "live"),
                )
            )
        )

        rospy.loginfo("[LBCBEV-Viz] ROS_MASTER_URI=%s", os.environ.get("ROS_MASTER_URI", "(unset)"))

        ws_root = Path(
            os.path.expanduser(
                rospy.get_param("~aim_ws_root", str(asmc_ws_root()))
            )
        )
        rospy.loginfo("[LBCBEV-Viz] Loading HD map (~3s)...")
        t0 = time.time()
        self.renderer = LBCRenderer(ws_root, pixels_ahead=self._pixels_ahead)
        rospy.loginfo("[LBCBEV-Viz] LBCRenderer ready in %.2fs", time.time() - t0)
        self._tl_catalog = load_traffic_light_catalog(
            self.renderer.tl_json, vehicle_only=True
        )
        synced_path = ws_root / "R_KR_PG_KATRI/synced_traffic_light_set.json"
        self._intscn_catalog = load_intscn_traffic_light_map(
            synced_path, vehicle_signal_ids=set(self._tl_catalog.keys())
        )
        inttl_keys = [k for k in self._intscn_catalog if k.startswith("IntTL")]
        rospy.loginfo(
            "[LBCBEV-Viz] TL=%d IntTL*=%d pixels_ahead=%d | Ioniq5 %.2fx%.2fm (rear axle ego)",
            len(self._tl_catalog),
            len(inttl_keys),
            self._pixels_ahead,
            IONIQ5.length_m,
            IONIQ5.width_m,
        )
        self._bridge = CvBridge()
        self._imshow_ready = False
        self._frame_busy = False
        self._skipped_frames = 0

        self.ego_pose = None
        self.latest_obj = None
        self._tl_cache = TrafficLightStateCache()
        self._tl_cache.set_traffic_groups(
            load_intersection_traffic_groups(
                synced_path, vehicle_signal_ids=set(self._tl_catalog.keys())
            )
        )
        self._turn_signal = TurnSignalTracker()
        self._lamps_topic_used = "(none)"
        self.lock = threading.Lock()

        self.pub_full = rospy.Publisher("/lbc_bev/image_full", Image, queue_size=1)
        self.pub_crop = rospy.Publisher("/lbc_bev/image_cropped", Image, queue_size=1)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self._obj_cb, queue_size=1)
        self._subscribe_traffic_light_optional()
        self._subscribe_intersection_status_optional()
        self._setup_morai_event_cmd_lamps()

        rospy.Timer(rospy.Duration(1.0 / PUBLISH_HZ), self._timer_cb)
        if self._debug:
            rospy.Timer(rospy.Duration(2.0), self._debug_timer_cb)
        if self._use_imshow:
            rospy.loginfo(
                "[LBCBEV-Viz] OpenCV imshow ON (%dx%d + %dx%d side-by-side, q=quit)",
                MAP_SIZE,
                MAP_SIZE,
                CROP_SIZE,
                CROP_SIZE,
            )
        rospy.loginfo(
            "[LBCBEV-Viz] perf: publish=%s skip_overrun=%s filter_r=%.0fm "
            "bev_margin=%dpx max_veh=%d",
            self._publish_images,
            self._skip_overrun,
            self._filter_radius_m,
            self._object_bev_margin_px,
            self._max_vehicles,
        )
        rospy.loginfo(
            "[LBCBEV-Viz] anchor=(%d,%d) LBC_train_ahead=%d px",
            EGO_PIXEL_COL,
            EGO_PIXEL_ROW,
            LBC_TRAINING_PIXELS_AHEAD,
        )
        rospy.spin()
        if self._use_imshow:
            cv2.destroyAllWindows()

    def _subscribe_traffic_light_optional(self):
        param_topic = rospy.get_param("~traffic_light_topic", "")
        topics = [param_topic] if param_topic else list(DEFAULT_TL_TOPICS)
        try:
            from morai_msgs.msg import (
                GetTrafficLightStatus,
                IntscnTL,
                MoraiTLInfo,
                TrafficLight,
            )

            tl_msg_types = (
                GetTrafficLightStatus,
                IntscnTL,
                MoraiTLInfo,
                TrafficLight,
            )
            subscribed: list[str] = []
            for topic in topics:
                if not topic:
                    continue
                for msg_type in tl_msg_types:
                    try:
                        rospy.Subscriber(topic, msg_type, self._tl_cb, queue_size=1)
                        subscribed.append(f"{topic} ({msg_type.__name__})")
                    except (rospy.ROSException, Exception):
                        continue
            if subscribed:
                self._tl_topic_used = ", ".join(subscribed)
                rospy.loginfo("[LBCBEV-Viz] TL subscribers: %s", self._tl_topic_used)
                for topic in topics:
                    if not topic:
                        continue
                    for msg_type in (GetTrafficLightStatus, IntscnTL):
                        try:
                            boot = rospy.wait_for_message(topic, msg_type, timeout=1.0)
                            self._tl_cache.ingest(boot)
                            rospy.loginfo(
                                "[LBCBEV-Viz] TL bootstrap from %s (%s)",
                                topic,
                                msg_type.__name__,
                            )
                            break
                        except rospy.ROSException:
                            continue
                return
        except Exception as exc:
            rospy.logwarn("[LBCBEV-Viz] TL subscribe failed: %s", exc)

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

    def _subscribe_intersection_status_optional(self):
        topics = [rospy.get_param("~intersection_status_topic", "/InsnStatus")]
        try:
            from morai_msgs.msg import IntersectionStatus

            for topic in topics:
                if not topic:
                    continue
                try:
                    rospy.Subscriber(
                        topic, IntersectionStatus, self._tl_cb, queue_size=1
                    )
                    rospy.loginfo("[LBCBEV-Viz] IntersectionStatus: %s", topic)
                    try:
                        boot = rospy.wait_for_message(
                            topic, IntersectionStatus, timeout=1.0
                        )
                        self._tl_cache.ingest(boot)
                    except rospy.ROSException:
                        pass
                    return
                except (rospy.ROSException, Exception):
                    continue
        except Exception as exc:
            rospy.logwarn("[LBCBEV-Viz] IntersectionStatus subscribe skipped: %s", exc)

    def _setup_morai_event_cmd_lamps(self) -> None:
        """Poll /Service_MoraiEventCmd only — do not use /Lamps_topic (empty on KATRI)."""
        proxy = setup_morai_event_cmd_turn_signal_polling(
            self._turn_signal, self.lock, log_tag="[LBCBEV-Viz]"
        )
        if proxy is not None:
            svc = str(rospy.get_param("~morai_event_cmd_service", "/Service_MoraiEventCmd"))
            self._lamps_topic_used = f"{svc} (MoraiEventCmd poll)"
        else:
            self._lamps_topic_used = "(MoraiEventCmd unavailable — straight TL only)"

    def _polygon_bev_pixels(self, poly_world: np.ndarray, out: dict) -> np.ndarray:
        baked = self.renderer.baked_maps
        pts = world_points_to_bev_pixels(poly_world, out["bev_affine"], baked)
        return pts.reshape(-1, 1, 2)

    def _draw_ioniq_ego_on_rgb(self, rgb: np.ndarray, ego: dict, out: dict) -> None:
        """Ioniq 5 footprint on an existing RGB buffer (no full-frame copy)."""
        poly_w = IONIQ5.footprint_polygon_world(ego["x"], ego["y"], ego["yaw_deg"])
        arr = self._polygon_bev_pixels(poly_w, out)
        cv2.fillPoly(rgb, [arr], EGO_VIS_COLOR)

    def _compose_imshow_panel(
        self,
        vis_full: np.ndarray,
        vis_crop: np.ndarray,
        *,
        ego: Optional[dict] = None,
        out: Optional[dict] = None,
    ) -> np.ndarray:
        """512×320 native: left 320 (optional ego) | right 192 LBC-spec (no ego)."""
        panel_h = MAP_SIZE
        panel_w = MAP_SIZE + CROP_SIZE
        panel = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
        panel[:, :MAP_SIZE] = vis_full
        if ego is not None and out is not None:
            self._draw_ioniq_ego_on_rgb(panel[:, :MAP_SIZE], ego, out)
        y0 = (MAP_SIZE - CROP_SIZE) // 2
        panel[y0 : y0 + CROP_SIZE, MAP_SIZE:] = vis_crop
        return panel

    def _show_imshow(
        self,
        vis_full: np.ndarray,
        vis_crop: np.ndarray,
        *,
        ego: Optional[dict] = None,
        out: Optional[dict] = None,
    ) -> bool:
        panel = self._compose_imshow_panel(
            vis_full, vis_crop, ego=ego, out=out
        )
        bgr = cv2.cvtColor(panel, cv2.COLOR_RGB2BGR)
        if not self._imshow_ready:
            cv2.namedWindow("LBC BEV", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("LBC BEV", MAP_SIZE + CROP_SIZE, MAP_SIZE)
            self._imshow_ready = True
        cv2.imshow("LBC BEV", bgr)
        key = cv2.waitKey(1) & 0xFF
        return key != ord("q")

    def _timer_cb(self, _event):
        try:
            self._timer_cb_impl(_event)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "[LBCBEV-Viz] frame error: %s", exc)

    def _timer_cb_impl(self, _event):
        if self._skip_overrun and self._frame_busy:
            self._skipped_frames += 1
            return
        self._frame_busy = True
        try:
            self._timer_cb_render(_event)
        finally:
            self._frame_busy = False

    def _timer_cb_render(self, _event):
        with self.lock:
            if self.ego_pose is None:
                return
            ego = dict(self.ego_pose)
            obj = self.latest_obj
            turn_mode = self._turn_signal.mode

        baked = self.renderer.baked_maps
        bev_affine = compute_bev_affine_matrix(
            baked,
            ego["x"],
            ego["y"],
            ego["yaw_deg"],
            pixels_ahead=self._pixels_ahead,
        )
        vehicles, pedestrians = object_status_list_to_states(
            obj,
            ego_xy=(ego["x"], ego["y"]),
            filter_radius_m=self._filter_radius_m,
            max_vehicles=self._max_vehicles,
            max_pedestrians=self._max_pedestrians,
        )
        vehicles, pedestrians = filter_dynamic_states_to_bev(
            vehicles,
            pedestrians,
            bev_affine,
            baked,
            margin_px=self._object_bev_margin_px,
        )

        traffic_lights = self._tl_cache.build_traffic_lights(
            self._tl_catalog,
            self._intscn_catalog,
            turn_mode=turn_mode,
            bev_affine=bev_affine,
            baked=baked,
            filter_to_bev=True,
            ego_xy=(ego["x"], ego["y"]),
            ego_yaw_deg=ego["yaw_deg"],
            ego_facing_only=True,
            require_confident_approach=True,
        )

        out = self.renderer.render(
            ego["x"],
            ego["y"],
            ego["yaw_deg"],
            vehicles=vehicles,
            pedestrians=pedestrians,
            traffic_lights=traffic_lights,
        )

        stamp = rospy.Time.now()
        vis_full = out.get("visualization")
        vis_crop = out.get("visualization_cropped")

        if self._publish_images:
            if vis_full is not None:
                pub_full = vis_full
                if self._draw_ego_imshow:
                    pub_full = vis_full.copy()
                    self._draw_ioniq_ego_on_rgb(pub_full, ego, out)
                self._publish_image(self.pub_full, pub_full, stamp)
            if vis_crop is not None:
                self._publish_image(self.pub_crop, vis_crop, stamp)

        if self._use_imshow and vis_full is not None and vis_crop is not None:
            draw_ego = self._draw_ego_imshow
            if not self._show_imshow(
                vis_full,
                vis_crop,
                ego=ego if draw_ego else None,
                out=out if draw_ego else None,
            ):
                rospy.signal_shutdown("imshow quit")

        self._pub_count += 1
        if self._save_snap and self._pub_count <= 10:
            self._snap_dir.mkdir(parents=True, exist_ok=True)
            if vis_full is not None and vis_crop is not None:
                panel = self._compose_imshow_panel(
                    vis_full,
                    vis_crop,
                    ego=ego if self._draw_ego_imshow else None,
                    out=out if self._draw_ego_imshow else None,
                )
                p = self._snap_dir / f"bev_panel_{self._pub_count:04d}.png"
                cv2.imwrite(str(p), cv2.cvtColor(panel, cv2.COLOR_RGB2BGR))
                rospy.loginfo("[LBCBEV-Viz] saved %s", p)

        if self._debug and self._pub_count <= 5:
            bv = out["birdview"]
            nz = [int(np.count_nonzero(bv[:, :, i])) for i in range(bv.shape[2])]
            tl_colors = {}
            for t in traffic_lights:
                tl_colors[t.state] = tl_colors.get(t.state, 0) + 1
            rospy.loginfo(
                "[LBCBEV-Viz] #%d veh=%d ped=%d tl=%d turn=%s lamps=%s nz=%s tl_by_color=%s",
                self._pub_count,
                len(vehicles),
                len(pedestrians),
                len(traffic_lights),
                turn_mode,
                self._lamps_topic_used,
                nz,
                tl_colors,
            )

    def _debug_timer_cb(self, _event):
        n_pub = sum(1 for p in (self.pub_full, self.pub_crop) if p.get_num_connections() > 0)
        rospy.loginfo(
            "[LBCBEV-Viz] frames=%d skipped=%d image_subscribers=%d",
            self._pub_count,
            self._skipped_frames,
            n_pub,
        )

    def _publish_image(self, pub, rgb, stamp):
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        msg = self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        pub.publish(msg)


def main():
    try:
        LBCBEVVisualizer()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
