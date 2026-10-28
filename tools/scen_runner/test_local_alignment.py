#!/usr/bin/env python3
"""LOCAL-2A ROS arrival/identity regression; private master, no control output."""
import copy
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    import rosgraph
    import rospy
    from asmc_msgs.msg import (
        CameraGroundProjection,
        LaneBoundaryFamily,
        LaneBoundaryFamilyMember,
        LidarTrackingObservation,
        LocalTrajectory,
    )
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import Image, Imu
    from morai_msgs.msg import EgoVehicleStatus
    from std_msgs.msg import String, UInt8, Bool

    processes = []
    logs = []
    with tempfile.TemporaryDirectory(prefix="local-alignment-") as folder:

        def start(args, name):
            log = open(Path(folder) / name, "w")
            logs.append(log)
            child = subprocess.Popen(
                args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            processes.append(child)
            return child

        try:
            start(["roscore", "-p", str(port)], "master.log")
            master = rosgraph.Master("/local_alignment_test")
            for _ in range(60):
                try:
                    master.getPid()
                    break
                except Exception:
                    time.sleep(0.1)
            rospy.init_node("local_alignment_test", disable_signals=True)
            node = start(
                [
                    "rosrun",
                    "asmc_planner",
                    "asmc_local_planner_node",
                    "__name:=local_alignment_planner",
                    "_speed_callerid:=/local_alignment_test",
                    "_run_id:=unit-local2a",
                    "_active_in_jamming_only:=true",
                ],
                "planner.log",
            )
            preparation = rospy.Publisher("/planning/mode/local_prepare", Bool, queue_size=1)
            mode = rospy.Publisher("/planning/mode", UInt8, queue_size=1, latch=True)
            pub = {}
            for name, topic, kind in [
                ("speed", "/Ego_topic", EgoVehicleStatus),
                ("imu", "/imu", Imu),
                (
                    "context",
                    "/experimental/shadow/semantic_space_trial/context",
                    CameraGroundProjection,
                ),
                (
                    "probability",
                    "/experimental/shadow/semantic_space_trial/drivable_probability",
                    Image,
                ),
                (
                    "coverage",
                    "/experimental/shadow/semantic_space_trial/coverage",
                    Image,
                ),
                ("family", "/perception/lane/boundary_family", LaneBoundaryFamily),
                ("lidar", "/perception/lidar/tracks_local", LidarTrackingObservation),
            ]:
                pub[name] = rospy.Publisher(topic, kind, queue_size=16)
            rows, plans = [], []
            subscriptions = [
                rospy.Subscriber(
                    "/planning/local/trace",
                    String,
                    lambda m: rows.append(json.loads(m.data)),
                    queue_size=100,
                ),
                rospy.Subscriber(
                    "/planning/local/trajectory",
                    LocalTrajectory,
                    plans.append,
                    queue_size=100,
                ),
            ]
            time.sleep(1.0)
            mode.publish(UInt8(0))
            time.sleep(.3)
            assert rows and all(r["reason"] == "inactive_normal_mode" for r in rows)
            mode.publish(UInt8(1))
            time.sleep(.1)
            sequence = 0

            prepare_mode = False
            def feed(seconds, speed, status=1, session="camera-a", duplicate=False, camera_age=.20, folded_role=0, family_valid=True):
                nonlocal sequence
                rows.clear()
                plans.clear()
                begin = time.monotonic()
                next_scene = 0
                while time.monotonic() - begin < seconds:
                    if prepare_mode:
                        preparation.publish(Bool(True))
                    now = rospy.Time.now()
                    v = EgoVehicleStatus()
                    v.header.stamp = now
                    v.velocity.x = speed
                    pub["speed"].publish(v)
                    imu = Imu()
                    imu.header.stamp = now
                    imu.orientation.w = 1.0
                    pub["imu"].publish(imu)
                    if time.monotonic() >= next_scene:
                        next_scene = time.monotonic() + 0.1
                        sequence += 1
                        stamp = now - rospy.Duration(camera_age)
                        ctx = CameraGroundProjection()
                        ctx.header.stamp = stamp
                        ctx.producer_session_id = session
                        ctx.projection_session_id = "projection"
                        ctx.config_id = "fixture"
                        ctx.imu_corrected = True
                        ctx.views = ["front", "left", "right"]
                        ctx.plane_coefficients = [0.0, 0.0, -0.36] * 3
                        family = LaneBoundaryFamily()
                        family.header.stamp = stamp
                        family.header.frame_id = "base_link"
                        family.valid = family_valid
                        family.operating_domain = "shadow_two_way_v1"
                        family.config_sha256 = "fixture"
                        family.algorithm_id = (
                            "twinlite_v6_shadow_boundary_family_candidate_v3"
                        )
                        family.representation = (
                            "shared_parametric_bspline_normal_offsets_v2"
                        )
                        for role, y in ((1, 5.25), (2, 1.75), (3, -1.75)):
                            line = LaneBoundaryFamilyMember()
                            line.role = role
                            line.polyline = [
                                Point(x * 0.5, y, -0.36) for x in range(41)
                            ]
                            line.support_state = [1] * 41
                            if role == folded_role:
                                line.polyline[10].x = line.polyline[9].x - .2
                            family.boundaries.append(line)
                        if not family_valid:
                            family.boundaries=[]
                            family.status_reason='too_few_observed_roles'
                        raster = Image()
                        raster.header.stamp = stamp
                        raster.header.frame_id = "base_link"
                        raster.height = 250
                        raster.width = 160
                        raster.step = 160
                        raster.encoding = "mono8"
                        raster.data = bytes([255]) * 40000
                        pub["context"].publish(ctx)
                        pub["family"].publish(family)
                        pub["probability"].publish(raster)
                        pub["coverage"].publish(raster)
                        lidar = LidarTrackingObservation()
                        o = lidar.observation
                        o.producer_session_id = "lidar-a"
                        o.config_id = "fixture"
                        o.calibration_id = "fixture"
                        o.scan_sequence = sequence
                        o.objects.header.stamp = now - rospy.Duration(0.02)
                        o.objects.header.frame_id = "base_link"
                        o.objects.status = status
                        pub["lidar"].publish(lidar)
                        if duplicate:
                            pub["lidar"].publish(lidar)
                    time.sleep(0.02)
                return list(rows), list(plans)

            # OBSERVED_EMPTY is obtained from the message, never guessed numeric.
            empty = LidarTrackingObservation().observation.objects.OBSERVED_EMPTY
            invalid = LidarTrackingObservation().observation.objects.INPUT_INVALID
            for speed in (10.0 / 3.6, 25.0 / 3.6, -1.0):
                result, messages = feed(2.0, speed, status=empty, duplicate=True)
                observed = [r for r in result if r.get("scene_valid")]
                assert len(observed) >= 6, (speed, [r["reason"] for r in result])
                for r in observed:
                    ids = r["inputs"]
                    # ROS callbacks are independent: the newest LiDAR packet
                    # can still be in flight while the camera bundle is ready.
                    # Check the actual chosen stamps, never assume an atomic
                    # four-topic publication or borrow a future LiDAR sample.
                    expected_skew=(ids["lidar_stamp_ns"]-r["input_join"]["selected_stamp_ns"])*1e-6
                    assert abs(ids["lidar_minus_family_source_ms"]-expected_skew)<.001, ids
                    assert abs(1000*(ids["camera_age_s"]-ids["lidar_age_s"])-expected_skew)<.001, ids
                    assert ids["camera_age_s"] < 0.35 and ids["lidar_age_s"] < 0.15
                    assert (
                        r["input_join"]["selection_policy"]
                        == "independent_latest_to_plan"
                    )
                    # Constant speed part after the initial transition settles.
                assert any(r["inputs"]["lidar_minus_family_source_ms"]>150 for r in observed), (
                    speed, [r["inputs"] for r in observed])
                tail = observed[-1]
                shift = tail["scene"]["source_in_plan"][0]
                assert abs(shift + speed * tail["inputs"]["camera_age_s"]) < 0.06, (
                    speed,
                    shift,
                    tail["inputs"],
                )
                for m in messages:
                    if m.scene_stamp.to_nsec():
                        assert m.header.stamp == m.ego_at_plan.header.stamp
                        assert m.header.stamp > m.scene_stamp
                if speed < 0:
                    assert tail["reason"] == "reverse_control_not_supported"
                    assert len({r["frame"] for r in observed}) == 1
                print(
                    "independent latest / signed motion", speed, "PASS", len(observed)
                )
            result, _ = feed(1.0, 1.0, status=invalid)
            assert any(r["reason"] == "local_objects_invalid" for r in result)
            assert not any(r.get("scene_valid") for r in result[-3:])
            print("latest INVALID never revives older valid scan PASS")
            result, _ = feed(1.2, 1., status=empty, folded_role=1)
            assert any(r.get('valid') for r in result), [r['reason'] for r in result]
            result, _ = feed(.7, 1., status=empty, folded_role=3)
            assert result[-1]['reason']=='family_not_monotonic'
            print('Unused opposite boundary cannot invalidate home; required boundary remains strict PASS')
            result, _ = feed(.7, 1., status=empty, family_valid=False)
            assert result[-1]['reason']=='family_plane_invalid'
            print('Unvalidated semantic-reference candidate remains OFF in default runtime PASS')
            result, _ = feed(1.5, 1.0, status=empty, camera_age=.275)
            late = [r for r in result if r.get('valid')]
            assert late, [r['reason'] for r in result]
            assert all(r['frame_budget_ms'] <= 30 for r in late), late[-1]
            print('Late complete bundles use a shorter compute budget;350ms freshness unchanged PASS')
            result, _ = feed(.6, 1.0, status=empty, camera_age=.36)
            assert not any(r.get('valid') for r in result[-3:])
            result, _ = feed(1.5, 1.0, status=empty, session="camera-b")
            valid = [r for r in result if r.get("scene_valid")]
            assert valid and valid[-1]["inputs"]["camera_session"] == "camera-b"
            result, _ = feed(0.6, 1.0, status=empty, session="camera-a")
            assert not any(
                r.get("scene_valid") and r["inputs"].get("camera_session") == "camera-a"
                for r in result
            )
            print("retired session cannot return PASS")
            mode.publish(UInt8(0))
            time.sleep(.2)
            result, _ = feed(.6, 1.0, status=empty, session="camera-b")
            assert result and all(r["reason"] == "inactive_normal_mode" for r in result)
            prepare_mode = True
            result, _ = feed(1.0, 1.0, status=empty, session="camera-b")
            assert any(r.get("scene_valid") and r.get("phase") == "prepare" for r in result)
            before_frames = {r['frame'] for r in result if r.get('scene_valid')}
            mode.publish(UInt8(1))
            time.sleep(.05)
            result, _ = feed(1.0, 1.0, status=empty, session="camera-b")
            assert any(r.get('phase') == 'active' and r['frame'] in before_frames for r in result if r.get('scene_valid'))
            assert any(r.get("scene_valid") for r in result)
            print("NORMAL idle / bounded preparation / same-frame ACTIVE entry PASS")
            assert node.poll() is None
            assert not dict(master.getSystemState()[0]).get("/ctrl_cmd")
            assert not dict(master.getSystemState()[0]).get("/ctrl_cmd/jamming")
            print("LOCAL-2A alignment boundary PASS; no command publisher")
        except Exception:
            print((Path(folder) / "planner.log").read_text()[-4000:])
            raise
        finally:
            for child in reversed(processes):
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGTERM)
                        child.wait(timeout=5)
            for log in logs:
                log.close()


if __name__ == "__main__":
    main()
