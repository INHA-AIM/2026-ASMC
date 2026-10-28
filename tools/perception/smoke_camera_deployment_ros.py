#!/usr/bin/env python3
"""Check the deployed three-view detector and canonical LiDAR receiver privately.

Uses saved RGB and synthetic LiDAR/IMU/CAN; no bridge, vehicle control or MORAI RPCs.
Run with ROS Python after sourcing the root workspace only. This tests transport,
identity and receiver admission, not real camera/LiDAR alignment or driving.
"""
import argparse
from collections import Counter, deque
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=11528)
    args = parser.parse_args()
    if not 11400 <= args.port <= 11999:
        raise ValueError('use a private test master port in 11400..11999')
    args.output.mkdir(parents=True, exist_ok=False)
    uri = 'http://127.0.0.1:{}'.format(args.port)
    master = xmlrpc.client.ServerProxy(uri)
    try:
        master.getPid('/deployment_smoke_preflight')
    except OSError:
        pass
    else:
        raise RuntimeError('private test master already in use')
    os.environ['ROS_MASTER_URI'] = uri
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_LOG_DIR'] = str(args.output/'roslogs')
    processes, parents, handles = [], [], []
    report = {}
    try:
        log = (args.output/'roscore.log').open('w'); handles.append(log)
        processes.append(subprocess.Popen(['roscore', '-p', str(args.port)], stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic()+20
        while True:
            try:
                master.getPid('/deployment_smoke_wait'); break
            except OSError:
                if time.monotonic() > deadline: raise RuntimeError('master start timeout')
                time.sleep(.1)
        import cv2
        import rospy
        import roslaunch
        import rospkg
        from sensor_msgs.msg import CompressedImage, Imu, PointCloud2, PointField
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header
        from morai_msgs.msg import EgoVehicleStatus
        from diagnostic_msgs.msg import DiagnosticArray
        from asmc_msgs.msg import CameraObjectBoxArray, CameraObjectObservation, InferenceStatus
        from asmc_camera.detection.mission import CLASSES_NO_CONE, group_of
        rospy.init_node('deployment_smoke', disable_signals=True)
        package = Path(rospkg.RosPack().get_path('asmc_camera'))
        lidar_package = Path(rospkg.RosPack().get_path('asmc_lidar'))
        rows = [json.loads(line) for line in args.manifest.read_text().splitlines()]
        sources = {}
        for view, label in [('front', 'stroller'), ('left', 'car'), ('right', 'truck')]:
            candidates = [r for r in rows if r.get('view') == view and any(
                a.get('training_class', a.get('canonical')) == label and a.get('supervision') == 'positive'
                for a in r['annotations'])]
            def largest(row):
                return max((a['xyxy'][2]-a['xyxy'][0])*(a['xyxy'][3]-a['xyxy'][1])
                           for a in row['annotations'] if a.get('training_class', a.get('canonical')) == label)
            sources[view] = max(candidates, key=largest)
        payloads = {v: cv2.imencode('.jpg', cv2.imread(r['image']), [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
                    for v, r in sources.items()}
        paths = dict(front='/image_jpeg/compressed', left='/cam_left/image_jpeg/compressed', right='/cam_right/image_jpeg/compressed')
        pubs = {v: rospy.Publisher(topic, CompressedImage, queue_size=1) for v, topic in paths.items()}
        imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
        can_pub = rospy.Publisher('/Ego_topic', EgoVehicleStatus, queue_size=1)
        cloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=1)
        # A ground plane and one box suffice to execute the receiver/fusion path.
        points = [(x*.3, y*.3, -1.86) for x in range(-10, 70) for y in range(-35, 36)]
        points += [(x*.2, y*.2, z*.2-1.86) for x in range(40, 61)
                   for y in range(-5, 6) for z in range(1, 9)
                   if x in (40, 60) or abs(y) == 5 or z == 8]
        fields = [PointField(name, 4*i, PointField.FLOAT32, 1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud = point_cloud2.create_cloud(Header(frame_id='velodyne'), fields,
                                         [(x, y, z, 1.0) for x, y, z in points])
        sent, camera, wire = set(), {}, {}
        counts, labels, bad = Counter(), Counter(), Counter()
        statuses, diagnostics = deque(maxlen=100), deque(maxlen=100)
        lidar_diagnostics = deque(maxlen=100)

        def values(message):
            return [[b.source_class_name, b.class_name, b.confidence, b.confidence_valid,
                     b.xmin, b.ymin, b.xmax, b.ymax] for b in message.boxes]

        def boxes(message):
            if not message.valid: return
            view = message.header.frame_id[len('camera_'):]
            stamp = message.header.stamp.to_nsec()
            counts[view] += 1
            if stamp not in sent: bad['unknown_stamp'] += 1
            if view not in sources: bad['unknown_view'] += 1; return
            if (message.image_width, message.image_height) != (sources[view]['width'], sources[view]['height']):
                bad['original_image_shape'] += 1
            for box in message.boxes:
                labels[box.source_class_name] += 1
                if box.source_class_name not in CLASSES_NO_CONE or box.class_name != group_of(box.source_class_name):
                    bad['class_name_or_group'] += 1
                if not (0 <= box.xmin < box.xmax <= message.image_width and 0 <= box.ymin < box.ymax <= message.image_height):
                    bad['bbox_coordinates'] += 1
            camera[(view, stamp)] = values(message)

        def observation(message):
            d = message.detections
            if not d.valid: return
            if not all((message.producer_session_id, message.calibration_id, message.model_id, message.config_id)):
                bad['missing_observation_identity'] += 1
            wire[(d.header.frame_id[len('camera_'):], d.header.stamp.to_nsec())] = values(d)

        subscriptions = [
            rospy.Subscriber('/perception/camera/objects', CameraObjectBoxArray, boxes, queue_size=100),
            rospy.Subscriber('/perception/camera/objects/observations', CameraObjectObservation, observation, queue_size=100),
            rospy.Subscriber('/test/deployment/diagnostics', DiagnosticArray,
                lambda m: lidar_diagnostics.extend({v.key: v.value for v in s.values} for s in m.status), queue_size=10),
            rospy.Subscriber('/perception/camera/objects/status', InferenceStatus,
                lambda m: statuses.append(dict(prepared=m.prepared, mode=m.mode, reason=m.reason)), queue_size=10),
            rospy.Subscriber('/perception/camera/objects/diagnostics', DiagnosticArray,
                lambda m: diagnostics.extend({v.key: v.value for v in s.values} for s in m.status), queue_size=10)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = roslaunch.parent.ROSLaunchParent(uuid, [(str(package/'launch/camera.launch'), ['models:=objects'])])
        parent.start(); parents.append(parent)
        lidar = roslaunch.parent.ROSLaunchParent(uuid, [(str(lidar_package/'launch/lidar.launch'),
            ['namespace:=/test/deployment', 'experimental_tunnel_filter:=false'])])
        lidar.start(); parents.append(lidar)
        deadline = time.monotonic()+60
        while not all(pub.get_num_connections() for pub in pubs.values()):
            if time.monotonic() > deadline: raise RuntimeError('detector input subscription timeout')
            time.sleep(.1)
        end = time.monotonic()+15
        while time.monotonic() < end:
            stamp = rospy.Time.now(); sent.add(stamp.to_nsec())
            imu = Imu(); imu.header.stamp = stamp; imu.orientation.w = 1.0
            imu_pub.publish(imu)
            can = EgoVehicleStatus(); can.header.stamp = stamp; can_pub.publish(can)
            cloud.header.stamp = stamp; cloud_pub.publish(cloud)
            for view, pub in pubs.items():
                message = CompressedImage()
                message.header.stamp = stamp; message.header.frame_id = 'camera_'+view
                message.format = 'jpeg'; message.data = payloads[view]; pub.publish(message)
            time.sleep(1/15)
        time.sleep(.3)
        for key, value in wire.items():
            if camera.get(key) != value: bad['observation_payload_mismatch'] += 1
        publishers = {topic: nodes for topic, nodes in master.getSystemState('/deployment_smoke_graph')[2][0]
                      if topic in ('/perception/camera/objects', '/perception/camera/objects/observations', '/YOLO/BBox')}
        prepared = any(s['prepared'] for s in statuses)
        received = Counter(view for view, _ in wire)
        admitted = {v: any(int(d.get('fusion_view_camera_'+v+'_candidates', 0)) > 0
                          for d in lidar_diagnostics) for v in sources}
        all_views = any(d.get('fusion_views') == 'all' for d in lidar_diagnostics)
        passed = (all(counts[v] >= 30 for v in sources) and len(wire) >= 30 and prepared and not bad
                  and all(received[v] >= 30 for v in sources) and all(admitted.values()) and all_views
                  and labels['stroller'] > 0 and '/YOLO/BBox' not in publishers
                  and len(publishers) == 2 and all(len(nodes) == 1 for nodes in publishers.values()))
        report = dict(passed=passed, master_uri=uri, inputs=len(sent), received=dict(counts),
                      typed_observations=dict(received), lidar_admitted=admitted, fusion_all_views=all_views,
                      labels=dict(labels), errors=dict(bad), prepared_seen=prepared,
                      publishers=publishers, diagnostics=list(diagnostics), status=list(statuses),
                      lidar_diagnostics=list(lidar_diagnostics),
                      sources={v: r['id'] for v, r in sources.items()},
                      scope='real three-view ROS/GPU + canonical LiDAR receiver, saved RGB/synthetic cloud; not spatial alignment or live driving')
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('diagnostics', 'lidar_diagnostics', 'status')}), flush=True)
        if not passed: raise RuntimeError('ROS deployment smoke failed; inspect report/logs')
    finally:
        for parent in reversed(parents): parent.shutdown()
        for process in reversed(processes):
            process.terminate()
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        for handle in handles: handle.close()


if __name__ == '__main__':
    main()
