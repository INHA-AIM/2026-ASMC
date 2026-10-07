#!/usr/bin/env python3
"""Make private replay bags with original sensor headers and frozen B0 boxes."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import tarfile

import rosbag
import rospy
import yaml
from Msg.msg import Yolo_bbox, Yolo_bbox_arr
from b1a_protocol import frozen_events, gps_action

TOPICS = ['/gps', '/gps/fix', '/imu', '/Ego_topic', '/velodyne_points', '/image_jpeg/compressed']


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(4194304), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    config = yaml.safe_load((root/'config/experiments/camera_lidar_b1a_v1.yaml').read_text())
    archive = root/config['frozen_yolo']['archive']
    manifest = dict(schema_version=1, config_sha256=digest(root/'config/experiments/camera_lidar_b1a_v1.yaml'),
                    original_archive_sha256=digest(archive), fresh_inference=False, bags={})
    with tarfile.open(archive) as tar:
        for name, info in config['bags'].items():
            member = next(m for m in tar.getmembers() if m.name.endswith(info['b0_trial']+'/observations.json'))
            observation = json.load(tar.extractfile(member))
            events = frozen_events(observation)
            event_path = args.output/(name+'-yolo-events.json')
            event_path.write_text(json.dumps(events, indent=2)+'\n')
            with rosbag.Bag(info['path']) as source:
                start = source.get_start_time()
                manifest['bags'][name] = dict(source=info['path'], source_sha256=digest(info['path']),
                    source_start_sec=start, source_duration_sec=source.get_end_time()-start,
                    source_topics=sorted(source.get_type_and_topic_info().topics),
                    yolo_events=len(events), yolo_event_sha256=digest(event_path), variants={})
                # Message data are immutable; reuse the same frozen sensor input in all variants.
                messages = list(source.read_messages(topics=TOPICS))
            for mode in config['gps_modes']:
                path = args.output/(name+'-'+mode+'.bag')
                counts, injected, omitted, event_index = {}, 0, 0, 0
                def write(out, topic, msg, stamp):
                    out.write(topic, msg, stamp)
                    counts[topic] = counts.get(topic, 0)+1
                with rosbag.Bag(str(path), 'w', compression=rosbag.Compression.LZ4) as out:
                    for topic, msg, record in messages:
                        while event_index < len(events) and events[event_index]['available_ns'] <= record.to_nsec():
                            event = events[event_index]
                            result = Yolo_bbox_arr()
                            result.header.stamp = rospy.Time.from_sec(event['stamp_ns']/1e9)
                            # Preserve nanoseconds exactly; float conversion above is only initialization.
                            result.header.stamp.secs, result.header.stamp.nsecs = divmod(event['stamp_ns'], 1000000000)
                            result.header.frame_id = event['frame_id']
                            for box in event['boxes']:
                                detection = Yolo_bbox()
                                detection.xMin, detection.yMin, detection.xMax, detection.yMax = box['xyxy']
                                detection.classes = box['label']
                                result.yolo_bbox_arr.append(detection)
                            result.objc = len(result.yolo_bbox_arr)
                            available = rospy.Time(*divmod(event['available_ns'], 1000000000))
                            write(out, '/YOLO/BBox', result, available)
                            event_index += 1
                        action = gps_action(mode, record.to_sec()-start, info['injection_window_sec'])
                        if topic in ('/gps', '/gps/fix') and action == 'missing':
                            omitted += 1
                            continue
                        if topic in ('/gps', '/gps/fix') and action == 'invalid':
                            msg = copy.deepcopy(msg)
                            if topic == '/gps':
                                msg.status = 0
                                msg.latitude = msg.longitude = msg.altitude = 0.
                            else:
                                msg.status.status = -1
                                msg.latitude = msg.longitude = msg.altitude = float('nan')
                            injected += 1
                        if topic == '/image_jpeg/compressed':
                            topic = '/camera/image/compressed'
                        write(out, topic, msg, record)
                    # Flush genuine late outputs at their recorded availability, not at EOF.
                    for event in events[event_index:]:
                        result = Yolo_bbox_arr()
                        result.header.stamp = rospy.Time(*divmod(event['stamp_ns'], 1000000000))
                        result.header.frame_id = event['frame_id']
                        for box in event['boxes']:
                            detection = Yolo_bbox()
                            detection.xMin, detection.yMin, detection.xMax, detection.yMax = box['xyxy']
                            detection.classes = box['label']
                            result.yolo_bbox_arr.append(detection)
                        result.objc = len(result.yolo_bbox_arr)
                        write(out, '/YOLO/BBox', result, rospy.Time(*divmod(event['available_ns'], 1000000000)))
                manifest['bags'][name]['variants'][mode] = dict(path=str(path), sha256=digest(path), topics=counts,
                    injected_gps_messages=injected, omitted_gps_messages=omitted,
                    image_bytes_and_sensor_headers_unchanged=True, natural_gps_denial=False)
                print(name, mode, counts, flush=True)
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    main()
