#!/usr/bin/env python3
"""Export source RGB and raw XYZ for qualitative B0 review, without projection.

Run after performance measurement. A wide unknown OBB is a review candidate,
not a wall, merged vehicle, or recall ground truth. Input selection is explicit
and saved; source bytes and full XYZ are retained alongside the preview.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag
from sensor_msgs import point_cloud2


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nearest(rows, target, key):
    return min(rows, key=lambda row: abs(key(row)-target)) if rows else None


def export(bag_path, observations, selections, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    targets = [row['stamp_ns'] for row in selections]
    cloud_messages, images = {}, {}
    # Only retain the closest raw messages for each explicit case.
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for topic, msg, _ in bag.read_messages(topics=['/velodyne_points', '/image_jpeg/compressed']):
            stamp = msg.header.stamp.to_nsec()
            table = cloud_messages if topic == '/velodyne_points' else images
            for target in targets:
                if target not in table or abs(stamp-target) < abs(table[target].header.stamp.to_nsec()-target):
                    table[target] = msg
    output.mkdir(parents=True, exist_ok=False)
    (output/'selection.json').write_text(json.dumps(selections, indent=2)+'\n')
    catalog = []
    yolo = observations['outputs']['yolo']
    for number, selection in enumerate(selections):
        target = selection['stamp_ns']
        cloud, image = cloud_messages.get(target), images.get(target)
        if cloud is None or image is None:
            catalog.append(dict(selection, export_error='missing_raw_camera_or_lidar'))
            continue
        folder = output/('case-%02d' % number)
        folder.mkdir()
        xyz = np.array(list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float32)
        np.savez_compressed(folder/'raw_xyz.npz', xyz=xyz)
        (folder/'source.jpg').write_bytes(bytes(image.data))
        fig, axis = plt.subplots(figsize=(8, 7))
        # XY coordinates are those of the original PointCloud2 header frame.
        # No map track center, leveled cloud, or camera projection is overlaid.
        if len(xyz):
            preview = xyz[::max(1, len(xyz)//50000)]
            scatter = axis.scatter(preview[:, 0], preview[:, 1], c=preview[:, 2], s=1,
                                   cmap='viridis', vmin=-3, vmax=5)
            fig.colorbar(scatter, ax=axis, label='raw Z (m)')
        axis.set(xlabel='raw X (m)', ylabel='raw Y (m)', aspect='equal',
                 xlim=(-40, 60), ylim=(-40, 40), title='Raw LiDAR: '+cloud.header.frame_id)
        axis.grid(alpha=.2)
        fig.tight_layout()
        fig.savefig(folder/'raw_xy.png', dpi=140)
        plt.close(fig)
        track = nearest(observations['tracks'], target, lambda row: row['stamp_ns'])
        camera_stamp = image.header.stamp.to_nsec()
        boxes = nearest(yolo, camera_stamp, lambda row: row['stamp_ns'])
        record = dict(selection, raw_lidar_stamp_ns=cloud.header.stamp.to_nsec(),
                      raw_lidar_frame_id=cloud.header.frame_id, raw_camera_stamp_ns=camera_stamp,
                      raw_camera_frame_id=image.header.frame_id,
                      camera_lidar_offset_ms=(camera_stamp-cloud.header.stamp.to_nsec())/1e6,
                      input_alignment_within_50ms=abs(camera_stamp-cloud.header.stamp.to_nsec()) <= 50000000,
                      raw_point_count=len(xyz), tracked_observation=track,
                      yolo_exact_camera_source=bool(boxes and boxes['stamp_ns'] == camera_stamp),
                      yolo_observation=boxes if boxes and boxes['stamp_ns'] == camera_stamp else None,
                      nearest_unmatched_yolo_observation=boxes if boxes and boxes['stamp_ns'] != camera_stamp else None,
                      independent_ground_truth=False, camera_projection_performed=False,
                      geometry_labels='unreviewed_candidate_not_accuracy_ground_truth',
                      source_files={p.name: sha(p) for p in folder.iterdir() if p.is_file()})
        (folder/'metadata.json').write_text(json.dumps(record, indent=2)+'\n')
        catalog.append(record)
    (output/'catalog.json').write_text(json.dumps(dict(bag=str(bag_path), cases=catalog,
        purpose='qualitative_failure_review_not_quantitative_accuracy'), indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--observations', type=Path, required=True)
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    export(args.bag, json.loads(args.observations.read_text()),
           json.loads(args.selection.read_text()), args.output)


if __name__ == '__main__':
    main()
