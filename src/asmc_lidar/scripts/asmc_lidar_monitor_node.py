#!/usr/bin/env python3
"""asmc_lidar_monitor_node: read-only web monitor of lidar_pipeline_node.

Subscribes to the shadow node's monitoring outputs (<debug_topic_prefix>/scan,
/raw, /roi, /voxel, /leveled, the diagnostics and the typed local array), groups
them by scan stamp and serves a fixed set of HTTP routes (web assets,
/api/health, /api/status, /api/frame) on the loopback address by default.
It publishes nothing, calls no service and sets no parameter. With ~open_browser
it opens the page once after the server answered its own health check.
"""
import os
import hashlib
import json
import math
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import asmc_lidar_monitor_core as core  # noqa: E402

import rosgraph  # noqa: E402
import rospkg  # noqa: E402
import rospy  # noqa: E402
from asmc_msgs.msg import LidarObjectArray  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402

DIAGNOSTIC_STATUS_NAME = 'lidar_pipeline'


def default_web_dir():
    try:
        return os.path.join(rospkg.RosPack().get_path('asmc_lidar'), 'web')
    except rospkg.ResourceNotFound:
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


def ros_logger(level, text):
    {'debug': rospy.logdebug, 'info': rospy.loginfo, 'warn': rospy.logwarn}.get(level, rospy.loginfo)(
        'asmc_lidar_monitor: %s', text)


def load_drivable_overlay(map_path, reference_path, pipeline_path):
    """Read-only display product. Never used for filtering or planning decisions."""
    try:
        import yaml
        with open(reference_path) as stream:
            reference = json.load(stream)['local_origin_in_global']
        with open(pipeline_path) as stream:
            origin = yaml.safe_load(stream)['map']['origin']
        if len(origin) != 3 or len(reference) != 3 or any(
                not math.isfinite(float(a)) or not math.isfinite(float(b)) or abs(float(a)-float(b)) > 1e-6
                for a, b in zip(origin, reference)):
            raise ValueError('지도와 LiDAR 원점 불일치')
        if os.path.getsize(map_path) > 16 * 1024 * 1024:
            raise ValueError('지도 크기 제한 초과')
        with open(map_path, 'rb') as stream:
            raw=stream.read()
        product=json.loads(raw)
        map_sha=hashlib.sha256(raw).hexdigest()
        if product.get('frame_id') != 'map':
            raise ValueError('지원하지 않는 지도 frame')
        polygons = []
        for area in product['drivable_areas']:
            polygon = [[float(p[0]), float(p[1])] for p in area['polygon']]
            if len(polygon) < 3 or any(not math.isfinite(v) for p in polygon for v in p):
                raise ValueError('잘못된 지도 polygon')
            polygons.append(polygon)
        if not polygons:
            raise ValueError('빈 주행가능영역')
        return {'available': True, 'build_id': product.get('build_id'), 'sha256':map_sha, 'frame_id': 'map',
                'origin': reference, 'polygons': polygons, 'height_mode': 'ego_reference_plane'}
    except (OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        return {'available': False, 'reason': str(exc)}


def main():
    rospy.init_node('asmc_lidar_monitor')
    port = int(rospy.get_param('~port', 8091))
    bind_address = str(rospy.get_param('~bind_address', '127.0.0.1'))
    open_browser = bool(rospy.get_param('~open_browser', False))
    max_clients = int(rospy.get_param('~max_clients', 8))
    header_timeout_s = float(rospy.get_param('~header_timeout_s', 5.0))
    request_timeout_s = float(rospy.get_param('~request_timeout_s', 20.0))
    web_dir = str(rospy.get_param('~web_dir', '')) or default_web_dir()
    prefix = rospy.resolve_name(str(rospy.get_param('~debug_topic_prefix', '/migration/lidar/debug')).rstrip('/'))
    diagnostics_topic = rospy.resolve_name(str(rospy.get_param('~diagnostics_topic', '/migration/lidar/diagnostics')))
    objects_local_topic = rospy.resolve_name(str(rospy.get_param('~objects_local_topic', '/migration/lidar/objects_local')))
    objects_map_topic = rospy.resolve_name(str(rospy.get_param('~objects_map_topic', '/migration/lidar/objects_map')))
    points_topic = rospy.resolve_name(str(rospy.get_param('~points_topic', '/velodyne_points')))
    max_scenes = int(rospy.get_param('~max_scenes', 4))
    complete_timeout_s = float(rospy.get_param('~scene_timeout_s', 0.6))
    master_check_s = float(rospy.get_param('~master_check_s', 2.0))

    drivable = {'available': False, 'reason': '지도 패키지 없음'}
    try:
        packages = rospkg.RosPack()
        planner = packages.get_path('asmc_planner')
        hd_map = packages.get_path('asmc_hd_map')
        lidar = packages.get_path('asmc_lidar')
        drivable = load_drivable_overlay(
            str(rospy.get_param('~drivable_map_file', os.path.join(hd_map, 'config/kcity2025_compact_drivable_map.json'))),
            str(rospy.get_param('~drivable_reference_file', os.path.join(planner, 'config/behavior/global_info.json'))),
            str(rospy.get_param('~pipeline_config', os.path.join(lidar, 'config/lidar_pipeline.yaml'))))
    except rospkg.ResourceNotFound:
        pass
    if not drivable['available']:
        rospy.logwarn('asmc_lidar_monitor: drivable overlay unavailable: %s', drivable['reason'])

    topics = {stage: '%s/%s' % (prefix, stage) for stage in core.STAGES}
    topics['scan'] = prefix + '/scan'
    store = core.SceneStore(max_scenes=max_scenes, complete_timeout_s=complete_timeout_s,
                            metadata_first=bool(rospy.get_param("~metadata_first", True)))
    state = core.MonitorState(store, config={
        'topics': dict(topics, diagnostics=diagnostics_topic, objects_local=objects_local_topic,
                       objects_map=objects_map_topic, points=points_topic),
        'port': port, 'bind_address': bind_address, 'max_clients': max_clients, 'header_timeout_s': header_timeout_s,
        'request_timeout_s': request_timeout_s, 'node': rospy.get_name()})

    try:
        server = core.MonitorServer(bind_address, port, state, web_dir, max_clients=max_clients, logger=ros_logger,
                                    header_timeout_s=header_timeout_s, request_timeout_s=request_timeout_s,
                                    drivable=drivable)
    except OSError as exc:
        rospy.logerr('asmc_lidar_monitor: cannot bind http://%s:%d/ (%s). The LiDAR node keeps running; '
                     'start the monitor again with another port, e.g. dashboard_port:=%d',
                     bind_address, port, exc, port + 1)
        return 2
    if not os.path.isfile(os.path.join(web_dir, 'index.html')):
        rospy.logwarn('asmc_lidar_monitor: web assets not found in %s (set ~web_dir)', web_dir)
    http_thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.5}, daemon=True)
    http_thread.start()
    rospy.loginfo('asmc_lidar_monitor: serving %s (bind %s:%d, web %s, session %s, max %d connections, '
                  'header/request timeout %.1f/%.1f s); reading %s/* , %s, %s', server.url, bind_address, port, web_dir,
                  state.session_id, server.max_clients, server.header_timeout_s, server.request_timeout_s, prefix,
                  diagnostics_topic, objects_local_topic)
    if bind_address not in ('127.0.0.1', 'localhost', '::1'):
        rospy.logwarn('asmc_lidar_monitor: bound to %s; the page is reachable from other hosts', bind_address)

    def on_scan(msg):
        meta = store.add_scan(msg.data)
        if meta is not None:
            scan_map = meta.get('map') if meta.get('stages') else None
            state.note_scan(meta['stamp_ns'], (scan_map or {}).get('valid'), (scan_map or {}).get('reason'))

    def cloud_callback(stage):
        def on_cloud(msg):
            try:
                cloud = core.parse_point_cloud([(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                                               msg.point_step, msg.width, msg.height, msg.data, msg.is_bigendian)
            except ValueError as exc:
                rospy.logwarn_throttle(5.0, 'asmc_lidar_monitor: %s cloud rejected: %s', stage, exc)
                return
            store.add_cloud(stage, msg.header.stamp.to_nsec(), cloud)
        return on_cloud

    def on_diagnostics(msg):
        for status in msg.status:
            if status.name == DIAGNOSTIC_STATUS_NAME:
                state.update_diagnostics(status.level, status.message, {kv.key: kv.value for kv in status.values})

    def on_objects(msg):
        state.update_objects_local(msg.status, msg.reason, msg.header.stamp.to_nsec(), len(msg.objects))

    def on_objects_map(msg):
        state.update_objects_map(msg.status, msg.reason, msg.header.stamp.to_nsec())

    subscribers = [rospy.Subscriber(topics['scan'], String, on_scan, queue_size=2, buff_size=2 ** 22)]
    for stage in core.STAGES:
        subscribers.append(rospy.Subscriber(topics[stage], PointCloud2, cloud_callback(stage), queue_size=1,
                                            buff_size=2 ** 23))
    subscribers.append(rospy.Subscriber(diagnostics_topic, DiagnosticArray, on_diagnostics, queue_size=2))
    subscribers.append(rospy.Subscriber(objects_local_topic, LidarObjectArray, on_objects, queue_size=2))
    subscribers.append(rospy.Subscriber(objects_map_topic, LidarObjectArray, on_objects_map, queue_size=2))

    def master_loop():
        master = rosgraph.Master(rospy.get_name())
        while not rospy.is_shutdown():
            try:
                publishers, _, _ = master.getSystemState()
                by_topic = {topic: nodes for topic, nodes in publishers}
                shadow_nodes = sorted(set(by_topic.get(topics['scan'], [])) | set(by_topic.get(diagnostics_topic, [])))
                state.update_master(True, '', len(by_topic.get(points_topic, [])), len(by_topic.get(topics['scan'], [])),
                                    shadow_nodes)
            except Exception as exc:  # pylint: disable=broad-except
                state.update_master(False, str(exc))
            time.sleep(master_check_s)

    threading.Thread(target=master_loop, daemon=True).start()

    if open_browser:
        def opener():
            if core.wait_for_server(server.url, timeout_s=10.0):
                opened, how = core.open_browser_once(server.url)
                if opened:
                    rospy.loginfo('asmc_lidar_monitor: browser opened once via %s -> %s', how, server.url)
                else:
                    rospy.logwarn('asmc_lidar_monitor: browser not opened (%s). Open %s manually', how, server.url)
            else:
                rospy.logwarn('asmc_lidar_monitor: server did not answer its health check; open %s manually', server.url)
        threading.Thread(target=opener, daemon=True).start()

    rospy.on_shutdown(server.shutdown)
    rospy.spin()
    server.server_close()
    rospy.loginfo('asmc_lidar_monitor: stopped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
