#!/usr/bin/env python3
"""Development-only GPS loss/recovery check; use a fresh isolated ROS master."""
import math
import time
import rospy
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu
from asmc_msgs.msg import MissionZone, MissionZoneArray
from asmc_hd_map.msg import HdMapContext, HdMapRoute
from asmc_hd_map.route_reference import RouteReference
from pyproj import Transformer


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def main():
    rospy.init_node('verify_hd_map_context', anonymous=True)
    require(rospy.get_param('~allow_synthetic_input', False),
            'requires _allow_synthetic_input:=true on an isolated development master')
    require(rospy.get_param('/local_map_crop/pose_source', '') == 'gps_imu', 'requires gps_imu source')
    require(not rospy.get_param('/local_map_crop/publish_crops', True), 'requires default production crops off')
    route = rospy.wait_for_message('/hd_map/global/route', HdMapRoute, timeout=15)
    missions = rospy.wait_for_message('/hd_map/global/mission_zones', MissionZoneArray, timeout=15)
    ref = RouteReference([(p.x, p.y, p.z) for p in route.points], route.stations_m)
    shadow = next(z for z in missions.zones if z.mission_type == MissionZone.TYPE_GPS_SHADOW)
    station = (shadow.route_station_start_m + shadow.route_station_end_m) / 2
    latest = [None]
    subscriber = rospy.Subscriber('/hd_map/context', HdMapContext, lambda m: latest.__setitem__(0, m), queue_size=1)
    gps = rospy.Publisher('/gps', GPSMessage, queue_size=1)
    imu = rospy.Publisher('/imu', Imu, queue_size=1)
    inverse = Transformer.from_crs(32652, 4326, always_xy=True)

    def send(s, valid=True, stamp=None):
        p = ref.point_at(s)
        a, b = ref.point_at(max(0, s - .5)), ref.point_at(min(ref.length_m, s + .5))
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        stamp = stamp or rospy.Time.now()
        orientation = Imu()
        orientation.header.stamp = stamp
        orientation.orientation.z, orientation.orientation.w = math.sin(.5*yaw), math.cos(.5*yaw)
        message = GPSMessage()
        message.header.stamp = stamp
        message.status = 1 if valid else 0
        message.eastOffset, message.northOffset = route.local_origin_in_global[:2]
        message.longitude, message.latitude = inverse.transform(
            p[0] + message.eastOffset + .35*math.cos(yaw),
            p[1] + message.northOffset + .35*math.sin(yaw))
        imu.publish(orientation)
        time.sleep(.01)
        gps.publish(message)
        return p, stamp

    def wait(predicate, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = latest[0]
            if m is not None and predicate(m):
                return m
            time.sleep(.01)
        raise RuntimeError('context condition timed out: ' + str(latest[0]))

    time.sleep(.5)
    p, stamp = send(station)
    first = wait(lambda m: m.pose_valid and m.pose_stamp == stamp and m.route_valid)
    require(math.hypot(first.ego_pose.position.x-p[0], first.ego_pose.position.y-p[1]) < .001,
            'GPS rear-axle correction differs from published map pose')
    require(abs(first.route_station_m - station) < .02, 'official station differs')
    require(shadow.id in first.gps_shadow_zone_ids, 'shadow zone missing')
    _, loss_stamp = send(station, valid=False)
    lost = wait(lambda m: m.pose_status == m.POSE_GPS_UNAVAILABLE)
    require((rospy.Time.now()-loss_stamp).to_sec() < .5, 'explicit loss waited for stale timeout')
    require(not lost.pose_valid and not lost.lane_valid and not lost.route_valid and not lost.boundaries,
            'outage exposes valid current map geometry/context')
    require(lost.gps_shadow_context_from_last_valid_pose and shadow.id in lost.gps_shadow_zone_ids,
            'historical shadow context missing')
    recovery_s = min(ref.length_m-1, shadow.route_station_end_m + 50)
    _, recovery_stamp = send(recovery_s)
    recovered = wait(lambda m: m.pose_valid and m.pose_stamp == recovery_stamp and m.route_valid)
    require(abs(recovered.route_station_m-recovery_s) < .02, 'GPS recovery failed to reacquire')
    require(shadow.id not in recovered.gps_shadow_zone_ids, 'old shadow zone survived recovery')
    send(station, valid=False, stamp=stamp)
    time.sleep(.1)
    require(latest[0].pose_valid, 'delayed outage packet invalidated recovered GPS')
    stale = wait(lambda m: m.pose_status == m.POSE_STALE)
    require(not stale.pose_valid and not stale.route_valid and not stale.boundaries, 'no-packet outage remains valid')
    topics = dict(rospy.get_published_topics())
    require('/hd_map/local/lanes' not in topics and '/hd_map/local/drivable_grid' not in topics,
            'default runtime advertises crop publishers')
    retained = rospy.wait_for_message('/hd_map/global/route', HdMapRoute, timeout=3)
    require(retained.route_sha256 == route.route_sha256, 'static route unavailable after loss')
    print('PASS context GPS rear-axle / official stations / immediate loss / historical shadow / recovery / stale / no crops')


if __name__ == '__main__':
    main()
