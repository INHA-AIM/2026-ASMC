#!/usr/bin/env python3
"""Read-only ownership check before Localization launch on an existing shared master.

Does not reserve the simulator, start a master, launch nodes or kill peers.
Automated vehicle trials also require the common physical simulator lease.
"""
import json
import os
from pathlib import Path


def main():
    import rosgraph
    import roslaunch.config
    import roslaunch.xmlloader
    import rospkg

    root = Path(__file__).resolve().parents[2]
    package = Path(rospkg.RosPack().get_path('asmc_planner')).resolve()
    if package != root / 'src/asmc_planner':
        raise RuntimeError('ROS package resolves to a different checkout: ' + str(package))
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(package / 'launch/autonomy_no_bridges_v2.launch'), config,
        argv=['jamming_planner:=localization', 'dashboard:=false'], verbose=False)
    expected = {n.namespace.rstrip('/') + '/' + n.name for n in config.nodes}
    master = rosgraph.Master('/localization_preflight')
    master.getPid()
    state = master.getSystemState()
    publishers = dict(state[0])
    existing = {node for rows in state[:2] for _, nodes in rows for node in nodes}
    conflicts = {topic: publishers[topic] for topic in (
        '/ctrl_cmd', '/ctrl_cmd/normal', '/ctrl_cmd/jamming', '/planning/mode',
        '/planning/localization/motion') if publishers.get(topic)}
    missing = [topic for topic in (
        '/gps', '/imu', '/Ego_topic', '/perception/lidar/tracks_local',
        '/perception/lidar/planning_observations_map') if not publishers.get(topic)]
    report = dict(master=os.environ.get('ROS_MASTER_URI'), package=str(package),
                  conflicting_nodes=sorted(expected & existing),
                  conflicting_publishers=conflicts, missing_inputs=missing,
                  check='publisher presence and ownership; freshness is checked by runtime nodes')
    print(json.dumps(report, indent=2))
    return int(bool(expected & existing or conflicts or missing))


if __name__ == '__main__':
    raise SystemExit(main())
