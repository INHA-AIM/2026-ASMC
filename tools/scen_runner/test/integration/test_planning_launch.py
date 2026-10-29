"""Resolve XML only: no ROS master, controller or simulator is started."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.config import REPO_ROOT, TOOL_ROOT

try:
    import roslaunch.config
except ImportError:
    roslaunch = None


@unittest.skipIf(roslaunch is None, 'Run in the ROS container')
class PlanningLaunchTest(unittest.TestCase):
    def test_runner_resolves_the_same_nodes_and_settings_as_the_normal_driving_command(self):
        runner=roslaunch.config.load_config_default([str(TOOL_ROOT/'launch/planning.launch')],None)
        ordinary=roslaunch.config.load_config_default([
            (str(REPO_ROOT/'src/asmc_planner/launch/autonomy_no_bridges_v2.launch'),
             ['dashboard:=false','rviz:=false'])],None)
        params=lambda cfg:{key:value.value for key,value in cfg.params.items()}
        nodes=lambda cfg:sorted((n.namespace+n.name,n.package,n.type,n.args) for n in cfg.nodes)
        self.assertEqual(params(runner),params(ordinary))
        self.assertEqual(nodes(runner),nodes(ordinary))
        names=[node.namespace+node.name for node in runner.nodes]
        self.assertEqual(len(names),len(set(names)))
        self.assertIn('/planning_competition_speed_state',names)
        self.assertTrue(runner.params['/planning_mode/shadow_zone/path_file'].value)
        self.assertTrue(runner.params['/camera_bbox_lane_change/return_to_entry_lane'].value)


if __name__ == '__main__':
    unittest.main()
