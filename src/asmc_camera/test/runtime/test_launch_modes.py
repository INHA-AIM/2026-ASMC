"""Resolve actual launch substitutions to check opt-in failure isolation."""
from pathlib import Path
import unittest
import roslaunch


LAUNCH = Path(__file__).resolve().parents[2] / 'launch'


class LaunchModesTest(unittest.TestCase):
    def config(self, managed):
        return roslaunch.config.load_config_default(
            [(str(LAUNCH/'perception.launch'), ['managed:='+managed,
              'start_camera_objects:=true','start_traffic_light:=true'])], None,
            verbose=False)

    def test_managed_models_do_not_take_other_models_down(self):
        config = self.config('true')
        self.assertFalse(any(node.required for node in config.nodes))
        for name in ('camera_semantic_inference','camera_object_detector','traffic_light_infer'):
            self.assertTrue(config.params['/'+name+'/managed'].value)
            self.assertEqual(0,config.params['/'+name+'/initial_mode'].value)
        self.assertNotIn('camera_region_example',[node.name for node in config.nodes])

    def test_legacy_required_nodes_and_switch_defaults_are_preserved(self):
        config=self.config('false')
        self.assertTrue(next(n for n in config.nodes if n.name=='camera_semantic_inference').required)
        self.assertTrue(next(n for n in config.nodes if n.name=='traffic_light_infer').required)
        self.assertFalse(next(n for n in config.nodes if n.name=='camera_object_detector').required)

    def test_region_example_is_off_unless_explicitly_requested(self):
        config=roslaunch.config.load_config_default([str(LAUNCH/'camera_region_example.launch')],None,verbose=False)
        self.assertFalse(config.nodes)
