"""Resolve actual ROS launch profiles, component overrides and failure isolation."""
from pathlib import Path
import unittest
import roslaunch

LAUNCH = Path(__file__).resolve().parents[2] / 'launch'
MODELS = ('camera_semantic_inference','camera_object_detector','traffic_light_infer')

class LaunchModesTest(unittest.TestCase):
    def config(self, *arguments):
        return roslaunch.config.load_config_default(
            [(str(LAUNCH/'perception.launch'),list(arguments))],None,verbose=False)

    def test_bare_launch_loads_no_models_or_visualizers(self):
        self.assertFalse(self.config().nodes)

    def test_planning_loads_all_models_standby_without_debug_or_region_owner(self):
        config=self.config('profile:=planning')
        self.assertFalse(any(n.required for n in config.nodes))
        for name in MODELS:
            self.assertTrue(config.params['/'+name+'/managed'].value)
            self.assertEqual(0,config.params['/'+name+'/initial_mode'].value)
        self.assertFalse(any('visualizer' in n.name or n.name=='camera_region_example' for n in config.nodes))
        self.assertFalse(config.params['/camera_lane_vectorizer/shared_lane_family/runtime_debug_enabled'].value)
        self.assertEqual(10.,config.params['/traffic_light_infer/target_hz'].value)

    def test_debug_starts_all_models_active_and_each_overlay(self):
        config=self.config('profile:=debug')
        for name in MODELS:
            self.assertTrue(config.params['/'+name+'/managed'].value)
            self.assertEqual(2,config.params['/'+name+'/initial_mode'].value)
        nodes={n.name for n in config.nodes}
        self.assertTrue({'camera_object_visualizer','camera_traffic_light_visualizer','camera_semantic_visualizer','camera_bev_visualizer'}<=nodes)
        self.assertFalse(any(n.required for n in config.nodes))
        self.assertNotIn('camera_region_example',nodes)

    def test_yolo_debug_can_run_without_lane_traffic_lidar_or_gps(self):
        config=self.config('profile:=debug','start_camera_lane:=false','start_traffic_light:=false')
        self.assertEqual({'camera_object_detector','camera_object_visualizer'},{n.name for n in config.nodes})

    def test_model_exclusion_and_initial_mode_override_are_independent(self):
        config=self.config('profile:=debug','start_camera_objects:=false','lane_initial_mode:=0')
        self.assertEqual(0,config.params['/camera_semantic_inference/initial_mode'].value)
        self.assertEqual(2,config.params['/traffic_light_infer/initial_mode'].value)
        self.assertFalse(any(n.name.startswith('camera_object') for n in config.nodes))

    def test_legacy_keeps_lane_unrestricted_and_required_without_debug(self):
        config=self.config('profile:=legacy')
        self.assertFalse(config.params['/camera_semantic_inference/managed'].value)
        self.assertTrue(next(n for n in config.nodes if n.name=='camera_semantic_inference').required)
        self.assertFalse(any('visualizer' in n.name or n.name in MODELS[1:] for n in config.nodes))

    def test_direct_backend_launches_default_to_managed_standby(self):
        for filename,name in [('camera_lane.launch',MODELS[0]),('camera_objects.launch',MODELS[1])]:
            config=roslaunch.config.load_config_default([str(LAUNCH/filename)],None,verbose=False)
            self.assertTrue(config.params['/'+name+'/managed'].value)
            self.assertEqual(0,config.params['/'+name+'/initial_mode'].value)

    def test_unknown_profile_is_rejected(self):
        with self.assertRaises(roslaunch.RLException):
            self.config('profile:=mistyped')

    def test_mask_options_are_forwarded_to_detector_and_overlay(self):
        for enabled in ('true','false'):
            config=self.config('profile:=debug','objects_hood_mask:='+enabled,
                               'objects_hood_mask_config:=/tmp/test-mask.yaml')
            for node in ('camera_object_detector','camera_object_visualizer'):
                self.assertEqual(enabled=='true',config.params['/'+node+'/hood_mask'].value)
                self.assertEqual('/tmp/test-mask.yaml',config.params['/'+node+'/hood_mask_config'].value)

    def test_region_example_is_off_unless_explicitly_requested(self):
        config=roslaunch.config.load_config_default([str(LAUNCH/'camera_region_example.launch')],None,verbose=False)
        self.assertFalse(config.nodes)

    def test_filter_and_raw_options_are_shared_and_debug_only(self):
        for profile in ['debug','planning']:
            config=self.config('profile:='+profile,'start_visualization:=true',
                'objects_hood_mask:=false','objects_ego_hood_filter:=true',
                'objects_ego_hood_filter_config:=/tmp/filter.yaml')
            for node in ['camera_object_detector','camera_object_visualizer']:
                self.assertTrue(config.params['/'+node+'/ego_hood_filter'].value)
                self.assertEqual('/tmp/filter.yaml',config.params['/'+node+'/ego_hood_filter_config'].value)
                self.assertEqual(profile=='debug',config.params['/'+node+'/debug_raw'].value)
        config=self.config('profile:=debug','objects_debug_raw:=false')
        self.assertFalse(config.params['/camera_object_detector/debug_raw'].value)

    def test_selected_mask_default_keeps_idle_off_and_filter_off(self):
        self.assertFalse(self.config('profile:=idle').nodes)
        config=self.config('profile:=debug')
        for node in ['camera_object_detector','camera_object_visualizer']:
            self.assertTrue(config.params['/'+node+'/hood_mask'].value)
            self.assertFalse(config.params['/'+node+'/ego_hood_filter'].value)
