"""Check the integrated camera-to-jamming handoff without ROS or a GPU."""
import ast
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace, MappingProxyType
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
CAMERA = ROOT / 'src/asmc_camera'
LAUNCH = ROOT / 'src/asmc_planner/launch'
JAMMING = ROOT / 'src/jamming_lane_boundary_adapter/launch'


def default_arguments(path):
    return {a.get('name'): a.get('default') for a in ET.parse(path).findall('arg')}


def canonical_family_hash():
    # Execute the actual constructor through its hash assignment. ROS, GPU
    # projectors and trackers are inert; calibration parsing remains real.
    component = default_arguments(CAMERA / 'launch/_components.launch')
    calibration_path = component['camera_calibration_config'].replace('$(find asmc_camera)', str(CAMERA))
    params = yaml.safe_load(Path(calibration_path).read_text())
    params.update(yaml.safe_load((CAMERA / 'config/bev/lane_fitting.yaml').read_text()))
    params['imu_motion']=yaml.safe_load((CAMERA/'config/bev/imu_motion.yaml').read_text())
    params.update(yaml.safe_load((CAMERA / 'config/camera/lane_input.yaml').read_text()))

    class Parameters:
        Subscriber = staticmethod(lambda *args, **kwargs: None)
        def get_param(self, name, *fallback):
            try:
                value = params
                for key in name.lstrip('~').split('/'):
                    value = value[key]
                return value
            except KeyError:
                if fallback:
                    return fallback[0]
                raise

    spec = importlib.util.spec_from_file_location(
        'jamming_contract_calibration', CAMERA / 'src/asmc_camera/bev/calibration.py')
    calibration = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = calibration
    spec.loader.exec_module(calibration)
    # Execute the real hash-policy class with its real literal defaults. Loading
    # the full BEV package here would unnecessarily require OpenCV and ROS.
    defaults_ast=ast.parse((CAMERA/'src/asmc_camera/bev/shared_lane_family.py').read_text())
    defaults=next(n for n in defaults_ast.body if isinstance(n,ast.Assign) and
                  any(isinstance(t,ast.Name) and t.id=='DEFAULT_SHARED_FAMILY_CONFIG' for t in n.targets))
    policy_ast=ast.parse((CAMERA/'src/asmc_camera/bev/family_fit_policy.py').read_text())
    policy=next(n for n in policy_ast.body if isinstance(n,ast.ClassDef) and n.name=='FamilyFitPolicy')
    policy_scope=dict(DEFAULT_SHARED_FAMILY_CONFIG=ast.literal_eval(defaults.value),
                      MappingProxyType=MappingProxyType,math=math)
    exec(compile(ast.Module(body=[policy],type_ignores=[]),'<family-fit-contract>', 'exec'),policy_scope)
    FamilyFitPolicy=policy_scope['FamilyFitPolicy']
    source = ast.parse((CAMERA / 'src/asmc_camera/nodes/lane_vectorizer_node.py').read_text())
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'LaneVectorizerNode')
    constructor = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    end = next(i for i, n in enumerate(constructor.body)
               if isinstance(n, ast.Assign) and any(
                   isinstance(t, ast.Attribute) and t.attr == 'boundary_family_config_sha256'
                   for t in n.targets))
    constructor.body = constructor.body[:end + 1]
    inert = lambda *args, **kwargs: None
    scope = dict(threading=threading, FamilyFitPolicy=FamilyFitPolicy,
                 association_backend_info=inert,  # runtime diagnostic, excluded from geometry hash
                 ImuTimeline=lambda **kwargs: SimpleNamespace(add_timing=inert),
                 Imu=object,SensorFrameTiming=object,rospy=Parameters(), hashlib=hashlib, json=json,
                 calibrations_from_dict=calibration.calibrations_from_dict,
                 LaneEpochGate=inert, ModelGridGroundProjector=inert,
                 invalid_support_config_from_dict=inert, planning_zones_from_config=inert,
                 DEFAULT_PLANNING_ZONES=[], TemporalBoundaryTracker=inert,
                 _names=lambda values: values)
    exec(compile(ast.Module(body=[constructor], type_ignores=[]), '<camera-hash-prefix>', 'exec'), scope)
    instance = SimpleNamespace(_runtime_invalidate=inert,_imu_callback=inert)
    scope['__init__'](instance)
    return instance.boundary_family_config_sha256, calibration_path


class JammingLaunchContractTest(unittest.TestCase):
    def test_default_run_owns_one_object_source_and_no_duplicate_signal_model(self):
        for name in ['autonomy_no_bridges_v2.launch', 'mode_control_integration.launch']:
            args=default_arguments(LAUNCH/name)
            self.assertNotIn('objects_source',args)
            self.assertNotIn('traffic_light',args)
            self.assertEqual('competition' if name=='autonomy_no_bridges_v2.launch' else 'gps_imu',args['state_source'])
            self.assertEqual('false',args['highway_merge'])
        root=ET.parse(LAUNCH/'autonomy_no_bridges_v2.launch')
        lidar=root.find("./include[@file='$(find planning_inputs)/launch/objects.launch']")
        self.assertIsNotNone(lidar)
        self.assertEqual('$(arg lidar_input_topic)',lidar.find("arg[@name='input_topic']").get('value'))
        self.assertEqual('/perception/lidar/planning_observations_map',
                         default_arguments(LAUNCH/'autonomy_no_bridges_v2.launch')['lidar_input_topic'])
        self.assertIsNone(root.find("./node[@type='morai_objects_adapter.py']"))

    def test_integrated_consumers_match_canonical_camera_geometry(self):
        expected_hash, calibration = canonical_family_hash()
        for name in ['autonomy_no_bridges_v2.launch', 'mode_control_integration.launch']:
            args = default_arguments(LAUNCH / name)
            self.assertEqual(expected_hash, args['expected_config_sha256'], name)
            self.assertEqual(calibration, args['calibration_file'].replace('$(find asmc_camera)', str(CAMERA)))
        integrated = ET.parse(LAUNCH / 'autonomy_no_bridges_v2.launch')
        change = integrated.find("./include[@file='$(find lane_boundary_adapter)/launch/camera_bbox_lane_change.launch']")
        self.assertEqual('$(arg expected_config_sha256)', change.find("arg[@name='expected_config_sha256']").get('value'))
        classifier = ET.parse(JAMMING / 'camera_bbox_lane_change.launch').find(
            "./include[@file='$(find lane_boundary_adapter)/launch/camera_bbox_lane.launch']")
        self.assertEqual('$(arg expected_config_sha256)', classifier.find("arg[@name='expected_hash']").get('value'))

    def test_jamming_keeps_main_camera_bbox_ownership(self):
        integrated=ET.parse(LAUNCH/'autonomy_no_bridges_v2.launch')
        self.assertIsNone(integrated.find("./include[@file='$(find lane_boundary_adapter)/launch/camera_bbox_detector.launch']"))
        change=integrated.find("./include[@file='$(find lane_boundary_adapter)/launch/camera_bbox_lane_change.launch']")
        # PR127 adds the explicit exit/recovery maneuver on top of PR128.
        self.assertEqual('true', change.find("arg[@name='return_to_entry_lane']").get('value'))
        self.assertEqual('/ctrl_cmd/jamming',change.find("arg[@name='control_topic']").get('value'))

    def test_1024_contact_geometry_and_longitudinal_speed_contract(self):
        package=ROOT/'src/jamming_lane_boundary_adapter'
        fields=(package/'msg/CameraBBoxLaneResult.msg').read_text()
        for name in ['contact_x','contact_y_min','contact_y_max']:
            self.assertIn('float32[] '+name,fields)
        controller=(package/'src/camera_bbox_lane_change_node.cpp').read_text()
        self.assertIn('std::fabs(message->velocity.x)',controller)
        self.assertNotIn('message->velocity.y',controller)
        self.assertIn('far_obstacle_lane_unresolved',controller)
        args=default_arguments(JAMMING/'camera_bbox_lane_change.launch')
        self.assertEqual('true',args['straddle_avoidance'])
        self.assertEqual('false',args['use_lidar_obstacles'])
        self.assertEqual('false',args['return_to_entry_lane'])  # standalone still opts in explicitly

    def test_highway_dashboard_and_recovery_passthrough_coexist(self):
        root=ET.parse(LAUNCH/'autonomy_no_bridges_v2.launch')
        planner=root.find("./include[@file='$(find asmc_planner)/launch/planner_v2.launch']")
        self.assertEqual('$(arg highway_merge)',planner.find("arg[@name='highway_merge']").get('value'))
        change=root.find("./include[@file='$(find lane_boundary_adapter)/launch/camera_bbox_lane_change.launch']")
        for name,parent in [('use_lidar_obstacles','jamming_use_lidar_obstacles'),
                            ('creep_speed_kph','jamming_creep_speed_kph'),
                            ('creep_clear_distance','jamming_creep_clear_distance')]:
            self.assertEqual('$(arg '+parent+')',change.find("arg[@name='"+name+"']").get('value'))
        child_args=default_arguments(JAMMING/'camera_bbox_lane_change.launch')
        self.assertIn('bbox_conf_thres',child_args)
        self.assertEqual('0.0',child_args['creep_speed_kph'])
        self.assertEqual('2.0',child_args['creep_clear_distance'])

    def test_dashboard_uses_the_same_planning_inputs(self):
        root = ET.parse(LAUNCH / 'autonomy_no_bridges_v2.launch')
        self.assertEqual('true', default_arguments(LAUNCH / 'autonomy_no_bridges_v2.launch')['dashboard'])
        dashboard = root.find("./include[@file='$(find asmc_planner)/launch/dashboard.launch']")
        self.assertEqual('$(arg dashboard)', dashboard.get('if'))
        self.assertEqual('$(arg state_topic)', dashboard.find("arg[@name='state_topic']").get('value'))
        self.assertEqual('$(arg canonical_objects_topic)', dashboard.find("arg[@name='dashboard_lidar_objects_topic']").get('value'))


if __name__ == '__main__':
    unittest.main()
