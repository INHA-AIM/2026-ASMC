from pathlib import Path
import tempfile
import unittest
import roslaunch

LAUNCH = Path(__file__).resolve().parents[2] / 'launch'


class CameraEntrypointsTest(unittest.TestCase):
    def config(self, filename='camera.launch', *args):
        return roslaunch.config.load_config_default([(str(LAUNCH / filename), list(args))], None, verbose=False)

    def test_public_default_starts_three_independent_active_workers_without_gui(self):
        config = self.config()
        names = {n.name for n in config.nodes}
        self.assertTrue({'camera_objects', 'camera_lane', 'camera_traffic_light'} <= names)
        self.assertNotIn('camera_runtime', names)
        self.assertFalse(any('visualiz' in n or 'mosaic' in n for n in names))
        for node in ('camera_objects', 'camera_lane', 'camera_traffic_light'):
            self.assertEqual(config.params['/' + node + '/initial_mode'].value, 2)
        self.assertEqual(config.params['/camera_objects/conf_thres'].value, .4)
        self.assertEqual(config.params['/camera_objects/nms_iou'].value, .5)
        self.assertEqual(config.params['/camera_lane/target_hz'].value, 15.)
        self.assertEqual(config.params['/camera_bev_fusion/max_fused_hz'].value, 15.)
        self.assertEqual(config.params['/camera_objects/model_kind'].value, 'yolo')
        self.assertEqual(config.params['/camera_lane/backend'].value,'twinlite_tensorrt')
        self.assertEqual(config.params['/camera_lane/model_config'].value,'large')
        self.assertTrue(config.params['/camera_lane/checkpoint'].value.endswith('/twinlite-large-v7-e8.pt'))
        self.assertTrue(config.params['/camera_lane_vectorizer/imu_motion/enabled'].value)
        self.assertTrue(config.params['/camera_lane_vectorizer/imu_motion/corrected_joint_width_fit'].value)
        self.assertEqual(config.params['/camera_lane_vectorizer/imu_motion/corrected_width_prior_strength'].value, 0.)

    def test_fixed_plane_recovery_overrides_only_the_attitude_policy(self):
        config = self.config('_imu_compare.launch', 'enabled:=false')
        self.assertFalse(config.params['/camera_lane_vectorizer/imu_motion/enabled'].value)
        self.assertEqual(config.params['/camera_lane/backend'].value, 'twinlite_tensorrt')
        self.assertEqual(config.params['/camera_lane/model_config'].value, 'large')
        self.assertEqual(config.params['/camera_lane_vectorizer/ground_plane/z_at_origin_m'].value, -.36)

    def test_explicit_standby_keeps_three_workers_loaded_without_inference(self):
        config = self.config('camera.launch', 'mode:=standby')
        for node in ('camera_objects', 'camera_lane', 'camera_traffic_light'):
            self.assertEqual(config.params['/' + node + '/initial_mode'].value, 0)

    def test_camera_and_planner_defaults_have_only_one_signal_producer(self):
        planner = LAUNCH.parents[1] / 'asmc_planner/launch'
        for name in ('autonomy_no_bridges_v2.launch', 'mode_control_integration.launch'):
            config = roslaunch.config.load_config_default([
                (str(LAUNCH / 'camera.launch'), []), (str(planner / name), [])], None, verbose=False)
            signals = [n for n in config.nodes if n.type == 'traffic_light_node.py']
            self.assertEqual([n.name for n in signals], ['camera_traffic_light'], name)
            self.assertEqual(config.params['/camera_traffic_light/initial_mode'].value, 2)

    def test_pytorch_recovery_is_explicit_and_uses_the_same_input_sizes(self):
        profile=LAUNCH.parent/'config/camera/lane_runtime_pytorch.yaml'
        config=self.config('_runtime.launch','lane_runtime_config:='+str(profile))
        self.assertEqual(config.params['/camera_lane/backend'].value,'twinlite')
        self.assertEqual(config.params['/camera_lane/model_config'].value,'large')
        self.assertTrue(config.params['/camera_lane/checkpoint'].value.endswith('/twinlite-large-v7-e8.pt'))
        self.assertEqual(config.params['/camera_lane/input_hw'].value,[480,640])
        self.assertEqual(config.params['/camera_lane/input_hw_by_view/front'].value,[544,960])

    def test_large_default_and_medium_recovery_resolve_separate_engine_manifests(self):
        from asmc_camera.camera.backends.twinlite_tensorrt import load_manifest
        for profile, scale in [('lane_runtime.yaml', 'large'), ('lane_runtime_large.yaml', 'large'),
                               ('lane_runtime_medium.yaml', 'medium')]:
            config = self.config('_runtime.launch', 'lane_runtime_config:=' +
                                 str(LAUNCH.parent/'config/camera'/profile))
            values = {k[len('/camera_lane/'):]: v.value for k, v in config.params.items()
                      if k.startswith('/camera_lane/')}
            self.assertEqual(values['model_config'], scale)
            manifest, directory = load_manifest(values['engine_manifest'], values['checkpoint'],
                tuple(values['input_hw']), {'front': tuple(values['input_hw_by_view/front'])}, scale)
            self.assertEqual(directory, LAUNCH.parent/'models/lane_segmentation')
            self.assertEqual([row['max_batch'] for row in manifest['engines']], [1, 2])

    def test_twinlite_candidate_resolves_local_engines_in_both_process_layouts(self):
        from asmc_camera.camera.backends.twinlite_tensorrt import load_manifest
        profile = LAUNCH.parent / 'config/camera/lane_runtime_tensorrt.yaml'
        for layout, prefix in [('separate', '/camera_lane/'),
                               ('shared', '/camera_runtime/components/lane/')]:
            config = self.config('_runtime.launch', 'implementation:=' + layout,
                                 'lane_runtime_config:=' + str(profile))
            params = {k[len(prefix):]: v.value for k, v in config.params.items() if k.startswith(prefix)}
            self.assertEqual(params['backend'], 'twinlite_tensorrt')
            self.assertEqual(params['checkpoint'],str(LAUNCH.parent/'models/lane_segmentation/twinlite-medium-v6.pt'))
            self.assertTrue(params['gpu_preprocess'] and params['lazy_outputs'])
            self.assertEqual(params['input_hw'], [480, 640])
            self.assertEqual(params['input_hw_by_view/front'], [544, 960])
            manifest, directory = load_manifest(params['engine_manifest'], params['checkpoint'],
                (480, 640), {'front': (544, 960)}, 'medium')
            self.assertEqual(directory, LAUNCH.parent / 'models/lane_segmentation')
            self.assertEqual([r['max_batch'] for r in manifest['engines']], [1, 2])

    def test_off_has_no_producers(self):
        self.assertFalse(self.config('camera.launch', 'mode:=off').nodes)

    def test_explicit_backend_profile_overrides_the_yolo_default(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / 'explicit-backend.yaml'
            profile.write_text('model_kind: rfdetr\n')
            config = self.config('camera_objects.launch', 'config:=' + str(profile))
            self.assertEqual(config.params['/camera_object_detector/model_kind'].value, 'rfdetr')

    def test_shared_workers_keep_lane_postprocessing_and_explicit_namespaces(self):
        config = self.config('_runtime.launch', 'implementation:=shared', 'mode:=active')
        names = {n.name for n in config.nodes}
        self.assertIn('camera_runtime', names)
        self.assertIn('camera_lane_vectorizer', names)
        self.assertFalse({'camera_objects', 'camera_lane'} & names)
        self.assertIn('camera_traffic_light', names)
        self.assertEqual(config.params['/camera_runtime/models'].value, ['objects', 'lane'])
        self.assertEqual(config.params['/camera_runtime/components/lane/target_hz'].value, 15.)

    def test_shared_traffic_only_does_not_start_an_empty_gpu_runtime(self):
        config = self.config('_runtime.launch', 'implementation:=shared', 'models:=traffic_light')
        self.assertEqual({n.name for n in config.nodes}, {'camera_traffic_light'})

    def test_package_weights_resolve_without_external_model_mount(self):
        for detector, filename in [('e8','yolo26l-finetuned_e8.engine'),('pretrained','yolo26l-pretrained_coco7.engine')]:
            config = self.config('camera.launch','models:=objects','detector:='+detector)
            path = Path(config.params['/camera_objects/weights'].value)
            self.assertEqual(path, LAUNCH.parent/'models/object_detection'/filename)
            self.assertTrue(path.is_file())
            self.assertFalse(path.is_symlink())

    def test_models_order_and_pretrained_selection(self):
        config = self.config('camera.launch', 'models:=traffic_light,objects', 'detector:=pretrained')
        self.assertEqual({n.name for n in config.nodes}, {'camera_traffic_light', 'camera_objects'})
        self.assertEqual(config.params['/camera_objects/deployment_profile'].value, 'pretrained_coco7')

    def test_invalid_public_arguments_are_rejected(self):
        for arg in ('mode:=bad', 'models:=objects,bad', 'models:=objects,objects',
                    'models:=objects,', 'models:=objectslane', 'models:=all,objects', 'detector:=bad'):
            with self.subTest(arg=arg), self.assertRaises(roslaunch.RLException):
                self.config('camera.launch', arg)

    def test_view_rejects_unknown_model_names(self):
        with self.assertRaises(roslaunch.RLException):
            self.config('view.launch','models:=objectslane')

    def test_view_never_starts_an_inference_producer(self):
        config = self.config('view.launch')
        names = {n.type for n in config.nodes}
        self.assertFalse(names & {'camera_runtime_node.py', 'object_detection_node.py',
                                 'semantic_inference_node.py', 'traffic_light_node.py'})
        self.assertTrue(config.nodes)

    def test_normal_lane_view_renders_the_published_family(self):
        config=self.config('view.launch','models:=lane')
        types={n.type for n in config.nodes}
        self.assertIn('family_view_node.py',types)
        self.assertNotIn('lane_vectorizer_node.py',types)
        self.assertNotIn('lane_vector_visualizer_node.py',types)

    def test_object_view_uses_one_renderer_and_keeps_legacy_diagnostics_explicit(self):
        config = self.config('view.launch','models:=objects')
        self.assertEqual([n.type for n in config.nodes],['object_view_node.py'])
        legacy = self.config('visualization.launch','camera_objects:=true','objects_debug_raw:=true')
        self.assertEqual(len(legacy.nodes),4)
        self.assertNotIn('object_view_node.py',{n.type for n in legacy.nodes})


if __name__ == '__main__':
    unittest.main()
