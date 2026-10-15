from pathlib import Path
import tempfile
import unittest
import roslaunch

LAUNCH = Path(__file__).resolve().parents[2] / 'launch'


class CameraEntrypointsTest(unittest.TestCase):
    def config(self, filename='camera.launch', *args):
        return roslaunch.config.load_config_default([(str(LAUNCH / filename), list(args))], None, verbose=False)

    def test_public_default_loads_three_independent_standby_workers(self):
        config = self.config()
        names = {n.name for n in config.nodes}
        self.assertTrue({'camera_objects', 'camera_lane', 'camera_traffic_light'} <= names)
        self.assertNotIn('camera_runtime', names)
        self.assertFalse(any('visualiz' in n or 'mosaic' in n for n in names))
        for node in ('camera_objects', 'camera_lane', 'camera_traffic_light'):
            self.assertEqual(config.params['/' + node + '/initial_mode'].value, 0)
        self.assertEqual(config.params['/camera_objects/conf_thres'].value, .4)
        self.assertEqual(config.params['/camera_objects/nms_iou'].value, .5)
        self.assertEqual(config.params['/camera_lane/target_hz'].value, 15.)
        self.assertEqual(config.params['/camera_bev_fusion/max_fused_hz'].value, 15.)
        self.assertEqual(config.params['/camera_objects/model_kind'].value, 'yolo')

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

    def test_object_view_uses_one_renderer_and_keeps_legacy_diagnostics_explicit(self):
        config = self.config('view.launch','models:=objects')
        self.assertEqual([n.type for n in config.nodes],['object_view_node.py'])
        legacy = self.config('visualization.launch','camera_objects:=true','objects_debug_raw:=true')
        self.assertEqual(len(legacy.nodes),4)
        self.assertNotIn('object_view_node.py',{n.type for n in legacy.nodes})


if __name__ == '__main__':
    unittest.main()
