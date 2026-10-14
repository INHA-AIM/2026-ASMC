from pathlib import Path
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
        self.assertEqual(config.params['/camera_objects/nms_iou'].value, .7)

    def test_off_has_no_producers(self):
        self.assertFalse(self.config('camera.launch', 'mode:=off').nodes)

    def test_shared_workers_keep_lane_postprocessing_and_explicit_namespaces(self):
        config = self.config('_runtime.launch', 'implementation:=shared', 'mode:=active')
        names = {n.name for n in config.nodes}
        self.assertIn('camera_runtime', names)
        self.assertIn('camera_lane_vectorizer', names)
        self.assertFalse({'camera_objects', 'camera_lane', 'camera_traffic_light'} & names)
        self.assertEqual(config.params['/camera_runtime/models'].value, ['objects', 'lane', 'traffic_light'])
        self.assertEqual(config.params['/camera_runtime/components/lane/target_hz'].value, 10.)

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


if __name__ == '__main__':
    unittest.main()
