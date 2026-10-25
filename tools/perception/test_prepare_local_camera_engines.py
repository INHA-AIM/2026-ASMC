import unittest

from prepare_local_camera_engines import nested_params


class CameraLaunchParametersTest(unittest.TestCase):
    def test_roslaunch_view_shape_reaches_backend_as_dictionary(self):
        params = nested_params({'input_hw': [480, 640],
                                'input_hw_by_view/front': [544, 960],
                                'camera_topics/front': '/front/image',
                                'camera_topics/left': '/left/image'})
        wanted = {tuple(params['input_hw'])} | {
            tuple(value) for value in params['input_hw_by_view'].values()}
        self.assertEqual(wanted, {(480, 640), (544, 960)})
        self.assertEqual(params['camera_topics']['left'], '/left/image')

    def test_scalars_and_existing_dictionary_are_preserved(self):
        params = nested_params({'model_config': 'large',
                                'input_hw_by_view': {'front': [544, 960]}})
        self.assertEqual(params['model_config'], 'large')
        self.assertEqual(params['input_hw_by_view'], {'front': [544, 960]})


if __name__ == '__main__':
    unittest.main()
