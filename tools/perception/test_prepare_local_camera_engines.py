import unittest
import hashlib
import tempfile
from pathlib import Path

from prepare_local_camera_engines import nested_params, object_source


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

    def test_selected_e3_checkpoint_is_used_and_head_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'e3.pt').write_bytes(b'e3-weights')
            manifest = {'files': [
                {'profile': 'finetuned_e8', 'path': 'e8.pt', 'sha256': 'unused'},
                {'profile': 'collect_e3', 'path': 'e3.pt',
                 'sha256': hashlib.sha256(b'e3-weights').hexdigest()}]}
            obj = dict(deployment_profile='collect_e3', head_profile='mission15_one2one_nms')
            self.assertEqual(object_source(obj, manifest, root), root/'e3.pt')
            obj['head_profile'] = 'mission16_one2one_nms'
            with self.assertRaisesRegex(RuntimeError, 'head profile'):
                object_source(obj, manifest, root)
            obj['head_profile'] = 'mission15_one2one_nms'
            (root/'e3.pt').write_bytes(b'wrong-weights')
            with self.assertRaisesRegex(RuntimeError, 'checksum mismatch'):
                object_source(obj, manifest, root)


if __name__ == '__main__':
    unittest.main()
