from pathlib import Path
import unittest
import roslaunch
from asmc_camera.detection.mission import CLASSES, GROUPS
from asmc_camera.detection.classes import load_class_groups


class ModelProfilesTest(unittest.TestCase):
    def config(self, *args):
        launch = Path(__file__).resolve().parents[2] / 'launch/perception.launch'
        return roslaunch.config.load_config_default([(str(launch), list(args))], None, verbose=False)

    def test_e3_e8_and_pretrained_share_consumer_contract(self):
        for profile, count, head in [('collect_e3', 15, 'mission15_one2one_nms'),
                                     ('finetuned_e8', 16, 'mission16_one2one_nms'),
                                     ('pretrained_coco7', 7, 'legacy_coco7')]:
            config = self.config('profile:=planning', 'objects_model_profile:=' + profile)
            values = {key.rsplit('/', 1)[-1]: value.value for key, value in config.params.items()
                      if key.startswith('/camera_object_detector/')}
            self.assertEqual(profile, values['deployment_profile'])
            self.assertEqual(count, len(values['allowed_names']))
            self.assertEqual(head, values['head_profile'])
            self.assertEqual('group', values['label_mode'])
            self.assertEqual(['front', 'left', 'right'], values['views'])
            self.assertEqual(15., values['target_hz'])
            self.assertEqual(0, values['initial_mode'])
            self.assertTrue(values['hood_mask'] and values['side_ego_mask'])
            prefix = '/camera_object_detector/class_groups/'
            groups = load_class_groups(dict(
                profile=config.params[prefix + 'profile'].value,
                default_group=config.params[prefix + 'default_group'].value,
                groups={group: config.params[prefix + 'groups/' + group].value for group in GROUPS}))
            for raw in values['allowed_names']:
                self.assertEqual(next(group for group, names in GROUPS.items() if raw in names), groups.group_of(raw))
        default = self.config('profile:=debug')
        self.assertEqual('collect_e3', default.params['/camera_object_detector/deployment_profile'].value)
        self.assertEqual(2, default.params['/camera_object_detector/initial_mode'].value)

    def test_bad_model_profile_fails_and_confidence_is_explicit(self):
        with self.assertRaises(roslaunch.RLException):
            self.config('profile:=planning', 'objects_model_profile:=typo')
        config = self.config('profile:=planning', 'objects_conf_thres:=0.30')
        self.assertEqual(.3, config.params['/camera_object_detector/conf_thres'].value)


if __name__ == '__main__':
    unittest.main()
