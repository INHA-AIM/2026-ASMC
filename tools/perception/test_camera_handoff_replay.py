import copy
import unittest

from tools.perception.compare_camera_handoff_replay import summarize


class ReplayParityTest(unittest.TestCase):
    def fixture(self):
        return [dict(stamp_ns=42, ready=True, objects=[dict(id=7, **{
            'class': 'vehicle', 'raw_class': 'truck', 'center': [1.0, 2.0, 3.0]})])]

    def test_raw_name_and_group_differences_cannot_be_hidden(self):
        for key, value in [('raw_class', 'bus'), ('class', 'unknown')]:
            candidate = self.fixture()
            candidate[0]['objects'][0][key] = value
            self.assertFalse(summarize(self.fixture(), candidate)['passed'])

    def test_stamp_object_loss_and_geometry_regressions_fail(self):
        variants = [self.fixture() for _ in range(3)]
        variants[0][0]['stamp_ns'] += 1
        variants[1][0]['objects'] = []
        variants[2][0]['objects'][0]['center'][0] += 1e-4
        for candidate in variants:
            self.assertFalse(summarize(self.fixture(), candidate)['passed'])
        self.assertFalse(summarize([], [])['passed'])

    def test_only_small_numeric_error_is_tolerated(self):
        candidate = copy.deepcopy(self.fixture())
        candidate[0]['objects'][0]['center'][0] += 1e-7
        report = summarize(self.fixture(), candidate)
        self.assertTrue(report['passed'])
        self.assertEqual((report['frames'], report['ready'], report['objects']), (1, 1, 1))
