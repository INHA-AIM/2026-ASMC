import unittest
from copy import deepcopy
from analyze_coco7_weather import stratify, weather_tag


class WeatherEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.rows = [dict(image='sunny.png', run='capture_sunny_sim11am', view='front',
                          annotation_complete=True, source_sha256=dict(intensity='sha'),
                          annotations=[dict(source_class_name='car', subtype_verified=True,
                                            xyxy=[0, 0, 10, 10])])]
        self.predictions = [dict(image='sunny.png', run='capture_sunny_sim11am', view='front',
                                 source_sha256=dict(intensity='sha'),
                                 predictions=[dict(source_class_name='car', confidence=.68,
                                                   xyxy=[0, 0, 10, 10])])]

    def test_weather_is_explicit(self):
        self.assertEqual(weather_tag('capture_foggy_sim1pm'), 'foggy')
        for name in ('capture_unknown', 'capture_sunny_foggy'):
            with self.assertRaises(ValueError):
                weather_tag(name)

    def test_frozen_confidence_and_missing_classes(self):
        low = stratify(self.rows, self.predictions, .65)['sunny']
        high = stratify(self.rows, self.predictions, .7)['sunny']
        self.assertEqual(low['labels']['car']['tp'], 1)
        self.assertEqual(high['labels']['car']['fn'], 1)
        self.assertIsNone(low['labels']['dog']['f1'])

    def test_missing_duplicate_and_wrong_rgb_fail(self):
        for predictions in ([], self.predictions * 2):
            with self.assertRaises(ValueError):
                stratify(self.rows, predictions, .7)
        self.predictions[0]['source_sha256']['intensity'] = 'other'
        with self.assertRaises(ValueError):
            stratify(self.rows, self.predictions, .7)

    def test_weather_strata_do_not_share_objects_or_false_positives(self):
        fog_gt = deepcopy(self.rows[0])
        fog_prediction = deepcopy(self.predictions[0])
        for row in (fog_gt, fog_prediction):
            row.update(image='foggy.png', run='capture_foggy_sim1pm')
        fog_prediction['predictions'][0]['source_class_name'] = 'truck'
        measured = stratify(self.rows + [fog_gt], self.predictions + [fog_prediction], .65)
        self.assertEqual(measured['sunny']['labels']['car']['tp'], 1)
        self.assertEqual(measured['sunny']['false_positives'], 0)
        self.assertEqual(measured['foggy']['labels']['car']['fn'], 1)
        self.assertEqual(measured['foggy']['labels']['truck']['fp'], 1)


if __name__ == '__main__':
    unittest.main()
