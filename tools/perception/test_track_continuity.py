import unittest
from tools.perception.analyze_track_continuity import summarize


def frame(stamp, objects):
    return dict(stamp_ns=stamp, objects=objects)


class TrackContinuityTest(unittest.TestCase):
    def test_reacquisition_and_class_loss_are_reported_as_interruptions(self):
        report = summarize([frame(100, [[1, 'vehicle', 'car']]), frame(200, []),
                            frame(300, [[1, 'unknown', 'unknown']])])
        self.assertEqual(1, report['presence_reacquisitions'])
        self.assertEqual(1, report['internal_missing_output_frames'])
        self.assertEqual(1, report['classification_loss_transitions'])

    def test_new_id_is_not_claimed_as_same_physical_object(self):
        report = summarize([frame(100, [[1, 'vehicle', 'car']]),
                            frame(200, [[2, 'vehicle', 'car']])])
        self.assertEqual(2, report['track_ids'])
        self.assertEqual('not_measured_without_independent_actor_identity',
                         report['physical_object_fragmentation'])

    def test_duplicate_and_reversed_observations_are_rejected(self):
        for rows in ([frame(100, []), frame(100, [])],
                     [frame(200, []), frame(100, [])],
                     [frame(100, [[1, 'car', 'car'], [1, 'car', 'car']])]):
            with self.assertRaises(ValueError):
                summarize(rows)

    def test_empty_and_continuous_known_track(self):
        self.assertEqual(0, summarize([])['track_ids'])
        report = summarize([frame(100000000, [[1, 'car', 'car']]),
                            frame(200000000, [[1, 'car', 'car']])])
        self.assertEqual(0, report['presence_reacquisitions'])
        self.assertEqual(100, report['tracks'][0]['max_observation_gap_ms'])


if __name__ == '__main__':
    unittest.main()
