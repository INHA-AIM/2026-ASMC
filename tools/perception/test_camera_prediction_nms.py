import unittest
from camera_prediction_nms import suppress


def box(name='person', xyxy=None, score=.8):
    return dict(class_name=name, xyxy=xyxy or [0, 0, 10, 10], confidence=score)


class NmsTest(unittest.TestCase):
    def test_same_class_duplicate_only(self):
        a, b, c = box(), box(score=.7), box('bicycle')
        kept, removed = suppress([a, b, c])
        self.assertEqual(kept, [a, c]); self.assertEqual(removed[0]['index'], 1)
        self.assertEqual(b['confidence'], .7)

    def test_adjacent_and_boundary(self):
        a, b = box(), box(xyxy=[10, 0, 20, 10])
        self.assertEqual(len(suppress([a,b])[0]), 2)
        self.assertEqual(len(suppress([a,a], threshold=1)[0]), 2)

    def test_threshold_tie_empty(self):
        a, b = box(score=.4), box(score=.4)
        self.assertIs(suppress([a,b])[0][0], a)
        self.assertEqual(suppress([box(score=.39)])[0], [])
        self.assertEqual(suppress([]), ([], []))

    def test_invalid_values(self):
        with self.assertRaises(ValueError): suppress([box(score=float('nan'))])
        with self.assertRaises(ValueError): suppress([box(xyxy=[10,0,0,10])])

    def test_does_not_hide_possible_neighbour_suppression(self):
        # Highly occluded neighbours can overlap above the NMS threshold.
        # They must be counted as lost TP by evaluation, not declared ignore.
        first, second = box(), box(xyxy=[.5,0,10.5,10],score=.7)
        kept, removed = suppress([first,second])
        self.assertEqual(len(kept),1)
        self.assertEqual(len(removed),1)
        self.assertNotIn('ignore',second)


if __name__ == '__main__': unittest.main()
