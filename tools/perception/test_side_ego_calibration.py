import unittest
import numpy as np
from prepare_side_ego_masks import corner_component


class SideCalibrationTest(unittest.TestCase):
    def test_only_rgb_confirmed_corner_component_is_selected(self):
        semantic=np.ones((100,100,3),np.uint8)*127
        instance=np.ones_like(semantic)*255
        semantic[70:,70:]=0;instance[70:,70:]=0
        # Black terrain in semantic alone must not connect into the Ego mask.
        semantic[:70,70:]=0
        component=corner_component(semantic,instance,'left')
        self.assertEqual(900,int(component.sum()))
        self.assertFalse(component[5,80])

    def test_wrong_corner_or_giant_component_is_not_accepted(self):
        for value in [0,255]:
            image=np.full((100,100,3),value,np.uint8)
            with self.assertRaises(ValueError):corner_component(image,image,'right')


if __name__=='__main__':unittest.main()
