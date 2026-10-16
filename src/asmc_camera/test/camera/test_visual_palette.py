import unittest
from unittest.mock import patch
import numpy as np
from asmc_camera.camera.visualization import render_panel


class PaletteParityTest(unittest.TestCase):
    def test_native_palette_matches_boolean_painting_without_mutating_inputs(self):
        image=np.zeros((2,8,3),np.uint8)
        probability=np.array([[-1,0,.25,.5,.75,1,np.inf,np.nan]]*2,np.float32)
        classes=np.array([[0,1,2,3,0,1,2,3]]*2,np.uint8)
        original=probability.copy()
        for mode in ('lane','road_marking'):
            for threshold in (0.,.5,1.):
                target=probability if mode=='lane' else classes
                active=np.clip(probability,0,1)>=threshold
                drive=np.zeros_like(image);drive[active]=(40,200,40)
                secondary=np.zeros_like(image)
                if mode=='lane':secondary[active]=(0,230,255)
                else:
                    for k,color in {1:(255,255,255),2:(0,230,255),3:(0,0,255)}.items():secondary[classes==k]=color
                with patch('asmc_camera.camera.visualization._label_tile',side_effect=lambda im,*args:im.copy()):
                    actual=render_panel(image,probability,target,'front',drivable_threshold=threshold,
                        lane_threshold=threshold,secondary_mode=mode)
                np.testing.assert_array_equal(actual,np.concatenate([image,drive,secondary]))
        np.testing.assert_array_equal(probability,original)
        self.assertFalse(image.any())


if __name__=='__main__':unittest.main()
