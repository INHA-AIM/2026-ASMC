import unittest
import numpy as np
from prepare_camera_backgrounds import has_ambiguous_map_objects
class MapBackgroundTest(unittest.TestCase):
 def test_standing_object_is_not_proven_negative(self):
  s=np.zeros((5,5,3),dtype=np.uint8);ego=np.zeros((5,5),bool)
  self.assertFalse(has_ambiguous_map_objects(s,ego))
  s[2,2]=[37,178,113]
  self.assertTrue(has_ambiguous_map_objects(s,ego))
  ego[2,2]=True
  self.assertFalse(has_ambiguous_map_objects(s,ego))
if __name__=='__main__':unittest.main()
