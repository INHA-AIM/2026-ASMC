import unittest
import numpy as np
from asmc_camera.bev.calibration import calibrations_from_dict
from study_seg_lidar_support import project_raw, point_support


class SupportTest(unittest.TestCase):
    def test_raw_mount_projection_and_mask_subset(self):
        document=dict(ground_plane=dict(frame_id='base_link',model='plane',z_at_origin_m=0,dz_dx=0,dz_dy=0,source='fixture'),
                      cameras=dict(front=dict(sensor_id=1,width=100,height=80,horizontal_fov_deg=90,
                      translation_m=[1,0,1],rotation_deg=[0,0,0],topics=dict(image='',drivable='',road_marking='',road_marking_confidence=''))))
        camera=calibrations_from_dict(document)['front']
        lidar=dict(pos=dict(x='1',y='0',z='1'),rot=dict(roll='0',pitch='0',yaw='0'))
        pixels,visible,depth=project_raw(np.array([[10,0,0],[10,2,0],[-10,0,0]],float),lidar,camera)
        np.testing.assert_allclose(pixels[:2],[[50,40],[40,40]])
        self.assertEqual([True,True,False],visible.tolist())
        mask=np.zeros((80,100),bool);mask[:,45:]=True
        inside,kept=point_support(pixels,visible,[0,0,100,80],mask)
        self.assertEqual([0,1],inside.tolist());self.assertEqual([0],kept.tolist())


if __name__=='__main__':unittest.main()
