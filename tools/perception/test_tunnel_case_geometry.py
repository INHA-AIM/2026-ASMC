import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from tunnel_case_geometry import rotation,projection,project_leveled,clouds


class GeometryTest(unittest.TestCase):
    def test_leveling_inverse_and_optical_mount_axes(self):
        xyz=np.array([[8.,1.,-.5],[12.,-3.,2.]])
        r=rotation(.15,-.2)
        np.testing.assert_allclose((xyz@r.T)@r,xyz,atol=1e-12)
        sensor=dict(x=0,y=0,z=0,roll=0,pitch=0,yaw=0)
        camera=dict(sensor,fx=640,fy=640,cx=640,cy=360)
        p=projection(dict(Camera=camera,LiDAR=sensor))
        uv,valid=project_leveled(xyz@r.T,.15,-.2,p)
        np.testing.assert_allclose(uv,[[560,400],[800,360-640*2/12]])
        self.assertTrue(valid.all())
        _,valid=project_leveled(np.array([[-1.,0,0]]),0,0,p)
        self.assertFalse(valid[0])

    def test_pinned_camera_projection_matches_original_cpp_log(self):
        camera=dict(x=1.9,y=0,z=1.2,roll=0,pitch=2,yaw=0,fx=640,fy=640,cx=640,cy=360)
        lidar=dict(x=.8,y=0,z=1.45,roll=0,pitch=0,yaw=0)
        p=projection(dict(Camera=camera,LiDAR=lidar))
        np.testing.assert_allclose(p,[[639.61,-640,-22.3357,-709.155],
                                     [337.445,0,-652.174,-534.233],
                                     [.999391,0,-.0348995,-1.10805]],atol=.001)

    def test_truncated_or_missing_frame_fails_instead_of_showing_wrong_cloud(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'clouds.bin'
            path.write_bytes(struct.pack('<idii',0,100.,1,1)+b'\0'*27)
            with self.assertRaisesRegex(ValueError,'truncated'):clouds(path,{0})
            path.write_bytes(struct.pack('<idii',0,100.,0,0))
            with self.assertRaisesRegex(ValueError,'missing'):clouds(path,{1})


if __name__=='__main__':unittest.main()
