"""Input-freezing regressions (no ROS import needed)."""
import unittest
from types import SimpleNamespace
import numpy as np
from run_geometry import float4_cloud, geometry_only


class GeometryInputTest(unittest.TestCase):
    def test_organized_cloud_row_padding_is_not_a_point(self):
        data=np.array([[1,2,3,4,99,99],[5,6,7,8,99,99]],dtype='<f4').tobytes()
        fields=[SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))]
        m=SimpleNamespace(is_bigendian=False,height=2,width=1,row_step=24,point_step=16,fields=fields,data=data)
        np.testing.assert_array_equal(float4_cloud(m),[[1,2,3,4],[5,6,7,8]])

    def test_time_measurement_excluded_geometry_retained(self):
        self.assertEqual(geometry_only({'timing_ms':{'a':1},'obb':{'corners':[1]},'preprocessing_ms':2}),
                         {'obb':{'corners':[1]}})
