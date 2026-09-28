import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np
from audit_aim import decode_cloud, safe_path


class AimEvidenceTest(unittest.TestCase):
    def test_ros_cloud_stride_and_axes_preserved(self):
        def text(s):return struct.pack('<I',len(s))+s.encode()
        payload=struct.pack('<III',7,12,34)+text('Lidar3D-5')+struct.pack('<III',1,2,4)
        for i,name in enumerate(('x','y','z','intensity')):
            payload+=text(name)+struct.pack('<IBI',i*4,7,1)
        data=struct.pack('<ffffIffffI',1,2,-1.48,9,0,-3,-4,-1,12,0)
        payload+=struct.pack('<BIII',0,20,40,40)+data+struct.pack('<B',0)
        cloud,header=decode_cloud(payload)
        np.testing.assert_array_equal(cloud,np.array([[1,2,-1.48,9],[-3,-4,-1,12]],dtype='<f4'))
        self.assertEqual(header['stamp_ns'],12000000034)

    def test_source_path_cannot_escape_dataset(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):safe_path(Path(d),'../outside')

    def test_truncated_payload_fails(self):
        with self.assertRaises((ValueError,struct.error)):decode_cloud(b'\x00')
