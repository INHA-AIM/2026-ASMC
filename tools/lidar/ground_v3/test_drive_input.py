#!/usr/bin/env python3
"""Small read-only admission/launch regressions; no simulator connection."""
from pathlib import Path
from types import SimpleNamespace as N
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from extract_drive_gt import field, geometry, key
from extract_tunnel_bag import xyzi


def cloud():
    data=bytearray(2*48)
    values=np.array([[1.,2.,3.,4.],[5.,6.,7.,8.],[9.,10.,11.,12.],[13.,14.,15.,16.]],dtype='<f4')
    for row in range(2):
        for col in range(2):
            begin=row*48+col*20
            data[begin:begin+16]=values[row*2+col].tobytes()
            data[begin+16:begin+20]=np.array([100+row*2+col],dtype='<u4').tobytes()
    return N(height=2,width=2,row_step=48,point_step=20,is_bigendian=False,data=bytes(data),
        fields=[N(name=k,datatype=7,count=1,offset=i*4) for i,k in enumerate(['x','y','z','intensity'])]+
        [N(name='semantic_id',datatype=6,count=1,offset=16)]),values


class AdmissionTests(unittest.TestCase):
    def test_padded_layout_and_intensity(self):
        msg,values=cloud();np.testing.assert_array_equal(xyzi(msg),values)
        np.testing.assert_array_equal(geometry(msg),values[:,:3])
        np.testing.assert_array_equal(field(msg,'semantic_id','<u4'),[100,101,102,103])
    def test_correspondence_is_exact_and_ordered(self):
        msg,_=cloud();x=geometry(msg)
        self.assertEqual(key(x),key(x.copy()))
        self.assertNotEqual(key(x),key(x[::-1]))
        y=x.copy();y[0,0]=np.nextafter(y[0,0],np.float32(np.inf));self.assertNotEqual(key(x),key(y))
    def test_wrong_label_storage_rejected(self):
        msg,_=cloud();msg.fields[-1].datatype=7
        with self.assertRaises(ValueError):field(msg,'semantic_id','<u4')
    def test_wrong_endian_rejected(self):
        msg,_=cloud();msg.is_bigendian=True
        with self.assertRaises(ValueError):geometry(msg)
    def test_replay_cannot_publish_operating_input(self):
        root=Path(__file__).resolve().parents[3]
        tree=ET.parse(root/'src/asmc_lidar/launch/ground_compare_recording.launch')
        player=next(n for n in tree.findall('node') if n.attrib['pkg']=='rosbag')
        topics=player.attrib['args'].split('--topics',1)[1].split()
        self.assertGreater(len(topics),10)
        self.assertTrue(all(t.startswith('/lidar_candidate/ground_live/') for t in topics))
        self.assertNotIn('--clock',player.attrib['args'])
        self.assertFalse(tree.findall('include'))


if __name__=='__main__':unittest.main()
