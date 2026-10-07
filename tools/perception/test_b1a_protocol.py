import unittest
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from b1a_protocol import frozen_events, gps_action, legacy_boxes, jobs, test_launch, launch_values
from analyze_b1a_matrix import offline, realtime


class ProtocolTest(unittest.TestCase):
    def row(self, **extra):
        return dict(valid=True, frame_id='camera_front', stamp_ns=100, ros_ns=120,
                    arrival=1.2, boxes=[dict(label='car', confidence=.7, xyxy=[-.9, 1.9, 3.2, 4.8])], **extra)

    def test_half_open_gps_window_and_normal_recovery(self):
        self.assertEqual([gps_action('invalid', t, [6, 20]) for t in [5.999,6,19.999,20]],
                         ['normal','invalid','invalid','normal'])
        self.assertEqual(gps_action('missing',10,[6,20]),'missing')
        self.assertEqual(gps_action('normal',10,[6,20]),'normal')
        with self.assertRaises(ValueError): gps_action('other',10,[6,20])

    def test_legacy_coordinates_are_truncated_without_mutating_source(self):
        r=self.row()
        self.assertEqual(legacy_boxes(r),[dict(label='car',xyxy=[0,1,3,4])])
        self.assertEqual(r['boxes'][0]['xyxy'],[-.9,1.9,3.2,4.8])

    def test_confidence_boundary_classes_and_invalid(self):
        r=self.row(); r['boxes'] += [dict(label='person',confidence=.699999,xyxy=[0,0,1,1]),dict(label='chair',confidence=.99,xyxy=[0,0,1,1])]
        self.assertEqual(len(legacy_boxes(r)),1)
        r['valid']=False; self.assertEqual(legacy_boxes(r),[])
        r['valid']=True; r['frame_id']='camera_left'; self.assertEqual(legacy_boxes(r),[])

    def test_frozen_events_keep_availability_and_reject_duplicate_or_backward_clock(self):
        r=self.row(); empty=self.row(); empty.update(stamp_ns=200,ros_ns=220,boxes=[])
        events=frozen_events(dict(outputs=dict(yolo=[r,empty])))
        self.assertEqual([e['available_ns'] for e in events],[120,220])
        self.assertEqual(events[1]['boxes'],[])
        with self.assertRaises(ValueError): frozen_events(dict(outputs=dict(yolo=[r,r])))
        empty['ros_ns']=110
        with self.assertRaises(ValueError): frozen_events(dict(outputs=dict(yolo=[r,empty])))

    def test_test_launch_retains_main_settings_and_quarantines_controls(self):
        source='<launch><arg name="x" default="true"/><node name="Main" pkg="Main" type="main"><param name="x" value="$(arg x)"/></node><node name="yolo_detection_node" pkg="Main" type="yolo"/></launch>'
        t=ET.fromstring(test_launch(source))
        self.assertEqual([n.attrib['name'] for n in t.findall('node')],['Main'])
        remaps={m.attrib['from']:m.attrib['to'] for m in t.find('node').findall('remap')}
        self.assertEqual(remaps['/ctrl_cmd'],'/b1a/discarded_ctrl_cmd')
        self.assertEqual(remaps['/camera/image/compressed'],'/b1a/front_forwarded')
        self.assertEqual(launch_values(source),{'x':True})

    def test_protocol_has_complete_rotating_repetitions(self):
        root=Path(__file__).resolve().parents[2]
        config=yaml.safe_load((root/'config/experiments/camera_lidar_b1a_v1.yaml').read_text())
        matrix=jobs(config,'realtime')
        self.assertEqual(len(matrix),36)
        self.assertEqual(len(jobs(config,'offline')),8)
        self.assertEqual([r['profile'] for r in matrix[:2]],['T1','T3'])
        self.assertEqual([r['profile'] for r in matrix[12:14]],['T3','T1'])

    def test_removal_accounting_detects_missing_points_without_GT_claim(self):
        frame=dict(stamp=100.0,denied=True,A=dict(raw=20,roi=15,voxel=10,ransac=7,objs=[]),
                   B=dict(active=True,counts=[10,1,2,3,4,1,2],protect=[True,50.0],ground=['new']))
        self.assertEqual(offline([frame])['stage_accounting_failures'],[])
        frame['B']['counts'][4]=3
        self.assertEqual(offline([frame])['stage_accounting_failures'],[100.0])
        self.assertTrue(offline([frame])['counts_are_not_GT_accuracy'])

    def test_replay_phases_use_source_stamp_and_do_not_turn_missing_GPS_into_invalid(self):
        ns=lambda t:int((100+t)*1e9)
        observation=dict(inputs=dict(lidar=[dict(stamp_ns=ns(t)) for t in (5,10,21)],gps=[]),
                         outputs=dict(nogps=[dict(stamp_ns=ns(10),arrival=999,objects=[{}])]),diagnostics=[])
        report=dict(job=dict(bag_id='shadow'))
        data=realtime(observation,report,dict(bags=dict(shadow=dict(injection_window_sec=[6,20]))),100)
        self.assertEqual(data['phases']['window']['publications']['nogps'],1)
        self.assertEqual(data['phases']['window']['inputs']['gps'],0)
        self.assertIsNone(data['active_first_source_sec'])
        self.assertEqual(data['first_three_restored_gps_stamps_ns'],[])


if __name__ == '__main__': unittest.main()
