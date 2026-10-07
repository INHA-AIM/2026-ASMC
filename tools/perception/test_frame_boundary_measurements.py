import copy
from pathlib import Path
import unittest
import yaml
from tools.perception.frame_boundary_measurements import jobs, stamp_table, compare_local, map_gate, frame_totals


class MeasurementsTest(unittest.TestCase):
    def test_final_accounting_missing_or_duplicate_is_failure(self):
        log='[LIDAR_FRAME_TOTAL] processed=208 invalid=0 stamp_errors=0 source_stamp_ns=123000000'
        self.assertEqual(frame_totals(log)['processed'],208)
        for invalid in ('',log+'\n'+log):
            with self.assertRaises(ValueError):frame_totals(invalid)
    def row(self, stamp=1000000000, status=2):
        return dict(stamp_ns=stamp, arrival=10., status=status, objects=[], frame_id='base_link',
            stamp_provenance='bridge_receive_ms')

    def test_protocol_preserves_all_profiles_repetitions_and_subscriptions(self):
        config=yaml.safe_load((Path(__file__).resolve().parents[2]/
            'config/experiments/lidar_camera_frame_boundaries_v1.yaml').read_text())
        self.assertEqual(len(jobs(config,'state')),54)
        self.assertEqual(len(jobs(config,'cost')),24)
        self.assertEqual(len(jobs(config,'diagnostic')),6)
        self.assertEqual({j['subscribers'] for j in jobs(config,'cost')},{'none','local','map','both'})

    def test_zero_duplicate_regression_and_invalid_payload_rejected(self):
        for rows in ([self.row(0)],[self.row(),self.row()],
            [self.row(),self.row(1)]):
            with self.assertRaises(ValueError): stamp_table(rows)
        row=self.row(status=4);row['objects']=[{}]
        with self.assertRaises(ValueError): stamp_table([row])

    def test_missing_and_changed_observations_not_silently_ignored(self):
        a=[self.row(),self.row(1100000000)]
        self.assertFalse(compare_local(a,a[:1])['exact'])
        b=copy.deepcopy(a);b[1]['status']=3
        self.assertEqual(compare_local(a,b)['changed_ns'],[1100000000])

    def test_map_age_invalid_and_recovery_boundaries(self):
        gps=[dict(stamp_ns=t,arrival=1.,status=1,latitude=37.,longitude=126.,altitude=30.)
            for t in (200000000,400000000,600000000)]
        self.assertFalse(map_gate([self.row(900000000)],gps)['violations'])
        self.assertTrue(map_gate([self.row(900000001)],gps)['violations'])
        late=copy.deepcopy(gps)
        late[-1]['arrival']=9.
        self.assertTrue(map_gate([self.row(900000000)],late,
            [dict(stamp_ns=900000000,arrival=8.)])['violations'])
        gps.append(dict(gps[-1],stamp_ns=800000000,status=0,latitude=0.,longitude=0.))
        self.assertTrue(map_gate([self.row(800000000)],gps)['violations'])
        for t in (1000000000,1200000000,1400000000):
            gps.append(dict(gps[0],stamp_ns=t))
            self.assertEqual(bool(map_gate([self.row(t)],gps)['violations']),t!=1400000000)


if __name__=='__main__': unittest.main()
