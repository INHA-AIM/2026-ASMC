import unittest,math,copy
from unittest.mock import patch
import numpy as np
from alignment import Series,body_transform
from generate import build,normalize_snapshots,records_frame
BASE=1792762250000000000
POLICY={"max_gap_ns":100000000,"dimension_break_m":.005,"max_reference_speed_mps":80.,"jump_slack_m":1.,"velocity":"model derivative; interval provenance"}
def state(ms,p=(0.,0.,0.),heading=0.,**kw):
    return dict(ns=BASE+int(ms)*1000000,p=list(p),heading=heading,**kw)
class AlignmentTests(unittest.TestCase):
    def test_stationary_object_moving_ego_ground_velocity(self):
        e=Series([state(0),state(100,(1,0,0))],POLICY).query(BASE+50000000)[0]
        o=Series([state(0,(10,0,0),offset=[1.,0.]),state(100,(10,0,0),offset=[1.,0.])],POLICY).query(BASE+50000000)[0]
        v=body_transform(e,o)
        self.assertAlmostEqual(v["x"],10.5)
        np.testing.assert_allclose([v["vx"],v["vy"]],[0,0])
    def test_constant_velocity_independent_timestamps(self):
        e=Series([state(0),state(100,(1,0,0))],POLICY).query(BASE+60000000)[0]
        o=Series([state(20,(10.04,0,0)),state(90,(10.18,0,0))],POLICY).query(BASE+60000000)[0]
        v=body_transform(e,o)
        self.assertAlmostEqual(v["x"],9.52);self.assertAlmostEqual(v["vx"],2.)
        self.assertEqual(o["velocity_a_ns"],BASE+20000000);self.assertEqual(o["velocity_b_ns"],BASE+90000000)
    def test_rotation_offset_after_heading_interpolation(self):
        tr=Series([state(0,offset=[2.,.5]),state(100,heading=math.pi/2,offset=[2.,.5])],POLICY)
        t=BASE+50000000;o=tr.query(t)[0]
        np.testing.assert_allclose(o["center"],[math.sqrt(2),math.sqrt(2),.5])
        before=tr.query(t-1000)[0];after=tr.query(t+1000)[0]
        numeric=(np.array(after["center"])-before["center"])/.000002
        np.testing.assert_allclose(o["velocity"],numeric,atol=1e-7)
        self.assertNotAlmostEqual(o["velocity"][0],o["center_interval_average_velocity"][0])
    def test_plus_minus_180_and_physical_not_pi_axis(self):
        tr=Series([state(0,heading=math.radians(179)),state(100,heading=math.radians(-179))],POLICY)
        self.assertAlmostEqual(abs(tr.query(BASE+50000000)[0]["heading"]),math.pi)
        tr=Series([state(0),state(100,heading=math.radians(170))],POLICY)
        self.assertAlmostEqual(math.degrees(tr.query(BASE+50000000)[0]["heading"]),85.)
    def test_duplicate_identical_and_conflicting(self):
        a=state(0);tr=Series([a,dict(a),state(100)],POLICY)
        self.assertEqual(tr.audit["identical_duplicate_rows"],1)
        self.assertIsNone(tr.query(BASE+50000000)[1])
        tr=Series([a,state(0,(1,0,0)),state(100)],POLICY)
        self.assertEqual(tr.query(BASE)[1],"conflict_timestamp")
        self.assertEqual(tr.query(BASE+50000000)[1],"conflict_timestamp")
    def test_gap_exact_sample_and_outside_rejected(self):
        tr=Series([state(0),state(500)],POLICY)
        self.assertEqual(tr.query(BASE+100000000)[1],"long_gap")
        self.assertIsNone(tr.query(BASE+500000000)[1])
        self.assertTrue(np.isnan(tr.query(BASE+500000000)[0]["velocity"]).all())
        self.assertEqual(tr.query(BASE-1)[1],"before_range")
        self.assertEqual(tr.query(BASE+500000001)[1],"after_range")
    def test_id_reuse_absence_and_dimensions(self):
        tr=Series([state(0,seq=0,dims=[4,2,1],cls=0),state(100,seq=2,dims=[4,2,1],cls=0)],POLICY)
        self.assertEqual(tr.query(BASE+50000000)[1],"object_absence")
        self.assertNotEqual(tr.rows[0]["generation"],tr.rows[1]["generation"])
        tr=Series([state(0,dims=[4,2,1]),state(100,dims=[5,2,1])],POLICY)
        self.assertEqual(tr.query(BASE+50000000)[1],"dimension_change")
    def test_teleport_and_nonmonotonic_rejected(self):
        tr=Series([state(0),state(50,(100,0,0))],POLICY)
        self.assertEqual(tr.query(BASE+20000000)[1],"position_jump")
        tr=Series([state(100),state(0)],POLICY)
        self.assertEqual(tr.query(BASE+50000000)[1],"nonmonotonic_stream")
    def test_snapshot_conflicts_not_averaged(self):
        o={"id":1,"cls":0,"p":[0,0,0]};other=dict(o,p=[10,0,0])
        rows=[{"ns":BASE,"objects":[o]},{"ns":BASE,"objects":[other]}]
        out=normalize_snapshots(rows)
        self.assertEqual(len(out),1);self.assertTrue(out[0]["conflict"])
    def test_evaluation_build_has_no_inference_io_or_input_mutation(self):
        obj=lambda ms,x:{"ns":BASE+ms*1000000,"receipt_ns":BASE+ms*1000000+1,"frame_id":"misleading","objects":[{"id":1,"cls":1,"p":[x,0,0],"heading_deg":0.,"dims":[.5,.5,1.8],"name":""}]}
        raw={"ego":[dict(state(0),heading_deg=0),dict(state(100),heading_deg=0)],
             "snapshots":[obj(0,10),obj(100,10.2)],
             "frames":[{"idx":0,"stamp_ns":BASE+50000000,"receipt_ns":BASE+50000001}]}
        original=copy.deepcopy(raw)
        with patch("builtins.open",side_effect=AssertionError("no IO in build")),patch("subprocess.Popen",side_effect=AssertionError("no inference")),patch("socket.socket",side_effect=AssertionError("no publisher")):
            F,G,reject,cov,events,audit=build(raw,POLICY,{},0.)
        self.assertEqual(raw,original);self.assertEqual(len(F),1);self.assertEqual(len(G),1)
        self.assertAlmostEqual(G.iloc[0].vx,2.)
    def test_integer_ns_with_missing_provenance_retains_single_ns(self):
        import io,pandas as pd
        df=records_frame([{"source_ns":BASE+1},{"source_ns":None},{"source_ns":BASE+3}])
        loaded=pd.read_csv(io.StringIO(df.to_csv(index=False)),dtype={"source_ns":"Int64"})
        self.assertEqual(int(loaded.source_ns.iloc[0]),BASE+1)
        self.assertEqual(int(loaded.source_ns.iloc[2]),BASE+3)
    def test_all_lidar_frames_preserved_when_ego_invalid(self):
        raw={"ego":[dict(state(0),heading_deg=0)],"snapshots":[],"frames":[{"idx":0,"stamp_ns":BASE-1},{"idx":1,"stamp_ns":BASE+1}]}
        F,*_=build(raw,POLICY,{},0.)
        self.assertEqual(len(F),2);self.assertFalse(F.ego_gt_valid.any())
if __name__=="__main__":unittest.main()
