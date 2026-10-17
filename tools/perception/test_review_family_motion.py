"""Independent geometry, time, reset and rendering checks for offline review."""
import unittest
import numpy as np
from tools.perception.review_family_motion import (
    rotation, rpy, interpolate, attitude_plane, move_xy, add_history,
    temporal_delta, render_family,
)
from tools.perception.collect_family_motion_inputs import select_windows
from tools.perception.summarize_family_motion import sample_boundary, matched_pairs, VARIANTS
from asmc_camera.bev.calibration import CameraCalibration, GroundPlane
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.bev.model_grid_projection import MetricEvidence
from asmc_camera.bev.visualization import draw_metric_grid


def q_axis(axis, degrees):
    q=np.zeros(4);q[axis]=np.sin(np.deg2rad(degrees)/2);q[3]=np.cos(np.deg2rad(degrees)/2)
    return q


def camera():
    return CameraCalibration('front',1,1280,720,90.,(1.9,0,1.2),(0,2,0),
        'rgb','drive','mark','confidence',GroundPlane('base_link',-.35,0,0,'fixture'))


def evidence(xy):
    xy=np.array(xy,dtype=np.float32);n=len(xy)
    return MetricEvidence(xy,np.ones(n,dtype=np.uint8),np.ones(n,dtype=np.float32),
                          np.ones(n,dtype=np.float32),np.ones(n,dtype=np.uint8))


class MotionReviewTest(unittest.TestCase):
    def test_left_forward_asymmetry_in_native_and_bev(self):
        points=np.array([[10,3],[5,-2],[0,0]])
        grid=BevGridSpec(-5,20,-8,8,.1)
        px=grid.metric_to_pixel(points)
        np.testing.assert_allclose(px,[[50,100],[100,150],[80,200]])
        uv,_=camera().project_ground(points[:2])
        self.assertLess(uv[0,0],640);self.assertGreater(uv[1,0],640)

    def test_invalid_quaternion(self):
        for q in ([0,0,0,0],[0,0,np.nan,1]):
            with self.assertRaises(ValueError):rotation(q)

    def test_body_roll_changes_plane_not_mount(self):
        original=camera();corrected=attitude_plane(original,q_axis(0,10))
        self.assertEqual(original.rotation_deg,corrected.rotation_deg)
        self.assertAlmostEqual(-np.tan(np.deg2rad(10)),corrected.ground_plane.dz_dy)
        self.assertAlmostEqual(original.ground_plane.z_at_origin_m,
            corrected.ground_plane.z_at_origin_m*np.cos(np.deg2rad(10)))

    def test_yaw_alone_does_not_tilt_flat_ground(self):
        for yaw in (-179,0,87):
            c=attitude_plane(camera(),q_axis(2,yaw))
            np.testing.assert_allclose(c.ground_plane.coefficients,[0,0,-.35],atol=1e-12)

    def test_rest_variant_cancels_same_attitude(self):
        q=q_axis(1,2)
        c=attitude_plane(camera(),q,q)
        np.testing.assert_allclose(c.ground_plane.coefficients,[0,0,-.35],atol=1e-12)

    def test_slerp_opposite_quaternion_sign(self):
        q=q_axis(2,30)
        samples=[dict(source_ns=10,q_xyzw=q.tolist()),dict(source_ns=30,q_xyzw=(-q).tolist())]
        result=interpolate(samples,20,'imu')
        np.testing.assert_allclose(rotation(result['q_xyzw']),rotation(q),atol=1e-12)

    def test_no_extrapolation_or_long_imu_gap(self):
        samples=[dict(source_ns=100,q_xyzw=[0,0,0,1]),
                 dict(source_ns=200_000_101,q_xyzw=[0,0,0,1])]
        for stamp in (0,150,300_000_000):self.assertIsNone(interpolate(samples,stamp,'imu'))

    def test_invalid_motion_sample_is_unavailable(self):
        bad=[dict(source_ns=10,q_xyzw=[0,0,0,0]),dict(source_ns=30,q_xyzw=[0,0,0,1])]
        self.assertIsNone(interpolate(bad,10,'imu'))
        self.assertIsNone(interpolate(bad,20,'imu'))
        bad=[dict(source_ns=10,position_xyz=[0,float('nan'),0],heading_deg=0)]
        self.assertIsNone(interpolate(bad,10,'pose'))

    def test_pose_interpolation_wrap(self):
        samples=[dict(source_ns=0,position_xyz=[0,0,0],heading_deg=179),
                 dict(source_ns=100,position_xyz=[2,0,0],heading_deg=-179)]
        result=interpolate(samples,50,'pose')
        self.assertEqual(180,result['heading_deg']);self.assertEqual([1,0,0],result['position_xyz'])

    def test_ego_translation_and_turn_inverse(self):
        a=dict(position_xyz=[0,0,0],heading_deg=0)
        b=dict(position_xyz=[1,0,0],heading_deg=90)
        transformed=move_xy([[10,0]],a,b)
        np.testing.assert_allclose(transformed,[[0,-9]],atol=1e-12)
        np.testing.assert_allclose(move_xy(transformed,b,a),[[10,0]],atol=1e-12)

    def test_current_cells_win_and_history_expiry(self):
        current=evidence([[1,0]]);past=evidence([[1.01,.01],[2,0]])
        result,count=add_history(current,past,100_000_000)
        self.assertEqual(1,count);self.assertEqual(2,len(result.xy))
        self.assertEqual(.25,result.geometric_quality[-1])
        for age in (-1,0,200_000_001):
            value,n=add_history(current,past,age);self.assertIs(value,current);self.assertEqual(0,n)
        self.assertEqual(0,add_history(current,past,1,same_context=False)[1])

    def test_same_world_curve_is_stable_during_translation(self):
        old=dict(boundaries=[dict(role='a',xy=[[x,1] for x in range(10)],support=[1]*10)])
        new=dict(boundaries=[dict(role='a',xy=[[x-1,1] for x in range(10)],support=[1]*10)])
        a=dict(position_xyz=[0,0,0],heading_deg=0);b=dict(position_xyz=[1,0,0],heading_deg=0)
        self.assertEqual(0,temporal_delta(old,new,a,b)['max'])

    def test_invalid_gap_not_used_for_temporal_interpolation(self):
        a=dict(boundaries=[dict(role='a',xy=[[0,1],[1,1],[2,1]],support=[1,0,1])])
        b=dict(boundaries=[dict(role='a',xy=[[1,1]],support=[1])])
        pose=dict(position_xyz=[0,0,0],heading_deg=0)
        self.assertEqual(0,temporal_delta(a,b,pose,pose)['count'])

    def test_cached_render_identical_and_does_not_mutate_background(self):
        grid=BevGridSpec(-5,20,-8,8,.1)
        background=np.zeros((*grid.shape,3),np.uint8);draw_metric_grid(background,grid)
        copy=background.copy();family=dict(reason='accepted',boundaries=[])
        np.testing.assert_array_equal(render_family(family,grid),render_family(family,grid,background))
        np.testing.assert_array_equal(copy,background)

    def test_windows_keep_consecutive_original_order(self):
        frames=[dict(phase=phase,i=i) for phase in ('entry','interior','exit') for i in range(20)]
        selected=select_windows(frames,5)
        self.assertEqual(15,len(selected))
        for phase in ('entry','interior','exit'):
            indices=[r['i'] for r in selected if r['phase']==phase]
            self.assertEqual([1]*4,np.diff(indices).tolist())

    def test_common_queries_do_not_reward_shrunk_range(self):
        roles=('left_outer_boundary','yellow_centre_boundary','right_outer_boundary')
        rows=[]
        for stamp in (10,20):
            variants={}
            for v in VARIANTS:
                length=16 if v=='M0' else 13
                variants[v]=dict(boundaries=[dict(role=role,xy=[[x,offset+(1 if stamp==20 and x>13 else 0)]
                    for x in range(length)],support=[1]*length) for role,offset in zip(roles,(3,0,-3))])
            rows.append(dict(split='continuous',run_id='r',phase='interior',source_ns=stamp,target_ns=stamp,variants=variants))
        ego=[dict(topic='/Ego_topic',run_id='r',source_ns=t,position_xyz=[0,0,0],heading_deg=0) for t in (10,20)]
        pairs=matched_pairs(rows,ego)
        self.assertEqual(1,len(pairs))
        for v in VARIANTS:
            self.assertEqual(0,pairs[0]['variants'][v]['max'])
            self.assertEqual(pairs[0]['variants']['M0']['count'],pairs[0]['variants'][v]['count'])


if __name__=='__main__':unittest.main()
