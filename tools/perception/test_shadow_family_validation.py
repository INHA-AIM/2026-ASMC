import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parent))
from prepare_shadow_family_fixture import nearest_index, phase
from shadow_family_parity import compare_family


class ShadowValidationTest(unittest.TestCase):
    def test_measurement_waits_for_the_requested_epoch_and_a_valid_lane_pipeline(self):
        from compare_camera_runtime import transition_ready
        status={'lane':NS(generation=1,mode=1,prepared=True,pipeline_ready=True)}
        self.assertTrue(transition_ready(status,{'lane':1},1))
        self.assertFalse(transition_ready(status,{'lane':2},1))
        self.assertFalse(transition_ready(status,{'lane':1},2))
        status['lane'].pipeline_ready=False
        self.assertFalse(transition_ready(status,{'lane':1},1))

    def test_diagnostic_profiles_cannot_be_used_for_timing_acceptance(self):
        from compare_camera_runtime import acceptance_eligible
        self.assertTrue(acceptance_eligible({}))
        self.assertFalse(acceptance_eligible(dict(allocation_profile=True)))
        self.assertFalse(acceptance_eligible(dict(profile_lane_fit=True)))
        self.assertFalse(acceptance_eligible(dict(acceptance_eligible=False)))

    def test_camera_pairing_does_not_accept_out_of_window_or_missing_inputs(self):
        self.assertIsNone(nearest_index([],10,3))
        self.assertIsNone(nearest_index([1,20],10,3))
        self.assertEqual(nearest_index([1,10,20],13,3),1)
        self.assertEqual(nearest_index([10,20],15,5),0)
        self.assertEqual([phase(s,10,20) for s in (9,10,15,20,21)],
                         ['entry','interior','interior','interior','exit'])

    def family(self, valid=True, delta=0, role=1):
        return NS(valid=valid,status_reason='test',boundaries=[NS(role=role,color=1,
            polyline=[NS(x=i,y=delta) for i in range(4)],support_state=[1]*4)])

    def test_parity_reports_new_invalidity_role_confusion_and_geometry_separately(self):
        r=compare_family(self.family(),self.family(delta=.1))
        self.assertTrue(r['validity_equal'] and r['roles_equal'])
        self.assertAlmostEqual(r['geometry_delta_p95_m'],.1)
        self.assertFalse(compare_family(self.family(),self.family(False))['validity_equal'])
        self.assertFalse(compare_family(self.family(),self.family(role=2))['roles_equal'])
        changed=self.family(); changed.boundaries[0].support_state=[1,0,0,1]
        self.assertEqual(compare_family(self.family(),changed)['support_changed_roles'],1)

    def test_shifted_arc_samples_are_compared_at_the_same_forward_distance(self):
        reference=self.family(); candidate=self.family()
        for message,shift in [(reference,0.),(candidate,.4)]:
            for i,p in enumerate(message.boundaries[0].polyline):
                p.x=i+shift; p.y=.1*p.x
        compared=compare_family(reference,candidate)
        self.assertLess(compared['geometry_delta_max_m'],1e-12)
        self.assertGreater(compared['sample_index_distance_p95_m'],.4)

    def test_allocation_probe_restores_methods_and_does_not_change_gc_policy(self):
        import gc
        from asmc_msgs.msg import LaneBoundaryFamily
        from camera_runtime_probe import install_allocation_profile
        original=LaneBoundaryFamily.deserialize
        enabled,thresholds=gc.isenabled(),gc.get_threshold()
        rows=Mock(); callbacks=[]
        with tempfile.TemporaryDirectory() as directory, \
                patch('camera_runtime_probe.measurement_journal',return_value=rows), \
                patch('atexit.register',side_effect=callbacks.append):
            install_allocation_profile(Path(directory)/'profile.jsonl')
            try:
                encoded=io.BytesIO(); LaneBoundaryFamily().serialize(encoded)
                LaneBoundaryFamily().deserialize(encoded.getvalue())
            finally:
                callbacks[-1]()
        self.assertIs(LaneBoundaryFamily.deserialize,original)
        events=[call.args[0] for call in rows.append.call_args_list]
        self.assertTrue(any(e.get('stage')=='deserialize' for e in events))
        self.assertFalse(events[-1]['acceptance_timing'])
        rows.close.assert_called_once()
        self.assertEqual((gc.isenabled(),gc.get_threshold()),(enabled,thresholds))


if __name__=='__main__': unittest.main()
