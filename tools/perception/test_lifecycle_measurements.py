import tempfile
import unittest
from pathlib import Path

from tools.perception.lifecycle_measurements import rate_report, trace_trial, trace_integrity, ten_hz_pass
from asmc_camera.runtime.trace import LifecycleTrace
from asmc_camera.runtime.state import InferenceState, ACTIVE


class LifecycleMeasurementsTest(unittest.TestCase):
    def test_server_acceptance_counts_immediate_result_and_rejects_old_source(self):
        request=dict(generation=2,mode=2,call_started=10.,call_returned=10.1)
        trace=[dict(event='request_accepted',generation=2,mode=2,monotonic=10.05,source_cutoff_ns=100)]
        rows={'front':[(10.08,101,True),(10.09,100,True),(11.06,102,True)]}
        result=trace_trial(request,trace,rows,20.05)
        self.assertEqual(1,result['outputs']['front']['first_second'])
        self.assertEqual(1,result['outputs']['front']['excluded_old_epoch_messages'])
        self.assertAlmostEqual(.03,result['outputs']['front']['first_output_sec'])

    def test_old_generation_and_missing_or_duplicate_acceptance_fail(self):
        request=dict(generation=3,mode=2,call_started=10.,call_returned=10.1)
        row=dict(event='request_accepted',generation=2,mode=2,monotonic=10.05,source_cutoff_ns=100)
        with self.assertRaises(ValueError):trace_trial(request,[row])
        row['generation']=3
        with self.assertRaises(ValueError):trace_trial(request,[row,row])
        row['monotonic']=10.2
        with self.assertRaises(ValueError):trace_trial(request,[row])

    def test_preparation_is_per_generation_and_all_views(self):
        request=dict(generation=3,mode=1,call_started=1.,call_returned=1.1)
        trace=[dict(event='request_accepted',generation=3,mode=1,monotonic=1.05,source_cutoff_ns=10),
               dict(event='complete',generation=2,monotonic=1.1,hits={'front':3,'right':3}),
               dict(event='complete',generation=3,monotonic=1.2,hits={'front':3,'right':2}),
               dict(event='complete',generation=3,monotonic=1.3,hits={'front':4,'right':3}),
               dict(event='status',generation=3,monotonic=1.4,prepared=True,pipeline_ready=False),
               dict(event='status',generation=3,monotonic=1.5,prepared=True,pipeline_ready=True)]
        result=trace_trial(request,trace)
        self.assertAlmostEqual(.25,result['actual_image_ready_sec'])
        self.assertAlmostEqual(.45,result['pipeline_ready_sec'])

    def test_first_second_eight_fails_despite_later_ten_hz(self):
        rows=[(.2+i*.1,i+1,True) for i in range(99)]
        metric=rate_report(rows,0.,10.)
        self.assertEqual(8,metric['first_second'])
        self.assertFalse(ten_hz_pass(metric))
        rows=[(.05+i*.1,i+1,True) for i in range(100)]
        self.assertTrue(ten_hz_pass(rate_report(rows,0.,10.)))
        rows[5]=(rows[5][0],rows[4][1],True)
        self.assertFalse(ten_hz_pass(rate_report(rows,0.,10.)))

    def test_trace_disabled_no_io_and_overflow_does_not_pass(self):
        disabled=LifecycleTrace()
        disabled('input');disabled.close()
        self.assertEqual([],disabled.rows)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'trace.jsonl'
            trace=LifecycleTrace(str(path),capacity=1)
            trace('input');trace('begin')
            self.assertFalse(path.exists())
            trace.close()
            import json
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertFalse(trace_integrity(rows))
            rows[-1]['dropped']=0
            self.assertTrue(trace_integrity(rows))
            self.assertFalse(trace_integrity(rows[:-1]))

    def test_state_emits_accepted_processing_and_input_timestamps(self):
        events=[]
        state=InferenceState(['front'],managed=True,trace=lambda event,**values:events.append(dict(event=event,**values)))
        state.observe('front',100)
        state.request(ACTIVE,10)
        token=state.begin({'front':101})
        state.complete(token,{'front':101})
        self.assertEqual(['input','request_accepted','begin','complete'],[r['event'] for r in events])
        self.assertEqual(100,events[1]['source_cutoff_ns'])
        self.assertEqual(token,events[-1]['generation'])


if __name__=='__main__':unittest.main()
