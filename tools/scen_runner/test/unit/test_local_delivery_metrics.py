import json
from pathlib import Path
import tempfile
import unittest
from scen_runner.records.analyze_local import analyze


def check_delivery_clock_and_marked_drive_window(tmp_path):
    def rows(name, values):
        (tmp_path/name).write_text(''.join(json.dumps(v)+'\n' for v in values))
    rows('decisions.jsonl', [])
    rows('deliveries.jsonl', [dict(valid=v, reason='ok', received_steady=t)
                            for t,v in [(1,False),(2,True),(2.1,True),(2.4,True)]])
    rows('events.jsonl', [dict(event='driving_started',monotonic=2),
                         dict(event='driving_ended',monotonic=3)])
    rows('wire.jsonl', [dict(received_steady=t) for t in [1,2,2.05,2.1,4]])
    rows('resources.jsonl', [dict(processes=[dict(name='planner',rss_kib=x)]) for x in [10,12,11]])
    result=analyze(tmp_path)
    assert result['plans']==4 and result['valid']==3
    assert abs(result['valid_plan_gap_ms']['max']-300)<1e-6
    assert result['wire_command_receive_gap_ms']['max']>1000
    assert abs(result['driving_wire_command_receive_gap_ms']['max']-50)<1e-6
    assert result['rss_kib']['planner']==dict(first=10,last=11,maximum=12,samples=3)


class DeliveryMetricsTest(unittest.TestCase):
    def test_full_delivery_clock_and_marked_drive_window(self):
        with tempfile.TemporaryDirectory() as root:
            check_delivery_clock_and_marked_drive_window(Path(root))


if __name__=='__main__': unittest.main()
