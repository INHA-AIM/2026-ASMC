import copy
from pathlib import Path
import sys
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from b1a_protocol import jobs
from summarize_tunnel_evidence import protocol_closed


class ClosureTest(unittest.TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[2]
        self.config=yaml.safe_load((root/'config/experiments/camera_lidar_b1a_v1.yaml').read_text())
        self.entries=[dict(job=j,complete=True) for p in ('offline','realtime') for j in jobs(self.config,p)]

    def test_all_unique_complete_jobs_required(self):
        self.assertTrue(protocol_closed(self.entries,self.config))
        self.assertFalse(protocol_closed(self.entries[:-1],self.config))
        self.entries[-1]['complete']=False
        self.assertFalse(protocol_closed(self.entries,self.config))

    def test_duplicate_success_cannot_replace_missing_repetition(self):
        self.entries[-1]=copy.deepcopy(self.entries[0])
        self.assertFalse(protocol_closed(self.entries,self.config))


if __name__=='__main__':unittest.main()
