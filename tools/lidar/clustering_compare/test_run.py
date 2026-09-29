import json
from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np
import yaml

from .projection import sha
from .run import admission, analytic_admitted, stable
from .score import raw_score
from .score import run as score_run
from .fixtures import cloud, CASES

CONFIG = Path(__file__).resolve().parents[3] / "config/baselines/lidar_l1b5a_clustering_20260929.yaml"


class AdmissionAndScoring(unittest.TestCase):
    def test_failed_capture_audit_cannot_be_cherry_picked(self):
        with tempfile.TemporaryDirectory() as folder:
            d = Path(folder)
            m = d / "input.json"
            m.write_text("{}")
            (d / "summary.json").write_text(json.dumps({"admitted": False, "admitted_frames": 97}))
            (d / "manifest.json").write_text(json.dumps({"config_sha256": sha(CONFIG), "admission_sha256": sha(m),
                "artifacts": {"summary.json": sha(d / "summary.json")}}))
            self.assertFalse(admission("capture", d, CONFIG, m))
            with self.assertRaises(ValueError):
                admission("capture", None, CONFIG, m)

    def test_dense_arbitrary_points_have_no_projection_admission(self):
        self.assertFalse(analytic_admitted({"fixture_family": "historical_dense"}, np.zeros((10,4)),
                                          yaml.safe_load(CONFIG.read_text())))

    def test_raw_count_does_not_create_sufficient_observation(self):
        points = 20
        mode = {"labels": [0]*points, "cluster_id": [0]*points,
                "objects": [{"removed": False, "center": [0,0,0], "dimensions": [1,1,1]}]}
        s = raw_score(3, np.full(points, 3), np.zeros(points, dtype=int), [0], mode)
        self.assertTrue(s["accepted_target"])
        self.assertFalse(s["sufficient_target_voxels_for_10_point_cluster"])
        self.assertEqual(s["wall_stage_target_voxels"], 1)

    def test_stable_digest_excludes_only_measured_time(self):
        self.assertEqual(stable({"scan_stamp_ns": 42, "timing_ms": {"obb": 1}, "lineage_ms": 2}),
                         {"scan_stamp_ns": 42})

    def test_outside_roi_is_not_ground_failure(self):
        mode = {"labels": [255], "cluster_id": [-1], "objects": []}
        s = raw_score(3, np.array([3]), np.array([-1]), [], mode)
        self.assertEqual(s["failure_reason"], "outside_roi")

    def test_new_first_return_fixtures_are_deterministic_and_admitted(self):
        config = yaml.safe_load(CONFIG.read_text())
        for name in CASES:
            points, labels = cloud(name)
            other, other_labels = cloud(name)
            self.assertTrue(np.array_equal(points, other))
            self.assertTrue(np.array_equal(labels, other_labels))
            self.assertTrue(np.all(points[:, 3] == 10))
            self.assertTrue(analytic_admitted({"fixture_family": "l1b5_analytic_first_return"}, points, config))
            self.assertLessEqual(np.unique(labels).size, 6)

    def test_synthetic_scores_are_json_serializable(self):
        with tempfile.TemporaryDirectory() as folder:
            d = Path(folder)
            (d / "clouds").mkdir()
            np.arange(20,dtype="<i4").tofile(d / "clouds/fixture.indices.bin")
            (d / "snapshots.jsonl").write_text(json.dumps({"input_id":"fixture","camera_state":"empty"})+"\n")
            mode = {"labels":[0]*20,"cluster_id":[0]*20,"projection_reasons":[1]*20,
                    "objects":[{"removed":False,"center":[0,0,0],"dimensions":[1,1,1]}]}
            row = {"input_id":"fixture","measurement":True,"pre_wall_labels":[0]*20,
                   "baseline":mode,"raw_euclidean":mode,"travel_aos":{"status":"blocked_projection_admission"}}
            (d / "geometry.jsonl").write_text(json.dumps(row)+"\n")
            (d / "manifest.json").write_text("{}")
            truth = d / "truth.jsonl"
            truth.write_text(json.dumps({"input_id":"fixture","kind":"car","labels":[3]*20})+"\n")
            with redirect_stdout(io.StringIO()):
                score_run(SimpleNamespace(kind="synthetic",run=d,truth=truth,output=d/"scores"))
            self.assertEqual(json.loads((d/"scores/summary.json").read_text())["measured_frames"],1)


if __name__ == "__main__":
    unittest.main()
