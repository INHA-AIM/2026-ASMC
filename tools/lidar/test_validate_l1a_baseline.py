"""Small tests for the read-only, pinned L1-A verifier."""

import hashlib
from pathlib import Path
import tempfile
import unittest

from validate_l1a_baseline import digest_file, safe_child


class L1ABaselineVerifierTest(unittest.TestCase):
    def test_streaming_sha(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "fixture.bin"
            payload = b"lidar-l1a-fixture" * 131072
            path.write_bytes(payload)
            self.assertEqual(digest_file(path), hashlib.sha256(payload).hexdigest())

    def test_path_must_stay_under_data_root(self):
        with tempfile.TemporaryDirectory() as root:
            self.assertEqual(safe_child(Path(root), "lidar/a.bag"),
                             Path(root) / "lidar" / "a.bag")
            with self.assertRaises(ValueError):
                safe_child(Path(root), "../../outside.bag")


if __name__ == "__main__":
    unittest.main()
