#!/usr/bin/env python3
"""Cross-language camera producer digest, without importing camera/model code."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml

BINARY = sys.argv.pop(1)


class CalibrationHashTest(unittest.TestCase):
    def compare(self, document):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'calibration.yaml'
            path.write_text(yaml.safe_dump(document, allow_unicode=True))
            actual = dict(line.split() for line in subprocess.check_output([BINARY, str(path)], text=True).splitlines())
        expected = {'camera_'+view: 'sha256:'+hashlib.sha256(json.dumps(
            dict(camera=row, source_sha256=document['source_sha256']), sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()
            for view, row in document['cameras'].items()}
        self.assertEqual(actual, expected)

    def test_finite_geometry_and_json_types(self):
        row = dict(width=1280, height=720, horizontal_fov_deg=90.0,
                   translation_m=[1.90, 0.00, 1.20], rotation_deg=[0., 2., 0.],
                   topics=dict(image='/image_jpeg/compressed'),
                   metadata=dict(text='한글\\"\n', quoted_number='123', enabled=True,
                                 missing=None, values=[-0.0, 1.e-5, 1.e16, 0.3, 12, -12, 999999.9]))
        self.compare(dict(source_sha256='a'*64, cameras={view: row for view in ('front','left','right')}))

    def test_current_camera_owned_file_when_present(self):
        path = Path(__file__).resolve().parents[2] / 'asmc_camera/config/cameras_v3.yaml'
        if not path.exists():
            self.skipTest('camera package absent from isolated build')
        self.compare(yaml.safe_load(path.read_text()))


if __name__ == '__main__':
    unittest.main()
