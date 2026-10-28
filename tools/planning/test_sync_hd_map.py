import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('sync_hd_map', Path(__file__).with_name('sync_hd_map.py'))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PlannerMapBundleTest(unittest.TestCase):
    def fixture(self, root):
        for name in ['src/asmc_hd_map/config/kcity2025_compact_drivable_map.json',
                     'src/asmc_hd_map/config/kcity2025_compact_drivable_grid.npz']:
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(MODULE.ROOT / name, target)
        MODULE.export_bundle(root)

    def test_committed_bundle_matches_official_vector_and_every_raster_cell(self):
        self.assertEqual(MODULE.export_bundle(MODULE.ROOT, True),
                         'kcity2025_reviewed_high_speed_adjacent_drivable_v7')

    def test_stale_planner_grid_cannot_pass_even_with_unchanged_dimensions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            path = root / 'src/asmc_planner/config/frenet/hd_map/drivable_grid.rle'
            with path.open('a') as stream:
                stream.write('0 0 1\n')
            with self.assertRaisesRegex(ValueError, 'bundle is stale'):
                MODULE.export_bundle(root, True)

    def test_wrong_official_npz_refuses_export(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            path = root / 'src/asmc_hd_map/config/kcity2025_compact_drivable_grid.npz'
            with path.open('ab') as stream:
                stream.write(b'wrong bundle')
            with self.assertRaisesRegex(ValueError, 'NPZ digest mismatch'):
                MODULE.export_bundle(root)


if __name__ == '__main__':
    unittest.main()
