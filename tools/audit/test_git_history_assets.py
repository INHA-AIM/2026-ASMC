import copy
import hashlib
from pathlib import Path
import tempfile
import unittest

from tools.audit.check_git_history_assets import check_retired, validate_inventory


class HistoryAssetsTest(unittest.TestCase):
    def fixture(self):
        return dict(schema_version=1, history_commit='a'*40, assets=[dict(
            path='models/old.pt', bytes=3, sha256=hashlib.sha256(b'old').hexdigest(), git_blob='b'*40)])

    def test_restored_asset_cannot_silently_return_to_current_distribution(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);registry=self.fixture()
            self.assertEqual(check_retired(root, registry), [])
            (root/'models').mkdir();(root/'models/old.pt').write_bytes(b'old')
            self.assertIn('reintroduced', check_retired(root, registry)[0])
            (root/'models/old.pt').unlink()
            (root/'models/old.pt').symlink_to(root/'missing')
            self.assertIn('reintroduced', check_retired(root, registry)[0])

    def test_mutable_reference_duplicate_and_escape_are_rejected(self):
        for mutate in (lambda j:j.update(history_commit='main'),
                       lambda j:j['assets'].append(copy.deepcopy(j['assets'][0])),
                       lambda j:j['assets'][0].update(path='../outside.pt'),
                       lambda j:j['assets'][0].update(sha256='unknown')):
            registry=self.fixture();mutate(registry)
            with self.assertRaises(ValueError):validate_inventory(registry)


if __name__ == '__main__':
    unittest.main()
