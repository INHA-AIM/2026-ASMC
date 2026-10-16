import hashlib
import tempfile
import unittest
from pathlib import Path
from install_camera_model_release import install, link_package_models


class ReleaseInstallTest(unittest.TestCase):
    def test_local_links_are_verified_repeatable_and_never_replace_existing_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            manifest, source, target = self.fixture(root)
            install(manifest,source,target)
            package=root/'package/models'
            result=link_package_models(manifest,target,package)
            self.assertEqual(result['copied_bytes'],0)
            link_package_models(manifest,target,package)
            self.assertTrue((package/'b.pt').is_symlink())
            self.assertEqual((package/'b.pt').read_bytes(),b'pinned checkpoint')
            (package/'a.engine').unlink()
            (package/'a.engine').write_bytes(b'legacy model')
            with self.assertRaises(ValueError):
                link_package_models(manifest,target,package)
            self.assertEqual((package/'a.engine').read_bytes(),b'legacy model')

    def fixture(self, root):
        source = root / 'download'
        source.mkdir()
        (source / 'a.engine').write_bytes(b'pinned engine')
        (source / 'b.pt').write_bytes(b'pinned checkpoint')
        files = [dict(asset=p.name, relative_path='model/' + p.name,
                      bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                 for p in sorted(source.iterdir())]
        return dict(schema_version=1, release_tag='test', files=files), source, root / 'models'

    def test_verified_install_and_repeat(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest, source, target = self.fixture(Path(directory))
            install(manifest, source, target, True)
            self.assertFalse(target.exists())
            install(manifest, source, target)
            install(manifest, source, target)
            self.assertEqual((source / 'a.engine').read_bytes(), (target / 'model/a.engine').read_bytes())

    def test_one_corrupt_asset_prevents_partial_install(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest, source, target = self.fixture(Path(directory))
            (source / 'b.pt').write_bytes(b'corrupt')
            with self.assertRaises(ValueError):
                install(manifest, source, target)
            self.assertFalse(target.exists())

    def test_existing_other_model_and_escape_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest, source, target = self.fixture(Path(directory))
            (target / 'model').mkdir(parents=True)
            (target / 'model/a.engine').write_bytes(b'existing other engine')
            with self.assertRaises(ValueError):
                install(manifest, source, target)
            manifest['files'][0]['relative_path'] = '../escaped.engine'
            with self.assertRaises(ValueError):
                install(manifest, source, target)


if __name__ == '__main__':
    unittest.main()
