import hashlib
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.audit import check_documentation_migration as audit
from tools.audit.documentation_links import anchors, links, without_link_destinations


class MarkdownReferencesTest(unittest.TestCase):
    def test_image_reference_html_and_parentheses_keep_target_boundaries(self):
        text = ('![image](<images/a b.png>)\n[weight](model(1).md "title")\n'
                '[ref][guide]\n[guide]: target.md#part\n<a href="other.md#part">x</a>\n')
        self.assertEqual([x.target for x in links(text)],
                         ['images/a b.png', 'model(1).md', 'target.md#part', 'other.md#part'])

    def test_historical_commands_and_inline_examples_are_not_live_links(self):
        text = ('```bash\n[old](removed.md)\n```\n`[old](also-removed.md)`\n'
                '[live](actual.md)\n')
        self.assertEqual([x.target for x in links(text)], ['actual.md'])
        self.assertIn('[old](removed.md)', without_link_destinations(text))

    def test_unicode_duplicate_and_explicit_anchors(self):
        text = '# 차량·차선 — 검사\n## Same\n## Same\n<a id="custom"></a>\n```\n# fake\n```\n'
        self.assertTrue({'차량차선--검사', 'same', 'same-1', 'custom'} <= anchors(text))
        self.assertNotIn('fake', anchors(text))


class MigrationPolicyTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        for name in audit.CENTRAL_DIRECTORIES:
            path = self.root / 'docs' / name / 'README.md'
            path.parent.mkdir(parents=True)
            path.write_text('# Guide\n')
        self.root_patch = patch.object(audit, 'REPO_ROOT', self.root)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)
        self.files_patch = patch.object(audit, 'tracked_markdown_files',
                                       side_effect=lambda: list(self.root.rglob('*.md')))
        self.files_patch.start()
        self.addCleanup(self.files_patch.stop)
        self.manifest = dict(schema_version=1, baseline=dict(markdown_files=0, generated_text_files=0,
                              unchanged_binary_assets=0), retention=[], frozen_current_files=[],
                              redirects=[], retired_redirects=[], unchanged_assets=[])

    def inventory(self, source, target, original):
        return dict(schema_version=2, base_commit='base', entries=[dict(source=source, target=target, action='move',
                    source_sha256=hashlib.sha256(original).hexdigest(), frozen=False)],
                    new_files=[p.relative_to(self.root).as_posix() for p in self.root.rglob('*.md')
                               if p.relative_to(self.root).as_posix() != target])

    def test_living_document_updates_pass_default_but_fail_one_time_relocation(self):
        source = 'docs/old.md'
        target = 'docs/operations/camera.md'
        original = b'# Camera\nOriginal explanation.\n'
        (self.root/target).write_text('# Camera\nUpdated explanation.\n')
        self.assertEqual(audit.validate_current_tree(self.manifest), [])
        with patch.object(audit, 'git_bytes', return_value=original), \
                patch.object(audit, 'git_tree_paths', return_value=[source]):
            errors = audit.validate_relocation(self.inventory(source, target, original))
        self.assertTrue(any('non-navigation content changed' in x for x in errors))

    def test_relocation_allows_only_links_and_explicit_navigation(self):
        source = 'docs/README.md'
        target = 'docs/README.md'
        original = b'# Camera\n[contract](old-contract.md)\n'
        (self.root/target).write_text('# Camera\n[contract](../contracts/current.md)\n'
            '<!-- documentation-structure-v2:start -->\nNavigation.\n'
            '<!-- documentation-structure-v2:end -->\n')
        with patch.object(audit, 'git_bytes', return_value=original), \
                patch.object(audit, 'git_tree_paths', return_value=[source]):
            self.assertEqual(audit.validate_relocation(self.inventory(source, target, original)), [])

    def test_frozen_bytes_and_retired_stub_reintroduction_are_rejected(self):
        path = self.root/'docs/records/frozen.md'
        path.write_text('original')
        self.manifest['frozen_current_files'] = [dict(path='docs/records/frozen.md',
            sha256=hashlib.sha256(b'original').hexdigest())]
        path.write_text('changed')
        self.manifest['retired_redirects'] = [dict(legacy='docs/old.md', target='docs/operations/README.md')]
        (self.root/'docs/old.md').write_text('reintroduced')
        errors = audit.validate_current_tree(self.manifest)
        self.assertTrue(any('frozen file SHA mismatch' in x for x in errors))
        self.assertTrue(any('retired compatibility page reintroduced' in x for x in errors))

    def test_missing_anchor_and_image_fail_while_code_examples_are_ignored(self):
        (self.root/'docs/operations/README.md').write_text(
            '# Guide\n[missing](#not-present)\n![missing](absent.png)\n'
            '```\n[historical](removed.md)\n```\n')
        errors = audit.validate_current_tree(self.manifest)
        self.assertTrue(any('broken Markdown anchor' in x for x in errors))
        self.assertTrue(any('absent.png' in x for x in errors))
        self.assertFalse(any('removed.md' in x for x in errors))


if __name__ == '__main__':
    unittest.main()
