import json
from pathlib import Path
import tempfile
import unittest

from tools.ci.run_checks import run_groups


class GroupsTest(unittest.TestCase):
    def run_fixture(self, scripts, workers=2, timeout=5):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        groups = []
        for name, script in scripts.items():
            (root/(name+'.sh')).write_text(script)
            groups.append(dict(name=name, script=name+'.sh', timeout_s=timeout))
        return root, groups, run_groups(groups, root/'result', workers, root)

    def test_failure_does_not_skip_other_groups_and_logs_survive(self):
        root, _, result = self.run_fixture({'bad':'echo failing; exit 7', 'later':'echo checked'})
        self.assertFalse(result['passed'])
        self.assertEqual([g['returncode'] for g in result['groups']], [7, 0])
        self.assertIn('checked', (root/'result/later.log').read_text())
        self.assertEqual(json.loads((root/'result/summary.json').read_text()), result)

    def test_independent_groups_really_overlap(self):
        script='date +%s.%N > "$ASMC_CI_OUTPUT/$0.start"\nsleep .35\ndate +%s.%N > "$ASMC_CI_OUTPUT/$0.end"'
        root, _, result = self.run_fixture({'a':script, 'b':script})
        self.assertTrue(result['passed'])
        a,b=[float((root/('result/'+n+'.sh.start')).read_text()) for n in ('a','b')]
        ends=[float((root/('result/'+n+'.sh.end')).read_text()) for n in ('a','b')]
        self.assertLess(max(a,b), min(ends))

    def test_timeout_is_failure_and_later_group_still_runs(self):
        root, _, result = self.run_fixture({'hung':'sleep 20', 'next':'echo tested'}, workers=1, timeout=.1)
        self.assertEqual(result['groups'][0]['status'], 'timeout')
        self.assertEqual(result['groups'][1]['status'], 'passed')
        self.assertFalse(result['passed'])

    def test_missing_empty_duplicate_and_overwrite_fail_closed(self):
        root, groups, _ = self.run_fixture({'ok':'true'})
        with self.assertRaises(FileExistsError):
            run_groups(groups, root/'result', root=root)
        for invalid in ([], groups+groups, [dict(name='missing',script='absent.sh',timeout_s=1)]):
            with self.assertRaises(ValueError):
                run_groups(invalid,root/'new',root=root)

    def test_exclusive_group_finishes_before_parallel_load_even_if_it_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'exclusive.sh').write_text('sleep .1; touch "$ASMC_CI_OUTPUT/finished"; exit 9')
            (root/'normal.sh').write_text('test -f "$ASMC_CI_OUTPUT/finished"')
            groups=[dict(name='normal',script='normal.sh',timeout_s=2),
                    dict(name='exclusive',script='exclusive.sh',timeout_s=2,exclusive=True)]
            result=run_groups(groups,root/'out',root=root)
            self.assertFalse(result['passed'])
            self.assertEqual([(g['name'],g['returncode']) for g in result['groups']],
                             [('exclusive',9),('normal',0)])


if __name__ == '__main__':
    unittest.main()
