"""Exercise the real C++ continuous replayer, including corrupt-trace controls."""
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


def fixture():
    def obj(count=0, label='unknown', raw='unknown', candidate='unknown'):
        return dict(id=7, **{'class': label}, raw_class=raw, candidate=candidate,
                    confirmations=count, class_misses=0, lidar_misses=0,
                    locked=False, center=[1, 2, 3])
    def event(kind, stamp, **fields):
        return dict(event=kind, stamp_ns=stamp, **copy.deepcopy(fields))
    rows = [event('class_track_birth', 100000000, object=obj())]
    current = obj()
    for index in range(1, 7):
        stamp = index * 100000000
        before = copy.deepcopy(current)
        after = obj(index, candidate='car') if index < 5 else obj(label='car', raw='truck')
        rows += [event('class_lidar_ready', stamp, generation=index, objects=[before]),
                 event('yolo_admitted', stamp),
                 event('match_decision', stamp, action='match', waited_ns=0,
                       yolo_stamps_ns=[stamp], selected_yolo_stamp_ns=stamp, offset_ns=0),
                 event('class_before', stamp, yolo_stamp_ns=stamp, objects=[before]),
                 event('class_observation', stamp, id=7, label='car', raw_label='truck'),
                 event('class_after', stamp, yolo_stamp_ns=stamp, objects=[after]),
                 event('class_current', stamp, yolo_stamp_ns=stamp, objects=[after]),
                 event('class_publish_input', stamp, generation=index, objects=[after],
                       origins=[dict(id=7, source_lidar_stamp_ns=stamp,
                                     source_yolo_stamp_ns=stamp,
                                     applied_at_lidar_stamp_ns=stamp, updated_in_frame=True)])]
        current = after
    rows.append(event('fusion_summary', 0, matched=6, dropped=0, pending_lidar=0, pending_yolo=0))
    return rows


class ContinuousReplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('MAIN_CONTINUOUS_REPLAY'):
            cls.executable = os.environ['MAIN_CONTINUOUS_REPLAY']
        else:
            from roslib.packages import find_node
            candidates = find_node('Main', 'fusion_continuous_trace_replay')
            if not candidates:
                raise RuntimeError('build fusion_continuous_trace_replay before testing')
            cls.executable = candidates[0]

    def replay(self, rows, initial=5):
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / 'fixture.jsonl'
            trace.write_text(''.join(json.dumps(row) + '\n' for row in rows))
            return subprocess.run([self.executable, str(trace), str(initial), '3', '3'],
                                  text=True, capture_output=True)

    def rejected(self, rows, message):
        result = self.replay(rows)
        self.assertNotEqual(0, result.returncode, result.stdout)
        self.assertIn(message, result.stderr)

    def test_continuous_fixture(self):
        result = self.replay(fixture())
        self.assertEqual(0, result.returncode, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(6, report['matched'])
        self.assertEqual(6, report['carried_checks'])
        self.assertEqual(1, report['births'])

    def test_wrong_threshold_rejected(self):
        result = self.replay(fixture(), initial=6)
        self.assertNotEqual(0, result.returncode)
        self.assertIn('continuous class_after', result.stderr)

    def test_empty_detection_miss(self):
        rows = fixture()[:9]
        rows[5] = dict(event='class_miss', stamp_ns=100000000, id=7)
        after = copy.deepcopy(rows[1]['objects'][0])
        after['class_misses'] = 1
        for index in (6, 7, 8):
            rows[index]['objects'] = [copy.deepcopy(after)]
        rows.append(dict(event='fusion_summary', stamp_ns=0, matched=1,
                         dropped=0, pending_lidar=0, pending_yolo=0))
        result = self.replay(rows)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(1, json.loads(result.stdout)['misses'])

    def test_explicit_ground_noise_recovery(self):
        rows = fixture()[:17]
        noise = copy.deepcopy(rows[0]['object'])
        noise['class'] = 'noise'
        rows[0]['object'] = copy.deepcopy(noise)
        for index in (1, 4, 6, 7, 8):
            rows[index]['objects'] = [copy.deepcopy(noise)]
        initial = copy.deepcopy(fixture()[0]['object'])
        rows[9]['objects'] = [copy.deepcopy(initial)]
        rows[12]['objects'] = [copy.deepcopy(initial)]
        for index in (14, 15, 16):
            rows[index]['objects'][0]['confirmations'] = 1
        del rows[5]
        rows.insert(8, dict(event='class_tracking_reset', stamp_ns=200000000,
                           object=copy.deepcopy(initial)))
        rows.append(dict(event='fusion_summary', stamp_ns=0, matched=2,
                         dropped=0, pending_lidar=0, pending_yolo=0))
        result = self.replay(rows)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(1, json.loads(result.stdout)['noise_recoveries'])

    def test_disappearance_does_not_inject_new_state(self):
        rows = fixture()[:17]
        for index in (9, 12, 14, 15, 16):
            rows[index]['objects'] = []
        rows[16]['origins'] = []
        del rows[13]
        rows.append(dict(event='fusion_summary', stamp_ns=0, matched=2,
                         dropped=0, pending_lidar=0, pending_yolo=0))
        result = self.replay(rows)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(1, json.loads(result.stdout)['births'])

    def test_state_carry_loss_rejected(self):
        rows = fixture()
        next(row for row in rows if row['event'] == 'class_lidar_ready' and
             row['stamp_ns'] == 200000000)['objects'][0]['confirmations'] = 0
        self.rejected(rows, 'tracking carry')

    def test_class_before_is_oracle_not_initial_condition(self):
        rows = fixture()
        next(row for row in rows if row['event'] == 'class_before')['objects'][0]['confirmations'] = 99
        self.rejected(rows, 'historical carry')

    def test_duplicate_consumption_rejected(self):
        rows = fixture()
        rows.insert(9, dict(event='yolo_admitted', stamp_ns=100000000))
        self.rejected(rows, 'duplicate admitted/consumed')

    def test_missing_observation_is_not_filled(self):
        rows = fixture()
        del rows[5]
        self.rejected(rows, 'missing geometric observation')

    def test_missing_summary_rejected(self):
        self.rejected(fixture()[:-1], 'missing complete final summary')

    def test_missing_publication_rejected(self):
        rows = fixture()
        del rows[8]
        self.rejected(rows, 'unfinished previous frame')

    def test_bad_provenance_rejected(self):
        rows = fixture()
        rows[8]['origins'][0]['source_lidar_stamp_ns'] += 1
        self.rejected(rows, 'publication provenance mismatch')

    def test_missing_admission_rejected(self):
        rows = fixture()
        del rows[2]
        self.rejected(rows, 'YOLO history admission/pruning mismatch')

    def test_publication_generation_must_match_processed_frame(self):
        rows = fixture()
        rows[8]['generation'] = 2
        self.rejected(rows, 'publication generation mismatch')

    def test_duplicate_final_summary_rejected(self):
        rows = fixture()
        rows.append(copy.deepcopy(rows[-1]))
        self.rejected(rows, 'missing complete final summary')


if __name__ == '__main__':
    unittest.main()
