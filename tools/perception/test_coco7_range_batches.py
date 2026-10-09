import unittest
import json
from pathlib import Path
import tempfile
from analyze_coco7_range_batches import summarize_batches, summarize_ranges, target_observation


class RangeBatchEvidenceTests(unittest.TestCase):
    def test_other_actor_same_label_does_not_count(self):
        boxes=[dict(label='car',confidence=.9,xyxy=[0,0,20,20]),
               dict(label='truck',confidence=.8,xyxy=[590,290,650,350])]
        result=target_observation(boxes,'car',[570,230,685,455])
        self.assertFalse(result['expected_label_present'])
        self.assertEqual(result['labels'],['truck'])

    def test_failures_not_dropped_or_marked_completed(self):
        report=dict(runs=[dict(model='m',width=640,batch='batch3',repeat=1,error='OOM')])
        row=summarize_batches(report)[0]
        self.assertEqual(row['completed_repeats'],0)
        self.assertEqual(row['failures'],['OOM'])
        report['runs']*=2
        with self.assertRaises(ValueError):summarize_batches(report)

    def test_unfinished_missing_duplicate_and_wrong_rgb_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture=Path(tmp)/'sunny';capture.mkdir()
            profile=Path(tmp)/'model';profile.mkdir()
            case=dict(index=0,asset=dict(api_name='Car',expected_label='car'),placement='front20',
                      requested_forward_anchor_m=20,bbox_center_distance_from_ego_anchor_m=21,
                      frames=[dict(view='front',source_stamp_ns=1,sha256='rgb')])
            manifest=dict(ego_after={'stationary':True},cases=[case])
            row=dict(registry=str(capture),case=0,source_stamp_ns=1,sha256='rgb',predictions=[])
            inspection=dict(candidates=[dict(profile_path=profile,config={})])
            (capture/'capture.json').write_text(json.dumps(manifest))
            predictions=profile/'predictions.jsonl'
            for rows in ([],[row,row],[dict(row,sha256='other')]):
                predictions.write_text('\n'.join(json.dumps(r) for r in rows))
                with self.assertRaises(ValueError):summarize_ranges(inspection,[capture])
            predictions.write_text(json.dumps(row))
            self.assertEqual(summarize_ranges(inspection,[capture])[0]['observations'][0]['frames'],1)
            manifest.pop('ego_after')
            (capture/'capture.json').write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):summarize_ranges(inspection,[capture])


if __name__=='__main__':unittest.main()
