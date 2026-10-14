import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/perception'))
sys.path.insert(0, str(ROOT/'tools/audit'))
from compare_camera_runtime import measurements
from validate_p1b_contract import load_message_specs, all_message_md5
from evaluate_camera_bbox_pilot import frame_counts, summarize


class RuntimeMeasurementsTest(unittest.TestCase):
    def test_raw_percentiles_full_windows_gaps_and_duplicate_count(self):
        rows = [dict(event='output', at=.1+i*.1, stamp=i+1, received=.05+i*.1,
                     wall_ms=i, valid=True, stale=False,
                     topic='/perception/camera/objects', frame='camera_front') for i in range(59)]
        result = next(iter(measurements(rows,0,5.9,15).values()))
        self.assertEqual(len(result['five_second_hz']), 1)
        self.assertEqual(result['five_second_hz'][0],9.8)
        self.assertAlmostEqual(result['wall_ms']['p50'],28.5)
        self.assertAlmostEqual(result['wall_ms']['p95'],54.15)
        self.assertFalse(result['target_95pct_pass'])
        self.assertTrue(result['minimum_9_5_pass'])
        self.assertEqual(result['duplicate_stamps'],0)
        rows.append(dict(rows[10],at=5.85))
        result = next(iter(measurements(rows,0,5.9,15).values()))
        self.assertEqual(result['duplicate_stamps'],1)
        self.assertEqual(result['reversed_stamps'],1)

    def test_invalid_and_stale_outputs_do_not_raise_fresh_rate(self):
        row = dict(event='output',at=1,stamp=1,received=.9,wall_ms=100,
                   topic='/perception/camera/objects',frame='camera_front',valid=False,stale=False)
        self.assertFalse(measurements([row,dict(row,valid=True,stale=True)],0,5,15))

    def test_frozen_old_md5_and_additive_observation_fields(self):
        specs = load_message_specs(ROOT/'src/asmc_msgs/msg')
        md5 = all_message_md5(specs)
        self.assertEqual(md5['CameraObjectBox'],'34412c90553e06734f1acade77c8e1e3')
        self.assertEqual(md5['CameraObjectBoxArray'],'310c671afb3693e7909db6453d115b79')
        self.assertEqual(md5['CameraObjectObservation'],'fbddb8fe0071685b60811037c0302106')
        self.assertEqual([f.name for f in specs['CameraObjectObservation'].fields],
                         ['detections','producer_session_id','generation','calibration_id','model_id','config_id'])

    def test_frozen_small_positive_duplicate_fp_partial_and_sha(self):
        row=dict(id='a',image_sha256='sha',supervision='complete',split_group='run',annotations=[
            dict(training_class='car',xyxy=[0,0,8,8],supervision='positive'),
            dict(training_class='person',xyxy=[20,20,22,22],supervision='ignore')])
        boxes=[dict(class_name='car',xyxy=[0,0,8,8],confidence=.8),
               dict(class_name='car',xyxy=[0,0,8,8],confidence=.7),
               dict(class_name='person',xyxy=[20,20,22,22],confidence=.9)]
        counts,pairs=frame_counts(row,boxes,.4,pixel_scope=True)
        self.assertEqual(len(pairs),1)
        self.assertEqual(counts['car']['complete_gt'],1)  # no legacy 24/12px display filter
        self.assertEqual(counts['car']['complete_predictions'],2)
        self.assertEqual(counts['person']['complete_predictions'],0)
        partial=dict(row,supervision='partial')
        values,_=frame_counts(partial,boxes,.4,pixel_scope=True)
        self.assertEqual(values['car']['tp'],1)
        self.assertEqual(values['car']['complete_predictions'],0)
        with self.assertRaises(ValueError):
            summarize([row],[dict(id='a',image_sha256='wrong',predictions=boxes)],.4,pixel_scope=True)


if __name__ == '__main__':
    unittest.main()
