import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from asmc_camera.camera.backend import BackendInfo, CategoricalOutput
from asmc_camera.camera.inference_engine import InferenceEngine
from asmc_camera.camera.backends.twinlite_tensorrt import load_manifest
from asmc_camera.runtime.tensorrt import sha256_file
from asmc_camera.runtime.jsonl_journal import JsonlJournal


class StubBackend:
    info = BackendInfo('stub', 'medium', ('drivable','road_marking'), (32,64), 'cpu',
        '', '', 0, 'ema', '', {'road_marking':('background','white_lane','yellow_lane','stopline')})
    def infer(self, batch):
        shape = (batch.shape[0],)+tuple(batch.shape[2:])
        classes = np.zeros(shape,np.uint8); classes[:,10:12,15:20]=2
        return {'drivable':np.full(shape,.8,np.float32),
                'road_marking':CategoricalOutput(classes,np.full(shape,.9,np.float32))}


class DeploymentTest(unittest.TestCase):
    def test_lazy_native_does_not_restore_and_preserves_model_grid(self):
        image=np.zeros((48,64,3),np.uint8)
        engine=InferenceEngine(StubBackend())
        with patch('asmc_camera.camera.inference_engine.restore_class_ids') as restore_ids, \
             patch('asmc_camera.camera.inference_engine.restore_probability') as restore_probability:
            value=engine.infer({'front':image},native_outputs={'front':set()}).views['front']
        restore_ids.assert_not_called();restore_probability.assert_not_called()
        self.assertFalse(value.probabilities)
        self.assertFalse(value.categorical_confidences)
        self.assertEqual(set(value.model_grid_probabilities),{'road_marking'})
        self.assertEqual(value.model_grid_probabilities['road_marking'].shape,(32,64))

    def test_new_subscriber_next_batch_matches_original_output(self):
        engine=InferenceEngine(StubBackend());image=np.zeros((48,64,3),np.uint8)
        engine.infer({'front':image},native_outputs={'front':set()})
        selected=engine.infer({'front':image},native_outputs={'front':{'road_marking_confidence'}}).views['front']
        all_outputs=engine.infer({'front':image}).views['front']
        self.assertFalse(selected.probabilities)
        np.testing.assert_array_equal(selected.categorical_confidences['road_marking'],
                                      all_outputs.categorical_confidences['road_marking'])

    def test_manifest_rejects_wrong_classes_checkpoint_shape_or_engine(self):
        from asmc_camera.camera.backends import PINNED_TWINLITE_COMMIT
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);checkpoint=root/'source.pt';checkpoint.write_bytes(b'checkpoint')
            engine=root/'side.engine';engine.write_bytes(b'engine')
            row=dict(schema='asmc_twinlite_tensorrt_v1',completed=True,
                upstream_commit=PINNED_TWINLITE_COMMIT,model_config='medium',
                checkpoint_sha256=sha256_file(checkpoint),
                class_names={'drivable':['background','drivable'],
                             'road_marking':['background','white_lane','yellow_lane','stopline']},
                engines=[dict(hw=[480,640],max_batch=2,output_mode='logits',file=engine.name,sha256=sha256_file(engine))])
            path=root/'manifest.json'
            def check(data):
                path.write_text(json.dumps(data))
                return load_manifest(path,checkpoint,(480,640),{},'medium')
            check(row)
            for key,value in [('completed',False),('checkpoint_sha256','bad'),('model_config','large'),('upstream_commit','bad')]:
                with self.assertRaises(ValueError):check(dict(row,**{key:value}))
            wrong=json.loads(json.dumps(row));wrong['class_names']['road_marking'][1]='stopline'
            with self.assertRaises(ValueError):check(wrong)
            wrong=json.loads(json.dumps(row));wrong['engines'][0]['hw']=[384,640]
            with self.assertRaises(ValueError):check(wrong)
            engine.write_bytes(b'changed')
            with self.assertRaises(ValueError):check(row)

    def test_journal_is_bounded_and_drains_on_close(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'events.jsonl';journal=JsonlJournal(path,capacity=32)
            for index in range(1000):journal.append(dict(event='sample',index=index))
            journal.close()
            rows=list(map(json.loads,path.read_text().splitlines()))
            self.assertLessEqual(journal.high_water,32)
            self.assertEqual(journal.written+journal.dropped,1000)
            self.assertEqual(rows[-1]['event'],'journal_end')
            ids=[r['index'] for r in rows if r['event']=='sample']
            self.assertEqual(ids,sorted(set(ids)))
            self.assertFalse(journal.thread.is_alive())


if __name__=='__main__':unittest.main()
