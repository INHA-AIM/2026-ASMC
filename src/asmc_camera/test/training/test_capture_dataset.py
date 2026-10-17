"""Mixed deployment geometry, packed target and bounded balance contracts."""
from collections import Counter
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader

from asmc_camera.training.twinlite.batching import ShapeBatchSampler
from asmc_camera.training.twinlite.dataset import MoraiDatasetError, MoraiTwinLiteDataset
from asmc_camera.training.twinlite.tasks import ROAD_MARKING_CLASS_VALUES
from asmc_camera.training.twinlite.tasks import ROAD_MARKING_TASK


class CaptureDatasetTests(unittest.TestCase):
    def build(self, root):
        data = root/'prepared'; source = root/'raw'
        source.mkdir(); version = data/'dataset_versions/capture'
        version.mkdir(parents=True)
        rows = []
        for index,(view,h,w) in enumerate([('front',8,12),('left',8,10),('right',8,10)]):
            cv2.imwrite(str(source/(view+'.png')),np.full((h-2,w,3),(20,40,60),np.uint8))
            packed = np.full((h,w),8,np.uint8)
            packed[[0,-1]] = 0
            packed[2,2] = 11; packed[3,3] = 13; packed[4,4] = 15
            cv2.imwrite(str(data/(view+'.png')),packed)
            rows.append(dict(schema_version='morai-perception-sample-1.2.0',
                sample_id=view,run_id='fixture',frame_id=index,view=view,split='train',
                image='raw:'+view+'.png',source_ref=dict(root='raw',path=view+'.png'),
                packed_targets=view+'.png',targets=dict(drivable=view+'.png',road_marking=view+'.png'),
                valid_mask=view+'.png',
                geometry=dict(original_hw=[h-2,w],target_hw=[h,w],resized_hw=[h-2,w],
                    pad_ltrb=[0,1,0,1],scale=1.,scale_xy=[1.,1.],valid_roi_xyxy=[0,1,w,h-1]),
                target_encodings=dict(road_marking=dict(type='categorical_uint8',class_values=ROAD_MARKING_CLASS_VALUES),
                    packed_targets=dict(type='uint8_bitfield',drivable_bit=0,road_marking_bits=[1,2],valid_bit=3)),
                weather='SUNNY',hour=11))
        (version/'dataset.json').write_text(json.dumps(dict(schema_version='morai-perception-dataset-1.2.0',
            target_hw=[8,10],target_hw_by_view=dict(front=[8,12],left=[8,10],right=[8,10]),
            training_heads=['drivable','road_marking'],source_roots_required=['raw'])))
        (version/'train.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        (version/'_SUCCESS').write_text('{}')
        return data,source,rows

    def test_packed_targets_and_front_padding_reach_batch(self):
        with tempfile.TemporaryDirectory() as tmp:
            data,source,rows = self.build(Path(tmp))
            ds = MoraiTwinLiteDataset(data,'capture',source_roots={'raw':source})
            sample = ds[0]
            self.assertEqual(tuple(sample['image'].shape),(3,8,12))
            self.assertEqual(sample['targets']['road_marking'][2,2].item(),1)
            self.assertEqual(sample['targets']['road_marking'][3,3].item(),2)
            self.assertEqual(sample['targets']['road_marking'][4,4].item(),3)
            self.assertFalse(sample['valid_mask'][0].any())
            self.assertTrue(torch.allclose(sample['image'][:,0],torch.full((3,12),114/255)))
            batches = list(DataLoader(ds,batch_sampler=ShapeBatchSampler(rows,2)))
            self.assertEqual(sorted(tuple(b['image'].shape) for b in batches),[(1,3,8,12),(2,3,8,10)])

    def test_roots_are_explicit_and_cannot_escape(self):
        with tempfile.TemporaryDirectory() as tmp:
            data,source,rows = self.build(Path(tmp))
            with self.assertRaisesRegex(MoraiDatasetError,'explicit source roots'):
                MoraiTwinLiteDataset(data,'capture',source_roots={})
            rows[0]['source_ref']['path'] = '../outside.png'
            (data/'dataset_versions/capture/train.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            with self.assertRaisesRegex(MoraiDatasetError,'escapes'):
                MoraiTwinLiteDataset(data,'capture',source_roots={'raw':source})

    def test_reserved_bits_and_wrong_target_geometry_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            data,source,_ = self.build(Path(tmp))
            ds = MoraiTwinLiteDataset(data,'capture',source_roots={'raw':source})
            cv2.imwrite(str(data/'front.png'),np.full((8,12),128,np.uint8))
            with self.assertRaisesRegex(MoraiDatasetError,'invalid packed'):
                ds[0]
            cv2.imwrite(str(data/'front.png'),np.zeros((4,6),np.uint8))
            with self.assertRaisesRegex(MoraiDatasetError,'invalid packed'):
                ds[0]

    def test_condition_balance_uses_every_image_at_most_twice(self):
        rows = [dict(geometry={'target_hw':[8,12] if i%3 == 0 else [8,10]},
                     weather='SUNNY' if i<8 else 'FOGGY',hour=11) for i in range(10)]
        sampler = ShapeBatchSampler(rows,3,seed=17,balance_conditions=True)
        batches = list(sampler)
        counts = Counter(i for batch in batches for i in batch)
        self.assertEqual(set(counts),set(range(10)))
        self.assertEqual(max(counts.values()),2)
        self.assertEqual(counts[8],2); self.assertEqual(counts[9],2)
        self.assertEqual(batches,list(sampler))
        self.assertEqual(len(batches),len(sampler))
        for batch in batches:
            self.assertEqual(len({tuple(rows[i]['geometry']['target_hw']) for i in batch}),1)
        sampler.set_epoch(1)
        self.assertNotEqual(batches,list(sampler))
        evaluation = ShapeBatchSampler(rows,3,shuffle=False)
        self.assertEqual(Counter(i for b in evaluation for i in b),Counter(range(10)))

    def test_weather_time_macro_does_not_follow_the_largest_condition(self):
        path = Path(__file__).resolve().parents[4]/'scripts/train_twinlite.py'
        spec = importlib.util.spec_from_file_location('capture_training_metrics',path)
        trainer = importlib.util.module_from_spec(spec); spec.loader.exec_module(trainer)
        confusion = trainer._new_confusion(ROAD_MARKING_TASK)
        for index,(weather,hour) in enumerate((w,h) for w in ('SUNNY','FOGGY') for h in ('11','13','15')):
            count = 20 if index == 0 else 1
            target = torch.ones(count,2,2,dtype=torch.long)
            outputs = {}
            for head,classes in [('drivable',2),('road_marking',4)]:
                logits = torch.zeros(count,classes,2,2)
                logits[:,1 if index == 0 else 0] = 1
                outputs[head] = logits
            batch = dict(meta=dict(view=['front']*count,weather=[weather]*count,hour=[hour]*count),
                         targets=dict(drivable=target,road_marking=target),
                         valid_mask=torch.ones_like(target,dtype=torch.bool))
            trainer._update_confusion(confusion,outputs,batch,ROAD_MARKING_TASK)
        metrics = trainer._confusion_to_metrics(confusion,ROAD_MARKING_TASK)
        self.assertAlmostEqual(metrics['drivable']['all']['iou'],20/25)
        scores = [metrics['drivable'][w+':'+h]['iou'] for w in ('SUNNY','FOGGY') for h in ('11','13','15')]
        self.assertAlmostEqual(sum(scores)/len(scores),1/6)

    def test_frozen_four_class_evaluator_accepts_mixed_geometry(self):
        class Tiny(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.da = torch.nn.Conv2d(3,2,1)
                self.marking = torch.nn.Conv2d(3,4,1)
            def forward(self,image):
                return self.da(image),self.marking(image)
        path = Path(__file__).resolve().parents[4]/'scripts/evaluate_twinlite.py'
        spec = importlib.util.spec_from_file_location('capture_frozen_eval',path)
        evaluator = importlib.util.module_from_spec(spec); spec.loader.exec_module(evaluator)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); data,source,_ = self.build(root)
            model = Tiny(); checkpoint = root/'synthetic-fixture.pt'
            torch.save(dict(model_state_dict=model.state_dict(),upstream_commit='fixture',
                            task_spec=ROAD_MARKING_TASK.as_dict()),checkpoint)
            args = SimpleNamespace(data_root=str(data),dataset_version='capture',split=['train'],
                checkpoint=str(checkpoint),output_dir=str(root/'results'),twinlite_root=None,
                config='large',batch_size=2,num_workers=0,device='cpu',weights='model',
                no_amp=True,channels_last=False,log_interval=0,max_samples=None)
            with patch.object(evaluator,'parse_args',return_value=args), \
                 patch.object(evaluator,'load_external_twinlite',return_value=(model,'fixture')), \
                 patch.object(evaluator,'configure_twinlite_task'), \
                 patch.dict('os.environ',{'ASMC_TWINLITE_SOURCE_ROOTS':json.dumps({'raw':str(source)})}), \
                 patch('builtins.print'):
                evaluator.main()
            report = json.loads((root/'results/metrics.json').read_text())
            self.assertFalse(report['training_performed'])
            self.assertEqual(report['results']['train']['sample_count'],3)
            self.assertIn('road_marking',report['results']['train']['metrics'])


if __name__ == '__main__':
    unittest.main()
