from datetime import timedelta
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import torch
from torch.utils.data import Dataset,DataLoader

import run_twinlite_training as training


class Tiny(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.dropout=torch.nn.Dropout2d(.1)
        self.da=torch.nn.Conv2d(3,2,1)
        self.mark=torch.nn.Conv2d(3,4,1)
    def forward(self,image):
        image=self.dropout(image)
        return self.da(image),self.mark(image)


class Samples(Dataset):
    def __init__(self):
        self.rows=[dict(geometry={'target_hw':[8,10]}) for _ in range(8)]
    def __len__(self):return len(self.rows)
    def __getitem__(self,index):
        return dict(image=torch.full((3,8,10),(index+1)/10),
                    targets=dict(drivable=torch.ones(8,10,dtype=torch.long),
                                 road_marking=torch.ones(8,10,dtype=torch.long)),
                    valid_mask=torch.ones(8,10,dtype=torch.bool))


class SessionTests(unittest.TestCase):
    def session(self,root):
        args=SimpleNamespace(epochs=4,batch_size=2,accumulate=3,lr=.0002,
                             weight_decay=.0005,seed=17,config='fixture',checkpoint_batches=3)
        started=training.utc();limit=dict(started_at_utc=started.isoformat(),
            deadline_utc=(started+timedelta(hours=24)).isoformat(),max_hours=24)
        reporter=training.Reporter(root/'monitor',{})
        s=training.Session(args,reporter,limit);s.output=root;s.monitor=root/'monitor'
        torch.manual_seed(9);s.model=Tiny();s.device=torch.device('cpu')
        s.ema=training.base.ModelEMA(s.model)
        s.optimizer=torch.optim.AdamW(s.model.parameters(),lr=args.lr,weight_decay=args.weight_decay)
        s.scheduler=torch.optim.lr_scheduler.LambdaLR(s.optimizer,lambda epoch:(1-epoch/4)**1.5)
        s.scaler=torch.cuda.amp.GradScaler(enabled=False)
        s.criterion=training.base.MaskedTwinLiteLoss(task_spec=training.base.ROAD_MARKING_TASK)
        s.task=training.base.ROAD_MARKING_TASK;s.upstream='fixture';s.provenance={'sha':'fixture'}
        s.initialization={'fixture':True};s.sampler=training.ShapeBatchSampler(Samples().rows,2,shuffle=False)
        s.offset=training.OffsetBatches(s.sampler)
        s.train_loader=DataLoader(Samples(),batch_sampler=s.offset,
                                 generator=torch.Generator().manual_seed(17))
        s.epoch_exposures=8
        return s

    def test_mid_epoch_resume_restores_optimizer_ema_rng_and_next_batch(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);full_dir=root/'full';full_dir.mkdir()
            full=self.session(full_dir);torch.manual_seed(100);full.train_epoch()
            part_dir=root/'part';part_dir.mkdir();part=self.session(part_dir)
            original=part.save_progress
            def save_and_request_stop():
                original();part.requested_stop=True
            part.save_progress=save_and_request_stop
            torch.manual_seed(100)
            with self.assertRaises(training.StopTraining):part.train_epoch()
            checkpoint=torch.load(part_dir/'latest-progress.pt',map_location='cpu')
            self.assertEqual(checkpoint['next_batch'],3)
            self.assertFalse(checkpoint['epoch_complete'])
            resumed=self.session(part_dir);resumed.limit=part.limit
            resumed.restore(checkpoint);resumed.train_epoch()
            self.assertEqual(resumed.optimizer_steps,full.optimizer_steps)
            self.assertEqual(resumed.epoch_samples,8)
            self.assertEqual(resumed.next_batch,4)
            for name,tensor in full.model.state_dict().items():
                self.assertTrue(torch.equal(tensor,resumed.model.state_dict()[name]),name)
            for name,tensor in full.ema.model.state_dict().items():
                self.assertTrue(torch.equal(tensor,resumed.ema.model.state_dict()[name]),name)

    def test_completed_checkpoint_resumes_next_epoch_and_keeps_deadline(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);s=self.session(root);s.train_epoch();s.scheduler.step()
            s.completed_epochs=1;s.history=[dict(epoch=1,validation={'complete':True})]
            s.save_progress();checkpoint=torch.load(root/'latest-progress.pt',map_location='cpu')
            self.assertTrue(checkpoint['epoch_complete'])
            resumed=self.session(root);resumed.limit=s.limit;resumed.restore(checkpoint)
            self.assertEqual(resumed.epoch,2);self.assertEqual(resumed.next_batch,0)
            self.assertEqual(resumed.optimizer.param_groups[0]['lr'],s.optimizer.param_groups[0]['lr'])
            self.assertEqual(resumed.limit,s.limit)
            checkpoint['dataset_provenance']={'sha':'wrong'}
            with self.assertRaisesRegex(RuntimeError,'dataset SHA'):resumed.restore(checkpoint)

    def test_budget_stop_and_reporter_are_explicit(self):
        with tempfile.TemporaryDirectory() as temp:
            s=self.session(Path(temp));s.deadline=training.utc()-timedelta(seconds=1)
            with self.assertRaises(training.StopTraining):s.check_stop()
            self.assertEqual(s.stop_reason,'time_budget')
            s.report(force=True,state='stopped')
            data=json.loads((Path(temp)/'monitor/progress.json').read_text())
            self.assertFalse(data['training_performed'])
            self.assertEqual(data['state'],'stopped')

    def test_offset_sampler_skips_without_reading_dataset(self):
        original=training.ShapeBatchSampler(Samples().rows,2,shuffle=False)
        offset=training.OffsetBatches(original);offset.start=3
        self.assertEqual(list(offset),list(original)[3:])
        self.assertEqual(len(offset),1)


if __name__=='__main__':unittest.main()
