#!/usr/bin/env python3
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from score import evaluate


class ScoringTest(unittest.TestCase):
    def test_object_loss_is_not_clean_ground_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'input';results=Path(tmp)/'results';out=Path(tmp)/'score'
            (root/'labels').mkdir(parents=True);results.mkdir()
            # One observed vehicle, one pedestrian, one low obstacle, asphalt and sidewalk.
            xyz=np.array([[i*.04, j*2., -1.8+(i%4)*.1, i] for j in range(5) for i in range(12)],np.float32)
            xyz.tofile(root/'cloud.bin')
            semantic=np.repeat([125,118,164,127,129],12);instance=np.repeat([1,2,3,0,0],12)
            np.savez(root/'labels'/'test.npz',semantic=semantic,instance=instance)
            (root/'snapshots.jsonl').write_text(json.dumps({'input_id':'test','scan_effective_stamp_ns':123,
                'cloud_path':'cloud.bin','pitch':0.})+'\n')
            configs=Path(__file__).resolve().parents[3]/'src/asmc_lidar/config/ground_v2'
            for config in configs.glob('*.yaml'):
                name=config.stem;methods=['jaeho','patchwork','linefit'] if name=='v1' else ['patchwork' if name.startswith('pp') else 'linefit']
                row={'input_id':'test','stamp_ns':123,'raw_to_voxel':list(range(60)),'prepare_ms':1,
                     'config_id':name,'config_sha256':'test','methods':{}}
                for method in methods:
                    mask=[2]*60
                    if name=='pp_min_5':mask[:12]=[1]*12
                    if name=='lf_covered':mask[36:]=[1]*24
                    if name=='pp_min_15':mask[:12]=[3]*12
                    row['methods'][method]={'mask':mask,'reasons':[1]*60,'pre_guard_mask':mask,'events':{},
                        'copy_ms':0,'segment_ms':1,'guard_ms':0,'restore_ms':0}
                (results/(name+'.jsonl')).write_text(json.dumps(row)+'\n')
            evaluate(root,results,out)
            scored=json.loads((out/'summary.json').read_text())
            self.assertFalse(scored['gates']['pp_min_5']['object_pass'])
            self.assertEqual(scored['regressions']['pp_min_5']['additional_complete_loss'],1)
            self.assertIn('unclassified_increase',scored['gates']['pp_min_15']['violations'])
            self.assertTrue(scored['gates']['lf_covered']['object_pass'])
            self.assertEqual(scored['objects']['v1_jaeho']['vehicle/observed10plus']['observations'],1)
            self.assertNotIn('asphalt/observed10plus',scored['objects']['v1_jaeho'])


if __name__=='__main__':unittest.main()
