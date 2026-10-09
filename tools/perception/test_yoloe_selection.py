import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from prepare_yoloe_l import export_profile
from write_yoloe_selection_configs import configuration
from evaluate_coco7_models import evaluate
from check_yoloe_selection_engine import compare
from analyze_yoloe_selection import best_threshold,EVALUATION_GROUPS,input_pauses,resolution_choice
from asmc_camera.detection.batching import shapes_for_front,pack_image,ImageTransform
from asmc_camera.detection.batched_backend import BatchedSegmentationDetector
from asmc_camera.detection.backend import Detection
from run_coco7_weather_matrix import candidate_order
from b0_measurements import quantiles, output_report


class YoloeSelectionTest(unittest.TestCase):
    def test_resolution_selection_requires_both_weather_and_all_stack_gates(self):
        quality=[dict(model='yoloe-26l-seg',width=w,reviewed_fixed=dict(pooled=dict(macro_f1=f),
            temporal_objects=[],temporal_summary=dict(not_stably_detected=2))) for w,f in [(960,.26),(1280,.27)]]
        live=[dict(candidate=dict(model='yoloe-26l-seg',width=w),weather=weather,whole_stack_passed=True,
            wall_p95_ms_max=100.,peak_whole_vram_mib=5500.) for w in (960,1280) for weather in ('sunny','foggy')]
        self.assertEqual(1280,resolution_choice(quality,live)['selected']['width'])
        live[-1]['whole_stack_passed']=False
        self.assertEqual(960,resolution_choice(quality,live)['selected']['width'])
        live[0]['whole_stack_passed']=False
        self.assertIsNone(resolution_choice(quality,live)['selected'])
        self.assertIsNone(resolution_choice(quality,[])['selected'])

    def test_fixture_override_keeps_nine_models_and_rejects_foreign_models(self):
        from coco7_scene import FIXTURE,load_fixture
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.json'
            rows=[list(row) for row in FIXTURE];rows[4][3]=2.
            path.write_text(json.dumps(rows))
            self.assertEqual(2.,load_fixture(path)[4][3])
            rows[4][1]='unknown';path.write_text(json.dumps(rows))
            with self.assertRaises(ValueError):load_fixture(path)

    def test_three_candidate_cycles_balance_each_position_without_changing_legacy_order(self):
        candidates=['baseline','960','1280']
        rounds=[candidate_order(candidates,r,'cyclic') for r in (1,2,3)]
        for position in range(3):
            self.assertEqual(set(candidates),{r[position] for r in rounds})
        self.assertEqual(candidates,candidate_order(candidates,1,'alternate'))
        self.assertEqual(candidates[::-1],candidate_order(candidates,2,'alternate'))

    def test_latency_percentiles_use_observations_not_mean_of_run_medians(self):
        runs=[[10.,10.,10.],[100.]]
        self.assertEqual(10.,quantiles(sum(runs,[]))['p50'])
        self.assertNotEqual(55.,quantiles(sum(runs,[]))['p50'])
        self.assertIsNone(quantiles([])['p50'])

    def test_latency_excludes_old_generation_and_incomplete_windows_fail(self):
        rows=[dict(arrival=t,stamp_ns=stamp,valid=True,wall_latency_ms=latency,
                   source_age_ms=latency,input_anchor='matched')
              for t,stamp,latency in [(10.1,100,999.),(10.2,101,40.),(10.3,102,60.),(11.,103,500.)]]
        report=output_report(rows,10.,10.5,100,1.)
        self.assertFalse(report['complete'])
        self.assertEqual(50.,report['input_to_output_wall_ms']['p50'])
        self.assertEqual(2,report['input_to_output_wall_ms']['samples'])

    def test_input_pause_diagnostics_exclude_stale_rows_and_keep_source_time_separate(self):
        rows=[dict(arrival=t,stamp_ns=ns) for t,ns in
              [(1.,0),(10.1,100_000_000),(10.5,150_000_000),(10.55,200_000_000),(30.,300_000_000)]]
        report=input_pauses(rows,10.,20.)
        self.assertAlmostEqual(400.,report['maximum_gap_ms'])
        self.assertEqual(1,len(report['pauses']))
        self.assertEqual(50.,report['pauses'][0]['source_gap_ms'])
        self.assertIsNone(input_pauses(rows,15.,20.)['maximum_gap_ms'])

    def test_selection_tie_and_evaluation_only_groups(self):
        thresholds=[dict(confidence=.2,macro_f1=.5),dict(confidence=.4,macro_f1=.5),dict(confidence=.7,macro_f1=None)]
        self.assertEqual(.4,best_threshold(thresholds)['confidence'])
        self.assertIsNone(best_threshold([]))
        self.assertEqual('animal',EVALUATION_GROUPS['deer'])
        self.assertEqual('vehicle',EVALUATION_GROUPS['scooter'])
        self.assertEqual('obstacle',EVALUATION_GROUPS['traffic bollard'])
        self.assertEqual(18,len(EVALUATION_GROUPS))  # original17 plus tested alternative name

    def test_engine_correspondence_preserves_labels_and_one_to_one_matching(self):
        ref=[dict(group='car',xyxy=[0,0,10,10])]
        current=[dict(group='truck',xyxy=[0,0,10,10])]
        self.assertEqual(0,compare(ref,current)[1]['matched'])
        current=[dict(group='car',xyxy=[0,0,10,10])]*2
        self.assertEqual(1,compare(ref,current)[1]['engine_unmatched'])

    def test_export_profile_covers_front_sides_and_common_batch_without_upscaling(self):
        for width in (640,960,1280):
            front,minimum,optimum,maximum=export_profile(width)
            shapes=shapes_for_front(width)
            for shape in [(1,3,*front),(2,3,480,640),(3,3,max(front[0],480),width)]:
                self.assertTrue(all(lo<=x<=hi for lo,x,hi in zip(minimum,shape,maximum)))
            image=np.zeros((720,1280,3),np.uint8)
            _,t=pack_image(image,'front',shapes=shapes)
            d=Detection(t.left,t.top,t.left+t.resized_hw[1],t.top+t.resized_hw[0],'car',.8)
            restored=t.box(d)
            self.assertEqual([0.,0.,1280.,720.],[restored.xmin,restored.ymin,restored.xmax,restored.ymax])
            bitmap=np.zeros(t.tensor_hw,bool);bitmap[t.top:t.top+t.resized_hw[0],t.left:t.left+t.resized_hw[1]]=True
            self.assertTrue(t.mask(bitmap).all())
        self.assertEqual(384,export_profile(640)[0][0])
        with self.assertRaises(ValueError):export_profile(800)

    def test_actual_yoloe_batch_path_honors_width_and_isolates_absent_views(self):
        class Model:
            def __init__(self):self.calls=[]
            def predict(self,images,**kwargs):
                self.calls.append([im.shape for im in images])
                return [SimpleNamespace(names={0:'person'},boxes=[],masks=None) for im in images]
        node=object.__new__(BatchedSegmentationDetector)
        node.device='cuda:0';node.conf=.4;node.names_list=['person']
        node.torch=SimpleNamespace(cuda=SimpleNamespace(synchronize=lambda _:None))
        images=dict(front=np.zeros((720,1280,3),np.uint8),left=np.zeros((480,640,3),np.uint8),right=np.zeros((480,640,3),np.uint8))
        for width in (640,960,1280):
            node.shapes=shapes_for_front(width)
            for mode in ('2plus1','batch3'):
                node.mode=mode;node.model=Model()
                with patch.dict(sys.modules,{'ultralytics.utils.ops':SimpleNamespace(scale_masks=lambda *a:None)}):
                    output=node.infer_images(images,['front','left','right'])
                self.assertEqual(set(images),set(output))
                self.assertEqual(3,sum(len(c) for c in node.model.calls))
                if mode=='2plus1':self.assertEqual((node.shapes['front'][0],width,3),node.model.calls[0][0])
                else:self.assertEqual([(max(node.shapes['front'][0],480),width,3)]*3,node.model.calls[0])
                node.model.calls=[]
                with patch.dict(sys.modules,{'ultralytics.utils.ops':SimpleNamespace(scale_masks=lambda *a:None)}):
                    self.assertEqual({'front'},set(node.infer_images(images,['front'])))
                self.assertEqual(1,sum(len(c) for c in node.model.calls))

    def test_checkpoint_prompt_and_exported_vocabulary_must_agree(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);sub=root/'m';sub.mkdir();checkpoint=sub/'yoloe-26m-seg.pt';checkpoint.write_bytes(b'm-checkpoint')
            digest=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
            prompt=sub/'obstacle17.npz'
            np.savez(prompt,names=['car'],embeddings=np.zeros((1,1,8)),weights_sha256=np.array(digest))
            engine=sub/'obstacle17-640.engine'
            def header(names):
                data=json.dumps(dict(names={0:names},end2end=False,task='segment')).encode()
                engine.write_bytes(struct.pack('<i',len(data))+data+b'fake-plan')
            header('car')
            cfg=configuration(root,'m',640,['car'],'/models')
            self.assertEqual('source',cfg['label_mode'])
            self.assertFalse(cfg['batched_runtime'])  # Override the preceding COCO launch's private parameter.
            header('truck')
            with self.assertRaises(ValueError):configuration(root,'m',640,['car'],'/models')
            header('car');checkpoint.write_bytes(b'l-checkpoint')
            with self.assertRaises(ValueError):configuration(root,'m',640,['car'],'/models')

    def test_unannotated_new_prompt_is_not_a_false_positive_in_coco7_metric(self):
        gt=[dict(source_class_name='car',subtype_verified=True,actor_id=1,xyxy=[0,0,10,10])]
        predicted=[dict(source_class_name='car',confidence=.8,xyxy=[0,0,10,10]),dict(source_class_name='wooden box',confidence=.9,xyxy=[20,20,30,30])]
        counts,_,_,_,_=evaluate(gt,predicted,.4)
        self.assertEqual(1,counts['car']['tp'])
        self.assertEqual(0,sum(c['fp'] for c in counts.values()))


if __name__=='__main__':unittest.main()
