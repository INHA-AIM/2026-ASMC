import copy
import json
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/asmc_camera/src'))
from asmc_camera.detection.rfdetr_core import COCO7,COCO7_IDS,MEAN,STD,decode,groups,preprocess,validate_metadata
from evaluate_rfdetr_candidate import next_step,quality_gate,seven_rows,validate_saved_evaluation
from camera_data_audit import sha
from check_rfdetr_parity import compare
from asmc_camera.detection.backend import Detection


class RFDetrTest(unittest.TestCase):
    def output(self,batch=1,queries=1):
        return np.tile([[[.5,.5,.4,.2]]],(batch,queries,1)),np.full((batch,queries,91),-20.,np.float32)

    def test_rgb_normalization_and_complete_fov(self):
        image=np.zeros((16,40,3),np.uint8);image[:]=(0,127,255)
        value=preprocess(image,[32,64])
        self.assertEqual(value.shape,(3,32,64));self.assertTrue(value.flags.c_contiguous)
        np.testing.assert_allclose(value[:,0,0],(np.array([1,127/255,0])-MEAN)/STD,atol=1e-6)
        np.testing.assert_equal(value[:,0,0],value[:,-1,-1])
        for shape in ([31,64],[0,32],[32]):
            with self.assertRaises(ValueError):preprocess(image,shape)

    def test_sparse_coco_ids_background_and_original_coordinates(self):
        b,l=self.output();l[0,0,0]=10;l[0,0,1]=3;l[0,0,18]=2;l[0,0,90]=9
        out=decode(b,l,[(100,200)],.4)[0]
        self.assertEqual([d.source_class_name for d in out],['person','dog'])
        np.testing.assert_allclose([out[0].xmin,out[0].ymin,out[0].xmax,out[0].ymax],[60,40,140,60])

    def test_global_top300_before_seven_class_filter(self):
        b,l=self.output(queries=301);l[0,:,5]=10;l[0,0,1]=9
        self.assertEqual(decode(b,l,[(100,200)],.4),[[]])

    def test_batch_correspondence_and_empty_result(self):
        b,l=self.output(batch=3)
        for i,cid in enumerate((1,3,18)):l[i,0,cid]=2
        ds=decode(b,l,[(100,200)]*3)
        self.assertEqual([d[0].source_class_name for d in ds],['person','car','dog'])
        self.assertEqual(decode(b,l,[(100,200)]*3,1),[[],[],[]])

    def test_invalid_outputs_are_not_empty_normal_observations(self):
        b,l=self.output();l[0,0,1]=np.nan
        with self.assertRaises(ValueError):decode(b,l,[(100,200)])
        with self.assertRaises(ValueError):decode(b,l[:,:,:80],[(100,200)])

    def test_stopped_view_does_not_block_other_batches(self):
        s={v:[704,704] for v in ('front','left','right')}
        self.assertEqual(groups(['right'],'batch3',s),[['right']])
        self.assertEqual(groups(['front','right'],'2plus1',s),[['right'],['front']])
        s['front']=[544,960]
        with self.assertRaises(ValueError):groups(['front','left'],'batch3',s)
        self.assertEqual(groups(['front','left','right'],'2plus1',s),[['left','right'],['front']])

    def test_default_launch_resets_backend_before_experiment_override(self):
        root=Path(__file__).resolve().parents[2]
        node=ET.parse(root/'src/asmc_camera/launch/camera_objects.launch').getroot().find('node')
        children=list(node)
        reset=next(c for c in children if c.tag=='rosparam' and c.get('param')=='model_kind')
        config=next(c for c in children if c.tag=='rosparam' and c.get('file')=='$(arg config)')
        self.assertEqual(reset.get('command'),'delete')
        self.assertLess(children.index(reset),children.index(config))

    def test_quality_threshold_and_small_sample_cannot_pass(self):
        b={'classes':{c:dict(sufficient_sample=True,f1=.8,gt=30,annotated_recall=.9) for c in COCO7}}
        c=copy.deepcopy(b);self.assertTrue(quality_gate(b,c)['passed'])
        c['classes']['motorcycle']['annotated_recall']=.87
        self.assertFalse(quality_gate(b,c)['passed'])
        c=copy.deepcopy(b)
        for v in c['classes'].values():v['f1']=.78
        self.assertFalse(quality_gate(b,c)['passed'])
        for v in b['classes'].values():v['sufficient_sample']=False
        self.assertFalse(quality_gate(b,c)['passed'])

    def test_early_stop_and_conditional_fallbacks(self):
        self.assertIn('resolve_backend_parity',next_step(True,True,parity_pass=False))
        self.assertIn('skip_M',next_step(True,True))
        self.assertEqual(next_step(True,False),'l-640-batch3')
        self.assertEqual(next_step(True,False,attempt='l-640'),'m-576-batch3')
        self.assertEqual(next_step(False,True,small_object_cause=True),'l-front960-sides640-2plus1')
        self.assertEqual(next_step(False,False,batch_wait_cause=True),'l-704-2plus1')

    def test_filter_does_not_mutate_original_gt_or_hide_unsupported_objects(self):
        r=[dict(annotations=[dict(training_class='box',xyxy=[0,0,20,20]),dict(training_class='person')])]
        before=copy.deepcopy(r);out=seven_rows(r)
        self.assertEqual(r,before);self.assertEqual(out[0]['annotations'][0]['disposition'],'excluded')
        self.assertEqual(out[0]['annotations'][1]['training_class'],'person')

    def test_saved_evaluation_rejects_stale_split_missing_or_wrong_rgb(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)
            for name in ('manifest.jsonl','temporal-probe.jsonl'):(p/name).write_text('{}\n')
            info=dict(dataset_manifest_sha256=sha(p/'manifest.jsonl'),
                      temporal_manifest_sha256=sha(p/'temporal-probe.jsonl'))
            (p/'inference.json').write_text(json.dumps(info))
            rows=[dict(id='a',image_sha256='rgb')];pred=[dict(id='a',image_sha256='rgb')]
            validate_saved_evaluation(p,p,rows,pred)
            for bad in ([],pred+pred,[dict(id='a',image_sha256='another')]):
                with self.assertRaises(ValueError):validate_saved_evaluation(p,p,rows,bad)
            (p/'manifest.jsonl').write_text('{"split":"changed"}\n')
            with self.assertRaises(ValueError):validate_saved_evaluation(p,p,rows,pred)

    def test_engine_runtime_mapping_shape_and_batch_metadata(self):
        m=dict(engine_sha256='sha',fp16_enabled=True,tensorrt='10.13.0.35',class_ids={str(k):v for k,v in COCO7_IDS.items()},
            output_names=['dets','labels'],input_name='input',shape=[704,704],
            batch_profile=dict(minimum=1,optimum=3,maximum=3))
        validate_metadata(m,'sha','10.13.0.35',{'front':[704,704]})
        for key,value in [('engine_sha256','wrong'),('tensorrt','11'),('class_ids',{'0':'person'}),
                          ('output_names',['labels','dets']),('shape',[640,640]),('batch_profile',dict(minimum=3,optimum=3,maximum=3))]:
            with self.assertRaises(ValueError):validate_metadata(dict(m,**{key:value}),'sha','10.13.0.35',{'front':[704,704]})
        with self.assertRaises(ValueError):validate_metadata(dict(m,fp16_enabled=False),'sha','10.13.0.35',{'front':[704,704]})
        validate_metadata(dict(m,fp16_enabled=False),'sha','10.13.0.35',{'front':[704,704]},require_fp16=False)

    def test_parity_class_error_and_duplicate_are_not_matches(self):
        d=Detection(0,0,100,100,'person',.8)
        self.assertTrue(compare([d],[d],.4)['passed'])
        self.assertFalse(compare([d],[Detection(0,0,100,100,'car',.8)],.4)['passed'])
        self.assertFalse(compare([d],[d,d],.4)['passed'])
        boundary=Detection(0,0,100,100,'person',.405)
        self.assertTrue(compare([boundary],[],.4)['passed'])
        self.assertTrue(compare([boundary],[],.4)['unmatched'][0]['threshold_boundary'])

    def test_parity_duplicate_query_pairing_uses_both_fixed_tolerances(self):
        a=Detection(0,0,100,100,'person',.8);b=Detection(.2,0,100.2,100,'person',.5)
        c=Detection(.3,0,100.3,100,'person',.8);d=Detection(0,0,100,100,'person',.5)
        self.assertTrue(compare([a,b],[c,d],.4)['passed'])


if __name__=='__main__':unittest.main()
