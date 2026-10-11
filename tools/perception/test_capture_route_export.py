import unittest
import hashlib
from pathlib import Path
import tempfile
import numpy as np
from PIL import Image
from export_capture_route_bbox import convert
from verify_capture_route_dataset import verify_pair, evaluation_hashes


class CaptureRouteExportTest(unittest.TestCase):
    def test_visible_components_and_invisible_raw_box(self):
        instance=np.zeros((20,20,4),np.uint8);semantic=np.zeros((20,20,3),np.uint8)
        instance[3:8,4:9]=[0,0,82,255];semantic[3:8,4:9]=[236,255,2]
        boxes=[dict(actor_id='10001',gt_source_label='Obstacle',rgba=[0,0,82,255],source_xyxy=[1,1,10,10]),
               dict(actor_id='1',gt_source_label='Pedestrian',rgba=[0,41,0,255],source_xyxy=[0,0,10,10])]
        ledger=[dict(native_ids=['1','10001'],placement_approved=True,identity_approved=True,
            target_class='stroller',api_name='BabyStroller1',native_class_overrides={'1':'person','10001':'stroller'},
            native_source_classes={'10001':{'Obstacle':'stroller'}},native_rgba={'10001':{'Obstacle':[0,0,82,255]}})]
        result=convert(instance,semantic,boxes,ledger)
        self.assertEqual(len(result),1)
        self.assertEqual(result[0]['canonical'],'stroller')
        self.assertEqual(result[0]['xyxy'],[4,3,9,8])
        self.assertEqual(result[0]['source_xyxy'],[1,1,10,10])

    def test_missing_visible_object_rejects_frame(self):
        inst=np.zeros((10,10,4),np.uint8);sem=np.zeros((10,10,3),np.uint8)
        sem[2:4,2:4]=[98,2,255]
        with self.assertRaises(ValueError):convert(inst,sem,[],[])

    def test_wrong_or_unreviewed_actor_cannot_be_exported(self):
        inst=np.zeros((5,5,4),np.uint8);sem=np.zeros((5,5,3),np.uint8)
        inst[1:3,1:3]=[1,2,3,255]
        boxes=[dict(actor_id='9',gt_source_label='Obstacle',rgba=[1,2,3,255])]
        with self.assertRaises(ValueError):convert(inst,sem,boxes,[])

    def test_independent_yolo_coordinate_and_sha_check(self):
        with tempfile.TemporaryDirectory() as folder:
            image=Path(folder)/'image.png';label=Path(folder)/'label.txt'
            Image.new('RGB',(100,50),'red').save(image)
            label.write_text('9 0.3 0.4 0.4 0.4\n')
            row=dict(image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
                annotation_sha256=hashlib.sha256(label.read_bytes()).hexdigest(),
                pixel_sha256=hashlib.sha256(Image.open(image).convert('RGB').tobytes()).hexdigest(),
                annotations=[dict(canonical='box',xyxy=[10,10,50,30])])
            verify_pair(row,image,label)
            label.write_text('9 0.3 0.4 0.3 0.4\n')
            row['annotation_sha256']=hashlib.sha256(label.read_bytes()).hexdigest()
            with self.assertRaisesRegex(ValueError,'normalization'):verify_pair(row,image,label)
            row['image_sha256']='wrong'
            with self.assertRaisesRegex(ValueError,'image SHA'):verify_pair(row,image,label)

    def test_eval_hashes_respect_frozen_splits(self):
        pixels,files=evaluation_hashes([
            dict(split='train',pixel_sha256='train',image_sha256='train-file'),
            dict(frozen_split='heldout',pixel_sha256='eval',image_sha256='eval-file')])
        self.assertEqual(pixels,{'eval'})
        self.assertEqual(files,{'eval-file'})


if __name__=='__main__':unittest.main()
