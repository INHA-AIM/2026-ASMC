import unittest
import hashlib
from pathlib import Path
import tempfile
import numpy as np
from PIL import Image
from export_capture_route_bbox import convert, validate_rendered_rgb
from verify_capture_route_dataset import verify_pair, evaluation_hashes


class CaptureRouteExportTest(unittest.TestCase):
    def test_blank_rgb_is_rejected_even_if_gt_files_exist(self):
        image=Image.new('RGB',(8,8),'black')
        with self.assertRaisesRegex(ValueError,'render failure'):validate_rendered_rgb(image)
        image.putpixel((1,1),(1,2,3))
        with self.assertRaisesRegex(ValueError,'render failure'):validate_rendered_rgb(image)
        image.putdata([(x,y,100) for y in range(8) for x in range(8)])
        validate_rendered_rgb(image)

    def test_native_truck_and_bus_names_do_not_require_vehicle_alias(self):
        inst=np.zeros((20,20,4),np.uint8);sem=np.zeros((20,20,3),np.uint8)
        for kind,label,color in [('Truck','truck',[255,90,90]),('Bus','bus',[255,105,105])]:
            inst[3:15,4:18]=[0,0,42,255];sem[3:15,4:18]=color
            boxes=[dict(actor_id='172',gt_source_label=kind,rgba=[0,0,42,255],source_xyxy=[4,3,18,15])]
            ledger=[dict(native_ids=['172'],placement_approved=True,identity_approved=True,
                target_class=label,api_name='checked-vehicle',native_source_classes={'172':{kind:label}},
                native_rgba={'172':{kind:[0,0,42,255]}})]
            result=convert(inst,sem,boxes,ledger)
            self.assertEqual(result[0]['canonical'],label)
            boxes[0]['gt_source_label']='Vehicle'
            with self.assertRaises(ValueError):convert(inst,sem,boxes,ledger)

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

    def test_only_stroller_infant_is_excluded_with_pixels_still_resolved(self):
        inst=np.zeros((12,12,4),np.uint8);sem=np.zeros((12,12,3),np.uint8)
        boxes=[];typed={};colors={}
        for i,(native,kind,label) in enumerate([('1','Pedestrian','person'),('10001','Obstacle','stroller'),('10002','Pedestrian','person')]):
            rgba=[0,0,i+1,255];inst[2:4,1+i*3:3+i*3]=rgba;sem[2:4,1+i*3:3+i*3]=[98,2,255]
            boxes.append(dict(actor_id=native,gt_source_label=kind,rgba=rgba))
            typed[native]={kind:label};colors[native]={kind:rgba}
        ledger=[dict(native_ids=['1','10001','10002'],placement_approved=True,identity_approved=True,
            target_class='stroller',api_name='BabyStroller1',native_source_classes=typed,native_rgba=colors)]
        excluded=[];result=convert(inst,sem,boxes,ledger,excluded_annotations=excluded)
        self.assertEqual([x['actor_id'] for x in result],['1','10001'])
        self.assertEqual([x['actor_id'] for x in excluded],['10002'])
        self.assertEqual(len(convert(inst,sem,boxes,ledger,exclude_stroller_infants=False)),3)

    def test_strided_rgba_preserves_alpha_identity_and_edge_pixels(self):
        raw=np.zeros((8,12,4),np.uint8);inst=raw[:,::2];sem=np.zeros((8,6,3),np.uint8)
        inst[2:5,3:6]=[1,2,3,255];inst[7,0]=[1,2,3,254]
        boxes=[dict(actor_id='9',gt_source_label='Obstacle',rgba=[1,2,3,255])]
        ledger=[dict(native_ids=['9'],placement_approved=True,identity_approved=True,
            target_class='box',api_name='WoodBox',native_source_classes={'9':{'Obstacle':'box'}},
            native_rgba={'9':{'Obstacle':[1,2,3,255]}})]
        result=convert(inst,sem,boxes,ledger)
        self.assertEqual(result[0]['xyxy'],[3,2,6,5])
        self.assertEqual(result[0]['visible_pixels'],9)

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
