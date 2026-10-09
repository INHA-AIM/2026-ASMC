import copy
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from PIL import Image

from camera_data_audit import (classify, valid_box, polygon_issues, curate, image_facts,
    import_coco, export_dataset, apply_review, review_samples, near_duplicate_candidates, build_gallery, verify_export)
from download_roboflow_sources import latest_version, extract_checked, error_kind
from snapshot_roboflow_projects import boxes_to_coco, search_records, dataset_records

ROOT = Path(__file__).resolve().parents[2]
POLICY = json.loads((ROOT / 'config/datasets/camera_object_labels_v1.json').read_text())


def row(name='car', source='roboflow:test/project', ident='a', split=None):
    canonical, disposition = classify(name, POLICY)
    return dict(id=ident, source=source, split_group=source, frozen_split=split,
        image='/unused/image.png', image_sha256=ident*64, pixel_sha256=ident*64,
        width=100, height=80, issues=[], annotation_complete=False, provenance_verified=False,
        verified_target_classes=[n for names in POLICY['groups'].values() for n in names],
        annotations=[dict(id='1',original_label=name,canonical=canonical,
            disposition=disposition,xyxy=[10,10,30,30],issues=[])])


def accept(item, **changes):
    result=dict(image_sha256=item['image_sha256'],decision='accept',
        all_targets_annotated=True,bbox_policy=POLICY['bbox_policy'])
    result.update(changes)
    return result


class DownloadTests(unittest.TestCase):
    def test_offset_overlap_is_reconciled_and_incomplete_listing_fails(self):
        class Project:
            calls=0
            def search_all(self,**kwargs):
                self.calls+=1
                yield [dict(id='a'),dict(id='a'),dict(id='b' if self.calls>1 else 'a')]
        self.assertEqual({r['id'] for r in dataset_records(Project(),['id'],2)},{'a','b'})
        with self.assertRaisesRegex(ValueError,'incomplete_source_enumeration'):
            dataset_records(Project(),['id'],3)
        with self.assertRaisesRegex(ValueError,'source_membership_count_changed'):
            dataset_records(Project(),['id'],0)

    def test_sdk_search_generator_yields_pages(self):
        class Project:
            def search_all(self,**kwargs):
                assert kwargs['in_dataset'] is True
                yield [dict(id='a'),dict(id='b')]
                yield [dict(id='c')]
        self.assertEqual([r['id'] for r in search_records(Project(),['id'])],['a','b','c'])

    def test_snapshot_native_centre_coordinates_preserved(self):
        native=dict(annotation=dict(width=640,height=480,boxes=[dict(label='5',x='626.00',y='257.00',width='26.00',height='30.00')]))
        before=copy.deepcopy(native);categories={}
        anns,w,h=boxes_to_coco(native,9,categories,18)
        self.assertEqual(anns[0]['bbox'],[613,242,26,30]);self.assertEqual(categories,{'5':0})
        self.assertEqual(native,before);self.assertEqual((w,h),(640,480))

    def test_snapshot_polygon_is_native_not_fake_bbox_mask(self):
        native=dict(annotation=dict(width=100,height=80,boxes=[dict(label='cargo box',x='20',y='20',width='20',height='20',
            points=[dict(x='10',y='10'),dict(x='30',y='10'),dict(x='30',y='30')])]))
        anns,_,_=boxes_to_coco(native,1,{},0)
        self.assertEqual(anns[0]['segmentation'],[[10,10,30,10,30,30]])
        plain=dict(annotation=dict(width=100,height=80,boxes=[dict(label='car',x='20',y='20',width='20',height='20')]))
        self.assertNotIn('segmentation',boxes_to_coco(plain,1,{},0)[0][0])

    def test_latest_numeric_completed_not_lexical(self):
        items=[dict(id='w/p/9',images=10),dict(id='w/p/10',images=10),dict(id='w/p/11',images=10,generating=True)]
        self.assertEqual(latest_version(items)['id'],'w/p/10')

    def test_no_version_not_empty_dataset(self):
        with self.assertRaises(ValueError):latest_version([])
        with self.assertRaises(ValueError):latest_version([dict(id='w/p/1',images=0)])

    def test_exception_secrets_never_echoed(self):
        self.assertEqual(error_kind(RuntimeError('401 https://api.example/?api_key=SECRET')),'authentication')
        self.assertNotIn('SECRET', error_kind(RuntimeError('SECRET')))

    def test_zip_traversal_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);archive=root/'raw.zip'
            with zipfile.ZipFile(archive,'w') as z:z.writestr('../outside.txt','bad')
            with self.assertRaises(ValueError):extract_checked(archive,root/'export')
            self.assertFalse((root/'outside.txt').exists())


class AuditTests(unittest.TestCase):
    def test_rejection_keeps_review_reason_for_gallery(self):
        original=row();decision=accept(original,decision='reject',notes='Visible barrel has no annotation')
        result,verified=apply_review(original,decision,POLICY)
        self.assertFalse(verified)
        self.assertEqual(result['review_record']['notes'],decision['notes'])
        self.assertIn('review_rejected',result['issues'])

    def test_snapshot_inventory_identity_uses_native_image_id(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            data=dict(images=[dict(id=3,original_image_id='native-abc',file_name='x.png',width=100,height=80)],
                      annotations=[],categories=[])
            path=root/'_annotations.coco.json';path.write_text(json.dumps(data))
            before=import_coco(root,'w/p','unversioned-snapshot',POLICY)[0]['id']
            data['images'][0]['id']=92;path.write_text(json.dumps(data))
            self.assertEqual(before,import_coco(root,'w/p','unversioned-snapshot',POLICY)[0]['id'])

    def test_gallery_contains_reviewed_capture_and_valid_string_escapes(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);path=root/'x.png';Image.new('RGB',(100,80)).save(path)
            original=row(source='capture:run',split='selection');original['image']=str(path)
            result,_=curate([original],POLICY,{'a':accept(original)})
            build_gallery(result,{},root/'gallery',POLICY)
            entries=json.loads((root/'gallery/review-data.json').read_text())
            self.assertEqual(entries[0]['id'],'a')
            self.assertEqual(entries[0]['review_record']['decision'],'accept')
            page=(root/'gallery/index.html').read_text()
            self.assertIn("r.source+'\\n'",page)
            self.assertNotIn("r.source+'\n'",page)

    def test_seven_classes_and_new_names(self):
        for name in ('person','dog','bicycle','car','motorcycle','bus','truck'):
            self.assertEqual(classify(name,POLICY),(name,'candidate'))
        self.assertEqual(classify('WoodBox',POLICY),('wooden_box','candidate'))
        self.assertEqual(classify('cargo_box',POLICY),('cargo_box','candidate'))

    def test_deer_gt_not_dog(self):
        self.assertEqual(classify('deer',POLICY),('deer','candidate'))

    def test_mixed_and_composites_not_guessed(self):
        for name in ('Obstacle','Vehicle','scooter','bike','bicycle man'):
            self.assertEqual(classify(name,POLICY)[1],'ambiguous')

    def test_infrastructure_not_obstacle(self):
        for name in ('traffic light','fence','lane separator','fire','tree'):
            self.assertEqual(classify(name,POLICY)[1],'excluded')

    def test_box_and_polygon_boundaries(self):
        self.assertTrue(valid_box([0,0,100,80],100,80))
        for box in ([0,0,0,10],[-1,0,10,10],[0,0,101,10],[0,0,float('nan'),10]):
            self.assertFalse(valid_box(box,100,80))
        self.assertEqual(polygon_issues([[0,0,100,0,100,80]],100,80),[])
        self.assertTrue(polygon_issues([[0,0,101,0,100,80]],100,80))
        self.assertTrue(polygon_issues([[0,0,1]],100,80))
        self.assertTrue(polygon_issues({'size':[80,100],'counts':[10,20]},100,80))

    def test_unreviewed_external_quarantined(self):
        rows,_=curate([row()],POLICY)
        self.assertFalse(rows[0]['eligible'])

    def test_valid_review_preserves_box_and_original(self):
        original=row();before=copy.deepcopy(original)
        rows,_=curate([original],POLICY,{'a':accept(original)})
        self.assertTrue(rows[0]['eligible']);self.assertEqual(original,before)
        self.assertEqual(rows[0]['annotations'][0]['xyxy'],[10,10,30,30])

    def test_explicit_geometry_fix_preserves_original_and_review_proof(self):
        original=row();original['annotations'][0]['xyxy']=[-1,10,30,30]
        original['annotations'][0]['issues']=['invalid_bbox']
        d=accept(original,reviewer='test',notes='visible extent inspected',
                 overrides=[dict(id='1',canonical='car',xyxy=[0,10,30,30])])
        result=curate([original],POLICY,{'a':d})[0][0]
        self.assertTrue(result['eligible'])
        self.assertEqual(result['annotations'][0]['original_xyxy'],[-1,10,30,30])
        self.assertEqual(result['review_record']['notes'],'visible extent inspected')
        d['overrides']=[dict(id='1',canonical='car')]
        self.assertFalse(curate([original],POLICY,{'a':d})[0][0]['eligible'])

    def test_review_must_match_bytes_and_completeness(self):
        original=row()
        with self.assertRaises(ValueError):apply_review(original,accept(original,image_sha256='old'),POLICY)
        rows,_=curate([original],POLICY,{'a':accept(original,all_targets_annotated=False)})
        self.assertFalse(rows[0]['eligible'])

    def test_true_empty_requires_review(self):
        original=row();original['annotations']=[]
        self.assertFalse(curate([original],POLICY)[0][0]['eligible'])
        self.assertTrue(curate([original],POLICY,{'a':accept(original)})[0][0]['eligible'])

    def test_unknown_not_promoted_by_complete_review(self):
        original=row('obstacle')
        self.assertFalse(curate([original],POLICY,{'a':accept(original)})[0][0]['eligible'])
        decision=accept(original,overrides=[dict(id='1',canonical='cargo_box')])
        result=curate([original],POLICY,{'a':decision})[0][0]
        self.assertTrue(result['eligible']);self.assertEqual(result['annotations'][0]['canonical'],'cargo_box')

    def test_failed_sample_blocks_bucket(self):
        a=row(ident='a');b=row(ident='b')
        reviews={'a':accept(a),'b':accept(b,decision='reject')}
        result,_=curate([a,b],POLICY,reviews)
        self.assertTrue(result[0]['eligible']);self.assertFalse(result[1]['eligible'])

    def test_canonical_override_does_not_approve_whole_source(self):
        a=row('obstacle',ident='a');b=row('obstacle',ident='b')
        policy=copy.deepcopy(POLICY);policy['review']['samples_per_source_class']=1
        sampled=next(iter(review_samples([a,b],policy).values()))[0]
        item=a if sampled=='a' else b
        result,_=curate([a,b],policy,{sampled:accept(item,overrides=[dict(id='1',canonical='cargo_box')])})
        self.assertEqual(sum(r['eligible'] for r in result),1)

    def test_capture_complete_known_accepted(self):
        a=row(source='capture:r1',split='train');a.update(provenance_verified=True,annotation_complete=True)
        self.assertTrue(curate([a],POLICY)[0][0]['eligible'])

    def test_capture_old_four_class_completeness_does_not_cover_new_obstacles(self):
        a=row(source='capture:r1',split='train')
        a.update(provenance_verified=True,annotation_complete=True,
                 verified_target_classes=['person','car','truck','bus'])
        result=curate([a],POLICY)[0][0]
        self.assertFalse(result['eligible'])
        self.assertIn('capture_target_scope_unverified',result['reasons'])
        self.assertTrue(curate([a],POLICY,{'a':accept(a)})[0][0]['eligible'])

    def test_external_group_apportionment_has_evaluation_groups(self):
        originals=[row(source='roboflow:'+str(i),ident=str(i)) for i in range(20)]
        result,_=curate(originals,POLICY)
        from collections import Counter
        self.assertEqual(Counter(r['split'] for r in result),dict(train=16,selection=2,heldout=2))
        reverse,_=curate(list(reversed(originals)),POLICY)
        self.assertEqual({r['id']:r['split'] for r in result},{r['id']:r['split'] for r in reverse})

    def test_capture_incomplete_not_negative(self):
        a=row(source='capture:r1',split='train');a.update(provenance_verified=True,annotation_complete=False)
        a['annotations']=[]
        self.assertFalse(curate([a],POLICY)[0][0]['eligible'])

    def test_frozen_roles_never_moved(self):
        rows=[row(source='capture:'+s,ident=s,split=s) for s in ('train','selection','heldout')]
        for r in rows:r.update(provenance_verified=True,annotation_complete=True)
        result,_=curate(rows,POLICY)
        self.assertEqual([r['split'] for r in result],['train','selection','heldout'])

    def test_external_duplicate_cannot_leak_into_training(self):
        c=row(source='capture:r',split='heldout');c.update(provenance_verified=True,annotation_complete=True)
        r=row(source='roboflow:p',ident='b');r['pixel_sha256']=c['pixel_sha256']
        result,_=curate([c,r],POLICY,{'b':accept(r)})
        self.assertEqual(result[1]['split'],'heldout')
        self.assertEqual(sum(r['eligible'] for r in result),1)

    def test_conflicting_duplicates_not_preferred_by_box_count(self):
        a=row();b=row(ident='b');b['pixel_sha256']=a['pixel_sha256'];b['annotations'][0]['xyxy']=[11,10,30,30]
        result,_=curate([a,b],POLICY,{'a':accept(a),'b':accept(b)})
        self.assertFalse(any(r['eligible'] for r in result))

    def test_frozen_overlap_excluded_not_reassigned(self):
        a=row(source='capture:r1',split='train');b=row(source='capture:r2',ident='b',split='heldout')
        b['pixel_sha256']=a['pixel_sha256']
        for r in (a,b):r.update(provenance_verified=True,annotation_complete=True)
        result,_=curate([a,b],POLICY)
        self.assertEqual([r['split'] for r in result],['train','heldout'])
        self.assertFalse(any(r['eligible'] for r in result))

    def test_near_duplicate_is_review_candidate_not_deleted(self):
        a=row(source='capture:r1',split='train');b=row(source='capture:r2',ident='b',split='heldout')
        for r in (a,b):r.update(provenance_verified=True,annotation_complete=True)
        a['phash']='0000000000000000';b['phash']='0000000000000003'
        result,_=curate([a,b],POLICY);pairs=near_duplicate_candidates(result)
        self.assertEqual(len(pairs),1);self.assertFalse(pairs[0]['confirmed'])
        self.assertFalse(any(r['eligible'] for r in result));self.assertEqual(len(result),2)

    def test_unsafe_coco_image_not_read(self):
        a=row();a['issues']=['unsafe_image_path'];a['image']='/etc/passwd'
        self.assertNotIn('image_sha256', {k:v for k,v in image_facts(a).items() if k not in a})

    def test_missing_image_never_exported(self):
        a=row();a['image']='/does/not/exist.png'
        facts=image_facts(a)
        self.assertIn('missing_or_broken_image', facts['issues'])
        self.assertFalse(curate([facts],POLICY,{'a':accept(facts)})[0][0]['eligible'])

    def test_coco_sparse_category_ids_and_polygon_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);Image.new('RGB',(100,80)).save(root/'x.png')
            data=dict(images=[dict(id=9,file_name='x.png',width=100,height=80)],categories=[dict(id=99,name='car')],
                annotations=[dict(id=18,image_id=9,category_id=99,bbox=[10,10,20,20],segmentation=[[10,10,30,10,30,30]])])
            (root/'_annotations.coco.json').write_text(json.dumps(data))
            result=import_coco(root,'w/p',1,POLICY)
            self.assertEqual(result[0]['annotations'][0]['canonical'],'car')
            self.assertEqual(result[0]['annotations'][0]['native_segmentation'],data['annotations'][0]['segmentation'])
            facts=image_facts(result[0]);facts['id']='a'
            curated,_=curate([facts],POLICY,{'a':accept(facts)})
            export_dataset(curated,POLICY,root/'derived')
            label=next((root/'derived/labels').rglob('*.txt')).read_text()
            self.assertTrue(label.startswith('0 0.2000000000 0.2500000000'))
            self.assertEqual(json.loads((root/'derived/summary.json').read_text())['classes'],['car'])
            import yaml
            self.assertEqual(yaml.safe_load((root/'derived/data.yaml').read_text())['names'],['car'])
            self.assertEqual(verify_export(root/'derived')['verified_objects'],1)
            label_path=next((root/'derived/labels').rglob('*.txt'))
            label_path.write_text('0 0.2000000000 0.2500000000 0.1 0.1\n')
            with self.assertRaisesRegex(ValueError,'export_geometry_changed'):
                verify_export(root/'derived')


if __name__=='__main__':unittest.main()
