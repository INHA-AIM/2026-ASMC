import copy
import unittest

from prepare_camera_obstacle_review import class_reviews, select


class ObstacleReviewTests(unittest.TestCase):
    def test_selection_is_stable_and_keeps_evaluation_duplicates_and_accepted_out(self):
        config=dict(seed=42, selection=[dict(source='roboflow:test', labels=['mixed'], limit=2)])
        rows=[dict(id=str(i), source='roboflow:test', split='train', eligible=False,
              reasons=[], annotations=[dict(original_label='mixed')]) for i in range(7)]
        rows[2]['split']='heldout';rows[3]['eligible']=True
        rows[4]['reasons']=['cross_split_similar_image_review']
        before=copy.deepcopy(rows)
        a=select(rows,config);b=select(list(reversed(rows)),config)
        self.assertEqual([r['id'] for r in a],[r['id'] for r in b])
        self.assertEqual(len(a),2)
        self.assertTrue(all(r['id'] in ('0','1','5','6') for r in a))
        self.assertEqual(rows,before)

    def sample(self):
        return dict(id='a',image_sha256='SHA',source='roboflow:test')

    def crop(self, ident='1'):
        return dict(index=0,id='a',image_sha256='SHA',sheet='annotation-crops-000.jpg',
                    annotation=dict(id=ident))

    def test_crop_decisions_do_not_claim_human_or_complete_review(self):
        old={};out=class_reviews([self.sample()],[self.crop()],{'0':'P'},old)['a']
        self.assertFalse(out['all_targets_annotated']);self.assertFalse(out['human_confirmed'])
        self.assertEqual(out['decision'],'pending')
        self.assertEqual(out['overrides'],[dict(id='1',canonical='plastic_barrier')])
        self.assertEqual(old,{})

    def test_question_mark_never_promotes_old_unconfirmed_proposal(self):
        old={'a':dict(image_sha256='SHA',decision='pending',
                     overrides=[dict(id='2',canonical='box')])}
        out=class_reviews([self.sample()],[self.crop('1'),self.crop('2')],{'0':'D ?'},old)['a']
        self.assertEqual(out['overrides'],[dict(id='1',canonical='traffic_drum')])
        self.assertEqual(out['evidence']['previous_review'],old['a'])

    def test_identity_count_unknown_code_and_geometry_reviews_fail(self):
        for code in ('X','P P'):
            with self.assertRaises(ValueError):class_reviews([self.sample()],[self.crop()],{'0':code},{})
        bad=self.crop();bad['image_sha256']='changed'
        with self.assertRaises(ValueError):class_reviews([self.sample()],[bad],{'0':'B'},{})
        old={'a':dict(image_sha256='SHA',overrides=[dict(id='1',xyxy=[1,1,2,2])])}
        with self.assertRaises(ValueError):class_reviews([self.sample()],[self.crop()],{'0':'B'},old)


if __name__ == '__main__':
    unittest.main()
