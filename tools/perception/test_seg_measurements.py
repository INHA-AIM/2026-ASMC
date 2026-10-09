import unittest
from collections import Counter
import numpy as np
from seg_measurements import GROUPS, match_boxes, frame_metrics, summarize, choose_threshold, make_protocol


def box(group='vehicle',xyxy=(0,0,10,10),score=.9):
    return dict(group=group,xyxy=xyxy,confidence=score,source_class_name='car',subtype_verified=True)


class SegMeasurementsTest(unittest.TestCase):
    def test_one_to_one_and_group_mismatch(self):
        a=box(); b=box('person')
        self.assertEqual([(0,0,1.)],match_boxes([a],[a,a]))
        self.assertEqual([],match_boxes([a],[b]))
        counts,_,_,_=frame_metrics([a],[a,a])
        self.assertEqual((1,1,0),(counts['vehicle']['tp'],counts['vehicle']['fp'],counts['vehicle']['fn']))

    def test_iou_boundary_inclusive_and_tiny_boxes(self):
        self.assertEqual(1,len(match_boxes([box(xyxy=(0,0,20,10))],[box()])))
        self.assertEqual([],match_boxes([box(xyxy=(0,0,21,10))],[box()]))
        self.assertEqual([],match_boxes([box(xyxy=(0,0,0,10))],[box()]))

    def test_mask_correspondence_and_unknown_groups(self):
        mask=np.zeros((20,20),bool);mask[:10,:10]=True
        counts,values,names,pairs=frame_metrics([box()],[box()], [mask],[mask.copy()])
        summary=summarize(counts,values,names)
        self.assertEqual(1.,summary['mask_iou'])
        self.assertEqual(1.,summary['macro_f1'])
        self.assertFalse(summary['groups']['animal']['evaluable'])

    def test_no_gt_never_perfect_or_selected(self):
        c={g:Counter() for g in GROUPS}
        summary=summarize(c,[],Counter())
        self.assertIsNone(summary['macro_f1'])
        self.assertIsNone(choose_threshold([dict(confidence=.7,metrics=summary)]))

    def test_threshold_tie_prefers_high_and_failures_preserved(self):
        rows=[dict(confidence=c,metrics=dict(macro_f1=.5)) for c in (.1,.2,.7)]
        self.assertEqual(.7,choose_threshold(rows)['confidence'])
        jobs=make_protocol(dict(repeats=3),['camera','debug','fusion'])
        self.assertEqual(36,sum(j['phase']=='camera' and not j['debug'] for j in jobs))
        self.assertEqual(6,sum(j['debug'] for j in jobs))
        self.assertEqual(12,sum(j['phase']=='fusion' for j in jobs))
        self.assertEqual(2,len({j['model'] for j in jobs if j['phase']=='camera'}))
        self.assertEqual(3,len({j['profile'] for j in jobs if j['phase']=='camera' and not j['debug']}))


if __name__=='__main__':unittest.main()
