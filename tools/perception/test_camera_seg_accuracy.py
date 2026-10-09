import unittest
from analyze_camera_seg_accuracy import analyze


class AccuracyTest(unittest.TestCase):
    def test_absent_actor_and_quarantined_frame_do_not_invent_continuity(self):
        ann=dict(actor_id='1',group='vehicle',xyxy=[0,0,10,10],source_xyxy=[0,0,10,10],visible_pixels=100)
        gt={};rows=[]
        for frame,detected in ((1,True),(2,False),(4,True),(6,False)):
            name=str(frame).zfill(6);key=('run','front',name)
            gt[key]=dict(annotations=[ann],width=100,height=100)
            rows.append(dict(run='run',view='front',frame=name,predictions=[dict(group='vehicle',source_class_name='car',confidence=.9,xyxy=[0,0,10,10])] if detected else []))
        result=analyze(gt,rows,.7)
        self.assertEqual({'vehicle':1},result['matched_to_missed_transitions'])
        self.assertEqual({'vehicle':1},result['visible_consecutive_intervals'])
        self.assertEqual({'vehicle':2},result['actor_reappearances'])


if __name__=='__main__':unittest.main()
