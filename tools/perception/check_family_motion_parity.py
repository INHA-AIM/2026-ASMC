#!/usr/bin/env python3
"""Check offline M0 against the production ROS family path on cached grids."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import yaml

from review_family_dropout import isolated_master, sha
from review_family_motion import Fitter, MetricEvidence, family_dict


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('fixture','grids','config','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    with isolated_master(11365,args.output):
        import rospy
        from std_msgs.msg import Header
        from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode,_ViewFrame
        rospy.init_node('motion_m0_parity',anonymous=True,disable_signals=True)
        for name in ('cameras_v3.yaml','bev/lane_fitting.yaml','camera/lane_input.yaml'):
            for k,v in yaml.safe_load((args.config/name).read_text()).items():rospy.set_param('~'+k,v)
        rospy.set_param('~managed',False);rospy.set_param('~enable_boundary_family_candidate',True)
        rospy.set_param('~enable_independent_boundary_pipeline',False)
        node=LaneVectorizerNode();node.demand_inputs.close();node.diagnostics_timer.shutdown()
        fitter=Fitter(yaml.safe_load((args.config/'bev/lane_fitting.yaml').read_text()))
        indices={};rows=[]
        try:
            for frame in map(json.loads,args.fixture.read_text().splitlines()):
                run=frame['run_id'];index=indices.get(run,0);indices[run]=index+1
                path=args.grids/(run+'-%03d.npz'%index);arrays=dict(np.load(path));frames={};e=[]
                for view in ('front','left','right'):
                    stamp=frame['views'][view]['source_ns']
                    header=Header(stamp=rospy.Time(stamp//10**9,stamp%10**9),frame_id=frame['views'][view]['frame_id'])
                    frames[view]=_ViewFrame(view,stamp,header,arrays[view+'_class'],arrays[view+'_confidence'],0.,node.epoch_gate.context())
                    e.append(node.projectors[view].extract(arrays[view+'_class'],arrays[view+'_confidence'],node.confidence_by_class))
                _,_,_,_,production,audit=node._compute_family_frame(frames,{},dict(projection={},association={},prefit={},shared_family={}))
                offline,off_audit,_=fitter.fit(MetricEvidence.concatenate(e))
                a,b=family_dict(production,audit),family_dict(offline,off_audit)
                if a['valid']!=b['valid']:raise RuntimeError('validity mismatch')
                maximum=0.
                for left,right in zip(a['boundaries'],b['boundaries']):
                    if (left['role'],left['class_id'],left['support'])!=(right['role'],right['class_id'],right['support']):
                        raise RuntimeError('role/color/support mismatch')
                    maximum=max(maximum,float(np.max(np.abs(np.array(left['xy'])-right['xy']))))
                if maximum>1e-6:raise RuntimeError('geometry mismatch')
                rows.append(dict(run_id=run,index=index,grid_sha256=sha(path),max_delta_m=maximum,valid=a['valid']))
        finally:node._shutdown();rospy.signal_shutdown('parity complete')
    report=dict(frames=len(rows),maximum_delta_m=max(r['max_delta_m'] for r in rows),
                valid=sum(r['valid'] for r in rows),fixture_sha256=sha(args.fixture),pass_parity=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
