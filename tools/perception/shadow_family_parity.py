"""Paired family diagnostics through the production vectorizer on pinned RGB.

Uses an isolated ROS master, never a public driving graph. Geometry deltas are
development diagnostics; this does not measure live readiness or create GT.
"""
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
from urllib.parse import urlparse
import numpy as np
import time
import importlib.util
import sys


def compare_family(reference, candidate):
    a={m.role:m for m in reference.boundaries}; b={m.role:m for m in candidate.boundaries}
    deltas=[]; indexed=[]; support_changes=0
    for role in sorted(set(a)&set(b)):
        x=np.asarray([(p.x,p.y) for p in a[role].polyline])
        y=np.asarray([(p.x,p.y) for p in b[role].polyline])
        if len(x) and len(y):
            if x.shape==y.shape:
                indexed.extend(np.linalg.norm(x-y,axis=1).tolist())
            else:
                indexed.extend(np.min(np.linalg.norm(x[:,None,:]-y[None,:,:],axis=2),axis=1).tolist())
            # Arc-length sample indices can shift when the accepted extent
            # changes. Compare lateral position at common forward x instead.
            curves=[]
            for points in (x,y):
                points=points[np.argsort(points[:,0],kind='stable')]
                _,unique=np.unique(points[:,0],return_index=True)
                curves.append(points[unique])
            lo=max(c[0,0] for c in curves); hi=min(c[-1,0] for c in curves)
            lo=max(lo,getattr(reference,'valid_x_min_m',lo),getattr(candidate,'valid_x_min_m',lo))
            hi=min(hi,getattr(reference,'valid_x_max_m',hi),getattr(candidate,'valid_x_max_m',hi))
            if hi>lo:
                samples=np.linspace(lo,hi,max(len(x),len(y),3))
                deltas.extend(np.abs(np.interp(samples,curves[0][:,0],curves[0][:,1])-
                                     np.interp(samples,curves[1][:,0],curves[1][:,1])).tolist())
        support_changes += int(list(a[role].support_state)!=list(b[role].support_state))
    return dict(validity_equal=reference.valid==candidate.valid,
        reference_valid=bool(reference.valid),candidate_valid=bool(candidate.valid),
        roles_equal=set(a)==set(b),colors_equal=all(a[k].color==b[k].color for k in set(a)&set(b)),
        support_changed_roles=support_changes,
        geometry_delta_p95_m=float(np.percentile(deltas,95)) if deltas else None,
        geometry_delta_max_m=max(deltas) if deltas else None,
        sample_index_distance_p95_m=float(np.percentile(indexed,95)) if indexed else None,
        reference_reason=reference.status_reason,candidate_reason=candidate.status_reason)


def run(args, engines, gpu_cache):
    import cv2
    import rosbag
    import rospy
    import yaml
    from contextlib import ExitStack
    from std_msgs.msg import Header
    from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode, _ViewFrame
    from asmc_camera.bev.grid import BevGridSpec
    from asmc_camera.bev.family_view import family_segments
    from asmc_camera.bev.visualization import draw_metric_grid, draw_stateful_lane_family
    from asmc_camera.runtime.tensorrt import sha256_file
    uri=urlparse(os.environ.get('ROS_MASTER_URI','http://localhost:11311'))
    if uri.hostname not in ('localhost','127.0.0.1') or uri.port==11311:
        raise RuntimeError('family parity requires a local isolated ROS master on a non-default port')
    rospy.init_node('shadow_family_parity',anonymous=True,disable_signals=True)
    config=Path(__file__).resolve().parents[2]/'src/asmc_camera/config'
    for name in ('cameras_v3.yaml','bev/lane_fitting.yaml','camera/lane_input.yaml'):
        for key,value in yaml.safe_load((config/name).read_text()).items():
            rospy.set_param('~'+key,value)
    rospy.set_param('~managed',False)
    rospy.set_param('~enable_boundary_family_candidate',True)
    rospy.set_param('~enable_independent_boundary_pipeline',False)
    reference_runtime=getattr(args,'family_reference_runtime',None)
    reference_label=getattr(args,'reference_label','PyTorch')
    candidate_label=getattr(args,'candidate_label','TensorRT')
    reference_hashes={}
    if reference_runtime:
        source=reference_runtime/'src/asmc_camera/src/asmc_camera'
        def load_reference(name,path):
            spec=importlib.util.spec_from_file_location(name,path)
            module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
            reference_hashes[str(path)]=sha256_file(path)
            return module
        fitter=load_reference('asmc_camera.bev._reference_shared_family',source/'bev/shared_lane_family.py')
        reference_node=load_reference('asmc_camera.nodes._reference_lane_vectorizer',source/'nodes/lane_vectorizer_node.py')
        reference_node.fit_shared_lane_family_with_audit=fitter.fit_shared_lane_family_with_audit
        nodes=[reference_node.LaneVectorizerNode(),LaneVectorizerNode()]
    else:
        nodes=[LaneVectorizerNode(),LaneVectorizerNode()]
    class Capture:
        message=None
        def publish(self,message): self.message=message
        def get_num_connections(self): return 0
    for node in nodes:
        node.demand_inputs.close()
        node.diagnostics_timer.shutdown()
        node.boundary_family_publisher=Capture()
    pin=json.loads((args.family_fixture.parent/'summary.json').read_text())
    if sha256_file(args.family_fixture)!=pin['manifest_sha256']:
        raise ValueError('fixture manifest SHA changed')
    rows=[json.loads(line) for line in args.family_fixture.read_text().splitlines()]
    groups=defaultdict(list)
    for row in rows: groups[row['run_id']].append(row)
    grid=BevGridSpec.from_mapping(yaml.safe_load((config/'bev/bev_grid.yaml').read_text()))
    results=[]; gallery=[]
    try:
        for run_id, frames in sorted(groups.items()):
            if args.limit: frames=frames[:args.limit]
            for node in nodes: node.boundary_tracker.reset()
            bag_path=args.data_root/frames[0]['bag']
            expected=next(r['bag_sha256'] for r in pin['runs'] if r['run_id']==run_id)
            if sha256_file(bag_path)!=expected: raise ValueError('pinned bag SHA changed')
            wanted={(v['topic'],v['bag_ns']) for frame in frames for v in frame['views'].values()}
            blobs={}
            with rosbag.Bag(str(bag_path)) as bag:
                for topic,message,stamp in bag.read_messages(topics=sorted({key[0] for key in wanted})):
                    key=(topic,stamp.to_nsec())
                    if key in wanted: blobs[key]=bytes(message.data)
            for index,frame in enumerate(frames):
                images={}
                for view,source in frame['views'].items():
                    blob=blobs[(source['topic'],source['bag_ns'])]
                    if hashlib.sha256(blob).hexdigest()!=source['sha256']: raise ValueError('RGB SHA mismatch')
                    images[view]=cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR)
                    if images[view] is None: raise ValueError('undecodable fixture JPEG')
                predictions=[] if reference_runtime else [engines[0].infer(images,native_outputs={v:set() for v in images})]
                with ExitStack() as stack:
                    inputs={v:stack.enter_context(gpu_cache.lease(v,(run_id,index),image))
                            for v,image in images.items()} if gpu_cache else None
                    predictions.append(engines[-1].infer(images,gpu_inputs=inputs,
                                                       native_outputs={v:set() for v in images}))
                if reference_runtime:
                    predictions=[predictions[0],predictions[0]]
                messages=[];compute_times=[];grid_hashes=[]
                for node,prediction in zip(nodes,predictions):
                    inputs={}
                    for view,value in prediction.views.items():
                        stamp=frame['views'][view]['source_ns']
                        header=Header(stamp=rospy.Time(stamp//1000000000,stamp%1000000000),
                                      frame_id=frame['views'][view]['frame_id'])
                        inputs[view]=_ViewFrame(view,stamp,header,
                            value.model_grid_probabilities['road_marking'].astype(np.uint8),
                            np.rint(np.clip(value.model_grid_categorical_confidences['road_marking'],0,1)*255).astype(np.uint8),
                            0.,node.epoch_gate.context())
                    node.boundary_family_publisher.message=None
                    hashes={v:hashlib.sha256(f.class_id.tobytes()+f.confidence.tobytes()).hexdigest() for v,f in inputs.items()}
                    for value in inputs.values():
                        value.class_id.setflags(write=False);value.confidence.setflags(write=False)
                    begun=time.perf_counter()
                    node._process(inputs)
                    if reference_runtime and (node.errors or node.shared_family_errors):
                        raise RuntimeError('family pipeline error during refactor parity')
                    compute_times.append(1000*(time.perf_counter()-begun))
                    if hashes!={v:hashlib.sha256(f.class_id.tobytes()+f.confidence.tobytes()).hexdigest() for v,f in inputs.items()}:
                        raise RuntimeError('family pipeline modified a model grid')
                    grid_hashes.append(hashes)
                    message=node.boundary_family_publisher.message
                    if message is None: raise RuntimeError('production vectorizer did not publish a family')
                    messages.append(message)
                compared=compare_family(*messages)
                if reference_runtime:
                    if grid_hashes[0]!=grid_hashes[1]:raise RuntimeError('paired model grids differ')
                    delta=0.;shape_equal=True
                    a={m.role:m for m in messages[0].boundaries};b={m.role:m for m in messages[1].boundaries}
                    for role in set(a)&set(b):
                        x=np.asarray([(p.x,p.y,p.z) for p in a[role].polyline]);y=np.asarray([(p.x,p.y,p.z) for p in b[role].polyline])
                        shape_equal &= x.shape==y.shape
                        if x.shape==y.shape and x.size:delta=max(delta,float(np.max(np.abs(x-y))))
                    compared.update(grid_sha256=grid_hashes[0],point_shapes_equal=bool(shape_equal),point_max_abs_delta_m=delta,
                                    compute_ms=dict(reference=compute_times[0],candidate=compute_times[1]))
                results.append(dict(run_id=run_id,index=index,phase=frame['phase'],pose=frame['pose'],**compared))
                if index%60==0 or not compared['validity_equal'] or not compared['roles_equal']:
                    panels=[]
                    for title,message in zip(('Previous family','Optimized family') if reference_runtime else (reference_label,candidate_label),messages):
                        canvas=np.zeros((*grid.shape,3),np.uint8); draw_metric_grid(canvas,grid)
                        draw_stateful_lane_family(canvas,family_segments(message),grid,title+': '+message.status_reason)
                        panels.append(cv2.resize(canvas,(320,500)))
                    native=cv2.resize(images['front'],(640,360))
                    panel=np.vstack((native,np.hstack(panels)))
                    filename='{}-{:04d}.jpg'.format(run_id,index)
                    cv2.imwrite(str(args.output/filename),panel)
                    gallery.append((filename,run_id,frame['phase']))
            print('family parity: {} {} triplets'.format(run_id,len(frames)),flush=True)
    finally:
        for node in nodes: node._shutdown()
    (args.output/'families.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in results))
    summary=dict(schema='shadow_family_parity_v2',triplets=len(results),
        fixture_sha256=sha256_file(args.family_fixture),checkpoint_sha256=sha256_file(args.checkpoint),
        engine_manifest_sha256=sha256_file(args.engine_manifest) if args.engine_manifest else None,
        reference_checkpoint_sha256=engines[0].backend.info.checkpoint_sha256,
        reference_model_config=engines[0].backend.info.model_config,
        candidate_model_config=engines[-1].backend.info.model_config,
        validity_changes=sum(not r['validity_equal'] for r in results),
        role_changes=sum(not r['roles_equal'] for r in results),
        support_changes=sum(r['support_changed_roles']>0 for r in results),
        live_readiness_verified=False,automatic_promotion_allowed=False,
        geometry_definition='Absolute lateral delta at common forward x within both accepted extents; sample-index distance retained separately. Support-state changes are reported independently.',
        note='Production vectorizer, same RGB/pose and quantization; geometry changes require review. Offline parity is not live readiness or GT accuracy.')
    if reference_runtime:
        summary.update(schema='shadow_family_refactor_parity_v1',reference_sources=reference_hashes,
            grid_inputs_equal=True,point_max_abs_delta_m=max((r['point_max_abs_delta_m'] for r in results),default=0),
            shape_changes=sum(not r['point_shapes_equal'] for r in results),
            parity_pass=all(r['validity_equal'] and r['roles_equal'] and r['colors_equal'] and not r['support_changed_roles'] and
                            r['point_shapes_equal'] and r['point_max_abs_delta_m']<=1e-6 for r in results),
            compute_ms={name:{'p'+str(q):float(np.percentile([r['compute_ms'][name] for r in results],q)) for q in (50,95,99)}
                        for name in ('reference','candidate')},
            note='Same TensorRT grids; refactor parity, not PyTorch/TensorRT accuracy. Offline timing includes message construction and is not live acceptance.')
    (args.output/'report.json').write_text(json.dumps(summary,indent=2)+'\n')
    title='TensorRT grids: previous / optimized family' if reference_runtime else 'Shadow family {} / {}'.format(reference_label,candidate_label)
    (args.output/'index.html').write_text('<meta charset="utf-8"><h1>'+title+'</h1>'+''.join(
        '<p>{} {}</p><img loading="lazy" src="{}">'.format(run,phase,file) for file,run,phase in gallery))
    return 0 if (summary['parity_pass'] if reference_runtime else not summary['validity_changes'] and not summary['role_changes']) else 1
