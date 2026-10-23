"""Rescore immutable predictions. Never launch inference or write its inputs."""
import argparse,importlib.util,json,math,collections,hashlib
from pathlib import Path
import numpy as np,pandas as pd

def stats(values):
    a=np.asarray(values,dtype=float);a=a[np.isfinite(a)]
    return {"n":len(a),"mae":float(np.mean(a)) if len(a) else None,
            "p95":float(np.percentile(a,95)) if len(a) else None,
            "p99":float(np.percentile(a,99)) if len(a) else None}

def valid_velocity(tp):
    return tp[(tp.kfc>=2)&np.isfinite(tp[["pvx","pvy","gvx","gvy"]]).all(axis=1)].copy()

def paired(p0,ab):
    p=valid_velocity(p0);a=valid_velocity(ab)
    cols=["idx","gt_tid","pid","pvx","pvy","gvx","gvy"]
    z=p[cols].merge(a[cols],on=["idx","gt_tid"],suffixes=("_p0","_ab"),validate="one_to_one")
    assert np.allclose(z[["gvx_p0","gvy_p0"]],z[["gvx_ab","gvy_ab"]])
    for name in ["p0","ab"]:
        z["error_"+name]=np.hypot(z["pvx_"+name]-z["gvx_"+name],z["pvy_"+name]-z["gvy_"+name])
    return z

def jitter(sample):
    result={}
    t=sample["tpairs"]
    for cls in ["all","vehicle","pedestrian"]:
        a=t if cls=="all" else t[t.cls==cls]
        result[cls]={}
        for name,mask in [("same_predicted_ID",a.same_pid),("predicted_ID_change_boundary",~a.same_pid)]:
            z=a[mask];result[cls][name]={"n":len(z),"center_residual_change_m":stats(z.cdelta),
             "yaw_residual_change_deg":stats(np.degrees(z.ydelta)),"long_residual_change_m":stats(z.dlong_res),"short_residual_change_m":stats(z.dshort_res)}
    return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--work",type=Path,required=True);ap.add_argument("--previous",type=Path,required=True);a=ap.parse_args()
    W=a.work;OLD=a.previous;repo=Path(__file__).resolve().parents[3]
    lib=repo/"tools/experiments/lidar_obb_quality_temporal/eval_lib.py"
    spec=importlib.util.spec_from_file_location("ev_alignment",lib);ev=importlib.util.module_from_spec(spec);spec.loader.exec_module(ev)
    contract=json.loads((lib.parent/"EVAL_CONTRACT.json").read_text())
    original_segment=ev.segment_gt
    def with_generation(gt,frames,c):
        out=original_segment(gt,frames,c)
        if "gt_generation" in out:
            out["seg"]=out["seg"]+"_g"+out.gt_generation.astype(int).astype(str)
        return out
    ev.segment_gt=with_generation
    allresults={};pairs={};coverage={};perobj=[];loaded={}
    for version,source in [("nearest_v0",OLD/"inputs"),("aligned_v1",W/"gt_aligned_v1")]:
        inp=W/"evaluation_inputs"/version/"actual_lap";inp.mkdir(parents=True,exist_ok=False)
        F=pd.read_csv(source/"actual_lap/frames.csv",dtype={"stamp_ns":"int64"})
        G=pd.read_csv(source/"actual_lap/gt.csv",dtype={"stamp_ns":"int64"})
        origin=int(F.stamp_ns.iloc[0]);F["source_lidar_stamp_ns"]=F.stamp_ns
        F["stamp_ns"]=F.stamp_ns-origin;G["source_lidar_stamp_ns"]=G.stamp_ns;G["stamp_ns"]=G.stamp_ns-origin
        F.to_csv(inp/"frames.csv",index=False);G.to_csv(inp/"gt.csv",index=False)
        weights=np.r_[np.diff(F.stamp_ns.to_numpy(dtype=np.int64)),0]/1e9
        good=G[G.kind=="gt"];vel=good[np.isfinite(good[["vx","vy"]]).all(axis=1)]
        maskgeom=F.idx.isin(good.idx).to_numpy();maskvel=F.idx.isin(vel.idx).to_numpy()
        coverage[version]={"frames":len(F),"geometry_rows":len(good),"velocity_rows":len(vel),
            "ignore_rows":int((G.kind=="ignore").sum()),"frames_with_geometry":int(maskgeom.sum()),
            "frames_with_velocity":int(maskvel.sum()),"duration_s":float(weights.sum()),
            "geometry_time_coverage_s":float(weights[maskgeom].sum()),"velocity_time_coverage_s":float(weights[maskvel].sum()),
            "ego_valid_frames":int(F.ego_gt_valid.sum()) if "ego_gt_valid" in F else len(F),
            "note":"time coverage weights each frame by interval to next LiDAR; last contributes0; geometry coverage is NOT in-mask TP coverage"}
        allresults[version]={};loaded[version]={}
        for arm,runname in [("p0","P0_reader_v2"),("ab","AB_ON_reader_v2")]:
            run=OLD/"runs"/runname
            sample=ev.eval_scene(contract,str(inp.parent),str(run/"out.jsonl"),"actual_lap",str(run/"points.bin"))
            dest=W/"samples"/version/arm;dest.mkdir(parents=True,exist_ok=False)
            pd.to_pickle(sample,dest/"actual_lap.pkl");sample["tp"].to_csv(dest/"tp.csv",index=False)
            metric=ev.compute_metrics([sample],contract)
            valid=valid_velocity(sample["tp"]);errors=np.hypot(valid.pvx-valid.gvx,valid.pvy-valid.gvy)
            metric["velocity_finite_GT_and_prediction"]=stats(errors)
            metric["valid_velocity_exclusions"]={"tp_total":len(sample["tp"]),"not_kfc_ready":int((sample["tp"].kfc<2).sum()),
                "GT_not_finite":int((~np.isfinite(sample["tp"][["gvx","gvy"]]).all(axis=1)).sum()),
                "prediction_not_finite":int((~np.isfinite(sample["tp"][["pvx","pvy"]]).all(axis=1)).sum()),
                "note":"flags overlap"}
            metric["jitter_by_ID_continuity"]=jitter(sample)
            allresults[version][arm]=metric;loaded[version][arm]=sample
            for tid,z in valid.groupby("gt_tid"):
                st=stats(np.hypot(z.pvx-z.gvx,z.pvy-z.gvy));perobj.append({"version":version,"arm":arm,"gt_id":tid,**st})
            print(version,arm,metric["velocity_finite_GT_and_prediction"],flush=True)
        pair=paired(loaded[version]["p0"]["tp"],loaded[version]["ab"]["tp"])
        pairs[version]=pair;pair.to_csv(W/"results"/("PAIRS_"+version+".csv"),index=False)
        allresults[version]["common"]={name:stats(pair["error_"+name]) for name in ["p0","ab"]}
    oldpairs=pairs["nearest_v0"].set_index(["idx","gt_tid"]);newpairs=pairs["aligned_v1"].set_index(["idx","gt_tid"])
    common=oldpairs.index.intersection(newpairs.index);removed=oldpairs.index.difference(newpairs.index);added=newpairs.index.difference(oldpairs.index)
    changes=[]
    rejects=pd.read_csv(W/"results/GT_REJECTIONS.csv")
    rejection_lookup={(int(x.idx),int(x.track_id)):x.reason for x in rejects.itertuples()}
    for kind,keys in [("intersection",common),("excluded_from_old_pairs",removed),("newly_added_pairs",added)]:
        for key in keys:
            row={"idx":key[0],"gt_tid":key[1],"status":kind,"reason":rejection_lookup.get(key,"matching_mask_or_velocity_support_change")}
            for version,pair in pairs.items():
                sel=pair[(pair.idx==key[0])&(pair.gt_tid==key[1])]
                if len(sel):
                    for col in ["error_p0","error_ab","pid_p0","pid_ab"]:row[version+"_"+col]=sel.iloc[0][col]
            changes.append(row)
    pd.DataFrame(changes).to_csv(W/"results/PAIR_MEMBERSHIP_CHANGES.csv",index=False)
    bridge={"old_n":len(oldpairs),"new_n":len(newpairs),"intersection_n":len(common),"removed_n":len(removed),"added_n":len(added)}
    for label,keys in [("intersection",common),("removed",removed),("added",added)]:
        bridge[label]={}
        for version,df in [("nearest_v0",oldpairs),("aligned_v1",newpairs)]:
            selected=df.loc[df.index.intersection(keys)]
            bridge[label][version]={arm:stats(selected["error_"+arm]) for arm in ["p0","ab"]}
    same=common[(oldpairs.loc[common,"pid_p0"].to_numpy()==newpairs.loc[common,"pid_p0"].to_numpy())&(oldpairs.loc[common,"pid_ab"].to_numpy()==newpairs.loc[common,"pid_ab"].to_numpy())]
    bridge["intersection_same_prediction_ID_n"]=len(same)
    bridge["intersection_same_prediction_ID"]={v:{arm:stats(df.loc[same,"error_"+arm]) for arm in ["p0","ab"]} for v,df in [("nearest_v0",oldpairs),("aligned_v1",newpairs)]}
    (W/"results/PAIR_SET_COMPARISON.json").write_text(json.dumps(bridge,indent=2))
    pd.DataFrame(perobj).to_csv(W/"results/PER_OBJECT_ALL_VALID_SPEED.csv",index=False)
    paired_objects=[]
    for v,df in pairs.items():
        for tid,z in df.groupby("gt_tid"):
            paired_objects.append({"version":v,"gt_id":tid,**{arm+"_"+k:value for arm in ["p0","ab"] for k,value in stats(z["error_"+arm]).items()}})
    pd.DataFrame(paired_objects).to_csv(W/"results/PER_OBJECT_COMMON_SPEED.csv",index=False)
    (W/"results/RESULTS_BEFORE_AFTER.json").write_text(json.dumps(allresults,indent=2))
    (W/"results/COVERAGE_BEFORE_AFTER.json").write_text(json.dumps(coverage,indent=2))
    prior=json.loads((OLD/"results/ACTUAL_FULL_LAP_BASELINE.json").read_text())
    identity={}
    for arm,name in [("p0","P0_reader_v2"),("ab","AB_ON_reader_v2")]:
        new=allresults["nearest_v0"][arm];before=prior[name]
        identity[arm]={k:{"old":v,"recomputed":new.get(k)} for k,v in before.items() if k in new and not ((isinstance(v,float) and math.isnan(v) and isinstance(new[k],float) and math.isnan(new[k])) or v==new[k])}
    (W/"results/OLD_EVALUATION_IDENTITY.json").write_text(json.dumps(identity,indent=2))
    print("COMMON",json.dumps({v:allresults[v]["common"] for v in allresults},indent=2))
    print("MEMBERSHIP",json.dumps(bridge,indent=2))
if __name__=="__main__":main()
