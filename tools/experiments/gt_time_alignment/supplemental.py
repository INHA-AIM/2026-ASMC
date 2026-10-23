import argparse,json,math,hashlib,collections,subprocess
from pathlib import Path
import numpy as np,pandas as pd
from evaluate_saved import stats,valid_velocity
def main():
    ap=argparse.ArgumentParser();ap.add_argument("--work",type=Path,required=True);ap.add_argument("--previous",type=Path,required=True);a=ap.parse_args();W=a.work;OLD=a.previous
    tables={v:pd.read_csv(p/"gt.csv") for v,p in [("nearest_v0",OLD/"inputs/actual_lap"),("aligned_v1",W/"gt_aligned_v1/actual_lap")]}
    samples={v:{arm:pd.read_pickle(W/"samples"/v/arm/"actual_lap.pkl") for arm in ["p0","ab"]} for v in tables}
    contract=json.loads((Path(__file__).resolve().parents[3]/"tools/experiments/lidar_obb_quality_temporal/EVAL_CONTRACT.json").read_text())
    def cause(version,idx,tid):
        g=tables[version];g=g[(g.idx==idx)&(g.track_id==tid)]
        if not len(g):return "GT_unavailable"
        g=g.iloc[0]
        if g.kind!="gt":return "reference_unresolved_ignore"
        radius=np.hypot(g.x,g.y)
        if not (g.x>=0 and radius<=35):return "GT_outside_evaluation_mask"
        flags=[]
        for arm in ["p0","ab"]:
            tp=samples[version][arm]["tp"];tp=tp[(tp.idx==idx)&(tp.gt_tid==tid)]
            if not len(tp):flags.append(arm+"_not_matched")
            else:
                x=tp.iloc[0]
                if x.kfc<2:flags.append(arm+"_filter_not_ready")
                if not np.isfinite([x.gvx,x.gvy]).all():flags.append(arm+"_GT_velocity_invalid")
                if not np.isfinite([x.pvx,x.pvy]).all():flags.append(arm+"_prediction_velocity_invalid")
        return ";".join(flags) or "valid"
    changes=pd.read_csv(W/"results/PAIR_MEMBERSHIP_CHANGES.csv")
    for i,x in changes.iterrows():
        changes.loc[i,"old_support"]=cause("nearest_v0",x.idx,x.gt_tid)
        changes.loc[i,"new_support"]=cause("aligned_v1",x.idx,x.gt_tid)
    changes.to_csv(W/"results/PAIR_MEMBERSHIP_DETAILS.csv",index=False)
    geometry={}
    old=tables["nearest_v0"];new=tables["aligned_v1"]
    ka=set(map(tuple,old[old.kind=="gt"][["idx","track_id"]].to_numpy()));kb=set(map(tuple,new[new.kind=="gt"][["idx","track_id"]].to_numpy()))
    rejected=pd.read_csv(W/"results/GT_REJECTIONS.csv");lookup={(int(x.idx),int(x.track_id)):x.reason for x in rejected.itertuples()}
    gr=[]
    for label,keys in [("removed",ka-kb),("added",kb-ka)]:
        for idx,tid in sorted(keys):gr.append({"idx":idx,"track_id":tid,"change":label,"reason":lookup.get((idx,tid),"new_interpolation_support")})
    pd.DataFrame(gr).to_csv(W/"results/GT_GEOMETRY_MEMBERSHIP.csv",index=False)
    geometry={"intersection":len(ka&kb),"removed":len(ka-kb),"added":len(kb-ka),"removed_reasons":dict(collections.Counter(x["reason"] for x in gr if x["change"]=="removed"))}
    (W/"results/GT_GEOMETRY_MEMBERSHIP.json").write_text(json.dumps(geometry,indent=2))
    results=json.loads((W/"results/RESULTS_BEFORE_AFTER.json").read_text());rows=[]
    for v in results:
        for arm in ["p0","ab"]:
            m=results[v][arm]
            row={"version":v,"arm":arm,**{k:m.get(k) for k in ["tp","fn","center_mae_vehicle","long_mae_vehicle","short_mae_vehicle","yaw_mae_vehicle_deg","idsw","fragmentations"]}}
            row.update({"all_velocity_"+k:x for k,x in m["velocity_finite_GT_and_prediction"].items()})
            row.update({"common_velocity_"+k:x for k,x in results[v]["common"][arm].items()});rows.append(row)
    pd.DataFrame(rows).to_csv(W/"results/METRIC_TABLE.csv",index=False)
    fixed={}
    for arm in ["p0","ab"]:
        frames={}
        for v in samples:
            t=samples[v][arm]["tpairs"].copy();t["gt_tid"]=t.seg.str.split("_").str[0].astype(int)
            frames[v]=t.set_index(["idx","gt_tid"])
        keys=frames["nearest_v0"].index.intersection(frames["aligned_v1"].index)
        fixed[arm]={}
        for same in [True,False]:
            both=keys[(frames["nearest_v0"].loc[keys,"same_pid"].to_numpy()==same)&(frames["aligned_v1"].loc[keys,"same_pid"].to_numpy()==same)]
            name="same_ID" if same else "ID_change_boundary"
            fixed[arm][name]={"n":len(both)}
            for v,t in frames.items():
                z=t.loc[both]
                fixed[arm][name][v]={"center_change_m":stats(z.cdelta),"yaw_change_deg":stats(np.degrees(z.ydelta)),"long_change_m":stats(z.dlong_res),"short_change_m":stats(z.dshort_res)}
    (W/"results/JITTER_FIXED_CONTINUITY_PAIRS.json").write_text(json.dumps(fixed,indent=2))
    # Code snapshots establish what header timestamps meant; no bridge modifications.
    repo=Path(__file__).resolve().parents[3];live=Path("/home/autonav/2026AIM/Yangtest_lidar_web_monitor")
    evidence={"files":{},"excerpt_files":[]}
    for root,label in [(repo,"experiment"),(live,"live_workspace")]:
        for name in ["morai_sensor_bridge.py","morai_udp_bridge.py"]:
            source=root/"src/asmc_bridge/scripts"/name;lines=source.read_text().splitlines()
            evidence["files"][str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
            dest=W/"results"/(label+"_"+name+".txt")
            wanted=["def _lidar_publish","def _stamp_from_velodyne","def packet_stamp"]
            selected=set()
            for i,line in enumerate(lines):
                if any(x in line for x in wanted):selected.update(range(i,min(i+65,len(lines))))
            dest.write_text("\n".join(str(i+1)+": "+lines[i] for i in sorted(selected)));evidence["excerpt_files"].append(dest.name)
    (W/"results/TIMESTAMP_CODE_EVIDENCE.json").write_text(json.dumps(evidence,indent=2))
    print(json.dumps({"geometry":geometry,"membership_changes":changes[changes.status!="intersection"].to_dict("records"),"fixed_jitter_counts":{arm:{k:x["n"] for k,x in val.items()} for arm,val in fixed.items()}},indent=2))
if __name__=="__main__":main()
