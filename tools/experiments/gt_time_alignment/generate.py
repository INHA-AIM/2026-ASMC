"""Produce evaluation-only GT from immutable extracted source records."""
import json,math,collections,hashlib,argparse
from pathlib import Path
from bisect import bisect_left
import numpy as np,pandas as pd
from alignment import Series,body_transform,angle_delta

def normalize_snapshots(rows):
    groups={}
    for r in rows:groups.setdefault(r["ns"],[]).append(r)
    out=[]
    for ns,group in sorted(groups.items()):
        sig=lambda r:json.dumps(sorted(r["objects"],key=lambda o:(o["id"],o["cls"])),sort_keys=True)
        row=dict(group[0]);row["conflict"]=len({sig(x) for x in group})>1
        if row["conflict"]:
            merged={o["id"]:o for g in group for o in g["objects"]}
            row["objects"]=list(merged.values())
        row["seq"]=len(out);out.append(row)
    return out

def model_offset(obj,table):
    if obj["cls"]==1:return [0.,0.],"legacy_pedestrian_reference_assumption",[]
    dims=obj["dims"]
    candidates=[(n,v) for n,v in table.items() if max(abs(v[k]-x) for k,x in zip(["length","width","height"],dims))<.011]
    offsets={(round(v["center_offset_x"],5),round(v["center_offset_z"],5)) for n,v in candidates}
    return (list(next(iter(offsets))) if len(offsets)==1 else None,
            "dimension_matched_provisional" if len(offsets)==1 else "unresolved_box_reference",
            [n for n,v in candidates])

def records_frame(rows):
    df=pd.DataFrame(rows)
    for key in df.columns:
        if key.endswith("_ns"):
            df[key]=pd.array([r.get(key) for r in rows],dtype="Int64")
    return df

def build(raw,policy,table,correction_deg):
    if any(b["ns"]<a["ns"] for a,b in zip(raw["snapshots"],raw["snapshots"][1:])):
        raise ValueError("Object source stream nonmonotonic; do not silently sort/interpolate")
    eg=[]
    for e in raw["ego"]:
        eg.append(dict(e,heading=math.radians(e["heading_deg"]+correction_deg)))
    ego=Series(eg,policy)
    snaps=normalize_snapshots(raw["snapshots"]);times=[s["ns"] for s in snaps]
    tracks=collections.defaultdict(list);events=[]
    for s in snaps:
        ids=collections.Counter(o["id"] for o in s["objects"])
        for o in s["objects"]:
            offset,reference,candidates=model_offset(o,table)
            state=dict(o,ns=s["ns"],receipt_ns=s["receipt_ns"],seq=s["seq"],
                       heading=math.radians(o["heading_deg"]+correction_deg),offset=offset,
                       reference=reference,model_candidates=candidates,
                       conflict=s["conflict"] or ids[o["id"]]>1)
            tracks[o["id"]].append(state)
    tracks={tid:Series(rows,policy) for tid,rows in tracks.items()}
    for tid,tr in tracks.items():
        for i,reason in enumerate(tr.edges):
            if reason:
                a,b=tr.rows[i:i+2]
                events.append({"id":tid,"reason":reason,"a_ns":a["ns"],"b_ns":b["ns"],
                               "a_dims":a["dims"],"b_dims":b["dims"],"position_jump_m":float(np.linalg.norm(np.array(a["p"])-b["p"]))})
    frames=[];rows=[];rejections=[];coverage=[]
    for f in raw["frames"]:
        t=f["stamp_ns"];e,er=ego.query(t)
        fr=dict(f,ego_yaw=e["heading"] if e else 0.,ego_x=e["p"][0] if e else np.nan,
                ego_y=e["p"][1] if e else np.nan,ego_z=e["p"][2] if e else np.nan,
                ego_gt_valid=e is not None,ego_gt_reason=er or "valid",
                ego_source_a_ns=e["source_a_ns"] if e else None,
                ego_source_b_ns=e["source_b_ns"] if e else None,ego_alpha=e["alpha"] if e else np.nan)
        frames.append(fr)
        k=bisect_left(times,t)
        inds=[k] if k<len(times) and times[k]==t else [i for i in (k-1,k) if 0<=i<len(times)]
        candidates=sorted({o["id"] for i in inds for o in snaps[i]["objects"] if o["cls"] in [0,1]})
        cov={"idx":f["idx"],"stamp_ns":t,"candidate_n":len(candidates),"ego_valid":e is not None,"geometry_gt_n":0,"velocity_gt_n":0,"ignore_n":0}
        for tid in candidates:
            o,reason=tracks[tid].query(t)
            if er:reason="ego_"+er
            if reason:
                rejections.append({"idx":f["idx"],"stamp_ns":t,"track_id":tid,"reason":reason})
                continue
            if o["cls"] not in [0,1]:continue
            loc=body_transform(e,o);resolved=o["offset"] is not None
            row=dict(idx=f["idx"],stamp_ns=t,kind="gt" if resolved else "ignore",
                     track_id=tid,gt_generation=o["generation"],class_id=o["cls"],
                     **loc,l=o["dims"][0],w=o["dims"][1],h=o["dims"][2],
                     gx=o["center"][0],gy=o["center"][1],gz=o["center"][2],
                     reference_x=o["p"][0],reference_y=o["p"][1],reference_z=o["p"][2],
                     yaw_global=o["heading"],reference=o["reference"],reference_resolved=resolved,
                     source_a_ns=o["source_a_ns"],source_b_ns=o["source_b_ns"],alpha=o["alpha"],gt_mode=o["mode"],
                     velocity_a_ns=o.get("velocity_a_ns"),velocity_b_ns=o.get("velocity_b_ns"),
                     velocity_reason=o["velocity_reason"],velocity_kind=policy["velocity"],
                     center_interval_average_vx=o.get("center_interval_average_velocity",[np.nan]*3)[0],
                     center_interval_average_vy=o.get("center_interval_average_velocity",[np.nan]*3)[1],
                     reference_interval_average_vx=o.get("reference_interval_velocity",[np.nan]*3)[0],
                     reference_interval_average_vy=o.get("reference_interval_velocity",[np.nan]*3)[1],
                     omega=o.get("angular_rate",np.nan),model_candidates=json.dumps(o["model_candidates"]))
            if not resolved:row["vx"]=row["vy"]=np.nan;row["velocity_reason"]="unresolved_box_reference"
            rows.append(row)
            if resolved:
                cov["geometry_gt_n"]+=1;cov["velocity_gt_n"]+=int(np.isfinite([row["vx"],row["vy"]]).all())
            else:
                cov["ignore_n"]+=1
                rejections.append({"idx":f["idx"],"stamp_ns":t,"track_id":tid,"reason":"unresolved_box_reference"})
        coverage.append(cov)
    audit={"ego":dict(ego.audit),"objects":{str(k):dict(v.audit) for k,v in tracks.items()},
           "source_boundary_events_n":len(events),"rejected_reasons":dict(collections.Counter(x["reason"] for x in rejections)),
           "n_lidar_frames":len(frames),"ego_valid_frames":sum(f["ego_gt_valid"] for f in frames),
           "gt_geometry_rows":sum(r["kind"]=="gt" for r in rows),
           "gt_velocity_rows":int(sum(r["kind"]=="gt" and np.isfinite([r["vx"],r["vy"]]).all() for r in rows))}
    return tuple(records_frame(v) for v in [frames,rows,rejections,coverage,events])+(audit,)

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--work",type=Path,required=True);ap.add_argument("--reference-table",type=Path,required=True);ap.add_argument("--previous",type=Path,required=True);a=ap.parse_args()
    raw=json.loads((a.work/"source_gt_records.json").read_text());policy=json.loads((a.work/"results/TIME_POLICY.json").read_text())
    table=json.loads(a.reference_table.read_text())["models"]
    correction=json.loads((a.previous/"results/PREPARE.json").read_text())["map_heading_additive_correction_deg"]
    F,G,reject,cov,events,audit=build(raw,policy,table,correction)
    out=a.work/"gt_aligned_v1/actual_lap";out.mkdir(parents=True,exist_ok=False)
    F.to_csv(out/"frames.csv",index=False);G.to_csv(out/"gt.csv",index=False)
    reject.to_csv(a.work/"results/GT_REJECTIONS.csv",index=False);cov.to_csv(a.work/"results/GT_TIME_COVERAGE.csv",index=False);events.to_csv(a.work/"results/GT_BOUNDARIES.csv",index=False)
    manifest={"version":policy["version"],"policy":policy,"heading_correction_deg":correction,"offset_tuned":False,"evaluation_only":True,"files":{},"audit":audit}
    for p in [out/"frames.csv",out/"gt.csv",a.work/"source_gt_records.json",a.reference_table,Path(__file__),Path(__file__).with_name("alignment.py")]:
        manifest["files"][str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    (a.work/"results/GT_VERSION.json").write_text(json.dumps(manifest,indent=2))
    print(json.dumps(audit,indent=2))
if __name__=="__main__":main()
