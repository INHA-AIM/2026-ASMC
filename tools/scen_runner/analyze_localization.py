#!/usr/bin/env python3
"""Independent native-pose evaluation of an empty-scene position trial.

Native actor poses are evaluation only. Reuses scen_runner's route geometry;
never publishes a sensor, pose correction or control message.
"""
import argparse
import bisect
from collections import Counter
import json
import math
from pathlib import Path
import numpy as np
from scen_runner.config import load_config,repo_path
from scen_runner.planning.motion_curve import Curve
from scen_runner.morai.signal_trial_coordinates import Coordinates


def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []


def stats(values):
    values=sorted(values)
    return {name:values[min(len(values)-1,round((len(values)-1)*q))] if values else None
            for name,q in (("p50",.5),("p95",.95),("max",1.))}


def analyze(root):
    result=json.loads((root/"trial-result.json").read_text())
    events=[x for x in rows(root/"events.jsonl") if x.get("event")=="native_sample"]
    decisions=rows(root/"decisions.jsonl");control=rows(root/"control.jsonl")
    route=Curve(np.loadtxt(repo_path(load_config()["map"]["global_path"]))[:,:3])
    coordinates=Coordinates([302595.,4124145.])
    lateral=[];heading=[];stations=[]
    for row in events:
        ego=row["ego"];p=ego["pose"];s,error=route.project(p["x"],p["y"])
        x,y,z,yaw=route.pose(s);stations.append(s)
        if ego["speed_kmh"]>.5:
            lateral.append(abs(-math.sin(yaw)*(p["x"]-x)+math.cos(yaw)*(p["y"]-y)))
            heading.append(abs(math.degrees(math.remainder(coordinates.grid_yaw(p["x"],p["y"],p["yaw"])-yaw,2*math.pi))))
    moving=[x for x in decisions if x.get("speed_kph",0)>.5]
    valid=[x for x in moving if x.get("valid")]
    commands=[]
    try:
        import rosbag
        for path in sorted(root.glob("*.bag")):
            with rosbag.Bag(str(path)) as bag:
                commands += [t.to_sec() for _,_,t in bag.read_messages(topics=["/ctrl_cmd"])]
    except ImportError:
        pass
    all_gaps=[b-a for a,b in zip(commands,commands[1:])]
    active_commands=[t for t in commands if events and events[0]["ros_time"]<=t<=events[-1]["ros_time"]]
    gaps=[b-a for a,b in zip(active_commands,active_commands[1:])]
    bag_gaps=gaps
    receipts=[x["monotonic"] for x in rows(root/"command-receipts.jsonl")
              if events and events[0]["monotonic"]<=x["monotonic"]<=events[-1]["monotonic"]]
    if receipts:gaps=[b-a for a,b in zip(receipts,receipts[1:])]
    fixed_lateral=[];fixed_heading=[];pose_error=[]
    event_times=[x["ros_time"] for x in events]
    for decision in valid:
        if "guide_map_x" not in decision:continue
        t=decision["stamp"];j=bisect.bisect_left(event_times,t)
        if not 0<j<len(events) or event_times[j]-event_times[j-1]>.3:continue
        a,b=events[j-1]["ego"],events[j]["ego"]
        if a["speed_kmh"]<=.5:continue
        q=(t-event_times[j-1])/(event_times[j]-event_times[j-1])
        x=a["pose"]["x"]+q*(b["pose"]["x"]-a["pose"]["x"])
        y=a["pose"]["y"]+q*(b["pose"]["y"]-a["pose"]["y"])
        yaw=a["pose"]["yaw"]+q*math.remainder(b["pose"]["yaw"]-a["pose"]["yaw"],360)
        h=decision["guide_map_yaw"]
        fixed_lateral.append(abs(-math.sin(h)*(x-decision["guide_map_x"])+math.cos(h)*(y-decision["guide_map_y"])))
        fixed_heading.append(abs(math.degrees(math.remainder(coordinates.grid_yaw(x,y,yaw)-h,2*math.pi))))
        pose_error.append(math.hypot(decision["map_x"]-x,decision["map_y"]-y))
    source=Counter(x.get("source") for x in decisions)
    lat=stats(lateral);yaw=stats(heading)
    summary=dict(run_id=root.name,result=result,native_moving_samples=len(lateral),
        native_route_progress_m=stations[-1]-stations[0] if len(stations)>1 else 0,
        native_path_lateral_m=lat,native_path_heading_deg=yaw,
        native_fixed_reference_lateral_m=stats(fixed_lateral),native_fixed_reference_heading_deg=stats(fixed_heading),
        estimated_position_error_m=stats(pose_error),
        guide_lateral_m=stats([abs(x["guide_lateral_m"]) for x in valid]),
        position_processing_ms=stats([x["processing_ms"] for x in valid]),
        mpc_processing_ms=stats([x["processing_ms"] for x in control if x.get("valid")]),
        valid_moving_decisions=len(valid),moving_decisions=len(moving),
        reasons=Counter(x.get("reason") for x in decisions),pose_sources=source,
        command_count=len(commands),command_gap_s=stats(gaps),
        bag_command_gap_s=stats(bag_gaps),independent_command_receipts=len(receipts),
        command_gap_including_setup_cleanup_s=stats(all_gaps),
        native_contact_count=len(result["native_contacts"]),
        duplicate_compute_counters={k:max((x.get(k,0) for x in decisions),default=0)
            for k in ("integration_calls","inference_calls","lattice_calls","speed_profiles")},
        evaluation_basis="native gRPC actor pose and ENU->grid heading against fixed sampled native guide (raw route also reported); no fit/snap; receive clocks; GT not input")
    if fixed_lateral:lat=stats(fixed_lateral);yaw=stats(fixed_heading)
    enough=len(lateral)>=20
    summary["gates"]={
        "measured_motion":enough,
        "distance":result["reason"]=="distance_limit",
        "no_contact":not result["native_contacts"],
        "restored":not result["cleanup_errors"],
        "lateral_p95_le_25cm":bool(enough and lat["p95"]<=.25),
        "lateral_max_le_50cm":bool(enough and lat["max"]<=.5),
        "heading_p95_le_3deg":bool(enough and yaw["p95"]<=3),
        "moving_decisions_95percent_valid":bool(moving and len(valid)/len(moving)>=.95),
        "command_gap_le_250ms":bool(gaps and max(gaps)<=.25),
        "GPS_blackout_exercised":source["dead_reckoning"]>20,
        "position_p95_le_50ms":bool(valid and summary["position_processing_ms"]["p95"]<=50),
    }
    summary["stage_passed"]=all(summary["gates"].values())
    with (root/"position-assessment.json").open("x") as stream:
        stream.write(json.dumps(summary,indent=2)+"\n")
    return summary


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("root",type=Path)
    print(json.dumps(analyze(parser.parse_args().root),indent=2))
