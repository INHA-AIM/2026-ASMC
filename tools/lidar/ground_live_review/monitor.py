#!/usr/bin/env python3
"""Read-only live timing/counter recorder, no control commands."""
import argparse
import json
import time
from pathlib import Path
from collections import Counter, defaultdict
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray


def main():
    p=argparse.ArgumentParser()
    p.add_argument("output",type=Path)
    p.add_argument("--seconds",type=float,default=65)
    a=p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    rows=[]
    rospy.init_node("ground_live_monitor",anonymous=True)
    def callback(msg):
        for item in msg.status:
            if item.name=="asmc_lidar/ground_live":
                rows.append({"wall_monotonic":time.monotonic(),"state":item.message,
                             **{v.key:v.value for v in item.values}})
    subscriber=rospy.Subscriber("/lidar_candidate/ground_live/status",DiagnosticArray,callback,queue_size=200)
    start=time.monotonic()
    while not rospy.is_shutdown() and time.monotonic()-start<a.seconds:
        time.sleep(.1)
    subscriber.unregister()
    elapsed=time.monotonic()-start
    if not rows:
        raise RuntimeError("no diagnostic messages received; no successful measurement")
    valid=[r for r in rows if r["state"] in ("valid","roi_empty","observed_empty")]
    timings=defaultdict(list)
    for r in valid:
        for k,v in r.items():
            if k.endswith("_ms"):
                timings[k].append(float(v))
    delta={key:int(rows[-1][key])-int(rows[0][key]) if len(rows)>1 else 0
           for key in ("received","processed","invalid","queue_dropped","time_resets")}
    span=rows[-1]["wall_monotonic"]-rows[0]["wall_monotonic"] if len(rows)>1 else elapsed
    report={"elapsed_s":elapsed,"observed_span_s":span,"states":dict(Counter(r["state"] for r in rows)),
            "counter_delta":delta,"input_hz":delta["received"]/span,
            "processed_hz":delta["processed"]/span,
            "processing_fraction":delta["processed"]/delta["received"] if delta["received"] else None,
            "timing_ms":{k:{f"p{q}":float(np.percentile(v,q)) for q in (50,95,99)} for k,v in timings.items()},
            "rows":rows,"limits":["Receipt-to-publish excludes simulator acquisition/network and is not beam latency."]}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k!="rows"},indent=2))


if __name__=="__main__":
    main()
