#!/bin/bash
# Dedicated runtime comparison (no other experiment job may run concurrently): for every scene the baseline and
# the candidate harness run AT THE SAME TIME on two dedicated cores (taskset), core assignment alternating per
# scene, identical inputs, identical build flags; one fresh isolated namespace for the whole benchmark.
# Metric: per-scan lidar_process_ms (LidarProcess only) and frame_ms (pose lookup .. V2 build), peak RSS per process.
# usage: perf_bench.sh NAME BASE_VARIANT BASE_PARAMS CAND_VARIANT CAND_PARAMS [candidate harness args]
set -u
NAME=$1; BV=$2; BP=$3; CV=$4; CP=$5; shift 5; CEXTRA="$*"
RR=/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239
T=$RR/tools; IN=$RR/work/inputs
NSB=$RR/repo/src/lidar+sensorfusion/build
BB=$NSB/exp_$BV/devel/lib/Main/lidar_dataset_replay; CB=$NSB/exp_$CV/devel/lib/Main/lidar_dataset_replay
O=$RR/work/perf/$NAME; [ -e "$O" ] && { echo "exists $O"; exit 3; }
mkdir -p "$O/base" "$O/cand"
sha256sum "$BB" "$CB" "$BP" "$CP" > "$O/binary_sha256.txt"
echo "name=$NAME base=$BV cand=$CV cand_extra=[$CEXTRA] start=$(date -Is) nproc=$(nproc)" > "$O/meta.txt"
J=$O/jobs.txt; : > "$J"; n=0
for sc in $(python3 -c "import json;m=json.load(open('$IN/INPUT_MANIFEST.json'));print(' '.join(sorted(m['scenes'],key=lambda s:int(s[4:]))))"); do
  if [ $((n % 2)) = 0 ]; then cb=2; cc=5; else cb=5; cc=2; fi; n=$((n+1))
  # each arm runs under its OWN devel environment (LD_LIBRARY_PATH would otherwise override RUNPATH)
  echo "(source $NSB/exp_$BV/devel/setup.bash; exec taskset -c $cb $BB --frames $IN/$sc/frames.csv --scene $sc --params $BP --set imu_yaw_is_true_enu=false --out $O/base/$sc.jsonl > $O/base/$sc.log 2>&1) & (source $NSB/exp_$CV/devel/setup.bash; exec taskset -c $cc $CB --frames $IN/$sc/frames.csv --scene $sc --params $CP --set imu_yaw_is_true_enu=false $CEXTRA --out $O/cand/$sc.jsonl > $O/cand/$sc.log 2>&1) & wait" >> "$J"
done
bash "$T/ns_batch.sh" "perf_$NAME" "$NSB/exp_$CV/devel/setup.bash" "$J" 1 14400 > "$O/ns_batch.log" 2>&1
echo "ns rc=$? end=$(date -Is)" >> "$O/meta.txt"
cp "$RR/logs/ns/perf_$NAME/udp_snmp.json" "$O/udp_snmp.json"
python3 - "$O" > "$O/PERF_SUMMARY.json" <<'PY'
import json, os, re, sys
import numpy as np
O = sys.argv[1]
res = {}
for arm in ("base", "cand"):
    lp, fm, rss, scans = [], [], [], 0
    for f in sorted(os.listdir(os.path.join(O, arm))):
        p = os.path.join(O, arm, f)
        if f.endswith(".jsonl"):
            for line in open(p):
                d = json.loads(line); scans += 1
                if d["processed"]:
                    lp.append(d["timing"]["lidar_process_ms"]); fm.append(d["timing"]["frame_ms"])
        elif f.endswith(".log"):
            m = re.search(r"peak_rss_kb=(\d+)", open(p).read())
            if m: rss.append(int(m.group(1)))
    q = lambda a, p: float(np.percentile(a, p))
    res[arm] = {"scans": scans, "processed": len(lp), "lidar_process_ms": {"p50": q(lp, 50), "p95": q(lp, 95), "max": float(max(lp)), "mean": float(np.mean(lp))},
                "frame_ms": {"p50": q(fm, 50), "p95": q(fm, 95), "max": float(max(fm))},
                "peak_rss_kb": {"max": max(rss), "median": float(np.median(rss)), "processes": len(rss)}}
b, c = res["base"], res["cand"]
res["lidar_process_p95_rel_increase"] = c["lidar_process_ms"]["p95"] / b["lidar_process_ms"]["p95"] - 1.0
res["frame_p95_rel_increase"] = c["frame_ms"]["p95"] / b["frame_ms"]["p95"] - 1.0
res["peak_rss_max_rel_increase"] = c["peak_rss_kb"]["max"] / b["peak_rss_kb"]["max"] - 1.0
res["udp"] = json.load(open(os.path.join(O, "udp_snmp.json")))
print(json.dumps(res, indent=1))
PY
cat "$O/PERF_SUMMARY.json"
