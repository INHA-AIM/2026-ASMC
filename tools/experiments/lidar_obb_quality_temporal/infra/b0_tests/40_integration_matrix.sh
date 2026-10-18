#!/bin/bash
# B0 ROS integration matrix (G4): one FRESH hardened namespace per scenario run, real Main through
# Main/main_integration.launch (via b0_test_wrapper.launch), synthetic LiDAR/GPS/IMU/CAN feeder with fault
# injection, recorder of every final/legacy/marker/diagnostic/debug topic, strace -f over the whole graph.
# usage: 40_integration_matrix.sh PROFILE [SCENARIO_ID ...]
#   PROFILE integration      : planner_udp_enabled:=false (every B0 measurement run). Default: all non-optional scenarios.
#   PROFILE positive_control : LABELLED POSITIVE CONTROL (decision 7): product UDP default (true) + in-namespace UDP sink.
#   PROFILE production       : KNOWN_GAP/B1 characterization of main.launch (common geometry OFF), planner_udp_enabled:=false.
source "$(dirname "$(readlink -f "$0")")/common_b0.sh"
PROFILE=${1:-integration}; shift || true
SETUP=$NESTED_DEVEL/setup.bash
case $PROFILE in
  integration) WRAP=${B0_WRAP:-$SK/b0_test_wrapper.launch}; PREFIX=${B0_PREFIX:-b0_normal}; REC_EXTRA="";;
  positive_control) WRAP=$SK/b0_test_wrapper.launch; PREFIX=POSITIVE_CONTROL; REC_EXTRA="";;
  production) WRAP=$SK/b0_test_wrapper_production.launch; PREFIX=KNOWN_GAP_production; REC_EXTRA="--optional-v2";;
  target) WRAP=$SK/b0_test_wrapper_target.launch; PREFIX=TARGET; REC_EXTRA="--optional-v2"
          SETUP=$R/target_build/ws/devel/setup.bash;;
  *) echo "profile must be integration|positive_control|production|target"; exit 2;;
esac
if [ $# -gt 0 ]; then IDS="$*"; else
  IDS=$(python3 -I -c "import json;c=json.load(open('$CAT'));print(' '.join(s['id'] for s in c['scenarios'] if not s.get('optional')))"); fi
SUM=$LB/40_matrix_${PROFILE}.tsv; [ -s "$SUM" ] || printf "profile\tscenario\tverdict\tns_rc\tfeeder_rc\twall_s\n" > "$SUM"
for SID in $IDS; do
  LARGS=$(python3 -I - "$CAT" "$SID" "$PROFILE" <<'PY'
import json, sys
c = json.load(open(sys.argv[1])); s = next(x for x in c['scenarios'] if x['id'] == sys.argv[2]); prof = sys.argv[3]
if prof in ('production', 'target'):
    print('')
    raise SystemExit
a = {k: v for k, v in c['defaults']['launch_args'].items() if not k.startswith('_')}
a.update(s.get('launch_args', {}))
if prof == 'positive_control':
    a['planner_udp_enabled'] = None
out = []
for k, v in sorted(a.items()):
    if v is None:
        out.append('use_product_udp_default:=true')     # do not pass planner_udp_enabled: product default
    else:
        out.append('%s:=%s' % (k, v))
print(' '.join(out))
PY
)
  NAME=${PREFIX}_${SID}${B0_RUN_SUFFIX:-}        # B0_RUN_SUFFIX=_repN for repeated runs of one scenario
  D=$RT/$NAME; rm -rf "$D"; mkdir -p "$D"
  case $PROFILE in positive_control) SINK=1;; *) SINK=0;; esac
  CMDF=$RT/gen/$NAME.cmd.sh
  cat > "$CMDF" <<EOF
#!/bin/bash
# inner command: runs inside ns_exec_b0.sh after the isolation mounts and the clean ROS env
D='$D'
echo "profile=$PROFILE launch=$WRAP args=[$LARGS]" > "\$D/run_args.txt"
SINKPID=
if [ "$SINK" = 1 ]; then
  python3 -s '$SK/b0_udp_sink.py' --out "\$D" --port 7510 > "\$D/udp_sink.log" 2>&1 &
  SINKPID=\$!
  sleep 0.5
fi
setsid roslaunch '$WRAP' $LARGS fusion_trace_path:=\$D/fusion.jsonl > "\$D/launch.log" 2>&1 &
LAUNCH=\$!
for i in \$(seq 1 300); do rosnode list 2>/dev/null | grep -qx /Main && break; sleep 0.2; done
rosnode list > "\$D/nodes_start.txt" 2>&1
rosparam get /Main > "\$D/main_params.yaml" 2>&1
rosparam get /use_sim_time >> "\$D/main_params.yaml" 2>&1
python3 -s '$SK/b0_recorder.py' --out "\$D" $REC_EXTRA > "\$D/recorder.log" 2>&1 &
REC=\$!
python3 -s '$SK/b0_feeder.py' --catalogue '$CAT' --scenario '$SID' --out "\$D" > "\$D/feeder.log" 2>&1
echo "feeder_rc=\$?" > "\$D/feeder_rc.txt"
rostopic list -v > "\$D/topics_end.txt" 2>&1
rosnode info /Main > "\$D/main_node_info.txt" 2>&1
MAINPID=\$(pgrep -x main | head -1)
if [ -n "\$MAINPID" ]; then
  inodes=\$(for f in /proc/\$MAINPID/fd/*; do readlink "\$f"; done 2>/dev/null | sed -n 's/^socket:\[\([0-9]*\)\]\$/\1/p' | sort -u)
  udp=0; for n in \$inodes; do if awk -v n=\$n 'NR>1 && \$10==n {f=1} END {exit !f}' /proc/net/udp /proc/net/udp6; then udp=\$((udp+1)); fi; done
  echo "main_pid=\$MAINPID main_bound_udp_sockets=\$udp" > "\$D/main_sockets.txt"
fi
kill -INT \$REC; for i in \$(seq 1 50); do kill -0 \$REC 2>/dev/null || break; sleep 0.1; done
kill -INT \$LAUNCH; for i in \$(seq 1 200); do kill -0 \$LAUNCH 2>/dev/null || break; sleep 0.1; done
if kill -0 \$LAUNCH 2>/dev/null; then echo "launch still alive -> TERM" >> "\$D/stop.log"; kill -TERM -- -\$LAUNCH; sleep 3; kill -KILL -- -\$LAUNCH 2>/dev/null; fi
wait \$LAUNCH; echo "launch_rc=\$?" >> "\$D/stop.log"
if [ -n "\$SINKPID" ]; then touch "\$D/udp_sink.stop"; for i in \$(seq 1 30); do kill -0 \$SINKPID 2>/dev/null || break; sleep 0.1; done; kill \$SINKPID 2>/dev/null; fi
# known pre-existing (B-15): common-geometry SIGINT ends in boost::lock_error abort after the totals
grep -n -a -E 'Planner object|PlannerObjectSender|LIDAR_FRAME_TOTAL|LIDAR_V2_TOTAL|terminate called|lock_error|process has died|Rejected|Common geometry' "\$D/launch.log" | cut -c1-240 > "\$D/launch_key_lines.txt"
exit 0
EOF
  NSCAN=$(python3 -I -c "import json;c=json.load(open('$CAT'));s=next(x for x in c['scenarios'] if x['id']=='$SID');print(sum(p['scans'] for p in s['phases']))")
  TMO=$(( NSCAN / 10 + 150 ))
  T0=$(date +%s)
  bash "$SK/ns_exec_b0.sh" "$NAME" "$SETUP" "$D" "$TMO" 1 "bash $CMDF" > "$LB/$NAME.driver.log" 2>&1
  RC=$?
  T1=$(date +%s)
  # The fusion trace is ~28 MB per 40 scans (ground_points carry clouds). Keep the per-scan decision events and
  # a sha256/line count of the full file, then drop the bulk.
  if [ -f "$D/fusion.jsonl" ]; then
    { sha256sum "$D/fusion.jsonl"; wc -lc < "$D/fusion.jsonl"; } > "$D/fusion_trace_meta.txt"
    grep -a -E '"event":"(lidar_input|lidar_process_start|scan_input_selection|lidar_process_end|class_track_birth)"' \
      "$D/fusion.jsonl" > "$D/fusion_min.jsonl"; rm -f "$D/fusion.jsonl"
  fi
  python3 -I "$SK/b0_truth_table.py" --run-dir "$D" --catalogue "$CAT" --scenario "$SID" --variant "$PROFILE" \
    --profile "$PROFILE" > "$D/truth_table.json" 2> "$D/truth_table.err"
  V=$(python3 -I -c "import json;print(json.load(open('$D/truth_table.json'))['verdict'])" 2>/dev/null || echo ERROR)
  FRC=$(cat "$D/feeder_rc.txt" 2>/dev/null)
  printf "%s\t%s\t%s\tns_rc=%s\t%s\t%s\n" "$PROFILE" "$SID" "$V" "$RC" "$FRC" "$((T1-T0))" | tee -a "$SUM"
done
