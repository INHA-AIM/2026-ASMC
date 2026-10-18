#!/bin/bash
# B0 SMOKE replay of the READ-ONLY tunnel bag through Main/main_integration.launch (G4). No ground truth:
# smoke verdicts only, informational, NOT real-data validation.
# Bag: /home/autonav/2026AIM/data/tunnel_yolo_fusion.bag (root:root 0644; sha256 a8225512... precheck/bag_sha256_start.txt).
# Inputs played: /gps /imu /velodyne_points /Ego_topic /YOLO/BBox /tf_static and the camera image (remapped
# away as in the previous audit). Production filter defaults (drivable/known_static/static_structure on),
# enable_sensor_fusion=true (bag carries /YOLO/BBox), planner_udp_enabled:=false. strace -f, snmp/snmp6.
source "$(dirname "$(readlink -f "$0")")/common_b0.sh"
NAME=b0_normal_BAG_SMOKE
D=$RT/$NAME; rm -rf "$D"; mkdir -p "$D"
WRAP=$SK/b0_test_wrapper.launch
{ sha256sum "$BAG"; stat -c '%n size=%s mode=%A owner=%U:%G mtime=%y' "$BAG"; } > "$D/bag_sha256_before.txt"
[ "$(head -1 "$D/bag_sha256_before.txt" | cut -d' ' -f1)" = "$BAG_SHA_EXPECTED" ] || { echo "BAG SHA MISMATCH BEFORE -> abort"; exit 3; }
DUR=30
BAG_PLAY_RATE=${BAG_PLAY_RATE:-1.0}
BAG_STRACE=${BAG_STRACE:-1}
BAG_FUSION_TRACE=${BAG_FUSION_TRACE:-1}
FUSION_TRACE_ARG=""
[ "$BAG_FUSION_TRACE" = 1 ] && FUSION_TRACE_ARG='fusion_trace_path:=$D/fusion.jsonl'
LARGS="planner_udp_enabled:=false enable_sensor_fusion:=true drivable_filter_enabled:=true known_static_filter_enabled:=true static_structure_exclusion_enabled:=true"
ONLY=local_v2,map_v2,local_v1,map_v1,diag,last_lidar,tracked_map,motion_debug,last_markers,map_markers,tf,object_boxes,ctrl_discard,ctrl_public
cat > "$RT/gen/$NAME.cmd.sh" <<EOF
#!/bin/bash
D='$D'
echo "profile=bag_smoke launch=$WRAP args=[$LARGS]" > "\$D/run_args.txt"
setsid roslaunch '$WRAP' $LARGS $FUSION_TRACE_ARG > "\$D/launch.log" 2>&1 &
LAUNCH=\$!
for i in \$(seq 1 300); do rosnode list 2>/dev/null | grep -qx /Main && break; sleep 0.2; done
rosparam get /Main > "\$D/main_params.yaml" 2>&1
python3 -s '$SK/b0_recorder.py' --out "\$D" --record-inputs --only $ONLY > "\$D/recorder.log" 2>&1 &
REC=\$!
for i in \$(seq 1 300); do [ -s "\$D/recorder_ready.json" ] && break; sleep 0.2; done
rosbag play --quiet --delay=1 --rate='$BAG_PLAY_RATE' '$BAG' __name:=b0_play /camera/image/compressed:=/image_jpeg/compressed \
   --topics /gps /imu /velodyne_points /Ego_topic /YOLO/BBox /tf_static /camera/image/compressed > "\$D/play.log" 2>&1
echo "play_rc=\$?" > "\$D/play_rc.txt"
sleep 3
MAINPID=\$(pgrep -x main | head -1); echo "main_alive_after_play=\$([ -n "\$MAINPID" ] && echo yes || echo no) pid=\$MAINPID" > "\$D/main_alive.txt"
rostopic list -v > "\$D/topics_end.txt" 2>&1
kill -INT \$REC; for i in \$(seq 1 50); do kill -0 \$REC 2>/dev/null || break; sleep 0.1; done
kill -INT \$LAUNCH; for i in \$(seq 1 200); do kill -0 \$LAUNCH 2>/dev/null || break; sleep 0.1; done
kill -0 \$LAUNCH 2>/dev/null && { kill -TERM -- -\$LAUNCH; sleep 3; kill -KILL -- -\$LAUNCH 2>/dev/null; }
wait \$LAUNCH; echo "launch_rc=\$?" >> "\$D/stop.log"
grep -n -a -E 'Planner object|LIDAR_FRAME_TOTAL|LIDAR_V2_TOTAL|terminate called|lock_error|process has died|Common geometry' "\$D/launch.log" | cut -c1-240 > "\$D/launch_key_lines.txt"
exit 0
EOF
bash "$SK/ns_exec_b0.sh" "$NAME" "$NESTED_DEVEL/setup.bash" "$D" $((DUR + 180)) "$BAG_STRACE" "bash $RT/gen/$NAME.cmd.sh" > "$LB/$NAME.driver.log" 2>&1
echo "ns rc=$?" >> "$LB/$NAME.driver.log"
{ sha256sum "$BAG"; stat -c '%n size=%s mode=%A owner=%U:%G mtime=%y' "$BAG"; } > "$D/bag_sha256_after.txt"
if cmp -s <(head -1 "$D/bag_sha256_before.txt") <(head -1 "$D/bag_sha256_after.txt"); then echo "bag sha256 unchanged" | tee -a "$LB/$NAME.driver.log"
else echo "BAG SHA256 CHANGED" | tee -a "$LB/$NAME.driver.log"; fi
if [ -f "$D/fusion.jsonl" ]; then
  { sha256sum "$D/fusion.jsonl"; wc -lc < "$D/fusion.jsonl"; } > "$D/fusion_trace_meta.txt"
  grep -a -E '"event":"(lidar_input|lidar_process_start|scan_input_selection|lidar_process_end|class_track_birth)"' \
    "$D/fusion.jsonl" > "$D/fusion_min.jsonl"; rm -f "$D/fusion.jsonl"
fi
python3 -I "$SK/b0_truth_table.py" --run-dir "$D" --catalogue "$CAT" --scenario BAG_SMOKE --variant bag \
  --profile bag --feeder-file inputs.jsonl > "$D/truth_table.json" 2> "$D/truth_table.err"
python3 -I -c "import json;t=json.load(open('$D/truth_table.json'));print('bag smoke verdict',t['verdict'],'violations',{k:len(v) for k,v in t['violations'].items()},'stats',t['stats'])" | tee -a "$LB/$NAME.driver.log"
