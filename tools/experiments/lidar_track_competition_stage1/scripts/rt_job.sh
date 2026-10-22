#!/bin/bash
# One real-rate run of the real Main node inside the ns_batch namespace (private roscore already up).
# usage: rt_job.sh OUT_DIR FRAMES_COMP_CSV RATE SECONDS TRACE(0|1) [rosparam key=value ...]
set -u
O=$1; FR=$2; RATE=$3; SEC=$4; TRACE=$5; shift 5
RR=/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204
mkdir -p $O
for kv in "$@"; do rosparam set "/Main/${kv%%=*}" "${kv#*=}"; done
if [ "$TRACE" = 1 ]; then
  rosparam set /Main/lidar_track_trace_path $O/track_trace.jsonl
  rosparam set /Main/lidar_track_trace_run_id rt
fi
rosparam get /Main > $O/preset_params.yaml 2>&1
setsid roslaunch Main main_integration.launch planner_udp_enabled:=false > $O/roslaunch.log 2>&1 &
LPID=$!
for i in $(seq 1 200); do rosnode list 2>/dev/null | grep -q '^/Main$' && break; sleep 0.1; done
sleep 2
MPID=$(pgrep -f 'lib/Main/main' | head -1)
echo "launch_pid=$LPID main_pid=$MPID" > $O/pids.txt
cat /proc/$MPID/status | egrep 'Threads|VmRSS|Cpus_allowed_list' >> $O/pids.txt
python3 -B $RR/scripts/rt_observer.py $O/observer.csv > $O/observer.log 2>&1 &
OPID=$!
( echo "t_mono_ns,utime,stime,rss_kb,threads,load1,psi_some_avg10,run_delay_ns_sum"
  while kill -0 $MPID 2>/dev/null; do
    S=$(cat /proc/$MPID/stat 2>/dev/null) || break
    UT=$(echo "$S" | awk '{print $14}'); STT=$(echo "$S" | awk '{print $15}')
    RSS=$(awk '/VmRSS/{print $2}' /proc/$MPID/status 2>/dev/null); TH=$(awk '/Threads/{print $2}' /proc/$MPID/status 2>/dev/null)
    L1=$(cut -d' ' -f1 /proc/loadavg); PSI=$(awk '/some/{print $2}' /proc/pressure/cpu 2>/dev/null | cut -d= -f2)
    RD=$(cat /proc/$MPID/task/*/schedstat 2>/dev/null | awk '{s+=$2} END{print s+0}')
    echo "$(date +%s%N),$UT,$STT,$RSS,$TH,$L1,$PSI,$RD"
    sleep 0.5
  done ) > $O/proc_samples.csv 2>/dev/null &
SPID=$!
sleep 1
python3 -B $RR/scripts/rt_feeder.py $FR $O/feeder.csv --rate $RATE --seconds $SEC > $O/feeder.log 2>&1
echo "feeder rc=$?" >> $O/pids.txt
sleep 2
kill -INT $OPID 2>/dev/null
kill -INT $LPID 2>/dev/null
for i in $(seq 1 100); do kill -0 $LPID 2>/dev/null || break; sleep 0.1; done
kill -TERM $LPID 2>/dev/null; sleep 1
kill $SPID 2>/dev/null
wait $OPID 2>/dev/null
grep -E 'LIDAR_FRAME_TOTAL|LIDAR_V2_TOTAL|tracking experiment' $O/roslaunch.log > $O/node_totals.txt
echo RT_JOB_DONE >> $O/pids.txt
