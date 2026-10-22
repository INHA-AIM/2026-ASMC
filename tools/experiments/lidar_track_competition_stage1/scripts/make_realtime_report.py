#!/usr/bin/env python3
"""Build results/REALTIME_REPORT.json from work/realtime/RT_SUMMARY.json (+ fixed method text)."""
import hashlib, json, sys
sys.dont_write_bytecode = True
RR = "/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204"
runs = json.load(open(RR + "/work/realtime/RT_SUMMARY.json"))
by = {r["run"].split("/")[-1]: r for r in runs}


def pick(r):
    keys = ["published_scans", "input_rate_hz", "input_duration_s", "input_period_ms", "feeder_lateness_ms", "outputs_local",
            "outputs_matched_to_input", "outputs_map", "dropped_scans_total", "drop_rate", "drop_runs_max_consecutive", "output_rate_hz",
            "output_interval_ms", "e2e_latency_ms", "trace_scans_processed", "latest_frame_wins_skipped",
            "callbacks_not_reaching_processing_estimate", "lidar_process_ms", "stage_ms", "trace_bytes", "main_cpu_cores_mean",
            "main_rss_kb_max", "main_threads_max", "load1", "cpu_pressure_some_avg10", "main_run_delay_ms_per_s", "node_totals"]
    return {k: r.get(k) for k in keys if k in r}


rep = {
    "schema": "realtime_report_v1", "date": "2026-10-22",
    "verdict_summary": {
        "production_build_O0_15hz": "input 15.00 Hz sustained; 0.4% scans dropped (2/508, max 1 consecutive) on the urban scene scen21, 0 on scen75; end-to-end p95 66-90 ms, p99 69-126 ms, max 73-146 ms; LidarProcess p95 56-67 ms ~= the 66.7 ms period, i.e. NO margin: 15 Hz is met only marginally and only without camera decode, YOLO fusion, GPS/map, MotionClassifier and legacy planner publishing (not exercised here)",
        "optimised_build_O2_15hz": "0 drops in 4 runs; end-to-end p50 12-14 ms, p95 20-24 ms, max 24-33 ms; LidarProcess p95 13 ms (5x margin)",
        "options_A_B": "no measurable throughput cost at -O0 (2 drops, e2e p95 80 ms vs 90 ms off)",
        "trace_on_off": "-O0: e2e p95 122 ms (trace on) vs 90/90 ms (off, 2 repeats), drops 3 vs 2; -O2: 23.4 vs 24.0/21.8 ms. Trace volume ~12.7 MB per 34 s (all tracks).",
        "bottleneck": "-O0 Eigen per-point transforms: ROI p50 13-14 ms (p95 21 ms) and imu_correct ~4 ms; tracking FrameFilter p50 5-8 ms, p95 15-22 ms; L-shape objectinfo p95 ~9 ms. All collapse with -O2 (ROI 0.06 ms, FrameFilter 1.6 ms p50). PCL clustering (~6 ms p95) is unchanged (library already optimised). The production nested build has CMAKE_BUILD_TYPE unset (-O0).",
        "historical_claim_p95_145ms": "came from the offline harness at maximum speed with 2-3 processes on SMT siblings (exp05/06 PERFORMANCE_RESULTS): it is a contention-affected offline processing time, not a 15 Hz latency; not used here"},
    "method": {
        "node": "real Main node (lib/Main/main) via `roslaunch Main main_integration.launch planner_udp_enabled:=false` inside ns_batch.sh (unshare user/net/pid/mount, lo only, private roscore); one fresh namespace per run",
        "inputs": "scripts/rt_feeder.py: /velodyne_points PointCloud2 at 15 Hz paced by absolute CLOCK_MONOTONIC deadlines (lateness recorded), /imu and /Ego_topic (velocity.x only = Competition vel_x channel; main.launch remaps /Competition_topic to /Ego_topic) at 50 Hz, no GPS. Scans = Lidar_Data competition-input scenes (recorded ~9-10 Hz) replayed at 15 Hz; ~10k points/scan (VLP-16 pattern)",
        "consumer": "scripts/rt_observer.py subscribes /lidar/local_object and /lidar/map_object with tcp_nodelay and logs CLOCK_MONOTONIC receive time",
        "clocks": "end-to-end latency = observer receive CLOCK_MONOTONIC - feeder publish CLOCK_MONOTONIC (same host, same clock); header.stamp (ROS wall) is only the join key and is never subtracted from a monotonic time; LidarProcess/stage times = steady_clock inside the node (track trace scan_end)",
        "drop_accounting": "published scans vs V2 outputs joined by stamp; with trace on: scan id = callback generation, so latest-frame-wins skips = generation gaps and callbacks lost before processing = published - max generation",
        "resources": "proc_samples.csv every 0.5 s: utime+stime, VmRSS, threads, /proc/loadavg, /proc/pressure/cpu, sum of task schedstat run_delay",
        "builds": {"O0": "exp_cand, CMAKE_BUILD_TYPE unset = production convention (-std=gnu++17, no -O)", "O2": "exp_cand_o2, Release with -O2 only (no NDEBUG), same source commit 669859c1"},
        "order": "ABAB-like: O0 off, O2 off, O0 trace, O2 trace, O0 A+B, O0 off, O2 off (scen21) then O0 off, O2 off, O0 trace (scen75)",
        "duration_per_run_s": "~34 s of input (508 / 500 scans)"},
    "contamination": "another session (asmc_lidar_migration) was active earlier in the day; during these runs load1 0.4-2.2 and CPU PSI some avg10 0.05-0.34 (recorded per run); Main run_delay <= 0.3 ms/s",
    "not_covered": ["camera JPEG decode and YOLO fusion inside TemporalThread (no camera input)", "GPS/map pose, MotionClassifier, legacy /last_lidar publishing and planner filters (no GPS in the competition-restricted data)",
                    "real MORAI bridge timing and UDP receive path", "real 15 Hz LiDAR (scans are 10 Hz recordings replayed faster)", "long-duration (> 35 s) drift/memory growth"],
    "runs": {n: pick(r) for n, r in by.items()},
    "raw": RR + "/work/realtime (per run: feeder.csv, observer.csv, proc_samples.csv, roslaunch.log, node_totals.txt, track_trace.jsonl for trace runs)",
    "rt_summary_sha256": hashlib.sha256(open(RR + "/work/realtime/RT_SUMMARY.json", "rb").read()).hexdigest(),
}
json.dump(rep, open(RR + "/results/REALTIME_REPORT.json", "w"), indent=1)
print("ok", len(rep["runs"]))
