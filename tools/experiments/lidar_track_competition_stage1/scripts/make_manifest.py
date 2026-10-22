#!/usr/bin/env python3
"""Collect BASELINE_MANIFEST.json from the actual files of this run root (read-only on sources).
usage: python3 -B make_manifest.py OUT.json
"""
import glob, hashlib, json, os, re, subprocess, sys
sys.dont_write_bytecode = True
RR = "/home/autonav/2026AIM/Lidar_experimments/track_competition_stage1_20261022_134204"
E3 = "/home/autonav/2026AIM/Lidar_experimments/obb_init_turn_follow_20261019_032258"
REPO = RR + "/repo"
NS = REPO + "/src/lidar+sensorfusion"


def sha(p):
    if not os.path.isfile(p):
        return None
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def git(*a):
    return subprocess.run(["git", "-C", REPO] + list(a), capture_output=True, text=True, env=dict(os.environ, GIT_OPTIONAL_LOCKS="0")).stdout.strip()


def cache(variant, key):
    p = f"{NS}/build/exp_{variant}/build/CMakeCache.txt"
    if not os.path.isfile(p):
        return None
    for line in open(p):
        if line.startswith(key + ":"):
            return line.split("=", 1)[1].strip()
    return None


def flags(variant):
    p = f"{NS}/build/exp_{variant}/build/Main/CMakeFiles/lidar_dataset_replay.dir/flags.make"
    if not os.path.isfile(p):
        return None
    return [l.strip() for l in open(p) if l.startswith("CXX_FLAGS")]


env = dict(l.strip().split("=", 1) for l in open(RR + "/RUN_ENV") if "=" in l)
m = {"experiment_id": "track_competition_stage1_" + env.get("TS", ""), "run_env": env,
     "repository": {"shared_git": "/home/autonav/2026AIM/2026-ASMC (.git, worktree)", "worktree": REPO,
                    "branch": git("rev-parse", "--abbrev-ref", "HEAD"), "head": git("rev-parse", "HEAD"),
                    "log": git("log", "--format=%H %s", "916e4d9e..HEAD").splitlines(),
                    "status_porcelain": git("status", "--porcelain=v1").splitlines(),
                    "start_commit_P0": "39cffc57355a76eb35332e0fe70452f36cc70b42",
                    "product_base": "916e4d9efb38f9066aa4b079df05d16ef011c063",
                    "submodule_MORAI-ROS_morai_msgs": "45c6baf148f2327f4c9fefd48262f25bdfe4b567 (git archive from 2026-ASMC-nogps, logs/submodule_populate.log)",
                    "pushed": False}}
builds = {}
for var, desc in (("p0", "P0 rebuild at 39cffc57 (HISTORICAL_REPRODUCTION)"), ("cand", "stage-1 code 669859c1 (COMPETITION_BASELINE with options off, CANDIDATE with options on)"),
                  ("cand_o2", "same source as cand, Release -O2 (realtime comparison only)")):
    d = f"{NS}/build/exp_{var}/devel"
    rc = RR + f"/logs/build/nested_{var}.rc"
    builds[var] = {"description": desc, "build_rc": open(rc).read().strip() if os.path.isfile(rc) else None,
                   "CMAKE_BUILD_TYPE": cache(var, "CMAKE_BUILD_TYPE"), "CATKIN_ENABLE_TESTING": cache(var, "CATKIN_ENABLE_TESTING"),
                   "CMAKE_CXX_FLAGS_RELEASE": cache(var, "CMAKE_CXX_FLAGS_RELEASE"), "flags_make_lidar_dataset_replay": flags(var),
                   "binaries": {b: {"path": f"{d}/{b}", "sha256": sha(f"{d}/{b}")} for b in
                                ("lib/Main/lidar_dataset_replay", "lib/libmyHeader.so", "lib/Main/main", "lib/Main/test_tracking_frame_options")},
                   "outer_devel": REPO + "/devel", "libasmc_lidar_frame_time.so": sha(REPO + "/devel/lib/libasmc_lidar_frame_time.so")}
m["builds"] = builds
m["build_command"] = ("scripts/build_nested.sh (copied from exp03 tools, RR re-pointed): env -i ... catkin_make --build build/exp_<v>/build "
                      "-DCATKIN_DEVEL_PREFIX=build/exp_<v>/devel --only-pkg-with-deps Main --force-cmake -j6 -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON "
                      "-DCMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY=ON -DCATKIN_ENABLE_TESTING=ON (+ -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE=-O2 for cand_o2); "
                      "outer: scripts/build_outer.sh (scripts/build_ws.sh, CMAKE_BUILD_TYPE unset)")
m["toolchain"] = open(RR + "/results/ENVIRONMENT.txt").read() if os.path.isfile(RR + "/results/ENVIRONMENT.txt") else None
ev = REPO + "/tools/experiments/lidar_obb_quality_temporal"
m["evaluator"] = {"eval_lib.py": sha(ev + "/eval_lib.py"), "EVAL_CONTRACT.json": sha(ev + "/EVAL_CONTRACT.json"),
                  "identity_check.py": sha(ev + "/identity_check.py"), "contract_version": "obb_quality_temporal_eval_v2",
                  "supplementary": {p: sha(RR + "/scripts/" + p) for p in ("stage1_metrics.py", "diag_reference_point.py", "trace_link.py", "rt_analyze.py", "syn_report.py")}}
m["data"] = {"lidar_data_manifest": sha("/home/autonav/Lidar_Data/MANIFEST.json"),
             "historical_inputs": E3 + "/work/inputs (frozen exp01..06 inputs, INPUT_MANIFEST sha " + str(sha(E3 + "/work/inputs/INPUT_MANIFEST.json")) + ")",
             "static_bag_inputs": E3 + "/work/static_bag/static_bag",
             "competition_inputs": RR + "/work/inputs_comp (COMP_INPUT_MANIFEST sha " + str(sha(RR + "/work/inputs_comp/COMP_INPUT_MANIFEST.json")) + ")",
             "synthetic_inputs": RR + "/work/synthetic (DIAGNOSTIC_ONLY, SYNTHETIC_MANIFEST sha " + str(sha(RR + "/work/synthetic/SYNTHETIC_MANIFEST.json")) + ")",
             "gt": "labels_3d_lidar_v1 via frozen gt.csv (evaluation only)"}
m["reference_P0_run"] = {"path": E3 + "/work/runs/p0", "ARM.json": sha(E3 + "/work/runs/p0/ARM.json"), "METRICS.json": sha(E3 + "/work/runs/p0/METRICS.json"),
                         "binary_sha256_txt": open(E3 + "/work/runs/p0/binary_sha256.txt").read()}
runs = {}
for d in sorted(glob.glob(RR + "/work/runs/*")):
    n = os.path.basename(d)
    meta = open(d + "/run_meta.txt").read() if os.path.isfile(d + "/run_meta.txt") else ""
    jobs = open(d + "/jobs.txt").read().splitlines() if os.path.isfile(d + "/jobs.txt") else []
    runs[n] = {"run_meta": meta.splitlines()[:1], "end": [l for l in meta.splitlines() if l.startswith(("ns_batch", "nonzero", "eval", "end"))],
               "binary_sha256": open(d + "/binary_sha256.txt").read().splitlines() if os.path.isfile(d + "/binary_sha256.txt") else None,
               "params_yaml_sha256": sha(d + "/params.yaml"), "example_job": jobs[0] if jobs else None, "n_jobs": len(jobs),
               "udp": json.load(open(d + "/udp_snmp.json")).get("udp_out_total") if os.path.isfile(d + "/udp_snmp.json") else None}
m["runs"] = runs
m["execution"] = {"isolation": "scripts/ns_batch.sh: unshare --user --map-root-user --net --pid --fork --mount-proc, lo only (0 routes), private roscore port 20000-29999, Windows interop blocked; UDP datagram delta recorded per run",
                  "replay_speed": "offline harness: synchronous, as fast as possible (no pacing), sim time ros::Time::setNow(source stamp) per scan; realtime runs: real Main node via roslaunch main_integration.launch planner_udp_enabled:=false, 15 Hz CLOCK_MONOTONIC-paced feeder (scripts/rt_feeder.py), wall ROS time",
                  "external_outputs": "planner UDP disabled (planner_udp_enabled:=false in dumped params; the offline harness never creates the sender); vehicle control not started; namespace has no route",
                  "params_source": "roslaunch --dump-params Main main_integration.launch planner_udp_enabled:=false + --set imu_yaw_is_true_enu=false (+ option --set)",
                  "threads": "one harness process per scene, xargs -P 6, LidarProcess single-threaded"}
m["current_instance"] = "No live MORAI/vehicle instance was used in stage 1; all results are offline replays."
json.dump(m, open(sys.argv[1], "w"), indent=1)
print("manifest runs:", len(runs))
