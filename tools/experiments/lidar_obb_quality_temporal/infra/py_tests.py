#!/usr/bin/env python3
"""Offline Python tests/checks (copy of the B0 audit s22_python_tests.py, paths parametrised).
usage: py_tests.py REPO_ROOT NESTED_DEVEL_SETUP OUT_DIR [ci]
CI offline commands are parsed from REPO_ROOT/.github/workflows/ci.yml; Main nose files from its CMakeLists.
Run inside unshare --user --map-root-user --net (no network).
"""
import json, os, re, subprocess, sys, time, glob, shlex

BT = sys.argv[1]
LB = sys.argv[3]
NS = BT + "/src/lidar+sensorfusion"
PYH = LB + "/py_home"
os.makedirs(PYH + "/mpl", exist_ok=True)
os.makedirs(PYH + "/yolo", exist_ok=True)
sys.path.insert(0, "/usr/lib/python3/dist-packages")
import yaml  # noqa: E402

OUTER_SETUP = "/home/autonav/2026AIM/Lidar_experimments/obb_quality_temporal_20261018_170239/repo/devel/setup.bash"
NESTED_SETUP = sys.argv[2]


def ci_commands():
    ci = yaml.safe_load(open(BT + "/.github/workflows/ci.yml"))
    out, deferred = [], []
    for st in ci["jobs"]["catkin-build"]["steps"]:
        name, run = st.get("name", ""), st.get("run", "")
        if not run:
            continue
        lines = [l.strip() for l in run.splitlines() if l.strip()]
        sources_devel = any(l.startswith("source devel/setup.bash") for l in lines)
        if name.startswith("Check inference control faults"):
            deferred += [l for l in lines if l.startswith(("python3", "roslaunch", "roscore"))]
            continue
        if name in ("Install git and build deps", "catkin_make", "Verify morai_msgs submodule"):
            continue
        if name.startswith("Check LiDAR coordinate"):
            continue
        for l in lines:
            if l.startswith(("source ", "export ", "cd ")):
                continue
            out.append(("ci:" + name, l, sources_devel))
    return out, deferred


def nose(path):
    return "python3 -m nose -v " + shlex.quote(path)


def build_list(only_ci):
    items = []
    ci, deferred = ci_commands()
    for i, (grp, cmd, src) in enumerate(ci):
        items.append(dict(group=grp, name="ci%02d" % i, cmd=cmd, setup=(OUTER_SETUP if src else None)))
    if only_ci:
        return items, deferred
    ex = []
    # G6: every catkin_add_nosetests of Main/CMakeLists.txt (baseline 2 files + G3 test_main_launch_contract.py)
    main_nose = re.findall(r"^\s*catkin_add_nosetests\(\s*test/([A-Za-z0-9_]+\.py)", open(NS + "/src/Main/CMakeLists.txt").read(), re.M)
    for f in main_nose:
        ex.append(("extra:nested_Main_nosetests", "Main_" + f[:-3], nose(NS + "/src/Main/test/" + f), NESTED_SETUP))
    ex.append(("extra:asmc_msgs_nosetests", "asmc_msgs_test_message_serialization", nose("src/asmc_msgs/test/test_message_serialization.py"), OUTER_SETUP))
    ex.append(("extra:asmc_msgs_nosetests", "asmc_msgs_test_lidar_object_v2", nose("src/asmc_msgs/test/test_lidar_object_v2.py"), OUTER_SETUP))
    for p in ["test/test_facade_contract.py", "test/test_traffic_light_contract.py", "test/test_traffic_light_visualization.py",
              "test/camera", "test/bev", "test/detection", "test/runtime"]:
        ex.append(("extra:asmc_camera_nosetests", "asmc_camera_" + p.replace("test/", "").replace("/", "_").replace(".py", ""), nose("src/asmc_camera/" + p), OUTER_SETUP))
    for f in sorted(glob.glob(BT + "/src/data_collection/test/test_*.py")):
        b = os.path.basename(f)[:-3]
        ex.append(("extra:data_collection_nosetests", "data_collection_" + b, nose("src/data_collection/test/" + b + ".py"), OUTER_SETUP))
    for f in sorted(glob.glob(BT + "/src/asmc_bridge/test/test_*.py")):
        b = os.path.basename(f)[:-3]
        ex.append(("extra:asmc_bridge_nosetests", "asmc_bridge_" + b, nose("src/asmc_bridge/test/" + b + ".py"), OUTER_SETUP))
    hd = sorted(glob.glob(BT + "/src/asmc_hd_map/test/test_*.py"))
    ex.append(("extra:asmc_hd_map_nosetests", "asmc_hd_map_all", "python3 -m nose -v " + " ".join(shlex.quote(os.path.relpath(f, BT)) for f in hd), OUTER_SETUP))
    for f in ["object_filter_contract_test.py", "runtime_launch_contract_test.py"]:
        ex.append(("extra:planning_inputs_unregistered", "planning_inputs_" + f[:-3], "python3 -m unittest -v src/planning_inputs/test/" + f, OUTER_SETUP))
    ex.append(("extra:asmc_camera_unregistered", "asmc_camera_training", "python3 -m unittest discover -v -s src/asmc_camera/test/training -p 'test_*.py'", OUTER_SETUP))
    for f in ["test_camera_bbox_regression.py", "test_side_ego_calibration.py", "test_yoloe_prompt_preparation.py"]:
        ex.append(("extra:tools_perception_not_in_CI", "tools_perception_" + f[:-3], "python3 -m unittest discover -v -s tools/perception -p '%s'" % f, OUTER_SETUP))
    for f in ["test_build_runtime_baseline.py", "test_perception_runtime_baseline.py", "test_traffic_light_parity_report.py"]:
        ex.append(("extra:tools_audit_not_in_CI", "tools_audit_" + f[:-3], "python3 -m unittest -v tools/audit/" + f, OUTER_SETUP))
    for grp, name, cmd, setup in ex:
        items.append(dict(group=grp, name=name, cmd=cmd, setup=setup))
    return items, deferred


RAN = re.compile(r"^Ran (\d+) tests? in", re.M)
FIN = re.compile(r"^(OK|FAILED)(?: \(([^)]*)\))?\s*$", re.M)


def parse(text):
    ran = [int(x) for x in RAN.findall(text)]
    fins = FIN.findall(text)
    res = dict(ran=sum(ran) if ran else None, failures=0, errors=0, skipped=0, expected_failures=0, unittest_blocks=len(ran))
    for status, detail in fins:
        for k, v in re.findall(r"([A-Za-z_ ]+)=(\d+)", detail or ""):
            k = k.strip().lower()
            key = {"failures": "failures", "errors": "errors", "skipped": "skipped", "skip": "skipped",
                   "expected failures": "expected_failures"}.get(k)
            if key:
                res[key] += int(v)
    res["final_status"] = ",".join(s for s, _ in fins) if fins else None
    return res


def main():
    only_ci = len(sys.argv) > 4 and sys.argv[4] == "ci"
    os.makedirs(LB, exist_ok=True)
    items, deferred = build_list(only_ci)
    results = []
    for it in items:
        log = "%s/py_%s.log" % (LB, it["name"])
        pre = "source /opt/ros/noetic/setup.bash; source %s; " % shlex.quote(it["setup"]) if it["setup"] else ""
        script = ("cd %s && %sexport ROS_MASTER_URI=http://127.0.0.1:65530 ROS_HOSTNAME=localhost; "
                  "echo \"### cmd: %s\"; timeout 600 %s") % (shlex.quote(BT), pre, it["cmd"].replace('"', '\\"'), "bash -c " + shlex.quote(it["cmd"]))
        env = {"HOME": "/home/autonav", "USER": "autonav", "LOGNAME": "autonav", "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8",
               "TERM": "dumb", "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
               "ROS_HOME": LB + "/ros_home", "ROS_LOG_DIR": LB + "/ros_home/log", "PYTHONDONTWRITEBYTECODE": "1",
               "MPLCONFIGDIR": PYH + "/mpl", "YOLO_CONFIG_DIR": PYH + "/yolo", "YOLO_OFFLINE": "true",
               "HF_HUB_OFFLINE": "1", "TRANSFORMERS_OFFLINE": "1", "GIT_OPTIONAL_LOCKS": "0"}
        t0 = time.time()
        with open(log, "w") as fh:
            p = subprocess.run(["bash", "--noprofile", "--norc", "-c", script], stdout=fh, stderr=subprocess.STDOUT, env=env)
        dt = int((time.time() - t0) * 1000)
        text = open(log, errors="replace").read()
        r = dict(it, rc=p.returncode, ms=dt, log=log)
        r.update(parse(text))
        r["kind"] = "script" if r["ran"] is None else "unittest"
        imp = re.findall(r"^(?:ModuleNotFoundError|ImportError): .*$", text, re.M)
        r["import_errors"] = sorted(set(imp))[:10]
        r["result"] = "PASS" if p.returncode == 0 else ("TIMEOUT" if p.returncode == 124 else "FAIL")
        results.append(r)
        print("%-45s %-40s rc=%-3s ms=%-7s ran=%s f=%s e=%s s=%s" % (it["group"][:45], it["name"], p.returncode, dt, r["ran"], r["failures"], r["errors"], r["skipped"]), flush=True)
    ci = [r for r in results if r["group"].startswith("ci:")]
    ex = [r for r in results if not r["group"].startswith("ci:")]
    def summ(rs):
        return dict(commands=len(rs), failed=sum(r["rc"] != 0 for r in rs), ran=sum(r["ran"] or 0 for r in rs),
                    failures_errors=sum(r["failures"] + r["errors"] for r in rs), skipped=sum(r["skipped"] for r in rs))
    summary = dict(ci=summ(ci), extras=summ(ex))
    print("SUMMARY", json.dumps(summary))
    json.dump(dict(results=results, summary=summary, deferred_ci_master_step=deferred),
              open(LB + "/py_results%s.json" % ("_ci" if only_ci else ""), "w"), indent=1)


if __name__ == "__main__":
    main()
