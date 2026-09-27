#!/usr/bin/env python3
"""Run teammate evaluator on an isolated master, without editing its code.

Source the detached nested workspace first. This is not a live launch/replay.
Sweep timings are offline CPU timing, NOT input-to-output latency or GT accuracy.
"""
import argparse
import copy
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client


def descendant_of(pid, ancestor):
    """Linux-only ownership check before touching a private parameter server."""
    for _ in range(32):
        if pid == ancestor:
            return True
        if pid <= 1:
            return False
        try:
            pid = int(Path("/proc/{}/stat".format(pid)).read_text().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            return False
    return False


def stop_group(process, timeout=10):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=timeout)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", type=Path, required=True)
    p.add_argument("--params", type=Path, required=True,
                   help="rosparam dump of the actual /Main launch config")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--expect-main-path", type=Path, required=True,
                   help="exact detached Main package path; prevents wrong ROS overlay")
    p.add_argument("--port", type=int, default=11331)
    p.add_argument("--processing-delay", type=float, default=.13)
    p.add_argument("--timeout", type=float, default=1800)
    args = p.parse_args()
    if (not 11320 <= args.port <= 11999 or not math.isfinite(args.processing_delay)
            or args.processing_delay < 0 or not math.isfinite(args.timeout) or args.timeout <= 0):
        p.error("private port must be 11320..11999, delay nonnegative")
    if not args.bag.is_file() or not args.params.is_file():
        p.error("bag and parameters must exist")
    with socket.socket() as sock:
        # Refuse to reuse/overwrite a pre-existing ROS parameter server.
        sock.bind(("127.0.0.1", args.port))
    import yaml
    params = yaml.safe_load(args.params.read_text())
    if not isinstance(params, dict) or params.get("enable_tunnel_experimental_filter") is not True:
        p.error("this benchmark requires the teammate tunnel filter enabled")
    resolved_main = Path(subprocess.check_output(["rospack", "find", "Main"],
                                                 text=True).strip()).resolve()
    if resolved_main != args.expect_main_path.resolve():
        p.error("Main overlay mismatch: {}".format(resolved_main))
    args.output.mkdir(parents=True, exist_ok=False)
    params["sweep_configs"] = "baseline,no_protection,camera_missing"
    base = copy.deepcopy(params)
    base.pop("sweep_configs")
    params["sweep"] = {}
    for name, protect, camera_fail in (("baseline", True, False),
                                       ("no_protection", False, False),
                                       ("camera_missing", True, True)):
        params["sweep"][name] = dict(base, protect=protect, camera_fail=camera_fail)
    config = args.output / "resolved_params.yaml"
    config.write_text(yaml.safe_dump(params, sort_keys=True))
    env = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:{}".format(args.port),
               ROS_HOSTNAME="127.0.0.1")
    env.pop("ROS_IP", None)
    env.pop("ROS_NAMESPACE", None)
    command = ["rosrun", "Main", "tunnel_filter_offline_eval", "--bag",
               str(args.bag.resolve()), "--out", str(args.output.resolve()),
               "--processing-delay", str(args.processing_delay)]
    source_hashes = {str(path.relative_to(resolved_main)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in sorted(resolved_main.rglob("*"))
                     if path.is_file() and path.suffix in (".cpp", ".hpp", ".h", ".yaml", ".launch")}
    info = {"command": command, "master": env["ROS_MASTER_URI"], "status": "running",
            "resolved_main": str(resolved_main), "source_sha256": source_hashes,
            "source_params": str(args.params), "limitations": [
                "same scan with independent filter state for each sweep",
                "camera_missing uses existing ambiguous-point fallback, not disabled protection",
                "no_protection sweep differs from the current runtime flag implementation",
                "no ground-truth object precision or recall computed",
                "baseline replica mismatch compares legacy RANSAC against active tunnel path; "
                "not necessarily a reproduction failure",
                "processing-delay advances buffered pose data; not guaranteed scan-time pose"]}
    master = None
    evaluation = None
    start = time.monotonic()
    try:
        with (args.output / "master.log").open("w") as master_log, \
                (args.output / "eval.log").open("w") as eval_log:
            master = subprocess.Popen(["roscore", "-p", str(args.port)], env=env,
                                      stdout=master_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            ready = False
            for _ in range(100):
                if master.poll() is not None: break
                try:
                    response = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).getPid(
                        "/lidar_offline_runner")
                    ready = response[0] == 1 and descendant_of(response[2], master.pid)
                    if ready: break
                except OSError:
                    pass
                time.sleep(.1)
            if not ready: raise RuntimeError("private master did not start")
            subprocess.run(["rosparam", "load", str(config),
                            "/tunnel_filter_offline_eval"], env=env, check=True)
            evaluation = subprocess.Popen(command, env=env, stdout=eval_log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
            info["returncode"] = evaluation.wait(timeout=args.timeout)
            if info["returncode"]:
                raise subprocess.CalledProcessError(info["returncode"], command)
            info["status"] = "complete"
    except BaseException as exc:
        info["status"] = "failed"
        info["error"] = repr(exc)
        raise
    finally:
        try:
            for proc in (evaluation, master):
                try: stop_group(proc)
                except Exception as exc:
                    info.setdefault("cleanup_errors", []).append(repr(exc))
        finally:
            info["wall_elapsed_s"] = time.monotonic()-start
            (args.output / "invocation.json").write_text(json.dumps(info, indent=2))


if __name__ == "__main__":
    main()
