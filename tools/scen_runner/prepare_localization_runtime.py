#!/usr/bin/env python3
"""Stage an immutable code bundle, without replacing another ROS overlay.

Copy the result to runtime-prefix in the existing ROS container. Its process
environment prepends bundle/devel/lib, while historical standalone bundles remain reproducible. The current GPU full-stack
container uses its own fresh build/devel instead of this legacy bundle mode.
Experiment outputs are separate registered D runs, never this code directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path)
    p.add_argument("--runtime-prefix",type=Path,required=True)
    p.add_argument("--devel-space",type=Path,help="fresh isolated devel directory; source checkout unchanged")
    a=p.parse_args();source=Path(__file__).resolve().parents[2]
    output=a.output.resolve();prefix=a.runtime_prefix
    if not prefix.is_absolute() or (source/".runtime").resolve() not in output.parents:
        raise ValueError("code bundle must be under this checkout's .runtime; runtime-prefix must be absolute")
    paths=["devel/lib/asmc_planner/"+n for n in (
        "asmc_local_planner_node","asmc_shadow_localization_node","asmc_control_selector_node")]
    paths += ["devel/lib/asmc_controller/asmc_local_controller_node"]
    paths += ["devel/lib/"+n+".so" for n in (
        "libasmc_shadow_localization_core","libasmc_local_planner_core","libasmc_planner_frenet_core",
        "libasmc_signal_policy_core","libmpc_ros_converter","libmpc_core")]
    paths += ["src/asmc_planner/src/local/node.cpp","src/asmc_planner/src/local/motion.hpp",
        "src/asmc_planner/src/local/planner.cpp","src/asmc_planner/src/local/planner.hpp",
        "src/asmc_planner/src/Frenet/frenet_planner.cpp","src/asmc_planner/src/Frenet/frenet_planner.hpp",
        "src/asmc_planner/src/Planning/ControlSelector.cpp","src/asmc_planner/src/Planning/ControlSelector.hpp",
        "src/asmc_controller/src/local/node.cpp","src/asmc_planner/launch/shadow_localization.launch",
        "src/asmc_planner/config/frenet/params.yaml","src/asmc_planner/config/shadow_localization.yaml","src/asmc_controller/config/controller.yaml",
        "src/asmc_controller/config/local_controller.yaml","src/planning_inputs/config/vehicle_state.yaml",
        "R_KR_PG_KATRI/2026_molit_comp_global_path.txt"]
    paths += ["tools/scen_runner/"+n for n in (
        "run_local_trial.py","analyze_localization.py","prepare_localization_runtime.py",
        "localization_trial_policy.py","localization_assessment.py")]
    for directory in ("src/asmc_planner/config/frenet/hd_map","src/asmc_planner/src/shadow_localization"):
        paths += [str(f.relative_to(source)) for f in (source/directory).rglob("*") if f.is_file()]
    def original(name):
        return a.devel_space/name[len('devel/'):] if a.devel_space and name.startswith('devel/') else source/name
    missing=[name for name in paths if not original(name).is_file()]
    if missing:raise RuntimeError("build/source incomplete: "+str(missing))
    output.mkdir(parents=True,exist_ok=False)
    hashes={}
    for name in paths:
        target=output/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(original(name),target);hashes[name]=hashlib.sha256(target.read_bytes()).hexdigest()
    config_path=output/"src/asmc_planner/config/frenet/params.yaml"
    config=yaml.safe_load(config_path.read_text())
    config["waypoint_file"]=str(prefix/"R_KR_PG_KATRI/2026_molit_comp_global_path.txt")
    config["avoid_hd_map"]["directory"]=str(prefix/"src/asmc_planner/config/frenet/hd_map")
    config_path.write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False))
    record=dict(runtime_prefix=str(prefix),source_revision=subprocess.check_output(
        ["git","rev-parse","HEAD"],cwd=source,text=True).strip(),source_file_sha256=hashes,
        deployed_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        config_changes="only absolute waypoint and map bundle paths; algorithm settings unchanged")
    (output/"bundle-manifest.json").write_text(json.dumps(record,indent=2)+"\n")
    print(output)


if __name__=="__main__":main()
