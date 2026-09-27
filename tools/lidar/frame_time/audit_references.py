#!/usr/bin/env python3
"""Read-only checkout inventory; selected-source review is NOT a full code audit.

Never builds/imports upstream Python, installs dependencies, or changes checkouts.
License strings are file findings, not legal clearance for copying/distribution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

# name, official origin suffix, license finding, selected source, input/time/dependency finding
REFERENCES = [
    ("patchwork-plusplus", "url-kaist/patchwork-plusplus", "BSD-2-Clause core",
     "cpp/patchworkpp/include/patchwork/patchworkpp.h", "XYZI single cloud; sensor height/local plane assumptions; Eigen C++; L1-A 100-frame ground comparison only"),
    ("patchwork-plusplus-ros", "url-kaist/patchwork-plusplus-ros", "GPL-3.0",
     "include/patchworkpp/patchworkpp.hpp", "ROS1/PCL wrapper differs from BSD core; not separately built"),
    ("TRAVEL", "url-kaist/TRAVEL", "GPL-3.0",
     "cpp/travel/core/travel/tgs.hpp", "single cloud topology ground/object segmentation; C++ core/current ROS2 wrapper; ROS1 v0.1 is separate"),
    ("linefit_ground_segmentation", "lorenwel/linefit_ground_segmentation", "BSD-3-Clause",
     "linefit_ground_segmentation_ros/src/ground_segmentation_node.cc", "PCL/ROS1; gravity frame rotation with zeroed translation; sensor height must match input origin; first additional ground comparator"),
    ("depth_clustering", "PRBonn/depth_clustering", "MIT (LICENCE.txt)",
     "src/clusterers/image_based_clusterer.h", "range image projection/connectivity; VLP16 beam angles/missing beams require validation; PCL/OpenCV"),
    ("groundgrid", "dcmlr/groundgrid", "BSD-3-Clause",
     "src/GroundGridNodelet.cpp", "ROS1/PCL/grid_map; odometry filtered_map and map TF; no odom blocks clouds; conditional comparator"),
    ("adaptive_clustering", "yzrobot/adaptive_clustering", "root BSD-3-Clause; source GPL-3.0-or-later",
     "src/adaptive_clustering.cpp", "ROS1/PCL radial shells; VLP16 default sum about 28m; PointXYZ loses intensity; research-only license conflict"),
    ("DipG-Seg", "EEPT-LAB/DipG-Seg", "GPL-3.0",
     "src/include/projection_param.h", "ROS/PCL/OpenCV; empty YOURLIDAR_CONFIG; VLP16 projection gate; demo now/frame overwrite is not acceptable replay semantics"),
    ("SimpleTrack", "tusen-ai/SimpleTrack", "MIT",
     "mot_3d/mot.py", "box/ego/time_stamp inputs; association/redundancy/lifecycle reference; NumPy/SciPy; upstream FPS not local rejection criterion"),
    ("AB3DMOT", "xinshuoweng/AB3DMOT", "academic nonprofit noncommercial research",
     "AB3DMOT_libs/model.py", "boxes and frame-index lifecycle; KITTI axes/cadence adapter required; Python/KF; license review before redistribution"),
    ("Poly-MOT", "lixiaoyu2000/Poly-MOT", "README MIT; no root LICENSE found",
     "tracking/nusc_tracker.py", "nuScenes boxes/category/frame_id; sequence resets and two-stage association; no local latency measurement"),
    ("FastPoly", "lixiaoyu2000/FastPoly", "README MIT; no root LICENSE found",
     "tracking/nusc_tracker.py", "nuScenes box/category/frame_id; a-gIoU/voxel association masks; CPU/Python; no local latency measurement"),
    ("M-detector", "hku-mars/M-detector", "GPL-2.0",
     "src/DynObjFilter.cpp", "registered body clouds/odom and delayed temporal buffer; ROS/PCL/Livox build dependencies; not a static-obstacle detector"),
    ("LiDAR-MOS", "PRBonn/LiDAR-MOS", "MIT",
     "utils/gen_residual_images.py", "poses+calibration warp past clouds to residual range images; PyTorch; time/label preparation required"),
    ("4DMOS", "PRBonn/4DMOS", "MIT",
     "src/mos4d/datasets/mos4d_dataset.py", "past scans+timestamps+odometry; Lightning/MinkowskiEngine temporal network; not single-frame drop-in"),
    ("MotionBEV", "xiekkki/MotionBEV", "MIT",
     "dataloader/dataset.py", "precomputed residual history/semantic moving labels; PyTorch; no local time alignment or VLP16 validation"),
    ("OpenPCDet", "open-mmlab/OpenPCDet", "Apache-2.0",
     "pcdet/datasets/nuscenes/nuscenes_dataset.py", "sweep transform/time-lag dataset adapter; CUDA/PyTorch; backbone-dependent spconv; pretrained labels/domain not assumed valid"),
    ("CenterPoint", "tianweiy/CenterPoint", "MIT",
     "det3d/models/bbox_heads/center_head.py", "head forward(x) with configured velocity branch; loader has sweep/time-lag; two map views not universal input requirement; PyTorch/CUDA"),
    ("CenterPoint-ONNX-TensorRT", "CarkusL/CenterPoint", "MIT",
     "det3d/datasets/pipelines/loading.py", "export fork; sweep/time-lag loader plus fixed engine layout; CUDA/TensorRT; not identical to all CenterPoint variants"),
    ("Lidar_AI_Solution", "NVIDIA-AI-IOT/Lidar_AI_Solution", "MIT root; component licenses separate",
     "CUDA-PointPillars/src/pointpillar.cpp", "CUDA/TensorRT engine/voxel buffers; runtime dimensions/resources need measurement; no automatic VLP16 latency scaling"),
    ("FastPillars", "StiphyJay/FastPillars", "MIT code; dataset terms separate",
     "det3d/models/readers/pillar_encoder.py", "point/pillar aggregation with configured grid; PyTorch/CUDA/TRT path; dense BEV cost persists for sparse scans"),
    ("pillarnext", "qcraftai/pillarnext", "CC-BY-NC-SA-4.0 root; inherited reader MIT header",
     "det3d/models/readers/pillar_encoder.py", "point range/grid and torch_scatter dynamic pillars; whole repo license differs from inherited reader; training comparison later"),
    ("CLOCs", "pangsu0613/CLOCs", "MIT",
     "second/pytorch/models/fusion.py", "candidate 2D/3D fusion tensor; fixed candidate capacities; torch/spconv/SECOND; calibrated synchronous detections required"),
    ("bevfusion", "mit-han-lab/bevfusion", "Apache-2.0",
     "mmdet3d/models/fusion_models/bevfusion.py", "camera2ego/lidar2ego/lidar2camera inputs; calibration/time consistency; PyTorch/mmcv/CUDA; no resource rejection without test"),
    ("LIO-SAM", "TixiaoShan/LIO-SAM", "BSD-3-Clause",
     "src/imageProjection.cpp", "point time and ring; IMU scan coverage; sparse reference-only checkout; no SLAM runtime adoption"),
    ("autoware_universe", "autowarefoundation/autoware_universe", "Apache-2.0",
     "sensing/autoware_pointcloud_preprocessor/src/distortion_corrector/distortion_corrector.cpp",
     "per-point uint32 time_stamp and twist/IMU queues, 2D/3D correction; sparse reference-only checkout; no ROS2 runtime adoption"),
]


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def collect(root):
    entries = []
    for name, origin, license_note, selected, finding in REFERENCES:
        checkout = root / name
        url = git(checkout, "remote", "get-url", "origin")
        if url.removesuffix(".git").rstrip("/").lower() != ("https://github.com/"+origin).lower():
            raise ValueError("unexpected origin: " + name)
        evidence = {selected: digest(checkout / selected)}
        for path in sorted(checkout.iterdir()):
            if path.is_file() and (path.name.lower().startswith(("readme", "license", "licence"))):
                evidence[path.name] = digest(path)
        entries.append({"name": name, "official_origin": url, "sha": git(checkout, "rev-parse", "HEAD"),
                        "dirty_status": git(checkout, "status", "--porcelain"), "license_finding": license_note,
                        "input_time_frame_dependencies": finding, "file_sha256": evidence,
                        "inventory": "checked", "code_review": "selected_input_or_core_path_only_not_full_audit",
                        "build": "L1A_standalone_core_only" if name == "patchwork-plusplus" else "not_run",
                        "experiment": "L1A_100_admitted_ground_frames_only" if name == "patchwork-plusplus" else "not_run"})
    return {"schema": "asmc-lidar-reference-audit-v1", "date": "2026-09-27", "candidate_count": 24,
            "new_reference_count": 2, "runtime_dependencies_added": False, "repositories": entries}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baselines", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = collect(args.baselines)
    with args.output.open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, sort_keys=True, indent=2)
        stream.write("\n")
    print(f"Inventoried {len(report['repositories'])} checkouts; no builds executed")


if __name__ == "__main__":
    main()
