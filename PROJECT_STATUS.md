# LiDAR frozen handoff
Read REPRODUCE.md, handoff/lidar/DATA_MANIFEST.json and AGENTS.md next.

Problem: fragmented vehicle/background observations can transfer IDs and generate false velocity. Preserve P0-derived A+B with yaw EMA ON. This is NOT a replacement with Yangtest/main.

Frozen algorithm source: a6fcf88294db2e44fe303580e54c1f3fcff9065e. New handoff commits only prepare reproducibility.
A: second-observation initialization divides by actual accumulated elapsed time including missed scans.
B: ego translation from signed Competition vel_x and IMU finite rotation/lever arm alignment yields ground-referenced KF velocity. No second CAN addition. It assumes the permitted ego inputs/time/frame support are valid; it cannot correct a moving point-mean reference.
Required options are pinned explicitly in handoff/lidar/run.py: A+B=true, EMA=true; long-axis, NIS, point-support ownership, quality R, OBB center measurement, jerk Q, gap reset and two-point covariance all OFF.
Rejected code remains present. Presence does not mean activation.

Rejected: long-axis canonical state regressed speed/IDs; NIS gate increased actual IDSW66->104 and common speed MAE1.072->1.089; support ownership failed sparse actual case and ambiguous moving-object/background reveal. Split/rejoin also has an existing baseline limitation. Do not treat observationally identical sparse inputs with contradictory latent truths as simultaneously satisfiable tests: preserve and review the ambiguity.

Proven actual full lap baseline (gt_aligned_v1):722 valid speed samples, MAE1.117768m/s, p95 2.818771, p99 8.252803, max15.767373; IDSW66/fragmentation5. GT15 scan1150 inherits earlier background-fragment state. Runtime353 born1086, false vx8.8345 at1090. Results provenance is in handoff/lidar/evidence.
Invalid deleted static_bag AND derivatives are excluded. Do not restore or reuse them. New static validation, physical GT reference confirmation, clock/deskew, independent full-load MORAI15Hz validation are incomplete.
Actual lap and causal-emulated scen93/21/102 are separate conditions; do not aggregate improvement rates. Development scenes are reused diagnostics, not an untouched independent holdout. Primary84/highspeed have not passed the latest rejected candidate.

Inference: LiDAR, IMU, GPS, signed Competition vel_x only. Restricted bag /Ego_topic is a velocity wrapper, not evaluation Ego GT. GT pose/vel_y/z/acceleration/object GT/IDs/dimensions are evaluation-only. GT aligned_v1 uses bracketing samples only in evaluation; speed is derivative of an interpolation model with interval/reference limitations. FP is partial because of unresolved/unlabeled objects.
Next judgment: pre-birth surface history and distributed 3D/motion evidence are needed before asserting fragment ownership. This handoff introduces no algorithm.
