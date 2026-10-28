#!/usr/bin/env bash
# Run from the repository root after the single workspace build.
source .github/ci/ros-env.sh
python3 -m unittest discover -s tools/perception -p 'test_camera_model_release.py'
python3 -m unittest discover -s src/asmc_camera/test -p 'test_facade_contract.py'
python3 -m unittest discover -s src/asmc_camera/test/detection -p 'test_*.py'
python3 -m unittest discover -s src/asmc_camera/test/camera -p 'test_*.py'
python3 -m unittest tools/perception/test_package_twinlite_checkpoint.py
python3 -m unittest tools/perception/test_assess_twinlite_deployment.py
python3 -m unittest discover -s src/asmc_camera/test/bev -p 'test_*.py'
python3 -m unittest tools/perception/test_hood_evaluation.py
python3 -m unittest tools/perception/test_lifecycle_measurements.py
python3 -m unittest tools/perception/test_b0_measurements.py
python3 -m unittest tools/perception/test_coco7_evaluation.py
python3 -m unittest discover -s tools/perception -p 'test_coco7_weather.py'
python3 -m unittest discover -s tools/perception -p 'test_coco7_batch_benchmark.py'
python3 -m unittest discover -s tools/perception -p 'test_coco7_range_batches.py'
python3 -m unittest discover -s tools/perception -p 'test_coco7_finalists.py'
python3 -m unittest discover -s tools/perception -p 'test_coco7_broad_selection.py'
python3 -m unittest discover -s tools/perception -p 'test_coco7_confidence_followup.py'
python3 -m unittest discover -s tools/perception -p 'test_experiment_gallery_hub.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_coco_measurements.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_coco_matrix.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_seg_gt.py'
python3 -m unittest discover -s tools/perception -p 'test_seg_measurements.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_seg_matrix.py'
python3 -m unittest discover -s tools/perception -p 'test_seg_lidar_support.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_seg_accuracy.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_mission_classes.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_data_audit.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_data_readiness.py'
python3 -m unittest discover -s tools/perception -p 'test_prepare_camera_direct_review.py'
python3 -m unittest discover -s tools/perception -p 'test_prepare_camera_breadth_dataset.py'
python3 -m unittest discover -s tools/perception -p 'test_prepare_camera_obstacle_review.py'
python3 -m unittest discover -s tools/perception -p 'test_prepare_capture_training_data.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_bbox_pilot.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_bbox_training.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_pretrain.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_bbox_main.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_evaluation_inputs.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_capture_restoration.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_lr_branch.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_map_background.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_prediction_nms.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_preflight_revision.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_pretrain_torch.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_pretrain_v10.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_small_supervision.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_spatial_background.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_supervision_policy_review.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_validation16.py'
python3 -m unittest discover -s tools/perception -p 'test_fixed_map_drums.py'
python3 -m unittest discover -s tools/perception -p 'test_mission16_evaluation.py'
python3 -m unittest discover -s tools/perception -p 'test_rfdetr_candidate.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_label_changes.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_teacher_audit.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_teacher_expansion.py'
python3 -m unittest discover -s tools/perception -p 'test_capture_route_export.py'
python3 -m unittest discover -s tools/perception -p 'test_capture_collection_guard.py'
python3 -m unittest discover -s tools/perception -p 'test_capture_gt_sizes.py'
python3 -m unittest discover -s tools/perception -p 'test_capture_pedestrian_respawn.py'
# Synthetic capture fixtures write only TemporaryDirectory (/tmp).
ASMC_STORAGE_UNMANAGED_OUTPUT=1 PYTHONPATH=src/data_collection/src python3 -m unittest discover -s src/data_collection/test -p 'test_capture*.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_data_expansion_v7.py'
python3 -m unittest discover -s tools/perception -p 'test_yoloe_l_measurements.py'
python3 -m unittest discover -s tools/perception -p 'test_yoloe_selection.py'
python3 -m unittest discover -s tools/perception -p 'test_yoloe_l_diagnostics.py'
python3 -m unittest tools/perception/test_b1a_protocol.py
python3 -m unittest tools/perception/test_frame_boundary_measurements.py
python3 -m unittest tools/perception/test_lidar_continuous_measurements.py
python3 -m unittest tools/perception/test_tunnel_case_geometry.py
python3 -m unittest tools/perception/test_tunnel_summary.py
python3 -m unittest tools/perception/test_camera_lidar_evidence.py
python3 -m unittest discover -s src/asmc_camera/test/runtime -p 'test_*.py'
python3 -m unittest discover -s tools/perception -p 'test_camera_runtime_measurements.py'
python3 -m unittest discover -s tools/perception -p 'test_imu_runtime_metrics.py'
python3 -m unittest discover -s tools/perception -p 'test_imu_width_acceptance.py'
python3 -m unittest discover -s tools/perception -p 'test_shadow_family_validation.py'
python3 -m unittest discover -s tools/perception -p 'test_lane_engine_selection.py'
python3 -m unittest discover -s tools/perception -p 'test_object_engine_selection.py'
python3 -m unittest discover -s tools/perception -p 'test_prepare_local_camera_engines.py'
python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_camera_runtime.py'
