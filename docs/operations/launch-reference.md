# Launch 인자 목록

현재 XML의 최상위 인자를 자동 추출합니다. 실행 순서·권한·추천 조합은 [명령 안내](commands.md)를 따릅니다.
인자가 없다고 component를 독립 실행해도 되는 것은 아닙니다. 내부 namespace·모델 경로·실험 설정은 YAML을 우선합니다.
`$(...)`는 roslaunch가 해석하는 표현이며 여기에 표시된 문자열이 평가 결과는 아닙니다.

갱신: `python3 tools/audit/check_runtime_documentation.py --write`
검사: `python3 tools/audit/check_runtime_documentation.py`

## Bridge

[소스](../../src/asmc_bridge/launch/bridge.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `runtime_profile` | `morai_udp` | — |
| `transport_profile` | `local` | — |
| `stream_profile` | `competition` | — |
| `timestamp_mode` | `receive` | — |
| `publish_map_tf` | `false` | — |
| `lidar_frame_mode` | `azimuth` | — |
| `lidar_rotation_rate` | `15.0` | v3/GT assembly reference Hz; sensor rate is set in MORAI |
| `cmd_ip` | `$(optenv MORAI_CMD_IP 127.0.0.1)` | — |
| `cmd_port` | `9093` | — |
| `ego_port` | `9099` | — |
| `enable_ego_status` | `true` | — |
| `enable_competition_status` | `$(eval arg('transport_profile') == 'local')` | — |
| `competition_port` | `9798` | — |
| `competition_status_required` | `false` | — |
| `enable_vehicle_state` | `true` | — |

## Network bridge component

[소스](../../src/asmc_bridge/launch/morai_udp_bridge.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `runtime_profile` | `morai_udp` | — |
| `transport_profile` | `local` | — |
| `stream_profile` | `competition` | — |
| `enable_ego_status` | `true` | — |
| `enable_competition_status` | `$(eval arg('transport_profile') == 'local')` | — |
| `competition_status_required` | `false` | — |
| `enable_object` | `$(eval arg('stream_profile') == 'development')` | — |
| `enable_collision` | `true` | — |
| `ego_port` | `9099` | — |
| `competition_port` | `9798` | — |
| `object_port` | `7505` | — |
| `collision_port` | `9092` | — |
| `cmd_ip` | `127.0.0.1` | — |
| `cmd_port` | `9093` | — |
| `max_steer_deg` | `40.0` | — |
| `timestamp_mode` | `receive` | — |

## Sensor bridge component

[소스](../../src/asmc_bridge/launch/morai_sensor_bridge.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `runtime_profile` | `morai_udp` | — |
| `transport_profile` | `local` | — |
| `stream_profile` | `competition` | — |
| `lidar_port` | `9299` | — |
| `lidar_topic` | `/velodyne_points` | — |
| `lidar_host_port` | `0` | — |
| `enable_instance_lidar` | `$(eval arg('stream_profile') == 'development')` | — |
| `instance_lidar_port` | `9311` | — |
| `instance_lidar_topic` | `/velodyne_points_instance` | — |
| `timestamp_mode` | `receive` | — |
| `gps_port` | `9301` | — |
| `imu_port` | `9303` | — |
| `planner_forward_ip` | `127.0.0.1` | — |
| `planner_gps_port` | `9401` | — |
| `planner_imu_port` | `9403` | — |
| `gps_east_offset` | `302595.0` | — |
| `gps_north_offset` | `4124145.0` | — |
| `publish_map_tf` | `false` | — |
| `enable_lidar` | `true` | — |
| `enable_semantic_camera` | `$(eval arg('stream_profile') == 'development')` | — |
| `lidar_model` | `VLP-16` | — |
| `lidar_rotation_rate` | `15.0` | — |
| `lidar_frame_mode` | `azimuth` | — |
| `publish_lidar_tf` | `true` | — |
| `lidar_x` | `0.80` | — |
| `lidar_y` | `0.0` | — |
| `lidar_z` | `1.45` | — |
| `lidar_roll` | `0.0` | — |
| `lidar_pitch` | `0.0` | — |
| `lidar_yaw` | `0.0` | — |

## Bridge state component

[소스](../../src/asmc_bridge/launch/vehicle_state.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| — | 최상위 공개 인자 없음 | 구성은 소스/포함 launch 확인 |

## Camera

[소스](../../src/asmc_camera/launch/camera.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `mode` | `active` | off &#124; standby &#124; active; default continuous inference |
| `models` | `all` | all or comma-separated objects,lane,traffic_light |
| `detector` | `e8` | e8 &#124; pretrained; restart to change weights |
| `confidence` | `0.40` | — |
| `nms_iou` | `0.50` | — |

## Visualization

[소스](../../src/asmc_camera/launch/view.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `models` | `all` | — |

유효성 검사 전용 내부 인자는 일반 사용자 옵션에서 제외합니다.

## BDD Large experimental inference

[소스](../../src/asmc_camera/launch/bdd_lane.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `resolution` | `runtime` | runtime: front960/sides640 &#124; official: all384x640 |

유효성 검사 전용 내부 인자는 일반 사용자 옵션에서 제외합니다.

## BDD Large experimental visualization

[소스](../../src/asmc_camera/launch/bdd_lane_view.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| — | 최상위 공개 인자 없음 | 구성은 소스/포함 launch 확인 |

## HD Map

[소스](../../src/asmc_hd_map/launch/competition_map.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `rviz` | `false` | — |
| `data_root` | `/data` | — |
| `debug_republish_hz` | `0.0` | — |
| `debug_marker_detail` | `lite` | — |
| `baseline_path` | `(빈 문자열)` | — |
| `publish_local` | `false` | — |
| `local_pose_source` | `gps_imu` | — |
| `local_ego_topic` | `/Ego_topic` | — |
| `local_pose_topic` | `/localization/pose` | — |
| `local_gps_topic` | `/gps` | — |
| `local_imu_topic` | `/imu` | — |
| `local_extent_m` | `120.0` | — |
| `local_crop_mode` | `ego` | — |
| `local_front_m` | `90.0` | — |
| `local_rear_m` | `30.0` | — |
| `local_left_m` | `30.0` | — |
| `local_right_m` | `30.0` | — |
| `local_publish_hz` | `20.0` | — |
| `follow_ego` | `true` | — |

## HD Map RViz

[소스](../../src/asmc_hd_map/launch/competition_map_debug.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `rviz` | `true` | — |
| `data_root` | `/data` | — |
| `debug_marker_detail` | `lite` | — |
| `debug_republish_hz` | `0.0` | — |
| `map_path` | `$(find asmc_hd_map)/reports/kcity2025_route_curation_20260917/route_curated_candidate_map.json` | — |
| `metadata_path` | `$(find asmc_hd_map)/config/kcity2025_competition_metadata.yaml` | — |
| `drivable_product_path` | `$(find asmc_hd_map)/config/kcity2025_compact_drivable_map.json` | — |
| `baseline_path` | `(빈 문자열)` | — |
| `publish_local` | `false` | — |
| `local_pose_source` | `gps_imu` | — |
| `local_ego_topic` | `/Ego_topic` | — |
| `local_pose_topic` | `/localization/pose` | — |
| `local_gps_topic` | `/gps` | — |
| `local_imu_topic` | `/imu` | — |
| `local_extent_m` | `120.0` | — |
| `local_crop_mode` | `ego` | — |
| `local_front_m` | `90.0` | — |
| `local_rear_m` | `30.0` | — |
| `local_left_m` | `30.0` | — |
| `local_right_m` | `30.0` | — |
| `local_publish_hz` | `20.0` | — |
| `local_pose_stale_timeout_sec` | `0.5` | — |
| `publish_ego_markers` | `$(arg rviz)` | — |
| `publish_ego_tf` | `$(arg rviz)` | — |
| `ego_frame_id` | `base_link` | — |
| `follow_ego` | `true` | — |

## Main LiDAR/fusion

[소스](../../src/lidar+sensorfusion/src/Main/main.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `camera_class_profile` | `mission16_v1` | — |
| `enable_common_geometry` | `false` | — |
| `lidar_level_roll` | `false` | — |
| `lidar_mount_config` | `$(find asmc_lidar)/config/autonav_v3.yaml` | — |
| `fusion_camera_config` | `$(find asmc_camera)/config/cameras_v3.yaml` | — |
| `map_altitude_origin_m` | `0.0` | — |
| `fusion_yolo_policy` | `unique` | unique &#124; bounded_reuse |
| `camera_objects_conf_thres` | `0.0` | Optional stricter adapter cutoff. Camera already filters confidence; 0 passes its accepted candidates without a hidden 0.7 gate. |
| `lidar_port` | `9299` | — |
| `start_velodyne_driver` | `false` | — |
| `rviz` | `false` | — |
| `enable_sensor_fusion` | `true` | — |
| `fusion_max_time_offset_sec` | `0.05` | — |
| `fusion_sync_wait_timeout_sec` | `0.20` | — |
| `fusion_trace_path` | `(빈 문자열)` | — |
| `roi_min_x` | `-30.0` | — |
| `roi_max_x` | `50.0` | — |
| `roi_min_y` | `-20.0` | — |
| `roi_max_y` | `20.0` | — |
| `motion_debug_topic` | `/lidar/motion_debug` | — |
| `drivable_filter_enabled` | `true` | — |
| `drivable_margin_m` | `0.0` | — |
| `drivable_ratio_threshold` | `0.15` | — |
| `drivable_supersample` | `5` | — |
| `planning_minimum_obstacle_height_m` | `0.10` | — |
| `planning_confirmation_frames` | `3` | — |
| `planning_release_frames` | `5` | — |
| `known_static_filter_enabled` | `true` | — |
| `known_static_catalog` | `$(find Main)/known_static_objects.yaml` | — |
| `known_static_center_tolerance_m` | `2.5` | — |
| `known_static_application_region_enabled` | `true` | — |
| `known_static_application_min_x` | `-35.0` | — |
| `known_static_application_max_x` | `15.0` | — |
| `known_static_application_min_y` | `108.0` | — |
| `known_static_application_max_y` | `180.0` | — |
| `known_static_guardrail_extension_enabled` | `true` | — |
| `known_static_guardrail_extension_min_x` | `-107.0` | — |
| `known_static_guardrail_extension_max_x` | `15.0` | — |
| `known_static_guardrail_extension_min_y` | `175.0` | — |
| `known_static_guardrail_extension_max_y` | `345.0` | — |
| `static_structure_exclusion_enabled` | `true` | — |
| `static_structure_exclusion_config` | `$(find Main)/../../../planning_inputs/config/static_structure_exclusions.yaml` | — |
| `static_structure_minimum_height_m` | `2.5` | — |
| `static_structure_minimum_bottom_z_m` | `2.0` | — |
| `ground_distance_threshold_near_m` | `0.30` | — |
| `ground_distance_threshold_far_m` | `0.40` | — |
| `ground_threshold_far_range_m` | `50.0` | — |
| `ground_max_tilt_deg` | `25.0` | — |
| `ground_seed_min_z_m` | `-2.0` | — |
| `ground_seed_max_z_m` | `-0.65` | — |
| `ground_ransac_maximum_iterations` | `80` | — |
| `ground_minimum_seed_points` | `30` | — |
| `ground_fallback_frames` | `2` | — |
| `gps_sanity_maximum_speed_mps` | `80.0` | — |
| `gps_sanity_base_jump_allowance_m` | `3.0` | — |
| `gps_sanity_reanchor_after_s` | `2.0` | — |
| `lidar_pose_gps_max_age_s` | `0.30` | — |
| `imu_yaw_is_true_enu` | `true` | — |
| `lidar_pose_imu_max_age_s` | `0.12` | — |
| `lidar_pose_can_max_age_s` | `0.12` | — |
| `lidar_pose_wait_timeout_s` | `0.05` | — |
| `lidar_pose_maximum_speed_mps` | `35.0` | — |
| `benchmark_lidar_pipeline` | `false` | — |
| `benchmark_warmup_frames` | `30` | — |
| `benchmark_sample_frames` | `200` | — |
| `lidar_cut_angle` | `0.0` | — |

## Integrated planning/control

[소스](../../src/asmc_planner/launch/autonomy_no_bridges_v2.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `objects_source` | `lidar` | Supported source: lidar |
| `state_source` | `gps_imu` | gps_imu or external |
| `state_topic` | `/planning/vehicle/state` | — |
| `canonical_objects_topic` | `/planning/objects` | — |
| `lidar_input_topic` | `/last_lidar` | — |
| `lidar_source_policy` | `validated_lidar` | — |
| `avoid_path_margin_m` | `0.4` | — |
| `expected_config_sha256` | `2d17cf63adbf2ba049d70656233ee2f34e9a4bfb43373ee76b3371aab1e4dc2a` | — |
| `calibration_file` | `$(find asmc_camera)/config/cameras_v3.yaml` | — |

## Planner component

[소스](../../src/asmc_planner/launch/planner_v2.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `state_topic` | `/vehicle/state` | — |
| `behavior_objects_topic` | `/Object_topic` | — |
| `objects_input_type` | `morai` | — |
| `planning_objects_topic` | `/planning/objects` | — |
| `avoid_path_margin_m` | `0.4` | — |

## MPC component

[소스](../../src/asmc_controller/launch/controller.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `state_topic` | `/vehicle/state` | — |

## Planning state component

[소스](../../src/planning_inputs/launch/vehicle_state.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `state_topic` | `/planning/vehicle/state` | — |

## Planning object component

[소스](../../src/planning_inputs/launch/objects.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `input_topic` | `/last_lidar` | — |
| `source_policy` | `validated_lidar` | — |
| `motion_debug_topic` | `/lidar/motion_debug` | — |
| `output_topic` | `/planning/objects` | — |
| `state_topic` | `/planning/vehicle/state` | — |

## Dashboard

[소스](../../src/asmc_planner/launch/dashboard.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `state_topic` | `/vehicle/state` | — |
| `dashboard_port` | `8088` | — |
| `open_dashboard` | `false` | — |
| `dashboard_lidar_objects_topic` | `/perception/objects_lidar_test` | — |
| `dashboard_lidar_motion_debug_topic` | `/lidar/motion_debug` | — |

## Read-only family monitor

[소스](../../src/jamming_lane_boundary_adapter/launch/shadow_boundary_family_candidate.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `config` | `$(find lane_boundary_adapter)/config/shadow_boundary_family_consumer.yaml` | — |
| `rviz` | `true` | — |
| `rviz_config` | `$(find lane_boundary_adapter)/rviz/shadow_boundary_family_candidate.rviz` | — |

## Jamming control/preview

[소스](../../src/jamming_lane_boundary_adapter/launch/camera_bbox_lane_change.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `clipped_tracking_speed_kph` | `5.0` | — |
| `contact_band_height_px` | `8.0` | — |
| `contact_row_tolerance_px` | `8.0` | — |
| `min_contact_overlap_ratio` | `0.10` | — |
| `min_contact_support_rows` | `3` | — |
| `hood_crop_bottom_ratio` | `0.0` | — |
| `ego_mask_enabled` | `$(eval float(arg('hood_crop_bottom_ratio')) == 0)` | — |
| `ego_mask_config` | `$(find asmc_camera)/config/camera/yolo_hood_mask_v1.yaml` | — |
| `enable_control` | `false` | — |
| `lookahead_distance` | `8.0` | — |
| `target_speed_kph` | `20.0` | — |
| `inferred_speed_kph` | `20.0` | — |
| `wheelbase` | `3.0` | — |
| `steering_curvature_gain` | `0.844` | — |
| `max_steering_deg` | `40.0` | — |
| `max_steering_rate` | `1.0` | — |
| `max_lateral_accel` | `1.0` | — |
| `control_rate` | `20.0` | — |
| `source_timeout` | `0.5` | — |
| `receive_timeout` | `0.35` | — |
| `future_tolerance` | `0.05` | — |
| `lane_index` | `-1` | — |
| `lane_confirm_frames` | `3` | — |
| `lane_selection_distance` | `4.0` | — |
| `lane_boundary_margin` | `0.15` | — |
| `allow_inferred` | `true` | — |
| `min_confidence` | `0.25` | — |
| `join_tolerance` | `1e-4` | — |
| `stop_brake` | `1.0` | — |
| `shutdown_stop_duration` | `0.3` | — |
| `marker_z` | `-0.30` | — |
| `epsilon` | `1e-6` | — |
| `max_steering_dt` | `0.1` | — |
| `consumer_x_min` | `0.0` | — |
| `consumer_x_max` | `15.0` | — |
| `min_segment_length` | `0.25` | — |
| `lane_width_min` | `2.4` | — |
| `lane_width_max` | `4.6` | — |
| `max_width_variation` | `0.8` | — |
| `inferred_confidence_scale` | `0.60` | — |
| `expected_config_sha256` | `2d17cf63adbf2ba049d70656233ee2f34e9a4bfb43373ee76b3371aab1e4dc2a` | — |
| `input_topic` | `/perception/lane/boundary_family` | — |
| `control_topic` | `/ctrl_cmd/jamming` | — |
| `preview_topic` | `/planning/camera_bbox_lane_change/control_preview` | — |
| `target_topic` | `/planning/camera_bbox_lane_change/target` | — |
| `markers_topic` | `/planning/camera_bbox_lane_change/markers` | — |
| `diagnostics_topic` | `/planning/camera_bbox_lane_change/diagnostics` | — |
| `emergency_stop_topic` | `/planning/camera_bbox_lane_change/emergency_stop` | — |
| `bbox_topic` | `/planning/camera_bbox_lane/result` | — |
| `reference_lane_topic` | `/planning/camera_bbox_lane_change/reference_lane` | — |
| `far_obstacle_speed_kph` | `5.0` | — |
| `bbox_source_timeout` | `0.5` | — |
| `bbox_receive_timeout` | `0.5` | — |
| `max_sensor_skew` | `0.4` | — |
| `obstacle_confirm_frames` | `2` | — |
| `adjacent_clear_frames` | `3` | — |
| `ignore_bbox_during_lane_change` | `true` | — |
| `one_lane_change_only` | `false` | — |
| `obstacle_lateral_margin` | `0.30` | — |
| `obstacle_longitudinal_margin` | `0.5` | — |
| `vehicle_front` | `3.845` | — |
| `vehicle_rear` | `0.790` | — |
| `vehicle_width` | `1.892` | — |
| `planning_start_x` | `1.0` | — |
| `path_horizon` | `11.0` | — |
| `path_sample_step` | `0.25` | — |
| `max_lateral_step` | `2.4` | — |
| `lane_change_length` | `10.0` | — |
| `lane_change_speed_kph` | `15.0` | — |
| `completion_lateral_error` | `0.30` | — |
| `completion_heading_deg` | `8.0` | — |
| `completion_confirm_frames` | `3` | — |
| `lane_change_pause_timeout` | `30.0` | — |
| `lane_change_timeout` | `15.0` | — |
| `lane_change_cooldown` | `4.0` | — |
| `use_rviz` | `true` | — |
| `bbox_consumer_x_max` | `20.0` | — |
| `start_classifier` | `true` | — |
| `projection_context_required` | `true` | — |
| `projection_manifest` | `$(find asmc_camera)/config/bev/projection_context_v1.yaml` | — |
| `calibration_file` | `$(find asmc_camera)/config/cameras_v3.yaml` | — |

## Capture trigger

[소스](../../src/data_collection/launch/capture_mode_collector.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `capture_hz` | `1.0` | — |
| `max_captures` | `0` | — |
| `run_id` | `(빈 문자열)` | — |
| `data_root` | `(빈 문자열)` | — |
| `backend` | `auto` | — |
| `grpc_address` | `127.0.0.1` | — |
| `grpc_port` | `7789` | — |
| `save_topic` | `/SaveSensorData` | — |
| `morai_file_dir` | `(빈 문자열)` | — |
| `only_when_moving` | `false` | — |
| `min_speed_mps` | `0.2` | — |
| `require_ego` | `false` | — |
| `dry_run` | `false` | — |

## Capture route pilot

[소스](../../src/data_collection/launch/capture_route_collection.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `controller` | `mpc` | — |
| `actuator_transport` | `grpc` | — |
| `inspection_only` | `false` | — |
| `target_kph` | `20` | — |
| `max_captures` | `30` | — |
| `max_seconds` | `1800` | — |
| `run_id` | `(빈 문자열)` | — |
| `parent_run` | `(빈 문자열)` | — |
| `layout_id` | `(빈 문자열)` | — |
| `spawn_ledger` | `(빈 문자열)` | — |
| `data_root` | `/data` | — |
| `grpc_address` | `host.docker.internal` | — |
| `route` | `$(find data_collection)/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt` | — |

## Bag recorder

[소스](../../src/data_collection/launch/bag_recorder.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `profile` | `full_raw` | — |
| `scenario` | `(빈 문자열)` | — |
| `controller` | `(빈 문자열)` | — |
| `run_id` | `(빈 문자열)` | — |
| `rosbag_root` | `(빈 문자열)` | — |
| `map_slug` | `kcity2025` | — |
| `weather` | `(빈 문자열)` | — |
| `sim_hour` | `(빈 문자열)` | — |
| `grpc_address` | `127.0.0.1` | — |
| `grpc_port` | `7789` | — |
| `grpc_timeout_sec` | `1.0` | — |
| `grpc_connect_timeout_sec` | `2.0` | — |
| `environment_poll_sec` | `2.0` | — |
| `topic_timeout_sec` | `10.0` | — |
| `parent_raw_run_id` | `(빈 문자열)` | — |
| `model_id` | `(빈 문자열)` | — |
| `checkpoint` | `(빈 문자열)` | — |
| `min_free_gb` | `20.0` | — |
| `split_gb` | `4.0` | — |
| `buffer_mb` | `1024` | — |

## Bag replay

[소스](../../src/data_collection/launch/bag_replay.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `bag_path` | `필수` | — |
| `required_profile` | `camera_raw` | — |
| `rate` | `1.0` | — |
| `allow_privileged_gt` | `false` | — |
| `wait_for_start` | `true` | — |
| `post_roll_sec` | `2.0` | — |

## Scenario perception composition

[소스](../../tools/scen_runner/launch/perception.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| — | 최상위 공개 인자 없음 | 구성은 소스/포함 launch 확인 |

## Scenario planning composition

[소스](../../tools/scen_runner/launch/planning.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| — | 최상위 공개 인자 없음 | 구성은 소스/포함 launch 확인 |
