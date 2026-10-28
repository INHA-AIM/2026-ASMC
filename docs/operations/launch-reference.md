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
| `invert_steer` | `false` | Transport steering sign; AIM06 R1.H3 verified with true, team default retained |
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
| `invert_steer` | `false` | Preserve team default; current AIM06 R1.H3 requires true, verified against vehicle yaw |
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
| `detector` | `e3` | e3 &#124; e8 &#124; pretrained; restart to change weights |
| `confidence` | `0.40` | — |
| `nms_iou` | `0.50` | — |

## Visualization

[소스](../../src/asmc_camera/launch/view.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `models` | `all` | — |
| `detector` | `e3` | e3 &#124; e8 &#124; pretrained |

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

## LiDAR canonical runtime

[소스](../../src/asmc_lidar/launch/lidar.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `namespace` | `/perception/lidar` | Output namespace; inputs keep their canonical paths |
| `pipeline_config` | `$(find asmc_lidar)/config/lidar_pipeline.yaml` | Processing configuration YAML |
| `lidar_level_roll` | `(빈 문자열)` | — |
| `fusion` | `true` | Use three camera views for classification; false runs LiDAR alone |
| `rviz` | `false` | Open RViz and display this pipeline's published objects and ego box |
| `camera_calibration_config` | `(빈 문자열)` | — |
| `experimental_tunnel_filter` | `(빈 문자열)` | — |
| `publish_tf` | `false` | — |
| `skip_lidar_in_jamming` | `false` | — |
| `tf_child_frame` | `lidar_pipeline/lidar_leveled` | — |
| `debug` | `false` | Publish sampled stages for view.launch; no algorithm changes |
| `debug_rate_hz` | `5.0` | — |
| `debug_max_points` | `20000` | — |
| `points_topic` | `/velodyne_points` | — |
| `imu_topic` | `/imu` | — |
| `can_topic` | `/Ego_topic` | — |
| `gps_topic` | `/gps` | — |
| `camera_observations_topic` | `/perception/camera/objects/observations` | — |
| `camera_status_topic` | `/perception/camera/objects/status` | — |
| `objects_local_topic` | `objects_local` | — |
| `objects_map_topic` | `objects_map` | — |
| `observations_local_topic` | `observations_local` | — |
| `observations_map_topic` | `observations_map` | — |
| `tracks_local_topic` | `tracks_local` | — |
| `tracks_map_topic` | `tracks_map` | — |
| `planning_observations_topic` | `planning_observations_map` | — |
| `diagnostics_topic` | `diagnostics` | — |
| `trace_topic` | `trace` | — |

## Integrated planning/control

[소스](../../src/asmc_planner/launch/autonomy_no_bridges_v2.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `highway_merge` | `true` | — |
| `dashboard` | `true` | — |
| `dashboard_port` | `8088` | — |
| `open_dashboard` | `false` | — |
| `state_source` | `competition` | competition, gps_imu or external |
| `state_topic` | `/planning/vehicle/state` | — |
| `canonical_objects_topic` | `/planning/objects` | — |
| `lidar_input_topic` | `/perception/lidar/planning_observations_map` | — |
| `avoid_path_margin_m` | `0.4` | — |
| `expected_config_sha256` | `2d17cf63adbf2ba049d70656233ee2f34e9a4bfb43373ee76b3371aab1e4dc2a` | — |
| `calibration_file` | `$(find asmc_camera)/config/cameras_v3.yaml` | — |
| `rviz` | `false` | 재밍 카메라 차선변경 RViz(BBOX·차선·경로) 표시 |
| `jamming_line_edge_expand` | `0.30` | [m] 기준선 1·5를 바깥으로 확장(최대 0.5) |
| `jamming_avoid_line_extra` | `1.0` | 통과 가능한 선에서 바깥으로 더 미는 양(선 번호, 최대 2.0) |
| `jamming_avoid_clearance` | `0.05` | [m] 차체 측면-장애물 최소 간격 |
| `jamming_obstacle_lateral_margin` | `0.05` | [m] 차체 좌우 여유(도로 경계 검사 포함) |
| `jamming_avoid_line_overrun` | `1.5` | [m] 벽 미확인 쪽에서 차량 중심이 끝 차선을 넘을 수 있는 거리 |
| `jamming_use_lidar_obstacles` | `true` | — |
| `jamming_creep_speed_kph` | `3.0` | — |
| `jamming_creep_clear_distance` | `0.0` | [m] 0보다 크면 앞 범퍼부터 이 거리 안 경로 장애물에 정지 |
| `shadow_zone_start_m` | `1847.744798966` | — |

## Planner component

[소스](../../src/asmc_planner/launch/planner_v2.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `highway_merge` | `false` | — |
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
| `input_topic` | `/perception/lidar/planning_observations_map` | — |
| `output_topic` | `/planning/objects` | — |

## Dashboard

[소스](../../src/asmc_planner/launch/dashboard.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `state_topic` | `/planning/vehicle/state` | — |
| `dashboard_port` | `8088` | — |
| `open_dashboard` | `false` | — |
| `dashboard_lidar_objects_topic` | `/planning/objects` | — |
| `dashboard_lidar_motion_debug_topic` | `(빈 문자열)` | — |

## LiDAR read-only web viewer

[소스](../../src/asmc_lidar/launch/view.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `namespace` | `/perception/lidar` | — |
| `open_browser` | `false` | — |
| `dashboard_port` | `8091` | — |
| `bind_address` | `127.0.0.1` | — |
| `pipeline_config` | `$(find asmc_lidar)/config/lidar_pipeline.yaml` | — |
| `max_clients` | `8` | — |
| `http_header_timeout_s` | `5.0` | — |
| `http_request_timeout_s` | `20.0` | — |

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
| `bbox_conf_thres` | `0.7` | — |
| `reference_lane_topic` | `/planning/camera_bbox_lane_change/reference_lane` | — |
| `far_obstacle_speed_kph` | `5.0` | — |
| `bbox_source_timeout` | `0.5` | — |
| `bbox_receive_timeout` | `0.5` | — |
| `person_stop` | `true` | — |
| `person_stop_sec` | `3.0` | — |
| `person_ignore_sec` | `4.0` | — |
| `max_sensor_skew` | `0.4` | — |
| `obstacle_confirm_frames` | `2` | — |
| `adjacent_clear_frames` | `3` | — |
| `ignore_bbox_during_lane_change` | `true` | — |
| `one_lane_change_only` | `false` | — |
| `return_to_entry_lane` | `false` | — |
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
| `straddle_avoidance` | `true` | — |
| `ego_topic` | `/Ego_topic` | — |
| `ego_speed_timeout` | `0.3` | — |
| `avoid_clearance` | `0.5` | — |
| `avoid_speed_kph` | `8.0` | — |
| `avoid_obstacle_length` | `6.0` | — |
| `avoid_pass_margin` | `2.0` | — |
| `avoid_lane_tie` | `0.10` | — |
| `avoid_shift_length` | `5.0` | — |
| `avoid_lookahead_distance` | `5.0` | — |
| `avoid_line_straddle` | `true` | — |
| `avoid_max_offset` | `3.5` | — |
| `avoid_max_shoulder` | `2.0` | — |
| `avoid_wall_margin` | `0.3` | — |
| `avoid_line_overrun` | `1.0` | — |
| `avoid_line_step` | `0.25` | — |
| `creep_speed_kph` | `0.0` | 인식·판단 정지 대신 기어가는 속도. 0이면 정지 |
| `creep_clear_distance` | `2.0` | [m] 기어가기 전 앞 범퍼부터 비어 있어야 하는 경로 거리 |
| `line_obstacle_reach` | `20.0` | [m] 선 선택에 쓰는 장애물 최대 전방 거리. 0 이하면 제한 없음 |
| `avoid_line_extra` | `0.0` | — |
| `line_edge_expand` | `0.0` | — |
| `lane_hold_distance` | `15.0` | — |
| `lane_hold_time` | `8.0` | — |
| `imu_topic` | `/imu` | — |
| `pedal_speed_control` | `true` | — |
| `speed_kp` | `0.3` | — |
| `speed_ki` | `0.1` | — |
| `speed_integral_limit` | `3.0` | — |
| `max_accel_pedal` | `0.5` | — |
| `max_brake_pedal` | `0.5` | — |
| `avoid_reconsider_distance` | `0.5` | — |
| `avoid_plan_margin` | `0.05` | — |
| `use_lidar_obstacles` | `false` | — |
| `lidar_objects_topic` | `/perception/lidar/objects_local` | — |
| `lidar_timeout` | `0.6` | — |
| `lidar_max_x` | `30.0` | — |
| `lidar_min_height` | `0.15` | — |
| `lidar_max_length` | `20.0` | — |
| `lidar_min_points` | `5` | — |
| `lidar_road_min_overlap` | `0.20` | — |
| `lidar_lane_min_overlap` | `0.10` | — |
| `lidar_structure_hold` | `1.5` | — |
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

## Controlled reference handover trial

[소스](../../src/asmc_planner/launch/reference_handover_trial.launch)

| 인자 | 기본값 / 필수 | 소스 설명 |
|---|---|---|
| `enable_control` | `false` | — |
| `data_root` | `/data` | — |
| `state_topic` | `/planning/vehicle/state` | — |
| `objects_topic` | `/planning/objects` | — |
| `config` | `$(find asmc_planner)/config/reference_handover_trial.yaml` | — |
