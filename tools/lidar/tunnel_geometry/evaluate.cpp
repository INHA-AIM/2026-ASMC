// Isolated L1-B4 evaluator. No subscriptions, publishers, ROS master or control.
#include "evaluation_helpers.hpp"
#include "policy.hpp"
#include "lineage.hpp"

using namespace tunnel_filter;

YAML::Node Serialize(const FrameResult& r, const std::vector<std::uint8_t>& protect,
                     const std::vector<std::uint8_t>& ambiguous, bool available, int boxes,
                     double protection_ms, double policy_ms, double recluster_ms) {
    YAML::Node m;
    m["protection_available"] = available; m["protection_boxes"] = boxes;
    m["protected_points"] = r.protected_points;
    m["ambiguous_points"] = std::count(ambiguous.begin(), ambiguous.end(), 1);
    std::vector<int> protected_ids;
    for (std::size_t i = 0; i < protect.size(); ++i) if (protect[i] != kProtectNone) protected_ids.push_back(i);
    m["protected_indices"] = Sequence(protected_ids);
    m["ground_removed"] = r.ground_removed; m["wall_removed"] = r.wall_removed;
    m["ceiling_removed"] = r.ceiling_removed; m["kept_points"] = r.kept_points;
    m["clusters"] = r.clusters.size();
    m["final_objects"] = std::count_if(r.objects.begin(), r.objects.end(), [](const auto& o) { return !o.flags.removed; });
    m["objects_removed_wall"] = r.objects_removed_wall;
    m["ground_source"] = static_cast<int>(r.ground.source);
    m["wall_left_removal"] = r.walls[0].removal_allowed;
    m["wall_right_removal"] = r.walls[1].removal_allowed;
    m["wall_ambiguous_skipped"] = r.walls[0].ambiguous_skipped + r.walls[1].ambiguous_skipped;
    m["labels"] = Sequence(std::vector<int>(r.labels.begin(), r.labels.end()));
    m["kept_indices"] = Sequence(r.kept_indices); m["cluster_id"] = Sequence(r.cluster_id);
    m["cluster_point_indices"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& c : r.clusters) m["cluster_point_indices"].push_back(Sequence(c.indices));
    m["leveling"] = Sequence(std::vector<float>(r.leveling.begin(), r.leveling.end()));
    m["leveled_xyzi_fnv1a64"] = CloudHash(*r.leveled);
    m["objects"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& o : r.objects) {
        YAML::Node a;
        a["center"] = Sequence(std::vector<float>{o.obb.cx, o.obb.cy, o.obb.cz});
        a["dimensions"] = Sequence(std::vector<float>{o.obb.length, o.obb.width, o.obb.z_max - o.obb.z_min});
        a["yaw"] = o.obb.yaw; a["removed"] = o.flags.removed; a["protected"] = o.is_protected;
        a["point_count"] = o.point_count; a["protect_class"] = int(o.protect_class);
        a["corners"] = YAML::Node(YAML::NodeType::Sequence);
        for (const auto& c : o.obb.corners) a["corners"].push_back(Sequence(std::vector<float>{c.x, c.y, c.z}));
        a["flags"]["wall"] = o.flags.wall; a["flags"]["ceiling"] = o.flags.ceiling;
        a["flags"]["wall_attached"] = o.flags.wall_attached;
        a["flags"]["ground_residual"] = o.flags.ground_residual;
        m["objects"].push_back(a);
    }
    m["timing_ms"]["protection"] = protection_ms;
    m["timing_ms"]["reference_pipeline"] = r.timings.total_ms;
    m["timing_ms"]["shared_pre_wall"] = r.timings.level_ms + r.timings.ceiling_ms +
        r.timings.ground_fit_ms + r.timings.ground_remove_ms + r.timings.wall_fit_ms;
    m["timing_ms"]["pipeline_estimate"] = recluster_ms == 0 ? r.timings.total_ms :
        m["timing_ms"]["shared_pre_wall"].as<double>() + policy_ms + recluster_ms;
    m["timing_ms"]["policy"] = policy_ms; m["timing_ms"]["recluster"] = recluster_ms;
    m["timing_ms"]["cluster"] = r.timings.cluster_ms;
    m["timing_ms"]["obb"] = r.timings.obb_ms; m["timing_ms"]["obb_filter"] = r.timings.obb_filter_ms;
    return m;
}

int main(int argc, char** argv) try {
    if (argc != 8) throw std::runtime_error("usage: tunnel_policy_eval SNAPSHOTS SETTING PARAMS CONFIG OUTPUT CLOUD_DIR DECISION_KEY");
    const fs::path input = argv[1], clouds = argv[6];
    if (fs::exists(argv[5]) || fs::exists(clouds)) throw std::runtime_error("output exists");
    fs::create_directories(clouds);
    const auto setting = YAML::LoadFile(argv[2]); resolved_params = YAML::LoadFile(argv[3]);
    const auto config = YAML::LoadFile(argv[4])["geometry"];
    l1b4::Params candidate;
    candidate.band_min = config["wall_band_min_m"].as<double>();
    candidate.band_max = config["wall_band_max_m"].as<double>();
    candidate.mad_scale = config["wall_band_mad_scale"].as<double>();
    candidate.mad_normal_scale = config["mad_normal_scale"].as<double>();
    candidate.connection_radius = config["connection_radius_m"].as<double>();
    candidate.expansion_max = config["local_expansion_max_m"].as<double>();
    const auto v = setting["Vehicle"], l = setting["LiDAR"]["Filter"];
    st_VehicleSpec.f32_Width = v["Width"].as<float>(); st_VehicleSpec.f32_WheelBase = v["WheelBase"].as<float>();
    st_VehicleSpec.f32_FrontOverhang = v["FrontOverhang"].as<float>();
    st_VehicleSpec.f32_RearOverhang = v["RearOverhang"].as<float>();
    st_VehicleSpec.st_LiDARPos = {v["LiDAR"]["x"].as<float>(), v["LiDAR"]["y"].as<float>(), v["LiDAR"]["z"].as<float>()};
    st_VehicleSpec.st_LiDARRotationDeg = {v["LiDAR"]["roll"].as<float>(), v["LiDAR"]["pitch"].as<float>(), v["LiDAR"]["yaw"].as<float>()};
    LiDAR lidar{};
    lidar.st_Range.f32_Min = l["Range"]["Min"].as<float>(); lidar.st_Range.f32_Max = l["Range"]["Max"].as<float>();
    lidar.st_ROI.f32_MinX = l["ROI"]["MinX"].as<float>(); lidar.st_ROI.f32_MaxX = l["ROI"]["MaxX"].as<float>();
    lidar.st_ROI.f32_MinY = l["ROI"]["MinY"].as<float>(); lidar.st_ROI.f32_MaxY = l["ROI"]["MaxY"].as<float>();
    lidar.st_ROI.f32_MinZ = l["ROI"]["MinZ"].as<float>(); lidar.st_ROI.f32_MaxZ = l["ROI"]["MaxZ"].as<float>();
    OfflineParam("roi_min_x", lidar.st_ROI.f32_MinX, lidar.st_ROI.f32_MinX);
    OfflineParam("roi_max_x", lidar.st_ROI.f32_MaxX, lidar.st_ROI.f32_MaxX);
    OfflineParam("roi_min_y", lidar.st_ROI.f32_MinY, lidar.st_ROI.f32_MinY);
    OfflineParam("roi_max_y", lidar.st_ROI.f32_MaxY, lidar.st_ROI.f32_MaxY);
    pinned_camera::OfflineNode node;
    const auto params = pinned_camera::LoadParams(node, tunnel_filter::Params());
    CAMERA camera{};
    camera.params.world_points_vector = setting["Camera"]["BEV"]["WorldPoints"].as<std::vector<float>>();
    camera.params.moving_average_filter_size = setting["Camera"]["BEV"]["MAF"].as<int>();
    InitProjectionMatrix(camera, setting);
    pinned_camera::ExperimentState protection; protection.projection = camera.params.sensor_fusion_projection_matrix;
    node.param("tunnel_protect_enable", protection.protection.enable, protection.protection.enable);
    node.param("tunnel_protect_max_dt_s", protection.protection.max_dt_s, protection.protection.max_dt_s);
    for (auto item : std::initializer_list<std::pair<const char*, float*>>{
        {"tunnel_protect_box_margin_ratio", &protection.protection.box_margin_ratio},
        {"tunnel_protect_box_margin_px", &protection.protection.box_margin_px},
        {"tunnel_protect_depth_vehicle_m", &protection.protection.depth_vehicle_m},
        {"tunnel_protect_depth_person_m", &protection.protection.depth_person_m},
        {"tunnel_protect_depth_front_margin_m", &protection.protection.depth_front_margin_m},
        {"tunnel_protect_depth_ref_percentile", &protection.protection.depth_ref_percentile},
        {"tunnel_protect_ambiguous_max_range_m", &protection.protection.ambiguous_max_range_m}})
        pinned_camera::ParamFloat(node, item.first, item.second);
    node.param("tunnel_protect_min_box_points", protection.protection.min_box_points, protection.protection.min_box_points);
    TunnelPipeline pipeline(params, pinned_camera::MakeTrackingObbFunction());
    std::string phase, line; bool anchor = false; float previous_yaw = 0, previous_speed = 0;
    std::int64_t previous_stamp = 0;
    std::ifstream in(input); std::ofstream out(argv[5]);
    while (std::getline(in, line)) {
        const auto row = YAML::Load(line); YAML::Node result;
        result["input_id"] = row["input_id"].as<std::string>(); result["scene"] = row["scene"].as<std::string>();
        result["measurement"] = row["measurement"].as<bool>();
        const auto current_phase = row["phase"].as<std::string>();
        if (current_phase != phase) { pipeline.Reset(); anchor = false; phase = current_phase; }
        result["phase"] = phase;
        if (!row["geometry_valid"].as<bool>()) {
            result["skipped_reason"] = row["geometry_reject_reason"].as<std::string>();
            anchor = false; pipeline.Reset(); Json(out, result); out << '\n'; continue;
        }
        auto raw = ReadCloud(input.parent_path() / row["cloud_path"].as<std::string>());
        Cloud::Ptr filtered(new Cloud), voxel(new Cloud);
        const auto preprocessing_start = Clock::now();
        if (row["already_preprocessed"].as<bool>(false)) *voxel = *raw;
        else {
            Roi(raw, filtered, lidar, st_VehicleSpec);
            if (params.dynamic_roi_enable) {
                auto& points = filtered->points;
                points.erase(std::remove_if(points.begin(), points.end(), [&](const auto& p) {
                    return p.x < params.roi_min_x_m || p.x > params.roi_max_x_m || p.y < params.roi_min_y_m ||
                        p.y > params.roi_max_y_m || p.z < params.roi_min_z_m || p.z > params.roi_max_z_m;
                }), points.end());
                filtered->width = points.size(); filtered->height = 1;
            }
            Voxel(filtered, voxel);
        }
        result["preprocessing_ms"] = Ms(preprocessing_start);
        result["raw_points"] = raw->size(); result["voxel_points"] = voxel->size();
        const auto lineage_start = Clock::now();
        const auto lineage = l1b4::Trace(*raw, *filtered, *voxel, row["already_preprocessed"].as<bool>(false));
        result["lineage_errors"] = lineage.errors;
        result["raw_to_voxel_fnv1a64"] = Hash(lineage.raw_to_voxel.data(), lineage.raw_to_voxel.size() * sizeof(std::int32_t));
        {
            std::ofstream stream(clouds / (result["input_id"].as<std::string>() + ".indices.bin"), std::ios::binary);
            stream.write(reinterpret_cast<const char*>(lineage.raw_to_voxel.data()), lineage.raw_to_voxel.size() * sizeof(std::int32_t));
        }
        result["lineage_ms"] = Ms(lineage_start);
        result["frozen_voxel_fnv1a64"] = CloudHash(*voxel);
        const auto stamp = row["scan_effective_stamp_ns"].as<std::int64_t>();
        result["scan_stamp_ns"] = row["scan_stamp_ns"].as<std::int64_t>(); result["scan_effective_stamp_ns"] = stamp;
        const float roll = row["roll"].as<float>(), pitch = row["pitch"].as<float>();
        const float yaw = row["yaw"].as<float>(), speed = row["speed_mps"].as<float>();
        EgoMotion motion;
        if (anchor) {
            const float dt = (stamp - previous_stamp) / 1e9;
            const float dyaw = std::atan2(std::sin(yaw - previous_yaw), std::cos(yaw - previous_yaw));
            const float distance = .5f * (speed + previous_speed) * dt;
            if (dt > 0 && std::isfinite(dyaw) && std::isfinite(distance)) {
                motion.valid = true; motion.dt_s = dt; motion.dyaw_rad = dyaw;
                motion.dx_m = distance * std::cos(.5f * dyaw); motion.dy_m = distance * std::sin(.5f * dyaw);
            }
        }
        anchor = true; previous_stamp = stamp; previous_yaw = yaw; previous_speed = speed;
        const auto decision = row[argv[7]];
        const bool available = decision["selected_yolo_index"] && !decision["selected_yolo_index"].IsNull() &&
            decision["protection_enabled"].as<bool>(true);
        pinned_camera::YoloFrame yolo;
        if (available) for (const auto& b : decision["boxes"]) {
            BBox box; box.st_Min_pt.f32_X = b["xmin"].as<float>(); box.st_Min_pt.f32_Y = b["ymin"].as<float>();
            box.st_Max_pt.f32_X = b["xmax"].as<float>(); box.st_Max_pt.f32_Y = b["ymax"].as<float>();
            box.str_class = b["class"].as<std::string>(); box.str_raw_class = box.str_class; yolo.boxes.push_back(box);
        }
        std::vector<std::uint8_t> protect, ambiguous; int protected_boxes = 0;
        const auto protection_start = Clock::now();
        pinned_camera::ComputeProtection(protection, *voxel, available ? &yolo : nullptr,
                                          &protect, &ambiguous, &protected_boxes);
        const auto protection_ms = Ms(protection_start);
        const auto& reference_view = pipeline.Process(*voxel, roll, pitch, &protect, &ambiguous, available, &motion);
        const auto copy_start = Clock::now();
        const FrameResult reference = reference_view;
        result["frozen_copy_ms"] = Ms(copy_start);
        const auto pre = l1b4::PreWallLabels(reference);
        result["pre_wall_labels"] = Sequence(std::vector<int>(pre.begin(), pre.end()));
        result["ground_plane"] = Sequence(std::vector<float>{reference.ground.plane.a, reference.ground.plane.b,
                                                              reference.ground.plane.c, reference.ground.plane.d});
        result["ground_reference_height"] = reference.ground_reference_height;
        const auto file = result["input_id"].as<std::string>() + ".bin";
        WriteCloud(*reference.leveled, clouds / file); result["voxel_path"] = file;
        result["point_count_preserved"] = reference.leveled->size() == voxel->size();
        int errors = 0;
        for (std::size_t i = 0; i < voxel->size(); ++i)
            errors += std::memcmp(&voxel->points[i].intensity, &reference.leveled->points[i].intensity, sizeof(float)) != 0;
        result["intensity_preservation_errors"] = errors;
        result["baseline"] = Serialize(reference, protect, ambiguous, available, protected_boxes, protection_ms, 0, 0);
        const auto off = l1b4::Apply(reference, protect, params, candidate, false);
        const auto off_result = L1B4Harness::Recluster(pipeline, reference, off.labels, protect);
        result["candidate_off"] = Serialize(off_result, protect, ambiguous, available, protected_boxes, protection_ms, 0, 0);
        auto policy_labels = pre;
        const bool policy_available = available && !yolo.boxes.empty();
        const auto policy_start = Clock::now();
        for (std::size_t i = 0; i < pre.size(); ++i) {
            if (pre[i] != kKept || protect[i] != kProtectNone || (!policy_available && ambiguous[i])) continue;
            const auto& p = reference.leveled->points[i];
            const float ground = reference.ground.source == PlaneSource::kUnavailable
                ? reference.ground_reference_height : reference.ground.plane.HeightAt(p.x, p.y);
            if (p.z - ground < params.wall_remove_min_height_m) continue;
            for (const auto& wall : reference.walls) {
                if (!wall.removal_allowed) continue;
                for (const auto& segment : wall.segments)
                    if (std::abs(segment.plane.SignedDistance(p.x, p.y, p.z)) <= params.wall_remove_distance_m &&
                        segment.InIntervals(segment.AlongWall(p.x, p.y), params.wall_remove_margin_m)) {
                        policy_labels[i] = kWall; break;
                    }
            }
        }
        const auto policy_ms = Ms(policy_start);
        const auto recluster_start = Clock::now();
        const auto policy_result = L1B4Harness::Recluster(pipeline, reference, policy_labels, protect);
        result["policy_only"] = Serialize(policy_result, protect, ambiguous, policy_available,
            protected_boxes, protection_ms, policy_ms, Ms(recluster_start));
        result["policy_only"]["wall_ambiguous_skipped"] = reference.walls[0].ambiguous_skipped +
            reference.walls[1].ambiguous_skipped + std::max(0, reference.wall_removed - policy_result.wall_removed);
        const auto geometry_start = Clock::now();
        const auto geometry = l1b4::Apply(reference, protect, params, candidate);
        const auto geometry_ms = Ms(geometry_start);
        const auto geometry_recluster_start = Clock::now();
        const auto geometry_result = L1B4Harness::Recluster(pipeline, reference, geometry.labels, protect);
        result["geometry"] = Serialize(geometry_result, protect, ambiguous, available,
            protected_boxes, protection_ms, geometry_ms, Ms(geometry_recluster_start));
        result["geometry"]["wall_ambiguous_skipped"] = 0;
        result["geometry"]["reasons"] = Sequence(std::vector<int>(geometry.reasons.begin(), geometry.reasons.end()));
        result["geometry"]["geometry_protected"] = Sequence(std::vector<int>(geometry.geometry_protected.begin(), geometry.geometry_protected.end()));
        result["geometry"]["uncertain"] = Sequence(std::vector<int>(geometry.uncertain.begin(), geometry.uncertain.end()));
        result["geometry"]["component_id"] = Sequence(geometry.component_id);
        result["geometry"]["geometry_components"] = geometry.components;
        result["wall_models"] = YAML::Node(YAML::NodeType::Sequence);
        for (const auto& model : geometry.models) {
            YAML::Node a; const auto& p = model.segment.plane;
            a["plane"] = Sequence(std::vector<float>{p.a, p.b, p.c, p.d}); a["band"] = model.band;
            a["median_abs_residual"] = model.median; a["mad"] = model.mad; a["residual_samples"] = model.residual_samples;
            a["intervals"] = YAML::Node(YAML::NodeType::Sequence);
            for (auto interval : model.segment.intervals) a["intervals"].push_back(Sequence(std::vector<float>{interval.first, interval.second}));
            result["wall_models"].push_back(a);
        }
        if (CloudHash(*voxel) != result["frozen_voxel_fnv1a64"].as<std::string>()) throw std::runtime_error("shared input mutated");
        Json(out, result); out << '\n';
    }
    return 0;
} catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
