#include <Lidar/TunnelFilter/TunnelFilterRos.hpp>

#include <Lidar/Tracking/tracking_utils.hpp>
#include <Msg/TrackedObjectArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

extern VehicleSpec st_VehicleSpec;

namespace tunnel_filter {
namespace {

using Clock = std::chrono::steady_clock;

double ElapsedMs(const Clock::time_point& start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start)
        .count();
}

struct YoloFrame {
    ros::Time stamp;
    std::vector<BBox> boxes;
};

// 카메라/YOLO 보호 설정. 투영은 기존 퓨전의 sensor_fusion_projection_matrix를
// 그대로 쓴다 (별도 extrinsic을 만들지 않는다).
struct ProtectionParams {
    bool enable = true;
    double max_dt_s = 0.05;  // 기존 fusion_max_time_offset_sec와 같은 기본값
    float box_margin_ratio = 0.10f;
    float box_margin_px = 8.0f;
    float depth_vehicle_m = 6.0f;
    float depth_person_m = 1.5f;
    float depth_front_margin_m = 1.0f;
    float depth_ref_percentile = 0.10f;
    int min_box_points = 3;
    float ambiguous_max_range_m = 30.0f;
};

struct PendingFrame {
    bool valid = false;
    bool denied = false;
    ros::Time stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float speed_mps = 0.0f;
    bool imu_ready = false;
};

// 직전 처리 프레임의 자세/속도 (벽 추적용 자차 이동 계산).
struct MotionAnchor {
    bool valid = false;
    ros::Time stamp;
    float yaw = 0.0f;
    float speed_mps = 0.0f;
};

struct ExperimentState {
    Params params;
    ProtectionParams protection;
    std::unique_ptr<TunnelPipeline> pipeline;
    Eigen::Matrix<double, 3, 4> projection;
    int image_width = 1280;
    int image_height = 720;
    PendingFrame pending;
    bool was_active = false;
    MotionAnchor motion_anchor;
    ros::WallTime last_inactive_diagnostic;
    ExperimentFrame last;

    std::mutex yolo_mutex;
    std::deque<YoloFrame> yolo_frames;
    ros::Subscriber yolo_subscriber;

    ros::Publisher pub_imu_corrected;
    ros::Publisher pub_ceiling_filtered;
    ros::Publisher pub_ground_removed;
    ros::Publisher pub_wall_candidates;
    ros::Publisher pub_wall_filtered;
    ros::Publisher pub_clustered;
    ros::Publisher pub_objects;
    ros::Publisher pub_markers;
    ros::Publisher pub_final_obstacle_markers;
    ros::Publisher pub_diagnostics;
};

std::unique_ptr<ExperimentState> g_state;
const ExperimentFrame kEmptyFrame;
const Params kDefaultParams;

template <typename T>
void Param(ros::NodeHandle& node, const std::string& name, T* value) {
    node.param(name, *value, *value);
}

void ParamFloat(ros::NodeHandle& node, const std::string& name, float* value) {
    double temporary = *value;
    node.param(name, temporary, temporary);
    *value = static_cast<float>(temporary);
}

void OnYolo(const Msg::Yolo_bbox_arr::ConstPtr& message) {
    std::vector<BBox> boxes;
    boxes.reserve(message->yolo_bbox_arr.size());
    for (const auto& detection : message->yolo_bbox_arr) {
        BBox box;
        box.st_Min_pt.f32_X = detection.xMin;
        box.st_Min_pt.f32_Y = detection.yMin;
        box.st_Max_pt.f32_X = detection.xMax;
        box.st_Max_pt.f32_Y = detection.yMax;
        box.str_class = detection.classes;
        box.str_raw_class = detection.classes;
        boxes.push_back(box);
    }
    TunnelExperimentIngestYolo(message->header.stamp, boxes);
}

bool SelectYoloFrame(ExperimentState& state, const ros::Time& lidar_stamp,
                     YoloFrame* selected, double* offset_ms) {
    std::lock_guard<std::mutex> lock(state.yolo_mutex);
    double best = std::numeric_limits<double>::infinity();
    const YoloFrame* best_frame = nullptr;
    for (const auto& frame : state.yolo_frames) {
        const double offset = std::fabs((frame.stamp - lidar_stamp).toSec());
        if (offset < best) {
            best = offset;
            best_frame = &frame;
        }
    }
    if (best_frame == nullptr || best > state.protection.max_dt_s) {
        *offset_ms = std::isfinite(best) ? best * 1000.0 : -1.0;
        return false;
    }
    *selected = *best_frame;
    *offset_ms = best * 1000.0;
    return true;
}

// 동기화된 YOLO 박스 안으로 투영되는 LiDAR 점을 보호점으로 표시한다.
// 2D 박스에는 뒤쪽 벽도 들어오므로, 박스 안 점들의 가까운 쪽 거리(10퍼센타일)
// 에서 클래스별 깊이 범위 안의 점만 보호한다.
void ComputeProtection(const ExperimentState& state,
                       const pcl::PointCloud<pcl::PointXYZI>& cloud,
                       const YoloFrame* frame, std::vector<std::uint8_t>* protect,
                       std::vector<std::uint8_t>* ambiguous, int* protect_boxes) {
    const std::size_t n = cloud.points.size();
    protect->assign(n, kProtectNone);
    ambiguous->assign(n, 0);
    *protect_boxes = 0;
    const auto& P = state.projection;
    std::vector<float> u(n, -1.0f);
    std::vector<float> v(n, -1.0f);
    std::vector<float> range(n, 0.0f);
    std::vector<std::uint8_t> in_image(n, 0);
    const float width = static_cast<float>(state.image_width);
    const float height = static_cast<float>(state.image_height);
    for (std::size_t i = 0; i < n; ++i) {
        const auto& p = cloud.points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        const double cz = P(2, 0) * p.x + P(2, 1) * p.y + P(2, 2) * p.z + P(2, 3);
        if (cz <= 0.1) continue;
        const double cu = (P(0, 0) * p.x + P(0, 1) * p.y + P(0, 2) * p.z + P(0, 3)) / cz;
        const double cv = (P(1, 0) * p.x + P(1, 1) * p.y + P(1, 2) * p.z + P(1, 3)) / cz;
        if (cu < 0.0 || cu >= width || cv < 0.0 || cv >= height) continue;
        u[i] = static_cast<float>(cu);
        v[i] = static_cast<float>(cv);
        range[i] = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        in_image[i] = 1;
        if (range[i] <= state.protection.ambiguous_max_range_m) (*ambiguous)[i] = 1;
    }
    if (frame == nullptr) return;

    std::vector<std::pair<float, std::size_t>> in_box;
    for (const auto& box : frame->boxes) {
        const std::uint8_t protect_class = ProtectClassFromLabel(box.str_class);
        if (protect_class == kProtectNone) continue;
        const float box_width = box.st_Max_pt.f32_X - box.st_Min_pt.f32_X;
        const float box_height = box.st_Max_pt.f32_Y - box.st_Min_pt.f32_Y;
        if (!(box_width > 0.0f) || !(box_height > 0.0f)) continue;
        // 기존 퓨전과 같은 자차 보닛 오검출 제외 규칙.
        const bool vehicle = protect_class == kProtectCar ||
                             protect_class == kProtectTruck ||
                             protect_class == kProtectBus;
        const float center_y = 0.5f * (box.st_Min_pt.f32_Y + box.st_Max_pt.f32_Y);
        if (vehicle && box_width >= width * 0.60f && center_y >= height * 0.85f &&
            box.st_Max_pt.f32_Y >= height * 0.95f) {
            continue;
        }
        const float margin_x = state.protection.box_margin_px +
                               state.protection.box_margin_ratio * box_width;
        const float margin_y = state.protection.box_margin_px +
                               state.protection.box_margin_ratio * box_height;
        const float x0 = box.st_Min_pt.f32_X - margin_x;
        const float x1 = box.st_Max_pt.f32_X + margin_x;
        const float y0 = box.st_Min_pt.f32_Y - margin_y;
        const float y1 = box.st_Max_pt.f32_Y + margin_y;
        in_box.clear();
        for (std::size_t i = 0; i < n; ++i) {
            if (!in_image[i]) continue;
            if (u[i] < x0 || u[i] > x1 || v[i] < y0 || v[i] > y1) continue;
            in_box.emplace_back(range[i], i);
        }
        if (static_cast<int>(in_box.size()) < state.protection.min_box_points) {
            continue;
        }
        std::sort(in_box.begin(), in_box.end());
        const std::size_t ref_index = static_cast<std::size_t>(std::floor(
            state.protection.depth_ref_percentile *
            static_cast<float>(in_box.size() - 1)));
        const float reference = in_box[ref_index].first;
        const float extent = IsPersonClass(protect_class)
            ? state.protection.depth_person_m
            : state.protection.depth_vehicle_m;
        const float near_limit = reference - state.protection.depth_front_margin_m;
        const float far_limit = reference + extent;
        for (const auto& item : in_box) {
            if (item.first < near_limit || item.first > far_limit) continue;
            if ((*protect)[item.second] == kProtectNone) {
                (*protect)[item.second] = protect_class;
            }
        }
        ++(*protect_boxes);
    }
}

pcl::PointCloud<pcl::PointXYZI> Subset(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                       const std::vector<int>& indices) {
    pcl::PointCloud<pcl::PointXYZI> out;
    out.points.reserve(indices.size());
    for (const int index : indices) {
        out.points.push_back(cloud.points[static_cast<std::size_t>(index)]);
    }
    out.width = static_cast<std::uint32_t>(out.points.size());
    out.height = 1;
    out.is_dense = false;
    return out;
}

pcl::PointCloud<pcl::PointXYZI> SelectByLabel(const FrameResult& result,
                                              bool keep_kept, bool keep_wall,
                                              bool keep_ground,
                                              bool keep_ceiling) {
    pcl::PointCloud<pcl::PointXYZI> out;
    out.points.reserve(result.labels.size());
    for (std::size_t i = 0; i < result.labels.size(); ++i) {
        const auto label = result.labels[i];
        const bool keep = (label == kKept && keep_kept) ||
                          (label == kWall && keep_wall) ||
                          (label == kGround && keep_ground) ||
                          (label == kCeiling && keep_ceiling);
        if (keep) out.points.push_back(result.leveled->points[i]);
    }
    out.width = static_cast<std::uint32_t>(out.points.size());
    out.height = 1;
    out.is_dense = false;
    return out;
}

void PublishCloud(ros::Publisher& publisher,
                  const pcl::PointCloud<pcl::PointXYZI>& cloud,
                  const std_msgs::Header& header) {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    publisher.publish(message);
}

std::string Format(double value, int precision = 3) {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(precision);
    stream << value;
    return stream.str();
}

void AddKeyValue(diagnostic_msgs::DiagnosticStatus* status,
                 const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status->values.push_back(item);
}

geometry_msgs::Point MakePoint(float x, float y, float z) {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

visualization_msgs::Marker BaseMarker(const std_msgs::Header& header,
                                      const std::string& ns, int id, int type) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.5);
    return marker;
}

void SetColor(visualization_msgs::Marker* marker, float r, float g, float b,
              float a = 1.0f) {
    marker->color.r = r;
    marker->color.g = g;
    marker->color.b = b;
    marker->color.a = a;
}

void AppendMarkers(const FrameResult& result, const std_msgs::Header& header,
                   visualization_msgs::MarkerArray* markers) {
    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers->markers.push_back(clear);

    static constexpr int kEdges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
        {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (std::size_t i = 0; i < result.objects.size(); ++i) {
        const TunnelObject& object = result.objects[i];
        auto box = BaseMarker(header, "tunnel_obb", static_cast<int>(i),
                              visualization_msgs::Marker::LINE_LIST);
        box.scale.x = object.flags.removed ? 0.05 : 0.08;
        if (object.flags.removed && object.flags.ceiling) {
            SetColor(&box, 1.0f, 0.1f, 0.1f);          // 제거: 천장
        } else if (object.flags.removed) {
            SetColor(&box, 1.0f, 0.5f, 0.0f);          // 제거: 벽/지면잔여
        } else if (object.is_protected) {
            SetColor(&box, 0.0f, 1.0f, 1.0f);          // 보호 객체
        } else if (object.flags.ground_residual) {
            SetColor(&box, 0.6f, 0.6f, 0.6f);          // 지면 잔여 후보(진단)
        } else {
            SetColor(&box, 0.1f, 1.0f, 0.1f);          // 유지
        }
        for (const auto& edge : kEdges) {
            const auto& a = object.obb.corners[static_cast<std::size_t>(edge[0])];
            const auto& b = object.obb.corners[static_cast<std::size_t>(edge[1])];
            box.points.push_back(MakePoint(a.x, a.y, a.z));
            box.points.push_back(MakePoint(b.x, b.y, b.z));
        }
        markers->markers.push_back(box);
        if (object.is_protected || object.flags.removed) {
            auto text = BaseMarker(header, "tunnel_obb_text", static_cast<int>(i),
                                   visualization_msgs::Marker::TEXT_VIEW_FACING);
            text.pose.position = MakePoint(object.obb.cx, object.obb.cy,
                                           object.obb.z_max + 0.4f);
            text.scale.z = 0.5;
            SetColor(&text, 1.0f, 1.0f, 1.0f);
            std::string label = object.is_protected
                ? std::string("protected:") + ProtectClassName(object.protect_class)
                : std::string("removed:") +
                      (object.flags.ceiling ? "ceiling" :
                       object.flags.wall ? "wall" : "ground");
            text.text = label + " L" + Format(object.obb.length, 1);
            markers->markers.push_back(text);
        }
    }

    for (std::size_t side = 0; side < result.walls.size(); ++side) {
        const WallSideResult& wall = result.walls[side];
        int marker_id = 0;
        for (std::size_t k = 0; k < wall.segments.size(); ++k) {
          const WallSegment& segment = wall.segments[k];
          for (const auto& interval : segment.intervals) {
            auto quad = BaseMarker(header, side == 0 ? "tunnel_wall_left"
                                                     : "tunnel_wall_right",
                                   marker_id++,
                                   visualization_msgs::Marker::LINE_LIST);
            quad.scale.x = 0.10;
            if (wall.removal_allowed) {
                SetColor(&quad, 0.2f, 0.4f, 1.0f);   // 확정·제거 적용
            } else {
                SetColor(&quad, 1.0f, 1.0f, 0.0f);   // 검출됐지만 지속성 미충족
            }
            const Plane& plane = segment.plane;
            const float horizontal = std::sqrt(plane.a * plane.a + plane.b * plane.b);
            const float ground = result.ground_reference_height;
            const float z_values[2] = {ground, ground + 3.0f};
            const float s_values[2] = {interval.first, interval.second};
            geometry_msgs::Point corner[2][2];
            for (int zi = 0; zi < 2; ++zi) {
                for (int si = 0; si < 2; ++si) {
                    const float z = z_values[zi];
                    const float q = -(plane.c * z + plane.d) / horizontal;
                    const float x = segment.tangent_x * s_values[si] +
                                    plane.a / horizontal * q;
                    const float y = segment.tangent_y * s_values[si] +
                                    plane.b / horizontal * q;
                    corner[zi][si] = MakePoint(x, y, z);
                }
            }
            const geometry_msgs::Point outline[4] = {
                corner[0][0], corner[0][1], corner[1][1], corner[1][0]};
            for (int e = 0; e < 4; ++e) {
                quad.points.push_back(outline[e]);
                quad.points.push_back(outline[(e + 1) % 4]);
            }
            markers->markers.push_back(quad);
          }
        }
    }

    auto status = BaseMarker(header, "tunnel_status", 0,
                             visualization_msgs::Marker::TEXT_VIEW_FACING);
    status.pose.position = MakePoint(0.0f, 0.0f, 3.0f);
    status.scale.z = 0.6;
    SetColor(&status, 1.0f, 1.0f, 1.0f);
    status.text = std::string("TUNNEL EXP ground:") +
                  PlaneSourceName(result.ground.source) +
                  " L:" + (result.walls[0].removal_allowed ? "on" :
                           result.walls[0].fit_valid ? "wait" : "off") +
                  " R:" + (result.walls[1].removal_allowed ? "on" :
                           result.walls[1].fit_valid ? "wait" : "off");
    markers->markers.push_back(status);
}

// RViz verification output that mirrors /lidar/tunnel/objects exactly.
// Unlike /lidar/tunnel/markers, this topic intentionally excludes wall planes,
// rejected ceiling/wall/ground OBBs, and diagnostic status markers. Every box
// shown here is an object that survived the complete tunnel filter.
void AppendFinalObstacleMarkers(const FrameResult& result,
                                const std_msgs::Header& header,
                                visualization_msgs::MarkerArray* markers) {
    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers->markers.push_back(clear);

    static constexpr int kEdges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
        {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    int marker_id = 0;
    for (const TunnelObject& object : result.objects) {
        if (object.flags.removed) continue;

        auto box = BaseMarker(header, "tunnel_final_obstacle", marker_id++,
                              visualization_msgs::Marker::LINE_LIST);
        box.scale.x = 0.10;
        SetColor(&box, 0.1f, 1.0f, 0.1f);  // final output: bright green
        for (const auto& edge : kEdges) {
            const auto& a = object.obb.corners[static_cast<std::size_t>(edge[0])];
            const auto& b = object.obb.corners[static_cast<std::size_t>(edge[1])];
            box.points.push_back(MakePoint(a.x, a.y, a.z));
            box.points.push_back(MakePoint(b.x, b.y, b.z));
        }
        markers->markers.push_back(box);
    }
}

void PublishDiagnostics(ExperimentState& state, const ExperimentFrame& frame,
                        const std_msgs::Header& header) {
    diagnostic_msgs::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "tunnel_experimental_filter";
    status.hardware_id = "velodyne";
    if (!frame.active || frame.result == nullptr) {
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.message = "inactive (GPS valid)";
        AddKeyValue(&status, "tunnel_active", "false");
        array.status.push_back(status);
        state.pub_diagnostics.publish(array);
        return;
    }
    const FrameResult& r = *frame.result;
    const bool ground_valid = r.ground.source == PlaneSource::kCurrent;
    status.level = r.ground.source == PlaneSource::kUnavailable
        ? diagnostic_msgs::DiagnosticStatus::ERROR
        : (ground_valid ? diagnostic_msgs::DiagnosticStatus::OK
                        : diagnostic_msgs::DiagnosticStatus::WARN);
    status.message = std::string("ground=") + PlaneSourceName(r.ground.source);
    AddKeyValue(&status, "tunnel_active", "true");
    AddKeyValue(&status, "ground_valid", ground_valid ? "true" : "false");
    AddKeyValue(&status, "plane_source", PlaneSourceName(r.ground.source));
    AddKeyValue(&status, "plane_a", Format(r.ground.plane.a, 5));
    AddKeyValue(&status, "plane_b", Format(r.ground.plane.b, 5));
    AddKeyValue(&status, "plane_c", Format(r.ground.plane.c, 5));
    AddKeyValue(&status, "plane_d", Format(r.ground.plane.d, 5));
    AddKeyValue(&status, "plane_height_at_origin_m",
                Format(r.ground_reference_height));
    AddKeyValue(&status, "normal_angle_deg", Format(r.ground.plane.TiltDeg()));
    AddKeyValue(&status, "current_fit_reason", r.ground_fit.reject_reason);
    AddKeyValue(&status, "candidate_count", std::to_string(r.ground_fit.candidates));
    AddKeyValue(&status, "ransac_input_count",
                std::to_string(r.ground_fit.refined_candidates));
    AddKeyValue(&status, "candidate_prior",
                r.ground_fit.prior_source == 0 ? "previous_plane"
                : r.ground_fit.prior_source == 1 ? "z_mode" : "none");
    AddKeyValue(&status, "inlier_count", std::to_string(r.ground_fit.inliers));
    AddKeyValue(&status, "inlier_ratio", Format(r.ground_fit.inlier_ratio));
    AddKeyValue(&status, "ground_refit_used", r.ground_fit.refit_used ? "true" : "false");
    AddKeyValue(&status, "ground_refit_angle_deg", Format(r.ground_fit.refit_angle_deg));
    AddKeyValue(&status, "ground_refit_height_delta_m",
                Format(r.ground_fit.refit_height_delta_m));
    AddKeyValue(&status, "consecutive_ground_failures",
                std::to_string(r.ground.consecutive_failures));
    AddKeyValue(&status, "ground_fallback_used",
                r.ground_fallback_used ? "true" : "false");
    AddKeyValue(&status, "ground_removed_count", std::to_string(r.ground_removed));
    AddKeyValue(&status, "ceiling_removed_count", std::to_string(r.ceiling_removed));
    for (std::size_t side = 0; side < 2; ++side) {
        const std::string prefix = side == 0 ? "wall_left_" : "wall_right_";
        const WallSideResult& wall = r.walls[side];
        AddKeyValue(&status, prefix + "fit_valid", wall.fit_valid ? "true" : "false");
        AddKeyValue(&status, prefix + "removal_applied",
                    wall.removal_allowed ? "true" : "false");
        AddKeyValue(&status, prefix + "persist_frames", std::to_string(wall.track.persist));
        AddKeyValue(&status, prefix + "segments", std::to_string(wall.segments.size()));
        AddKeyValue(&status, prefix + "reason", wall.first_attempt.reject_reason);
        AddKeyValue(&status, prefix + "inliers",
                    std::to_string(wall.fit_valid ? wall.segments.front().inliers : 0));
        AddKeyValue(&status, prefix + "removed", std::to_string(wall.removed));
    }
    AddKeyValue(&status, "ego_motion_valid", r.motion.valid ? "true" : "false");
    AddKeyValue(&status, "ego_motion_dt_s", Format(r.motion.dt_s));
    AddKeyValue(&status, "ego_motion_dyaw_deg", Format(r.motion.dyaw_rad * 57.29578f));
    AddKeyValue(&status, "ego_motion_dx_m", Format(r.motion.dx_m));
    AddKeyValue(&status, "protection_available",
                frame.protection_available ? "true" : "false");
    AddKeyValue(&status, "yolo_offset_ms", Format(frame.yolo_offset_ms, 1));
    AddKeyValue(&status, "protected_points", std::to_string(r.protected_points));
    AddKeyValue(&status, "clusters", std::to_string(r.clusters.size()));
    AddKeyValue(&status, "objects_removed_ceiling",
                std::to_string(r.objects_removed_ceiling));
    AddKeyValue(&status, "objects_removed_wall", std::to_string(r.objects_removed_wall));
    AddKeyValue(&status, "kept_points", std::to_string(r.kept_points));
    AddKeyValue(&status, "time_imu_ms", Format(r.timings.level_ms));
    AddKeyValue(&status, "time_ceiling_ms", Format(r.timings.ceiling_ms));
    AddKeyValue(&status, "time_ground_fit_ms", Format(r.timings.ground_fit_ms));
    AddKeyValue(&status, "time_ground_remove_ms", Format(r.timings.ground_remove_ms));
    AddKeyValue(&status, "time_wall_fit_ms", Format(r.timings.wall_fit_ms));
    AddKeyValue(&status, "time_wall_remove_ms", Format(r.timings.wall_remove_ms));
    AddKeyValue(&status, "time_cluster_ms", Format(r.timings.cluster_ms));
    AddKeyValue(&status, "time_obb_ms", Format(r.timings.obb_ms + r.timings.obb_filter_ms));
    AddKeyValue(&status, "time_protection_ms", Format(frame.protection_ms));
    AddKeyValue(&status, "time_publish_ms", Format(frame.publish_ms));
    AddKeyValue(&status, "time_pipeline_total_ms", Format(r.timings.total_ms));
    AddKeyValue(&status, "time_frame_total_ms", Format(frame.total_ms));
    array.status.push_back(status);
    state.pub_diagnostics.publish(array);
}

void PublishFrame(ExperimentState& state, const ExperimentFrame& frame) {
    const FrameResult& r = *frame.result;
    std_msgs::Header header;
    header.stamp = frame.stamp;
    header.frame_id = "velodyne";

    if (state.pub_imu_corrected.getNumSubscribers() > 0) {
        PublishCloud(state.pub_imu_corrected,
                     SelectByLabel(r, true, true, true, true), header);
    }
    if (state.pub_ceiling_filtered.getNumSubscribers() > 0) {
        PublishCloud(state.pub_ceiling_filtered,
                     SelectByLabel(r, true, true, true, false), header);
    }
    if (state.pub_ground_removed.getNumSubscribers() > 0) {
        PublishCloud(state.pub_ground_removed,
                     SelectByLabel(r, true, true, false, false), header);
    }
    if (state.pub_wall_candidates.getNumSubscribers() > 0) {
        PublishCloud(state.pub_wall_candidates,
                     Subset(*r.leveled, r.wall_candidate_indices), header);
    }
    if (state.pub_wall_filtered.getNumSubscribers() > 0) {
        PublishCloud(state.pub_wall_filtered, Subset(*r.leveled, r.kept_indices),
                     header);
    }
    if (state.pub_clustered.getNumSubscribers() > 0) {
        pcl::PointCloud<pcl::PointXYZI> clustered;
        for (std::size_t c = 0; c < r.clusters.size(); ++c) {
            for (const int index : r.clusters[c].indices) {
                pcl::PointXYZI point = r.leveled->points[static_cast<std::size_t>(index)];
                point.intensity = static_cast<float>(c);  // 클러스터 id
                clustered.points.push_back(point);
            }
        }
        clustered.width = static_cast<std::uint32_t>(clustered.points.size());
        clustered.height = 1;
        clustered.is_dense = false;
        PublishCloud(state.pub_clustered, clustered, header);
    }

    Msg::TrackedObjectArray objects;
    objects.header = header;
    for (std::size_t i = 0; i < r.objects.size(); ++i) {
        const TunnelObject& object = r.objects[i];
        if (object.flags.removed) continue;
        Msg::TrackedObject message;
        message.id = static_cast<int32_t>(i);  // 프레임 내 클러스터 번호(추적 아님)
        const std::string cls = object.is_protected
            ? ProtectClassName(object.protect_class) : "unknown";
        message.type = IsPersonClass(object.protect_class) && object.is_protected
            ? 1 : (object.is_protected ? 2 : 3);
        message.classification = "unknown";
        message.yolo_class = cls;
        message.is_dynamic = false;
        message.age = 0;
        message.miss_count = 0;
        message.cluster_center = MakePoint(object.obb.cx, object.obb.cy, object.obb.cz);
        message.obb_center = message.cluster_center;
        for (std::size_t k = 0; k < 8; ++k) {
            const auto& corner = object.obb.corners[k];
            message.obb_corners[k] = MakePoint(corner.x, corner.y, corner.z);
        }
        message.obb_yaw = object.obb.yaw;
        message.speed_kph = 0.0;
        message.length = object.obb.length;
        message.width = object.obb.width;
        message.height = object.obb.z_max - object.obb.z_min;
        objects.objects.push_back(message);
    }
    state.pub_objects.publish(objects);

    if (state.pub_markers.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray markers;
        AppendMarkers(r, header, &markers);
        state.pub_markers.publish(markers);
    }
    if (state.pub_final_obstacle_markers.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray markers;
        AppendFinalObstacleMarkers(r, header, &markers);
        state.pub_final_obstacle_markers.publish(markers);
    }
    // 진단은 발행 시간과 프레임 전체 시간을 채운 뒤 TunnelExperimentRun에서 보낸다.
}

void PublishInactive(ExperimentState& state, const ros::Time& stamp) {
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = "velodyne";
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    state.pub_markers.publish(markers);
    state.pub_final_obstacle_markers.publish(markers);
    Msg::TrackedObjectArray objects;
    objects.header = header;
    state.pub_objects.publish(objects);
    ExperimentFrame inactive;
    PublishDiagnostics(state, inactive, header);
}

}  // namespace

Params LoadParams(ros::NodeHandle& node, const Params& base) {
    Params p = base;
    Param(node, "tunnel_dynamic_roi_enable", &p.dynamic_roi_enable);
    ParamFloat(node, "tunnel_roi_min_x_m", &p.roi_min_x_m);
    ParamFloat(node, "tunnel_roi_max_x_m", &p.roi_max_x_m);
    ParamFloat(node, "tunnel_roi_min_y_m", &p.roi_min_y_m);
    ParamFloat(node, "tunnel_roi_max_y_m", &p.roi_max_y_m);
    ParamFloat(node, "tunnel_roi_min_z_m", &p.roi_min_z_m);
    ParamFloat(node, "tunnel_roi_max_z_m", &p.roi_max_z_m);
    Param(node, "tunnel_level_roll", &p.level_roll);
    Param(node, "tunnel_level_pitch", &p.level_pitch);
    ParamFloat(node, "tunnel_ceiling_max_z_lidar_m", &p.ceiling_max_z_lidar_m);
    ParamFloat(node, "tunnel_expected_ground_z_lidar_m", &p.expected_ground_z_lidar_m);
    ParamFloat(node, "tunnel_ground_candidate_z_min_m", &p.ground_candidate_z_min_m);
    ParamFloat(node, "tunnel_ground_candidate_z_max_m", &p.ground_candidate_z_max_m);
    ParamFloat(node, "tunnel_ground_candidate_max_range_m", &p.ground_candidate_max_range_m);
    ParamFloat(node, "tunnel_ground_prior_band_m", &p.ground_prior_band_m);
    ParamFloat(node, "tunnel_ground_mode_bin_m", &p.ground_mode_bin_m);
    ParamFloat(node, "tunnel_ground_mode_max_range_m", &p.ground_mode_max_range_m);
    Param(node, "tunnel_ground_ransac_max_iterations", &p.ground_ransac_max_iterations);
    ParamFloat(node, "tunnel_ground_ransac_distance_threshold_m",
               &p.ground_ransac_distance_threshold_m);
    ParamFloat(node, "tunnel_ground_normal_max_angle_deg", &p.ground_normal_max_angle_deg);
    Param(node, "tunnel_ground_refit_enable", &p.ground_refit_enable);
    ParamFloat(node, "tunnel_ground_refit_band_m", &p.ground_refit_band_m);
    ParamFloat(node, "tunnel_ground_refit_distance_m", &p.ground_refit_distance_m);
    Param(node, "tunnel_ground_refit_iterations", &p.ground_refit_iterations);
    ParamFloat(node, "tunnel_ground_refit_max_angle_deg", &p.ground_refit_max_angle_deg);
    ParamFloat(node, "tunnel_ground_refit_max_height_delta_m",
               &p.ground_refit_max_height_delta_m);
    ParamFloat(node, "tunnel_ground_height_min_m", &p.ground_height_min_m);
    ParamFloat(node, "tunnel_ground_height_max_m", &p.ground_height_max_m);
    Param(node, "tunnel_ground_min_inliers", &p.ground_min_inliers);
    ParamFloat(node, "tunnel_ground_min_inlier_ratio", &p.ground_min_inlier_ratio);
    ParamFloat(node, "tunnel_ground_max_height_jump_m", &p.ground_max_height_jump_m);
    ParamFloat(node, "tunnel_ground_max_normal_jump_deg", &p.ground_max_normal_jump_deg);
    Param(node, "tunnel_ground_reuse_max_frames", &p.ground_reuse_max_frames);
    ParamFloat(node, "tunnel_ground_remove_distance_m", &p.ground_remove_distance_m);
    Param(node, "tunnel_ground_remove_below_plane", &p.ground_remove_below_plane);
    ParamFloat(node, "tunnel_ground_fallback_band_m", &p.ground_fallback_band_m);
    Param(node, "tunnel_wall_enable", &p.wall_enable);
    ParamFloat(node, "tunnel_wall_search_min_abs_y_m", &p.wall_search_min_abs_y_m);
    ParamFloat(node, "tunnel_wall_search_x_min_m", &p.wall_search_x_min_m);
    ParamFloat(node, "tunnel_wall_search_x_max_m", &p.wall_search_x_max_m);
    ParamFloat(node, "tunnel_wall_candidate_min_height_m", &p.wall_candidate_min_height_m);
    ParamFloat(node, "tunnel_wall_candidate_max_height_m", &p.wall_candidate_max_height_m);
    Param(node, "tunnel_wall_ransac_max_iterations", &p.wall_ransac_max_iterations);
    ParamFloat(node, "tunnel_wall_fit_distance_m", &p.wall_fit_distance_m);
    ParamFloat(node, "tunnel_wall_max_tilt_deg", &p.wall_max_tilt_deg);
    Param(node, "tunnel_wall_min_inliers", &p.wall_min_inliers);
    ParamFloat(node, "tunnel_wall_min_inlier_ratio", &p.wall_min_inlier_ratio);
    ParamFloat(node, "tunnel_wall_min_span_m", &p.wall_min_span_m);
    ParamFloat(node, "tunnel_wall_bin_size_m", &p.wall_bin_size_m);
    Param(node, "tunnel_wall_bin_min_points", &p.wall_bin_min_points);
    Param(node, "tunnel_wall_bin_max_gap", &p.wall_bin_max_gap);
    Param(node, "tunnel_wall_min_occupied_bins", &p.wall_min_occupied_bins);
    ParamFloat(node, "tunnel_wall_min_vertical_span_m", &p.wall_min_vertical_span_m);
    ParamFloat(node, "tunnel_wall_min_lateral_distance_m", &p.wall_min_lateral_distance_m);
    ParamFloat(node, "tunnel_wall_beyond_margin_m", &p.wall_beyond_margin_m);
    ParamFloat(node, "tunnel_wall_max_beyond_fraction", &p.wall_max_beyond_fraction);
    Param(node, "tunnel_wall_max_segments", &p.wall_max_segments);
    ParamFloat(node, "tunnel_wall_extend_min_run_m", &p.wall_extend_min_run_m);
    ParamFloat(node, "tunnel_wall_extend_max_gap_m", &p.wall_extend_max_gap_m);
    ParamFloat(node, "tunnel_wall_step_max_angle_deg", &p.wall_step_max_angle_deg);
    ParamFloat(node, "tunnel_wall_step_max_offset_m", &p.wall_step_max_offset_m);
    ParamFloat(node, "tunnel_wall_step_min_vertical_span_m",
               &p.wall_step_min_vertical_span_m);
    ParamFloat(node, "tunnel_wall_continuation_max_angle_deg",
               &p.wall_continuation_max_angle_deg);
    ParamFloat(node, "tunnel_wall_continuation_max_gap_m", &p.wall_continuation_max_gap_m);
    ParamFloat(node, "tunnel_wall_continuation_min_span_m", &p.wall_continuation_min_span_m);
    Param(node, "tunnel_wall_continuation_min_bins", &p.wall_continuation_min_bins);
    ParamFloat(node, "tunnel_wall_continuation_search_m", &p.wall_continuation_search_m);
    Param(node, "tunnel_wall_min_persist_frames", &p.wall_min_persist_frames);
    ParamFloat(node, "tunnel_wall_track_max_angle_deg", &p.wall_track_max_angle_deg);
    ParamFloat(node, "tunnel_wall_track_max_offset_m", &p.wall_track_max_offset_m);
    Param(node, "tunnel_wall_track_max_miss", &p.wall_track_max_miss);
    Param(node, "tunnel_wall_track_use_ego_motion", &p.wall_track_use_ego_motion);
    ParamFloat(node, "tunnel_wall_track_motion_max_dt_s", &p.wall_track_motion_max_dt_s);
    ParamFloat(node, "tunnel_wall_remove_distance_m", &p.wall_remove_distance_m);
    ParamFloat(node, "tunnel_wall_remove_margin_m", &p.wall_remove_margin_m);
    ParamFloat(node, "tunnel_wall_remove_min_height_m", &p.wall_remove_min_height_m);
    ParamFloat(node, "tunnel_cluster_tolerance_m", &p.cluster_tolerance_m);
    Param(node, "tunnel_cluster_min_points", &p.cluster_min_points);
    Param(node, "tunnel_cluster_max_points", &p.cluster_max_points);
    Param(node, "tunnel_obb_filter_enable", &p.obb_filter_enable);
    ParamFloat(node, "tunnel_obb_ceiling_min_z_road_m", &p.obb_ceiling_min_z_road_m);
    ParamFloat(node, "tunnel_obb_wall_min_abs_y_m", &p.obb_wall_min_abs_y_m);
    ParamFloat(node, "tunnel_obb_wall_min_length_m", &p.obb_wall_min_length_m);
    ParamFloat(node, "tunnel_obb_wall_min_aspect_ratio", &p.obb_wall_min_aspect_ratio);
    ParamFloat(node, "tunnel_obb_wall_attach_distance_m", &p.obb_wall_attach_distance_m);
    ParamFloat(node, "tunnel_obb_wall_attach_max_angle_deg",
               &p.obb_wall_attach_max_angle_deg);
    ParamFloat(node, "tunnel_obb_wall_attached_min_length_m",
               &p.obb_wall_attached_min_length_m);
    ParamFloat(node, "tunnel_obb_ground_residual_max_height_m",
               &p.obb_ground_residual_max_height_m);
    ParamFloat(node, "tunnel_obb_ground_residual_max_top_road_m",
               &p.obb_ground_residual_max_top_road_m);
    Param(node, "tunnel_obb_remove_ground_residual", &p.obb_remove_ground_residual);
    Param(node, "tunnel_protect_obb_min_points", &p.protect_obb_min_points);
    ParamFloat(node, "tunnel_protect_obb_min_fraction", &p.protect_obb_min_fraction);
    return p;
}

ObbFunction MakeTrackingObbFunction() {
    return [](const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
              const std::vector<pcl::PointIndices>& clusters,
              std::vector<Obb>* obbs) {
        obbs->assign(clusters.size(), Obb());
        if (!cloud || clusters.empty()) return;
        pcl::PointCloud<pcl::PointXYZI>::Ptr centroids(
            new pcl::PointCloud<pcl::PointXYZI>());
        centroids->points.reserve(clusters.size());
        for (const auto& cluster : clusters) {
            pcl::PointXYZI centroid;
            centroid.x = centroid.y = centroid.z = 0.0f;
            centroid.intensity = 0.0f;
            for (const int index : cluster.indices) {
                const auto& p = cloud->points[static_cast<std::size_t>(index)];
                centroid.x += p.x;
                centroid.y += p.y;
                centroid.z += p.z;
            }
            const float count = static_cast<float>(std::max<std::size_t>(1, cluster.indices.size()));
            centroid.x /= count;
            centroid.y /= count;
            centroid.z /= count;
            centroids->points.push_back(centroid);
        }
        // 기존 추적기와 같은 L-shape OBB를 재사용한다 (추적 상태는 건드리지 않음).
        std::vector<LiDARObjectInfo> infos;
        GPS_IMU unused_gps_imu{};
        Objectinfo_for_km(infos, cloud, centroids, clusters, unused_gps_imu,
                          st_VehicleSpec);
        for (std::size_t c = 0; c < infos.size() && c < clusters.size(); ++c) {
            const auto& info = infos[c];
            Obb& obb = (*obbs)[c];
            obb.cx = info.f32_OBB_CenterX;
            obb.cy = info.f32_OBB_CenterY;
            obb.cz = info.f32_OBB_CenterZ;
            obb.length = std::max(info.f32_OBB_Width, info.f32_OBB_Height);
            obb.width = std::min(info.f32_OBB_Width, info.f32_OBB_Height);
            obb.z_min = info.f32_OBB_ZMin;
            obb.z_max = info.f32_OBB_ZMax;
            obb.yaw = info.f32_OBB_YawRad;
            for (std::size_t k = 0; k < 8; ++k) {
                obb.corners[k] = pcl::PointXYZ(info.OBB_Corners_Body[k].x,
                                               info.OBB_Corners_Body[k].y,
                                               info.OBB_Corners_Body[k].z);
            }
        }
    };
}

bool InitTunnelExperiment(ros::NodeHandle& node_handle,
                          ros::NodeHandle& private_node, const CAMERA& camera,
                          int image_width, int image_height,
                          bool subscribe_yolo) {
    bool enabled = false;
    private_node.param("enable_tunnel_experimental_filter", enabled, false);
    if (!enabled) {
        g_state.reset();
        return false;
    }
    g_state = std::make_unique<ExperimentState>();
    ExperimentState& state = *g_state;
    state.params = LoadParams(private_node);
    Param(private_node, "tunnel_protect_enable", &state.protection.enable);
    Param(private_node, "tunnel_protect_max_dt_s", &state.protection.max_dt_s);
    ParamFloat(private_node, "tunnel_protect_box_margin_ratio",
               &state.protection.box_margin_ratio);
    ParamFloat(private_node, "tunnel_protect_box_margin_px",
               &state.protection.box_margin_px);
    ParamFloat(private_node, "tunnel_protect_depth_vehicle_m",
               &state.protection.depth_vehicle_m);
    ParamFloat(private_node, "tunnel_protect_depth_person_m",
               &state.protection.depth_person_m);
    ParamFloat(private_node, "tunnel_protect_depth_front_margin_m",
               &state.protection.depth_front_margin_m);
    ParamFloat(private_node, "tunnel_protect_depth_ref_percentile",
               &state.protection.depth_ref_percentile);
    Param(private_node, "tunnel_protect_min_box_points",
          &state.protection.min_box_points);
    ParamFloat(private_node, "tunnel_protect_ambiguous_max_range_m",
               &state.protection.ambiguous_max_range_m);
    state.projection = camera.params.sensor_fusion_projection_matrix;
    state.image_width = image_width;
    state.image_height = image_height;
    state.pipeline = std::make_unique<TunnelPipeline>(state.params,
                                                      MakeTrackingObbFunction());

    state.pub_imu_corrected = node_handle.advertise<sensor_msgs::PointCloud2>(
        "/lidar/tunnel/debug_imu_corrected", 1);
    state.pub_ceiling_filtered = node_handle.advertise<sensor_msgs::PointCloud2>(
        "/lidar/tunnel/debug_ceiling_filtered", 1);
    state.pub_ground_removed = node_handle.advertise<sensor_msgs::PointCloud2>(
        "/lidar/tunnel/debug_ground_removed", 1);
    state.pub_wall_candidates = node_handle.advertise<sensor_msgs::PointCloud2>(
        "/lidar/tunnel/debug_wall_candidates", 1);
    state.pub_wall_filtered = node_handle.advertise<sensor_msgs::PointCloud2>(
        "/lidar/tunnel/debug_wall_filtered", 1);
    state.pub_clustered = node_handle.advertise<sensor_msgs::PointCloud2>(
        "/lidar/tunnel/debug_clustered", 1);
    state.pub_objects = node_handle.advertise<Msg::TrackedObjectArray>(
        "/lidar/tunnel/objects", 2);
    state.pub_markers = node_handle.advertise<visualization_msgs::MarkerArray>(
        "/lidar/tunnel/markers", 2);
    state.pub_final_obstacle_markers =
        node_handle.advertise<visualization_msgs::MarkerArray>(
            "/lidar/tunnel/final_obstacle_markers", 2);
    state.pub_diagnostics =
        node_handle.advertise<diagnostic_msgs::DiagnosticArray>(
            "/lidar/tunnel/diagnostics", 5);
    if (subscribe_yolo && state.protection.enable) {
        state.yolo_subscriber = node_handle.subscribe("/YOLO/BBox", 10, OnYolo);
    }
    ROS_WARN(
        "Tunnel experimental filter ENABLED (runs only while GPS is denied): "
        "ceiling>%.2f ground_z=[%.2f,%.2f] ransac=%.2fm/%ddeg wall_rm=%.2fm "
        "tol=%.2fm protect=%s",
        state.params.ceiling_max_z_lidar_m, state.params.ground_candidate_z_min_m,
        state.params.ground_candidate_z_max_m,
        state.params.ground_ransac_distance_threshold_m,
        static_cast<int>(state.params.ground_normal_max_angle_deg),
        state.params.wall_remove_distance_m, state.params.cluster_tolerance_m,
        state.protection.enable ? "on" : "off");
    return true;
}

bool TunnelExperimentEnabled() { return static_cast<bool>(g_state); }

void TunnelExperimentCapture(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                             const ros::Time& stamp, const GPS_IMU& gps_imu) {
    if (!g_state) return;
    PendingFrame& pending = g_state->pending;
    pending.valid = true;
    pending.denied = IsGpsDenied(gps_imu.st_GPS.b_isJamming,
                                 gps_imu.st_GPS.b_isTunnel);
    pending.stamp = stamp;
    // GPS 정상 구간에서는 점군을 잡아두지도 않는다 (기존 파이프라인만 동작).
    pending.cloud = pending.denied ? cloud : pcl::PointCloud<pcl::PointXYZI>::Ptr();
    pending.roll = gps_imu.st_IMU.f32_Roll_rad;
    pending.pitch = gps_imu.st_IMU.f32_Pitch_rad;
    pending.yaw = gps_imu.st_IMU.f32_Yaw_rad;
    pending.speed_mps = gps_imu.st_CAN.f32_CurrentVel_MPS;
    pending.imu_ready = gps_imu.st_IMU.b_ImuReady;
}

void TunnelExperimentRun() {
    if (!g_state || !g_state->pending.valid) return;
    ExperimentState& state = *g_state;
    PendingFrame pending = state.pending;
    state.pending = PendingFrame();
    const auto frame_start = Clock::now();

    ExperimentFrame& frame = state.last;
    ++frame.sequence;
    frame.stamp = pending.stamp;
    frame.active = pending.denied && pending.cloud;
    frame.result = nullptr;
    frame.input = pending.cloud;
    if (!frame.active) {
        if (state.was_active) {
            state.pipeline->Reset();
            state.was_active = false;
            PublishInactive(state, pending.stamp);
            state.last_inactive_diagnostic = ros::WallTime::now();
        } else if ((ros::WallTime::now() - state.last_inactive_diagnostic)
                       .toSec() >= 1.0) {
            // 기능 ON + GPS 정상: 점군은 처리하지 않고 비활성 상태만 1 Hz로 알린다.
            std_msgs::Header header;
            header.stamp = pending.stamp;
            header.frame_id = "velodyne";
            ExperimentFrame inactive;
            PublishDiagnostics(state, inactive, header);
            state.last_inactive_diagnostic = ros::WallTime::now();
        }
        frame.protect_class.clear();
        frame.ambiguous.clear();
        return;
    }
    if (!state.was_active) {
        state.pipeline->Reset();  // 터널 진입마다 이전 지면/벽 이력 없이 시작
        state.was_active = true;
        state.motion_anchor = MotionAnchor();
    }
    // 직전 처리 프레임 이후 자차 이동: IMU yaw 변화 + 평균 속도×dt (평면 운동 근사).
    EgoMotion motion;
    const MotionAnchor& anchor = state.motion_anchor;
    if (anchor.valid && pending.imu_ready) {
        const float dt = static_cast<float>((pending.stamp - anchor.stamp).toSec());
        const float dyaw = std::atan2(std::sin(pending.yaw - anchor.yaw),
                                      std::cos(pending.yaw - anchor.yaw));
        const float distance = 0.5f * (pending.speed_mps + anchor.speed_mps) * dt;
        if (dt > 0.0f && std::isfinite(dyaw) && std::isfinite(distance)) {
            motion.valid = true;
            motion.dt_s = dt;
            motion.dyaw_rad = dyaw;
            motion.dx_m = distance * std::cos(0.5f * dyaw);
            motion.dy_m = distance * std::sin(0.5f * dyaw);
        }
    }
    state.motion_anchor.valid = pending.imu_ready;
    state.motion_anchor.stamp = pending.stamp;
    state.motion_anchor.yaw = pending.yaw;
    state.motion_anchor.speed_mps = pending.speed_mps;
    frame.roll_rad = pending.roll;
    frame.pitch_rad = pending.pitch;
    frame.imu_ready = pending.imu_ready;

    const auto protection_start = Clock::now();
    YoloFrame yolo;
    frame.protection_available = false;
    frame.yolo_boxes = 0;
    if (state.protection.enable) {
        frame.protection_available =
            SelectYoloFrame(state, pending.stamp, &yolo, &frame.yolo_offset_ms);
        frame.yolo_boxes = frame.protection_available
            ? static_cast<int>(yolo.boxes.size()) : 0;
    } else {
        frame.yolo_offset_ms = -1.0;
    }
    ComputeProtection(state, *pending.cloud,
                      frame.protection_available ? &yolo : nullptr,
                      &frame.protect_class, &frame.ambiguous,
                      &frame.protect_boxes);
    frame.protection_ms = ElapsedMs(protection_start);

    frame.result = &state.pipeline->Process(
        *pending.cloud, pending.roll, pending.pitch, &frame.protect_class,
        &frame.ambiguous, frame.protection_available, &motion);

    const auto publish_start = Clock::now();
    PublishFrame(state, frame);
    frame.publish_ms = ElapsedMs(publish_start);
    frame.total_ms = ElapsedMs(frame_start);
    std_msgs::Header header;
    header.stamp = frame.stamp;
    header.frame_id = "velodyne";
    PublishDiagnostics(state, frame, header);
    ROS_INFO_THROTTLE(
        2.0,
        "[TUNNEL_EXP] ground=%s h=%.2f L=%s R=%s kept=%d clusters=%zu removed(obb) c=%d w=%d total=%.1fms",
        PlaneSourceName(frame.result->ground.source),
        frame.result->ground_reference_height,
        frame.result->walls[0].removal_allowed ? "on" : "off",
        frame.result->walls[1].removal_allowed ? "on" : "off",
        frame.result->kept_points, frame.result->clusters.size(),
        frame.result->objects_removed_ceiling,
        frame.result->objects_removed_wall, frame.total_ms);
}

void TunnelExperimentIngestYolo(const ros::Time& image_stamp,
                                const std::vector<BBox>& boxes) {
    if (!g_state || image_stamp.isZero()) return;
    ExperimentState& state = *g_state;
    std::lock_guard<std::mutex> lock(state.yolo_mutex);
    YoloFrame frame;
    frame.stamp = image_stamp;
    frame.boxes = boxes;
    const auto position = std::upper_bound(
        state.yolo_frames.begin(), state.yolo_frames.end(), image_stamp,
        [](const ros::Time& stamp, const YoloFrame& candidate) {
            return stamp < candidate.stamp;
        });
    state.yolo_frames.insert(position, std::move(frame));
    // 오래된 결과는 쓰지 않는다: 최신 기준 2초보다 오래된 프레임은 버린다.
    const ros::Time newest = state.yolo_frames.back().stamp;
    while (!state.yolo_frames.empty() &&
           (newest - state.yolo_frames.front().stamp).toSec() > 2.0) {
        state.yolo_frames.pop_front();
    }
    while (state.yolo_frames.size() > 64) state.yolo_frames.pop_front();
}

const ExperimentFrame& TunnelExperimentLastFrame() {
    return g_state ? g_state->last : kEmptyFrame;
}

const Params& TunnelExperimentParams() {
    return g_state ? g_state->params : kDefaultParams;
}

}  // namespace tunnel_filter
