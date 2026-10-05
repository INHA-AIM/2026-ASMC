#include <PlannerInterface/DrivableBoundaryFilter.hpp>

#include <PlannerInterface/DrivableFootprint.hpp>
#include <PlannerInterface/PlannerObjectAdapter.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

namespace {

std::string NormalizeLabel(std::string label) {
    label.erase(label.begin(), std::find_if(
        label.begin(), label.end(), [](unsigned char value) {
            return !std::isspace(value);
        }));
    label.erase(std::find_if(
        label.rbegin(), label.rend(), [](unsigned char value) {
            return !std::isspace(value);
        }).base(), label.end());
    std::transform(label.begin(), label.end(), label.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    return label;
}

bool IsPedestrian(const PlannerObject& object) {
    const std::string classification = NormalizeLabel(object.classification);
    if (classification == "person" || classification == "pedestrian") {
        return true;
    }

    const std::string yolo = NormalizeLabel(object.yolo_class);
    return yolo == "person" || yolo == "pedestrian";
}

bool HasMotionEvidence(const PlannerObject& object) {
    // asmc motion-state contract: 2=DYNAMIC_PENDING, 3=DYNAMIC.
    return object.is_dynamic || object.motion_state == 2 ||
           object.motion_state == 3;
}

bool GroundFootprint(const PlannerObject& object,
                     std::array<cv::Point2d, 4>* corners) {
    std::vector<cv::Point2f> footprint;
    footprint.reserve(object.obb_corners_map.size());
    for (const auto& corner : object.obb_corners_map) {
        if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) {
            return false;
        }
        footprint.emplace_back(static_cast<float>(corner.x),
                               static_cast<float>(corner.y));
    }

    // The producer supplies four bottom and four top points, but their order
    // is not part of the safety boundary contract. Canonicalize all XY points
    // before raster overlap evaluation.
    const cv::RotatedRect rectangle = cv::minAreaRect(footprint);
    if (!std::isfinite(rectangle.size.width) ||
        !std::isfinite(rectangle.size.height) ||
        !(rectangle.size.width > 1e-5F) ||
        !(rectangle.size.height > 1e-5F)) {
        return false;
    }
    std::array<cv::Point2f, 4> canonical;
    rectangle.points(canonical.data());
    for (std::size_t index = 0; index < canonical.size(); ++index) {
        (*corners)[index] = {canonical[index].x, canonical[index].y};
    }
    return true;
}

}  // namespace

void DrivableBoundaryFilter::Reset(DrivableMaskStatus status,
                                   const std::string& message) {
    drivable_mask_.release();
    pixels_per_meter_ = 0.0;
    world_offset_x_ = 0.0;
    world_offset_y_ = 0.0;
    loaded_map_id_.clear();
    status_ = status;
    status_message_ = message;
}

bool DrivableBoundaryFilter::Configure(
    const std::string& image_path, const std::string& metadata_path,
    const std::string& expected_map_id,
    const std::array<double, 3>& expected_origin, double margin_m,
    double road_ratio_threshold, int samples_per_axis) {
    Reset(DrivableMaskStatus::kNotConfigured, "configuration in");
    road_ratio_threshold_ = std::max(
        0.0, std::min(1.0, road_ratio_threshold));
    supersample_ = std::max(1, samples_per_axis);
    margin_m_ = std::max(0.0, margin_m);

    cv::Mat mask = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (mask.empty()) {
        Reset(DrivableMaskStatus::kImageLoadFailed,
              "cannot load drivable mask image: " + image_path);
        return false;
    }

    YAML::Node metadata;
    try {
        metadata = YAML::LoadFile(metadata_path);
    } catch (const std::exception& error) {
        Reset(DrivableMaskStatus::kMetadataLoadFailed,
              "cannot load drivable mask metadata: " +
                  metadata_path + " (" + error.what() + ")");
        return false;
    }

    try {
        loaded_map_id_ = metadata["map_id"].as<std::string>();
        pixels_per_meter_ = metadata["pixels_per_meter"].as<double>();
        const YAML::Node offset = metadata["world_offset"];
        world_offset_x_ = offset[0].as<double>();
        world_offset_y_ = offset[1].as<double>();
    } catch (const std::exception& error) {
        Reset(DrivableMaskStatus::kInvalidMetadata,
              "invalid drivable mask metadata: " +
                  metadata_path + " (" + error.what() + ")");
        return false;
    }

    if (!expected_map_id.empty() && loaded_map_id_ != expected_map_id) {
        const std::string actual_map_id = loaded_map_id_;
        Reset(DrivableMaskStatus::kMapIdMismatch,
              "drivable map id mismatch: expected " + expected_map_id +
                  ", got " + actual_map_id);
        return false;
    }
    try {
        const YAML::Node origin = metadata["local_origin_in_global"];
        for (std::size_t i = 0; i < expected_origin.size(); ++i) {
            const double actual = origin[i].as<double>();
            if (!std::isfinite(actual) ||
                std::abs(actual - expected_origin[i]) > 1e-6) {
                Reset(DrivableMaskStatus::kOriginMismatch,
                      "drivable map local origin mismatch");
                return false;
            }
        }
    } catch (const std::exception& error) {
        Reset(DrivableMaskStatus::kInvalidMetadata,
              "invalid drivable mask local origin: " +
                  std::string(error.what()));
        return false;
    }
    if (!(pixels_per_meter_ > 0.0) || !std::isfinite(pixels_per_meter_) ||
        !std::isfinite(world_offset_x_) || !std::isfinite(world_offset_y_)) {
        Reset(DrivableMaskStatus::kInvalidMetadata,
              "drivable mask metadata contains invalid numeric values");
        return false;
    }

    if (margin_m_ > 0.0) {
        const int kernel_size = std::max(
            3, static_cast<int>(std::lround(
                2.0 * margin_m_ * pixels_per_meter_)) + 1);
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::dilate(mask, mask, kernel);
    }
    drivable_mask_ = mask;
    status_ = DrivableMaskStatus::kReady;
    status_message_ = "ready";
    return true;
}

double DrivableBoundaryFilter::DrivableRatio(
    const PlannerObject& object) const {
    std::array<cv::Point2d, 4> corners;
    if (!GroundFootprint(object, &corners)) return -1.0;
    return DrivableFootprintRatio(
        drivable_mask_, pixels_per_meter_, world_offset_x_, world_offset_y_,
        corners, supersample_);
}

DrivableFilterDecision DrivableBoundaryFilter::Evaluate(
    const PlannerObject& object) const {
    DrivableFilterDecision decision;
    decision.filter_available = ready();
    if (!ready()) return decision;

    // 진단과 후속 계획 관련성 판정이 같은 기하 정보를 보도록 예외 객체도
    // 현재 road overlap부터 계산한다. 과거에는 PENDING/DYNAMIC이면 -1을
    // 남기고 무조건 통과해 도로 밖 속도 오검출이 planner까지 들어갔다.
    decision.road_ratio = DrivableRatio(object);
    if (IsPedestrian(object)) {
        decision.reason = "pedestrian_keep";
        return decision;
    }
    if (decision.road_ratio < 0.0 || !std::isfinite(decision.road_ratio)) {
        // Invalid OBBs are diagnosed upstream and remain fail-open here.
        decision.reason = "invalid_geometry_keep";
        return decision;
    }

    decision.boundary_policy_applied = true;
    if (HasMotionEvidence(object)) {
        decision.predicted_road_ratio = decision.road_ratio;
        if (decision.road_ratio >= road_ratio_threshold_) {
            decision.reason = "motion_on_drivable_keep";
            return decision;
        }

        // 도로 밖 이동 객체를 무조건 버리지는 않는다. 원본 map-frame 절대
        // 속도로 OBB를 평행 이동했을 때 3초 안에 road 영역으로 들어오는 경우만
        // planner 입력에 남긴다. 따라서 옆 도로 시설물의 가짜 속도는 제거되고,
        // 실제 횡단/합류 객체는 보존된다.
        const double vx = object.velocity_map_mps.x;
        const double vy = object.velocity_map_mps.y;
        if (std::isfinite(vx) && std::isfinite(vy) &&
            std::hypot(vx, vy) > 0.05) {
            constexpr double kPredictionStepSeconds = 0.5;
            constexpr int kPredictionSteps = 6;
            for (int step = 1; step <= kPredictionSteps; ++step) {
                const double t = kPredictionStepSeconds * step;
                PlannerObject predicted = object;
                predicted.obb_center_map.x += vx * t;
                predicted.obb_center_map.y += vy * t;
                predicted.cluster_center_map.x += vx * t;
                predicted.cluster_center_map.y += vy * t;
                for (auto& corner : predicted.obb_corners_map) {
                    corner.x += vx * t;
                    corner.y += vy * t;
                }
                decision.predicted_road_ratio = std::max(
                    decision.predicted_road_ratio,
                    DrivableRatio(predicted));
            }
        }
        decision.keep =
            decision.predicted_road_ratio >= road_ratio_threshold_;
        decision.reason = decision.keep
            ? "motion_predicted_drivable_keep"
            : "motion_outside_drivable_drop";
        return decision;
    }

    decision.keep = decision.road_ratio >= road_ratio_threshold_;
    decision.reason = decision.keep ? "drivable_overlap_keep"
                                    : "outside_drivable_drop";
    return decision;
}

bool DrivableBoundaryFilter::BuildPlanningObject(
    const PlannerObject& object,
    const DrivableFilterDecision& decision,
    PlannerObject* planning_object,
    bool* geometry_clipped) const {
    if (planning_object == nullptr) return false;
    *planning_object = object;
    if (geometry_clipped != nullptr) *geometry_clipped = false;
    if (!ready() || !decision.keep || IsPedestrian(object) ||
        HasMotionEvidence(object) || !decision.boundary_policy_applied ||
        decision.road_ratio >= 0.999) {
        return true;
    }

    std::array<cv::Point2d, 4> original;
    std::array<cv::Point2d, 4> clipped;
    if (!GroundFootprint(object, &original) ||
        !DrivableFootprintIntersectionRect(
            drivable_mask_, pixels_per_meter_, world_offset_x_,
            world_offset_y_, original, &clipped, supersample_)) {
        // 기하 변환 실패는 객체 삭제로 바꾸지 않는다. Evaluate가 KEEP한 원본을
        // 그대로 내보내는 fail-open이 좌표/맵 오류에 더 안전하다.
        return true;
    }

    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto& corner : object.obb_corners_map) {
        min_z = std::min(min_z, corner.z);
        max_z = std::max(max_z, corner.z);
    }
    if (!std::isfinite(min_z) || !std::isfinite(max_z)) return true;

    for (std::size_t i = 0; i < clipped.size(); ++i) {
        planning_object->obb_corners_map[i] = {
            clipped[i].x, clipped[i].y, min_z};
        planning_object->obb_corners_map[i + clipped.size()] = {
            clipped[i].x, clipped[i].y, max_z};
    }
    const double edge_a = std::hypot(
        clipped[1].x - clipped[0].x,
        clipped[1].y - clipped[0].y);
    const double edge_b = std::hypot(
        clipped[2].x - clipped[1].x,
        clipped[2].y - clipped[1].y);
    const bool a_is_length = edge_a >= edge_b;
    const cv::Point2d direction = a_is_length
        ? clipped[1] - clipped[0]
        : clipped[2] - clipped[1];
    planning_object->length = std::max(edge_a, edge_b);
    planning_object->width = std::min(edge_a, edge_b);
    planning_object->height = std::max(0.0, max_z - min_z);
    planning_object->obb_yaw_map_rad =
        std::atan2(direction.y, direction.x);
    planning_object->obb_center_map = {};
    for (const auto& corner : clipped) {
        planning_object->obb_center_map.x += corner.x * 0.25;
        planning_object->obb_center_map.y += corner.y * 0.25;
    }
    planning_object->obb_center_map.z = 0.5 * (min_z + max_z);
    if (geometry_clipped != nullptr) *geometry_clipped = true;
    return true;
}
