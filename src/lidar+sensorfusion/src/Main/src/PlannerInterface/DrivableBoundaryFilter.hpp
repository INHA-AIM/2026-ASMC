#pragma once

#include <array>
#include <string>

#include <opencv2/core.hpp>

struct PlannerObject;

enum class DrivableMaskStatus {
    kNotConfigured = 0,
    kReady,
    kImageLoadFailed,
    kMetadataLoadFailed,
    kMapIdMismatch,
    kOriginMismatch,
    kInvalidMetadata,
};

struct DrivableFilterDecision {
    bool keep = true;
    bool filter_available = false;
    bool boundary_policy_applied = false;
    double road_ratio = -1.0;
    double predicted_road_ratio = -1.0;
    std::string reason = "filter_unavailable_keep";
};

// Final /last_lidar publication policy. This class deliberately does not
// participate in clustering, tracking, motion classification, or fusion.
// A missing/mismatched map disables deletion (fail-safe KEEP).
class DrivableBoundaryFilter {
public:
    bool Configure(const std::string& image_path,
                   const std::string& metadata_path,
                   const std::string& expected_map_id,
                   const std::array<double, 3>& expected_origin,
                   double margin_m,
                   double road_ratio_threshold,
                   int samples_per_axis);

    DrivableFilterDecision Evaluate(const PlannerObject& object) const;

    // 정적인 경계 객체는 원본 OBB 전체가 아니라 road mask와 실제로 겹치는
    // 부분만 planner 충돌 형상으로 사용한다. 보행자/이동 객체는 미래 예측 시
    // 원래 차체 형상이 필요하므로 자르지 않는다. 실패 시 원본을 보존한다.
    bool BuildPlanningObject(const PlannerObject& object,
                             const DrivableFilterDecision& decision,
                             PlannerObject* planning_object,
                             bool* geometry_clipped = nullptr) const;

    bool ready() const { return status_ == DrivableMaskStatus::kReady; }
    DrivableMaskStatus status() const { return status_; }
    const std::string& status_message() const { return status_message_; }
    const std::string& loaded_map_id() const { return loaded_map_id_; }
    double margin_m() const { return margin_m_; }
    double road_ratio_threshold() const { return road_ratio_threshold_; }
    int supersample() const { return supersample_; }

private:
    double DrivableRatio(const PlannerObject& object) const;
    void Reset(DrivableMaskStatus status, const std::string& message);

    cv::Mat drivable_mask_;
    double pixels_per_meter_ = 0.0;
    double world_offset_x_ = 0.0;
    double world_offset_y_ = 0.0;
    double margin_m_ = 0.0;
    double road_ratio_threshold_ = 0.15;
    int supersample_ = 5;
    DrivableMaskStatus status_ = DrivableMaskStatus::kNotConfigured;
    std::string status_message_ = "not configured";
    std::string loaded_map_id_;
};
