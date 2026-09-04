#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>

#include "global/utils.hpp"
#include "frenet_planner/obstacle/legacy_object_adapter.hpp"

namespace {

const HighwayMergeZone* HighwayZoneAt(const HighwayMergeConfig& cfg, int index) {
    if (index < 0 || static_cast<size_t>(index) >= cfg.zones.size()) return nullptr;
    return &cfg.zones[static_cast<size_t>(index)];
}

const HighwayMergeCheckpoint* HighwayCheckpointAt(
        const HighwayMergeConfig& cfg, int zone_index, int checkpoint_index) {
    const HighwayMergeZone* zone = HighwayZoneAt(cfg, zone_index);
    if (!zone || checkpoint_index < 0 ||
        static_cast<size_t>(checkpoint_index) >= zone->checkpoints.size()) return nullptr;
    return &zone->checkpoints[static_cast<size_t>(checkpoint_index)];
}

int FindHighwayMergeZone(const HighwayMergeConfig& cfg, double s) {
    if (!cfg.enabled) return -1;
    for (size_t i = 0; i < cfg.zones.size(); ++i) {
        const auto& zone = cfg.zones[i];
        if (s >= zone.start_s && s <= zone.completion_s)
            return static_cast<int>(i);
    }
    return -1;
}

// 횡가속도 한계에서 곡률별 연속 목표속도를 계산한다.
double VelocityFromCurvature(double k, const CurveSpeedConfig& cfg, double max_lateral_accel) {
    constexpr double kSafetyMargin = 0.8;         // 한계의 80%만 사용 (여유폭 확보)
    constexpr double kMinKappaForCalc = 1e-4;      // 0 나누기 방지 (거의 직선)
    const double v_curve = std::sqrt(max_lateral_accel * kSafetyMargin / std::max(k, kMinKappaForCalc));
    return clip(v_curve, 0.0, cfg.target_vel);
}

// 현재 속도의 제동거리와 기본 lookahead 중 큰 범위의 최대 곡률을 사용한다.
double LookaheadTargetSpeed(const RefLine& ref, double s_start, double ego_speed,
                             const CurveSpeedConfig& cfg, double max_longitudinal_accel,
                             double max_lateral_accel, double max_curvature) {
    constexpr double kComfortableDecelFactor = 0.5;  // 실제 평균 감속도는 max의 절반 정도로 가정
    constexpr double kReactionMargin = 5.0;          // [m] 추가 여유거리
    const double usable_decel = std::max(0.5, max_longitudinal_accel * kComfortableDecelFactor);
    const double v_floor = VelocityFromCurvature(max_curvature, cfg, max_lateral_accel);
    const double speed_drop = std::max(0.0, ego_speed * ego_speed - v_floor * v_floor);
    const double required_dist = speed_drop / (2.0 * usable_decel);
    const double lookahead_m = std::max(cfg.curve_lookahead_m, required_dist + kReactionMargin);

    // 폐루프 wrap-around: s_start+lookahead_m이 s_max를 넘으면(피니시라인
    // 근처) 원래 코드는 뒤가 안 나와 break만 되고 이음새 너머(실제로는 바로
    // 이어지는 트랙 시작 구간)의 곡률을 놓쳤다 - 그 구간에 급커브가 있으면
    // 감속 없이 그대로 진입하게 된다. s_max로 감아서 [s_start, s_max]와
    // [0, 넘친 만큼] 두 구간을 모두 본다.
    const double s_max = ref.points.back().s;
    double s_start_w = std::fmod(s_start, s_max);
    if (s_start_w < 0.0) s_start_w += s_max;
    const double s_end_w = s_start_w + lookahead_m;
    const double wrapped_end = s_end_w - s_max;  // s_end_w > s_max일 때만 유효

    double max_k = 0.0;
    for (const auto& p : ref.points) {
        const bool in_range = (s_end_w <= s_max)
            ? (p.s >= s_start_w && p.s <= s_end_w)
            : (p.s >= s_start_w || p.s <= wrapped_end);
        if (in_range) max_k = std::max(max_k, std::fabs(p.kappa));
    }
    return VelocityFromCurvature(max_k, cfg, max_lateral_accel);
}

const char* ModeName(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING: return "LANE_KEEPING";
        case FOLLOWING: return "FOLLOWING";
        case AVOID: return "AVOID";
        case MERGE: return "MERGE";
        default: return "OTHER";
    }
}

const char* AvoidPhaseName(int phase) {
    switch (phase) {
        case 0: return "TRACKING";
        case 1: return "SHIFT";
        case 2: return "PASS";
        default: return "NONE";
    }
}

const char* MergePhaseName(int phase) {
    switch (phase) {
        case 0: return "APPROACH";
        case 1: return "WAIT";
        case 2: return "COMMIT";
        case 3: return "CROSS";
        case 4: return "CLEAR";
        default: return "NONE";
    }
}

void ExtendMergePathThroughConflict(CartesianPath& path, const RefLine& ref,
                                    double conflict_s, double completion_distance,
                                    double path_end_s, double target_speed, double dt) {
    if (path.x.empty()) return;
    const double speed = std::max(target_speed, 0.5);
    for (double s = std::max(conflict_s, path_end_s) + speed * dt;
         s <= conflict_s + completion_distance + 1e-6; s += speed * dt) {
        const RefPoint rp = Interpolate(ref, s);
        path.x.push_back(rp.x);
        path.y.push_back(rp.y);
        path.yaw.push_back(rp.theta);
        path.kappa.push_back(rp.kappa);
        path.v.push_back(speed);
        path.a.push_back(0.0);
    }
}

CartesianPath RemainingPath(const CartesianPath& path, double ego_x, double ego_y,
                            size_t& progress_index) {
    CartesianPath result;
    if (path.x.empty()) return result;
    const size_t count = std::min({path.x.size(), path.y.size(), path.yaw.size(),
                                   path.kappa.size(), path.v.size(), path.a.size()});
    const size_t search_begin = std::min(progress_index, count - 1);
    size_t first = search_begin;
    double best_dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = search_begin; i < count; ++i) {
        const double dx = path.x[i] - ego_x;
        const double dy = path.y[i] - ego_y;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < best_dist2) {
            best_dist2 = dist2;
            first = i;
        }
    }
    progress_index = first;
    result.x.assign(path.x.begin() + first, path.x.begin() + count);
    result.y.assign(path.y.begin() + first, path.y.begin() + count);
    result.yaw.assign(path.yaw.begin() + first, path.yaw.begin() + count);
    result.kappa.assign(path.kappa.begin() + first, path.kappa.begin() + count);
    result.v.assign(path.v.begin() + first, path.v.begin() + count);
    result.a.assign(path.a.begin() + first, path.a.begin() + count);
    return result;
}

bool CartesianTrajectoryClear(const CartesianPath& path,
                              const std::vector<TrackedObstacle>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id = nullptr) {
    if (blocking_id) *blocking_id = -1;
    const size_t count = std::min({path.x.size(), path.y.size(), path.yaw.size()});
    for (size_t i = 0; i < count; ++i) {
        CartesianState ego{};
        ego.x = path.x[i];
        ego.y = path.y[i];
        ego.yaw = path.yaw[i];
        const double t = static_cast<double>(i) * dt;
        const double margin = collision_cfg.safety_margin +
                              collision_cfg.margin_growth_rate * t;
        // 기존 구현은 ego와 obstacle 양쪽에 margin을 적용했다. 장애물 원본
        // corner를 변경하지 않으면서 총 여유를 보수적으로 유지하도록 ego에
        // 두 배 margin을 적용한다.
        const geometry::Polygon2D ego_footprint =
            MakeEgoFootprint(ego, ego_shape, 2.0 * margin);
        for (const auto& obj : obstacles) {
            if (CheckObstacleOverlap(ego_footprint, {}, obj, t)) {
                if (blocking_id) *blocking_id = obj.id;
                return false;
            }
        }
    }
    return true;
}

// 후보 경로의 최대 기하 곡률에서 a_lat=v^2*|kappa| 한계를
// 만족하는 속도를 계산한다. 제어/모델 오차 여유로 한계의 80%를 쓴다.
double CandidateCurvatureSpeedLimit(const FrenetPath& path, const RefLine& ref,
                                    double max_lateral_accel) {
    constexpr double kLateralAccelMargin = 0.8;
    constexpr double kMinCurvature = 1e-4;
    const GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
    double max_kappa = 0.0;
    for (double kappa : geo.kappa) max_kappa = std::max(max_kappa, std::fabs(kappa));
    if (max_kappa < kMinCurvature) return std::numeric_limits<double>::infinity();
    return std::sqrt(max_lateral_accel * kLateralAccelMargin / max_kappa);
}

// 재생성된 후보의 실제 샘플별 속도²×곡률을 검증한다. 단순히
// 기하 곡률만 작은 경로가 아니라, 해당 속도에서 실제로 횡가속도
// 한계를 만족하는 경로만 남긴다.
void FilterByCartesianLateralAcceleration(std::vector<FrenetPath>& candidates,
                                          const RefLine& ref,
                                          double max_lateral_accel) {
    constexpr double kLateralAccelMargin = 0.8;
    const double limit = max_lateral_accel * kLateralAccelMargin;
    for (auto& path : candidates) {
        if (!path.valid) continue;
        const GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
        const size_t count = std::min(geo.kappa.size(), path.s_d.size());
        for (size_t i = 0; i < count; ++i) {
            const double lateral_accel = path.s_d[i] * path.s_d[i] * std::fabs(geo.kappa[i]);
            if (lateral_accel > limit) {
                path.valid = false;
                path.rejection_reason = RejectionReason::CURVATURE;
                break;
            }
        }
    }
}

}  // namespace

bool FrenetPlanner::Init(const std::string& yaml_path) {
    std::string waypoint_file;
    LoadParams(yaml_path, path_cfg_, limits_, cost_weights_, vehicle_shape_,
               collision_cfg_, curve_speed_cfg_, following_cfg_, avoid_cfg_,
               stop_cfg_, merge_cfg_, highway_merge_cfg_, visualization_cfg_,
               wheelbase_, lane_width_, waypoint_file);

    if (!waypoint_file.empty() && waypoint_file.front() != '/') {
        waypoint_file = std::string(FRENET_PACKAGE_SRC_DIR) + "/" + waypoint_file;
    }

    debug_writer_ = std::make_unique<PlannerDebugWriter>(visualization_cfg_);

    if (waypoint_file.empty() || !LoadReferenceLine(waypoint_file, ref_, limits_.max_curvature)) {
        std::printf("[FrenetPlanner] Reference line load failed (waypoint_file='%s')\n",
                    waypoint_file.c_str());
        return false;
    }
    if (merge_cfg_.conflict_s < 0.0) {
        std::printf("[FrenetPlanner] Roundabout MERGE disabled: set "
                    "planner.merge.conflict_s in params.yaml\n");
    } else {
        const RefPoint conflict = Interpolate(ref_, merge_cfg_.conflict_s);
        std::printf("[FrenetPlanner] Roundabout MERGE conflict: s=%.2f pos=(%.2f, %.2f)\n",
                    merge_cfg_.conflict_s, conflict.x, conflict.y);
    }
    if (!highway_merge_cfg_.enabled || highway_merge_cfg_.zones.empty()) {
        std::printf("[FrenetPlanner] Highway MERGE disabled: measure start/conflict/"
                    "completion_s, then set planner.highway_merge.enabled=true\n");
    } else {
        for (const auto& zone : highway_merge_cfg_.zones) {
            bool valid_zone = zone.start_s >= 0.0 && !zone.checkpoints.empty();
            double previous_clear_s = zone.start_s;
            for (const auto& checkpoint : zone.checkpoints) {
                valid_zone = valid_zone && checkpoint.conflict_s > previous_clear_s &&
                    checkpoint.clear_s > checkpoint.conflict_s;
                previous_clear_s = checkpoint.clear_s;
            }
            valid_zone = valid_zone && zone.completion_s >= previous_clear_s;
            if (!valid_zone) {
                std::printf("[FrenetPlanner] Highway MERGE zone '%s' invalid s ordering; "
                            "policy disabled\n", zone.name.c_str());
                highway_merge_cfg_.enabled = false;
                break;
            }
            std::printf("[FrenetPlanner] Highway MERGE '%s': start=%.2f complete=%.2f "
                        "checkpoints=%zu\n", zone.name.c_str(), zone.start_s,
                        zone.completion_s, zone.checkpoints.size());
            for (size_t i = 0; i < zone.checkpoints.size(); ++i) {
                const auto& checkpoint = zone.checkpoints[i];
                const RefPoint conflict = Interpolate(ref_, checkpoint.conflict_s);
                std::printf("  checkpoint[%zu]: conflict=%.2f clear=%.2f pos=(%.2f, %.2f)\n",
                            i, checkpoint.conflict_s, checkpoint.clear_s,
                            conflict.x, conflict.y);
            }
        }
    }
    ref_loaded_ = true;
    return true;
}

// 저속 fallback은 시간 대신 이동거리 기준 cosine smoothstep으로 d를 복귀시킨다.
// 양 끝의 d'가 0이므로 정지 근처에서도 급격한 횡이동과 곡률 spike가 생기지 않는다.
FrenetPlanner::FollowTargetStatus FrenetPlanner::ResolveExternalFollowTarget(
    double ego_s, int target_object_id,
    const std::vector<ObjectInfo>& obstacles,
    FollowingContext& context) const {
    if (target_object_id < 0) return FollowTargetStatus::LOST;

    // 지정된 ID만 찾는다. FindLeader()처럼 "가장 가까운" 차량을 고르지
    // 않는다 - 대상 선택 권한은 FSM에 있다.
    const auto it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [target_object_id](const ObjectInfo& object) {
            return object.id == target_object_id;
        });

    if (it != obstacles.end()) {
        double object_s = 0.0, object_d = 0.0, object_s_dot = 0.0;
        ProjectObjectToFrenet(ref_, *it, object_s, object_d, object_s_dot);

        // 폐루프 트랙에서 gap이 음수로 크게 튀지 않도록 감아준다.
        const double track_length =
            ref_.points.empty() ? 0.0 : ref_.points.back().s;
        double gap = object_s - ego_s;
        if (gap <= 0.0 && track_length > 0.0) gap += track_length;

        // 검증 기준은 legacy와 동일하다. 다만 "이 조건을 만족하는 차량을
        // 찾는다"가 아니라 "지정된 차량이 이 조건을 만족하는가"를 본다.
        //
        // 탐색거리도 legacy와 같은 방식으로 확장한다. config 값을 그대로 쓰면
        // 충돌 lookahead보다 짧아져, leader를 FOLLOW 대상이 아니라 충돌물로만
        // 먼저 보고 정지하는 현상이 생긴다(legacy가 이 문제로 확장을 넣었다).
        //
        // 현재 config 기준:
        //   config 값        60.0 m
        //   충돌 horizon     10.0 * 8.0 + 4.5 + 0.3 = 84.8 m
        //   유지 한계        84.8 + 10.0 = 94.8 m
        // 확장하지 않으면 70 m가 되어 80 m 앞의 leader가 v2에서만
        // TARGET_OBJECT_LOST가 된다.
        const double collision_horizon =
            curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead +
            vehicle_shape_.length + collision_cfg_.safety_margin;
        const double effective_search_s =
            std::max(following_cfg_.max_leader_search_s, collision_horizon);
        const double search_limit =
            effective_search_s + following_cfg_.exit_search_margin;
        // 속도 하한(min_leader_speed)을 적용하지 않는다.
        //
        // 그 값(0.5 m/s)은 legacy에서 "움직이는 차량은 FOLLOW, 정지 객체는
        // AVOID"를 planner가 스스로 분류하기 위한 policy였다. v2에서 그
        // 분류 권한은 FSM에 있으므로, FSM이 FOLLOW id=15를 지정했는데
        // Frenet이 "15가 0 m/s니까 FOLLOW하지 않겠다"고 하면 행동 선택에
        // 다시 개입하는 셈이다.
        //
        // 실제 도로에서도 앞차가 신호에 멈추면 그 뒤에 같이 멈추는 것이
        // FOLLOW다. GenerateFollowingCandidates는 정지 leader에 대해
        // min_gap을 유지하는 종방향 계획을 이미 할 수 있다.
        //
        // 역방향으로 움직이는 객체만 거부한다 - 마주 오는 차량을 추종하면
        // gap이 음수로 발산한다.
        const bool same_direction = object_s_dot >= 0.0;
        const bool same_lane = std::fabs(object_d) <= lane_width_ * 0.5;
        const bool in_range = gap > 0.0 && gap <= search_limit;

        if (same_direction && same_lane && in_range) {
            context.active = true;
            context.leader_id = target_object_id;
            context.leader_s = ego_s + gap;
            context.leader_d = object_d;
            context.leader_speed = object_s_dot;
            context.leader_accel = 0.0;
            context.missing_cycles = 0;
            return FollowTargetStatus::VALID;
        }

        // 관측되었지만 쓸 수 없는 상태다. dropout이 아니므로 유예를 주지
        // 않는다 - 예측으로 메꿀 대상이 아니라 조건 자체가 틀렸다
        // (계약 8.2절).
        context = FollowingContext{};
        return FollowTargetStatus::LOST;
    }

    // 관측되지 않았다. 직전에 확보한 상태가 있으면 유예한다.
    //
    // 한 번도 확보하지 못한 대상에는 유예를 주지 않는다. 예측할 기반
    // (leader_s / leader_speed)이 없으므로 유예는 의미가 없고, FSM이 존재하지
    // 않는 ID를 보낸 경우를 조용히 넘기게 된다.
    if (!context.active || context.leader_id != target_object_id) {
        context = FollowingContext{};
        return FollowTargetStatus::LOST;
    }

    if (++context.missing_cycles > following_cfg_.dropout_grace_cycles) {
        context = FollowingContext{};
        return FollowTargetStatus::LOST;
    }

    // 유예 중에는 직전 속도로 leader 위치를 예측한다(legacy와 동일).
    constexpr double kPlannerCycleDt = 0.05;  // 20 Hz
    context.leader_s += context.leader_speed * kPlannerCycleDt;
    return FollowTargetStatus::GRACE;
}

double FrenetPlanner::EffectiveAvoidDetectionDistance() const {
    // legacy AVOID 등록거리와 같은 식이다(frenet_planner.cpp의 acquisition_cfg).
    // config 값을 그대로 쓰면 충돌 lookahead보다 짧아져, 장애물을 회피 대상으로
    // 등록하기 전에 충돌 필터가 중앙 후보를 전부 제거한다 -> 속도 급락 후
    // 등록되면 다시 가속하는 진동이 생긴다.
    //
    // 현재 config 기준:
    //   config 값     60.0 m
    //   충돌 horizon  10.0 * 8.0 + 4.5 + 0.3 = 84.8 m
    const double collision_horizon =
        curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead +
        vehicle_shape_.length + collision_cfg_.safety_margin;
    return std::max(avoid_cfg_.detection_distance, collision_horizon);
}

FrenetPlanner::AvoidTargetStatus FrenetPlanner::ResolveExternalAvoidTarget(
    double ego_s, int target_object_id,
    const std::vector<ObjectInfo>& obstacles,
    AvoidanceContext& context) const {
    if (target_object_id < 0) return AvoidTargetStatus::LOST;

    // 지정된 ID만 찾는다. FindStaticAvoidanceTarget()처럼 "가장 가까운 정적
    // 장애물"을 고르지 않는다 - 대상 선택 권한은 FSM에 있다(헤더 주석 참고).
    const auto it = std::find_if(
        obstacles.begin(), obstacles.end(),
        [target_object_id](const ObjectInfo& object) {
            return object.id == target_object_id;
        });

    if (it != obstacles.end()) {
        double object_s = 0.0, object_d = 0.0, object_s_dot = 0.0;
        ProjectObjectToFrenet(ref_, *it, object_s, object_d, object_s_dot);

        // 폐루프 트랙에서 gap이 음수로 크게 튀지 않도록 감아준다(FOLLOW와 동일).
        const double track_length =
            ref_.points.empty() ? 0.0 : ref_.points.back().s;
        double gap = object_s - ego_s;
        if (gap <= 0.0 && track_length > 0.0) gap += track_length;

        const double search_limit = EffectiveAvoidDetectionDistance();

        // 속도 조건(trigger_max_speed)을 적용하지 않는다 - 헤더 주석 참고.
        // 정지/저속/주행 중 어느 쪽이든 FSM이 AVOID를 지정했으면 대상이다.
        const bool in_front = gap > 0.0;
        const bool in_range = gap <= search_limit;
        // 회피는 같은 corridor의 장애물을 대상으로 한다. 옆 차선 객체를
        // 회피하려고 횡이동하면 오히려 그쪽으로 들어간다.
        const bool same_corridor = std::fabs(object_d) <= lane_width_ * 0.5;
        // geometry가 0이면 회피 오프셋을 계산할 수 없다. 후보 생성 단계에서
        // 0 폭 장애물을 비켜가는 target_d는 의미가 없다.
        const bool geometry_ok = it->width > 0.0 && it->length > 0.0;

        if (in_front && in_range && same_corridor && geometry_ok) {
            context.active = true;
            // 요청 ID가 아니라 **실제로 찾은 객체의 ID**를 기록한다. 둘은
            // 항상 같아야 하지만, 같다고 가정해 요청 ID를 그대로 넣으면
            // 조회가 잘못되어도(예: nearest selection으로 회귀) 확보 ID는
            // 정상으로 보인다 - 진단과 회귀 검증이 그 오류를 놓친다.
            context.obstacle_id = it->id;
            context.obstacle_s = ego_s + gap;
            context.obstacle_d = object_d;
            context.obstacle_width = it->width;
            context.obstacle_length = it->length;
            context.missing_cycles = 0;
            return AvoidTargetStatus::VALID;
        }

        // 관측되었지만 쓸 수 없는 상태다. dropout이 아니므로 유예를 주지
        // 않는다 - 예측으로 메꿀 대상이 아니라 조건 자체가 틀렸다
        // (계약 8.2절). phase/target_d/start_d는 호출자가 관리하므로
        // 여기서는 확보 상태만 해제한다.
        context.active = false;
        context.obstacle_id = -1;
        context.missing_cycles = 0;
        return AvoidTargetStatus::LOST;
    }

    // 관측되지 않았다. 직전에 확보한 상태가 있으면 유예한다.
    //
    // 한 번도 확보하지 못한 대상에는 유예를 주지 않는다. FSM이 존재하지 않는
    // ID를 보낸 경우를 조용히 넘기게 된다(FOLLOW와 동일한 판단).
    if (!context.active || context.obstacle_id != target_object_id) {
        context.active = false;
        context.obstacle_id = -1;
        context.missing_cycles = 0;
        return AvoidTargetStatus::LOST;
    }

    if (++context.missing_cycles > avoid_cfg_.dropout_grace_cycles) {
        context.active = false;
        context.obstacle_id = -1;
        context.missing_cycles = 0;
        return AvoidTargetStatus::LOST;
    }

    // 유예 중에는 마지막으로 관측한 위치를 그대로 유지한다.
    //
    // FOLLOW는 leader_speed로 leader_s를 전진시키지만, AVOID 대상은 정적
    // 장애물이 주 대상이고 관측된 속도를 보관하지도 않는다. 없는 속도를
    // 추정해 위치를 밀면 실제보다 멀리 있다고 판단해 회피 시작이 늦어진다.
    return AvoidTargetStatus::GRACE;
}

FrenetPlanner::AvoidTargetPlan FrenetPlanner::DecideAvoidTargets(
    const AvoidanceContext& context, LateralDirection direction) const {
    AvoidTargetPlan plan;
    if (!context.active) return plan;

    // ---- target pose가 있으면 그 d가 authoritative ----------------------
    if (context.has_requested_target_d) {
        plan.candidates[0] = context.requested_target_d;
        plan.count = 1;
        plan.from_target_pose = true;
        // 방향 요청과의 정합성만 표시한다. 여기서 거부하지 않는 이유는
        // 헤더 AvoidTargetPlan 주석에 있다 - clearance 부족은 충돌 검사가
        // 판정할 일이고, 그 결과는 WAITING이다.
        if (direction == LateralDirection::LEFT) {
            plan.direction_conflict =
                context.requested_target_d <= context.obstacle_d;
        } else if (direction == LateralDirection::RIGHT) {
            plan.direction_conflict =
                context.requested_target_d >= context.obstacle_d;
        }
        return plan;
    }

    // ---- pose가 없으면 legacy geometry 계산을 재사용 --------------------
    //
    // legacy begin_avoidance 분기와 **같은 식**이다. 고정 ±avoid_offset(5.0)이
    // 아니라 장애물의 실제 횡위치 + 실제 폭을 반영한다.
    constexpr double kDirectionExtraMargin = 0.3;
    const double clearance =
        0.5 * (vehicle_shape_.width + context.obstacle_width) +
        collision_cfg_.safety_margin + kDirectionExtraMargin;
    const double left_target = context.obstacle_d + clearance;
    const double right_target = context.obstacle_d - clearance;

    switch (direction) {
        case LateralDirection::LEFT:
            plan.candidates[0] = left_target;
            plan.count = 1;
            break;
        case LateralDirection::RIGHT:
            plan.candidates[0] = right_target;
            plan.count = 1;
            break;
        case LateralDirection::NONE:
            // 방향 미지정이면 Frenet이 고른다 - 그것이 DIRECTION_NONE의
            // 의미다(BehaviorCommand.msg). 순서는 legacy와 같이 left 먼저.
            plan.candidates[0] = left_target;
            plan.candidates[1] = right_target;
            plan.count = 2;
            break;
    }
    return plan;
}

bool FrenetPlanner::GenerateDecelToZeroPath(double s, double d, double s_dot,
                                            CartesianPath& out_path) const {
    // D4-3에서 STOP BRAKING fail-safe에서 추출했다. 두 경로가 정확히 같은
    // 감속 프로파일을 쓰도록 공유한다 - 복사해두면 한쪽만 고쳐지는 문제가
    // 생긴다.
    const double decel = std::max(0.1, stop_cfg_.comfortable_decel);
    const double v0 = std::max(0.0, s_dot);
    const int n = static_cast<int>(std::ceil(v0 / decel / path_cfg_.dt)) + 1;
    out_path.x.assign(n, 0.0); out_path.y.assign(n, 0.0);
    out_path.yaw.assign(n, 0.0); out_path.kappa.assign(n, 0.0);
    out_path.v.assign(n, 0.0); out_path.a.assign(n, 0.0);
    double s_i = s;
    double v_prev = v0;
    for (int i = 0; i < n; ++i) {
        const double t = i * path_cfg_.dt;
        const double v_i = std::max(0.0, v0 - decel * t);
        if (i > 0) s_i += 0.5 * (v_i + v_prev) * path_cfg_.dt;
        v_prev = v_i;
        const RefPoint rp = Interpolate(ref_, s_i);
        // 정지 직전 구간이라 횡방향은 현재 오프셋을 그대로 유지한다.
        const CartesianState cs =
            FrenetToCartesian(rp, s_i, v_i, v_i > 0.0 ? -decel : 0.0,
                              d, 0.0, 0.0);
        out_path.x[i] = cs.x;
        out_path.y[i] = cs.y;
        out_path.yaw[i] = cs.yaw;
        out_path.kappa[i] = cs.kappa;
        out_path.v[i] = v_i;
        out_path.a[i] = v_i > 0.0 ? -decel : 0.0;
    }
    return n > 0;
}

bool FrenetPlanner::PlanLowSpeedFallback(const CartesianState& ego, double s, double d,
                                          double target_d,
                                          CartesianPath& out_path, double duration_s,
                                          double speed_cap) const {
    constexpr double kRecenterDistM = 15.0;   // [m] d를 0으로 되돌리는 데 쓰는 이동거리
    const int n = static_cast<int>(duration_s / path_cfg_.dt) + 1;
    out_path.x.resize(n); out_path.y.resize(n); out_path.yaw.resize(n);
    out_path.kappa.resize(n); out_path.v.resize(n); out_path.a.resize(n);

    double s_i = s;
    double v_prev = std::max(ego.v, kMinCreepSpeed);
    for (int i = 0; i < n; ++i) {
        const double t = i * path_cfg_.dt;
        const double crawl_speed = clip(
            ego.v + kCrawlAccel * t, kMinCreepSpeed, kCrawlSpeedCap);
        const double v_i = std::min(crawl_speed, speed_cap);
        if (i > 0) s_i += 0.5 * (v_i + v_prev) * path_cfg_.dt;
        v_prev = v_i;

        const double ds = s_i - s;
        const double frac = clip(ds / kRecenterDistM, 0.0, 1.0);
        // smoothstep(frac) = 0.5*(1+cos(pi*frac)): frac=0 -> d, frac=1 -> 0, 양끝 기울기 0.
        const double d_error = d - target_d;
        const double d_i      = target_d + d_error * 0.5 * (1.0 + std::cos(M_PI * frac));
        const double d_prime  = (ds >= kRecenterDistM) ? 0.0
                               : d_error * (-0.5 * M_PI / kRecenterDistM) * std::sin(M_PI * frac);
        const double d_pprime = (ds >= kRecenterDistM) ? 0.0
                               : d_error * (-0.5 * M_PI * M_PI / (kRecenterDistM * kRecenterDistM))
                                     * std::cos(M_PI * frac);

        RefPoint rp = Interpolate(ref_, s_i);
        const double a_i = (v_i < kCrawlSpeedCap) ? kCrawlAccel : 0.0;
        CartesianState cs = FrenetToCartesian(rp, s_i, v_i, a_i, d_i, d_prime, d_pprime);

        out_path.x[i] = cs.x;
        out_path.y[i] = cs.y;
        out_path.yaw[i] = cs.yaw;
        out_path.kappa[i] = cs.kappa;
        out_path.v[i] = v_i;
        out_path.a[i] = a_i;
    }
    return true;
}

// Cartesian 정지 목표 -> reference line 종방향 좌표.
bool FrenetPlanner::ProjectPointToFrenet(double x, double y, double s_hint,
                                         double search_window,
                                         double max_lateral_distance,
                                         double& out_s, double& out_d) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    // 창과 허용 거리는 호출자가 정한다. 잘못된 값으로 조용히 통과시키면
    // 투영 방어가 사실상 사라진다.
    if (!std::isfinite(search_window) || search_window <= 0.0) return false;
    if (!std::isfinite(max_lateral_distance) || max_lateral_distance <= 0.0) {
        return false;
    }

    // 탐색창은 ego localization용 기본값(30m)이 아니라 호출자가 준 값을 쓴다.
    // 목표점은 ego보다 앞에 있어 기본 창을 벗어나고, 그러면 잘린 초기
    // 추정값에서 출발한 Newton이 트랙의 다른 구간으로 발산한다.
    const double* hint = std::isfinite(s_hint) ? &s_hint : nullptr;
    const double s = FindClosestS(ref_, x, y, hint, search_window);
    if (!std::isfinite(s)) return false;

    // 최근접 투영은 목표가 경로에서 아무리 멀어도 어떤 s든 돌려준다. 실제로
    // 그 s가 목표를 대표하는지 확인하지 않으면, 다른 도로 위의 목표점이
    // 엉뚱한 위치의 명령으로 둔갑한다.
    const RefPoint rp = Interpolate(ref_, s);
    const double lateral_error = std::hypot(x - rp.x, y - rp.y);
    if (!std::isfinite(lateral_error) ||
        lateral_error > max_lateral_distance) {
        return false;
    }

    // d의 부호 규약은 CartesianToFrenet 식 (4)와 같아야 한다. 다른 식을 쓰면
    // 같은 점이 경로에 따라 좌/우가 반대로 나온다.
    //   d = (x - r(s))^T n_r(s),  n_r = (-sin(theta_r), cos(theta_r))
    // 즉 진행 방향 기준 좌측이 양수다.
    const double d = (x - rp.x) * (-std::sin(rp.theta)) +
                     (y - rp.y) * std::cos(rp.theta);
    if (!std::isfinite(d)) return false;

    out_s = s;
    out_d = d;
    return true;
}

bool FrenetPlanner::ProjectPointToS(double x, double y, double s_hint,
                                    double& out_s) const {
    // STOP 전용 wrapper. 탐색창과 허용 횡거리를 STOP config로 고정하고 d는
    // 버린다. v1 STOP도 이 함수를 쓰고 있으므로 동작을 바꾸지 않는다
    // (ProjectPointToFrenet 헤더 주석 참고).
    double s = 0.0;
    double ignored_d = 0.0;
    if (!ProjectPointToFrenet(x, y, s_hint,
                              stop_cfg_.projection_search_window,
                              stop_cfg_.max_lateral_offset, s, ignored_d)) {
        return false;
    }
    out_s = s;
    return true;
}

bool FrenetPlanner::ReferenceHeadingAt(double s, double& out_heading) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(s)) return false;
    // 폐루프 트랙이라 s는 [0, s_max]로 감아서 조회한다. Interpolate가 범위
    // 밖 s에 대해 무엇을 반환하는지에 의존하지 않도록 여기서 정규화한다.
    const double s_max = ref_.points.back().s;
    if (!(s_max > 0.0)) return false;
    double s_wrapped = std::fmod(s, s_max);
    if (s_wrapped < 0.0) s_wrapped += s_max;

    const RefPoint rp = Interpolate(ref_, s_wrapped);
    if (!std::isfinite(rp.theta)) return false;
    out_heading = rp.theta;
    return true;
}

bool FrenetPlanner::ProjectEgoToS(const CartesianState& ego, double& out_s) const {
    if (!ref_loaded_ || ref_.points.empty()) return false;
    if (!std::isfinite(ego.x) || !std::isfinite(ego.y)) return false;
    const double s = FindClosestS(ref_, ego.x, ego.y,
                                  has_last_s_ ? &last_s_ : nullptr);
    if (!std::isfinite(s)) return false;
    out_s = s;
    return true;
}

namespace {

// 폐루프 트랙에서 current_s -> target_s의 전방 호길이.
// 단순 (target - current)는 finish line 이음새에서 음수로 크게 튄다
// (예: ego_s=2200, target_s=20이면 실제로는 전방 30m인데 -2180m).
double ForwardArcDistance(double current_s, double target_s, double s_max) {
    double ds = target_s - current_s;
    if (!std::isfinite(ds)) return ds;
    if (s_max > 0.0) {
        // 이음새를 갓 지난 목표(작은 음수)는 "뒤"가 아니라 한 바퀴 앞이 아니다.
        // 절반 이상 뒤로 떨어진 경우만 wrap으로 간주해 전방 거리로 되돌린다.
        if (ds < -0.5 * s_max) ds += s_max;
        else if (ds > 0.5 * s_max) ds -= s_max;
    }
    return ds;
}

// 외부 command로 실제 열려 있는 모드. core에는 FOLLOWING/AVOID/MERGE/LANE_CHANGE
// 구현이 이미 있지만, ROS 계약(leader binding, target_d, merge decision)이 FSM과
// 확정되기 전까지 production 진입점에서는 KEEP/STOP만 허용한다. 확정된 모드부터
// 여기에 case를 추가하고 ValidateExternalCommand에 필드 검증을 같이 넣는다.
bool IsSupportedExternalMode(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING:
        case STOP:
        case EMERGENCY:
        case FOLLOWING:
        // D6-1: AVOID는 대상 확보(resolver)까지만 열렸다. 후보 생성은 D6-2다.
        // 여기서 열지 않으면 resolver를 검증할 수 없고, 열어도 아래
        // ResolveManeuver 직전에서 UNSUPPORTED_BEHAVIOR로 막으므로
        // 조용히 velocity keeping 궤적이 나가지는 않는다.
        //
        // node의 SetImplementedManeuver(AVOID)는 계속 닫혀 있으므로 ROS
        // 경로로는 여기까지 도달하지 않는다(계약 12.7.1절).
        case AVOID:
            return true;

        default:
            return false;
    }
}

bool IsValidOptionalSpeed(bool has_value, double value) {
    if (!has_value) return true;
    return std::isfinite(value) && value >= 0.0;
}

// 모드 whitelist + 모드별 필수 필드 검증. 실패 시 reason으로 원인을 구분해
// 호출자가 "즉시 정지"와 "이전 궤적 재사용"을 나눌 수 있게 한다.
bool ValidateExternalCommand(const PlannerCommand& command,
                             FrenetPlanner::PlanFailureReason& reason) {
    if (!IsSupportedExternalMode(command.mode)) {
        reason = FrenetPlanner::PlanFailureReason::UNSUPPORTED_BEHAVIOR;
        return false;
    }

    // 전 모드 공통: 외부가 지정한 속도는 유한하고 음수가 아니어야 한다.
    if (!IsValidOptionalSpeed(command.has_target_speed, command.target_speed) ||
        !IsValidOptionalSpeed(command.has_max_speed, command.max_speed)) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // EMERGENCY는 stop_position을 요구하지 않는다. BehaviorCommand가 목표점
    // 없이(has_target_pose=false) 즉시 긴급정지를 지시하므로 정지 위치 계산
    // 소유권이 Frenet에 있다(계약 8.1절). 아래 STOP 검증을 EMERGENCY에
    // 적용하면 정상 긴급정지가 INVALID_COMMAND로 거부된다.

    // FOLLOWING: 추종 대상이 지정되어야 한다. v2에서 대상 선택 권한은 FSM에
    // 있으므로(BehaviorCommand.primary_object_id) planner가 스스로 고르지
    // 않는다. -1이면 무엇을 따라갈지 알 수 없다.
    if (command.mode == FOLLOWING && command.target_object_id < 0) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // AVOID: 회피 대상이 지정되어야 한다. FOLLOWING과 같은 이유다 - v2에서
    // 대상 선택 권한은 FSM에 있고(primary_object_id), planner가
    // FindStaticAvoidanceTarget()으로 스스로 고르지 않는다(D6-1).
    if (command.mode == AVOID && command.target_object_id < 0) {
        reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    // STOP: 정지 목표 s가 실제로 지정되어야 한다. 기본값 NaN이 그대로 오면
    // GenerateStoppingCandidates가 NaN 경계조건으로 후보를 만들게 된다.
    if (command.mode == STOP) {
        if (!std::isfinite(command.stop_position) || command.stop_position < 0.0) {
            reason = FrenetPlanner::PlanFailureReason::INVALID_COMMAND;
            return false;
        }
    }

    reason = FrenetPlanner::PlanFailureReason::NONE;
    return true;
}

}  // namespace

bool FrenetPlanner::Plan(const CartesianState& ego,
                         const std::vector<ObjectInfo>& obstacles,
                         CartesianPath& out_path) {
    return PlanInternal(ego, obstacles, nullptr, nullptr,
                        ExternalPlanningPurpose::NORMAL, out_path, nullptr);
}

bool FrenetPlanner::Plan(const CartesianState& ego,
                         const std::vector<ObjectInfo>& obstacles,
                         const PlannerCommand& command,
                         CartesianPath& out_path) {
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    return PlanInternal(ego, obstacles, &command, nullptr,
                        ExternalPlanningPurpose::NORMAL, out_path, nullptr);
}

bool FrenetPlanner::Plan(
    const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
    const std::vector<TrackedObstacle>& collision_obstacles,
    const PlannerCommand& command, CartesianPath& out_path) {
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    // collision geometry는 ObjectInfo와 1:1로 대응해야 한다. 어긋나면 command가
    // 아니라 호출자의 입력 조립이 잘못된 것이므로 역시 즉시 거부한다.
    if (collision_obstacles.size() != obstacles.size()) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    return PlanInternal(ego, obstacles, &command, &collision_obstacles,
                        ExternalPlanningPurpose::NORMAL, out_path, nullptr);
}

void FrenetPlanner::SelectLegacyBehavior(
    bool begin_avoidance, bool nearest_found, const LeaderTarget& nearest_leader,
    PlannerCommand& cmd, double& out_avoid_offset, double& out_leader_s,
    double& out_leader_speed, double& out_leader_accel) const {
    // D1-2 순수 추출: 아래 본문은 PlanInternal의 legacy 분기에서 그대로 옮겨온
    // 것이며 동작을 바꾸지 않았다. 지역변수 avoid_offset/leader_* 만 출력
    // 파라미터로 이름이 바뀌었다.
    double& avoid_offset = out_avoid_offset;
    double& leader_s = out_leader_s;
    double& leader_speed = out_leader_speed;
    double& leader_accel = out_leader_accel;
    if (begin_avoidance || (avoidance_.active && avoidance_.phase != AvoidPhase::TRACKING)) {
        cmd.mode = AVOID;
        avoid_offset = (avoidance_.phase == AvoidPhase::SHIFT || avoidance_.phase == AvoidPhase::PASS)
                     ? avoidance_.target_d : 0.0;
        cmd.avoidance_d_offset = avoid_offset;
    } else if (merge_.active) {
        const bool highway = merge_.type == MergeType::HIGHWAY;
        const HighwayMergeZone* active_highway_zone = HighwayZoneAt(
            highway_merge_cfg_, merge_.highway_zone_index);
        const HighwayMergeCheckpoint* active_highway_checkpoint = HighwayCheckpointAt(
            highway_merge_cfg_, merge_.highway_zone_index, merge_.highway_conflict_index);
        const double active_conflict_s = highway
            && active_highway_checkpoint
                ? active_highway_checkpoint->conflict_s : merge_cfg_.conflict_s;
        const double active_stop_buffer = highway
            ? highway_merge_cfg_.stop_buffer : merge_cfg_.stop_buffer;
        cmd.mode = MERGE;
        cmd.target_lane = 0;
        cmd.merge_target_d = 0.0;
        cmd.merge_gap_safe = merge_.gap_safe;
        cmd.merge_conflict_s = active_conflict_s;
        cmd.merge_stop_s = active_conflict_s - active_stop_buffer;
        cmd.merge_entry_time = merge_.entry_time;
        cmd.merge_sa_id = merge_.preceding_id;
        cmd.merge_sb_id = merge_.following_id;
        cmd.merge_committed = merge_.gap_locked || merge_.commit_pending;
        cmd.merge_crossing = merge_.phase == MergePhase::CROSS;
        cmd.merge_type = merge_.type;
        // 합류 후 같은 corridor에 들어온 선행차는 일반 장애물이 아니라
        // FOLLOWING과 동일한 종방향 gap 제어 대상으로 전달한다.
        if (nearest_found) {
            cmd.target_object_id = nearest_leader.id;
            cmd.leader_s = nearest_leader.s;
            cmd.leader_speed = nearest_leader.speed;
            cmd.leader_accel = nearest_leader.accel;
            cmd.time_gap = following_cfg_.time_gap;
            cmd.min_gap = following_cfg_.min_gap;
            cmd.gap_gain = following_cfg_.gap_gain;
        } else {
            cmd.target_object_id = -1;
        }
    } else if (following_.active) {
        leader_s = following_.leader_s;
        leader_speed = following_.leader_speed;
        leader_accel = following_.leader_accel;
        cmd.mode = FOLLOWING;
        cmd.target_object_id = following_.leader_id;
        cmd.leader_s = leader_s;
        cmd.leader_speed = leader_speed;
        cmd.leader_accel = leader_accel;
        cmd.time_gap = following_cfg_.time_gap;
        cmd.min_gap = following_cfg_.min_gap;
        cmd.gap_gain = following_cfg_.gap_gain;
    } else {
        cmd.mode = LANE_KEEPING;
    }
}

bool FrenetPlanner::PrepareAvoid(
    const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
    const std::vector<TrackedObstacle>& collision_obstacles,
    const PlannerCommand& command, CartesianPath& maintenance_path,
    AvoidPrepareResult& result) {
    result = AvoidPrepareResult{};
    PlanFailureReason reason = PlanFailureReason::NONE;
    if (!ValidateExternalCommand(command, reason)) {
        last_failure_reason_ = reason;
        return false;
    }
    // PREPARE는 AVOID 전용이다. 다른 모드로 호출되면 호출자의 실수다.
    if (command.mode != AVOID) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    if (collision_obstacles.size() != obstacles.size()) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }
    return PlanInternal(ego, obstacles, &command, &collision_obstacles,
                        ExternalPlanningPurpose::AVOID_PREPARE,
                        maintenance_path, &result);
}

void FrenetPlanner::BuildZeroSpeedHoldPath(const CartesianState& ego, double s,
                                           CartesianPath& out_path) const {
    // STOP HOLD와 같은 형태다. 한 점 0속도 궤적.
    const RefPoint rp = Interpolate(ref_, s);
    out_path.x.assign(1, ego.x);
    out_path.y.assign(1, ego.y);
    out_path.yaw.assign(1, std::isfinite(ego.yaw) ? ego.yaw : rp.theta);
    out_path.kappa.assign(1, 0.0);
    out_path.v.assign(1, 0.0);
    out_path.a.assign(1, 0.0);
}

bool FrenetPlanner::BuildAvoidMaintenancePath(
    const CartesianState& ego, const FrenetState& start,
    const PlannerCommand& base_command,
    const std::vector<TrackedObstacle>& collision_obstacles,
    CartesianPath& out_path) const {
    const double s = start.s;
    const double d = start.d;

    // ---- 1순위: 충돌 검사를 통과한 유지 궤적 --------------------------
    //
    // 저속에서는 정상 후보 파이프라인이 lateral candidate를 제한하므로
    // crawl을 쓴다. 정상 속도에서는 중앙 유지(KEEP) 후보를 만든다.
    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s] (PlanInternal과 동일)
    const bool low_speed = std::fabs(ego.v) < kLowSpeedFallbackV;

    if (low_speed) {
        // crawl은 현재 d를 유지한다. PREPARE 중에 횡이동을 시작하면 안 된다.
        CartesianPath crawl;
        if (PlanLowSpeedFallback(ego, s, d, d, crawl, 2.0,
                                 base_command.target_speed) &&
            crawl.x.size() >= 2) {
            // PlanLowSpeedFallback은 충돌 필터를 타지 않는다. 그대로 쓰면
            // 장애물 쪽으로 기어간다.
            int blocking_id = -1;
            if (CartesianTrajectoryClear(crawl, collision_obstacles,
                                         vehicle_shape_, collision_cfg_,
                                         path_cfg_.dt, &blocking_id)) {
                out_path = std::move(crawl);
                return true;
            }
        }
    } else {
        PlannerCommand keep_cmd = base_command;
        keep_cmd.mode = LANE_KEEPING;
        keep_cmd.avoidance_d_offset = 0.0;
        PlannerDebugStats keep_stats;
        auto keep_candidates =
            ResolveManeuver(start, keep_cmd, ref_, path_cfg_, limits_,
                            lane_width_, collision_obstacles, vehicle_shape_,
                            collision_cfg_, &keep_stats);
        EvaluateCosts(keep_candidates, cost_weights_, 0.0);
        FilterByCartesianLateralAcceleration(keep_candidates, ref_,
                                             limits_.max_lateral_accel);
        const FrenetPath* keep_best = SelectBestPath(keep_candidates);
        if (keep_best) {
            out_path = ConvertToCartesianPath(*keep_best, ref_);
            if (out_path.x.size() >= 2) return true;
        }
    }

    // ---- 2순위: 종방향 감속 -------------------------------------------
    //
    // 여기 오는 것은 정상이다 - AVOID 대상이 같은 corridor에 있으므로 직진
    // 후보가 그 장애물 때문에 전부 막힐 수 있다.
    CartesianPath decel;
    if (GenerateDecelToZeroPath(s, d, start.s_d, decel) &&
        decel.x.size() >= 2) {
        int blocking_id = -1;
        if (CartesianTrajectoryClear(decel, collision_obstacles, vehicle_shape_,
                                     collision_cfg_, path_cfg_.dt,
                                     &blocking_id)) {
            out_path = std::move(decel);
            return true;
        }
    }

    // ---- 3순위: 0속도 hold --------------------------------------------
    //
    // 감속 궤적조차 충돌로 막히면 그 자리에 멈추는 것이 마지막 수단이다.
    // 이 경로는 이동하지 않으므로 새 충돌을 만들지 않는다.
    BuildZeroSpeedHoldPath(ego, s, out_path);
    return !out_path.x.empty();
}

bool FrenetPlanner::PlanInternal(
    const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
    const PlannerCommand* external_command,
    const std::vector<TrackedObstacle>* collision_obstacles,
    ExternalPlanningPurpose purpose, CartesianPath& out_path,
    AvoidPrepareResult* avoid_prepare_result) {
    const bool use_legacy_behavior = external_command == nullptr;
    // purpose가 계산 내용을 정한다. 포인터 유무로 판단하지 않는다 - 진단용
    // 으로 포인터만 넘겨도 동작이 바뀌면 안 된다(헤더 주석 참고).
    const bool avoid_prepare =
        purpose == ExternalPlanningPurpose::AVOID_PREPARE;
    last_failure_reason_ = PlanFailureReason::NONE;
    if (!ref_loaded_) {
        last_failure_reason_ = PlanFailureReason::NOT_INITIALIZED;
        return false;
    }

    double s, s_dot, s_ddot, d, d_prime, d_pprime;
    CartesianToFrenet(ref_, ego, s, s_dot, s_ddot, d, d_prime, d_pprime,
                       has_last_s_ ? &last_s_ : nullptr);
    last_s_ = s;
    has_last_s_ = true;

    // ObjectInfo의 연속 heading으로 yaw-rate를 추정한다. MORAI 객체의 heading은
    // 회전교차로에서 계속 변하므로 이를 보존해야 접선 직선 예측의 누락을 피할 수 있다.
    std::unordered_map<int, double> object_yaw_rates;
    for (auto& item : object_tracks_) ++item.second.missing_cycles;
    for (const auto& obj : obstacles) {
        auto it = object_tracks_.find(obj.id);
        if (it == object_tracks_.end()) {
            object_tracks_[obj.id] = ObjectTrack{obj.heading, 0.0, 0};
        } else {
            const double measured = std::remainder(obj.heading - it->second.heading,
                                                    2.0 * M_PI) / 0.05;
            // UDP heading 양자화/순간 점프를 억제하고 차량 수준의 yaw-rate로 제한한다.
            const double bounded = clip(measured, -1.0, 1.0);
            it->second.yaw_rate = 0.8 * it->second.yaw_rate + 0.2 * bounded;
            it->second.heading = obj.heading;
            it->second.missing_cycles = 0;
        }
        object_yaw_rates[obj.id] = object_tracks_[obj.id].yaw_rate;
    }
    for (auto it = object_tracks_.begin(); it != object_tracks_.end();) {
        if (it->second.missing_cycles > 20) it = object_tracks_.erase(it);
        else ++it;
    }
    std::vector<ObjectInfo> predicted_obstacles = obstacles;
    for (auto& obj : predicted_obstacles) {
        const auto it = object_yaw_rates.find(obj.id);
        obj.yaw_rate = it == object_yaw_rates.end() ? 0.0 : it->second;
    }
    // ROS/LiDAR 경로는 perception이 준 corners를 그대로 사용한다. Legacy API만
    // ObjectInfo의 center/dimension/yaw로 기존 adapter geometry를 생성한다.
    const std::vector<TrackedObstacle> predicted_collision_obstacles =
        collision_obstacles ? *collision_obstacles
                            : MakeLegacyTrackedObstacles(predicted_obstacles);

    double resolved_target_speed = LookaheadTargetSpeed(
        ref_, s, ego.v, curve_speed_cfg_, limits_.max_longitudinal_accel,
        limits_.max_lateral_accel, limits_.max_curvature);
    if (external_command && external_command->has_target_speed) {
        resolved_target_speed = std::min(resolved_target_speed,
                                         external_command->target_speed);
    }
    if (external_command && external_command->has_max_speed) {
        resolved_target_speed = std::min(resolved_target_speed,
                                         external_command->max_speed);
    }

    // 저속(정지 근처)에서는 quintic 후보/곡률필터 파이프라인 전체를 건너뛰고
    // fallback으로 직접 경로를 만든다 (클래스 헤더 주석 참고 - 나누기 없는 안전한
    // 경로). AVOID/장애물 회피는 이 fallback에서는 다루지 않는다 - 정지 근처
    // 속도로는 위험한 회피 기동이 필요할 상황이 원래 적고, 속도가 오르면 바로
    // 아래의 정상 파이프라인으로 자동 복귀한다.
    last_d_ = d;

    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s]
    const HighwayMergeZone* context_highway_zone = use_legacy_behavior
        ? HighwayZoneAt(highway_merge_cfg_, merge_.highway_zone_index) : nullptr;
    const HighwayMergeCheckpoint* context_highway_checkpoint = use_legacy_behavior
        ? HighwayCheckpointAt(highway_merge_cfg_, merge_.highway_zone_index,
                              merge_.highway_conflict_index) : nullptr;
    const double context_conflict_s = use_legacy_behavior &&
        merge_.type == MergeType::HIGHWAY
        && context_highway_checkpoint
            ? context_highway_checkpoint->conflict_s : merge_cfg_.conflict_s;
    const double context_stop_buffer = use_legacy_behavior &&
        merge_.type == MergeType::HIGHWAY
        ? highway_merge_cfg_.stop_buffer : merge_cfg_.stop_buffer;
    const double low_speed_conflict_gap = merge_cfg_.conflict_s - s;
    const bool near_roundabout = merge_cfg_.conflict_s >= 0.0 &&
        low_speed_conflict_gap >= -merge_cfg_.completion_distance &&
        low_speed_conflict_gap <= merge_cfg_.approach_distance;
    const bool near_highway_merge = FindHighwayMergeZone(highway_merge_cfg_, s) >= 0;
    const bool near_any_merge = use_legacy_behavior &&
        (near_roundabout || near_highway_merge || merge_.active);

    // COMMIT 이후에는 noisy localization 상태로 exact-arrival 다항식을 매 cycle
    // 재생성하지 않는다. 최초 COMMIT 때 conflict zone 이탈까지 OBB 검증한 시간
    // 궤적을 시간에 맞춰 잘라 그대로 추종한다. 실제 동적 충돌이 새로 예측될 때만
    // 정지선 전에는 WAIT로 취소하고, 정지선 이후에는 비상정지를 요청한다.
    if (use_legacy_behavior && merge_.active && merge_.gap_locked &&
        merge_.phase == MergePhase::COMMIT) {
        if (s >= context_conflict_s) {
            const HighwayMergeCheckpoint* next_checkpoint =
                merge_.type == MergeType::HIGHWAY
                    ? HighwayCheckpointAt(highway_merge_cfg_, merge_.highway_zone_index,
                                          merge_.highway_conflict_index + 1)
                    : nullptr;
            if (next_checkpoint) {
                ++merge_.highway_conflict_index;
                merge_.gap_locked = false;
                merge_.commit_pending = false;
                merge_.gap_safe = false;
                merge_.safe_cycles = 0;
                merge_.phase = MergePhase::APPROACH;
                merge_.locked_path = CartesianPath{};
                merge_.locked_elapsed = 0.0;
                merge_.locked_path_index = 0;
                std::printf("[FrenetPlanner-MERGE] highway checkpoint advanced: "
                            "zone=%s index=%d next_conflict=%.2f s=%.2f\n",
                            context_highway_zone ? context_highway_zone->name.c_str() : "-",
                            merge_.highway_conflict_index,
                            next_checkpoint->conflict_s, s);
            } else {
                merge_.phase = MergePhase::CROSS;
                merge_.locked_path = CartesianPath{};
            }
        } else if (!merge_.locked_path.x.empty()) {
            CartesianPath remaining = RemainingPath(
                merge_.locked_path, ego.x, ego.y, merge_.locked_path_index);
            int blocking_id = -1;
            const bool clear = CartesianTrajectoryClear(
                remaining, predicted_collision_obstacles, vehicle_shape_, collision_cfg_,
                path_cfg_.dt, &blocking_id);
            if (!clear) {
                if (s < context_conflict_s - context_stop_buffer) {
                    std::printf("[FrenetPlanner-MERGE] locked trajectory emergency recheck "
                                "blocked before yield: object_id=%d; returning to WAIT\n",
                                blocking_id);
                    merge_.gap_locked = false;
                    merge_.gap_safe = false;
                    merge_.safe_cycles = 0;
                    merge_.phase = MergePhase::WAIT;
                    merge_.locked_path = CartesianPath{};
                    merge_.locked_elapsed = 0.0;
                    merge_.locked_path_index = 0;
                } else {
                    std::printf("[FrenetPlanner-MERGE] EMERGENCY: locked trajectory blocked "
                                "after yield by object_id=%d\n", blocking_id);
                    last_failure_reason_ = PlanFailureReason::COLLISION_BLOCKED;
                    return false;
                }
            } else if (remaining.x.size() >= 2) {
                out_path = std::move(remaining);
                static int locked_log_tick = 0;
                if (++locked_log_tick % 5 == 0) {
                    std::printf("[FrenetPlanner-STATE] mode=MERGE phase=COMMIT_LOCKED "
                                "s=%.2f d=%.3f speed=%.2f progress=%zu remaining=%zu\n",
                                s, d, ego.v, merge_.locked_path_index, out_path.x.size());
                }
                return true;
            } else {
                std::printf("[FrenetPlanner-MERGE] locked trajectory exhausted before conflict "
                            "(s=%.2f); requesting safe stop\n", s);
                last_failure_reason_ = PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
                return false;
            }
        }
    }
    // 회전교차로에서는 저속이어도 아래의 MERGE FSM을 반드시 거친다. 여기서
    // 조기 반환하면 WAIT/COMMIT 잠금보다 먼저 매 cycle 전체 도로가 빌 때까지
    // 검사하게 되어 conflict point 앞에서 두 번째 정지와 데드락이 발생한다.
    // 저속 fallback은 kMinCreepSpeed로 "기어서 전진"하는 경로다. 외부 STOP
    // 명령에서 이 분기를 타면 정지 목표를 넘어 계속 기어나가게 되므로, STOP은
    // 저속에서도 정상 stopping 후보 생성기(GenerateStoppingCandidates)로 보낸다.
    // FOLLOW가 아닌 외부 명령으로 넘어가면 v2 FOLLOW 상태를 즉시 비운다.
    //
    // **모든 early-return보다 앞에서** 해야 한다. 아래 EMERGENCY 분기와 저속
    // fallback은 명령 조립 지점에 도달하기 전에 return하므로, 그곳에서만
    // reset하면 context가 살아남는다. 그러면
    //
    //   FOLLOW id=10 target=15   -> 15 확보
    //   E-STOP id=11             -> context가 남는다
    //   FOLLOW id=12 target=15   -> 15가 관측되지 않아도 GRACE
    //
    // 가 되어, 첫 binding 실패인데 유예를 받아 존재하지 않는 leader를
    // 예측 추종한다(실측으로 재현 확인). legacy following_에는 영향이 없다.
    if (!use_legacy_behavior && external_command->mode != FOLLOWING) {
        v2_following_ = FollowingContext{};
    }
    // v2 AVOID 상태도 같은 이유로 같은 위치에서 비운다(D6-1). 여기서
    // reset하지 않으면
    //   AVOID id=20 target=21   -> 21 확보
    //   E-STOP id=21            -> context가 남는다
    //   AVOID id=22 target=21   -> 21이 관측되지 않아도 GRACE
    // 가 되어 첫 binding 실패인데 유예를 받는다(D5-2에서 FOLLOW로 실측 확인한
    // 것과 같은 구조). legacy avoidance_에는 영향이 없다.
    if (!use_legacy_behavior && external_command->mode != AVOID) {
        v2_avoidance_ = AvoidanceContext{};
        // 목표 목록도 함께 비운다(D6-2a). 남겨두면 다음 AVOID가 확보에
        // 실패했는데 이전 사이클의 목표로 후보를 만들 수 있다.
        v2_avoid_targets_ = AvoidTargetPlan{};
    }

    // =====================================================================
    // 외부 EMERGENCY_STOP: 후보 파이프라인을 건너뛰고 직접 감속한다.
    //
    // BehaviorCommand는 목표점 없이(has_target_pose=false) 즉시 긴급정지를
    // 지시하므로 정지 위치 계산 소유권이 Frenet에 있다(계약 8.1절).
    //
    // ResolveManeuver / 곡률 필터 / 충돌 필터로 내려보내지 않는 이유:
    //   1. stop_position이 NaN이므로 GenerateStoppingCandidates가 NaN
    //      경계조건으로 후보를 만든다.
    //   2. 후보가 전부 탈락하면 "긴급정지인데 아무 궤적도 못 보냄"이 된다.
    //      긴급정지에서 그 실패 방향 자체를 제거하는 것이 목적이다.
    //
    // 저속 fallback보다 먼저 처리한다. 그 fallback은 kMinCreepSpeed로 기어서
    // 전진하는 경로라 긴급정지 중에 차량이 계속 움직이게 된다.
    //
    // 차량이 이미 멈춰 있으면(v0 ~= 0) 0속도 궤적이 나온다. 새 정상 명령이
    // 올 때까지 긴급정지 상태를 계속 강제하는 것이 E-stop의 의미이므로,
    // 여기서 COMPLETED로 올리지 않는다 - ROS 계층이 EXECUTING을 유지한다.
    if (!use_legacy_behavior && external_command->mode == EMERGENCY) {
        last_mode_ = EMERGENCY;
        last_target_speed_ = 0.0;
        if (!GenerateDecelToZeroPath(s, d, s_dot, out_path)) {
            last_failure_reason_ = PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            return false;
        }
        static int emergency_log_tick = 0;
        if (++emergency_log_tick % 5 == 0) {
            std::printf("[FrenetPlanner-STATE] mode=EMERGENCY_STOP s=%.2f d=%.3f "
                        "speed=%.2f samples=%zu\n",
                        s, d, ego.v, out_path.v.size());
        }
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    // =====================================================================
    // 외부 AVOID 대상 확보 (D6-2a)
    //
    // ===== 왜 여기인가 =====
    //
    // D6-1에서는 이 처리가 명령 조립 지점(아래 external_command 분기)에
    // 있었다. 그런데 그 지점은 저속 fallback보다 **아래**다. 그래서
    // ego.v < kLowSpeedFallbackV(0.5 m/s)이면 fallback이 먼저 return해
    // resolver에 도달하지 않았다. 실측(AVOID target=21, 장애물 25 m 전방):
    //
    //   ego_v  Plan()  active_avoid_target_id  failure_reason
    //   0.60   false   21                      UNSUPPORTED_BEHAVIOR
    //   0.40   true    -1                      NONE        <- 확보 없이 성공
    //   0.00   true    -1                      NONE
    //
    // 저속 세 행은 Plan()이 성공을 반환하면서 대상을 확보하지 않았고, 나오는
    // 궤적은 fallback_target_d=0.0의 중앙 크롤링 경로다. gate를 열면 FSM
    // 입장에서 "회피 성공"으로 보이면서 장애물 쪽으로 기어간다.
    //
    // 그래서 확보를 저속 판단보다 위로 올린다. 장애물 앞에서 감속한 뒤
    // AVOID 명령을 받는 것은 오히려 **정상적인 시나리오**이므로, 그때
    // 대상조차 확보하지 못하는 구조는 맞지 않다(계약 12.7.2절).
    //
    // 확보만 여기서 하고, "그 목표로 실제 후보가 나오는가"는 D6-2b다.
    if (!use_legacy_behavior && external_command->mode == AVOID) {
        // 요청 방향을 context에 보존한다. mode=AVOID 하나로는
        // LEFT/RIGHT/NONE을 구분할 수 없다(계약 3.3.1절).
        v2_avoidance_.requested_direction =
            external_command->requested_direction;

        // target pose 투영 결과도 보존한다 (D6-1a). node가 이미 Frenet
        // (s, d)로 바꿔서 [A] 필드에 넣어 보냈다.
        //
        // avoidance_d_offset([B])에 옮기지 않는다. 그러면 "FSM이 지정한
        // 목표"와 "planner가 후보 비교로 정한 목표"를 구분할 수 없게 된다
        // (계약 3.3.3절).
        v2_avoidance_.has_requested_target_d =
            external_command->has_avoid_target_d;
        v2_avoidance_.requested_target_d =
            external_command->has_avoid_target_d
                ? external_command->avoid_target_d : 0.0;
        v2_avoidance_.requested_target_s =
            external_command->has_avoid_target_d
                ? external_command->avoid_target_s : 0.0;

        // FindStaticAvoidanceTarget()을 타지 않는다 - 그 함수는 가장 가까운
        // 정적 장애물과 회피 방향까지 자동 결정하는 legacy policy다
        // (계약 D1 0.1절 B 분류, 6.4절).
        const AvoidTargetStatus status = ResolveExternalAvoidTarget(
            s, external_command->target_object_id, obstacles, v2_avoidance_);
        if (status == AvoidTargetStatus::LOST) {
            // 대상을 쓸 수 없다. KEEP으로 자동 전환하지 않는다 - 다음 행동은
            // FSM이 새 command_id로 결정한다(계약 8.2절).
            //
            // D6-3에서 COMMIT 경계가 들어오면 COMMITTED/EXECUTING 중에는 이
            // 분기로 오지 않는다(계약 8.2.1절).
            // 목표 목록도 비운다 - 확보하지 못한 대상의 목표는 없다.
            v2_avoid_targets_ = AvoidTargetPlan{};
            last_mode_ = AVOID;
            last_failure_reason_ = PlanFailureReason::TARGET_OBJECT_LOST;
            return false;
        }

        // 확보 성공. 평가할 목표 d까지 정해 둔다 - 저속이든 정상속도든
        // 같은 결정 규칙을 쓴다.
        v2_avoid_targets_ =
            DecideAvoidTargets(v2_avoidance_, external_command->requested_direction);

        // 후보 생성과 안전성 판정은 아래 AVOID PREPARE 블록에서 한다
        // (D6-2b). 그 블록은 FrenetState start가 만들어진 뒤에 있다.
        //
        // NORMAL purpose로 AVOID가 들어오면 아직 실행 경로가 없다. 그대로
        // 내려보내면 ResolveManeuver의 AVOID 분기가 avoidance_d_offset=0으로
        // 중앙 유지 궤적을 만들어, FSM 입장에서는 "회피 성공"으로 보이면서
        // 실제로는 장애물 쪽으로 직진한다(계약 11.4절과 같은 원칙).
        // 실제 실행은 D6-3에서 붙인다.
        if (!avoid_prepare) {
            last_mode_ = AVOID;
            last_failure_reason_ = PlanFailureReason::UNSUPPORTED_BEHAVIOR;
            return false;
        }
    }
    // PREPARE인데 AVOID가 아니면 호출자의 실수다. PrepareAvoid()가 이미
    // 막지만, PlanInternal 내부에서도 조용히 다른 모드를 준비하지 않는다.
    if (avoid_prepare &&
        (use_legacy_behavior || external_command->mode != AVOID)) {
        last_failure_reason_ = PlanFailureReason::INVALID_COMMAND;
        return false;
    }

    const bool external_stop = !use_legacy_behavior && external_command->mode == STOP;
    // 외부 FOLLOW도 저속 fallback을 타지 않는다. 그 경로는 kMinCreepSpeed로
    // 기어서 전진하는데, 앞차가 정지해 있으면 그대로 다가가 붙는다. FOLLOW의
    // 종방향 목표는 선행차와의 gap이므로 정상 following 후보 생성기
    // (GenerateFollowingCandidates)로 보내야 한다 - 그 생성기는 leader가
    // 멈춰 있으면 min_gap을 유지하는 정지 목표를 만든다.
    const bool external_follow =
        !use_legacy_behavior && external_command->mode == FOLLOWING;
    // AVOID PREPARE도 이 fallback을 타지 않는다 (D6-2b).
    //
    // 이 경로는 충돌 필터를 거치지 않은 crawl을 그대로 반환하므로, PREPARE의
    // 유지 궤적으로 쓰면 장애물 쪽으로 기어간다. 저속 PREPARE의 유지 궤적은
    // BuildAvoidMaintenancePath가 **충돌 검사를 통과시킨 뒤** 만든다.
    //
    // 여기서 제외하지 않으면 PREPARE 블록에 도달하지도 못한다 - 이 분기가
    // 먼저 return하기 때문이다(실측으로 확인: v=0.3에서 low_speed_context이
    // 0으로 나오고 maintenance가 미검사 crawl이었다).
    if (std::fabs(ego.v) < kLowSpeedFallbackV && !near_any_merge &&
        !external_stop && !external_follow && !avoid_prepare) {
        static int low_speed_log_tick = 0;
        if (++low_speed_log_tick % 5 == 0) {
            std::printf("[FrenetPlanner-STATE] mode=LOW_SPEED_FALLBACK "
                        "s=%.2f d=%.3f speed=%.2f candidates=N/A\n",
                        s, d, ego.v);
        }
        // TRACKING 근거리에서 한 점짜리 v=0 경로를 무기한 보내던 분기는
        // ego.v==0 -> 같은 조기 return이라는 자기잠금을 만들었다. 정적 장애물을
        // 등록할 때 검증한 1차 회피 방향을 저속 경로의 목표로 사용해 재출발한다.
        // 정상 속도로 회복되면 기존 좌/우 전체 후보 비교가 방향을 다시 확정한다.
        const bool tracking_near = use_legacy_behavior && avoidance_.active &&
            avoidance_.phase == AvoidPhase::TRACKING &&
            avoidance_.obstacle_s - s <= avoid_cfg_.shift_start_distance;
        const bool hold_avoid_target = use_legacy_behavior && avoidance_.active &&
            (tracking_near || avoidance_.phase == AvoidPhase::SHIFT ||
             avoidance_.phase == AvoidPhase::PASS);
        const double fallback_target_d = hold_avoid_target ? avoidance_.target_d : 0.0;
        if (!use_legacy_behavior) {
            last_mode_ = external_command->mode;
            last_target_speed_ = resolved_target_speed;
        }
        const bool fallback_ok = PlanLowSpeedFallback(
            ego, s, d, fallback_target_d, out_path, 2.0,
            use_legacy_behavior
                ? std::numeric_limits<double>::infinity()
                : resolved_target_speed);
        return fallback_ok;
    }

    double d_dot, d_ddot;
    ArcDerivToTimeDeriv(s_dot, s_ddot, d_prime, d_pprime, d_dot, d_ddot);
    last_d_dot_ = d_dot;  // 클램프 전 원본 값 - 진단 목적(아래 kMaxDDot 클램프 폭 자체를 보려고)

    // 투영 오차가 후보의 초기 가속도 경계조건을 비현실적으로 키우지 않도록
    // 다항식 중간구간의 overshoot 여유를 포함해 한계의 80%로 제한한다.
    constexpr double kBoundaryMargin = 0.8;
    s_ddot = clip(s_ddot, -limits_.max_longitudinal_accel * kBoundaryMargin,
                  limits_.max_longitudinal_accel * kBoundaryMargin);
    d_ddot = clip(d_ddot, -limits_.max_lateral_accel * kBoundaryMargin,
                  limits_.max_lateral_accel * kBoundaryMargin);

    // 큰 heading 오차로 과대 추정될 수 있는 초기 횡속도도 제한한다.
    constexpr double kMaxDDot = 0.5;  // [m/s]
    d_dot = clip(d_dot, -kMaxDDot, kMaxDDot);

    FrenetState start{s, s_dot, s_ddot, d, d_dot, d_ddot};

    // =====================================================================
    // AVOID PREPARE: 후보 평가와 유지 궤적을 한 사이클에서 만든다 (D6-2b)
    //
    // 계약 5.2절. 두 결과를 절대 같은 것으로 취급하지 않는다:
    //   candidate  "AVOID를 실행해도 되는가"     -> result.candidate_valid
    //   out_path   "지금 MPC가 따라갈 것"        -> maintenance
    //
    // 위쪽 전처리(Cartesian->Frenet, object tracking/prediction)를 **한 번만**
    // 통과했다. PlanInternal을 두 번 부르면 last_s_ / missing_cycles /
    // yaw-rate 재추정이 중복된다(계약 12.7.1절).
    // =====================================================================
    if (avoid_prepare) {
        AvoidPrepareResult& result = *avoid_prepare_result;
        last_mode_ = AVOID;
        last_target_speed_ = resolved_target_speed;
        result.direction_conflict = v2_avoid_targets_.direction_conflict;

        constexpr double kLowSpeedFallbackV = 0.5;  // [m/s]
        // 상황 정보다. 상태 판단(READY/WAITING)에는 쓰지 않는다 - 그것은
        // candidate_valid만으로 정한다(헤더 주석 참고).
        result.low_speed_context = std::fabs(ego.v) < kLowSpeedFallbackV;

        // ---- target pose sanity: ego 전방이어야 한다 -------------------
        //
        // 뒤에 있는 목표 pose의 d를 그대로 lateral goal로 쓰면, 이미 지나간
        // 지점의 횡위치로 회피하게 된다. 다만 "장애물보다 반드시 앞/뒤"
        // 같은 강한 규칙은 넣지 않는다 - AVOID 목표점은 shift 완료점일 수도,
        // 통과 후 점일 수도 있어 FSM 계약이 더 명확해지기 전에는 과하다
        // (계약 3.3.4절).
        if (v2_avoidance_.has_requested_target_d) {
            const double track_length =
                ref_.points.empty() ? 0.0 : ref_.points.back().s;
            double target_gap = v2_avoidance_.requested_target_s - s;
            if (target_gap <= 0.0 && track_length > 0.0) {
                target_gap += track_length;
            }
            // 폐루프에서 감아 올린 gap이 트랙 절반을 넘으면 실제로는 후방이다.
            const bool behind =
                !(target_gap > 0.0) ||
                (track_length > 0.0 && target_gap > track_length * 0.5);
            result.target_behind_ego = behind;
        }

        // ---- AVOID 후보 평가 -------------------------------------------
        //
        // 방향 모순이나 후방 목표이면 후보를 만들지 않는다.
        //
        // ===== 방향 모순을 충돌 검사에만 맡기면 안 되는 이유 =====
        //
        // FSM이 LEFT를 지시했고 pose가 장애물 오른쪽 -4 m라면, 그 오른쪽
        // 경로는 물리적으로 아주 안전할 수 있다. 충돌 검사는 CLEAR를
        // 돌려주고, 그대로 READY까지 올라가면
        //   FSM은 LEFT를 명령 / Frenet은 RIGHT 후보를 READY
        // 라는 계약 위반이 된다. 그래서 feasibility를 여기서 false로 만든다.
        //
        // 단 terminal 실패가 아니다. "현재 요청으로 실행 가능한 후보가
        // 없다"이므로 결과는 WAITING이다(계약 3.3.4절).
        const bool blocked_by_request =
            result.direction_conflict || result.target_behind_ego;

        // 저속이라고 후보 생성을 건너뛰지 않는다. "저속에서는 어차피 안 될
        // 것"이라고 가정하면, 실제로 가능한 상황에서도 READY에 도달하지
        // 못한다. 생성을 시도하고 결과로 판단한다 - 저속에서 후보가 0개면
        // low_speed_context이 그 사유를 설명한다.
        if (!blocked_by_request && v2_avoid_targets_.count > 0) {
            const FrenetPath* best_candidate = nullptr;
            std::vector<FrenetPath> best_set;
            double best_target_d = 0.0;
            int first_blocking_id = -1;

            for (int index = 0; index < v2_avoid_targets_.count; ++index) {
                const double target_d = v2_avoid_targets_.candidates[index];
                PlannerCommand side_cmd = *external_command;
                side_cmd.mode = AVOID;
                side_cmd.avoidance_d_offset = target_d;
                side_cmd.target_speed = resolved_target_speed;

                PlannerDebugStats side_stats;
                auto generated = ResolveManeuver(
                    start, side_cmd, ref_, path_cfg_, limits_, lane_width_,
                    predicted_collision_obstacles, vehicle_shape_,
                    collision_cfg_, &side_stats);
                EvaluateCosts(generated, cost_weights_, target_d);
                FilterByCartesianLateralAcceleration(generated, ref_,
                                                     limits_.max_lateral_accel);

                result.generated_candidate_count += generated.size();
                for (const auto& path : generated) {
                    if (path.valid) ++result.valid_candidate_count;
                    else if (first_blocking_id < 0 &&
                             path.rejection_reason == RejectionReason::COLLISION &&
                             path.collision_object_id >= 0) {
                        first_blocking_id = path.collision_object_id;
                    }
                }

                const FrenetPath* side_best = SelectBestPath(generated);
                if (!side_best) continue;
                // NONE이면 cost로 좌/우를 고른다. LEFT/RIGHT는 count가 1이라
                // 이 비교가 일어나지 않는다.
                if (!best_candidate ||
                    side_best->cost_total < best_candidate->cost_total) {
                    best_set = std::move(generated);
                    best_candidate = SelectBestPath(best_set);
                    best_target_d = target_d;
                }
            }

            if (best_candidate) {
                result.candidate_valid = true;
                // **요청 목표가 아니라 선택된 후보의 실제 종단 d를 보고한다.**
                //
                // ResolveManeuver의 lateral 격자는 target_d 주변
                // ±lateral_target_tolerance(0.5 m)이므로, 선택된 후보는 요청
                // 목표에서 그 범위 안으로 벗어날 수 있다. 실측: 요청 +2.50 ->
                // 실제 종단 +3.00.
                //
                // 요청값을 그대로 FrenetFeedback.target_d로 내보내면 FSM이
                // 실제 회피 폭을 0.5 m까지 잘못 알게 된다(계약 9.2절의
                // "지정했다 vs 확보했다"와 같은 원칙).
                result.selected_target_d = best_candidate->d.empty()
                    ? best_target_d : best_candidate->d.back();
                result.selected_cost = best_candidate->cost_total;
                result.candidate_path =
                    ConvertToCartesianPath(*best_candidate, ref_);
            } else {
                result.blocking_object_id = first_blocking_id;
            }
        } else if (blocked_by_request) {
            // 요청 자체로 후보를 만들지 않았다. 충돌이 아니므로 blocking
            // object는 없다.
            result.blocking_object_id = -1;
        }

        // ---- 유지 궤적 --------------------------------------------------
        //
        // **AVOID 후보를 여기 넣지 않는다.** PREPARE 중 횡이동 발행은 계약
        // 5.2절이 금지한다 - FSM이 EXECUTE를 주기 전에 차가 움직인다.
        PlannerCommand maintenance_base = *external_command;
        maintenance_base.target_speed = resolved_target_speed;
        if (!BuildAvoidMaintenancePath(ego, start, maintenance_base,
                                       predicted_collision_obstacles,
                                       out_path)) {
            last_failure_reason_ =
                PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
            return false;
        }

        // PREPARE 중에는 target_d를 lock하지 않는다. NONE이면 사이클마다
        // 좌/우가 바뀔 수 있고, FSM이 EXECUTE를 주지 않았으므로 그것이
        // 정상이다. lock은 D6-3의 COMMIT에서 한다(헤더 주석 참고).

        static int prepare_log_tick = 0;
        if (++prepare_log_tick % 5 == 0) {
            std::printf("[FrenetPlanner-STATE] mode=AVOID phase=PREPARE "
                        "s=%.2f d=%.3f speed=%.2f target_id=%d "
                        "candidate=%d target_d=%.3f evaluated=%d "
                        "low_speed=%d conflict=%d behind=%d "
                        "candidates=%zu/%zu maintenance=%zu\n",
                        s, d, ego.v, v2_avoidance_.obstacle_id,
                        result.candidate_valid ? 1 : 0, result.selected_target_d,
                        v2_avoid_targets_.count,
                        result.low_speed_context ? 1 : 0,
                        result.direction_conflict ? 1 : 0,
                        result.target_behind_ego ? 1 : 0,
                        result.valid_candidate_count,
                        result.generated_candidate_count, out_path.x.size());
        }

        // 반환값은 **maintenance가 유효한가**다. 후보 성공 여부는
        // result.candidate_valid로 따로 나간다(계약 8.3.1절).
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    // Behavior Planner 연동 전의 정적 장애물 AVOID context. 장애물 ID와
    // 회피 방향을 최초 1회만 고정한다. TRACKING 동안은 LANE_KEEPING을
    // 유지하고, 실제 기동 거리에서 SHIFT->PASS를 AVOID 안에서 수행한다.
    // 장애물 통과 후 중앙 복귀는 중복된 RETURN 단계 대신 LANE_KEEPING이 담당한다.
    PlannerCommand cmd{};
    double avoid_offset = 0.0;
    double leader_s = 0.0, leader_speed = 0.0, leader_accel = 0.0;
    bool begin_avoidance = false;

    if (use_legacy_behavior) {
    // 활성 context의 장애물 위치를 새 UDP 프레임으로 갱신한다. 일시적
    // packet dropout이 있어도 저장된 위치/길이로 회피를 계속한다.
    if (avoidance_.active) {
        for (const auto& obj : obstacles) {
            if (obj.id != avoidance_.obstacle_id) continue;
            double obj_s, obj_d, obj_s_dot;
            ProjectObjectToFrenet(ref_, obj, obj_s, obj_d, obj_s_dot);
            avoidance_.obstacle_s = obj_s;
            avoidance_.obstacle_d = obj_d;
            avoidance_.obstacle_width = obj.width;
            avoidance_.obstacle_length = obj.length;
            break;
        }

        const double gap = avoidance_.obstacle_s - s;
        const double clear_distance = 0.5 * (vehicle_shape_.length + avoidance_.obstacle_length)
                                    + avoid_cfg_.pass_clearance;
        if (avoidance_.phase == AvoidPhase::TRACKING && gap <= avoid_cfg_.shift_start_distance) {
            // 이 사이클에서 좌/우 전체 후보를 비교한 뒤, 유효 경로가
            // 있는 방향만 SHIFT로 잠그다.
            begin_avoidance = true;
        } else if (avoidance_.phase == AvoidPhase::SHIFT &&
                   std::fabs(d - avoidance_.target_d) <= avoid_cfg_.lateral_tolerance) {
            avoidance_.phase = AvoidPhase::PASS;
            std::printf("[FrenetPlanner] avoid phase: SHIFT -> PASS (id=%d gap=%.2f)\n",
                        avoidance_.obstacle_id, gap);
        } else if ((avoidance_.phase == AvoidPhase::SHIFT || avoidance_.phase == AvoidPhase::PASS) &&
                   gap < -clear_distance) {
            // 자차 뒤가 장애물을 완전히 통과했으므로 AVOID를 즉시 종료한다.
            // 이 아래의 !avoidance_.active 탐색이 같은 사이클에서 다음 정적
            // 장애물을 TRACKING으로 등록하고, 중앙 복귀는 LANE_KEEPING이 수행한다.
            std::printf("[FrenetPlanner] avoid complete after PASS: id=%d gap=%.2f\n",
                        avoidance_.obstacle_id, gap);
            avoidance_ = AvoidanceContext{};
        }
    }

    if (!avoidance_.active) {
        StaticObstacleTarget target;
        // LANE_KEEPING 충돌 필터는 reactive_lookahead 시간까지 미리 본다.
        // 정적 장애물 추적 거리가 그보다 짧으면, 장애물을 TRACKING으로
        // 등록하기 전에 충돌 필터가 중앙 후보를 전부 제거 -> stop path ->
        // 등록 후 다시 가속하는 속도 급락/반등이 생긴다. 설정값은 최소
        // 탐지거리로 쓰고, 실제 등록 거리는 "최고속도×충돌 lookahead +
        // 차량 길이/마진"보다 항상 길게 잡는다.
        AvoidConfig acquisition_cfg = avoid_cfg_;
        const double collision_horizon_distance =
            curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead
            + vehicle_shape_.length + collision_cfg_.safety_margin;
        acquisition_cfg.detection_distance =
            std::max(avoid_cfg_.detection_distance, collision_horizon_distance);
        if (FindStaticAvoidanceTarget(ref_, start, obstacles, lane_width_, vehicle_shape_,
                                      acquisition_cfg, target)) {
            avoidance_.active = true;
            avoidance_.obstacle_id = target.id;
            avoidance_.phase = AvoidPhase::TRACKING;
            avoidance_.obstacle_s = target.s;
            avoidance_.obstacle_d = target.d;
            avoidance_.obstacle_width = target.width;
            avoidance_.obstacle_length = target.length;
            // 저속/정지 상태에서도 영구 정지하지 않도록 selector가 확인한
            // 1차 방향을 보관한다. 정상 속도에서는 회피 시작 직전 양쪽의
            // 전체 후보를 비교해 더 정확한 target_d로 덮어쓴다.
            avoidance_.target_d = target.avoidance_offset;
            std::printf("[FrenetPlanner] static obstacle tracked: id=%d gap=%.2f "
                        "effective_detection=%.2f\n",
                        target.id, target.s - s, acquisition_cfg.detection_distance);
        }
    }

    // FOLLOWING 탐색거리도 충돌 lookahead보다 항상 길게 잡아,
    // leader를 모드 전환 전에 충돌물로만 먼저 보고 정지하는 현상을 막는다.
    FollowingConfig effective_following_cfg = following_cfg_;
    const double following_collision_horizon =
        curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead
        + vehicle_shape_.length + collision_cfg_.safety_margin;
    effective_following_cfg.max_leader_search_s =
        std::max(following_cfg_.max_leader_search_s, following_collision_horizon);

    // 직전 leader ID를 유지해 탐색거리 경계/패킷 누락에서 모드가
    // FOLLOWING<->LANE_KEEPING으로 번복되지 않게 한다.
    if (following_.active) {
        bool id_seen = false;
        bool leader_valid = false;
        const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
        for (const auto& obj : obstacles) {
            if (obj.id != following_.leader_id) continue;
            id_seen = true;
            double obj_s, obj_d, obj_s_dot;
            ProjectObjectToFrenet(ref_, obj, obj_s, obj_d, obj_s_dot);
            double gap = obj_s - s;
            if (gap <= 0.0 && track_length > 0.0) gap += track_length;
            const bool same_direction = obj_s_dot > following_cfg_.min_leader_speed;
            const bool same_lane = std::fabs(obj_d) <= lane_width_ * 0.5;
            const bool in_range = gap > 0.0 &&
                gap <= effective_following_cfg.max_leader_search_s
                     + following_cfg_.exit_search_margin;
            if (same_direction && same_lane && in_range) {
                following_.leader_s = s + gap;
                following_.leader_d = obj_d;
                following_.leader_speed = obj_s_dot;
                following_.leader_accel = 0.0;
                following_.missing_cycles = 0;
                leader_valid = true;
            }
            break;
        }

        if (!leader_valid) {
            if (id_seen) {
                // 관측됐지만 차선/방향/범위가 바뀐 것은 dropout이 아니므로 즉시 해제.
                following_ = FollowingContext{};
            } else if (++following_.missing_cycles <= following_cfg_.dropout_grace_cycles) {
                // ObjectInfo 일시 누락 동안은 직전 속도로 leader s를 예측.
                constexpr double kPlannerCycleDt = 0.05;  // main.cpp 20Hz
                following_.leader_s += following_.leader_speed * kPlannerCycleDt;
            } else {
                following_ = FollowingContext{};
            }
        }
    }

    // 유지 중인 ID만 보는 것으로 끝내면 합류 직후 실제로 더 가까워진 본선 차량을
    // 놓칠 수 있다. 매 cycle 동일차선 최인접 차량도 찾되, 작은 투영 흔들림으로
    // ID가 왕복하지 않도록 거리 hysteresis를 만족할 때만 인계한다.
    LeaderTarget nearest_leader;
    const bool nearest_found = FindLeader(
        ref_, start, obstacles, lane_width_, effective_following_cfg, nearest_leader);
    if (following_.active && nearest_found &&
        nearest_leader.id != following_.leader_id) {
        const double current_gap = following_.leader_s - s;
        const double nearest_gap = nearest_leader.s - s;
        if (nearest_gap + following_cfg_.leader_switch_margin < current_gap) {
            const int old_id = following_.leader_id;
            following_.leader_id = nearest_leader.id;
            following_.leader_s = nearest_leader.s;
            following_.leader_d = nearest_leader.d;
            following_.leader_speed = nearest_leader.speed;
            following_.leader_accel = nearest_leader.accel;
            following_.missing_cycles = 0;
            std::printf("[FrenetPlanner] leader handoff: old=%d new=%d "
                        "old_gap=%.2f new_gap=%.2f\n",
                        old_id, nearest_leader.id, current_gap, nearest_gap);
        }
    }

    if (!following_.active && nearest_found) {
            const LeaderTarget& leader = nearest_leader;
            following_.active = true;
            following_.leader_id = leader.id;
            following_.leader_s = leader.s;
            following_.leader_d = leader.d;
            following_.leader_speed = leader.speed;
            following_.leader_accel = leader.accel;
            following_.missing_cycles = 0;
            std::printf("[FrenetPlanner] leader acquired: id=%d gap=%.2f speed=%.2f "
                        "effective_search=%.2f\n",
                        leader.id, leader.s - s, leader.speed,
                        effective_following_cfg.max_leader_search_s);
    }

    // 두 MERGE 모두 Global Path(d=0)를 그대로 따른다. 회전교차로는 conflict
    // point 도착시간, 고주로는 목표 본선의 완료시점 앞/뒤 gap+TTC를 판단한다.
    const double braking_distance = ego.v * ego.v /
        (2.0 * std::max(limits_.max_longitudinal_accel, 0.1));
    const double dynamic_roundabout_approach = std::max(
        merge_cfg_.approach_distance,
        ego.v * collision_cfg_.reactive_lookahead + braking_distance + 5.0);
    const double roundabout_conflict_gap = merge_cfg_.conflict_s - s;
    const bool roundabout_window = merge_cfg_.conflict_s >= 0.0 &&
        roundabout_conflict_gap >= -merge_cfg_.completion_distance &&
        roundabout_conflict_gap <= dynamic_roundabout_approach;
    const int detected_highway_zone = FindHighwayMergeZone(highway_merge_cfg_, s);
    const bool highway_window = detected_highway_zone >= 0;

    MergeType requested_merge_type = merge_.active ? merge_.type : MergeType::NONE;
    if (requested_merge_type == MergeType::NONE) {
        if (roundabout_window) requested_merge_type = MergeType::ROUNDABOUT;
        else if (highway_window) requested_merge_type = MergeType::HIGHWAY;
    }
    const bool in_merge_approach =
        (requested_merge_type == MergeType::ROUNDABOUT &&
         (roundabout_window || (merge_.active &&
          roundabout_conflict_gap >= -merge_cfg_.completion_distance))) ||
        (requested_merge_type == MergeType::HIGHWAY &&
         (merge_.active
              ? (context_highway_zone && s <= context_highway_zone->completion_s)
              : highway_window));
    if (in_merge_approach) {
        if (!merge_.active) {
            merge_ = MergeContext{};
            merge_.type = requested_merge_type;
            if (merge_.type == MergeType::HIGHWAY)
                merge_.highway_zone_index = detected_highway_zone;
            const HighwayMergeZone* activated_zone = HighwayZoneAt(
                highway_merge_cfg_, merge_.highway_zone_index);
            std::printf("[FrenetPlanner-MERGE] policy activated: %s zone=%s s=%.2f\n",
                        merge_.type == MergeType::HIGHWAY ? "HIGHWAY" : "ROUNDABOUT",
                        activated_zone ? activated_zone->name.c_str() : "-", s);
        }
        merge_.active = true;

        const bool highway = merge_.type == MergeType::HIGHWAY;
        const HighwayMergeZone* active_highway_zone = HighwayZoneAt(
            highway_merge_cfg_, merge_.highway_zone_index);
        const HighwayMergeCheckpoint* active_highway_checkpoint = HighwayCheckpointAt(
            highway_merge_cfg_, merge_.highway_zone_index, merge_.highway_conflict_index);
        if (highway && !active_highway_checkpoint) {
            std::printf("[FrenetPlanner-MERGE] active highway zone missing; reset\n");
            merge_ = MergeContext{};
            return false;
        }
        const double active_conflict_s = highway
            ? active_highway_checkpoint->conflict_s : merge_cfg_.conflict_s;
        const double conflict_gap = active_conflict_s - s;
        const double active_commit_distance = highway
            ? highway_merge_cfg_.commit_distance : merge_cfg_.commit_distance;
        const int active_confirm_cycles = highway
            ? highway_merge_cfg_.safe_confirm_cycles : merge_cfg_.safe_confirm_cycles;
        const double active_cross_speed = highway
            ? highway_merge_cfg_.cross_speed_floor : merge_cfg_.cross_speed_floor;

        if (merge_.gap_locked) {
            merge_.commit_pending = false;
            merge_.entry_time = std::max(0.0, merge_.entry_time - 0.05);
            if (merge_.preceding_time >= 0.0)
                merge_.preceding_time = std::max(0.0, merge_.preceding_time - 0.05);
            if (merge_.following_time >= 0.0)
                merge_.following_time = std::max(0.0, merge_.following_time - 0.05);
            merge_.gap_safe = true;
            // stop_s는 WAIT 정지 위치이지 CROSS 시작점이 아니다. 대표 충돌점에
            // 실제로 도달한 뒤부터 교차구역 통과 단계로 전환한다.
            if (s >= active_conflict_s) {
                const HighwayMergeCheckpoint* next_checkpoint = highway
                    ? HighwayCheckpointAt(highway_merge_cfg_, merge_.highway_zone_index,
                                          merge_.highway_conflict_index + 1)
                    : nullptr;
                if (next_checkpoint) {
                    ++merge_.highway_conflict_index;
                    merge_.gap_locked = false;
                    merge_.gap_safe = false;
                    merge_.safe_cycles = 0;
                    merge_.phase = MergePhase::APPROACH;
                    merge_.locked_path = CartesianPath{};
                    merge_.locked_elapsed = 0.0;
                    merge_.locked_path_index = 0;
                    merge_.preceding_id = -1;
                    merge_.following_id = -1;
                    merge_.preceding_time = -1.0;
                    merge_.following_time = -1.0;
                    std::printf("[FrenetPlanner-MERGE] highway checkpoint advanced: "
                                "zone=%s index=%d next_conflict=%.2f s=%.2f\n",
                                active_highway_zone ? active_highway_zone->name.c_str() : "-",
                                merge_.highway_conflict_index,
                                next_checkpoint->conflict_s, s);
                } else {
                    merge_.phase = MergePhase::CROSS;
                }
            }
        } else {
            const RoundaboutGap gap = highway
                ? FindHighwayMergeGap(
                    ref_, start, predicted_obstacles, predicted_collision_obstacles,
                    highway_merge_cfg_,
                    *active_highway_checkpoint,
                    vehicle_shape_, collision_cfg_,
                    std::max(active_cross_speed, ego.v),
                    limits_.max_longitudinal_accel)
                : FindRoundaboutGap(
                    ref_, start, predicted_obstacles, merge_cfg_,
                    std::max(active_cross_speed, ego.v),
                    limits_.max_longitudinal_accel, &object_yaw_rates);
            const bool same_gap = merge_.preceding_id == gap.preceding_id &&
                                  merge_.following_id == gap.following_id;
            merge_.safe_cycles = gap.confirmed && gap.safe
                ? (same_gap ? merge_.safe_cycles + 1 : 1) : 0;
            merge_.gap_safe = gap.safe;
            merge_.entry_time = gap.entry_time;
            merge_.clear_time = gap.clear_time;
            merge_.preceding_id = gap.preceding_id;
            merge_.preceding_time = gap.preceding_time;
            merge_.following_id = gap.following_id;
            merge_.following_time = gap.following_time;
            merge_.crossing_vehicle_count = gap.crossing_vehicle_count;

            if (conflict_gap > active_commit_distance) {
                // APPROACH에서도 실제 gap 판정을 유지한다. unsafe이면 path_generator가
                // 제동거리 밖에서는 순항하고 제동구간부터 정지 경로로 전환한다.
                merge_.phase = MergePhase::APPROACH;
                // 예측창 밖(unconfirmed)은 진입 허가가 아니라 접근만 허용한다.
                merge_.gap_safe = !gap.confirmed || gap.safe;
                merge_.commit_pending = false;
            } else if (gap.confirmed && gap.safe &&
                       merge_.safe_cycles >= active_confirm_cycles) {
                // 아직 상태를 잠그지 않는다. 이 cycle의 실제 종/횡 결합 후보가
                // 곡률과 충돌검사를 통과한 뒤에만 아래에서 COMMIT으로 승격한다.
                merge_.phase = MergePhase::WAIT;
                merge_.commit_pending = true;
            } else {
                merge_.phase = MergePhase::WAIT;
                merge_.gap_safe = false;
                merge_.commit_pending = false;
            }
        }
    } else {
        merge_ = MergeContext{};
    }

    // 행동 결정(AVOID > MERGE > FOLLOW > KEEP)은 SelectLegacyBehavior 한 곳에만
    // 있다. v2에서는 이 호출이 FSM의 maneuver_type으로 대체된다.
    SelectLegacyBehavior(begin_avoidance, nearest_found, nearest_leader, cmd,
                         avoid_offset, leader_s, leader_speed, leader_accel);
    } else {
        // 외부 command 경로는 legacy AVOID/FOLLOW/MERGE context를 읽거나
        // 갱신하지 않는다. 진입 가능한 모드는 ValidateExternalCommand의
        // whitelist(현재 LANE_KEEPING/STOP)가 이미 걸러낸 상태다.
        cmd = *external_command;

        // 외부 FOLLOW: FSM이 지정한 대상만 확보한다. FindLeader()나 자동
        // handoff를 타지 않는다(계약 D1 0.1절 B 분류).
        if (cmd.mode == FOLLOWING) {
            const FollowTargetStatus status = ResolveExternalFollowTarget(
                s, cmd.target_object_id, obstacles, v2_following_);
            if (status == FollowTargetStatus::LOST) {
                // 대상을 쓸 수 없다. KEEP으로 자동 전환하지 않는다 - 다음
                // 행동은 FSM이 새 command_id로 결정한다(계약 12.5절 D5).
                last_mode_ = FOLLOWING;
                last_failure_reason_ = PlanFailureReason::TARGET_OBJECT_LOST;
                return false;
            }
            // VALID / GRACE 모두 확보된 상태를 후보 생성기에 전달한다.
            cmd.leader_s = v2_following_.leader_s;
            cmd.leader_speed = v2_following_.leader_speed;
            cmd.leader_accel = v2_following_.leader_accel;
            // 계약 11.8절: adapter는 custom gap이 꺼져 있으면 0.0 sentinel을
            // 넘긴다. config 기본값 적용은 이 소비 지점에서 한다 - 변환이
            // config에 의존하면 "FSM이 준 값"과 "Frenet 기본값"을 피드백에서
            // 구분할 수 없다.
            if (!(cmd.time_gap > 0.0)) cmd.time_gap = following_cfg_.time_gap;
            if (!(cmd.min_gap > 0.0)) cmd.min_gap = following_cfg_.min_gap;
            cmd.gap_gain = following_cfg_.gap_gain;
        }

        // 외부 AVOID의 대상 확보는 **저속 fallback보다 위**에서 이미 끝났다
        // (D6-2a). 여기서 하면 ego < 0.5 m/s일 때 도달하지 않는다.
        // AVOID/FOLLOW가 아닌 명령의 context reset도 위쪽에서 처리했다.
    }

    cmd.target_speed = resolved_target_speed;
    if (cmd.mode == MERGE && merge_.phase == MergePhase::CROSS) {
        const double floor = merge_.type == MergeType::HIGHWAY
            ? highway_merge_cfg_.cross_speed_floor : merge_cfg_.cross_speed_floor;
        cmd.target_speed = std::max(cmd.target_speed, floor);
    }

    // =====================================================================
    // 외부 STOP 실행 (APPROACH / BRAKING / HOLD)
    //
    // GenerateStoppingCandidates는 종료조건을 s(T)=stop_s, v(T)=0, a(T)=0으로
    // 강제한다. time_horizon 상한이 5초이므로 정지 목표가 멀면(예: 5m/s에서
    // 30m) 그 안에 도달하려는 후보가 전부 비현실적 가감속이 되어 가속도
    // 필터에서 탈락한다. horizon을 늘리는 대신, receding horizon 구조에 맞게
    // 먼 구간은 속도만 낮추며 접근하고(APPROACH) 제동거리 안에 들어왔을 때
    // 정지 다항식을 만든다(BRAKING).
    // =====================================================================
    const bool external_stop_mode = !use_legacy_behavior && cmd.mode == STOP;
    double stop_distance = 0.0;
    if (external_stop_mode) {
        const double s_max = ref_.points.empty() ? 0.0 : ref_.points.back().s;
        stop_distance = ForwardArcDistance(s, cmd.stop_position, s_max);

        // 목표가 바뀌면 새 STOP 명령으로 보고 실행 단계를 리셋한다. command_id는
        // publish마다 증가할 수 있어 phase 유지 판단 기준으로 쓸 수 없다.
        if (!stop_.target_valid ||
            !std::isfinite(stop_.target_s) ||
            std::fabs(cmd.stop_position - stop_.target_s) >
                stop_cfg_.target_change_tolerance) {
            stop_.phase = StopExecutionPhase::APPROACH;
            stop_.target_s = cmd.stop_position;
            stop_.target_valid = true;
        }

        const double decel = std::max(0.1, stop_cfg_.comfortable_decel);
        const double speed = std::max(0.0, s_dot);
        const double braking_distance = speed * speed / (2.0 * decel);
        const double trigger_distance = braking_distance + stop_cfg_.braking_margin;

        // APPROACH -> BRAKING은 단방향(latch). 경계에서 측정 노이즈로 왕복하면
        // 후보 생성기가 매 사이클 바뀌어 속도 프로파일이 흔들린다.
        if (stop_.phase == StopExecutionPhase::APPROACH &&
            stop_distance <= trigger_distance) {
            stop_.phase = StopExecutionPhase::BRAKING;
        }

        // 목표에 도달했고 충분히 느리면 HOLD. 목표를 지나쳤어도(음수 거리)
        // 후진하지 않고 그 자리에서 정지를 유지한다.
        const bool at_target = stop_distance <= stop_cfg_.position_tolerance;
        if (at_target && std::fabs(s_dot) <= stop_cfg_.speed_tolerance) {
            stop_.phase = StopExecutionPhase::HOLD;
        }

        if (stop_.phase == StopExecutionPhase::APPROACH) {
            // 목표점이 아니라 buffer만큼 앞을 향해 감속한다.
            const double effective =
                std::max(0.0, stop_distance - stop_cfg_.stop_buffer);
            const double v_safe = std::sqrt(2.0 * decel * effective);
            // 순항 목표속도와 비교해 더 낮은 쪽. 멀면 기존 속도 유지, 가까워질
            // 수록 자연스럽게 낮아진다.
            cmd.target_speed = std::min(cmd.target_speed, v_safe);
            // 후보 생성만 velocity keeping으로 위임한다. 외부에 보고되는
            // last_mode_는 아래에서 STOP으로 유지한다.
            cmd.mode = LANE_KEEPING;
        } else if (stop_.phase == StopExecutionPhase::BRAKING) {
            // 목표를 이미 지나쳤다면 되돌아가는 정지 다항식을 만들지 않는다.
            if (stop_distance < 0.0) {
                cmd.stop_position = s;
            }
            // BRAKING의 종방향 프로파일은 stopping quintic이 정하므로
            // cmd.target_speed는 후보 생성에 쓰이지 않는다. 다만 진단값
            // (last_target_speed_)이 순항속도로 남으면 실제로는 0을 향해
            // 감속 중인데 10m/s를 지령한 것처럼 보이므로 정지 목표에 맞춘
            // 감속 프로파일 값으로 보고한다.
            const double effective =
                std::max(0.0, stop_distance - stop_cfg_.stop_buffer);
            cmd.target_speed = std::min(cmd.target_speed,
                                        std::sqrt(2.0 * decel * effective));
        }
    }

    last_mode_ = external_stop_mode ? STOP : cmd.mode;
    last_target_speed_ = cmd.target_speed;

    // HOLD는 후보 생성/충돌 필터 파이프라인을 거치지 않는다. 이미 정지한
    // 상태이므로 0속도 궤적을 그대로 유지한다.
    if (external_stop_mode && stop_.phase == StopExecutionPhase::HOLD) {
        const RefPoint rp = Interpolate(ref_, s);
        out_path.x.assign(1, ego.x);
        out_path.y.assign(1, ego.y);
        out_path.yaw.assign(1, std::isfinite(ego.yaw) ? ego.yaw : rp.theta);
        out_path.kappa.assign(1, 0.0);
        out_path.v.assign(1, 0.0);
        out_path.a.assign(1, 0.0);
        last_failure_reason_ = PlanFailureReason::NONE;
        return true;
    }

    PlannerDebugStats stats;
    // TRACKING은 아직 회피를 시작하지 않는 구간이다. 8초 충돌 lookahead가
    // 먼 정적 장애물까지 미리 중앙 경로를 전부 제거하면 stop/go가 발생하므로,
    // 잠금된 해당 장애물만 이 구간의 충돌 필터에서 제외한다. 다른
    // 장애물은 계속 필터링되고, 25m 진입 시 잠금 장애물도 즉시 다시 포함된다.
    std::vector<TrackedObstacle> planning_collision_obstacles =
        predicted_collision_obstacles;
    if (use_legacy_behavior && avoidance_.active &&
        avoidance_.phase == AvoidPhase::TRACKING && !begin_avoidance) {
        planning_collision_obstacles.erase(
            std::remove_if(planning_collision_obstacles.begin(),
                           planning_collision_obstacles.end(),
                           [this](const TrackedObstacle& obj) {
                               return obj.id == avoidance_.obstacle_id;
                           }),
            planning_collision_obstacles.end());
    }

    std::vector<FrenetPath> candidates;
    // AVOID의 고정 속도 대신 후보 곡률로 속도를 자동 결정한다.
    // 1차 일반 목표속도 생성 -> 최적 후보 곡률의 허용속도 계산 ->
    // 필요할 때만 낮은 목표속도로 2차 재생성 순서다.
    auto generate_adaptive_avoid = [&](PlannerCommand& avoid_cmd, double target_d,
                                       PlannerDebugStats& avoid_stats) {
        auto generate_once = [&](PlannerDebugStats& generated_stats) {
            generated_stats = PlannerDebugStats{};
            auto generated = ResolveManeuver(start, avoid_cmd, ref_, path_cfg_, limits_,
                                             lane_width_, planning_collision_obstacles,
                                             vehicle_shape_,
                                             collision_cfg_, &generated_stats);
            EvaluateCosts(generated, cost_weights_, target_d);
            return generated;
        };

        std::vector<FrenetPath> generated = generate_once(avoid_stats);
        const FrenetPath* initial_best = SelectBestPath(generated);
        if (initial_best) {
            const double speed_limit = CandidateCurvatureSpeedLimit(
                *initial_best, ref_, limits_.max_lateral_accel);
            if (std::isfinite(speed_limit) && speed_limit + 0.1 < avoid_cmd.target_speed) {
                avoid_cmd.target_speed = speed_limit;
                generated = generate_once(avoid_stats);
            }
        }

        FilterByCartesianLateralAcceleration(generated, ref_, limits_.max_lateral_accel);
        // 추가 횡가속도 필터를 반영해 시각화 통계를 재계산한다.
        avoid_stats.combined_valid_after_curvature = 0;
        avoid_stats.combined_valid_after_collision = 0;
        for (const auto& path : generated) {
            if (path.valid || path.rejection_reason == RejectionReason::COLLISION) {
                ++avoid_stats.combined_valid_after_curvature;
            }
            if (path.valid) ++avoid_stats.combined_valid_after_collision;
        }
        return generated;
    };

    if (begin_avoidance) {
        // 장애물 d에서 자차/장애물 반폭과 SAT 안전 마진을 더한 지점을
        // 양쪽 목표로 삼는다. 기존 ±3m 절대값보다 장애물의 실제
        // 횡위치를 반영하므로 불필요하게 멀리 이동하지 않는다.
        constexpr double kDirectionExtraMargin = 0.3;
        const double clearance = 0.5 * (vehicle_shape_.width + avoidance_.obstacle_width)
                               + collision_cfg_.safety_margin + kDirectionExtraMargin;
        const double left_target = avoidance_.obstacle_d + clearance;
        const double right_target = avoidance_.obstacle_d - clearance;

        auto generate_side = [&](double target_d, PlannerDebugStats& side_stats,
                                 double& used_target_speed) {
            PlannerCommand side_cmd = cmd;
            side_cmd.avoidance_d_offset = target_d;
            auto side = generate_adaptive_avoid(side_cmd, target_d, side_stats);
            used_target_speed = side_cmd.target_speed;
            return side;
        };

        PlannerDebugStats left_stats, right_stats;
        double left_target_speed = cmd.target_speed;
        double right_target_speed = cmd.target_speed;
        std::vector<FrenetPath> left_candidates =
            generate_side(left_target, left_stats, left_target_speed);
        std::vector<FrenetPath> right_candidates =
            generate_side(right_target, right_stats, right_target_speed);
        const FrenetPath* left_best = SelectBestPath(left_candidates);
        const FrenetPath* right_best = SelectBestPath(right_candidates);

        bool choose_right = false;
        bool direction_found = left_best || right_best;
        if (!left_best && right_best) {
            choose_right = true;
        } else if (left_best && right_best) {
            const double tie_margin = std::max(1.0, std::fabs(left_best->cost_total)) * 0.05;
            choose_right = right_best->cost_total < left_best->cost_total ||
                (avoid_cfg_.prefer_right_when_equal &&
                 right_best->cost_total <= left_best->cost_total + tie_margin);
        }

        if (direction_found) {
            avoidance_.target_d = choose_right ? right_target : left_target;
            avoidance_.phase = AvoidPhase::SHIFT;
            cmd.avoidance_d_offset = avoidance_.target_d;
            cmd.target_speed = choose_right ? right_target_speed : left_target_speed;
            avoid_offset = avoidance_.target_d;
            candidates = choose_right ? std::move(right_candidates) : std::move(left_candidates);
            stats = choose_right ? right_stats : left_stats;
            std::printf("[FrenetPlanner] avoid direction locked: %s id=%d gap=%.2f "
                        "target_d=%.2f target_speed=%.2f valid(left=%zu right=%zu)\n",
                        choose_right ? "RIGHT" : "LEFT", avoidance_.obstacle_id,
                        avoidance_.obstacle_s - s, avoidance_.target_d, cmd.target_speed,
                        left_stats.combined_valid_after_collision,
                        right_stats.combined_valid_after_collision);
        } else {
            // 양쪽 모두 실패한 경우 방향을 잠그지 않고 두 집합을 모두
            // 시각화한다. 상위의 no-valid 처리가 안전 정지를 보낸 뒤
            // 다음 사이클에서 좌/우를 다시 평가한다.
            candidates = std::move(left_candidates);
            candidates.insert(candidates.end(),
                              std::make_move_iterator(right_candidates.begin()),
                              std::make_move_iterator(right_candidates.end()));
            stats.lateral_total = left_stats.lateral_total + right_stats.lateral_total;
            stats.lateral_valid = left_stats.lateral_valid + right_stats.lateral_valid;
            stats.longitudinal_total = left_stats.longitudinal_total + right_stats.longitudinal_total;
            stats.longitudinal_valid = left_stats.longitudinal_valid + right_stats.longitudinal_valid;
            stats.combined_total = left_stats.combined_total + right_stats.combined_total;
            stats.combined_valid_after_curvature = left_stats.combined_valid_after_curvature
                                                 + right_stats.combined_valid_after_curvature;
            stats.combined_valid_after_collision = 0;
            std::printf("[FrenetPlanner] avoid direction unavailable: id=%d gap=%.2f "
                        "valid(left=0 right=0)\n",
                        avoidance_.obstacle_id, avoidance_.obstacle_s - s);
        }
    } else {
        if (cmd.mode == AVOID) {
            candidates = generate_adaptive_avoid(cmd, cmd.avoidance_d_offset, stats);
        } else {
            candidates = ResolveManeuver(start, cmd, ref_, path_cfg_, limits_, lane_width_,
                                         planning_collision_obstacles, vehicle_shape_,
                                         collision_cfg_, &stats);
            // 회전교차로 MERGE도 횡방향으로는 global path(d=0)를 추종한다.
            EvaluateCosts(candidates, cost_weights_, 0.0);
        }
    }

    const bool legacy_merge = use_legacy_behavior && cmd.mode == MERGE;
    const double active_merge_conflict_gap = legacy_merge
        ? cmd.merge_conflict_s - s : 0.0;
    const HighwayMergeCheckpoint* output_highway_checkpoint = legacy_merge
        ? HighwayCheckpointAt(highway_merge_cfg_, merge_.highway_zone_index,
                              merge_.highway_conflict_index)
        : nullptr;
    const double active_merge_completion_distance =
        legacy_merge && merge_.type == MergeType::HIGHWAY && output_highway_checkpoint
            ? output_highway_checkpoint->clear_s - output_highway_checkpoint->conflict_s
            : merge_cfg_.completion_distance;
    const double active_merge_cross_speed = legacy_merge &&
        merge_.type == MergeType::HIGHWAY
        ? highway_merge_cfg_.cross_speed_floor : merge_cfg_.cross_speed_floor;

    if (cmd.mode != prev_mode_) {
        std::printf("[FrenetPlanner] mode: %s -> %s (s=%.2f d=%.3f)\n",
                    ModeName(prev_mode_), ModeName(cmd.mode), s, d);
        if (cmd.mode == AVOID) {
            std::printf("[FrenetPlanner]   avoid_id=%d phase=%s target_d=%.3f\n",
                        avoidance_.obstacle_id,
                        AvoidPhaseName(static_cast<int>(avoidance_.phase)), avoid_offset);
        } else if (cmd.mode == FOLLOWING) {
            const double gap = leader_s - s;
            const double desired_gap = following_cfg_.min_gap
                                     + following_cfg_.time_gap * leader_speed;
            std::printf("[FrenetPlanner]   leader_id=%d leader_s=%.2f leader_speed=%.2f "
                        "gap=%.2f desired_gap=%.2f\n",
                        following_.leader_id, leader_s, leader_speed, gap, desired_gap);
        } else if (cmd.mode == MERGE) {
            std::printf("[FrenetPlanner]   %s conflict_gap=%.2f safe=%d "
                        "entry_time=%.2f preceding_id=%d following_id=%d crossing=%zu\n",
                        merge_.type == MergeType::HIGHWAY ? "highway" : "roundabout",
                        active_merge_conflict_gap, merge_.gap_safe ? 1 : 0,
                        merge_.entry_time, merge_.preceding_id, merge_.following_id,
                        merge_.crossing_vehicle_count);
        }
        prev_mode_ = cmd.mode;
    }
    if (cmd.mode == MERGE && merge_.phase != prev_merge_phase_) {
        std::printf("[FrenetPlanner-MERGE] phase: %s -> %s s=%.2f gap=%.2f "
                    "safe=%d locked=%d front=%d rear=%d entry=%.2f\n",
                    MergePhaseName(static_cast<int>(prev_merge_phase_)),
                    MergePhaseName(static_cast<int>(merge_.phase)),
                    s, active_merge_conflict_gap, merge_.gap_safe ? 1 : 0,
                    merge_.gap_locked ? 1 : 0, merge_.preceding_id,
                    merge_.following_id, merge_.entry_time);
        prev_merge_phase_ = merge_.phase;
    } else if (cmd.mode != MERGE) {
        prev_merge_phase_ = MergePhase::CLEAR;
    }

    // gap selector의 point-arrival 판정만으로 COMMIT하지 않는다. 실제 후보를
    // conflict zone 이탈점까지 연장한 뒤, 동일 CTRV+OBB 검사로 전체 시간 궤적이
    // 안전한 후보만 남긴다. 이 검사를 통과한 경로 자체가 아래에서 잠긴다.
    if (cmd.mode == MERGE && merge_.commit_pending) {
        for (auto& candidate : candidates) {
            if (!candidate.valid) continue;
            CartesianPath full = ConvertToCartesianPath(candidate, ref_);
            ExtendMergePathThroughConflict(
                full, ref_, cmd.merge_conflict_s, active_merge_completion_distance,
                candidate.s.empty() ? cmd.merge_conflict_s : candidate.s.back(),
                std::max(cmd.target_speed, active_merge_cross_speed), path_cfg_.dt);
            int blocking_id = -1;
            if (!CartesianTrajectoryClear(full, predicted_collision_obstacles, vehicle_shape_,
                                          collision_cfg_, path_cfg_.dt, &blocking_id)) {
                candidate.valid = false;
                candidate.rejection_reason = RejectionReason::COLLISION;
                candidate.collision_object_id = blocking_id;
            }
        }
        stats.combined_valid_after_collision = 0;
        for (const auto& candidate : candidates)
            if (candidate.valid) ++stats.combined_valid_after_collision;
    }

    const FrenetPath* best = SelectBestPath(candidates);
    const int selected_index = best
        ? static_cast<int>(best - candidates.data())
        : -1;

    if (debug_writer_) {
        debug_writer_->Publish(ref_, ego, start, cmd.mode,
                               cmd.mode == AVOID
                                   ? AvoidPhaseName(static_cast<int>(avoidance_.phase))
                                   : (cmd.mode == MERGE
                                      ? MergePhaseName(static_cast<int>(merge_.phase)) : "NONE"),
                               cmd.target_speed,
                               predicted_collision_obstacles, candidates,
                               selected_index, stats, cmd);
    }

    if (!best) {
        // Gap 계산만 통과하고 실제 ego 궤적이 충돌 검사를 실패했다면 COMMIT은
        // 성립하지 않는다. 정지선 통과 전에는 잠금을 즉시 풀고 WAIT로 되돌린다.
        if (cmd.mode == MERGE && merge_.commit_pending &&
            s < cmd.merge_stop_s) {
            std::printf("[FrenetPlanner-MERGE] commit trajectory rejected "
                        "(curvature=%zu collision=%zu); returning to WAIT (s=%.2f)\n",
                        stats.combined_valid_after_curvature,
                        stats.combined_valid_after_collision, s);
            merge_.gap_locked = false;
            merge_.commit_pending = false;
            merge_.gap_safe = false;
            merge_.safe_cycles = 0;
            merge_.phase = MergePhase::WAIT;
        }
        // 후보가 곡률 단계까지는 살아 있었는데 충돌 검사에서 전부 제거된 것은
        // 일시적인 생성 실패가 아니라 명시적인 안전 차단이다. main.cpp가 이
        // 값을 보고 과거 경로 재전송을 금지한다.
        last_failure_reason_ =
            stats.combined_valid_after_curvature > 0 &&
            stats.combined_valid_after_collision == 0
                ? PlanFailureReason::COLLISION_BLOCKED
                : PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
        std::printf("[FrenetPlanner] No valid candidate this cycle "
                    "(mode=%s, %zu generated)\n",
                    ModeName(cmd.mode), candidates.size());
        std::printf("[FrenetPlanner-DEBUG] ego: x=%.3f y=%.3f yaw=%.3f v=%.3f a=%.3f\n",
                     ego.x, ego.y, ego.yaw, ego.v, ego.a);
        std::printf("[FrenetPlanner-DEBUG] start: s=%.2f d=%.3f s_dot=%.2f s_ddot=%.2f d_dot=%.3f d_ddot=%.3f "
                     "target_speed=%.2f\n",
                     s, d, s_dot, s_ddot, d_dot, d_ddot, cmd.target_speed);
        std::printf("[FrenetPlanner-DEBUG] lateral %zu/%zu valid | longitudinal %zu/%zu valid | "
                     "combined %zu -> after curvature %zu -> after collision %zu valid\n",
                     stats.lateral_valid, stats.lateral_total,
                     stats.longitudinal_valid, stats.longitudinal_total,
                     stats.combined_total, stats.combined_valid_after_curvature,
                     stats.combined_valid_after_collision);
        if (stats.combined_valid_after_curvature > 0 &&
            stats.combined_valid_after_collision == 0) {
            std::unordered_map<int, int> blocker_counts;
            std::unordered_map<int, double> blocker_first_times;
            for (const auto& candidate : candidates) {
                if (candidate.rejection_reason != RejectionReason::COLLISION ||
                    candidate.collision_object_id < 0) continue;
                const int blocker_id = candidate.collision_object_id;
                ++blocker_counts[blocker_id];
                double collision_t = -1.0;
                if (candidate.collision_sample_index >= 0 &&
                    static_cast<size_t>(candidate.collision_sample_index) < candidate.t.size()) {
                    collision_t = candidate.t[static_cast<size_t>(candidate.collision_sample_index)];
                }
                auto time_it = blocker_first_times.find(blocker_id);
                if (collision_t >= 0.0 &&
                    (time_it == blocker_first_times.end() || collision_t < time_it->second))
                    blocker_first_times[blocker_id] = collision_t;
            }
            std::vector<std::pair<int, int>> ranked_blockers(
                blocker_counts.begin(), blocker_counts.end());
            std::sort(ranked_blockers.begin(), ranked_blockers.end(),
                      [](const auto& lhs, const auto& rhs) {
                          return lhs.second > rhs.second;
                      });
            const size_t report_count = std::min<size_t>(3, ranked_blockers.size());
            for (size_t rank = 0; rank < report_count; ++rank) {
                const int blocker_id = ranked_blockers[rank].first;
                const int blocked_paths = ranked_blockers[rank].second;
                const auto obj_it = std::find_if(
                    predicted_obstacles.begin(), predicted_obstacles.end(),
                    [blocker_id](const ObjectInfo& obj) { return obj.id == blocker_id; });
                if (obj_it == predicted_obstacles.end()) {
                    std::printf("[FrenetPlanner-COLLISION] rank=%zu blocking_id=%d "
                                "paths=%d object=missing leader_id=%d\n",
                                rank + 1, blocker_id, blocked_paths,
                                use_legacy_behavior && following_.active
                                    ? following_.leader_id : -1);
                    continue;
                }
                double obj_s, obj_d, obj_s_dot;
                ProjectObjectToFrenet(ref_, *obj_it, obj_s, obj_d, obj_s_dot);
                double obj_gap = obj_s - s;
                const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
                if (obj_gap <= 0.0 && track_length > 0.0) obj_gap += track_length;
                const auto time_it = blocker_first_times.find(blocker_id);
                const double first_t = time_it == blocker_first_times.end()
                    ? -1.0 : time_it->second;
                std::printf("[FrenetPlanner-COLLISION] rank=%zu blocking_id=%d paths=%d "
                            "first_t=%.2f obj_s=%.2f obj_d=%.2f gap=%.2f speed_s=%.2f "
                            "same_lane=%d leader_id=%d leader_match=%d\n",
                            rank + 1, blocker_id, blocked_paths, first_t,
                            obj_s, obj_d, obj_gap, obj_s_dot,
                            std::fabs(obj_d) <= lane_width_ * 0.5 ? 1 : 0,
                            use_legacy_behavior && following_.active
                                ? following_.leader_id : -1,
                            use_legacy_behavior && following_.active &&
                                following_.leader_id == blocker_id ? 1 : 0);
            }
        }
        // STOP BRAKING에서 후보가 0개인 것은 흔한 상황이다. 정지 목표가
        // 아주 가까우면(예: 5m/s에서 3m 앞) exact-stop quintic이 가속도
        // 한계를 넘어 전부 탈락한다. 여기서 false를 반환하면 노드가 이전
        // 궤적을 재사용해 정지 목표를 지나쳐 계속 주행하게 된다 - STOP에서
        // 가장 위험한 실패 방향이다. 충돌 차단이 아닌 경우에 한해 그 자리에서
        // 감속해 정지하는 fail-safe 궤적을 직접 만들어 반환한다.
        if (external_stop_mode &&
            stop_.phase == StopExecutionPhase::BRAKING &&
            last_failure_reason_ != PlanFailureReason::COLLISION_BLOCKED) {
            GenerateDecelToZeroPath(s, d, s_dot, out_path);
            const double v0 = std::max(0.0, s_dot);
            std::printf("[FrenetPlanner-STOP] BRAKING fail-safe: no candidate, "
                        "emitting decel-to-zero path (s=%.2f v=%.2f dist=%.2f)\n",
                        s, v0, stop_distance);
            last_failure_reason_ = PlanFailureReason::NONE;
            return true;
        }
        return false;
    }

    CartesianPath selected_cartesian = ConvertToCartesianPath(*best, ref_);

    // STOP은 절대 후진하지 않는다. 정지 목표를 이미 지나쳤거나 아주 가까울 때
    // stopping quintic은 목표 s에 "도달"하려고 0을 지나 음수 속도로 내려가
    // 후진 궤적을 만든다(실측: v가 0 이후 -0.15m/s까지 내려감). 속도가 0에
    // 닿는 시점에서 궤적을 잘라 그 자리에 정지시킨다.
    if (external_stop_mode) {
        std::size_t stop_index = selected_cartesian.v.size();
        for (std::size_t i = 0; i < selected_cartesian.v.size(); ++i) {
            if (selected_cartesian.v[i] < 0.0) { stop_index = i; break; }
        }
        if (stop_index < selected_cartesian.v.size()) {
            // 최소 한 점은 남겨 MPC가 추종할 자세 정보를 잃지 않게 한다.
            const std::size_t keep = std::max<std::size_t>(1, stop_index);
            selected_cartesian.x.resize(keep);
            selected_cartesian.y.resize(keep);
            selected_cartesian.yaw.resize(keep);
            selected_cartesian.kappa.resize(keep);
            selected_cartesian.v.resize(keep);
            selected_cartesian.a.resize(keep);
            selected_cartesian.v.back() = 0.0;
            selected_cartesian.a.back() = 0.0;
        }
    }

    if (cmd.mode == MERGE && merge_.commit_pending) {
        ExtendMergePathThroughConflict(
            selected_cartesian, ref_, cmd.merge_conflict_s,
            active_merge_completion_distance,
            best->s.empty() ? cmd.merge_conflict_s : best->s.back(),
            std::max(cmd.target_speed, active_merge_cross_speed), path_cfg_.dt);
        merge_.commit_pending = false;
        merge_.gap_locked = true;
        merge_.phase = MergePhase::COMMIT;
        merge_.locked_path = selected_cartesian;
        merge_.locked_elapsed = 0.0;
        merge_.locked_path_index = 0;
        std::printf("[FrenetPlanner-MERGE] gap committed after trajectory validation: "
                    "front=%d(%.2fs) rear=%d(%.2fs) entry=%.2fs s=%.2f samples=%zu\n",
                    merge_.preceding_id, merge_.preceding_time,
                    merge_.following_id, merge_.following_time,
                    merge_.entry_time, s, merge_.locked_path.x.size());
    }

    // 모든 모드의 현재 상태를 20Hz 기준 약 4Hz로 출력한다. 모드 전환 로그만으로는
    // 후보가 사라지는 바로 그 순간이 LANE_KEEPING/AVOID 중 어느 상태였는지 알 수
    // 없으므로, s/d/속도와 필터 후 후보 수를 같은 줄에 남긴다.
    static int state_log_tick = 0;
    if (++state_log_tick % 5 == 0) {
        if (cmd.mode == FOLLOWING) {
            const double gap = leader_s - s;
            const double desired_gap = following_cfg_.min_gap
                                     + following_cfg_.time_gap * leader_speed;
            const double approach_speed = std::max(0.0, std::min(
                cmd.target_speed,
                leader_speed + following_cfg_.gap_gain * (gap - desired_gap)));
            std::printf("[FrenetPlanner-STATE] mode=FOLLOWING leader_id=%d s=%.2f d=%.3f "
                        "ego_speed=%.2f leader_speed=%.2f relative_speed=%.2f "
                        "gap=%.2f desired_gap=%.2f gap_error=%.2f approach_speed=%.2f chosen_d=%.3f "
                        "cost=%.2f candidates=%zu/%zu missing=%d\n",
                        following_.leader_id, s, d, ego.v, leader_speed,
                        ego.v - leader_speed, gap, desired_gap, gap - desired_gap,
                        approach_speed, best->d.empty() ? d : best->d.back(), best->cost_total,
                        stats.combined_valid_after_collision, stats.combined_total,
                        following_.missing_cycles);
        } else if (cmd.mode == MERGE) {
            std::printf("[FrenetPlanner-STATE] mode=MERGE phase=%s s=%.2f d=%.3f "
                        "speed=%.2f target_speed=%.2f conflict_gap=%.2f "
                        "entry=%.2f clear=%.2f front=%d(%.2f) rear=%d(%.2f) "
                        "locked=%d candidates=%zu/%zu\n",
                        MergePhaseName(static_cast<int>(merge_.phase)),
                        s, d, ego.v, cmd.target_speed, active_merge_conflict_gap,
                        merge_.entry_time, merge_.clear_time,
                        merge_.preceding_id, merge_.preceding_time,
                        merge_.following_id, merge_.following_time,
                        merge_.gap_locked ? 1 : 0,
                        stats.combined_valid_after_collision, stats.combined_total);
        } else {
            std::printf("[FrenetPlanner-STATE] mode=%s phase=%s s=%.2f d=%.3f speed=%.2f target_speed=%.2f "
                        "target_d=%.3f chosen_d=%.3f cost=%.2f candidates=%zu/%zu\n",
                        ModeName(cmd.mode), cmd.mode == AVOID
                            ? AvoidPhaseName(static_cast<int>(avoidance_.phase))
                            : (cmd.mode == MERGE
                               ? MergePhaseName(static_cast<int>(merge_.phase)) : "NONE"),
                        s, d, ego.v, cmd.target_speed,
                        cmd.mode == AVOID ? avoid_offset : 0.0,
                        best->d.empty() ? d : best->d.back(), best->cost_total,
                        stats.combined_valid_after_collision, stats.combined_total);
        }
    }

    out_path = std::move(selected_cartesian);
    return true;
}
