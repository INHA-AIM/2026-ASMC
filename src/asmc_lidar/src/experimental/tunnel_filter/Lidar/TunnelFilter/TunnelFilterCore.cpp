#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>

namespace tunnel_filter {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float DegToRad(float degrees) { return degrees * kPi / 180.0f; }
float RadToDeg(float radians) { return radians * 180.0f / kPi; }

using Clock = std::chrono::steady_clock;

double ElapsedMs(const Clock::time_point& start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start)
        .count();
}

bool FinitePoint(const pcl::PointXYZI& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

// PCL 평면 계수를 단위 법선 평면으로 바꾼다.
bool ToPlane(const pcl::ModelCoefficients& coefficients, Plane* plane) {
    if (coefficients.values.size() < 4) return false;
    const float a = coefficients.values[0];
    const float b = coefficients.values[1];
    const float c = coefficients.values[2];
    const float d = coefficients.values[3];
    const float norm = std::sqrt(a * a + b * b + c * c);
    if (!std::isfinite(norm) || norm < 1e-6f) return false;
    plane->a = a / norm;
    plane->b = b / norm;
    plane->c = c / norm;
    plane->d = d / norm;
    return plane->Finite();
}

Plane Negated(const Plane& plane) {
    Plane result;
    result.a = -plane.a;
    result.b = -plane.b;
    result.c = -plane.c;
    result.d = -plane.d;
    return result;
}

}  // namespace

const char* ProtectClassName(std::uint8_t protect_class) {
    switch (protect_class) {
        case kProtectCar: return "car";
        case kProtectTruck: return "truck";
        case kProtectBus: return "bus";
        case kProtectPerson: return "person";
        case kProtectPedestrian: return "pedestrian";
        default: return "none";
    }
}

std::uint8_t ProtectClassFromLabel(const std::string& label) {
    std::string lower = label;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    if (lower == "car") return kProtectCar;
    if (lower == "truck") return kProtectTruck;
    if (lower == "bus") return kProtectBus;
    if (lower == "person") return kProtectPerson;
    if (lower == "pedestrian") return kProtectPedestrian;
    return kProtectNone;
}

bool IsPersonClass(std::uint8_t protect_class) {
    return protect_class == kProtectPerson ||
           protect_class == kProtectPedestrian;
}

float Plane::HeightAt(float x, float y) const {
    if (std::fabs(c) < 1e-6f) return std::numeric_limits<float>::quiet_NaN();
    return -(a * x + b * y + d) / c;
}

float Plane::TiltDeg() const {
    return RadToDeg(std::acos(std::min(1.0f, std::fabs(c))));
}

bool Plane::Finite() const {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) &&
           std::isfinite(d);
}

float AngleBetweenNormalsDeg(const Plane& lhs, const Plane& rhs) {
    const float dot = std::fabs(lhs.a * rhs.a + lhs.b * rhs.b + lhs.c * rhs.c);
    return RadToDeg(std::acos(std::min(1.0f, dot)));
}

float HorizontalAngleBetweenNormalsDeg(const Plane& lhs, const Plane& rhs) {
    const float nl = std::hypot(lhs.a, lhs.b);
    const float nr = std::hypot(rhs.a, rhs.b);
    if (nl < 1e-6f || nr < 1e-6f) return 90.0f;
    const float dot = std::fabs(lhs.a * rhs.a + lhs.b * rhs.b) / (nl * nr);
    return RadToDeg(std::acos(std::min(1.0f, dot)));
}

const char* PlaneSourceName(PlaneSource source) {
    switch (source) {
        case PlaneSource::kCurrent: return "current";
        case PlaneSource::kPrevious: return "previous";
        default: return "unavailable";
    }
}

bool ValidateGroundFit(GroundFit* fit, const Params& params,
                       const Plane* prev) {
    fit->valid = false;
    fit->inlier_ratio = fit->candidates > 0
        ? static_cast<float>(fit->inliers) / static_cast<float>(fit->candidates)
        : 0.0f;
    if (!fit->found) {
        fit->reject_reason = "no_model";
        return false;
    }
    if (!fit->plane.Finite()) {
        fit->reject_reason = "non_finite";
        return false;
    }
    fit->tilt_deg = fit->plane.TiltDeg();
    fit->height_at_origin = fit->plane.HeightAt(0.0f, 0.0f);
    if (!std::isfinite(fit->height_at_origin)) {
        fit->reject_reason = "non_finite";
        return false;
    }
    if (fit->tilt_deg > params.ground_normal_max_angle_deg) {
        fit->reject_reason = "normal_tilt";
        return false;
    }
    // 천장처럼 수평이어도 높이가 지면 범위 밖이면 절대 지면으로 인정하지 않는다.
    if (fit->height_at_origin < params.ground_height_min_m ||
        fit->height_at_origin > params.ground_height_max_m) {
        fit->reject_reason = "height_out_of_range";
        return false;
    }
    if (fit->inliers < params.ground_min_inliers) {
        fit->reject_reason = "few_inliers";
        return false;
    }
    if (fit->inlier_ratio < params.ground_min_inlier_ratio) {
        fit->reject_reason = "low_inlier_ratio";
        return false;
    }
    if (prev != nullptr) {
        const float height_jump =
            std::fabs(fit->height_at_origin - prev->HeightAt(0.0f, 0.0f));
        if (height_jump > params.ground_max_height_jump_m) {
            fit->reject_reason = "height_jump";
            return false;
        }
        if (AngleBetweenNormalsDeg(fit->plane, *prev) >
            params.ground_max_normal_jump_deg) {
            fit->reject_reason = "normal_jump";
            return false;
        }
    }
    fit->valid = true;
    fit->reject_reason = "ok";
    return true;
}

GroundTemporalFilter::GroundTemporalFilter(int reuse_max_frames)
    : reuse_max_frames_(std::max(0, reuse_max_frames)) {}

GroundDecision GroundTemporalFilter::Update(bool current_valid,
                                            const Plane& current) {
    GroundDecision decision;
    if (current_valid) {
        has_plane_ = true;
        last_ = current;
        consecutive_failures_ = 0;
        decision.source = PlaneSource::kCurrent;
        decision.plane = current;
        return decision;
    }
    ++consecutive_failures_;
    decision.consecutive_failures = consecutive_failures_;
    decision.plane = last_;
    decision.source = (has_plane_ && consecutive_failures_ <= reuse_max_frames_)
        ? PlaneSource::kPrevious
        : PlaneSource::kUnavailable;
    return decision;
}

const Plane* GroundTemporalFilter::ConsistencyReference() const {
    return (has_plane_ && consecutive_failures_ <= reuse_max_frames_)
        ? &last_
        : nullptr;
}

void GroundTemporalFilter::Reset() {
    has_plane_ = false;
    last_ = Plane();
    consecutive_failures_ = 0;
}

std::vector<WallSupport> ComputeWallRuns(const std::vector<float>& s_values,
                                         float bin_size_m, int bin_min_points,
                                         int max_gap_bins) {
    std::vector<WallSupport> runs;
    if (s_values.empty() || !(bin_size_m > 0.0f)) return runs;
    const auto bounds = std::minmax_element(s_values.begin(), s_values.end());
    const float s0 = *bounds.first;
    const float extent = *bounds.second - s0;
    if (!std::isfinite(extent)) return runs;
    const int bins = static_cast<int>(std::floor(extent / bin_size_m)) + 1;
    if (bins <= 0 || bins > 100000) return runs;

    std::vector<int> counts(static_cast<std::size_t>(bins), 0);
    for (const float value : s_values) {
        int bin = static_cast<int>(std::floor((value - s0) / bin_size_m));
        bin = std::max(0, std::min(bins - 1, bin));
        ++counts[static_cast<std::size_t>(bin)];
    }

    int run_start = -1;
    int last_occupied = -1;
    int occupied = 0;
    const auto close_run = [&]() {
        if (run_start < 0) return;
        WallSupport run;
        run.ok = true;
        run.s_start = s0 + static_cast<float>(run_start) * bin_size_m;
        run.s_end = s0 + static_cast<float>(last_occupied + 1) * bin_size_m;
        run.occupied_bins = occupied;
        runs.push_back(run);
    };
    for (int bin = 0; bin < bins; ++bin) {
        if (counts[static_cast<std::size_t>(bin)] < bin_min_points) continue;
        if (run_start < 0 || bin - last_occupied - 1 > max_gap_bins) {
            close_run();
            run_start = bin;
            occupied = 0;
        }
        last_occupied = bin;
        ++occupied;
    }
    close_run();
    // 길이 내림차순 (같으면 앞쪽 구간 우선: 안정 정렬).
    std::stable_sort(runs.begin(), runs.end(),
                     [](const WallSupport& lhs, const WallSupport& rhs) {
                         return lhs.s_end - lhs.s_start > rhs.s_end - rhs.s_start;
                     });
    return runs;
}

WallSupport ComputeWallSupport(const std::vector<float>& s_values,
                               float bin_size_m, int bin_min_points,
                               int max_gap_bins) {
    const std::vector<WallSupport> runs =
        ComputeWallRuns(s_values, bin_size_m, bin_min_points, max_gap_bins);
    return runs.empty() ? WallSupport() : runs.front();
}

bool ValidateWallSegment(WallSegment* segment, const Params& params,
                         float min_vertical_span_m, float min_span_m,
                         int min_bins) {
    if (min_vertical_span_m < 0.0f) {
        min_vertical_span_m = params.wall_min_vertical_span_m;
    }
    if (min_span_m < 0.0f) min_span_m = params.wall_min_span_m;
    if (min_bins < 0) min_bins = params.wall_min_occupied_bins;
    segment->valid = false;
    if (!segment->plane.Finite()) {
        segment->reject_reason = "non_finite";
        return false;
    }
    segment->tilt_deg =
        RadToDeg(std::asin(std::min(1.0f, std::fabs(segment->plane.c))));
    if (segment->tilt_deg > params.wall_max_tilt_deg) {
        segment->reject_reason = "not_vertical";
        return false;
    }
    if (segment->lateral_distance < params.wall_min_lateral_distance_m) {
        segment->reject_reason = "too_close";
        return false;
    }
    if (segment->inliers < params.wall_min_inliers) {
        segment->reject_reason = "few_inliers";
        return false;
    }
    if (segment->inlier_ratio < params.wall_min_inlier_ratio) {
        segment->reject_reason = "low_inlier_ratio";
        return false;
    }
    if (segment->span < min_span_m) {
        segment->reject_reason = "short_span";
        return false;
    }
    if (segment->occupied_bins < min_bins) {
        segment->reject_reason = "few_bins";
        return false;
    }
    if (segment->vertical_span < min_vertical_span_m) {
        segment->reject_reason = "low_vertical_span";
        return false;
    }
    // 벽은 측면 경계다. 평면 뒤쪽에 후보점이 많으면 벽 앞의 물체(차량 옆면 등)다.
    if (segment->beyond_fraction > params.wall_max_beyond_fraction) {
        segment->reject_reason = "not_outermost";
        return false;
    }
    segment->valid = true;
    segment->reject_reason = "ok";
    return true;
}

Plane TransformPlaneToCurrent(const Plane& previous, const EgoMotion& motion) {
    // n·p_prev + d = 0, p_prev = R p_cur + t  →  (Rᵀn)·p_cur + (n·t + d) = 0
    const float c = std::cos(motion.dyaw_rad);
    const float s = std::sin(motion.dyaw_rad);
    Plane plane;
    plane.a = c * previous.a + s * previous.b;
    plane.b = -s * previous.a + c * previous.b;
    plane.c = previous.c;
    plane.d = previous.d + previous.a * motion.dx_m + previous.b * motion.dy_m;
    return plane;
}

bool WallTrack::Update(const std::vector<WallSegment>& segments,
                       const Params& params, const EgoMotion* motion) {
    // 저장된 평면은 항상 직전 처리 프레임 좌표계다: 먼저 현재 좌표계로 옮긴다
    // (놓친 프레임이 있어도 매 호출의 이동이 누적된다).
    if (state_.has && params.wall_track_use_ego_motion && motion != nullptr &&
        motion->valid) {
        for (Plane& stored : state_.planes) {
            stored = TransformPlaneToCurrent(stored, *motion);
        }
    }
    const WallSegment* primary = segments.empty() ? nullptr : &segments.front();
    if (primary != nullptr && primary->valid) {
        bool consistent = false;
        if (state_.has) {
            // 수직도는 검증 단계에서 이미 제한했으므로(±tilt), 추적 일관성은 벽의
            // 수평 방향(방위각)으로만 비교한다. 3D 각도는 tilt 부호가 바뀌면 ±tilt가
            // 합쳐져 같은 벽도 불일치로 판정된다.
            for (const Plane& previous : state_.planes) {
                const float angle =
                    HorizontalAngleBetweenNormalsDeg(primary->plane, previous);
                const float offset = std::fabs(primary->plane.d - previous.d);
                if (angle <= params.wall_track_max_angle_deg &&
                    offset <= params.wall_track_max_offset_m) {
                    consistent = true;
                    break;
                }
            }
        }
        state_.persist = consistent ? state_.persist + 1 : 1;
        state_.has = true;
        state_.planes.clear();
        for (const WallSegment& segment : segments) {
            if (segment.valid) state_.planes.push_back(segment.plane);
        }
        state_.miss = 0;
        state_.confirmed =
            state_.persist >= std::max(1, params.wall_min_persist_frames);
        return state_.confirmed;
    }
    if (state_.has) {
        ++state_.miss;
        if (state_.miss > params.wall_track_max_miss) state_ = WallTrackState();
    }
    // 현재 프레임에 검증된 평면이 없으면 제거하지 않는다 (fail-open).
    return false;
}

void DefaultObbFunction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                        const std::vector<pcl::PointIndices>& clusters,
                        std::vector<Obb>* obbs) {
    obbs->assign(clusters.size(), Obb());
    if (!cloud) return;
    for (std::size_t c = 0; c < clusters.size(); ++c) {
        const auto& indices = clusters[c].indices;
        if (indices.empty()) continue;
        float best_area = std::numeric_limits<float>::infinity();
        float best_angle = 0.0f;
        for (int step = 0; step < 45; ++step) {
            const float angle = DegToRad(2.0f * static_cast<float>(step));
            const float cs = std::cos(angle);
            const float sn = std::sin(angle);
            float u_min = std::numeric_limits<float>::infinity();
            float u_max = -u_min;
            float v_min = u_min;
            float v_max = -u_min;
            for (const int index : indices) {
                const auto& p = cloud->points[static_cast<std::size_t>(index)];
                const float u = p.x * cs + p.y * sn;
                const float v = -p.x * sn + p.y * cs;
                u_min = std::min(u_min, u);
                u_max = std::max(u_max, u);
                v_min = std::min(v_min, v);
                v_max = std::max(v_max, v);
            }
            const float area = (u_max - u_min) * (v_max - v_min);
            if (area < best_area) {
                best_area = area;
                best_angle = angle;
            }
        }
        const float cs = std::cos(best_angle);
        const float sn = std::sin(best_angle);
        float u_min = std::numeric_limits<float>::infinity();
        float u_max = -u_min;
        float v_min = u_min;
        float v_max = -u_min;
        float z_min = u_min;
        float z_max = -u_min;
        for (const int index : indices) {
            const auto& p = cloud->points[static_cast<std::size_t>(index)];
            const float u = p.x * cs + p.y * sn;
            const float v = -p.x * sn + p.y * cs;
            u_min = std::min(u_min, u);
            u_max = std::max(u_max, u);
            v_min = std::min(v_min, v);
            v_max = std::max(v_max, v);
            z_min = std::min(z_min, p.z);
            z_max = std::max(z_max, p.z);
        }
        Obb& obb = (*obbs)[c];
        const float cu = 0.5f * (u_min + u_max);
        const float cv = 0.5f * (v_min + v_max);
        obb.cx = cu * cs - cv * sn;
        obb.cy = cu * sn + cv * cs;
        obb.cz = 0.5f * (z_min + z_max);
        obb.z_min = z_min;
        obb.z_max = z_max;
        const float du = u_max - u_min;
        const float dv = v_max - v_min;
        obb.length = std::max(du, dv);
        obb.width = std::min(du, dv);
        obb.yaw = du >= dv ? best_angle : best_angle + 0.5f * kPi;
        const float signs[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
        for (int k = 0; k < 4; ++k) {
            const float u = cu + signs[k][0] * 0.5f * du;
            const float v = cv + signs[k][1] * 0.5f * dv;
            const float x = u * cs - v * sn;
            const float y = u * sn + v * cs;
            obb.corners[static_cast<std::size_t>(k)] = pcl::PointXYZ(x, y, z_min);
            obb.corners[static_cast<std::size_t>(k + 4)] =
                pcl::PointXYZ(x, y, z_max);
        }
    }
}

ObbStructureFlags ClassifyObb(const Obb& obb, float z_min_road,
                              float z_max_road, bool is_protected,
                              bool wall_attached, const Params& params) {
    ObbStructureFlags flags;
    const float long_side = std::max(obb.length, obb.width);
    const float short_side = std::max(0.05f, std::min(obb.length, obb.width));
    // 중심 z 하나가 아니라 OBB 바닥(z_min)이 떠 있어야 천장으로 본다.
    flags.ceiling = z_min_road >= params.obb_ceiling_min_z_road_m;
    flags.wall_attached = wall_attached;
    const bool far_lateral = std::fabs(obb.cy) >= params.obb_wall_min_abs_y_m &&
                             long_side >= params.obb_wall_min_length_m;
    const bool attached = wall_attached &&
                          long_side >= params.obb_wall_attached_min_length_m;
    flags.wall = (far_lateral || attached) &&
                 long_side / short_side >= params.obb_wall_min_aspect_ratio;
    flags.ground_residual =
        (obb.z_max - obb.z_min) <= params.obb_ground_residual_max_height_m &&
        z_max_road <= params.obb_ground_residual_max_top_road_m;
    flags.removed = params.obb_filter_enable && !is_protected &&
                    (flags.ceiling || flags.wall ||
                     (flags.ground_residual && params.obb_remove_ground_residual));
    return flags;
}

void WallEndpoint(const WallSegment& segment, float s, float z, float* x,
                  float* y) {
    const Plane& plane = segment.plane;
    const float horizontal = std::sqrt(plane.a * plane.a + plane.b * plane.b);
    const float q = horizontal > 1e-6f ? -(plane.c * z + plane.d) / horizontal : 0.0f;
    const float nx = horizontal > 1e-6f ? plane.a / horizontal : 0.0f;
    const float ny = horizontal > 1e-6f ? plane.b / horizontal : 0.0f;
    *x = segment.tangent_x * s + nx * q;
    *y = segment.tangent_y * s + ny * q;
}

std::array<float, 9> LevelingRotation(float roll_rad, float pitch_rad,
                                      bool use_roll, bool use_pitch) {
    const float roll = use_roll ? roll_rad : 0.0f;
    const float pitch = use_pitch ? pitch_rad : 0.0f;
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    // Ry(pitch) * Rx(roll)
    return {cp, sp * sr, sp * cr,
            0.0f, cr, -sr,
            -sp, cp * sr, cp * cr};
}

void ApplyRotation(const pcl::PointCloud<pcl::PointXYZI>& input,
                   const std::array<float, 9>& r,
                   pcl::PointCloud<pcl::PointXYZI>* output) {
    output->points.resize(input.points.size());
    for (std::size_t i = 0; i < input.points.size(); ++i) {
        const auto& p = input.points[i];
        auto& q = output->points[i];
        q.x = r[0] * p.x + r[1] * p.y + r[2] * p.z;
        q.y = r[3] * p.x + r[4] * p.y + r[5] * p.z;
        q.z = r[6] * p.x + r[7] * p.y + r[8] * p.z;
        q.intensity = p.intensity;  // 기존 CorrectingPoints와 달리 보존한다.
    }
    output->width = static_cast<std::uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = false;
}

int MarkCeiling(const pcl::PointCloud<pcl::PointXYZI>& cloud, float max_z,
                std::vector<std::uint8_t>* labels) {
    int removed = 0;
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        if ((*labels)[i] != kKept) continue;
        if (cloud.points[i].z > max_z) {
            (*labels)[i] = kCeiling;
            ++removed;
        }
    }
    return removed;
}

TunnelPipeline::TunnelPipeline(const Params& params, ObbFunction obb_function)
    : params_(params),
      obb_function_(obb_function ? std::move(obb_function)
                                 : ObbFunction(DefaultObbFunction)),
      ground_filter_(params.ground_reuse_max_frames) {}

void TunnelPipeline::Reset() {
    ground_filter_.Reset();
    for (auto& track : wall_tracks_) track.Reset();
}

float TunnelPipeline::GroundHeightAt(float x, float y) const {
    if (!result_.ground_fallback_used) {
        const float height = result_.ground.plane.HeightAt(x, y);
        if (std::isfinite(height)) return height;
    }
    return result_.ground_reference_height;
}

const FrameResult& TunnelPipeline::Process(
    const pcl::PointCloud<pcl::PointXYZI>& input, float roll_rad,
    float pitch_rad, const std::vector<std::uint8_t>* protect_class,
    const std::vector<std::uint8_t>* ambiguous, bool protection_available,
    const EgoMotion* motion) {
    const auto total_start = Clock::now();
    FrameResult& r = result_;
    r.motion = EgoMotion();
    if (motion != nullptr && motion->valid && std::isfinite(motion->dyaw_rad) &&
        std::isfinite(motion->dx_m) && std::isfinite(motion->dy_m) &&
        motion->dt_s > 0.0f && motion->dt_s <= params_.wall_track_motion_max_dt_s) {
        r.motion = *motion;
    }
    const std::size_t n = input.points.size();
    if (protect_class != nullptr && protect_class->size() != n) {
        protect_class = nullptr;
    }
    if (ambiguous != nullptr && ambiguous->size() != n) ambiguous = nullptr;

    r.leveled.reset(new pcl::PointCloud<pcl::PointXYZI>());
    r.wall_candidate_indices.clear();
    r.wall_candidate_side.clear();
    r.kept_indices.clear();
    r.clusters.clear();
    r.objects.clear();
    r.timings = StageTimings();
    r.input_points = static_cast<int>(n);
    r.roll_rad = roll_rad;
    r.pitch_rad = pitch_rad;
    r.protection_available = protection_available;
    r.protected_points = 0;
    if (protect_class != nullptr) {
        for (const auto value : *protect_class) {
            if (value != kProtectNone) ++r.protected_points;
        }
    }

    auto stage_start = Clock::now();
    r.leveling = LevelingRotation(roll_rad, pitch_rad, params_.level_roll,
                                  params_.level_pitch);
    ApplyRotation(input, r.leveling, r.leveled.get());
    r.labels.assign(n, kKept);
    for (std::size_t i = 0; i < n; ++i) {
        if (!FinitePoint(r.leveled->points[i])) r.labels[i] = kInvalid;
    }
    r.timings.level_ms = ElapsedMs(stage_start);

    stage_start = Clock::now();
    r.ceiling_removed =
        MarkCeiling(*r.leveled, params_.ceiling_max_z_lidar_m, &r.labels);
    r.timings.ceiling_ms = ElapsedMs(stage_start);

    stage_start = Clock::now();
    FitGround();
    r.timings.ground_fit_ms = ElapsedMs(stage_start);

    stage_start = Clock::now();
    RemoveGround();
    r.timings.ground_remove_ms = ElapsedMs(stage_start);

    ProcessWalls(protect_class, ambiguous, protection_available);
    ClusterAndFilter(protect_class);

    r.timings.total_ms = ElapsedMs(total_start);
    return r;
}

void TunnelPipeline::FitGround() {
    FrameResult& r = result_;
    GroundFit fit;
    std::vector<int> band;
    band.reserve(r.leveled->points.size() / 2);
    const float max_range = params_.ground_candidate_max_range_m;
    const float max_range_sq = max_range > 0.0f
        ? max_range * max_range
        : std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < r.leveled->points.size(); ++i) {
        if (r.labels[i] != kKept) continue;
        const auto& p = r.leveled->points[i];
        if (p.z < params_.ground_candidate_z_min_m ||
            p.z > params_.ground_candidate_z_max_m) {
            continue;
        }
        if (p.x * p.x + p.y * p.y > max_range_sq) continue;
        band.push_back(static_cast<int>(i));
    }
    fit.candidates = static_cast<int>(band.size());

    // 후보를 지면 높이 주변으로 좁힌다: 직전 정상 평면(재사용 창 안) ±band,
    // 없으면 근거리 z 히스토그램 최빈값 ±band. 방호벽 하단/차량 하부가 섞인
    // 약간 기울어진 평면이 노면과 경쟁하지 못하게 한다.
    pcl::IndicesPtr candidates(new std::vector<int>());
    candidates->reserve(band.size());
    const Plane* prior = ground_filter_.ConsistencyReference();
    const float prior_band = params_.ground_prior_band_m;
    if (prior != nullptr && prior_band > 0.0f) {
        fit.prior_source = 0;
        fit.prior_height = prior->HeightAt(0.0f, 0.0f);
        for (const int index : band) {
            const auto& p = r.leveled->points[static_cast<std::size_t>(index)];
            if (std::fabs(prior->SignedDistance(p.x, p.y, p.z)) <= prior_band) {
                candidates->push_back(index);
            }
        }
    } else if (prior_band > 0.0f) {
        const float bin = std::max(0.01f, params_.ground_mode_bin_m);
        const float z0 = params_.ground_candidate_z_min_m;
        const int bins = static_cast<int>(std::ceil(
            (params_.ground_candidate_z_max_m - z0) / bin)) + 1;
        std::vector<int> counts(static_cast<std::size_t>(std::max(1, bins)), 0);
        const float mode_range_sq =
            params_.ground_mode_max_range_m * params_.ground_mode_max_range_m;
        for (const int index : band) {
            const auto& p = r.leveled->points[static_cast<std::size_t>(index)];
            if (p.x * p.x + p.y * p.y > mode_range_sq) continue;
            const int k = std::max(0, std::min(bins - 1,
                static_cast<int>(std::floor((p.z - z0) / bin))));
            ++counts[static_cast<std::size_t>(k)];
        }
        const auto best = std::max_element(counts.begin(), counts.end());
        if (*best >= 10) {
            const float mode = z0 + (static_cast<float>(best - counts.begin()) + 0.5f) * bin;
            fit.prior_source = 1;
            fit.prior_height = mode;
            for (const int index : band) {
                const auto& p = r.leveled->points[static_cast<std::size_t>(index)];
                if (std::fabs(p.z - mode) <= prior_band) candidates->push_back(index);
            }
        } else {
            candidates->assign(band.begin(), band.end());
        }
    } else {
        candidates->assign(band.begin(), band.end());
    }
    fit.refined_candidates = static_cast<int>(candidates->size());

    if (candidates->size() >= 3) {
        pcl::SACSegmentation<pcl::PointXYZI> segmentation;
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
        segmentation.setEpsAngle(DegToRad(params_.ground_normal_max_angle_deg));
        segmentation.setMaxIterations(
            std::max(1, params_.ground_ransac_max_iterations));
        segmentation.setDistanceThreshold(
            params_.ground_ransac_distance_threshold_m);
        segmentation.setInputCloud(r.leveled);
        segmentation.setIndices(candidates);
        pcl::PointIndices inliers;
        pcl::ModelCoefficients coefficients;
        segmentation.segment(inliers, coefficients);
        Plane plane;
        if (!inliers.indices.empty() && ToPlane(coefficients, &plane)) {
            if (plane.c < 0.0f) plane = Negated(plane);
            fit.found = true;
            fit.plane = plane;
            fit.inliers = static_cast<int>(inliers.indices.size());
        }
    }

    if (fit.found && params_.ground_refit_enable) {
        const Plane first = fit.plane;
        pcl::IndicesPtr refit_candidates(new std::vector<int>());
        refit_candidates->reserve(band.size());
        for (const int index : band) {
            const auto& p = r.leveled->points[static_cast<std::size_t>(index)];
            if (std::fabs(first.SignedDistance(p.x, p.y, p.z)) <=
                params_.ground_refit_band_m) {
                refit_candidates->push_back(index);
            }
        }
        fit.refit_candidates = static_cast<int>(refit_candidates->size());
        if (refit_candidates->size() >= 3) {
            pcl::SACSegmentation<pcl::PointXYZI> segmentation;
            segmentation.setOptimizeCoefficients(true);
            segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
            segmentation.setEpsAngle(DegToRad(params_.ground_normal_max_angle_deg));
            segmentation.setMaxIterations(std::max(1, params_.ground_refit_iterations));
            segmentation.setDistanceThreshold(params_.ground_refit_distance_m);
            segmentation.setInputCloud(r.leveled);
            segmentation.setIndices(refit_candidates);
            pcl::PointIndices inliers;
            pcl::ModelCoefficients coefficients;
            segmentation.segment(inliers, coefficients);
            Plane second;
            if (!inliers.indices.empty() && ToPlane(coefficients, &second)) {
                if (second.c < 0.0f) second = Negated(second);
                const float threshold = params_.ground_ransac_distance_threshold_m;
                int first_support = 0;
                int second_support = 0;
                for (const int index : band) {
                    const auto& p = r.leveled->points[static_cast<std::size_t>(index)];
                    if (std::fabs(first.SignedDistance(p.x, p.y, p.z)) <= threshold) {
                        ++first_support;
                    }
                    if (std::fabs(second.SignedDistance(p.x, p.y, p.z)) <= threshold) {
                        ++second_support;
                    }
                }
                fit.refit_angle_deg = AngleBetweenNormalsDeg(first, second);
                fit.refit_height_delta_m =
                    std::fabs(second.HeightAt(0.0f, 0.0f) - first.HeightAt(0.0f, 0.0f));
                if (second_support > first_support &&
                    fit.refit_angle_deg <= params_.ground_refit_max_angle_deg &&
                    fit.refit_height_delta_m <= params_.ground_refit_max_height_delta_m) {
                    fit.plane = second;
                    fit.inliers = second_support;
                    fit.refit_used = true;
                }
            }
        }
    }

    ValidateGroundFit(&fit, params_, ground_filter_.ConsistencyReference());
    r.ground_fit = fit;
    r.ground = ground_filter_.Update(fit.valid, fit.plane);
    if (r.ground.source == PlaneSource::kUnavailable) {
        // fail-safe: 평면 대신 마지막으로 알려진 지면 높이(없으면 v3 공칭값)의
        // 평평한 띠를 제거한다. ground_valid=false로 진단값에 남는다.
        r.ground_fallback_used = true;
        r.ground_reference_height = ground_filter_.HasEverBeenValid()
            ? ground_filter_.LastValid().HeightAt(0.0f, 0.0f)
            : params_.expected_ground_z_lidar_m;
    } else {
        r.ground_fallback_used = false;
        r.ground_reference_height = r.ground.plane.HeightAt(0.0f, 0.0f);
    }
}

void TunnelPipeline::RemoveGround() {
    FrameResult& r = result_;
    int removed = 0;
    const float threshold = params_.ground_remove_distance_m;
    if (!r.ground_fallback_used) {
        const Plane& plane = r.ground.plane;
        for (std::size_t i = 0; i < r.leveled->points.size(); ++i) {
            if (r.labels[i] != kKept) continue;
            const auto& p = r.leveled->points[i];
            const float distance = plane.SignedDistance(p.x, p.y, p.z);
            const bool below_ok =
                params_.ground_remove_below_plane || distance >= -threshold;
            if (distance <= threshold && below_ok) {
                r.labels[i] = kGround;
                ++removed;
            }
        }
    } else {
        const float limit =
            r.ground_reference_height + params_.ground_fallback_band_m;
        for (std::size_t i = 0; i < r.leveled->points.size(); ++i) {
            if (r.labels[i] != kKept) continue;
            if (r.leveled->points[i].z <= limit) {
                r.labels[i] = kGround;
                ++removed;
            }
        }
    }
    r.ground_removed = removed;
}

bool TunnelPipeline::FitWallSegment(const std::vector<int>& pool,
                                    const std::vector<std::uint8_t>* protect_class,
                                    WallSegment* segment,
                                    std::vector<int>* supported_inliers) const {
    const FrameResult& r = result_;
    supported_inliers->clear();
    *segment = WallSegment();
    segment->reject_reason = "no_model";
    if (pool.size() < 3) return false;

    pcl::IndicesPtr pool_indices(new std::vector<int>(pool));
    pcl::SACSegmentation<pcl::PointXYZI> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    segmentation.setEpsAngle(DegToRad(params_.wall_max_tilt_deg));
    segmentation.setMaxIterations(std::max(1, params_.wall_ransac_max_iterations));
    segmentation.setDistanceThreshold(params_.wall_fit_distance_m);
    segmentation.setInputCloud(r.leveled);
    segmentation.setIndices(pool_indices);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(inliers, coefficients);
    Plane plane;
    if (inliers.indices.empty() || !ToPlane(coefficients, &plane)) return false;
    // 법선이 센서 원점(차로) 쪽을 향하게 한다. d는 원점에서 평면까지의 거리.
    if (plane.d < 0.0f) plane = Negated(plane);
    segment->plane = plane;
    segment->lateral_distance = plane.d;
    const float horizontal = std::sqrt(plane.a * plane.a + plane.b * plane.b);
    if (horizontal < 1e-6f) {
        segment->reject_reason = "not_vertical";
        return true;
    }
    segment->tangent_x = -plane.b / horizontal;
    segment->tangent_y = plane.a / horizontal;

    // 평면 위의 전체 점: 유지 중·비보호·벽 높이 띠·x 범위 안에서 평면 거리 이내.
    // 좌/우 후보 영역(y)은 RANSAC 입력에만 쓰고 여기서는 쓰지 않는다.
    std::vector<int> band_points;   // 벽 높이 띠의 모든 유지점 (바깥쪽 검사용)
    std::vector<int> plane_points;  // 평면 지지점
    std::vector<float> s_values;
    for (std::size_t i = 0; i < r.leveled->points.size(); ++i) {
        if (r.labels[i] != kKept) continue;
        const auto& p = r.leveled->points[i];
        if (p.x < params_.wall_search_x_min_m || p.x > params_.wall_search_x_max_m) {
            continue;
        }
        const float z_road = p.z - GroundHeightAt(p.x, p.y);
        if (z_road < params_.wall_candidate_min_height_m ||
            z_road > params_.wall_candidate_max_height_m) {
            continue;
        }
        band_points.push_back(static_cast<int>(i));
        if (protect_class != nullptr && (*protect_class)[i] != kProtectNone) continue;
        if (std::fabs(plane.SignedDistance(p.x, p.y, p.z)) >
            params_.wall_fit_distance_m) {
            continue;
        }
        plane_points.push_back(static_cast<int>(i));
        s_values.push_back(segment->AlongWall(p.x, p.y));
    }
    const std::vector<WallSupport> runs = ComputeWallRuns(
        s_values, params_.wall_bin_size_m, params_.wall_bin_min_points,
        params_.wall_bin_max_gap);
    if (runs.empty()) {
        segment->reject_reason = "no_support";
        return true;
    }
    // 검증은 가장 긴 연속 구간 하나로 한다.
    const WallSupport& support = runs.front();
    segment->span = support.s_end - support.s_start;
    segment->occupied_bins = support.occupied_bins;
    segment->intervals.assign(1, {support.s_start, support.s_end});
    // 같은 평면 위의 인접 구간은 틈이 작고 충분히 길면 제거 구간에 포함한다.
    bool extended = true;
    while (extended) {
        extended = false;
        for (std::size_t k = 1; k < runs.size(); ++k) {
            const WallSupport& run = runs[k];
            if (run.s_end - run.s_start < params_.wall_extend_min_run_m) continue;
            bool already = false;
            float gap = std::numeric_limits<float>::infinity();
            for (const auto& interval : segment->intervals) {
                if (run.s_start >= interval.first && run.s_end <= interval.second) {
                    already = true;
                    break;
                }
                gap = std::min(gap, std::max(run.s_start - interval.second,
                                             interval.first - run.s_end));
            }
            if (already || gap > params_.wall_extend_max_gap_m) continue;
            segment->intervals.emplace_back(run.s_start, run.s_end);
            extended = true;
        }
    }
    segment->s_min = std::numeric_limits<float>::infinity();
    segment->s_max = -std::numeric_limits<float>::infinity();
    for (const auto& interval : segment->intervals) {
        segment->s_min = std::min(segment->s_min, interval.first);
        segment->s_max = std::max(segment->s_max, interval.second);
    }

    float z_road_min = std::numeric_limits<float>::infinity();
    float z_road_max = -z_road_min;
    int supported = 0;
    for (std::size_t k = 0; k < plane_points.size(); ++k) {
        if (!segment->InIntervals(s_values[k], 0.0f)) continue;
        ++supported;
        const auto& p = r.leveled->points[static_cast<std::size_t>(plane_points[k])];
        const float z_road = p.z - GroundHeightAt(p.x, p.y);
        z_road_min = std::min(z_road_min, z_road);
        z_road_max = std::max(z_road_max, z_road);
    }
    // 다음 평면 탐색에서 뺄 pool 점 (지지 구간 안에서 평면 거리 이내).
    for (const int index : pool) {
        const auto& p = r.leveled->points[static_cast<std::size_t>(index)];
        if (std::fabs(plane.SignedDistance(p.x, p.y, p.z)) >
            params_.wall_fit_distance_m) {
            continue;
        }
        if (segment->InIntervals(segment->AlongWall(p.x, p.y), 0.0f)) {
            supported_inliers->push_back(index);
        }
    }
    segment->inliers = supported;
    segment->pool_support = static_cast<int>(supported_inliers->size());
    segment->inlier_ratio = pool.empty()
        ? 0.0f
        : static_cast<float>(supported_inliers->size()) /
              static_cast<float>(pool.size());
    segment->vertical_span = supported > 0 ? z_road_max - z_road_min : 0.0f;

    int in_range = 0;
    int beyond = 0;
    for (const int index : band_points) {
        const auto& p = r.leveled->points[static_cast<std::size_t>(index)];
        const float s = segment->AlongWall(p.x, p.y);
        if (!segment->InIntervals(s, 0.0f)) continue;
        ++in_range;
        if (plane.SignedDistance(p.x, p.y, p.z) < -params_.wall_beyond_margin_m) {
            ++beyond;
        }
    }
    segment->beyond_fraction = in_range > 0
        ? static_cast<float>(beyond) / static_cast<float>(in_range)
        : 0.0f;
    ValidateWallSegment(segment, params_);
    return true;
}

void TunnelPipeline::ProcessWalls(
    const std::vector<std::uint8_t>* protect_class,
    const std::vector<std::uint8_t>* ambiguous, bool protection_available) {
    FrameResult& r = result_;
    const auto fit_start = Clock::now();
    const std::size_t n = r.leveled->points.size();
    for (auto& side : r.walls) side = WallSideResult();
    r.wall_removed = 0;

    // 지면 높이를 모르면 벽 후보 높이도 모른다: 벽 제거를 하지 않는다 (fail-open).
    if (!params_.wall_enable || r.ground_fallback_used) {
        for (std::size_t side = 0; side < 2; ++side) {
            wall_tracks_[side].Update(std::vector<WallSegment>(), params_, &r.motion);
            r.walls[side].track = wall_tracks_[side].State();
            r.walls[side].first_attempt.reject_reason =
                params_.wall_enable ? "ground_unavailable" : "disabled";
        }
        r.timings.wall_fit_ms = ElapsedMs(fit_start);
        return;
    }

    std::array<std::vector<int>, 2> candidates;
    for (std::size_t i = 0; i < n; ++i) {
        if (r.labels[i] != kKept) continue;
        if (protect_class != nullptr && (*protect_class)[i] != kProtectNone) {
            continue;  // 보호점은 벽 평면 추정에도 쓰지 않는다.
        }
        const auto& p = r.leveled->points[i];
        if (p.x < params_.wall_search_x_min_m || p.x > params_.wall_search_x_max_m) {
            continue;
        }
        const float z_road = p.z - GroundHeightAt(p.x, p.y);
        if (z_road < params_.wall_candidate_min_height_m ||
            z_road > params_.wall_candidate_max_height_m) {
            continue;
        }
        if (p.y >= params_.wall_search_min_abs_y_m) {
            candidates[0].push_back(static_cast<int>(i));
        } else if (p.y <= -params_.wall_search_min_abs_y_m) {
            candidates[1].push_back(static_cast<int>(i));
        }
    }

    std::vector<std::uint8_t> in_pool(n, 0);
    for (std::size_t side = 0; side < 2; ++side) {
        WallSideResult& result = r.walls[side];
        result.candidates = static_cast<int>(candidates[side].size());
        for (const int index : candidates[side]) {
            r.wall_candidate_indices.push_back(index);
            r.wall_candidate_side.push_back(static_cast<std::uint8_t>(side + 1));
            in_pool[static_cast<std::size_t>(index)] = 1;
        }
        std::vector<int> pool = candidates[side];
        std::vector<int> supported;
        std::vector<WallSegment> fitted;
        const int max_segments = std::max(1, params_.wall_max_segments);
        for (int segment_index = 0; segment_index < max_segments; ++segment_index) {
            if (static_cast<int>(pool.size()) <
                std::max(3, params_.wall_min_inliers)) {
                if (segment_index == 0) {
                    result.first_attempt.reject_reason = "few_candidates";
                }
                break;
            }
            WallSegment segment;
            const bool ok =
                FitWallSegment(pool, protect_class, &segment, &supported);
            if (!ok) {
                if (segment_index == 0) result.first_attempt = segment;
                break;
            }
            // 벽의 좌/우는 평면 위에서 차량에 가장 가까운 점(수선의 발)의 y 부호로
            // 정한다. 차량이 틀어져 반대쪽 벽의 먼 부분이 이쪽 후보에 섞여도
            // 그 평면을 이쪽 벽으로 삼지 않는다 (요잉과 무관).
            const float foot_y = -segment.plane.d * segment.plane.b;
            const bool wrong_side = side == 0 ? foot_y <= 0.0f : foot_y >= 0.0f;
            if (wrong_side && segment.valid) {
                segment.valid = false;
                segment.reject_reason = "other_side_wall";
            }
            if (segment_index == 0) result.first_attempt = segment;
            result.attempts.push_back(segment);
            result.attempts.back().reject_reason = "seq:" + segment.reject_reason;
            // 높이 범위나 길이만 부족한 평면(하부 방호벽, 곡선 구간 조각)과 반대쪽 벽은
            // 보류하고 다음 평면을 찾는다. 수직도/바깥쪽/inlier 실패는 거기서 멈춘다.
            const bool plausible =
                segment.valid || segment.reject_reason == "low_vertical_span" ||
                segment.reject_reason == "short_span" ||
                segment.reject_reason == "few_bins" ||
                segment.reject_reason == "other_side_wall";
            if (!plausible) break;
            if (!wrong_side) fitted.push_back(segment);
            for (const int index : supported) {
                in_pool[static_cast<std::size_t>(index)] = 0;
            }
            std::vector<int> next_pool;
            next_pool.reserve(pool.size());
            for (const int index : pool) {
                if (in_pool[static_cast<std::size_t>(index)]) {
                    next_pool.push_back(index);
                }
            }
            pool.swap(next_pool);
        }
        for (const int index : candidates[side]) {
            in_pool[static_cast<std::size_t>(index)] = 0;
        }
        // 주 평면: 모든 조건을 만족한 평면 중 이쪽 후보점 지지가 가장 많은 것.
        int primary = -1;
        for (std::size_t k = 0; k < fitted.size(); ++k) {
            if (!fitted[k].valid) continue;
            if (primary < 0 ||
                fitted[k].pool_support >
                    fitted[static_cast<std::size_t>(primary)].pool_support) {
                primary = static_cast<int>(k);
            }
        }
        if (primary >= 0) {
            const WallSegment main_plane = fitted[static_cast<std::size_t>(primary)];
            result.segments.push_back(main_plane);
            // 보조 평면 인정은 두 경우뿐이다.
            //  (1) 단차: 주 평면과 평행하고 벽 두께 층 안(|Δd| <= step_max_offset)
            //  (2) 곡선 연속: 주 평면 지지 구간 밖으로 이어지는 소각 평면
            // 벽보다 차로 쪽에 떠 있는 평면(사람·차량·연석이 섞인 가짜 평면)은
            // 길이/높이 조건을 만족해도 거부한다 (bag 16.1 s 사람 손실 재현).
            const auto accept_secondary = [&](WallSegment* segment) {
                // 두 평면 모두 수직도(tilt) 검증을 통과했으므로 방위각만 비교한다.
                // 3D 각도는 반대 방향 tilt(예: +4°/-1°)가 더해져 같은 벽의 단차를
                // 놓친다 (bag 14.3~20.9 s 좌측 방호벽 잔류).
                const float angle =
                    HorizontalAngleBetweenNormalsDeg(segment->plane, main_plane.plane);
                const bool near = std::fabs(segment->plane.d - main_plane.plane.d) <=
                                  params_.wall_step_max_offset_m;
                if (angle <= params_.wall_step_max_angle_deg && near) {
                    if (ValidateWallSegment(segment, params_,
                                            params_.wall_step_min_vertical_span_m)) {
                        segment->step = true;
                        return true;
                    }
                    segment->reject_reason = "step_" + segment->reject_reason;
                    return false;
                }
                // 주 평면 구간과 많이 겹치면 연장이 아니다.
                float length = 0.0f;
                float overlap = 0.0f;
                for (const auto& own : segment->intervals) {
                    length += own.second - own.first;
                    for (const auto& other : main_plane.intervals) {
                        overlap += std::max(0.0f, std::min(own.second, other.second) -
                                                      std::max(own.first, other.first));
                    }
                }
                if (length <= 0.0f || overlap > 0.3f * length) {
                    segment->reject_reason = "overlaps_primary";
                    return false;
                }
                const float z_mid = r.ground_reference_height + 1.5f;
                float gap = std::numeric_limits<float>::infinity();
                for (const auto& own : segment->intervals) {
                    for (const auto& other : main_plane.intervals) {
                        for (const float sa : {own.first, own.second}) {
                            for (const float sb : {other.first, other.second}) {
                                float xa, ya, xb, yb;
                                WallEndpoint(*segment, sa, z_mid, &xa, &ya);
                                WallEndpoint(main_plane, sb, z_mid, &xb, &yb);
                                gap = std::min(gap, std::hypot(xa - xb, ya - yb));
                            }
                        }
                    }
                }
                if (angle <= params_.wall_continuation_max_angle_deg &&
                    gap <= params_.wall_continuation_max_gap_m &&
                    ValidateWallSegment(segment, params_, -1.0f,
                                        params_.wall_continuation_min_span_m,
                                        params_.wall_continuation_min_bins)) {
                    segment->continuation = true;
                    return true;
                }
                if (segment->reject_reason == "ok") {
                    segment->reject_reason = "not_step_or_continuation";
                }
                return false;
            };
            for (std::size_t k = 0; k < fitted.size(); ++k) {
                if (static_cast<int>(k) == primary) continue;
                WallSegment segment = fitted[k];
                const bool accepted = accept_secondary(&segment);
                if (accepted) result.segments.push_back(segment);
                result.attempts.push_back(segment);
                result.attempts.back().reject_reason =
                    std::string("sec:") + (accepted ? (segment.step ? "step" : "continuation")
                                                    : "rejected:" + segment.reject_reason);
            }
            // 차량이 터널에 대해 틀어지면 전방 벽은 차로 앞(|y| 작음)을 가로질러
            // 좌/우 후보 영역에 들지 않는다. 주 평면 연장선 주변(y 무관)에서 아직
            // 덮이지 않은 벽 높이 점으로 연속 평면을 한 번 더 찾는다.
            if (params_.wall_continuation_search_m > 0.0f) {
                std::vector<int> cont_pool;
                for (std::size_t i = 0; i < n; ++i) {
                    if (r.labels[i] != kKept) continue;
                    if (protect_class != nullptr &&
                        (*protect_class)[i] != kProtectNone) {
                        continue;
                    }
                    const auto& p = r.leveled->points[i];
                    if (p.x < params_.wall_search_x_min_m ||
                        p.x > params_.wall_search_x_max_m) {
                        continue;
                    }
                    const float z_road = p.z - GroundHeightAt(p.x, p.y);
                    if (z_road < params_.wall_candidate_min_height_m ||
                        z_road > params_.wall_candidate_max_height_m) {
                        continue;
                    }
                    if (std::fabs(main_plane.plane.SignedDistance(p.x, p.y, p.z)) >
                        params_.wall_continuation_search_m) {
                        continue;
                    }
                    // 연장만 찾는다: 주 평면 지지 구간(+여유) 안쪽 점은 쓰지 않는다.
                    if (main_plane.InIntervals(main_plane.AlongWall(p.x, p.y),
                                               params_.wall_remove_margin_m)) {
                        continue;
                    }
                    bool covered = false;
                    for (const WallSegment& accepted : result.segments) {
                        if (std::fabs(accepted.plane.SignedDistance(p.x, p.y, p.z)) <=
                                params_.wall_fit_distance_m &&
                            accepted.InIntervals(accepted.AlongWall(p.x, p.y), 0.0f)) {
                            covered = true;
                            break;
                        }
                    }
                    if (!covered) cont_pool.push_back(static_cast<int>(i));
                }
                for (int attempt = 0; attempt < 2; ++attempt) {
                    if (static_cast<int>(cont_pool.size()) <
                        std::max(3, params_.wall_min_inliers)) {
                        break;
                    }
                    WallSegment segment;
                    if (!FitWallSegment(cont_pool, protect_class, &segment, &supported)) {
                        break;
                    }
                    const bool accepted = accept_secondary(&segment);
                    result.attempts.push_back(segment);
                    result.attempts.back().reject_reason =
                        std::string("cont:") + (accepted ? "ok" : "rejected:" + segment.reject_reason);
                    if (!accepted) break;
                    result.segments.push_back(segment);
                    std::vector<std::uint8_t> used(n, 0);
                    for (const int index : supported) {
                        used[static_cast<std::size_t>(index)] = 1;
                    }
                    std::vector<int> next_pool;
                    for (const int index : cont_pool) {
                        if (!used[static_cast<std::size_t>(index)]) next_pool.push_back(index);
                    }
                    if (next_pool.size() == cont_pool.size()) break;
                    cont_pool.swap(next_pool);
                }
            }
        }
        result.fit_valid = !result.segments.empty();
        result.removal_allowed =
            wall_tracks_[side].Update(result.segments, params_, &r.motion);
        result.track = wall_tracks_[side].State();
    }
    r.timings.wall_fit_ms = ElapsedMs(fit_start);

    const auto remove_start = Clock::now();
    if (r.walls[0].removal_allowed || r.walls[1].removal_allowed) {
        for (std::size_t i = 0; i < n; ++i) {
            if (r.labels[i] != kKept) continue;
            const auto& p = r.leveled->points[i];
            for (std::size_t side = 0; side < 2; ++side) {
                WallSideResult& result = r.walls[side];
                if (!result.removal_allowed) continue;
                bool matched = false;
                for (const WallSegment& segment : result.segments) {
                    const float distance =
                        segment.plane.SignedDistance(p.x, p.y, p.z);
                    if (std::fabs(distance) > params_.wall_remove_distance_m) {
                        continue;
                    }
                    const float s = segment.AlongWall(p.x, p.y);
                    if (!segment.InIntervals(s, params_.wall_remove_margin_m)) {
                        continue;
                    }
                    const float z_road = p.z - GroundHeightAt(p.x, p.y);
                    if (z_road < params_.wall_remove_min_height_m) continue;
                    matched = true;
                    break;
                }
                if (!matched) continue;
                if (protect_class != nullptr &&
                    (*protect_class)[i] != kProtectNone) {
                    ++result.protected_skipped;
                } else if (!protection_available && ambiguous != nullptr &&
                           (*ambiguous)[i]) {
                    ++result.ambiguous_skipped;
                } else {
                    r.labels[i] = kWall;
                    ++result.removed;
                    ++r.wall_removed;
                }
                break;
            }
        }
    }
    r.timings.wall_remove_ms = ElapsedMs(remove_start);
}

void TunnelPipeline::ClusterAndFilter(
    const std::vector<std::uint8_t>* protect_class) {
    FrameResult& r = result_;
    const std::size_t n = r.leveled->points.size();
    for (std::size_t i = 0; i < n; ++i) {
        if (r.labels[i] == kKept) r.kept_indices.push_back(static_cast<int>(i));
    }
    r.kept_points = static_cast<int>(r.kept_indices.size());

    auto stage_start = Clock::now();
    r.cluster_id.assign(n, -1);
    if (!r.kept_indices.empty()) {
        pcl::IndicesPtr kept(new std::vector<int>(r.kept_indices));
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZI>());
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
        extraction.setClusterTolerance(params_.cluster_tolerance_m);
        extraction.setMinClusterSize(std::max(1, params_.cluster_min_points));
        extraction.setMaxClusterSize(std::max(1, params_.cluster_max_points));
        extraction.setSearchMethod(tree);
        extraction.setInputCloud(r.leveled);
        extraction.setIndices(kept);
        extraction.extract(r.clusters);
    }
    for (std::size_t c = 0; c < r.clusters.size(); ++c) {
        for (const int index : r.clusters[c].indices) {
            r.cluster_id[static_cast<std::size_t>(index)] = static_cast<int>(c);
        }
    }
    r.timings.cluster_ms = ElapsedMs(stage_start);

    stage_start = Clock::now();
    std::vector<Obb> obbs;
    if (!r.clusters.empty()) obb_function_(r.leveled, r.clusters, &obbs);
    obbs.resize(r.clusters.size());
    r.timings.obb_ms = ElapsedMs(stage_start);

    stage_start = Clock::now();
    r.objects_removed_ceiling = 0;
    r.objects_removed_wall = 0;
    r.objects_ground_residual = 0;
    r.objects.reserve(r.clusters.size());
    for (std::size_t c = 0; c < r.clusters.size(); ++c) {
        TunnelObject object;
        object.obb = obbs[c];
        object.point_count = static_cast<int>(r.clusters[c].indices.size());
        std::array<int, 6> class_votes{};
        if (protect_class != nullptr) {
            for (const int index : r.clusters[c].indices) {
                const std::uint8_t value =
                    (*protect_class)[static_cast<std::size_t>(index)];
                if (value != kProtectNone && value < class_votes.size()) {
                    ++object.protected_points;
                    ++class_votes[value];
                }
            }
        }
        const int needed = std::max(
            params_.protect_obb_min_points,
            static_cast<int>(std::ceil(params_.protect_obb_min_fraction *
                                       static_cast<float>(object.point_count))));
        object.is_protected = object.protected_points > 0 &&
                              object.protected_points >= needed;
        if (object.protected_points > 0) {
            const auto best =
                std::max_element(class_votes.begin() + 1, class_votes.end());
            object.protect_class = static_cast<std::uint8_t>(
                std::distance(class_votes.begin(), best));
        }
        const float ground = GroundHeightAt(object.obb.cx, object.obb.cy);
        object.z_min_road = object.obb.z_min - ground;
        object.z_max_road = object.obb.z_max - ground;
        // 확정(제거 적용) 벽 평면에 붙어 있고 평행한 OBB인지.
        bool wall_attached = false;
        const float axis_x = std::cos(object.obb.yaw);
        const float axis_y = std::sin(object.obb.yaw);
        const float max_sin = std::sin(
            DegToRad(params_.obb_wall_attach_max_angle_deg));
        for (const WallSideResult& wall : r.walls) {
            if (!wall.removal_allowed || wall_attached) continue;
            for (const WallSegment& segment : wall.segments) {
                const float distance = std::fabs(segment.plane.SignedDistance(
                    object.obb.cx, object.obb.cy, object.obb.cz));
                const float cross = std::fabs(axis_x * segment.tangent_y -
                                              axis_y * segment.tangent_x);
                const float s = segment.AlongWall(object.obb.cx, object.obb.cy);
                if (distance <= params_.obb_wall_attach_distance_m &&
                    cross <= max_sin &&
                    segment.InIntervals(s, params_.wall_remove_margin_m)) {
                    wall_attached = true;
                    break;
                }
            }
        }
        object.flags = ClassifyObb(object.obb, object.z_min_road,
                                   object.z_max_road, object.is_protected,
                                   wall_attached, params_);
        if (object.flags.removed && object.flags.ceiling) {
            ++r.objects_removed_ceiling;
        } else if (object.flags.removed && object.flags.wall) {
            ++r.objects_removed_wall;
        }
        if (object.flags.ground_residual) ++r.objects_ground_residual;
        r.objects.push_back(object);
    }
    r.timings.obb_filter_ms = ElapsedMs(stage_start);
}

}  // namespace tunnel_filter
