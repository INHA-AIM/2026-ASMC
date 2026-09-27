#ifndef TUNNEL_FILTER_CORE_HPP
#define TUNNEL_FILTER_CORE_HPP

// GPS 음영구역(터널) 전용 구조물 제거 실험 파이프라인의 순수 로직.
//
// ROS 헤더에 의존하지 않으므로 단위 테스트와 오프라인 평가기에서 같은 코드를
// 그대로 쓴다. 일반도로 경로(CorrectingPoints/NormalVectorFiltering/Clustering)는
// 전혀 건드리지 않고, 이 파이프라인은 터널 모드에서만 병렬로 실행된다.
//
// 좌표계: 입력은 velodyne 센서 좌표(ROI+Voxel 결과)다. 내부 처리와 출력은
// IMU roll/pitch로 수평화한 velodyne 원점 좌표이며, yaw와 원점은 velodyne과
// 같다. 기존 /lidar/debug_ransac이 pitch 보정 좌표를 velodyne으로 표기하는
// 관례와 동일하다.

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace tunnel_filter {

// 기존 /lidar_nogps 경로와 같은 게이트다. Parser가 계산한 두 플래그를 그대로
// 쓴다: 무효 GPS 한 샘플로 즉시 진입(b_isJamming), 정상 GPS 3회 연속 뒤에만
// 해제(b_isTunnel 히스테리시스).
inline bool IsGpsDenied(bool gps_invalid_sample, bool tunnel_latched) {
    return gps_invalid_sample || tunnel_latched;
}

inline bool ShouldRunExperiment(bool enabled, bool gps_denied) {
    return enabled && gps_denied;
}

enum PointLabel : std::uint8_t {
    kKept = 0,
    kCeiling = 1,
    kGround = 2,
    kWall = 3,
    kInvalid = 4,  // 좌표가 finite가 아닌 점
};

enum ProtectClass : std::uint8_t {
    kProtectNone = 0,
    kProtectCar = 1,
    kProtectTruck = 2,
    kProtectBus = 3,
    kProtectPerson = 4,
    kProtectPedestrian = 5,
};

const char* ProtectClassName(std::uint8_t protect_class);
// 소문자/대문자 무관. 보호 대상이 아니면 kProtectNone.
std::uint8_t ProtectClassFromLabel(const std::string& label);
bool IsPersonClass(std::uint8_t protect_class);

struct Params {
    // GPS 음영일 때만 기존 ROI 결과를 한 번 더 줄인다. 일반도로 ROI 설정은
    // 변경하지 않으므로 음영 해제 다음 프레임부터 원래 범위로 자동 복귀한다.
    bool dynamic_roi_enable = true;
    float roi_min_x_m = -30.0f;
    float roi_max_x_m = 40.0f;
    float roi_min_y_m = -10.0f;
    float roi_max_y_m = 10.0f;
    float roi_min_z_m = -2.0f;
    float roi_max_z_m = 3.0f;

    // --- IMU 수평화 (yaw는 사용하지 않는다) ---
    bool level_roll = true;
    bool level_pitch = true;

    // --- 천장 선제 제거 (수평화된 velodyne z 기준) ---
    // GPS가 아직 정상인 터널 입구에서도 같은 높이 컷을 기존 경로의 RANSAC
    // 직전에 적용한다. false면 GPS 음영 터널 파이프라인 안에서만 제거한다.
    bool pre_gps_ceiling_filter_enable = true;
    float ceiling_max_z_lidar_m = 2.55f;
    // z 상한으로 잘린 천장과 양쪽 벽이 하나의 거대 cluster로 다시 합쳐지는
    // 터널 입구 사례를 /last_lidar 직전에서 한 번 더 막는다. 최소 폭 6 m는
    // 일반 승용차·버스·트럭 OBB와 터널 횡단 구조물을 구분하는 안전 조건이다.
    bool pre_gps_structural_obb_filter_enable = true;
    float pre_gps_structural_obb_min_length_m = 15.0f;
    float pre_gps_structural_obb_min_width_m = 6.0f;
    float pre_gps_structural_obb_min_height_m = 3.0f;

    // --- 터널 전용 지면 RANSAC ---
    float expected_ground_z_lidar_m = -1.45f;
    float ground_candidate_z_min_m = -1.90f;
    float ground_candidate_z_max_m = -1.00f;
    float ground_candidate_max_range_m = 40.0f;  // <=0 이면 제한 없음
    // 넓은 후보 띠에는 방호벽 하단·차량 하부도 섞인다. 직전 정상 평면(재사용 창
    // 안)이 있으면 그 평면 ±band, 없으면 근거리 z 히스토그램 최빈값 ±band 안의
    // 점만 RANSAC에 넣는다.
    // RANSAC 거리(0.15~0.20)보다 넓으면 노면 위 0.2 m 턱이 같은 평면으로
    // 묶여 평면이 들린다 (합성 테스트와 bag 11.7 s 구간에서 확인).
    float ground_prior_band_m = 0.15f;
    float ground_mode_bin_m = 0.05f;
    float ground_mode_max_range_m = 20.0f;
    int ground_ransac_max_iterations = 100;
    float ground_ransac_distance_threshold_m = 0.18f;
    // 2단계 재적합: 1단계 평면 ±band 안에서 더 좁은 거리로 다시 적합한다.
    // 직전 평면 띠(prior)는 처리 간격이 길거나 수평화 오차가 있으면 원거리 노면을
    // 잘라 평면이 근거리로 치우친다 (실시간 노드 스캔 건너뜀 재현 시 지면 flicker).
    // 채택 조건: 제거 거리 안 후보점이 더 많고, 1단계와 각도·원점 높이 차가 작을 것
    // (높이 제한이 방호벽 하단 같은 옆 단차에 지면이 들리는 것을 막는다).
    bool ground_refit_enable = true;
    float ground_refit_band_m = 0.25f;
    float ground_refit_distance_m = 0.10f;
    int ground_refit_iterations = 100;
    float ground_refit_max_angle_deg = 3.0f;
    float ground_refit_max_height_delta_m = 0.05f;
    float ground_normal_max_angle_deg = 15.0f;
    float ground_height_min_m = -1.90f;  // 원점 부근 평면 높이 허용 범위
    float ground_height_max_m = -1.00f;
    int ground_min_inliers = 150;
    float ground_min_inlier_ratio = 0.30f;
    float ground_max_height_jump_m = 0.15f;  // 직전 정상 평면 대비
    float ground_max_normal_jump_deg = 3.0f;
    int ground_reuse_max_frames = 3;
    float ground_remove_distance_m = 0.18f;
    // 평면 아래(노면 밑) 점도 함께 제거한다. 터널 노면 아래에는 장애물이
    // 있을 수 없고 다중반사 잡음만 남기 때문이다.
    bool ground_remove_below_plane = true;
    // 4프레임 이상 지면을 못 찾으면(unavailable) 마지막 정상 높이(없으면
    // expected_ground_z) + 이 띠 이하를 제거하는 fail-safe로 동작한다.
    float ground_fallback_band_m = 0.20f;

    // --- 좌우 벽 평면 ---
    bool wall_enable = true;
    float wall_search_min_abs_y_m = 2.75f;  // 보수적 코어 기본값; launch에서 1.75로 설정
    // ROI 전체(x -30~50). -10에서 자르면 차량 뒤쪽 벽이 20 m 객체로 남는다.
    float wall_search_x_min_m = -30.0f;
    float wall_search_x_max_m = 50.0f;
    float wall_candidate_min_height_m = 0.30f;  // 지면 기준
    // 상한은 사실상 천장 컷이 정한다. 2단 벽의 상부(노면 1.5 m~)가 높이 범위
    // 조건(2 m)을 만족하려면 3.5 m보다 높아야 한다.
    float wall_candidate_max_height_m = 4.50f;
    int wall_ransac_max_iterations = 200;
    float wall_fit_distance_m = 0.15f;
    // 법선과 수평면 사이 각. 이 bag의 터널 벽은 수직 2단(하부 방호벽+상부 벽)이며,
    // 큰 허용각은 두 단을 가로지르는 경사 평면을 만들어 상단을 남긴다.
    float wall_max_tilt_deg = 5.0f;
    int wall_min_inliers = 80;
    float wall_min_inlier_ratio = 0.20f;
    float wall_min_span_m = 15.0f;
    float wall_bin_size_m = 2.0f;
    int wall_bin_min_points = 3;
    int wall_bin_max_gap = 1;
    int wall_min_occupied_bins = 6;
    float wall_min_vertical_span_m = 2.0f;
    float wall_min_lateral_distance_m = 1.5f;
    float wall_beyond_margin_m = 0.5f;
    float wall_max_beyond_fraction = 0.25f;
    int wall_max_segments = 4;  // 곡선/단차 구간용 piecewise 평면 수
    // 15 m 이상 연속 지지로 검증된 평면 위에서, 가림/저밀도로 끊긴 인접 지지
    // 구간(길이 >= min_run, 틈 <= max_gap)만 같은 벽으로 제거 구간에 포함한다.
    float wall_extend_min_run_m = 4.0f;
    float wall_extend_max_gap_m = 12.0f;
    // 주 평면과 평행하고 가까운 보조 평면(방호벽 단차)은 높이 범위가 낮아도
    // 같은 벽 구조로 인정한다. span/연속성/inlier/바깥쪽 조건은 그대로 적용.
    float wall_step_max_angle_deg = 5.0f;
    // 벽 두께 층: bag의 방호벽-상부벽 단차 0.35~0.5 m. 이보다 앞쪽 평면은 벽이 아니다.
    float wall_step_max_offset_m = 0.6f;
    float wall_step_min_vertical_span_m = 0.8f;
    // 곡선 구간: 주 평면 끝에 이어지고(끝점 간격 <= max_gap) 약간 꺾인
    // (<= max_angle) 보조 평면은 짧은 연속 지지(>= min_span)로도 같은 벽으로
    // 인정한다. 높이 범위·inlier·바깥쪽 조건은 그대로 적용한다.
    float wall_continuation_max_angle_deg = 12.0f;
    float wall_continuation_max_gap_m = 6.0f;
    float wall_continuation_min_span_m = 6.0f;
    int wall_continuation_min_bins = 3;
    // 주 평면 연장선 ±이 거리 안(y 무관)에서 연속 평면을 추가로 찾는다. 0이면 끈다.
    float wall_continuation_search_m = 2.0f;
    int wall_min_persist_frames = 3;
    float wall_track_max_angle_deg = 6.0f;
    float wall_track_max_offset_m = 0.6f;
    int wall_track_max_miss = 2;
    // 이전 벽 평면을 자차 이동(IMU yaw 변화, 속도×dt)으로 현재 좌표계에 옮긴 뒤
    // 비교한다. 실시간 노드는 스캔을 건너뛰어(bag 재생 시 처리 간격 130~280 ms)
    // 요레이트 33°/s에서 정적인 벽도 프레임 사이 방위가 6° 이상 돈다.
    bool wall_track_use_ego_motion = true;
    float wall_track_motion_max_dt_s = 1.0f;  // 이보다 간격이 크면 보정하지 않는다
    float wall_remove_distance_m = 0.15f;  // 보수적 코어 기본값; launch에서 0.25로 설정
    float wall_remove_margin_m = 1.0f;  // 지지 구간 양끝 여유
    float wall_remove_min_height_m = 0.0f;

    // --- 터널 전용 클러스터링 ---
    // bag sweep(0.5/0.7/1.0/1.5): 1.0에서 원거리 차량 보존이 가장 좋고(0.5/0.7은
    // 점이 적은 원거리 차량을 놓침) 1.5는 15 m 이상 구조물 OBB가 다시 생긴다.
    float cluster_tolerance_m = 1.0f;
    int cluster_min_points = 10;
    int cluster_max_points = 30000;

    // --- OBB 후단 안전 필터 ---
    bool obb_filter_enable = true;
    float obb_ceiling_min_z_road_m = 2.5f;
    float obb_wall_min_abs_y_m = 4.0f;
    float obb_wall_min_length_m = 15.0f;
    float obb_wall_min_aspect_ratio = 6.0f;
    // |y| 조건 대신, 확정된 벽 평면에 붙어 있고 평행한 긴 OBB도 벽으로 본다
    // (차량이 벽에 4 m보다 가까이 붙어 달리는 구간용). 보호 객체는 제외.
    float obb_wall_attach_distance_m = 0.8f;
    float obb_wall_attach_max_angle_deg = 10.0f;
    // 확정 벽 평면에 붙은(중심 거리 <= 0.8 m) 평행 OBB는 차량일 수 없다
    // (차량 중심은 벽에서 반폭 이상 떨어진다). 그런 OBB는 이 길이부터 벽으로 본다.
    float obb_wall_attached_min_length_m = 8.0f;
    float obb_ground_residual_max_height_m = 0.25f;
    float obb_ground_residual_max_top_road_m = 0.40f;
    bool obb_remove_ground_residual = false;  // 진단만, 기본 비활성

    // --- 보호 대상 (카메라/YOLO) ---
    int protect_obb_min_points = 3;
    float protect_obb_min_fraction = 0.10f;
};

// n·p + d = 0, |n| = 1.
struct Plane {
    float a = 0.0f;
    float b = 0.0f;
    float c = 1.0f;
    float d = 0.0f;

    float SignedDistance(float x, float y, float z) const {
        return a * x + b * y + c * z + d;
    }
    // 수평에 가까운 평면에서 (x, y) 위치의 평면 z.
    float HeightAt(float x, float y) const;
    // 법선과 +z 사이 각(도).
    float TiltDeg() const;
    bool Finite() const;
};

float AngleBetweenNormalsDeg(const Plane& lhs, const Plane& rhs);
// 법선의 xy 성분만 비교한 각도 (수직 벽의 방위 비교용).
float HorizontalAngleBetweenNormalsDeg(const Plane& lhs, const Plane& rhs);

enum class PlaneSource : int {
    kCurrent = 0,
    kPrevious = 1,
    kUnavailable = 2,
};
const char* PlaneSourceName(PlaneSource source);

struct GroundFit {
    bool found = false;
    Plane plane;
    int candidates = 0;          // 높이 띠 후보 (inlier 비율의 분모)
    int refined_candidates = 0;  // prior/최빈값 띠로 좁힌 RANSAC 입력
    int prior_source = 2;        // 0 직전 정상 평면, 1 z 최빈값, 2 없음
    float prior_height = 0.0f;
    int inliers = 0;
    float inlier_ratio = 0.0f;
    float height_at_origin = 0.0f;
    float tilt_deg = 0.0f;
    // 2단계 재적합 (이번 프레임 평면 주변 띠): 채택 여부와 1단계 대비 변화
    int refit_candidates = 0;
    bool refit_used = false;
    float refit_angle_deg = 0.0f;
    float refit_height_delta_m = 0.0f;
    bool valid = false;
    std::string reject_reason;
};

// 직전 정상 평면과의 일관성 검사는 prev가 nullptr가 아닐 때만 한다.
bool ValidateGroundFit(GroundFit* fit, const Params& params, const Plane* prev);

struct GroundDecision {
    PlaneSource source = PlaneSource::kUnavailable;
    Plane plane;
    int consecutive_failures = 0;
};

// 현재 프레임 유효 → 저장. 무효 → 직전 정상 평면을 최대 reuse_max 프레임 재사용.
// 그 이후는 unavailable (호출자가 fail-safe 동작을 명시적으로 수행).
class GroundTemporalFilter {
public:
    explicit GroundTemporalFilter(int reuse_max_frames = 3);
    GroundDecision Update(bool current_valid, const Plane& current);
    // 새 평면의 일관성 검사 기준(재사용 창 안의 마지막 정상 평면).
    const Plane* ConsistencyReference() const;
    bool HasEverBeenValid() const { return has_plane_; }
    const Plane& LastValid() const { return last_; }
    int ConsecutiveFailures() const { return consecutive_failures_; }
    void Reset();

private:
    int reuse_max_frames_;
    bool has_plane_ = false;
    Plane last_;
    int consecutive_failures_ = 0;
};

struct WallSupport {
    bool ok = false;
    float s_start = 0.0f;
    float s_end = 0.0f;
    int occupied_bins = 0;
};

// 벽 방향 좌표 s를 bin으로 나누고, max_gap_bins 이하의 빈 칸만 허용하는
// 연속 지지 구간들을 찾는다 (길이 내림차순). 단순 span 하나가 아니라 여러
// bin의 연속성을 요구하므로 차량 한 면 같은 짧은 평면을 벽으로 보지 않는다.
std::vector<WallSupport> ComputeWallRuns(const std::vector<float>& s_values,
                                         float bin_size_m, int bin_min_points,
                                         int max_gap_bins);
// 가장 긴 연속 지지 구간.
WallSupport ComputeWallSupport(const std::vector<float>& s_values,
                               float bin_size_m, int bin_min_points,
                               int max_gap_bins);

struct WallSegment {
    Plane plane;  // 법선은 센서 원점(차로) 쪽을 향한다 (d > 0).
    float tangent_x = 1.0f;
    float tangent_y = 0.0f;
    float s_min = 0.0f;  // 제거 구간 전체의 최소/최대
    float s_max = 0.0f;
    // 제거 구간: 주 연속 구간 + 같은 평면 위 인접 연장 구간.
    std::vector<std::pair<float, float>> intervals;
    float span = 0.0f;  // 주 연속 구간 길이 (검증 기준)
    int occupied_bins = 0;
    int inliers = 0;
    float inlier_ratio = 0.0f;
    float vertical_span = 0.0f;
    float beyond_fraction = 0.0f;
    float lateral_distance = 0.0f;
    float tilt_deg = 0.0f;
    int pool_support = 0;       // 이 쪽 후보점 중 지지 구간 안 inlier 수 (주 평면 선택용)
    bool step = false;          // 주 평면에 평행한 단차 평면으로 인정됨
    bool continuation = false;  // 곡선 구간 연속 평면으로 인정됨
    bool valid = false;
    std::string reject_reason;

    float AlongWall(float x, float y) const {
        return tangent_x * x + tangent_y * y;
    }
    bool InIntervals(float s, float margin) const {
        for (const auto& interval : intervals) {
            if (s >= interval.first - margin && s <= interval.second + margin) {
                return true;
            }
        }
        return false;
    }
};

// 음수 인자는 params의 기본 기준(wall_min_vertical_span_m, wall_min_span_m,
// wall_min_occupied_bins)을 쓴다.
bool ValidateWallSegment(WallSegment* segment, const Params& params,
                         float min_vertical_span_m = -1.0f,
                         float min_span_m = -1.0f, int min_bins = -1);

struct WallTrackState {
    bool has = false;
    std::vector<Plane> planes;  // 직전 프레임에서 검증된 모든 구간 평면
    int persist = 0;
    int miss = 0;
    bool confirmed = false;
};

// 직전 처리 프레임 대비 센서(수평화 좌표계)의 평면 운동.
// 같은 정지점의 좌표 관계: p_prev = Rz(dyaw) * p_cur + (dx, dy, 0).
struct EgoMotion {
    bool valid = false;
    float dyaw_rad = 0.0f;
    float dx_m = 0.0f;
    float dy_m = 0.0f;
    float dt_s = 0.0f;
};

// 이전 프레임 좌표계의 평면을 현재 프레임 좌표계로 옮긴다.
Plane TransformPlaneToCurrent(const Plane& previous, const EgoMotion& motion);

// 한 쪽 벽의 시간 지속성. 현재 프레임에 유효한 주 평면이 있고, 직전 프레임의
// (주/보조) 평면 중 하나와 비슷한 위치·방향으로 wall_min_persist_frames 이상
// 이어졌을 때만 제거를 허용한다. 2단 벽에서 주 평면이 상부/하부로 번갈아
// 잡혀도 같은 벽으로 이어진다. motion이 유효하면 직전 평면을 자차 이동만큼
// 옮겨서 비교한다.
class WallTrack {
public:
    bool Update(const std::vector<WallSegment>& segments, const Params& params,
                const EgoMotion* motion = nullptr);
    const WallTrackState& State() const { return state_; }
    void Reset() { state_ = WallTrackState(); }

private:
    WallTrackState state_;
};

struct WallSideResult {
    int candidates = 0;
    std::vector<WallSegment> segments;  // 검증을 통과한 구간 (0번이 주 평면)
    WallSegment first_attempt;          // 첫 평면 (실패 사유 진단용)
    // 이 프레임에서 적합한 모든 평면과 판정 ("seq:", "sec:", "cont:" + 사유), 진단용.
    std::vector<WallSegment> attempts;
    bool fit_valid = false;
    bool removal_allowed = false;
    WallTrackState track;
    int removed = 0;
    int protected_skipped = 0;
    int ambiguous_skipped = 0;
};

struct Obb {
    float cx = 0.0f;
    float cy = 0.0f;
    float cz = 0.0f;
    float length = 0.0f;  // 긴 변
    float width = 0.0f;   // 짧은 변
    float z_min = 0.0f;
    float z_max = 0.0f;
    float yaw = 0.0f;
    std::array<pcl::PointXYZ, 8> corners{};
};

// 클러스터 OBB 계산기. ROS 경로와 평가기는 기존 추적기와 같은 L-shape OBB
// (Objectinfo_for_km)를 주입하고, 단위 테스트는 기본 구현을 쓴다.
using ObbFunction = std::function<void(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& clusters,
    std::vector<Obb>* obbs)>;
void DefaultObbFunction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                        const std::vector<pcl::PointIndices>& clusters,
                        std::vector<Obb>* obbs);

struct ObbStructureFlags {
    bool ceiling = false;
    bool wall = false;
    bool wall_attached = false;  // 확정 벽 평면에 붙은 평행한 OBB
    bool ground_residual = false;
    bool removed = false;
};

// z_min_road/z_max_road는 지면 평면 기준 높이. wall_attached는 파이프라인이
// 확정 벽 평면과의 거리/평행도로 계산한다. 보호 객체는 절대 제거하지 않는다.
ObbStructureFlags ClassifyObb(const Obb& obb, float z_min_road,
                              float z_max_road, bool is_protected,
                              bool wall_attached, const Params& params);

// GPS 정상 경로에서 천장/양쪽 벽이 합쳐진 비정상적으로 큰 OBB인지 판정한다.
// 세 축 조건을 모두 만족해야 하므로 긴 버스·트럭 하나만으로는 제거되지 않는다.
bool IsPreGpsStructuralObb(float horizontal_a_m, float horizontal_b_m,
                           float vertical_height_m, const Params& params);

// 벽 구간 끝점(평면 위, 주어진 높이)의 xy.
void WallEndpoint(const WallSegment& segment, float s, float z, float* x,
                  float* y);

struct TunnelObject {
    Obb obb;
    int point_count = 0;
    int protected_points = 0;
    std::uint8_t protect_class = kProtectNone;
    bool is_protected = false;
    float z_min_road = 0.0f;
    float z_max_road = 0.0f;
    ObbStructureFlags flags;
};

struct StageTimings {
    double level_ms = 0.0;
    double ceiling_ms = 0.0;
    double ground_fit_ms = 0.0;
    double ground_remove_ms = 0.0;
    double wall_fit_ms = 0.0;
    double wall_remove_ms = 0.0;
    double cluster_ms = 0.0;
    double obb_ms = 0.0;
    double obb_filter_ms = 0.0;
    double total_ms = 0.0;
};

struct FrameResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr leveled;  // 입력과 같은 순서
    std::vector<std::uint8_t> labels;               // PointLabel
    std::vector<int> wall_candidate_indices;
    std::vector<std::uint8_t> wall_candidate_side;  // 1 left, 2 right
    std::vector<int> kept_indices;                  // 최종 filtered cloud
    std::vector<int> cluster_id;                    // -1: 클러스터 없음
    std::vector<pcl::PointIndices> clusters;
    std::vector<TunnelObject> objects;

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    EgoMotion motion;                 // 벽 추적에 쓴 자차 이동 (진단용)
    std::array<float, 9> leveling{};  // row-major, p_level = R * p_sensor

    int input_points = 0;
    int ceiling_removed = 0;
    GroundFit ground_fit;
    GroundDecision ground;
    int ground_removed = 0;
    bool ground_fallback_used = false;
    float ground_reference_height = 0.0f;
    std::array<WallSideResult, 2> walls;  // 0 left, 1 right
    int wall_removed = 0;
    int protected_points = 0;
    bool protection_available = false;
    int kept_points = 0;
    int objects_removed_ceiling = 0;
    int objects_removed_wall = 0;
    int objects_ground_residual = 0;
    StageTimings timings;
};

// ROI 결과에 수평화 회전을 적용할 행렬. p_level = Ry(pitch) * Rx(roll) * p.
// roll을 끄면 기존 CorrectingPoints(Ry(pitch))와 같다.
std::array<float, 9> LevelingRotation(float roll_rad, float pitch_rad,
                                      bool use_roll, bool use_pitch);
void ApplyRotation(const pcl::PointCloud<pcl::PointXYZI>& input,
                   const std::array<float, 9>& rotation,
                   pcl::PointCloud<pcl::PointXYZI>* output);

// z > max_z 인 점만 천장으로 표시한다 (경계값 max_z는 유지).
int MarkCeiling(const pcl::PointCloud<pcl::PointXYZI>& cloud, float max_z,
                std::vector<std::uint8_t>* labels);

class TunnelPipeline {
public:
    explicit TunnelPipeline(const Params& params = Params(),
                            ObbFunction obb_function = DefaultObbFunction);

    // protect_class/ambiguous는 입력 점과 같은 길이(또는 nullptr).
    // protection_available=false면 ambiguous 점은 벽 제거에서 제외한다
    // (카메라 누락/동기 실패 시 fail-open).
    // motion: 직전 Process 호출 이후 자차 이동 (nullptr/invalid면 보정 없음).
    const FrameResult& Process(const pcl::PointCloud<pcl::PointXYZI>& input,
                               float roll_rad, float pitch_rad,
                               const std::vector<std::uint8_t>* protect_class,
                               const std::vector<std::uint8_t>* ambiguous,
                               bool protection_available,
                               const EgoMotion* motion = nullptr);

    void Reset();
    const Params& GetParams() const { return params_; }
    const FrameResult& LastResult() const { return result_; }

private:
    void FitGround();
    void RemoveGround();
    void ProcessWalls(const std::vector<std::uint8_t>* protect_class,
                      const std::vector<std::uint8_t>* ambiguous,
                      bool protection_available);
    void ClusterAndFilter(const std::vector<std::uint8_t>* protect_class);
    // pool(한쪽 후보)에서 수직(허용 기울기 이내) 평면 하나를 찾는다. 지지 구간과
    // 바깥쪽 검사는 후보 영역(y)과 무관하게 그 평면 위/뒤의 전체 점으로 계산하므로
    // 차량이 터널에 대해 틀어져 벽이 차로 앞을 가로질러도 끊기지 않는다.
    // supported_inliers는 다음 평면 탐색에서 뺄 pool 점이다.
    bool FitWallSegment(const std::vector<int>& pool,
                        const std::vector<std::uint8_t>* protect_class,
                        WallSegment* segment,
                        std::vector<int>* supported_inliers) const;
    float GroundHeightAt(float x, float y) const;

    Params params_;
    ObbFunction obb_function_;
    GroundTemporalFilter ground_filter_;
    std::array<WallTrack, 2> wall_tracks_;
    FrameResult result_;
};

}  // namespace tunnel_filter

#endif  // TUNNEL_FILTER_CORE_HPP
