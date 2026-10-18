// 직선 구간 BBOX 차로 판정. 거리/주행 명령은 출력하지 않는다.
// 기존 YOLO와 차선 모듈을 그대로 사용하고, 영상 하단 접점의 차로만 분류한다.
#include "lane_boundary_adapter/boundary_family_adapter.hpp"
#include <lane_boundary_adapter/CameraBBoxFrame.h>
#include <lane_boundary_adapter/CameraBBoxLaneResult.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>

namespace {
using Result = lane_boundary_adapter::CameraBBoxLaneResult;
using Segment = lane_boundary_adapter::FamilyLaneStripSegment;
using Point = lane_boundary_adapter::FamilyPoint2d;
using Context = lane_boundary_adapter::BoundaryFamilyStripContext;

// ============================================================================
// 1. Parameters: 기본값, 단위, 설명. 모두 private ROS parameter로 설정 가능.
// ============================================================================
struct Parameters {
    std::string reference_lane_topic;      // 선택 사항: 주행 노드의 기준 차로. 비어 있으면 관측 전용 자동 선택.
    double reference_timeout = 0.5;        // [s] 기준 차로 미수신 시 자동 선택으로 복귀.
    std::string calibration_file;          // cameras_v2.yaml. launch에서 실제 파일 경로 지정.
    std::string camera_view = "front";    // 보정 파일 내 카메라 이름.
    std::string expected_hash = "691b998247c960bba06d84513b217709a55f8744530978e32d9028289e217070";
    std::string lane_topic = "/perception/lane/boundary_family"; // 검증된 차선 계약.
    std::string frame_topic = "/planning/camera_bbox_lane/frame"; // 원본 영상+BBOX.
    std::string result_topic = "/planning/camera_bbox_lane/result"; // 차로 판정 결과.
    std::string image_topic = "/planning/camera_bbox_lane/image"; // RViz Image 표시.
    std::string diagnostics_topic = "/planning/camera_bbox_lane/diagnostics";
    double source_timeout = 0.5;           // [s] 원본 영상/차선의 최대 나이.
    double receive_timeout = 0.5;          // [s] 새로운 BBOX를 받지 못하면 UNKNOWN.
    double max_sensor_skew = 0.20;         // [s] BBOX와 차선 원본 시각 차이 상한.
    double future_tolerance = 0.05;        // [s] 미래 timestamp 허용 오차.
    double lane_selection_x = 4.0;         // [m] 내 차로 선택에만 사용. 장애물 거리 추정 아님.
    double selection_margin = 0.15;        // [m] 이 경계 여유 안에서 자차가 걸치면 내 차로 UNKNOWN.
    double boundary_margin_px = 3.0;       // [px] BBOX 하단과 차선 경계의 겹침 여유.
    double contact_band_height_px = 8.0;  // [px] BBOX 하단 위쪽의 접촉 영역 높이. 차체 상부는 판정에서 제외.
    double contact_row_tolerance_px = 8.0; // [px] 하단 아래쪽 접점 오차 허용. 실제 관측된 차로 영역만 사용.
    double min_contact_overlap_ratio = 0.10; // 접촉 영역 폭 중 각 차로와 겹쳐야 하는 최소 비율. 둘 다 넘으면 BOTH.
    int min_contact_support_rows = 3;     // 두 차로가 동시에 관측되는 최소 영상 row 수.
    double bottom_clip_px = 3.0;           // [px] 영상 하단 잘린 BBOX의 접점은 UNKNOWN.
    bool ego_mask_enabled = false;
    std::string ego_mask_config;
    double hood_crop_bottom_ratio = 0.0; // [0,0.5) YOLO 하단 제외 비율. launch에서 detector와 같은 값 전달.
    double clipped_track_max_gap = 0.25;  // [s] 같은 물체를 연속 관측했다고 보는 최대 frame 간격.
    double clipped_track_max_age = 3.0;   // [s] 마지막 정상 접점 이후 잘린 박스를 추적할 최대 시간. 갱신하지 않음.
    double clipped_track_min_iou = 0.30;  // 이전/현재 BBOX의 최소 IoU. 다대일/일대다 매칭은 거부.
    double clipped_track_min_overlap = 0.80; // 보이는 하단 영역이 기존 기하 차로와 겹쳐야 하는 비율.
    double clipped_track_max_other_overlap = 0.02; // 다른 차로 겹침이 이 값을 넘으면 추적 유지 금지.
    int clipped_track_confirm_frames = 2; // 잘리기 전 같은 차로에서 확인해야 하는 서로 다른 frame 수.
    bool defer_far_boxes = true;           // 두 차로 관측 끝보다 먼 영상 영역은 별도 미확정 상태로 전달.
    double far_boundary_margin_px = 0.0;   // [px] 관측 끝 여유. 0이면 관측 범위 진입 전 불필요한 정지 띠를 만들지 않음.
    double far_horizon_margin_px = 5.0;    // [px] 지면 소실선 바로 위/근처 BBOX는 분류 보류로 허용하지 않음.
    double far_min_lane_horizon = 12.0;    // [m] 두 차로 모두 최소 이 전방 범위까지 보여야 함.
    double min_confidence = 0.25;          // 사용할 차선 segment 신뢰도 하한.
    double consumer_x_max = 20.0;          // [m] 차선 관측 범위. 이 밖은 연장하지 않는다.
    int lane_index = -1;                   // -1: 자동, 0: 좌측 띠, 1: 우측 띠.
    int lane_confirm_frames = 3;           // 서로 다른 차선 frame에서 내 차로 연속 확인.
    int history_size = 12;                 // BBOX와 가까운 시각의 차선을 선택할 history 길이.
    int max_boxes = 100;                   // 입력 배열 크기 제한.
    double status_rate = 10.0;             // [Hz] 입력 중단 상태 검사.
} params;

struct Camera {
    int width = 0, height = 0;
    cv::Matx33d homography;                // base_link 지면 xy -> 원본 영상 uv.
    cv::Matx33d rotation;
    cv::Vec3d translation;
    double ground_z = 0.0;
    double slope_x = 0.0, slope_y = 0.0;
} camera;
std::vector<cv::Point2d> ego_polygon;
cv::Mat ego_bitmap;

struct LaneFrame {
    ros::Time stamp;
    Context context;
    int own_lane = -1;
    std::string error;
};
struct LaneQuad {
    int lane;
    std::vector<cv::Point2d> points;
    double x_max = 0.0;
};

struct BoxTrack {
    lane_boundary_adapter::CameraBBox box;
    int lane = -1;                         // OWN/ADJ가 아닌 기하 차로 0/1. 기준 차로 변경에도 유지.
    int confirmations = 0;
    ros::Time stamp, last_ground_stamp;    // 추적 frame 시각 / 마지막 정상 접점 관측 시각.
};
std::vector<BoxTrack> box_tracks;

lane_boundary_adapter::BoundaryFamilyAdapterConfig adapter;
std::deque<LaneFrame> history;
ros::Time last_lane_stamp, last_bbox_stamp;
ros::SteadyTime last_bbox_receive;
int pending_lane = -1, confirm_frames = 0;
int reference_lane = -1;
ros::SteadyTime reference_received;
ros::Publisher result_pub, image_pub, diagnostic_pub;
std::string last_error = "waiting_for_bbox";
cv::Mat last_image;
std_msgs::Header last_image_header;

void loadParameters(ros::NodeHandle& nh);
void loadCalibration();
void laneCallback(const asmc_msgs::LaneBoundaryFamily::ConstPtr& message);
void frameCallback(const lane_boundary_adapter::CameraBBoxFrame::ConstPtr& message);
void statusCallback(const ros::TimerEvent&);
void referenceCallback(const std_msgs::Int8::ConstPtr& message);
std::string stampError(const ros::Time& stamp);
int selectOwnLane(const Context& context);
bool projectPoint(const Point& point, cv::Point2d& pixel);
std::vector<LaneQuad> projectLanes(const Context& context);
std::vector<std::pair<double, double>> laneIntervals(const std::vector<LaneQuad>& quads, int lane, double row);
bool maskOccludesContact(const lane_boundary_adapter::CameraBBox& box) {
    if (!params.ego_mask_enabled) return false;
    if (ego_bitmap.rows != camera.height || ego_bitmap.cols != camera.width) {
        ego_bitmap = cv::Mat::zeros(camera.height, camera.width, CV_8U);
        std::vector<cv::Point> pixels;
        for (const auto& p : ego_polygon)
            pixels.emplace_back(std::min(camera.width-1, static_cast<int>(p.x*camera.width)),
                                std::min(camera.height-1, static_cast<int>(p.y*camera.height)));
        cv::fillPoly(ego_bitmap, std::vector<std::vector<cv::Point>>{pixels}, cv::Scalar(1));
    }
    const int x0=std::max(0, static_cast<int>(box.xmin));
    const int x1=std::min(camera.width, static_cast<int>(std::ceil(box.xmax)));
    const int y0=std::max(0, static_cast<int>(box.ymax-params.contact_band_height_px));
    const int y1=std::min(camera.height, static_cast<int>(std::ceil(box.ymax+params.contact_row_tolerance_px)));
    return x1>x0 && y1>y0 && cv::countNonZero(ego_bitmap(cv::Rect(x0,y0,x1-x0,y1-y0)))>0;
}

uint8_t classifyBox(const lane_boundary_adapter::CameraBBox& box, const std::vector<LaneQuad>& quads,
                    int own_lane, std::string& reason);
void publishResult(Result result);
void publishInvalid(const ros::Time& stamp, const std::string& reason);
void publishImage(cv::Mat image, const std_msgs::Header& header, const Result& result,
                  const std::vector<LaneQuad>& quads);
}  // namespace

// ============================================================================
// 2. MAIN: 관측/표시 전용. /ctrl_cmd publisher 없음.
// ============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_bbox_lane");
    ros::NodeHandle nh, private_nh("~");
    try {
        loadParameters(private_nh);
        loadCalibration();
        adapter.consumer_x_max_m = params.consumer_x_max;
        result_pub = nh.advertise<Result>(params.result_topic, 1);
        image_pub = nh.advertise<sensor_msgs::Image>(params.image_topic, 1);
        diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(params.diagnostics_topic, 1);
        const auto lane_sub = nh.subscribe(params.lane_topic, 1, laneCallback);
        const auto frame_sub = nh.subscribe(params.frame_topic, 1, frameCallback);
        ros::Subscriber reference_sub;
        if (!params.reference_lane_topic.empty()) reference_sub = nh.subscribe(params.reference_lane_topic, 1, referenceCallback);
        const auto timer = nh.createTimer(ros::Duration(1.0 / params.status_rate), statusCallback);
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("camera_bbox_lane: %s", error.what());
        return 1;
    }
    return 0;
}

namespace {
// ============================================================================
// 3. 차선 입력: 기존 adapter 검증 + 자차 차로 확인 + 시각별 보관.
// ============================================================================
std::string stampError(const ros::Time& stamp) {
    const double age = (ros::Time::now() - stamp).toSec();
    if (stamp.isZero()) return "zero_source_stamp";
    if (age > params.source_timeout || age < -params.future_tolerance) return "source_timestamp_expired";
    return "";
}

void referenceCallback(const std_msgs::Int8::ConstPtr& message) {
    reference_lane = (message->data == 0 || message->data == 1) ? message->data : -1;
    reference_received = ros::SteadyTime::now();
}

int selectOwnLane(const Context& context) {
    // 변경 중 기준 차로를 고정해 중앙선 통과를 자동 판정 실패로 오인하지 않는다.
    // 결과 own_lane에 기준을 함께 기록하므로 consumer는 기하 차로 0/1로 다시 변환 가능.
    if (!params.reference_lane_topic.empty() && reference_lane >= 0 &&
        (ros::SteadyTime::now()-reference_received).toSec() <= params.reference_timeout) return reference_lane;
    if (params.lane_index >= 0) return params.lane_index;
    int selected = -1;
    for (const auto& segment : context.segments) {
        if (segment.effective_confidence < params.min_confidence) continue;
        for (std::size_t i = 1; i < segment.centerline.size(); ++i) {
            const auto& first = segment.centerline[i - 1];
            const auto& second = segment.centerline[i];
            if (first.x > params.lane_selection_x || second.x < params.lane_selection_x || second.x <= first.x) continue;
            const double t = (params.lane_selection_x - first.x) / (second.x - first.x);
            const double left = segment.left_boundary[i-1].y * (1-t) + segment.left_boundary[i].y * t;
            const double right = segment.right_boundary[i-1].y * (1-t) + segment.right_boundary[i].y * t;
            if (right + params.selection_margin < 0 && left - params.selection_margin > 0) {
                if (selected >= 0 && selected != static_cast<int>(segment.lateral_index)) return -1;
                selected = static_cast<int>(segment.lateral_index);
            }
        }
    }
    return selected;
}

void laneCallback(const asmc_msgs::LaneBoundaryFamily::ConstPtr& message) {
    LaneFrame frame;
    frame.stamp = message->header.stamp;
    frame.error = stampError(frame.stamp);
    if (!frame.error.empty() || (!last_lane_stamp.isZero() && frame.stamp < last_lane_stamp)) {
        history.clear();
        pending_lane = -1; confirm_frames = 0;
        last_error = "invalid_lane_timestamp";
        return;
    }
    if (frame.stamp == last_lane_stamp) return;  // 중복 frame은 확인 횟수를 올리지 않음.
    last_lane_stamp = frame.stamp;
    // Planning handoff: keep this fail-closed gate until both the classifier and
    // follower accept the same validated geometry. Updating the hash alone does
    // not make the static projection below compatible with IMU family xyz.
    // See docs/handoffs/camera-to-planning.md, section 7.1.
    if (message->config_sha256 != params.expected_hash) {
        frame.error = "wrong_geometry_config_hash";
    } else {
        frame.context = lane_boundary_adapter::BuildBoundaryFamilyStripContext(*message, adapter);
        if (!frame.context.usable()) frame.error = frame.context.reason;
    }
    int candidate = frame.error.empty() ? selectOwnLane(frame.context) : -1;
    if (candidate < 0) {
        pending_lane = -1; confirm_frames = 0;
        if (frame.error.empty()) frame.error = "own_lane_unobserved_or_straddling";
    } else {
        confirm_frames = pending_lane == candidate ? confirm_frames + 1 : 1;
        pending_lane = candidate;
        const bool explicit_reference = !params.reference_lane_topic.empty() && reference_lane >= 0 &&
            (ros::SteadyTime::now()-reference_received).toSec() <= params.reference_timeout;
        if (explicit_reference || confirm_frames >= params.lane_confirm_frames) frame.own_lane = candidate;
        else frame.error = "confirming_own_lane";
    }
    history.push_back(frame);
    while (history.size() > static_cast<std::size_t>(params.history_size)) history.pop_front();
}

// ============================================================================
// 4. 지면 차선 경계만 원본 영상에 투영. BBOX의 거리/3D 크기는 만들지 않는다.
// ============================================================================
bool projectPoint(const Point& point, cv::Point2d& pixel) {
    // Compatibility projection: FamilyPoint2d has discarded the producer's z.
    // This reconstructs z from the fixed YAML plane, not the image-time IMU plane.
    // Planning must validate a z-preserving, image/calibration/time-matched path
    // before accepting the new family SHA; latest per-view metadata is not an
    // interchangeable plane for another image (no common-time ego-motion alignment).
    const cv::Vec3d body(point.x, point.y, camera.ground_z + camera.slope_x*point.x + camera.slope_y*point.y);
    const cv::Vec3d optical = camera.rotation * body + camera.translation;
    if (optical[2] <= 0.01) return false;
    const cv::Vec3d projected = camera.homography * cv::Vec3d(point.x, point.y, 1.0);
    pixel = cv::Point2d(projected[0]/projected[2], projected[1]/projected[2]);
    return std::isfinite(pixel.x) && std::isfinite(pixel.y);
}

std::vector<LaneQuad> projectLanes(const Context& context) {
    std::vector<LaneQuad> quads;
    for (const auto& segment : context.segments) {
        if (segment.effective_confidence < params.min_confidence) continue;
        for (std::size_t i = 1; i < segment.centerline.size(); ++i) {
            LaneQuad quad{static_cast<int>(segment.lateral_index), {}};
            quad.x_max = segment.centerline[i].x;
            bool valid = true;
            for (const auto& point : {segment.left_boundary[i-1], segment.left_boundary[i],
                                      segment.right_boundary[i], segment.right_boundary[i-1]}) {
                cv::Point2d pixel;
                if (!projectPoint(point, pixel)) { valid = false; break; }
                quad.points.push_back(pixel);
            }
            if (valid) quads.push_back(quad);
        }
    }
    return quads;
}

std::vector<std::pair<double, double>> laneIntervals(const std::vector<LaneQuad>& quads, int lane, double row) {
    std::vector<std::pair<double, double>> intervals;
    for (const auto& quad : quads) {
        if (quad.lane != lane) continue;
        std::vector<double> intersections;
        for (std::size_t i = 0; i < 4; ++i) {
            const auto& a = quad.points[i];
            const auto& b = quad.points[(i+1)%4];
            if (row < std::min(a.y,b.y) - 1e-6 || row > std::max(a.y,b.y) + 1e-6) continue;
            if (std::abs(a.y-b.y) < 1e-6) {
                intersections.push_back(a.x); intersections.push_back(b.x);
            } else {
                intersections.push_back(a.x + (row-a.y)/(b.y-a.y) * (b.x-a.x));
            }
        }
        if (intersections.size() < 2) continue;
        const auto bounds = std::minmax_element(intersections.begin(), intersections.end());
        const double left = std::max(0.0, *bounds.first);
        const double right = std::min(static_cast<double>(camera.width-1), *bounds.second);
        if (right > left) intervals.emplace_back(left, right);
    }
    return intervals;
}

bool isBeyondObservedLanes(const lane_boundary_adapter::CameraBBox& box,
                           const std::vector<LaneQuad>& quads) {
    if (!params.defer_far_boxes) return false;
    double top_row[2] = {static_cast<double>(camera.height), static_cast<double>(camera.height)};
    double far_x[2] = {0.0, 0.0};
    for (const auto& quad : quads) {
        if (quad.lane < 0 || quad.lane > 1) continue;
        far_x[quad.lane] = std::max(far_x[quad.lane], quad.x_max);
        for (const auto& point : quad.points) top_row[quad.lane] = std::min(top_row[quad.lane], point.y);
    }
    for (int lane = 0; lane < 2; ++lane) {
        if (far_x[lane] < params.far_min_lane_horizon ||
            box.ymax >= top_row[lane] - params.far_boundary_margin_px) return false;
    }
    // 지면 homography의 두 소실점으로 영상 소실선을 계산한다. 장애물 거리는 산출하지 않는다.
    const cv::Vec3d forward(camera.homography(0,0),camera.homography(1,0),camera.homography(2,0));
    const cv::Vec3d lateral(camera.homography(0,1),camera.homography(1,1),camera.homography(2,1));
    const auto horizon = forward.cross(lateral);
    if (std::abs(horizon[1]) < 1e-9) return false;
    for (double u : {static_cast<double>(box.xmin),static_cast<double>(box.xmax)}) {
        const double row = -(horizon[0]*u + horizon[2])/horizon[1];
        if (!std::isfinite(row) || box.ymax <= row + params.far_horizon_margin_px) return false;
    }
    return true;
}

// ============================================================================
// BBOX 하단 접촉 영역과 관측된 차로 폴리곤의 겹침. 거리/미관측 경계는 만들지 않는다.
// ============================================================================
struct ContactEvidence {
    bool valid = false;
    double top = 0.0, bottom = 0.0;
    double own_ratio = 0.0, adjacent_ratio = 0.0;
    int supported_rows = 0;
};

double overlapWidth(std::vector<std::pair<double,double>> intervals, double xmin, double xmax) {
    // 인접 segment의 겹친 영역을 두 번 세지 않도록 union 길이로 계산한다.
    std::sort(intervals.begin(), intervals.end());
    double total = 0.0, covered_until = xmin;
    for (const auto& interval : intervals) {
        const double left = std::max({xmin, interval.first - params.boundary_margin_px, covered_until});
        const double right = std::min(xmax, interval.second + params.boundary_margin_px);
        if (right > left) { total += right - left; covered_until = right; }
    }
    return total;
}

ContactEvidence measureContact(const lane_boundary_adapter::CameraBBox& box,
                               const std::vector<LaneQuad>& quads, int own_lane) {
    ContactEvidence evidence;
    if (own_lane < 0 || own_lane > 1 || box.xmax <= box.xmin || box.ymax <= box.ymin) return evidence;
    evidence.top = std::max<double>(box.ymin, box.ymax - params.contact_band_height_px);
    const int detector_height = static_cast<int>(camera.height * (1.0 - params.hood_crop_bottom_ratio));
    evidence.bottom = std::min(box.ymax + params.contact_row_tolerance_px,
                               detector_height - params.bottom_clip_px);
    // 잘린 BBOX의 접점은 시각화에서도 유효한 접촉 증거로 표시하지 않는다.
    if (box.ymax >= detector_height - params.bottom_clip_px) return evidence;

    // 하단 row의 support가 없으면 '관측 맨 끝'에서만 작은 픽셀 오차를 허용한다.
    // 더 먼 유효 조각이 있는 중간 INVALID gap은 이 조건을 통과할 수 없다.
    for (int lane = 0; lane < 2; ++lane) {
        if (!laneIntervals(quads, lane, box.ymax).empty()) continue;
        double far_top = camera.height, far_x = 0.0;
        for (const auto& quad : quads) {
            if (quad.lane != lane) continue;
            far_x = std::max(far_x, quad.x_max);
            for (const auto& point : quad.points) far_top = std::min(far_top, point.y);
        }
        if (far_x < params.far_min_lane_horizon || box.ymax > far_top + 1e-6 ||
            far_top > evidence.bottom) return evidence;
    }

    double own_width = 0.0, adjacent_width = 0.0;
    for (double row = evidence.top; row <= evidence.bottom + 1e-6; row += 1.0) {
        const auto own = laneIntervals(quads, own_lane, row);
        const auto adjacent = laneIntervals(quads, 1-own_lane, row);
        if (own.empty() || adjacent.empty()) continue;
        own_width += overlapWidth(own, box.xmin, box.xmax);
        adjacent_width += overlapWidth(adjacent, box.xmin, box.xmax);
        ++evidence.supported_rows;
    }
    if (evidence.supported_rows < params.min_contact_support_rows) return evidence;
    const double area = (box.xmax - box.xmin) * evidence.supported_rows;
    evidence.own_ratio = own_width / area;
    evidence.adjacent_ratio = adjacent_width / area;
    evidence.valid = true;
    return evidence;
}

uint8_t classifyBox(const lane_boundary_adapter::CameraBBox& box, const std::vector<LaneQuad>& quads,
                    int own_lane, std::string& reason) {
    for (double v : {box.xmin, box.ymin, box.xmax, box.ymax}) {
        if (!std::isfinite(v)) { reason = "non_finite_bbox"; return Result::UNKNOWN; }
    }
    if (box.xmin < 0 || box.ymin < 0 || box.xmax > camera.width || box.ymax > camera.height ||
        box.xmin >= box.xmax || box.ymin >= box.ymax) {
        reason = "invalid_bbox_coordinates"; return Result::UNKNOWN;
    }
    if (maskOccludesContact(box)) {
        reason = "bottom_contact_occluded_by_ego_mask"; return Result::UNKNOWN;
    }
    const int detector_height = static_cast<int>(camera.height * (1.0 - params.hood_crop_bottom_ratio));
    if (params.hood_crop_bottom_ratio > 0 && box.ymax >= detector_height - params.bottom_clip_px) {
        reason = "bottom_contact_clipped_by_hood_crop"; return Result::UNKNOWN;
    }
    if (box.ymax >= camera.height - params.bottom_clip_px) {
        reason = "bottom_contact_clipped"; return Result::UNKNOWN;
    }
    const auto contact = measureContact(box, quads, own_lane);
    if (!contact.valid) {
        if (isBeyondObservedLanes(box, quads)) {
            reason = "beyond_observed_lane_horizon";
            return Result::DEFERRED_FAR;
        }
        reason = "lane_support_missing_at_bbox_contact";
        return Result::UNKNOWN;
    }
    const bool on_own = contact.own_ratio >= params.min_contact_overlap_ratio;
    const bool on_adjacent = contact.adjacent_ratio >= params.min_contact_overlap_ratio;
    reason = "contact_overlap:own=" + std::to_string(static_cast<int>(100 * contact.own_ratio)) +
             "%;adjacent=" + std::to_string(static_cast<int>(100 * contact.adjacent_ratio)) +
             "%;rows=" + std::to_string(contact.supported_rows);
    if (on_own && on_adjacent) return Result::BOTH;
    if (on_own) return Result::OWN;
    if (on_adjacent) return Result::ADJACENT;
    if (contact.own_ratio > 0 || contact.adjacent_ratio > 0) {
        reason = "contact_overlap_too_small:" + reason;
        return Result::UNKNOWN;
    }
    return Result::OUTSIDE;
}

double boxIoU(const lane_boundary_adapter::CameraBBox& a, const lane_boundary_adapter::CameraBBox& b) {
    const double intersection = std::max(0.0, static_cast<double>(std::min(a.xmax,b.xmax)-std::max(a.xmin,b.xmin))) *
                                std::max(0.0, static_cast<double>(std::min(a.ymax,b.ymax)-std::max(a.ymin,b.ymin)));
    const double area = (a.xmax-a.xmin)*(a.ymax-a.ymin) + (b.xmax-b.xmin)*(b.ymax-b.ymin) - intersection;
    return area > 0 ? intersection / area : 0.0;
}

std::vector<int> matchBoxTracks(const std::vector<lane_boundary_adapter::CameraBBox>& boxes, const ros::Time& stamp) {
    std::vector<int> matches(boxes.size(), -1), uses(box_tracks.size(), 0);
    for (std::size_t i=0; i<boxes.size(); ++i) {
        int count = 0;
        for (std::size_t j=0; j<box_tracks.size(); ++j) {
            const auto& old = box_tracks[j];
            const double dt = (stamp-old.stamp).toSec();
            if (dt <= 0 || dt > params.clipped_track_max_gap || boxes[i].label != old.box.label ||
                boxIoU(boxes[i],old.box) < params.clipped_track_min_iou) continue;
            matches[i] = static_cast<int>(j); ++count; ++uses[j];
        }
        if (count != 1) matches[i] = -1;
    }
    for (int& match : matches) if (match >= 0 && uses[match] != 1) match = -1;
    return matches;
}

bool continueClippedTrack(const lane_boundary_adapter::CameraBBox& box, const BoxTrack& track,
                          const std::vector<LaneQuad>& quads, int own_lane, const ros::Time& stamp) {
    const int cutoff = static_cast<int>(camera.height*(1.0-params.hood_crop_bottom_ratio));
    if (box.ymax > cutoff + 1.0 || track.confirmations < params.clipped_track_confirm_frames ||
        (stamp-track.last_ground_stamp).toSec() > params.clipped_track_max_age) return false;
    // 현재도 같은 차로에 보이는지를 검사한다. 가려진 접점을 복원했다는 뜻은 아니다.
    auto visible = box;
    visible.ymax = static_cast<int>(camera.height*(1.0-params.hood_crop_bottom_ratio)) - params.bottom_clip_px - 1.0;
    visible.ymin = std::max(visible.ymin, static_cast<float>(visible.ymax-params.contact_band_height_px));
    const auto contact = measureContact(visible,quads,own_lane);
    if (!contact.valid) return false;
    const double same = track.lane == own_lane ? contact.own_ratio : contact.adjacent_ratio;
    const double other = track.lane == own_lane ? contact.adjacent_ratio : contact.own_ratio;
    return same >= params.clipped_track_min_overlap && other <= params.clipped_track_max_other_overlap;
}

// ============================================================================
// 5. BBOX 처리 / 결과. NO_DETECTION은 검출 없음이며 free-space 보장이 아니다.
// ============================================================================
void frameCallback(const lane_boundary_adapter::CameraBBoxFrame::ConstPtr& message) {
    const auto stamp = message->header.stamp;
    std::string error = stampError(stamp);
    if (error.empty() && !last_bbox_stamp.isZero() && stamp < last_bbox_stamp) error = "bbox_timestamp_regression";
    if (!error.empty()) { publishInvalid(stamp, error); return; }
    if (stamp == last_bbox_stamp) return;
    last_bbox_stamp = stamp;
    last_bbox_receive = ros::SteadyTime::now();
    if (!message->valid) { publishInvalid(stamp, message->reason); return; }
    if (message->image.header.stamp != stamp || message->image.header.frame_id != message->header.frame_id) {
        publishInvalid(stamp, "image_bbox_header_mismatch"); return;
    }
    cv::Mat image;
    try { image = cv::imdecode(message->image.data, cv::IMREAD_COLOR); }
    catch (const cv::Exception&) { publishInvalid(stamp, "image_decode_failed"); return; }
    if (image.empty() || image.cols != camera.width || image.rows != camera.height) {
        publishInvalid(stamp, "image_size_calibration_mismatch"); return;
    }
    last_image = image;
    last_image_header = message->header;
    if (message->boxes.size() > static_cast<std::size_t>(params.max_boxes)) {
        publishInvalid(stamp, "too_many_boxes"); return;
    }
    if (history.empty()) { publishInvalid(stamp, "waiting_for_lane"); return; }
    // 최신 차선이 invalid인 경우 이전 valid frame으로 실패를 가리지 않는다.
    if (!history.back().error.empty() || !stampError(history.back().stamp).empty()) {
        publishInvalid(stamp, history.back().error.empty() ? "lane_timestamp_expired" : history.back().error); return;
    }
    const auto nearest = std::min_element(history.begin(), history.end(), [stamp](const auto& a, const auto& b) {
        return std::abs((a.stamp-stamp).toSec()) < std::abs((b.stamp-stamp).toSec());
    });
    if (!nearest->error.empty() || !stampError(nearest->stamp).empty()) {
        publishInvalid(stamp, "matched_lane_unavailable"); return;
    }
    if (std::abs((nearest->stamp-stamp).toSec()) > params.max_sensor_skew) {
        publishInvalid(stamp, "camera_bbox_lane_timestamp_skew"); return;
    }
    Result result;
    result.header = message->header;
    result.lane_stamp = nearest->stamp;
    result.valid = true;
    result.reason = "classified";
    result.own_lane = nearest->own_lane;
    result.own_state = Result::NO_DETECTION;
    result.adjacent_state = Result::NO_DETECTION;
    result.boxes = message->boxes;
    const auto quads = projectLanes(nearest->context);
    bool own_hit = false, adjacent_hit = false, unknown = false, deferred_far = false;
    const auto matches = matchBoxTracks(result.boxes, stamp);
    std::vector<BoxTrack> next_tracks;
    for (std::size_t i=0; i<result.boxes.size(); ++i) {
        const auto& box = result.boxes[i];
        std::string reason;
        auto assignment = classifyBox(box, quads, result.own_lane, reason);
        bool associated = false;
        const BoxTrack* old = matches[i] >= 0 ? &box_tracks[matches[i]] : nullptr;
        if (assignment == Result::UNKNOWN && reason == "bottom_contact_clipped_by_hood_crop" && old &&
            continueClippedTrack(box,*old,quads,result.own_lane,stamp)) {
            assignment = old->lane == result.own_lane ? Result::OWN : Result::ADJACENT;
            associated = true;
            reason = "tracked_clipped_contact:geometric_lane=" + std::to_string(old->lane);
        }
        result.clipped_association.push_back(associated);
        if (assignment == Result::OWN || assignment == Result::ADJACENT) {
            BoxTrack track;
            track.box = box; track.stamp = stamp;
            track.lane = assignment == Result::OWN ? result.own_lane : 1-result.own_lane;
            track.confirmations = old && old->lane == track.lane ? std::min(params.clipped_track_confirm_frames, old->confirmations + (associated ? 0 : 1)) : 1;
            track.last_ground_stamp = associated ? old->last_ground_stamp : stamp;
            next_tracks.push_back(track);
        }
        result.assignments.push_back(assignment);
        result.box_reasons.push_back(reason);
        own_hit |= assignment == Result::OWN || assignment == Result::BOTH;
        adjacent_hit |= assignment == Result::ADJACENT || assignment == Result::BOTH;
        unknown |= assignment == Result::UNKNOWN;
        deferred_far |= assignment == Result::DEFERRED_FAR;
    }
    box_tracks = std::move(next_tracks);
    if (deferred_far) {
        result.reason = "far_boxes_deferred";
        result.own_state = result.adjacent_state = Result::UNOBSERVED;
    }
    if (unknown) {
        result.valid = false;
        result.reason = "some_boxes_unclassified";
        result.own_state = result.adjacent_state = Result::UNOBSERVED;
    }
    if (own_hit) result.own_state = Result::DETECTED;
    if (adjacent_hit) result.adjacent_state = Result::DETECTED;
    publishResult(result);
    publishImage(image.clone(), message->header, result, quads);
}

void publishResult(Result result) {
    last_error = result.valid ? "" : result.reason;
    result_pub.publish(result);
    diagnostic_msgs::DiagnosticArray diagnostic;
    diagnostic.header = result.header;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "camera_bbox_lane";
    status.level = result.valid ? 0 : 1;
    status.message = result.reason;
    const auto add = [&status](const std::string& key, const std::string& value) {
        diagnostic_msgs::KeyValue entry; entry.key=key; entry.value=value; status.values.push_back(entry);
    };
    add("own_lane", std::to_string(result.own_lane));
    add("own_state", std::to_string(result.own_state));
    add("adjacent_state", std::to_string(result.adjacent_state));
    add("publishes_control", "false");
    add("estimates_obstacle_distance", "false");
    diagnostic.status.push_back(status);
    diagnostic_pub.publish(diagnostic);
}

void publishInvalid(const ros::Time& stamp, const std::string& reason) {
    box_tracks.clear();
    Result result;
    result.header.stamp = stamp; // 새 시각을 붙여 오래된 관측을 신선하게 만들지 않음.
    result.valid = false;
    result.reason = reason;
    result.own_lane = -1;
    publishResult(result);
    if (!last_image.empty()) publishImage(last_image.clone(), last_image_header, result, {});
}

void statusCallback(const ros::TimerEvent&) {
    if (last_bbox_stamp.isZero()) publishInvalid(last_bbox_stamp, "waiting_for_bbox");
    else if ((ros::SteadyTime::now()-last_bbox_receive).toSec() > params.receive_timeout || !stampError(last_bbox_stamp).empty()) {
        publishInvalid(last_bbox_stamp, "bbox_timeout");
    } else if (history.empty() || !history.back().error.empty() || !stampError(last_lane_stamp).empty()) {
        publishInvalid(last_bbox_stamp, "lane_unavailable");
    }
}

void publishImage(cv::Mat image, const std_msgs::Header& header, const Result& result,
                  const std::vector<LaneQuad>& quads) {
    if (image_pub.getNumSubscribers() == 0) return;
    cv::Mat areas = image.clone();
    for (const auto& quad : quads) {
        std::vector<cv::Point> polygon;
        for (const auto& p : quad.points) polygon.emplace_back(
            static_cast<int>(std::clamp(p.x, -10000.0, 10000.0)),
            static_cast<int>(std::clamp(p.y, -10000.0, 10000.0)));
        const cv::Scalar lane_color = quad.lane == result.own_lane ? cv::Scalar(0,220,0) : cv::Scalar(220,0,220);
        cv::fillConvexPoly(areas, polygon, lane_color);
        cv::polylines(image, polygon, true, lane_color, 1);
    }
    cv::addWeighted(areas, 0.18, image, 0.82, 0.0, image);
    if (params.hood_crop_bottom_ratio > 0) {
        const int cutoff = static_cast<int>(image.rows * (1.0 - params.hood_crop_bottom_ratio));
        cv::rectangle(image, cv::Rect(0, cutoff, image.cols, image.rows - cutoff), cv::Scalar(35,35,35), cv::FILLED);
        cv::line(image, cv::Point(0,cutoff), cv::Point(image.cols-1,cutoff), cv::Scalar(0,180,255), 2);
        cv::putText(image, "YOLO EXCLUDED: EGO HOOD", cv::Point(20,cutoff+30),
                    cv::FONT_HERSHEY_SIMPLEX, .7, cv::Scalar(0,180,255), 2);
    }

    const char* names[] = {"UNKNOWN", "MY_LANE", "ADJACENT", "BOTH", "OUTSIDE", "FAR_UNRESOLVED"};
    for (std::size_t i = 0; i < result.boxes.size(); ++i) {
        const auto& box = result.boxes[i];
        if (!std::isfinite(box.xmin) || !std::isfinite(box.xmax) || !std::isfinite(box.ymin) || !std::isfinite(box.ymax)) continue;
        const auto label = result.assignments[i];
        const cv::Scalar color = (label == Result::OWN || label == Result::BOTH) ? cv::Scalar(0,0,255) :
                                  label == Result::ADJACENT ? cv::Scalar(255,150,0) : cv::Scalar(0,220,255);
        const cv::Point tl(static_cast<int>(std::clamp<double>(box.xmin,0,camera.width-1)), static_cast<int>(std::clamp<double>(box.ymin,0,camera.height-1)));
        const cv::Point br(static_cast<int>(std::clamp<double>(box.xmax,0,camera.width-1)), static_cast<int>(std::clamp<double>(box.ymax,0,camera.height-1)));
        cv::rectangle(image, tl, br, color, 2);
        const auto contact = measureContact(box, quads, result.own_lane);
        if (box.xmax > box.xmin && box.ymax > box.ymin && contact.bottom >= contact.top) {
            const int top = static_cast<int>(std::clamp(contact.top, 0.0, camera.height-1.0));
            const int bottom = static_cast<int>(std::clamp(contact.bottom, 0.0, camera.height-1.0));
            cv::rectangle(image, cv::Point(tl.x,top), cv::Point(br.x,bottom), cv::Scalar(0,220,255), 1);
            if (contact.valid) {
                // 실제 겹친 구간을 해당 차로 색으로 그려 접촉 판정 근거를 보여준다.
                for (int row = top; row <= bottom; ++row) {
                    if (laneIntervals(quads,0,row).empty() || laneIntervals(quads,1,row).empty()) continue;
                    for (int lane = 0; lane < 2; ++lane) {
                        for (const auto& span : laneIntervals(quads,lane,row)) {
                            const int left = static_cast<int>(std::max<double>(box.xmin,span.first));
                            const int right = static_cast<int>(std::min<double>(box.xmax,span.second));
                            if (left <= right) cv::line(image, cv::Point(left,row), cv::Point(right,row),
                                lane == result.own_lane ? cv::Scalar(0,255,0) : cv::Scalar(255,0,255), 1);
                        }
                    }
                }
                const std::string text = "OWN " + std::to_string(static_cast<int>(100*contact.own_ratio)) +
                    "% / ADJ " + std::to_string(static_cast<int>(100*contact.adjacent_ratio)) + "%";
                cv::putText(image, text, cv::Point(tl.x,std::min(camera.height-10,bottom+20)),
                            cv::FONT_HERSHEY_SIMPLEX,.5,cv::Scalar(0,220,255),1);
            }
        }
        cv::putText(image, box.label + " " + names[label], cv::Point(tl.x,std::max(18,tl.y-4)), cv::FONT_HERSHEY_SIMPLEX,.55,color,2);
        if (i < result.clipped_association.size() && result.clipped_association[i]) {
            cv::putText(image, "TRACKED CLIPPED (LIMITED)", cv::Point(tl.x,std::max(36,tl.y+18)),
                        cv::FONT_HERSHEY_SIMPLEX,.45,cv::Scalar(0,220,255),1);
        }
        if (label == Result::UNKNOWN && i < result.box_reasons.size()) {
            const bool clipped = result.box_reasons[i].find("clipped") != std::string::npos;
            cv::putText(image, clipped ? "CONTACT CLIPPED" : "CONTACT UNRESOLVED",
                        cv::Point(tl.x,std::max(36,tl.y+18)), cv::FONT_HERSHEY_SIMPLEX,.45,color,1);
        }
    }
    cv::putText(image, result.reason + " (OBSERVATION ONLY)", cv::Point(20,30), cv::FONT_HERSHEY_SIMPLEX,.65,cv::Scalar(0,220,255),2);
    sensor_msgs::Image output;
    output.header=header; output.height=image.rows; output.width=image.cols;
    output.encoding="bgr8"; output.step=image.cols*3;
    output.data.assign(image.data,image.data+image.total()*image.elemSize());
    image_pub.publish(output);
}

// ============================================================================
// 6. 설정 / 기존 보정 파일 로드. 깊이 모델과 라이다를 사용하지 않는다.
// ============================================================================
void loadParameters(ros::NodeHandle& nh) {
#define PARAM(name) nh.param(#name, params.name, params.name)
    PARAM(reference_lane_topic); PARAM(reference_timeout);
    PARAM(hood_crop_bottom_ratio); PARAM(ego_mask_enabled); PARAM(ego_mask_config);
    if (params.ego_mask_enabled) {
        if (params.hood_crop_bottom_ratio != 0) throw std::runtime_error("ego mask and legacy crop are mutually exclusive");
        const auto cfg = YAML::LoadFile(params.ego_mask_config);
        const auto points = cfg["polygon"];
        if (!points.IsSequence() || points.size()<3) throw std::runtime_error("invalid ego mask polygon");
        for (const auto& point : points) {
            if (!point.IsSequence() || point.size()!=2) throw std::runtime_error("invalid ego mask point");
            const double x=point[0].as<double>(), y=point[1].as<double>();
            if (!std::isfinite(x) || !std::isfinite(y) || x<0 || x>1 || y<0 || y>1)
                throw std::runtime_error("ego mask points must be normalized finite coordinates");
            ego_polygon.emplace_back(x,y);
        }
        if (std::abs(cv::contourArea(std::vector<cv::Point2f>(ego_polygon.begin(),ego_polygon.end())))<=0) throw std::runtime_error("degenerate ego mask");
    }
    PARAM(clipped_track_max_gap); PARAM(clipped_track_max_age); PARAM(clipped_track_min_iou);
    PARAM(clipped_track_min_overlap); PARAM(clipped_track_max_other_overlap); PARAM(clipped_track_confirm_frames);
    PARAM(contact_band_height_px); PARAM(contact_row_tolerance_px);
    PARAM(min_contact_overlap_ratio); PARAM(min_contact_support_rows);
    PARAM(defer_far_boxes); PARAM(far_boundary_margin_px); PARAM(far_horizon_margin_px); PARAM(far_min_lane_horizon);
    PARAM(calibration_file); PARAM(camera_view); PARAM(expected_hash); PARAM(lane_topic);
    PARAM(frame_topic); PARAM(result_topic); PARAM(image_topic); PARAM(diagnostics_topic);
    PARAM(source_timeout); PARAM(receive_timeout); PARAM(max_sensor_skew); PARAM(future_tolerance);
    PARAM(lane_selection_x); PARAM(selection_margin); PARAM(boundary_margin_px); PARAM(bottom_clip_px);
    PARAM(min_confidence); PARAM(consumer_x_max); PARAM(lane_index); PARAM(lane_confirm_frames);
    PARAM(history_size); PARAM(max_boxes); PARAM(status_rate);
#undef PARAM
    for (double v : {params.reference_timeout,params.source_timeout,params.receive_timeout,params.max_sensor_skew,params.future_tolerance,
         params.lane_selection_x,params.selection_margin,params.boundary_margin_px,params.bottom_clip_px,
         params.min_confidence,params.consumer_x_max,params.status_rate}) {
        if (!std::isfinite(v)) throw std::runtime_error("parameters must be finite");
    }
    if (!std::isfinite(params.clipped_track_max_gap) || params.clipped_track_max_gap <= 0 || params.clipped_track_max_gap > .5 ||
        !std::isfinite(params.clipped_track_max_age) || params.clipped_track_max_age <= 0 || params.clipped_track_max_age > 3 ||
        !std::isfinite(params.clipped_track_min_iou) || params.clipped_track_min_iou < .3 || params.clipped_track_min_iou > 1 ||
        !std::isfinite(params.clipped_track_min_overlap) || params.clipped_track_min_overlap < .8 || params.clipped_track_min_overlap > 1 ||
        !std::isfinite(params.clipped_track_max_other_overlap) || params.clipped_track_max_other_overlap < 0 || params.clipped_track_max_other_overlap > .02 ||
        params.clipped_track_confirm_frames < 2) throw std::runtime_error("invalid clipped tracking parameters");
    if (!std::isfinite(params.contact_band_height_px) || params.contact_band_height_px < 0 || params.contact_band_height_px > 32 ||
        !std::isfinite(params.contact_row_tolerance_px) || params.contact_row_tolerance_px < 0 || params.contact_row_tolerance_px > 16 ||
        !std::isfinite(params.min_contact_overlap_ratio) || params.min_contact_overlap_ratio <= 0 || params.min_contact_overlap_ratio > .5 ||
        params.min_contact_support_rows < 1 || params.min_contact_support_rows > 49)
        throw std::runtime_error("invalid contact overlap parameters");
    if (!std::isfinite(params.hood_crop_bottom_ratio) || params.hood_crop_bottom_ratio < 0 ||
        params.hood_crop_bottom_ratio >= 0.5) throw std::runtime_error("hood_crop_bottom_ratio must be in [0, 0.5)");
    if (!std::isfinite(params.far_boundary_margin_px) || params.far_boundary_margin_px < 0 ||
        !std::isfinite(params.far_horizon_margin_px) || params.far_horizon_margin_px < 0 ||
        !std::isfinite(params.far_min_lane_horizon) || params.far_min_lane_horizon < params.lane_selection_x ||
        params.far_min_lane_horizon > params.consumer_x_max) throw std::runtime_error("invalid far-box parameters");
    if (params.reference_timeout <= 0 || params.calibration_file.empty() || params.lane_index < -1 || params.lane_index > 1 ||
        params.source_timeout <= 0 || params.receive_timeout <= 0 || params.max_sensor_skew <= 0 ||
        params.future_tolerance < 0 || params.lane_selection_x <= 0 || params.consumer_x_max <= params.lane_selection_x ||
        params.selection_margin < 0 || params.boundary_margin_px < 0 || params.bottom_clip_px < 0 ||
        params.min_confidence < 0 || params.min_confidence > 1 || params.lane_confirm_frames < 1 ||
        params.history_size < 1 || params.max_boxes < 1 || params.status_rate <= 0) throw std::runtime_error("invalid parameters");
}

void loadCalibration() {
    const auto yaml = YAML::LoadFile(params.calibration_file);
    const auto view = yaml["cameras"][params.camera_view];
    camera.width=view["width"].as<int>(); camera.height=view["height"].as<int>();
    const double fov=view["horizontal_fov_deg"].as<double>();
    if (camera.width <= 0 || camera.height <= 0 || !std::isfinite(fov) || fov <= 0 || fov >= 180) throw std::runtime_error("invalid camera intrinsics");
    const double pi=std::acos(-1.0), focal=0.5*camera.width/std::tan(fov*pi/360.0);
    const cv::Matx33d K(focal,0,camera.width*.5, 0,focal,camera.height*.5, 0,0,1);
    const auto r=view["rotation_deg"].as<std::vector<double>>();
    const auto t=view["translation_m"].as<std::vector<double>>();
    if (r.size()!=3 || t.size()!=3) throw std::runtime_error("invalid camera extrinsics");
    for (double v : {r[0],r[1],r[2],t[0],t[1],t[2]}) if (!std::isfinite(v)) throw std::runtime_error("nonfinite calibration");
    const double cr=std::cos(r[0]*pi/180),sr=std::sin(r[0]*pi/180);
    const double cp=std::cos(r[1]*pi/180),sp=std::sin(r[1]*pi/180);
    const double cy=std::cos(r[2]*pi/180),sy=std::sin(r[2]*pi/180);
    const cv::Matx33d Rx(1,0,0,0,cr,-sr,0,sr,cr), Ry(cp,0,sp,0,1,0,-sp,0,cp), Rz(cy,-sy,0,sy,cy,0,0,0,1);
    const cv::Matx33d optical_from_mount(0,-1,0,0,0,-1,1,0,0);
    camera.rotation=optical_from_mount*(Rz*Ry*Rx).t();
    camera.translation=-(camera.rotation*cv::Vec3d(t[0],t[1],t[2]));
    const auto ground=yaml["ground_plane"];
    if (ground["frame_id"].as<std::string>()!="base_link" || ground["model"].as<std::string>()!="plane") throw std::runtime_error("unsupported ground plane");
    camera.ground_z=ground["z_at_origin_m"].as<double>();
    camera.slope_x=ground["dz_dx"].as<double>(); camera.slope_y=ground["dz_dy"].as<double>();
    for (double v : {camera.ground_z,camera.slope_x,camera.slope_y}) if (!std::isfinite(v)) throw std::runtime_error("nonfinite ground plane");
    cv::Matx33d ground_rotation;
    for (int i=0;i<3;++i) {
        ground_rotation(i,0)=camera.rotation(i,0)+camera.slope_x*camera.rotation(i,2);
        ground_rotation(i,1)=camera.rotation(i,1)+camera.slope_y*camera.rotation(i,2);
        ground_rotation(i,2)=camera.translation[i]+camera.ground_z*camera.rotation(i,2);
    }
    camera.homography=K*ground_rotation;
}
}  // namespace
