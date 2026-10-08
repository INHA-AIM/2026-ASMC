#include "SensorFusion.hpp" // (or your relevant main header)
#include "FusionClassPolicy.hpp"
#include "FusionTimeMatcher.hpp"
#include <asmc_camera/InferenceStatus.h>
#include "FusionTrace.hpp"
#include "FusionClassState.hpp"
#include "FusionStateTrace.hpp"
#include <Geometry/FrameBoundary.hpp>
#include "Global/Global.hpp"       // LiDARObjectInfo가 정의된 헤더
#include <iterator>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>

namespace {

bool IsGroundNoise(const LiDARObjectInfo& object)
{
    return object.str_ObjectClass == "noise" ||
           object.str_ObjectClass == "지면노이즈";
}

struct TimedLidarObjects {
    ros::Time stamp;
    std::vector<LiDARObjectInfo> objects;
    ros::WallTime queued_at;
    asmc::lidar::Context context;
};

struct TimedCameraImage {
    ros::Time stamp;
    std::vector<uint8_t> compressed;
};

struct TimedYoloFrame {
    ros::Time stamp;
    uint64 frame_seq = 0;
    std::vector<BBox> boxes;
    std::string camera_id = "front";
    std::string calibration_id;
    uint64 epoch = 0;
    bool consumed = false;
};

std::mutex g_FusionInputMutex;
std::deque<TimedCameraImage> g_CameraInputQueue;
std::deque<TimedYoloFrame> g_YoloInputQueue;
std::uint64_t g_TotalMatched = 0, g_TotalDropped = 0;
std::uint64_t g_TotalFresh = 0, g_TotalReused = 0;
uint64 g_CameraEpoch = 1, g_ControlGeneration = 0, g_SourceCutoff = 0;
bool g_CameraActive = true, g_HasControl = false;
ros::WallTime g_LastControl, g_LastYolo;
bool g_YoloSilent = false;
uint64 g_ControlBatches = 0;
uint64 g_ProcessingCameraEpoch = 1;

void CameraControl(const asmc_camera::InferenceStatus::ConstPtr& status) {
    std::lock_guard<std::mutex> lock(g_FusionInputMutex);
    const bool active = status->mode == asmc_camera::InferenceStatus::ACTIVE &&
        status->reason != "source_image_stale";
    if (!g_HasControl || status->generation != g_ControlGeneration || active != g_CameraActive ||
        status->successful_batches < g_ControlBatches) {
        ++g_CameraEpoch;
        g_YoloInputQueue.clear();
    }
    g_HasControl = true;
    g_ControlGeneration = status->generation;
    g_ControlBatches = status->successful_batches;
    g_SourceCutoff = status->source_cutoff.toNSec();
    g_CameraActive = active;
    g_LastControl = ros::WallTime::now();
}
std::size_t g_PendingLidar = 0, g_PendingYolo = 0;

void TraceObjectStates(const char* event, const ros::Time& lidar_stamp,
                       const ros::Time& yolo_stamp,
                       const std::vector<LiDARObjectInfo>& objects)
{
    fusion_state_trace::Snapshot(event, lidar_stamp, yolo_stamp, objects);
}

template <typename Frame>
void InsertByStamp(std::deque<Frame>& frames, Frame frame)
{
    const auto position = std::upper_bound(
        frames.begin(), frames.end(), frame.stamp,
        [](const ros::Time& stamp, const Frame& candidate) {
            return stamp < candidate.stamp;
        });
    frames.insert(position, std::move(frame));
}

void PruneInputsBefore(std::deque<TimedYoloFrame>& yolo_frames,
                       std::deque<TimedCameraImage>& camera_frames,
                       const ros::Time& processed_lidar_stamp,
                       double max_time_offset_sec)
{
    const ros::Time oldest_useful_yolo =
        processed_lidar_stamp - ros::Duration(max_time_offset_sec);
    while (!yolo_frames.empty() &&
           yolo_frames.front().stamp < oldest_useful_yolo) {
        fusion_trace::Event("yolo_expired", yolo_frames.front().stamp.toNSec(),
                            [](std::ostream&) {});
        yolo_frames.pop_front();
    }

    if (!camera_frames.empty()) {
        const ros::Time oldest_camera =
            camera_frames.back().stamp - ros::Duration(2.0);
        while (!camera_frames.empty() &&
               camera_frames.front().stamp < oldest_camera) {
            camera_frames.pop_front();
        }
    }
}

struct FusionSyncStats {
    uint64 matched = 0;
    uint64 dropped = 0;
    std::vector<double> offsets_ms;
    ros::WallTime interval_start = ros::WallTime::now();

    std::vector<double> OffsetSummary()
    {
        double median_ms = 0.0;
        double p95_ms = 0.0;
        double max_ms = 0.0;
        if (!offsets_ms.empty()) {
            std::sort(offsets_ms.begin(), offsets_ms.end());
            median_ms = offsets_ms[offsets_ms.size() / 2];
            const std::size_t p95_index = std::min(
                offsets_ms.size() - 1,
                static_cast<std::size_t>(
                    std::ceil(0.95 * offsets_ms.size())) - 1);
            p95_ms = offsets_ms[p95_index];
            max_ms = offsets_ms.back();
        }
        return {median_ms, p95_ms, max_ms};
    }

    void ReportIfDue(std::size_t pending_lidar,
                     std::size_t pending_yolo)
    {
        const ros::WallTime now = ros::WallTime::now();
        if ((now - interval_start).toSec() < 5.0) return;
        const auto offsets = OffsetSummary();
        ROS_INFO(
            "Fusion sync 5s: matched=%llu dropped=%llu offset median/p95/max=%.1f/%.1f/%.1f ms pending lidar/yolo=%zu/%zu",
            static_cast<unsigned long long>(matched),
            static_cast<unsigned long long>(dropped),
            offsets[0], offsets[1], offsets[2], pending_lidar, pending_yolo);
        matched = 0;
        dropped = 0;
        offsets_ms.clear();
        interval_start = now;
    }

    void ReportFinal(std::size_t pending_lidar, std::size_t pending_yolo)
    {
        const auto offsets = OffsetSummary();
        std::ostringstream line;
        line << std::fixed << std::setprecision(1)
             << "Fusion sync tail: matched=" << matched << " dropped=" << dropped
             << " offset median/p95/max=" << offsets[0] << '/' << offsets[1] << '/' << offsets[2]
             << " ms pending lidar/yolo=" << pending_lidar << '/' << pending_yolo
             << " duration=" << (ros::WallTime::now() - interval_start).toSec() << " s";
        std::cout << line.str() << std::endl;
    }
};

FusionSyncStats& SyncStats()
{
    static FusionSyncStats stats;
    return stats;
}

}  // namespace

void BufferFusionCameraFrame(const ros::Time& stamp,
                             const std::vector<uint8_t>& compressed)
{
    if (stamp.isZero() || compressed.empty()) return;
    std::lock_guard<std::mutex> lock(g_FusionInputMutex);
    InsertByStamp(g_CameraInputQueue, {stamp, compressed});
    fusion_trace::Event("camera_input", stamp.toNSec(), [&](std::ostream& out) {
        out << ",\"bytes\":" << compressed.size();
    });
    constexpr std::size_t kMaxBufferedCameraFrames = 80;
    while (g_CameraInputQueue.size() > kMaxBufferedCameraFrames) {
        fusion_trace::Event("camera_input_overflow", g_CameraInputQueue.front().stamp.toNSec(),
                            [](std::ostream&) {});
        g_CameraInputQueue.pop_front();
    }
}

void BufferFusionYoloFrame(uint64 frame_seq, const ros::Time& image_stamp,
                           const std::vector<BBox>& boxes)
{
    if (image_stamp.isZero()) return;
    std::lock_guard<std::mutex> lock(g_FusionInputMutex);
    if (!g_CameraActive || image_stamp.toNSec() <= g_SourceCutoff) return;
    g_LastYolo = ros::WallTime::now();
    g_YoloSilent = false;
    TimedYoloFrame frame{image_stamp, frame_seq, boxes};
    frame.epoch = g_CameraEpoch;
    static const std::string calibration_id = [] {
        std::string id;
        ros::param::param<std::string>("~camera_calibration_id", id, "legacy_vehicle_spec");
        return id;
    }();
    frame.calibration_id = calibration_id;
    InsertByStamp(g_YoloInputQueue, std::move(frame));
    fusion_trace::Event("yolo_input", image_stamp.toNSec(), [&](std::ostream& out) {
        out << ",\"sequence\":" << frame_seq << ",\"boxes\":[";
        for (std::size_t index = 0; index < boxes.size(); ++index) {
            if (index) out << ',';
            const auto& box = boxes[index];
            out << '[' << fusion_trace::Quote(box.str_class) << ','
                << box.st_Min_pt.f32_X << ',' << box.st_Min_pt.f32_Y << ','
                << box.st_Max_pt.f32_X << ',' << box.st_Max_pt.f32_Y << ']';
        }
        out << ']';
    });
    constexpr std::size_t kMaxBufferedYoloFrames = 128;
    while (g_YoloInputQueue.size() > kMaxBufferedYoloFrames) {
        fusion_trace::Event("yolo_input_overflow", g_YoloInputQueue.front().stamp.toNSec(),
                            [](std::ostream&) {});
        g_YoloInputQueue.pop_front();
    }
}

void ReportFusionTotals()
{
    // Workers have joined after ros::waitForShutdown(), so rosconsole may
    // already be shut down. Flush the final accounting directly to stdout
    // so trace-disabled acceptance runs retain the last partial interval.
    SyncStats().ReportFinal(g_PendingLidar, g_PendingYolo);
    std::cout << "Fusion sync total: matched=" << g_TotalMatched
              << " dropped=" << g_TotalDropped
              << " fresh=" << g_TotalFresh << " reused=" << g_TotalReused
              << " pending lidar/yolo=" << g_PendingLidar << '/' << g_PendingYolo
              << std::endl;
    fusion_trace::Event("fusion_summary", 0, [](std::ostream& out) {
        out << ",\"matched\":" << g_TotalMatched << ",\"dropped\":" << g_TotalDropped
            << ",\"fresh\":" << g_TotalFresh << ",\"reused\":" << g_TotalReused;
        out << ",\"pending_lidar\":" << g_PendingLidar << ",\"pending_yolo\":" << g_PendingYolo;
    });
}

/**
 * @brief y-밴드(절대 픽셀 좌표)와 겹치는지 검사합니다.
 */
static inline bool overlapsYBand(const cv::Rect& r, int ymin_px, int ymax_px, int H)
{
    if (ymin_px < 0 || ymax_px <= ymin_px) return false;
    ymin_px = std::max(0, std::min(ymin_px, H));
    ymax_px = std::max(0, std::min(ymax_px, H));
    int top = r.y;
    int bot = r.y + r.height;
    int overlap = std::max(0, std::min(bot, ymax_px) - std::max(top, ymin_px));
    return overlap > 0;
}

/**
 * @brief LiDAR 포인트 클라우드를 카메라 이미지 평면에 투영하여 2D 바운딩 박스를 생성합니다.
 * @param st_LiDAR LiDAR 데이터 구조체
 * @param st_Camera 카메라 데이터 구조체
 */
static void LidarToImage(LiDAR &st_LiDAR, CAMERA &st_Camera,
                         const CameraParameters& calibration)
{
    st_Camera.fusion_data.vec_Lidar_BBoxes.clear();

    Eigen::Matrix<double,3,4> projection=calibration.sensor_fusion_projection_matrix;
    if(frame_boundary::Enabled()) {
        asmc::lidar::Pose camera_mount;
        camera_mount.matrix()=calibration.bird_eye_view_transform.inverse();
        const auto p=asmc::lidar::camera_projection(st_LiDAR.frame_context,camera_mount,
            calibration.intrinsic_matrix_90_fov.leftCols<3>());
        if(!p) {ROS_ERROR_THROTTLE(1.,"Invalid scan projection context"); return;}
        projection=*p;
    }

    // Optional legacy hood y-band. Disabled by default because even a small
    // overlap can reject a legitimate nearby object.
    int ex_ymin = 637, ex_ymax = 720;
    bool exclude_hood_y_band = false;
    ros::param::get("~exclude_ymin_px", ex_ymin);
    ros::param::get("~exclude_ymax_px", ex_ymax);
    ros::param::param("~fusion_exclude_hood_y_band", exclude_hood_y_band,
                      exclude_hood_y_band);

    for (size_t i = 0; i < st_LiDAR.vec_Objects.size(); ++i) {
        auto& det = st_LiDAR.vec_Objects[i];
        // Do not associate a YOLO detection with a prediction-only ghost.
        // Its stored point cloud is from an older LiDAR frame and can steal
        // the class from the currently observed cluster.
        if (det.s32_MissCount > 0) continue;
        if (IsGroundNoise(det)) continue;
        std::vector<cv::Point> projected_points;

        for (const auto& pt : det.pcl_ClusterPoints.points) {
            Eigen::Vector4d pt_vec(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector3d cam = projection * pt_vec;

            if (cam[2] <= 0.0) continue; // 카메라 뒤쪽 포인트 제거

            int u = static_cast<int>(cam[0] / cam[2]);
            int v = static_cast<int>(cam[1] / cam[2]);

            if (u >= 0 && u < st_Camera.bgr_image.cols &&
                v >= 0 && v < st_Camera.bgr_image.rows) {
                projected_points.push_back(cv::Point(u, v));
            }
        }

        if (!projected_points.empty()) {
            cv::Rect bbox = cv::boundingRect(projected_points);

            // 보닛 y-밴드와 겹치면 제외
            if (exclude_hood_y_band &&
                overlapsYBand(bbox, ex_ymin, ex_ymax,
                              st_Camera.bgr_image.rows)) {
                continue;
            }

            BBox st_BBox;
            st_BBox.st_Min_pt.f32_X = static_cast<float32>(bbox.tl().x);
            st_BBox.st_Min_pt.f32_Y = static_cast<float32>(bbox.tl().y);
            st_BBox.st_Max_pt.f32_X = static_cast<float32>(bbox.br().x);
            st_BBox.st_Max_pt.f32_Y = static_cast<float32>(bbox.br().y);
            st_BBox.c_idx = i;
            st_Camera.fusion_data.vec_Lidar_BBoxes.push_back(st_BBox);
        }
    }
}

/**
 * @brief YOLO와 LiDAR BBox 간의 IoU를 계산하여 클래스를 매칭하고,
 * 연속 관측으로 클래스를 확정하되 이후 반대 관측으로 변경할 수 있습니다.
 * @param st_LiDAR LiDAR 데이터 구조체
 * @param st_Camera 카메라 데이터 구조체
 */
static void IOUFusion(LiDAR& st_LiDAR, CAMERA &st_Camera,
                      bool is_new_yolo_frame)
{
    st_LiDAR.vec_FusionIdx.clear();
    const float32 fusion_x_offset = 0;
    
    // Count real, time-synchronized YOLO results, not the faster Main loop.
    // Classification is deliberately reversible: initial acquisition needs
    // more evidence than changing an already established class.
    static int INITIAL_CONFIRMATIONS = 5;
    static int CHANGE_CONFIRMATIONS = 3;
    static int CLASS_MISS_GRACE = 3;
    static double FUSION_IOU_THRESHOLD = 0.25;
    static bool params_loaded = false;
    if (!params_loaded) {
        ros::param::param("~fusion_initial_confirmations",
                         INITIAL_CONFIRMATIONS, INITIAL_CONFIRMATIONS);
        ros::param::param("~fusion_change_confirmations",
                         CHANGE_CONFIRMATIONS, CHANGE_CONFIRMATIONS);
        ros::param::param("~fusion_class_miss_grace", CLASS_MISS_GRACE,
                         CLASS_MISS_GRACE);
        ros::param::param("~fusion_iou_threshold", FUSION_IOU_THRESHOLD,
                         FUSION_IOU_THRESHOLD);
        params_loaded = true;
        INITIAL_CONFIRMATIONS = std::max(1, INITIAL_CONFIRMATIONS);
        CHANGE_CONFIRMATIONS = std::max(1, CHANGE_CONFIRMATIONS);
        CLASS_MISS_GRACE = std::max(0, CLASS_MISS_GRACE);
    }

    struct MatchCandidate {
        double iou;
        size_t camera_idx;
        size_t lidar_bbox_idx;
    };
    std::vector<MatchCandidate> candidates;

    // Build all valid pairs first, then greedily select highest-IoU one-to-one
    // matches. This prevents one LiDAR cluster from increasing its continuity
    // counter multiple times from duplicate/overlapping YOLO boxes.
    for (size_t i = 0; i < st_Camera.yolo_results.detected_objects.size(); ++i) {
        const auto& cam = st_Camera.yolo_results.detected_objects[i];
        for (size_t j = 0; j < st_Camera.fusion_data.vec_Lidar_BBoxes.size(); ++j) {
            const auto& lid_bbox = st_Camera.fusion_data.vec_Lidar_BBoxes[j];

            float32 center_x = (lid_bbox.st_Min_pt.f32_X + lid_bbox.st_Max_pt.f32_X) / 2.0f;
            float32 center_y = (lid_bbox.st_Min_pt.f32_Y + lid_bbox.st_Max_pt.f32_Y) / 2.0f;
            float32 width = (lid_bbox.st_Max_pt.f32_X - lid_bbox.st_Min_pt.f32_X) * 1.3f;
            float32 height = (lid_bbox.st_Max_pt.f32_Y - lid_bbox.st_Min_pt.f32_Y) * 1.5f;

            BBox scaled_lid_box;
            scaled_lid_box.st_Min_pt.f32_X = center_x - width / 2.0f + fusion_x_offset;
            scaled_lid_box.st_Min_pt.f32_Y = center_y - height / 2.0f;
            scaled_lid_box.st_Max_pt.f32_X = center_x + width / 2.0f + fusion_x_offset;
            scaled_lid_box.st_Max_pt.f32_Y = center_y + height / 2.0f;

            const float32 x_left = std::max(cam.st_Min_pt.f32_X, scaled_lid_box.st_Min_pt.f32_X);
            const float32 y_top = std::max(cam.st_Min_pt.f32_Y, scaled_lid_box.st_Min_pt.f32_Y);
            const float32 x_right = std::min(cam.st_Max_pt.f32_X, scaled_lid_box.st_Max_pt.f32_X);
            const float32 y_bottom = std::min(cam.st_Max_pt.f32_Y, scaled_lid_box.st_Max_pt.f32_Y);
            const float32 inter_area = std::max(0.0f, x_right - x_left) *
                                       std::max(0.0f, y_bottom - y_top);
            const float32 cam_area = std::max(0.0f, cam.st_Max_pt.f32_X - cam.st_Min_pt.f32_X) *
                                     std::max(0.0f, cam.st_Max_pt.f32_Y - cam.st_Min_pt.f32_Y);
            const float32 lid_area = std::max(0.0f, scaled_lid_box.st_Max_pt.f32_X - scaled_lid_box.st_Min_pt.f32_X) *
                                     std::max(0.0f, scaled_lid_box.st_Max_pt.f32_Y - scaled_lid_box.st_Min_pt.f32_Y);
            const float32 union_area = cam_area + lid_area - inter_area;
            const double iou = union_area > 1e-6f ? inter_area / union_area : 0.0;
            if (iou > FUSION_IOU_THRESHOLD) {
                candidates.push_back({iou, i, j});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const MatchCandidate& a, const MatchCandidate& b) {
                  return a.iou > b.iou;
              });

    std::unordered_set<size_t> matched_lidar_indices_this_frame;
    std::unordered_set<size_t> matched_camera_indices;
    std::vector<std::vector<int32>> matches_by_camera(
        st_Camera.yolo_results.detected_objects.size());

    for (const auto& candidate : candidates) {
        const auto& lid_bbox =
            st_Camera.fusion_data.vec_Lidar_BBoxes[candidate.lidar_bbox_idx];
        const size_t lidar_idx = lid_bbox.c_idx;
        if (lidar_idx >= st_LiDAR.vec_Objects.size() ||
            matched_camera_indices.count(candidate.camera_idx) != 0 ||
            matched_lidar_indices_this_frame.count(lidar_idx) != 0) {
            continue;
        }

        matched_camera_indices.insert(candidate.camera_idx);
        matched_lidar_indices_this_frame.insert(lidar_idx);
        matches_by_camera[candidate.camera_idx].push_back(
            static_cast<int32>(lidar_idx));

        auto& lid_object = st_LiDAR.vec_Objects[lidar_idx];
        const auto& cam =
            st_Camera.yolo_results.detected_objects[candidate.camera_idx];

        if (is_new_yolo_frame && fusion_class::BeginObservation(lid_object,
                st_Camera.front_image_stamp.toNSec(), g_ProcessingCameraEpoch)) {
            fusion_trace::Event("class_observation", st_Camera.front_image_stamp.toNSec(),
                [&](std::ostream& out) {
                    out << ",\"id\":" << lid_object.s32_Id
                        << ",\"label\":" << fusion_trace::Quote(cam.str_class)
                        << ",\"raw_label\":" << fusion_trace::Quote(cam.str_raw_class)
                        << ",\"iou\":" << candidate.iou;
                });
            fusion_class::Observe(lid_object, cam.str_class,
                                  cam.str_raw_class,
                                  INITIAL_CONFIRMATIONS,
                                  CHANGE_CONFIRMATIONS);
        }
    }

    for (auto& matches : matches_by_camera) {
        if (!matches.empty()) st_LiDAR.vec_FusionIdx.push_back(matches);
    }
    
    // 5. YOLO에 매칭되지 않은 객체들의 연속성 카운터 감소 처리
    if (is_new_yolo_frame) for (size_t i = 0; i < st_LiDAR.vec_Objects.size(); ++i) {
        auto& object = st_LiDAR.vec_Objects[i];
        // Prediction-only tracks preserve their class until LiDAR tracking
        // itself decides whether the track survives.
        if (object.s32_MissCount > 0 || IsGroundNoise(object)) continue;
        if (matched_lidar_indices_this_frame.find(i) == matched_lidar_indices_this_frame.end() &&
            fusion_class::BeginObservation(object, st_Camera.front_image_stamp.toNSec(), g_ProcessingCameraEpoch))
        {
            fusion_trace::Event("class_miss", st_Camera.front_image_stamp.toNSec(),
                [&](std::ostream& out) { out << ",\"id\":" << object.s32_Id; });
            fusion_class::Miss(object, CLASS_MISS_GRACE);
        }
    }
}

// /**
//  * @brief 카메라 이미지에 YOLO와 투영된 LiDAR 바운딩 박스를 그립니다.
//  * @param st_LiDAR LiDAR 데이터 구조체 (객체 상태 확인을 위해 필요)
//  * @param st_Camera 카메라 데이터 구조체
//  */
// static void DrawBBox(LiDAR& st_LiDAR, CAMERA &st_Camera)
// {
//     if (st_Camera.bgr_image.empty()) return;
//     cv::Mat tmp_Img = st_Camera.bgr_image.clone();
//     const int visual_x_offset = 0;

//     // YOLO BBox (초록색)
//     for (const auto& box : st_Camera.yolo_results.detected_objects) {
//         cv::Point tl(static_cast<int>(box.st_Min_pt.f32_X), static_cast<int>(box.st_Min_pt.f32_Y));
//         cv::Point br(static_cast<int>(box.st_Max_pt.f32_X), static_cast<int>(box.st_Max_pt.f32_Y));
//         cv::rectangle(tmp_Img, tl, br, {0, 255, 0}, 2);
//     }

//     // LiDAR BBox (기본: 빨간색, 고정 시: 파란색)
//     for (const auto& box : st_Camera.fusion_data.vec_Lidar_BBoxes) {
//         const auto& lid_object = st_LiDAR.vec_Objects[box.c_idx];
//         cv::Scalar bbox_color = {0, 0, 255}; // 기본 색상 (빨간색)

//         if (lid_object.is_class_locked) {
//             bbox_color = {255, 0, 0}; // 고정된 객체 색상 (파란색)
//         }

//         float32 center_x = (box.st_Min_pt.f32_X + box.st_Max_pt.f32_X) / 2.0f;
//         float32 center_y = (box.st_Min_pt.f32_Y + box.st_Max_pt.f32_Y) / 2.0f;
//         float32 width = (box.st_Max_pt.f32_X - box.st_Min_pt.f32_X) * 1.3f;
//         float32 height = (box.st_Max_pt.f32_Y - box.st_Min_pt.f32_Y) * 1.5f;

//         cv::Point tl(static_cast<int>(center_x - width / 2.0f + visual_x_offset), static_cast<int>(center_y - height / 2.0f));
//         cv::Point br(static_cast<int>(center_x + width / 2.0f + visual_x_offset), static_cast<int>(center_y + height / 2.0f));

//         cv::rectangle(tmp_Img, tl, br, bbox_color, 2);
        
//         // 디버깅용 텍스트 (클래스, 카운터, 고정 상태) 표시
//         std::string label = lid_object.str_ObjectClass;
//         if (lid_object.is_class_locked) {
//             label += " (Locked)";
//         } else if(lid_object.str_ObjectClass != "unknown") {
//             label += " (" + std::to_string(lid_object.continuity_counter) + ")";
//         }
//         cv::putText(tmp_Img, label, tl, cv::FONT_HERSHEY_SIMPLEX, 0.5, bbox_color, 1, cv::LINE_AA);
//     }

//     // y-밴드 디버그용 표시
//     int ex_ymin = 637, ex_ymax = 720;
//     ros::param::get("~exclude_ymin_px", ex_ymin);
//     ros::param::get("~exclude_ymax_px", ex_ymax);
//     if (ex_ymax > ex_ymin) {
//         cv::rectangle(tmp_Img,
//                       cv::Rect(0, ex_ymin, tmp_Img.cols, ex_ymax - ex_ymin),
//                       cv::Scalar(40, 40, 40), 2);
//     }

//     if (st_Camera.s32_Display) {
//         cv::imshow("Fusion BBox", tmp_Img);
//         cv::waitKey(1);
//     }
// }


static ros::Publisher& FusionDebugImagePub()
{
    static ros::NodeHandle nh;
    static ros::Publisher pub =
        nh.advertise<sensor_msgs::CompressedImage>("/fusion/debug_image/compressed", 1);
    return pub;
}

static ros::Publisher& FusionDebugRawImagePub()
{
    static ros::NodeHandle nh;
    static ros::Publisher pub =
        nh.advertise<sensor_msgs::Image>("/fusion/debug_image", 1);
    return pub;
}

/**
 * @brief Publish an annotated image for checking camera/LiDAR/YOLO alignment.
 * Green boxes are YOLO detections, red boxes are projected LiDAR clusters,
 * and cyan boxes are LiDAR clusters matched to YOLO by IoU.
 */
static void PublishFusionDebugImage(LiDAR& st_LiDAR, CAMERA& st_Camera)
{
    if (st_Camera.bgr_image.empty()) return;
    if(!FusionDebugImagePub().getNumSubscribers() &&
        !FusionDebugRawImagePub().getNumSubscribers()) return;

    // The processing loop can run faster than the sensors. Limit JPEG encoding
    // so this diagnostic view does not steal CPU from YOLO.
    static ros::WallTime last_publish;
    const ros::WallTime now = ros::WallTime::now();
    if (!last_publish.isZero() && (now - last_publish).toSec() < (1.0 / 15.0)) {
        return;
    }
    last_publish = now;

    cv::Mat image = st_Camera.bgr_image.clone();

    for (const auto& box : st_Camera.yolo_results.detected_objects) {
        const cv::Point tl(static_cast<int>(box.st_Min_pt.f32_X),
                           static_cast<int>(box.st_Min_pt.f32_Y));
        const cv::Point br(static_cast<int>(box.st_Max_pt.f32_X),
                           static_cast<int>(box.st_Max_pt.f32_Y));
        cv::rectangle(image, tl, br, cv::Scalar(0, 255, 0), 3);
        cv::putText(image, "YOLO: " + box.str_class,
                    cv::Point(tl.x, std::max(24, tl.y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2,
                    cv::LINE_AA);
    }

    std::unordered_set<int32> matched_indices;
    for (const auto& group : st_LiDAR.vec_FusionIdx) {
        matched_indices.insert(group.begin(), group.end());
    }

    for (const auto& box : st_Camera.fusion_data.vec_Lidar_BBoxes) {
        const float32 center_x =
            (box.st_Min_pt.f32_X + box.st_Max_pt.f32_X) * 0.5f;
        const float32 center_y =
            (box.st_Min_pt.f32_Y + box.st_Max_pt.f32_Y) * 0.5f;
        const float32 width =
            (box.st_Max_pt.f32_X - box.st_Min_pt.f32_X) * 1.3f;
        const float32 height =
            (box.st_Max_pt.f32_Y - box.st_Min_pt.f32_Y) * 1.5f;

        const cv::Point tl(static_cast<int>(center_x - width * 0.5f),
                           static_cast<int>(center_y - height * 0.5f));
        const cv::Point br(static_cast<int>(center_x + width * 0.5f),
                           static_cast<int>(center_y + height * 0.5f));

        const bool valid_object = box.c_idx >= 0 &&
            static_cast<size_t>(box.c_idx) < st_LiDAR.vec_Objects.size();
        const bool matched = matched_indices.count(box.c_idx) != 0;
        const cv::Scalar color = matched ? cv::Scalar(255, 255, 0)
                                         : cv::Scalar(0, 0, 255);
        cv::rectangle(image, tl, br, color, matched ? 3 : 2);

        std::string label = "LiDAR";
        if (matched && valid_object) {
            label = "FUSED: " + st_LiDAR.vec_Objects[box.c_idx].str_ObjectClass;
        }
        cv::putText(image, label,
                    cv::Point(tl.x, std::min(image.rows - 8, br.y + 24)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv::LINE_AA);
    }

    cv::putText(image, "GREEN=YOLO  RED=LiDAR  CYAN=FUSED",
                cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    const ros::Time stamp = st_Camera.front_image_stamp.isZero()
        ? ros::Time::now() : st_Camera.front_image_stamp;

    // Publish a raw base topic as well. rqt_image_view subscribes to this
    // sensor_msgs/Image topic directly on this ROS Noetic installation even
    // when _image_transport:=compressed is supplied.
    sensor_msgs::Image raw_msg;
    raw_msg.header.stamp = stamp;
    raw_msg.header.frame_id = "camera";
    raw_msg.height = static_cast<uint32_t>(image.rows);
    raw_msg.width = static_cast<uint32_t>(image.cols);
    raw_msg.encoding = "bgr8";
    raw_msg.is_bigendian = false;
    raw_msg.step = static_cast<uint32_t>(image.cols * image.elemSize());
    if (image.isContinuous()) {
        raw_msg.data.assign(image.datastart, image.dataend);
    } else {
        const cv::Mat contiguous = image.clone();
        raw_msg.data.assign(contiguous.datastart, contiguous.dataend);
    }
    FusionDebugRawImagePub().publish(raw_msg);

    sensor_msgs::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera";
    msg.format = "jpeg";
    const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 80};
    if (cv::imencode(".jpg", image, msg.data, encode_params)) {
        FusionDebugImagePub().publish(msg);
    }
}

/**
 * @brief 카메라-LiDAR 퓨전 프로세스를 총괄하는 메인 함수입니다.
 */
void runCameraFusion(RAW_YOLO_DATA& st_RawYOLO, LiDAR& st_LiDAR,
                     CAMERA &st_Camera, const ros::Time& lidar_stamp,
                     const ros::WallTime& decision_time)
{
    // RAW_YOLO_DATA remains in the public interface for legacy callers.  Live
    // ROS input is queued at YoloCallback so no intermediate result is lost
    // when this processing loop is slower than the camera callback.
    (void)st_RawYOLO;

    static ros::NodeHandle control_nh;
    static ros::Subscriber control_sub = control_nh.subscribe(
        "/camera_object_detector/inference_status", 1, CameraControl);
    static uint64 admitted_epoch = 0;
    static std::deque<TimedLidarObjects> pending_lidar;
    static std::deque<TimedYoloFrame> yolo_history;
    static std::deque<TimedCameraImage> camera_history;
    static ros::Time last_queued_lidar_stamp;
    auto& stats = SyncStats();
    static bool received_any_yolo = false;
    ros::Time first_arriving_yolo_stamp;

    // Move callback-owned input to this processing thread without holding the
    // mutex during image decoding, projection, or IoU calculation.
    {
        std::deque<TimedYoloFrame> new_yolo;
        std::deque<TimedCameraImage> new_camera;
        {
            std::lock_guard<std::mutex> lock(g_FusionInputMutex);
            if (g_HasControl && (ros::WallTime::now()-g_LastControl).toSec() > .6 && g_CameraActive) {
                g_CameraActive = false;
                ++g_CameraEpoch;
                g_YoloInputQueue.clear();
            }
            if (!g_LastYolo.isZero() && !g_YoloSilent &&
                (ros::WallTime::now()-g_LastYolo).toSec() > .35) {
                g_YoloSilent = true;
                ++g_CameraEpoch;
                g_YoloInputQueue.clear();
            }
            if (admitted_epoch != g_CameraEpoch) {
                fusion_trace::Event("camera_epoch_invalidated", lidar_stamp.toNSec(), [&](std::ostream& out) {
                    out << ",\"epoch\":" << g_CameraEpoch
                        << ",\"discarded_pending_lidar\":" << pending_lidar.size()
                        << ",\"discarded_yolo\":" << yolo_history.size();
                });
                pending_lidar.clear(); yolo_history.clear(); camera_history.clear();
                g_PendingLidar = g_PendingYolo = 0;
                received_any_yolo = false;
                admitted_epoch = g_CameraEpoch;
                for (auto& object : st_LiDAR.vec_Objects)
                    fusion_class::InvalidateObservation(object, g_CameraEpoch);
                // A missed/reappearing track must not reacquire classification
                // from an older history snapshot after a producer restart.
                for (auto& snapshot : st_LiDAR.Prev_Objects_Queue)
                    for (auto& object : snapshot)
                        fusion_class::InvalidateObservation(object, g_CameraEpoch);
            }
            if (!g_CameraActive || g_YoloSilent) {
                for (auto& object : st_LiDAR.vec_Objects) {
                    fusion_class::InvalidateObservation(object, g_CameraEpoch);
                }
                if (!st_LiDAR.Prev_Objects_Queue.empty())
                    fusion_state::Persist(st_LiDAR.vec_Objects, st_LiDAR.Prev_Objects_Queue.back());
                return;
            }
            new_yolo.swap(g_YoloInputQueue);
            new_camera.swap(g_CameraInputQueue);
        }
        for (auto& frame : new_yolo) {
            if (!received_any_yolo &&
                (first_arriving_yolo_stamp.isZero() ||
                 frame.stamp < first_arriving_yolo_stamp)) {
                first_arriving_yolo_stamp = frame.stamp;
            }
            fusion_trace::Event("yolo_admitted", frame.stamp.toNSec(), [](std::ostream&) {});
            InsertByStamp(yolo_history, std::move(frame));
            received_any_yolo = true;
        }
        for (auto& frame : new_camera) {
            InsertByStamp(camera_history, std::move(frame));
        }
    }

    if (!lidar_stamp.isZero() &&
        lidar_stamp != last_queued_lidar_stamp) {
        if(frame_boundary::Enabled())
            fusion_trace::Event("lidar_coordinate_context",lidar_stamp.toNSec(),[&](std::ostream& out) {
                const auto& c=st_LiDAR.frame_context;
                out<<",\"generation\":"<<c.input_generation
                    <<",\"calibration_sha\":"<<fusion_trace::Quote(c.calibration_source)
                    <<",\"input_frame\":\"leveled_sensor\",\"map_valid\":"<<(c.map_pose.valid?"true":"false")
                    <<",\"map_reason\":"<<fusion_trace::Quote(c.map_pose.reason)<<",\"leveling\":[";
                for(unsigned i=0;i<9;++i) {if(i) out<<',';out<<std::setprecision(17)<<c.applied_leveling(i/3,i%3);}
                out<<']';
            });
        pending_lidar.push_back(
            {lidar_stamp, st_LiDAR.vec_Objects, decision_time, st_LiDAR.frame_context});
        last_queued_lidar_stamp = lidar_stamp;
    }

    static const std::string consumption_policy = [] {
        std::string policy;
        ros::param::param<std::string>("~fusion_yolo_policy", policy, "unique");
        if (policy != "unique" && policy != "bounded_reuse")
            throw std::runtime_error("invalid fusion_yolo_policy");
        return policy;
    }();
    const bool reuse = consumption_policy == "bounded_reuse";
    double max_time_offset_sec = 0.05;
    double wait_timeout_sec = 0.20;
    ros::param::param("~fusion_max_time_offset_sec", max_time_offset_sec,
                      max_time_offset_sec);
    ros::param::param("~fusion_sync_wait_timeout_sec", wait_timeout_sec,
                      wait_timeout_sec);
    max_time_offset_sec = std::max(0.001, max_time_offset_sec);
    wait_timeout_sec = std::max(max_time_offset_sec, wait_timeout_sec);
    const int64 max_offset_ns = static_cast<int64>(
        std::llround(max_time_offset_sec * 1e9));
    const int64 wait_timeout_ns = static_cast<int64>(
        std::llround(wait_timeout_sec * 1e9));

    // LiDAR frames retained while the model was loading predate the first
    // possible YOLO match.  Discard only those provably outside the strict
    // matching window; counting them as runtime sync failures would be a
    // misleading startup warning.
    if (!first_arriving_yolo_stamp.isZero()) {
        const ros::Time earliest_matchable_yolo =
            first_arriving_yolo_stamp - ros::Duration(max_time_offset_sec);
        while (!pending_lidar.empty() &&
               pending_lidar.front().stamp < earliest_matchable_yolo) {
            fusion_trace::Event("lidar_startup_discard", pending_lidar.front().stamp.toNSec(),
                                [](std::ostream&) {});
            pending_lidar.pop_front();
        }
    }

    // Model loading can take several seconds on first start.  That is not a
    // synchronization failure, so retain only the newest LiDAR snapshots and
    // begin timeout diagnostics after the first actual YOLO result arrives.
    if (!received_any_yolo) {
        while (pending_lidar.size() > 2) {
            fusion_trace::Event("lidar_startup_overflow", pending_lidar.front().stamp.toNSec(),
                                [](std::ostream&) {});
            pending_lidar.pop_front();
        }
        g_PendingLidar = pending_lidar.size();
        g_PendingYolo = yolo_history.size();
        stats.ReportIfDue(pending_lidar.size(), yolo_history.size());
        return;
    }

    while (!pending_lidar.empty()) {
        const TimedLidarObjects& lidar_frame = pending_lidar.front();
        std::vector<int64> yolo_stamps_ns;
        yolo_stamps_ns.reserve(yolo_history.size());
        const uint64 empty_barrier = reuse ? fusion_sync::LatestEmptyStamp(yolo_history) : 0;
        for (const auto& frame : yolo_history) {
            yolo_stamps_ns.push_back(frame.stamp.toNSec() < empty_barrier
                ? 0 : static_cast<int64>(frame.stamp.toNSec()));
        }
        const int64 waited_ns = static_cast<int64>(std::llround(
            (decision_time - lidar_frame.queued_at).toSec() * 1e9));
        const auto decision = fusion_sync::DecideMatch(
            static_cast<int64>(lidar_frame.stamp.toNSec()), yolo_stamps_ns,
            waited_ns, max_offset_ns, wait_timeout_ns);
        fusion_trace::Event("match_decision", lidar_frame.stamp.toNSec(), [&](std::ostream& out) {
            out << ",\"action\":" << fusion_trace::Quote(
                decision.action == fusion_sync::MatchAction::Wait ? "wait" :
                decision.action == fusion_sync::MatchAction::Match ? "match" : "drop")
                << ",\"waited_ns\":" << waited_ns << ",\"pending_lidar\":" << pending_lidar.size()
                << ",\"yolo_stamps_ns\":[";
            for (std::size_t index = 0; index < yolo_stamps_ns.size(); ++index) {
                if (index) out << ',';
                out << yolo_stamps_ns[index];
            }
            out << "]";
            if (decision.yolo_index < yolo_stamps_ns.size()) {
                out << ",\"selected_yolo_stamp_ns\":" << yolo_stamps_ns[decision.yolo_index]
                    << ",\"offset_ns\":" << decision.offset_ns;
            }
        });

        if (decision.action == fusion_sync::MatchAction::Wait) break;

        TimedLidarObjects synchronized_lidar_frame =
            std::move(pending_lidar.front());
        pending_lidar.pop_front();

        if (decision.action == fusion_sync::MatchAction::Drop) {
            ++stats.dropped;
            ++g_TotalDropped;
            const double nearest_ms =
                decision.offset_ns == std::numeric_limits<int64>::max()
                    ? -1.0
                    : decision.offset_ns / 1e6;
            ROS_WARN_THROTTLE(
                2.0,
                "Skipping LiDAR-centered fusion after %.1f ms wait: no eligible YOLO frame within %.1f ms (nearest %.1f ms)",
                waited_ns / 1e6, max_time_offset_sec * 1000.0,
                nearest_ms);
            PruneInputsBefore(yolo_history, camera_history,
                              synchronized_lidar_frame.stamp,
                              max_time_offset_sec);
            continue;
        }

        TimedYoloFrame yolo_frame = yolo_history[decision.yolo_index];
        g_ProcessingCameraEpoch = yolo_frame.epoch;

        LiDAR synchronized_lidar;
        synchronized_lidar.frame_context=synchronized_lidar_frame.context;
        synchronized_lidar.vec_Objects =
            std::move(synchronized_lidar_frame.objects);
        CAMERA synchronized_camera = st_Camera;
        auto closest_image = camera_history.end();
        double image_buffer_offset_sec =
            std::numeric_limits<double>::infinity();
        for (auto it = camera_history.begin(); it != camera_history.end(); ++it) {
            const double candidate_offset =
                std::abs((yolo_frame.stamp - it->stamp).toSec());
            if (candidate_offset < image_buffer_offset_sec) {
                closest_image = it;
                image_buffer_offset_sec = candidate_offset;
            }
        }
        if (closest_image != camera_history.end() &&
            image_buffer_offset_sec <= 0.002) {
            const cv::Mat encoded(
                1, static_cast<int>(closest_image->compressed.size()), CV_8UC1,
                closest_image->compressed.data());
            synchronized_camera.bgr_image =
                cv::imdecode(encoded, cv::IMREAD_COLOR);
            synchronized_camera.front_compressed_image =
                closest_image->compressed;
        } else {
            ROS_WARN_THROTTLE(2.0,
                "Fusion source image missing from callback buffer; debug background may be newer");
        }
        synchronized_camera.front_image_stamp = yolo_frame.stamp;
        if (synchronized_camera.bgr_image.empty()) {
            if (!reuse) yolo_history.erase(yolo_history.begin()+decision.yolo_index);
            ++stats.dropped;
            ++g_TotalDropped;
            fusion_trace::Event("camera_decode_drop", synchronized_lidar_frame.stamp.toNSec(),
                                [](std::ostream&) {});
            ROS_WARN_THROTTLE(
                2.0,
                "Skipping LiDAR-centered fusion: synchronized camera JPEG could not be decoded");
            PruneInputsBefore(yolo_history, camera_history,
                              synchronized_lidar_frame.stamp,
                              max_time_offset_sec);
            continue;
        }

        const bool reused = fusion_sync::ConsumeMatchedFrame(yolo_history, decision.yolo_index, reuse);
        fusion_trace::Event("camera_observation_use", synchronized_lidar_frame.stamp.toNSec(), [&](std::ostream& out) {
            out << ",\"camera_id\":" << fusion_trace::Quote(yolo_frame.camera_id)
                << ",\"calibration_id\":" << fusion_trace::Quote(yolo_frame.calibration_id)
                << ",\"epoch\":" << yolo_frame.epoch
                << ",\"source_stamp_ns\":" << yolo_frame.stamp.toNSec()
                << ",\"reused\":" << (reused ? "true" : "false");
        });

        // The geometry comes from the synchronized historical frame, while
        // confidence/candidate counters must continue from the current track.
        for (auto& historical_object : synchronized_lidar.vec_Objects) {
            if (LiDARObjectInfo* current = fusion_state::Find(
                    st_LiDAR.vec_Objects, historical_object.s32_Id)) {
                fusion_state::Copy(*current, historical_object, true);
            }
        }

        {
            synchronized_camera.yolo_results.detected_objects.clear();

            const float32 image_width =
                static_cast<float32>(synchronized_camera.bgr_image.cols);
            const float32 image_height =
                static_cast<float32>(synchronized_camera.bgr_image.rows);

            // Reject the ego hood false-positive seen at the bottom of the
            // front camera. The three conditions together avoid discarding a
            // normal nearby vehicle that merely touches the image bottom.
            for (const auto& box : yolo_frame.boxes) {
                const float32 width =
                    box.st_Max_pt.f32_X - box.st_Min_pt.f32_X;
                const float32 center_y =
                    (box.st_Min_pt.f32_Y + box.st_Max_pt.f32_Y) * 0.5f;
                const std::string normalized_class =
                    fusion_class::Normalize(box.str_class);
                const bool is_ego_hood =
                    normalized_class == "car" &&
                    width >= image_width * 0.60f &&
                    center_y >= image_height * 0.85f &&
                    box.st_Max_pt.f32_Y >= image_height * 0.95f;
                if (is_ego_hood) {
                    ROS_WARN_THROTTLE(5.0,
                        "Filtered ego-hood YOLO false positive from fusion input");
                    continue;
                }
                BBox normalized_box = box;
                normalized_box.str_class = normalized_class;
                synchronized_camera.yolo_results.detected_objects.push_back(
                    normalized_box);
            }
        }
        
        LidarToImage(synchronized_lidar, synchronized_camera, synchronized_camera.params);

        TraceObjectStates("class_before", synchronized_lidar_frame.stamp,
                          yolo_frame.stamp, synchronized_lidar.vec_Objects);

        IOUFusion(synchronized_lidar, synchronized_camera, true);
        TraceObjectStates("class_after", synchronized_lidar_frame.stamp,
                          yolo_frame.stamp, synchronized_lidar.vec_Objects);

        for (const auto& historical_object : synchronized_lidar.vec_Objects) {
            if (LiDARObjectInfo* current = fusion_state::Find(
                    st_LiDAR.vec_Objects, historical_object.s32_Id)) {
                fusion_state::Copy(historical_object, *current, true);
                fusion_state_trace::Transferred(current->s32_Id,
                    synchronized_lidar_frame.stamp,
                    ros::Time().fromNSec(current->camera_observation_ns), lidar_stamp);
            }
        }
        TraceObjectStates("class_current", lidar_stamp,
                          yolo_frame.stamp, st_LiDAR.vec_Objects);

        // LidarProcess stores the current tracking frame before camera fusion
        // runs. Persist only the fused classification state into that frame.
        // Copying the whole object also copies the AABB-updated CoreX/CoreY
        // into the tracking history, which creates a false displacement and
        // therefore a persistent false velocity on the next LiDAR frame.
        if (!st_LiDAR.Prev_Objects_Queue.empty()) {
            fusion_state::Persist(st_LiDAR.vec_Objects,
                                  st_LiDAR.Prev_Objects_Queue.back());
        }

        PublishFusionDebugImage(synchronized_lidar, synchronized_camera);

        ++stats.matched;
        ++g_TotalMatched;
        if (reused) ++g_TotalReused; else ++g_TotalFresh;
        stats.offsets_ms.push_back(decision.offset_ns / 1e6);
        PruneInputsBefore(yolo_history, camera_history,
                          synchronized_lidar_frame.stamp,
                          max_time_offset_sec);
    }

    // Hard bounds protect against a stopped downstream thread or malformed
    // timestamps even when the normal time-based pruning cannot make progress.
    while (pending_lidar.size() > 32) {
        fusion_trace::Event("lidar_pending_overflow", pending_lidar.front().stamp.toNSec(),
                            [](std::ostream&) {});
        pending_lidar.pop_front();
    }
    while (yolo_history.size() > 128) {
        fusion_trace::Event("yolo_history_overflow", yolo_history.front().stamp.toNSec(),
                            [](std::ostream&) {});
        yolo_history.pop_front();
    }
    while (camera_history.size() > 80) camera_history.pop_front();
    g_PendingLidar = pending_lidar.size();
    g_PendingYolo = yolo_history.size();
    stats.ReportIfDue(pending_lidar.size(), yolo_history.size());
}
