#include "SensorFusion.hpp" // (or your relevant main header)
#include "Global/Global.hpp"       // LiDARObjectInfo가 정의된 헤더
#include <iterator>
#include <vector>
#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

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
static void LidarToImage(LiDAR &st_LiDAR, CAMERA &st_Camera)
{
    st_Camera.fusion_data.vec_Lidar_BBoxes.clear();

    // 제외할 y-밴드
    int ex_ymin = 637, ex_ymax = 720;
    ros::param::get("~exclude_ymin_px", ex_ymin);
    ros::param::get("~exclude_ymax_px", ex_ymax);

    for (size_t i = 0; i < st_LiDAR.vec_Objects.size(); ++i) {
        auto& det = st_LiDAR.vec_Objects[i];
        // Do not associate a YOLO detection with a prediction-only ghost.
        // Its stored point cloud is from an older LiDAR frame and can steal
        // the class from the currently observed cluster.
        if (det.s32_MissCount > 0) continue;
        std::vector<cv::Point> projected_points;

        for (const auto& pt : det.pcl_ClusterPoints.points) {
            Eigen::Vector4d pt_vec(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector3d cam = st_Camera.params.sensor_fusion_projection_matrix * pt_vec;

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
            if (overlapsYBand(bbox, ex_ymin, ex_ymax, st_Camera.bgr_image.rows)) {
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
 * 일정 횟수 이상 감지되면 클래스를 고정합니다.
 * @param st_LiDAR LiDAR 데이터 구조체
 * @param st_Camera 카메라 데이터 구조체
 */
static void IOUFusion(LiDAR& st_LiDAR, CAMERA &st_Camera,
                      bool is_new_yolo_frame)
{
    st_LiDAR.vec_FusionIdx.clear();
    const float32 fusion_x_offset = 0;
    
    // Count real YOLO callbacks, not the faster Main loop. Roughly one second
    // of consistent matches locks a class; short IoU dropouts keep the label.
    static int CLASS_LOCK_THRESHOLD = 12;
    static int CLASS_MISS_GRACE = 8;
    static double FUSION_IOU_THRESHOLD = 0.25;
    static bool params_loaded = false;
    if (!params_loaded) {
        ros::param::param("~fusion_class_lock_frames", CLASS_LOCK_THRESHOLD,
                         CLASS_LOCK_THRESHOLD);
        ros::param::param("~fusion_class_miss_grace", CLASS_MISS_GRACE,
                         CLASS_MISS_GRACE);
        ros::param::param("~fusion_iou_threshold", FUSION_IOU_THRESHOLD,
                         FUSION_IOU_THRESHOLD);
        params_loaded = true;
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

        // Locked objects remain associated for visualization, but their class
        // and confidence counter no longer change.
        if (!lid_object.is_class_locked && is_new_yolo_frame) {
                lid_object.class_miss_counter = 0;
                if (lid_object.last_seen_class == cam.str_class) {
                    lid_object.continuity_counter++;
                } else {
                    lid_object.last_seen_class = cam.str_class;
                    lid_object.continuity_counter = 1;
                }
                
                // 3. 현재 프레임의 클래스를 즉시 할당
                lid_object.str_ObjectClass = cam.str_class;

                // 4. 클래스 고정 조건 확인
                if (lid_object.continuity_counter >= CLASS_LOCK_THRESHOLD) {
                    lid_object.is_class_locked = true;
                }
        }
    }

    for (auto& matches : matches_by_camera) {
        if (!matches.empty()) st_LiDAR.vec_FusionIdx.push_back(matches);
    }
    
    // 5. YOLO에 매칭되지 않은 객체들의 연속성 카운터 감소 처리
    if (is_new_yolo_frame) for (size_t i = 0; i < st_LiDAR.vec_Objects.size(); ++i) {
        // 이 프레임에서 매칭이 안됐고, 클래스가 고정되지 않았다면
        if (matched_lidar_indices_this_frame.find(i) == matched_lidar_indices_this_frame.end() && !st_LiDAR.vec_Objects[i].is_class_locked)
        {
            st_LiDAR.vec_Objects[i].class_miss_counter++;
            // 연속성 카운터를 1씩 감소시켜 몇 프레임의 여유를 줌
            if (st_LiDAR.vec_Objects[i].continuity_counter > 0) {
                st_LiDAR.vec_Objects[i].continuity_counter--; 
            }

            // 카운터가 0이 되면, 마지막으로 본 클래스 정보 리셋
            if (st_LiDAR.vec_Objects[i].continuity_counter == 0 &&
                st_LiDAR.vec_Objects[i].class_miss_counter > CLASS_MISS_GRACE) {
                st_LiDAR.vec_Objects[i].last_seen_class = "unknown";
                st_LiDAR.vec_Objects[i].str_ObjectClass = "unknown";
            }
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
        const bool locked = valid_object &&
            st_LiDAR.vec_Objects[box.c_idx].is_class_locked;
        const bool fused_or_locked = matched || locked;
        const cv::Scalar color = fused_or_locked ? cv::Scalar(255, 255, 0)
                                                 : cv::Scalar(0, 0, 255);
        cv::rectangle(image, tl, br, color, fused_or_locked ? 3 : 2);

        std::string label = "LiDAR";
        if (locked) {
            label = "LOCKED: " + st_LiDAR.vec_Objects[box.c_idx].str_ObjectClass;
        } else if (matched && valid_object) {
            label = "FUSED: " + st_LiDAR.vec_Objects[box.c_idx].str_ObjectClass;
        }
        cv::putText(image, label,
                    cv::Point(tl.x, std::min(image.rows - 8, br.y + 24)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv::LINE_AA);
    }

    cv::putText(image, "GREEN=YOLO  RED=LiDAR  CYAN=FUSED/LOCKED",
                cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    const ros::Time stamp = ros::Time::now();

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
void runCameraFusion(RAW_YOLO_DATA& st_RawYOLO, LiDAR& st_LiDAR, CAMERA &st_Camera)
{
    if (st_RawYOLO.b_Yolo_ready && !st_Camera.bgr_image.empty()) {
        static uint64 last_yolo_frame_seq = 0;
        const bool is_new_yolo_frame =
            st_RawYOLO.u64_FrameSeq != last_yolo_frame_seq;

        if (is_new_yolo_frame) {
            last_yolo_frame_seq = st_RawYOLO.u64_FrameSeq;
            st_Camera.yolo_results.detected_objects.clear();

            const float32 image_width =
                static_cast<float32>(st_Camera.bgr_image.cols);
            const float32 image_height =
                static_cast<float32>(st_Camera.bgr_image.rows);

            // Reject the ego hood false-positive seen at the bottom of the
            // front camera. The three conditions together avoid discarding a
            // normal nearby vehicle that merely touches the image bottom.
            for (const auto& box : st_RawYOLO.vec_YOLO) {
                const float32 width =
                    box.st_Max_pt.f32_X - box.st_Min_pt.f32_X;
                const float32 center_y =
                    (box.st_Min_pt.f32_Y + box.st_Max_pt.f32_Y) * 0.5f;
                const bool is_ego_hood =
                    box.str_class == "car" &&
                    width >= image_width * 0.60f &&
                    center_y >= image_height * 0.85f &&
                    box.st_Max_pt.f32_Y >= image_height * 0.95f;
                if (is_ego_hood) {
                    ROS_WARN_THROTTLE(5.0,
                        "Filtered ego-hood YOLO false positive from fusion input");
                    continue;
                }
                st_Camera.yolo_results.detected_objects.push_back(box);
            }
        }
        
        LidarToImage(st_LiDAR, st_Camera);

        IOUFusion(st_LiDAR, st_Camera, is_new_yolo_frame);

        // LidarProcess stores the current tracking frame before camera fusion
        // runs. Persist only the fused classification state into that frame.
        // Copying the whole object also copies the AABB-updated CoreX/CoreY
        // into the tracking history, which creates a false displacement and
        // therefore a persistent false velocity on the next LiDAR frame.
        if (!st_LiDAR.Prev_Objects_Queue.empty()) {
            auto& previous_objects = st_LiDAR.Prev_Objects_Queue.back();

            for (const auto& fused_object : st_LiDAR.vec_Objects) {
                auto previous_it = std::find_if(
                    previous_objects.begin(),
                    previous_objects.end(),
                    [&fused_object](const LiDARObjectInfo& previous_object) {
                        return previous_object.s32_Id == fused_object.s32_Id;
                    });

                if (previous_it == previous_objects.end()) {
                    continue;
                }

                previous_it->str_ObjectClass =
                    fused_object.str_ObjectClass;
                previous_it->last_seen_class =
                    fused_object.last_seen_class;
                previous_it->continuity_counter =
                    fused_object.continuity_counter;
                previous_it->class_miss_counter =
                    fused_object.class_miss_counter;
                previous_it->is_class_locked =
                    fused_object.is_class_locked;
            }
        }

        PublishFusionDebugImage(st_LiDAR, st_Camera);
    }
}
