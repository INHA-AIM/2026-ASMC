#ifndef TUNNEL_FILTER_ROS_HPP
#define TUNNEL_FILTER_ROS_HPP

// 터널 실험 필터의 ROS 경계.
//
// ~enable_tunnel_experimental_filter=false(기본)이면 Init은 아무것도 만들지
// 않고, Capture/Run은 불리언 하나만 확인하고 돌아간다. true여도 GPS가 정상인
// 프레임에서는 캡처만 건너뛰므로 기존 파이프라인만 동작한다.
//
// 실험 결과는 /lidar/tunnel/* 전용 토픽으로만 나간다. 기존 /last_lidar,
// /lidar_nogps, 추적기 입력은 바꾸지 않는다. 검증 후 연결할 함수 경계는
// tunnel_filter::TunnelPipeline::Process()의 kept_indices/objects다.

#include <Global/Global.hpp>
#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>

#include <cstdint>
#include <vector>

namespace tunnel_filter {

struct ExperimentFrame {
    std::uint64_t sequence = 0;  // 처리(또는 비활성 판정)할 때마다 증가
    bool active = false;         // 이 프레임이 GPS 음영 모드였는지
    ros::Time stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input;  // velodyne 센서 좌표(ROI+Voxel)
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    bool imu_ready = false;
    bool protection_available = false;
    double yolo_offset_ms = -1.0;
    int yolo_boxes = 0;
    int protect_boxes = 0;
    std::vector<std::uint8_t> protect_class;  // 입력 점별
    std::vector<std::uint8_t> ambiguous;      // 입력 점별 (카메라 FOV 안)
    const FrameResult* result = nullptr;      // 다음 Run 전까지 유효
    double protection_ms = 0.0;
    double publish_ms = 0.0;
    double total_ms = 0.0;
};

// ~enable_tunnel_experimental_filter를 읽는다. false면 아무 것도 생성하지 않고
// false를 반환한다. subscribe_yolo=false는 오프라인 평가기용(직접 주입).
bool InitTunnelExperiment(ros::NodeHandle& node_handle,
                          ros::NodeHandle& private_node,
                          const CAMERA& camera, int image_width,
                          int image_height, bool subscribe_yolo = true);
bool TunnelExperimentEnabled();

// LidarProcess 안, Voxel 직후에 호출한다. 포인터만 보관한다 (복사 없음).
void TunnelExperimentCapture(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                             const ros::Time& stamp, const GPS_IMU& gps_imu);

// TemporalThread에서 기존 출력(추적/퓨전/플래너 퍼블리시)을 모두 낸 뒤 호출한다.
void TunnelExperimentRun();

// /YOLO/BBox 콜백(또는 평가기)이 호출한다.
void TunnelExperimentIngestYolo(const ros::Time& image_stamp,
                                const std::vector<BBox>& boxes);

const ExperimentFrame& TunnelExperimentLastFrame();
const Params& TunnelExperimentParams();

// 기존 추적기와 같은 L-shape OBB(Objectinfo_for_km)를 쓰는 OBB 계산기.
ObbFunction MakeTrackingObbFunction();

// ROS 파라미터(~tunnel_*)로 Params를 채운다. 없는 키는 base 값을 유지한다
// (평가기 sweep은 기준 설정에서 바꾼 키만 덮어쓴다).
Params LoadParams(ros::NodeHandle& private_node, const Params& base = Params());

}  // namespace tunnel_filter

#endif  // TUNNEL_FILTER_ROS_HPP
