// 터널 실험 필터 결정론적 오프라인 평가기.
//
// 원본 bag을 읽기 전용으로 열고, 실시간 Main과 같은 라이브러리 함수
// (Parser 콜백 -> DeadReckoning -> LidarProcess(터널 필터 포함) ->
// FusionProcess -> PlannerObjectRosPublisher)를 bag 기록 순서대로
// 호출한다. 멀티스레드 타이밍 대신 "스캔 stamp + processing_delay 시점까지의
// 메시지를 먼저 반영한 뒤 그 스캔을 처리"하는 방식으로 실시간 지연을 재현하며,
// 모든 스캔을 빠짐없이 처리한다.
//
// 안전장치: 실시간 시스템(11311)과 다른 테스트 master(11312)에는 붙지 않는다.
// PlannerObjectSender(플래너 UDP)는 만들지 않는다.

#include <Camera/Camera.hpp>
#include <DeadReckoning/DeadReckoning.hpp>
#include <Global/Global.hpp>
#include <Lidar/LidarProcess/LidarProcess.hpp>
#include <Lidar/LidarProcess/PipelineProfile.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>
#include <Msg/TrackedObjectArray.h>
#include <Parser/Parser.hpp>
#include <PlannerInterface/PlannerObjectRosPublisher.hpp>
#include <SensorFusion/SensorFusion.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <chrono>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <unordered_set>

#ifdef TUNNEL_EVAL_BASELINE_ONLY
// 코드 변경 전 소스(터널 모듈 없음)에 같은 평가기를 링크해 기존 경로 출력이
// 비트 단위로 같은지 비교하기 위한 빌드. 실험 경로는 항상 비활성이다.
namespace tunnel_filter {
struct ExperimentFrame {
    std::uint64_t sequence = 0;
    bool active = false;
    ros::Time stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input;
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    bool protection_available = false;
    double yolo_offset_ms = -1.0;
    int yolo_boxes = 0;
    int protect_boxes = 0;
    std::vector<std::uint8_t> protect_class;
    std::vector<std::uint8_t> ambiguous;
    const FrameResult* result = nullptr;
    double protection_ms = 0.0;
    double publish_ms = 0.0;
    double total_ms = 0.0;
};
inline bool InitTunnelExperiment(ros::NodeHandle&, ros::NodeHandle&, const CAMERA&, int, int, bool) {
    return false;
}
inline void TunnelExperimentRun() {}
inline void TunnelExperimentIngestYolo(const ros::Time&, const std::vector<BBox>&) {}
inline const ExperimentFrame& TunnelExperimentLastFrame() {
    static ExperimentFrame frame;
    return frame;
}
inline bool TunnelExperimentOutputReady(const ros::Time&) { return false; }
inline const Params& TunnelExperimentParams() {
    static Params params;
    return params;
}
inline Params LoadParams(ros::NodeHandle&, const Params& base = Params()) { return base; }
inline ObbFunction MakeTrackingObbFunction() { return DefaultObbFunction; }
}  // namespace tunnel_filter
#else
#include <Lidar/TunnelFilter/TunnelFilterRos.hpp>
#endif

// ===== main.cpp와 같은 전역 정의 (libmyHeader가 extern으로 참조) =====
const string YamlURL = ros::package::getPath("Main") + "/setting.yaml";
ros::Publisher VizPub, CtrlPub;
ros::ServiceClient GearPub;
ros::Subscriber LiDAR_Sub, Camera_Sub;
ros::Publisher VelPlotPub;
ros::Timer VelPlotTimer;
VehicleSpec st_VehicleSpec;
GPS_IMU st_GPSIMU, st_GPSIMUTemporal;
Planning st_Planning, st_PlanningTemporal;
Control st_Control, st_ControlTemporal;
LiDAR st_LiDAR, st_LiDARTemporal;
CAMERA st_Camera, st_CameraTemporal;
GPS_REF st_GPSRef;
int32 GPSHZ, VisionHZ, LiDARHZ, IMUHZ, CANHZ;
int32 VIEWERHZ;
SENSOR_DATA st_SensorData;
SENSOR_DATA st_CurrentData;
pthread_mutex_t st_CopyMutex;
pthread_mutex_t st_DataTemporal;
pthread_cond_t st_DataTemporalCond;
bool b_isRunning = true;

namespace {

using Clock = std::chrono::steady_clock;
double ElapsedMs(const Clock::time_point& start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// ---------------------------------------------------------------- init (main.cpp와 동일)
void InitMutex() {
    pthread_mutex_init(&st_CopyMutex, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexLiDAR, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexCamera, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexGPS, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexIMU, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexCAN, NULL);
    pthread_mutex_init(&st_SensorData.st_MutexYOLO, NULL);
    pthread_mutex_init(&st_DataTemporal, NULL);
}

void InitVehicleSpec() {
    YAML::Node config = YAML::LoadFile(YamlURL);
    st_VehicleSpec.f32_Length = config["Vehicle"]["Length"].as<float32>();
    st_VehicleSpec.f32_Width = config["Vehicle"]["Width"].as<float32>();
    st_VehicleSpec.f32_WheelBase = config["Vehicle"]["WheelBase"].as<float32>();
    st_VehicleSpec.f32_FrontOverhang = config["Vehicle"]["FrontOverhang"].as<float32>();
    st_VehicleSpec.f32_RearOverhang = config["Vehicle"]["RearOverhang"].as<float32>();
    st_VehicleSpec.f32_TurningRadius = config["Vehicle"]["TurningRadius"].as<float32>();
    st_VehicleSpec.st_LiDARPos.f32_X = config["Vehicle"]["LiDAR"]["x"].as<float32>();
    st_VehicleSpec.st_LiDARPos.f32_Y = config["Vehicle"]["LiDAR"]["y"].as<float32>();
    st_VehicleSpec.st_LiDARPos.f32_Z = config["Vehicle"]["LiDAR"]["z"].as<float32>();
    st_VehicleSpec.st_LiDARRotationDeg.f32_X = config["Vehicle"]["LiDAR"]["roll"].as<float32>();
    st_VehicleSpec.st_LiDARRotationDeg.f32_Y = config["Vehicle"]["LiDAR"]["pitch"].as<float32>();
    st_VehicleSpec.st_LiDARRotationDeg.f32_Z = config["Vehicle"]["LiDAR"]["yaw"].as<float32>();
    st_VehicleSpec.st_GPSPos.f32_X = config["Vehicle"]["GPS"]["x"].as<float32>();
    st_VehicleSpec.st_GPSPos.f32_Y = config["Vehicle"]["GPS"]["y"].as<float32>();
    st_VehicleSpec.st_GPSPos.f32_Z = config["Vehicle"]["GPS"]["z"].as<float32>();
    GPSHZ = config["Vehicle"]["GPS"]["Hz"].as<int32>();
    IMUHZ = config["Vehicle"]["IMU"]["Hz"].as<int32>();
    VisionHZ = config["Vehicle"]["Camera"]["Hz"].as<int32>();
    LiDARHZ = config["Vehicle"]["LiDAR"]["Hz"].as<int32>();
    CANHZ = config["Vehicle"]["CAN"]["Hz"].as<int32>();
}

void InitLidarSettings(ros::NodeHandle& private_node) {
    YAML::Node config = YAML::LoadFile(YamlURL);
    st_LiDAR.s32_LiDARMode = config["LiDAR"]["LiDARMode"].as<int32>();
    st_LiDAR.st_Range.f32_Min = config["LiDAR"]["Filter"]["Range"]["Min"].as<float32>();
    st_LiDAR.st_Range.f32_Max = config["LiDAR"]["Filter"]["Range"]["Max"].as<float32>();
    st_LiDAR.st_ROI.f32_MinX = config["LiDAR"]["Filter"]["ROI"]["MinX"].as<float32>();
    st_LiDAR.st_ROI.f32_MaxX = config["LiDAR"]["Filter"]["ROI"]["MaxX"].as<float32>();
    st_LiDAR.st_ROI.f32_MinY = config["LiDAR"]["Filter"]["ROI"]["MinY"].as<float32>();
    st_LiDAR.st_ROI.f32_MaxY = config["LiDAR"]["Filter"]["ROI"]["MaxY"].as<float32>();
    st_LiDAR.st_ROI.f32_MinZ = config["LiDAR"]["Filter"]["ROI"]["MinZ"].as<float32>();
    st_LiDAR.st_ROI.f32_MaxZ = config["LiDAR"]["Filter"]["ROI"]["MaxZ"].as<float32>();
    st_LiDAR.st_Clustering.b_Enabled = config["LiDAR"]["Clustering"]["Enabled"].as<bool>();
    st_LiDAR.st_Clustering.f32_Eps = config["LiDAR"]["Clustering"]["Eps"].as<float32>();
    st_LiDAR.st_Clustering.s32_MinPoints = config["LiDAR"]["Clustering"]["MinPoints"].as<int32>();
    private_node.param("roi_min_x", st_LiDAR.st_ROI.f32_MinX, st_LiDAR.st_ROI.f32_MinX);
    private_node.param("roi_max_x", st_LiDAR.st_ROI.f32_MaxX, st_LiDAR.st_ROI.f32_MaxX);
    private_node.param("roi_min_y", st_LiDAR.st_ROI.f32_MinY, st_LiDAR.st_ROI.f32_MinY);
    private_node.param("roi_max_y", st_LiDAR.st_ROI.f32_MaxY, st_LiDAR.st_ROI.f32_MaxY);
}

// main.cpp InitializeCamera()에서 카메라 스레드 생성만 뺀 것.
void InitCameraSettings() {
    YAML::Node config = YAML::LoadFile(YamlURL);
    st_Camera.s32_CameraMode = config["Camera"]["CameraMode"].as<int32>();
    st_Camera.front_camera_udp.str_Ip = config["Camera"]["UDP1"]["ip"].as<string>();
    st_Camera.front_camera_udp.s32_Port = config["Camera"]["UDP1"]["port"].as<int32>();
    st_Camera.traffic_light_camera_udp.str_Ip = config["Camera"]["UDP2"]["ip"].as<string>();
    st_Camera.traffic_light_camera_udp.s32_Port = config["Camera"]["UDP2"]["port"].as<int32>();
    st_Camera.s32_Display = config["Camera"]["Display"].as<int32>();
    st_Camera.hsv_thresholds.lower_red1 = config["Camera"]["HSV"]["lower_red1"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_red1 = config["Camera"]["HSV"]["upper_red1"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_red2 = config["Camera"]["HSV"]["lower_red2"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_red2 = config["Camera"]["HSV"]["upper_red2"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_green = config["Camera"]["HSV"]["lower_green"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_green = config["Camera"]["HSV"]["upper_green"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_yellow = config["Camera"]["HSV"]["lower_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_yellow = config["Camera"]["HSV"]["upper_yellow"].as<vector<int32>>();
    st_Camera.params.world_points_vector = config["Camera"]["BEV"]["WorldPoints"].as<vector<float32>>();
    st_Camera.hsv_thresholds.lower_bright_yellow = config["Camera"]["HSV"]["lower_bright_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_bright_yellow = config["Camera"]["HSV"]["upper_bright_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_dark_yellow = config["Camera"]["HSV"]["lower_dark_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_dark_yellow = config["Camera"]["HSV"]["upper_dark_yellow"].as<vector<int32>>();
    st_Camera.hsv_thresholds.lower_white = config["Camera"]["HSV"]["lower_white"].as<vector<int32>>();
    st_Camera.hsv_thresholds.upper_white = config["Camera"]["HSV"]["upper_white"].as<vector<int32>>();
    st_Camera.traffic_light_state.s32_Threshold = config["Camera"]["TrafficLight"]["Threshold"].as<int32>();
    st_Camera.params.moving_average_filter_size = config["Camera"]["BEV"]["MAF"].as<int32>();
    InitProjectionMatrix(st_Camera, config);
}

void CopyData(SENSOR_DATA* data) {
    pthread_mutex_lock(&st_SensorData.st_MutexLiDAR);
    memcpy(&(data->st_RawLiDAR), &st_SensorData.st_RawLiDAR, sizeof(RAW_LIDAR_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexLiDAR);
    pthread_mutex_lock(&st_SensorData.st_MutexGPS);
    memcpy(&(data->st_RawGPS), &st_SensorData.st_RawGPS, sizeof(RAW_GPS_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexGPS);
    pthread_mutex_lock(&st_SensorData.st_MutexIMU);
    memcpy(&(data->st_RawIMU), &st_SensorData.st_RawIMU, sizeof(RAW_IMU_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexIMU);
    pthread_mutex_lock(&st_SensorData.st_MutexCAN);
    memcpy(&(data->st_RawCAN), &st_SensorData.st_RawCAN, sizeof(RAW_CAN_DATA));
    pthread_mutex_unlock(&st_SensorData.st_MutexCAN);
    pthread_mutex_lock(&st_SensorData.st_MutexYOLO);
    data->st_RawYOLO = st_SensorData.st_RawYOLO;
    pthread_mutex_unlock(&st_SensorData.st_MutexYOLO);
}

// ---------------------------------------------------------------- 기존 RANSAC 계측 복제
// Ransac.cpp NormalVectorFiltering()과 한 줄씩 같은 계산을 하면서 "지면"으로
// 고른 평면만 기록한다. 결과 점군이 실제 파이프라인 출력과 같은지 매 프레임 검사한다.
struct BaselinePlaneInfo {
    bool ground_found = false;
    float a = 0.0f, b = 0.0f, c = 0.0f, d = 0.0f;
    int ground_inliers = 0;
    int wall_planes_before = 0;
};

void BaselineRansacReplica(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud3,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud4,
                           BaselinePlaneInfo* info) {
    *info = BaselinePlaneInfo();
    if (!pcl_Cloud3 || pcl_Cloud3->points.size() < 3) {
        pcl_Cloud4->clear();
        return;
    }
    pcl::ModelCoefficients::Ptr pcl_Coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr pcl_Inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr pcl_Wall_inliers(new pcl::PointIndices());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud3_4(new pcl::PointCloud<pcl::PointXYZI>(*pcl_Cloud3));
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_Cloud4_4(new pcl::PointCloud<pcl::PointXYZI>);
    int32 s32_Max_iterations = 3;
    int32 s32_Iteration = 0;
    bool ground_removed = false;
    while (s32_Iteration++ < s32_Max_iterations) {
        pcl::SACSegmentation<pcl::PointXYZI> pcl_Seg;
        pcl_Seg.setOptimizeCoefficients(true);
        pcl_Seg.setModelType(pcl::SACMODEL_PLANE);
        pcl_Seg.setMethodType(pcl::SAC_RANSAC);
        pcl_Seg.setMaxIterations(20);
        pcl_Seg.setDistanceThreshold(0.3);
        pcl_Seg.setInputCloud(pcl_Cloud3_4);
        pcl_Seg.segment(*pcl_Inliers, *pcl_Coefficients);
        if (pcl_Inliers->indices.empty()) break;
        const float nx = pcl_Coefficients->values[0];
        const float ny = pcl_Coefficients->values[1];
        const float nz = pcl_Coefficients->values[2];
        const float normal_norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        const float f32_Dot_product = normal_norm > 1e-6f ? nz / normal_norm : 0.0f;
        if (std::fabs(f32_Dot_product) >= 0.82f) {
            info->ground_found = true;
            info->a = pcl_Coefficients->values[0] / normal_norm;
            info->b = pcl_Coefficients->values[1] / normal_norm;
            info->c = pcl_Coefficients->values[2] / normal_norm;
            info->d = pcl_Coefficients->values[3] / normal_norm;
            if (info->c < 0.0f) {
                info->a = -info->a; info->b = -info->b; info->c = -info->c; info->d = -info->d;
            }
            info->ground_inliers = static_cast<int>(pcl_Inliers->indices.size());
            pcl::ExtractIndices<pcl::PointXYZI> pcl_Extract;
            pcl_Extract.setInputCloud(pcl_Cloud3_4);
            pcl_Extract.setIndices(pcl_Inliers);
            pcl_Extract.setNegative(true);
            pcl_Extract.filter(*pcl_Cloud3_4);
            for (const auto& pt : pcl_Cloud3_4->points) {
                if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                    pcl_Cloud4_4->points.push_back(pt);
                }
            }
            pcl_Cloud4_4->width = static_cast<uint32_t>(pcl_Cloud4_4->points.size());
            pcl_Cloud4_4->height = 1;
            pcl_Cloud4_4->is_dense = false;
            *pcl_Cloud4 = *pcl_Cloud4_4;
            ground_removed = true;
            break;
        } else {
            ++info->wall_planes_before;
            *pcl_Wall_inliers = *pcl_Inliers;
            pcl::ExtractIndices<pcl::PointXYZI> pcl_ExtractWall;
            pcl_ExtractWall.setInputCloud(pcl_Cloud3_4);
            pcl_ExtractWall.setIndices(pcl_Wall_inliers);
            pcl_ExtractWall.setNegative(true);
            pcl_ExtractWall.filter(*pcl_Cloud3_4);
            continue;
        }
    }
    if (!ground_removed) *pcl_Cloud4 = *pcl_Cloud3_4;
    pcl_Cloud4->width = static_cast<uint32_t>(pcl_Cloud4->points.size());
    pcl_Cloud4->height = 1;
    pcl_Cloud4->is_dense = false;
}

// ---------------------------------------------------------------- 독립 기준 지면(평가 전용)
struct RefPlane {
    bool ok = false;
    tunnel_filter::Plane plane;
    int inliers = 0;
    int candidates = 0;
};

RefPlane FitReferenceGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                            float z_min, float z_max, float max_range) {
    RefPlane ref;
    pcl::IndicesPtr candidates(new std::vector<int>());
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& p = cloud->points[i];
        if (!std::isfinite(p.z) || p.z < z_min || p.z > z_max) continue;
        if (p.x * p.x + p.y * p.y > max_range * max_range) continue;
        candidates->push_back(static_cast<int>(i));
    }
    ref.candidates = static_cast<int>(candidates->size());
    if (candidates->size() < 30) return ref;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(20.0 * M_PI / 180.0);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.10);
    seg.setInputCloud(cloud);
    seg.setIndices(candidates);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    seg.segment(inliers, coefficients);
    if (inliers.indices.size() < 30 || coefficients.values.size() < 4) return ref;
    float a = coefficients.values[0], b = coefficients.values[1];
    float c = coefficients.values[2], d = coefficients.values[3];
    const float norm = std::sqrt(a * a + b * b + c * c);
    if (!(norm > 1e-6f)) return ref;
    a /= norm; b /= norm; c /= norm; d /= norm;
    if (c < 0.0f) { a = -a; b = -b; c = -c; d = -d; }
    ref.ok = true;
    ref.plane.a = a; ref.plane.b = b; ref.plane.c = c; ref.plane.d = d;
    ref.inliers = static_cast<int>(inliers.indices.size());
    return ref;
}

// ---------------------------------------------------------------- JSON 출력 도우미
class Json {
public:
    Json& Raw(const std::string& text) { out_ << text; return *this; }
    Json& Key(const std::string& key) {
        Comma();
        out_ << '"' << key << "\":";
        need_comma_ = false;
        return *this;
    }
    Json& Num(double value, int precision = 4) {
        Comma();
        if (!std::isfinite(value)) {
            out_ << "null";
        } else {
            std::ostringstream s;
            s.setf(std::ios::fixed);
            s.precision(precision);
            s << value;
            std::string text = s.str();
            if (text.find('.') != std::string::npos) {
                while (!text.empty() && text.back() == '0') text.pop_back();
                if (!text.empty() && text.back() == '.') text.pop_back();
            }
            if (text == "-0") text = "0";
            out_ << text;
        }
        need_comma_ = true;
        return *this;
    }
    Json& Int(long long value) { Comma(); out_ << value; need_comma_ = true; return *this; }
    Json& Bool(bool value) { Comma(); out_ << (value ? "true" : "false"); need_comma_ = true; return *this; }
    Json& Str(const std::string& value) {
        Comma();
        out_ << '"';
        for (const char ch : value) {
            if (ch == '"' || ch == '\\') out_ << '\\';
            out_ << ch;
        }
        out_ << '"';
        need_comma_ = true;
        return *this;
    }
    Json& Open(char bracket) { Comma(); out_ << bracket; need_comma_ = false; return *this; }
    Json& Close(char bracket) { out_ << bracket; need_comma_ = true; return *this; }
    std::string str() const { return out_.str(); }

private:
    void Comma() { if (need_comma_) out_ << ','; }
    std::ostringstream out_;
    bool need_comma_ = false;
};

// ---------------------------------------------------------------- 평가 대상 (정적 차량/사람)
// 월드 좌표의 방향 박스: name:wx:wy:half_len:half_wid:yaw_deg:zmin_road:zmax_road
struct Target {
    std::string name;
    double wx = 0.0, wy = 0.0;
    float half_length = 1.0f;
    float half_width = 1.0f;
    double yaw = 0.0;  // world, rad
    float z_min_road = 0.2f;
    float z_max_road = 2.5f;
};

std::vector<Target> ParseTargets(const std::string& text) {
    std::vector<Target> targets;
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ';')) {
        if (item.empty()) continue;
        std::stringstream fields(item);
        std::string field;
        std::vector<std::string> parts;
        while (std::getline(fields, field, ':')) parts.push_back(field);
        if (parts.size() < 8) continue;
        Target t;
        t.name = parts[0];
        t.wx = std::stod(parts[1]);
        t.wy = std::stod(parts[2]);
        t.half_length = std::stof(parts[3]);
        t.half_width = std::stof(parts[4]);
        t.yaw = std::stod(parts[5]) * M_PI / 180.0;
        t.z_min_road = std::stof(parts[6]);
        t.z_max_road = std::stof(parts[7]);
        targets.push_back(t);
    }
    return targets;
}

struct EgoSample {
    double stamp;
    double x, y, heading_rad;
    double speed;
};

bool InterpolateEgo(const std::vector<EgoSample>& samples, double stamp, EgoSample* out) {
    if (samples.empty()) return false;
    if (stamp <= samples.front().stamp) { *out = samples.front(); return true; }
    if (stamp >= samples.back().stamp) { *out = samples.back(); return true; }
    auto upper = std::upper_bound(samples.begin(), samples.end(), stamp,
        [](double s, const EgoSample& e) { return s < e.stamp; });
    const EgoSample& b = *upper;
    const EgoSample& a = *(upper - 1);
    const double t = (stamp - a.stamp) / std::max(1e-9, b.stamp - a.stamp);
    double dh = b.heading_rad - a.heading_rad;
    while (dh > M_PI) dh -= 2.0 * M_PI;
    while (dh < -M_PI) dh += 2.0 * M_PI;
    out->stamp = stamp;
    out->x = a.x + t * (b.x - a.x);
    out->y = a.y + t * (b.y - a.y);
    out->heading_rad = a.heading_rad + t * dh;
    out->speed = a.speed + t * (b.speed - a.speed);
    return true;
}

struct SweepConfig {
    std::string name;
    tunnel_filter::Params params;
    bool protect = true;
    bool camera_fail = false;  // YOLO 동기 실패 가정: 보호점 없음 + FOV 안 fail-open
    std::unique_ptr<tunnel_filter::TunnelPipeline> pipeline;
};

// 대상 박스의 센서(수평화) 좌표 중심과 방향.
struct TargetLocal {
    float lx = 0.0f, ly = 0.0f;
    float cos_yaw = 1.0f, sin_yaw = 0.0f;
    bool valid = false;
    // 박스 안(+margin)이면 true.
    bool Contains(const Target& t, float x, float y, float margin) const {
        const float dx = x - lx;
        const float dy = y - ly;
        const float u = cos_yaw * dx + sin_yaw * dy;
        const float v = -sin_yaw * dx + cos_yaw * dy;
        return std::fabs(u) <= t.half_length + margin &&
               std::fabs(v) <= t.half_width + margin;
    }
};

void WriteObjectsB(Json& json, const tunnel_filter::FrameResult& r) {
    json.Open('[');
    for (const auto& o : r.objects) {
        json.Open('[');
        json.Num(o.obb.cx, 2).Num(o.obb.cy, 2).Num(o.obb.cz, 2)
            .Num(o.obb.length, 2).Num(o.obb.width, 2)
            .Num(o.obb.z_min, 2).Num(o.obb.z_max, 2)
            .Num(o.z_min_road, 2).Num(o.z_max_road, 2)
            .Num(o.obb.yaw, 3).Int(o.point_count).Int(o.protected_points)
            .Int(o.protect_class).Bool(o.is_protected)
            .Bool(o.flags.ceiling).Bool(o.flags.wall)
            .Bool(o.flags.ground_residual).Bool(o.flags.removed)
            .Bool(o.flags.wall_attached);
        json.Close(']');
    }
    json.Close(']');
}

}  // namespace

int main(int argc, char** argv) {
    // ---- 안전장치: 실시간 master에는 붙지 않는다
    const char* master = std::getenv("ROS_MASTER_URI");
    const std::string master_uri = master ? master : "";
    bool allow_master = false;
    std::string bag_path, out_dir;
    double processing_delay = 0.13;
    bool dump_clouds = false;
    std::string only_stamps_path;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--bag" && i + 1 < argc) bag_path = argv[++i];
        else if (arg == "--out" && i + 1 < argc) out_dir = argv[++i];
        else if (arg == "--processing-delay" && i + 1 < argc) processing_delay = std::stod(argv[++i]);
        else if (arg == "--dump-clouds") dump_clouds = true;
        else if (arg == "--only-stamps" && i + 1 < argc) only_stamps_path = argv[++i];
        else if (arg == "--allow-any-master") allow_master = true;
    }
    if (!allow_master && (master_uri.empty() ||
                          master_uri.find(":11311") != std::string::npos ||
                          master_uri.find(":11312") != std::string::npos)) {
        std::fprintf(stderr,
                     "Refusing to run: ROS_MASTER_URI='%s' must point to a private "
                     "master (not 11311 live / 11312 test).\n", master_uri.c_str());
        return 2;
    }
    if (bag_path.empty() || out_dir.empty()) {
        std::fprintf(stderr, "usage: tunnel_filter_offline_eval --bag BAG --out DIR "
                             "[--processing-delay 0.13] [--dump-clouds] "
                             "[--only-stamps FILE]\n");
        return 2;
    }

    ros::init(argc, argv, "tunnel_filter_offline_eval",
              ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool enable_fusion = true;
    pnh.param("enable_sensor_fusion", enable_fusion, true);

    InitMutex();
    InitVehicleSpec();
    InitLidarSettings(pnh);
    InitCameraSettings();
    auto motion_classifier = std::make_unique<MotionClassifier>(pnh);
    auto planner_publisher = std::make_unique<PlannerObjectRosPublisher>(nh);
    InitLidarViz(nh);
    if (enable_fusion) InitFusionLidarViz(nh);
    int image_width = 1280, image_height = 720;
    {
        YAML::Node config = YAML::LoadFile(YamlURL);
        image_width = config["Vehicle"]["Camera"]["ImageWidth"].as<int>(1280);
        image_height = config["Vehicle"]["Camera"]["ImageHeight"].as<int>(720);
    }
    const bool experiment = tunnel_filter::InitTunnelExperiment(
        nh, pnh, st_Camera, image_width, image_height, false);

    // sweep 설정: ~sweep_configs="a,b,c", 각 ~sweep/<name>/tunnel_* 가 기준값을 덮어쓴다.
    std::vector<SweepConfig> sweeps;
    {
        std::string names;
        pnh.param("sweep_configs", names, std::string());
        std::stringstream stream(names);
        std::string name;
        while (std::getline(stream, name, ',')) {
            if (name.empty()) continue;
            ros::NodeHandle sweep_nh(pnh, "sweep/" + name);
            SweepConfig config;
            config.name = name;
            config.params = tunnel_filter::LoadParams(
                sweep_nh, tunnel_filter::TunnelExperimentParams());
            sweep_nh.param("protect", config.protect, true);
            sweep_nh.param("camera_fail", config.camera_fail, false);
            config.pipeline = std::make_unique<tunnel_filter::TunnelPipeline>(
                config.params, tunnel_filter::MakeTrackingObbFunction());
            sweeps.push_back(std::move(config));
        }
    }
    std::string target_text;
    pnh.param("eval_targets", target_text, std::string());
    const std::vector<Target> targets = ParseTargets(target_text);

    // 실제 파이프라인 출력 캡처 (같은 프로세스, 비공개 master).
    sensor_msgs::PointCloud2::ConstPtr last_ransac;
    Msg::TrackedObjectArray::ConstPtr last_nogps;
    Msg::TrackedObjectArray::ConstPtr last_lastlidar;
    ros::Subscriber sub_ransac = nh.subscribe<sensor_msgs::PointCloud2>(
        "/lidar/debug_ransac", 5,
        [&last_ransac](const sensor_msgs::PointCloud2::ConstPtr& m) { last_ransac = m; });
    ros::Subscriber sub_nogps = nh.subscribe<Msg::TrackedObjectArray>(
        "/lidar_nogps", 5,
        [&last_nogps](const Msg::TrackedObjectArray::ConstPtr& m) { last_nogps = m; });
    ros::Subscriber sub_last = nh.subscribe<Msg::TrackedObjectArray>(
        "/last_lidar", 5,
        [&last_lastlidar](const Msg::TrackedObjectArray::ConstPtr& m) { last_lastlidar = m; });

    // ---- 1차 패스: Ego 궤적 (월드 좌표 대상 변환용)
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<EgoSample> ego_samples;
    {
        rosbag::View ego_view(bag, rosbag::TopicQuery(std::vector<std::string>{"/Ego_topic"}));
        for (const rosbag::MessageInstance& m : ego_view) {
            auto ego = m.instantiate<morai_msgs::EgoVehicleStatus>();
            if (!ego) continue;
            EgoSample s;
            s.stamp = ego->header.stamp.toSec();
            s.x = ego->position.x;
            s.y = ego->position.y;
            s.heading_rad = ego->heading * M_PI / 180.0;
            s.speed = std::hypot(ego->velocity.x, ego->velocity.y);
            ego_samples.push_back(s);
        }
    }

    std::ofstream frames_out(out_dir + "/frames.jsonl");
    std::ofstream clouds_out;
    if (dump_clouds) clouds_out.open(out_dir + "/clouds.bin", std::ios::binary);

    const std::vector<std::string> topics = {
        "/gps", "/imu", "/Ego_topic", "/YOLO/BBox", "/camera/image/compressed",
        "/velodyne_points"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::deque<std::pair<double, sensor_msgs::PointCloud2::ConstPtr>> lidar_queue;
    // --only-stamps: 실시간 노드가 실제로 처리한 스캔(ms stamp 목록)만 처리해
    // 노드의 프레임 건너뜀을 그대로 재현한다. 나머지 스캔은 수신 즉시 덮어써진
    // 것과 같으므로 두 경로 모두 건너뛴다.
    std::unordered_set<long long> only_stamps;
    if (!only_stamps_path.empty()) {
        std::ifstream in(only_stamps_path);
        long long value = 0;
        while (in >> value) only_stamps.insert(value);
        ROS_WARN("processing only %zu scans listed in %s", only_stamps.size(),
                 only_stamps_path.c_str());
    }
    int skipped_scans = 0;
    auto scheduled = [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (only_stamps.empty()) return true;
        const long long ms = static_cast<long long>(msg->header.stamp.toNSec() / 1000000ULL);
        if (only_stamps.count(ms)) return true;
        ++skipped_scans;
        return false;
    };
    int frame_index = 0;
    int replica_mismatch = 0;

    auto process_lidar = [&](const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, double record_time) {
        LiDARParser_Callback(cloud_msg);
        CopyData(&st_CurrentData);
        DeadReckoningProcess(st_CurrentData.st_RawGPS, st_CurrentData.st_RawIMU,
                             st_CurrentData.st_RawCAN, st_GPSIMU);
        const auto lidar_start = Clock::now();
        LidarProcess(st_CurrentData.st_RawLiDAR, st_GPSIMU, st_LiDAR, *motion_classifier);
        const double lidar_ms = ElapsedMs(lidar_start);
        const LidarStageProfile profile = g_LidarStageProfile;
        const auto fusion_start = Clock::now();
        if (enable_fusion) {
            FusionProcess(st_CurrentData.st_RawYOLO, st_LiDAR, st_Camera, ts_CurrTime);
        }
        const double fusion_ms = ElapsedMs(fusion_start);
        const auto publish_start = Clock::now();
        planner_publisher->Publish(st_LiDAR, st_GPSIMU, st_VehicleSpec, ts_CurrTime);
        const double publish_ms = ElapsedMs(publish_start);
        // 터널 필터는 LidarProcess 안에서 동기 실행된다. last frame에 기록된
        // 전체 처리 시간을 별도 지표로 유지한다.
        const double tunnel_ms = experiment &&
                tunnel_filter::TunnelExperimentOutputReady(ts_CurrTime)
            ? tunnel_filter::TunnelExperimentLastFrame().total_ms : 0.0;
        // 같은 프로세스 구독이라도 전달 시점이 늦을 수 있으므로, 이번 스캔의
        // /last_lidar(항상 발행)을 받을 때까지 최대 100 ms 콜백을 처리한다.
        // 파이프라인 결과에는 영향이 없고 측정 누락만 막는다.
        for (int spin = 0; spin < 40; ++spin) {
            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.005));
            if (last_lastlidar &&
                std::fabs(last_lastlidar->header.stamp.toSec() - ts_CurrTime.toSec()) < 1e-3 &&
                last_nogps &&
                std::fabs(last_nogps->header.stamp.toSec() - ts_CurrTime.toSec()) < 1e-3 &&
                last_ransac &&
                std::fabs(last_ransac->header.stamp.toSec() - ts_CurrTime.toSec()) < 1e-4) {
                break;
            }
        }

        const double scan_stamp = cloud_msg->header.stamp.toSec();
        const bool denied = tunnel_filter::IsGpsDenied(st_GPSIMU.st_GPS.b_isJamming,
                                                       st_GPSIMU.st_GPS.b_isTunnel);
        const float roll = st_GPSIMU.st_IMU.f32_Roll_rad;
        const float pitch = st_GPSIMU.st_IMU.f32_Pitch_rad;
        const auto level = tunnel_filter::LevelingRotation(roll, pitch, true, true);
        const auto pitch_only = tunnel_filter::LevelingRotation(roll, pitch, false, true);

        // 기존 경로 중간 결과 재계산 (같은 함수, 결정론적)
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw = MakePointCloudFromRaw(st_CurrentData.st_RawLiDAR);
        pcl::PointCloud<pcl::PointXYZI>::Ptr roi(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr corrected(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr replica_out(new pcl::PointCloud<pcl::PointXYZI>);
        Roi(raw, roi, st_LiDAR, st_VehicleSpec);
        Voxel(roi, voxel);
        CorrectingPoints(voxel, corrected, st_GPSIMU);
        BaselinePlaneInfo baseline_plane;
        BaselineRansacReplica(corrected, replica_out, &baseline_plane);
        // 1: 실제 파이프라인이 발행한 /lidar/debug_ransac과 복제 결과가 비트 단위로 같음,
        // 0: 다름, -1: 이번 스캔의 발행 메시지를 받지 못함(구독 연결 전).
        int replica_state = -1;
        pcl::PointCloud<pcl::PointXYZI> published;
        if (last_ransac &&
            std::fabs(last_ransac->header.stamp.toSec() - ts_CurrTime.toSec()) < 1e-4) {
            pcl::fromROSMsg(*last_ransac, published);
            bool equal = published.points.size() == replica_out->points.size();
            for (std::size_t i = 0; equal && i < published.points.size(); ++i) {
                equal = published.points[i].x == replica_out->points[i].x &&
                        published.points[i].y == replica_out->points[i].y &&
                        published.points[i].z == replica_out->points[i].z;
            }
            replica_state = equal ? 1 : 0;
            if (!equal) ++replica_mismatch;
        }

        // 기존 RANSAC 출력(pitch 보정 좌표)을 roll+pitch 수평화 좌표로 옮긴다: L * Pp^T
        pcl::PointCloud<pcl::PointXYZI> a_level;
        a_level.points.reserve(replica_out->points.size());
        for (const auto& p : replica_out->points) {
            const float sx = pitch_only[0] * p.x + pitch_only[3] * p.y + pitch_only[6] * p.z;
            const float sy = pitch_only[1] * p.x + pitch_only[4] * p.y + pitch_only[7] * p.z;
            const float sz = pitch_only[2] * p.x + pitch_only[5] * p.y + pitch_only[8] * p.z;
            pcl::PointXYZI q;
            q.x = level[0] * sx + level[1] * sy + level[2] * sz;
            q.y = level[3] * sx + level[4] * sy + level[5] * sz;
            q.z = level[6] * sx + level[7] * sy + level[8] * sz;
            q.intensity = p.intensity;
            a_level.points.push_back(q);
        }

        // 독립 기준 지면 + IMU 보정 전후 비교 (평가 전용)
        pcl::PointCloud<pcl::PointXYZI>::Ptr leveled_input(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pitch_input(new pcl::PointCloud<pcl::PointXYZI>);
        tunnel_filter::ApplyRotation(*voxel, level, leveled_input.get());
        tunnel_filter::ApplyRotation(*voxel, pitch_only, pitch_input.get());
        const RefPlane ref = FitReferenceGround(leveled_input, -2.3f, -1.1f, 30.0f);
        const RefPlane raw_fit = FitReferenceGround(voxel, -2.3f, -1.1f, 30.0f);
        const RefPlane pitch_fit = FitReferenceGround(pitch_input, -2.3f, -1.1f, 30.0f);

        EgoSample ego;
        const bool has_ego = InterpolateEgo(ego_samples, scan_stamp, &ego);

        Json json;
        json.Open('{');
        json.Key("i").Int(frame_index);
        json.Key("stamp").Num(scan_stamp, 4);
        json.Key("rec").Num(record_time, 4);
        json.Key("gps_invalid").Bool(st_GPSIMU.st_GPS.b_isJamming);
        json.Key("gps_tunnel_latched").Bool(st_GPSIMU.st_GPS.b_isTunnel);
        json.Key("denied").Bool(denied);
        json.Key("roll").Num(roll, 5).Key("pitch").Num(pitch, 5);
        if (has_ego) {
            json.Key("ego").Open('[').Num(ego.x, 3).Num(ego.y, 3)
                .Num(ego.heading_rad, 5).Num(ego.speed, 3).Close(']');
        }
        // --- 기준 지면 / IMU 비교
        auto write_ref = [&json](const std::string& key, const RefPlane& r) {
            json.Key(key).Open('[').Bool(r.ok).Num(r.plane.a, 5).Num(r.plane.b, 5)
                .Num(r.plane.c, 5).Num(r.plane.d, 5)
                .Num(r.ok ? r.plane.HeightAt(0, 0) : NAN, 4)
                .Num(r.ok ? r.plane.TiltDeg() : NAN, 4).Int(r.inliers)
                .Int(r.candidates).Close(']');
        };
        write_ref("ref", ref);
        write_ref("ref_raw", raw_fit);
        write_ref("ref_pitch", pitch_fit);

        // --- A: 기존 파이프라인
        json.Key("A").Open('{');
        json.Key("raw").Int(profile.raw_points).Key("roi").Int(profile.roi_points)
            .Key("voxel").Int(profile.voxel_points).Key("ransac").Int(profile.ransac_points)
            .Key("clusters").Int(profile.clusters);
        json.Key("replica").Int(replica_state);
        json.Key("plane").Open('[').Bool(baseline_plane.ground_found)
            .Num(baseline_plane.a, 5).Num(baseline_plane.b, 5).Num(baseline_plane.c, 5)
            .Num(baseline_plane.d, 5)
            .Num(baseline_plane.ground_found && std::fabs(baseline_plane.c) > 1e-6f
                     ? -baseline_plane.d / baseline_plane.c : NAN, 4)
            .Int(baseline_plane.ground_inliers).Int(baseline_plane.wall_planes_before)
            .Close(']');
        json.Key("ms").Open('[').Num(lidar_ms, 3).Num(fusion_ms, 3).Num(publish_ms, 3)
            .Num(profile.roi_ms, 3).Num(profile.voxel_ms, 3).Num(profile.imu_correct_ms, 3)
            .Num(profile.ransac_ms, 3).Num(profile.cluster_ms, 3)
            .Num(profile.objectinfo_ms, 3).Num(profile.framefilter_ms, 3).Close(']');
        json.Key("objs").Open('[');
        for (const auto& o : st_LiDAR.vec_Objects) {
            if (o.s32_MissCount != 0 || o.s32_Id < 0) continue;
            if (o.f32_CoreX == 0.0f && o.f32_CoreY == 0.0f && o.f32_CoreZ == 0.0f) continue;
            json.Open('[');
            json.Int(o.s32_Id).Str(o.str_ObjectClass).Str(o.str_YoloClass)
                .Num(o.f32_OBB_CenterX, 2).Num(o.f32_OBB_CenterY, 2).Num(o.f32_OBB_CenterZ, 2)
                .Num(std::max(o.f32_OBB_Width, o.f32_OBB_Height), 2)
                .Num(std::min(o.f32_OBB_Width, o.f32_OBB_Height), 2)
                .Num(o.f32_OBB_ZMin, 2).Num(o.f32_OBB_ZMax, 2).Num(o.f32_OBB_YawRad, 3)
                .Int(static_cast<long long>(o.pcl_ClusterPoints.size()));
            json.Close(']');
        }
        json.Close(']');
        json.Key("nogps").Open('[');
        if (last_nogps && std::fabs(last_nogps->header.stamp.toSec() - ts_CurrTime.toSec()) < 1e-3) {
            for (const auto& o : last_nogps->objects) {
                json.Open('[').Int(o.id).Str(o.classification)
                    .Num(o.obb_center.x, 2).Num(o.obb_center.y, 2).Num(o.obb_center.z, 2)
                    .Num(std::max(o.length, o.width), 2).Num(std::min(o.length, o.width), 2)
                    .Num(o.height, 2).Close(']');
            }
        }
        json.Close(']');
        json.Key("last_lidar").Int(last_lastlidar &&
            std::fabs(last_lastlidar->header.stamp.toSec() - ts_CurrTime.toSec()) < 1e-3
            ? static_cast<long long>(last_lastlidar->objects.size()) : -1);
        json.Close('}');

        // --- B: 실험 파이프라인 (주 설정)
        const tunnel_filter::ExperimentFrame& frame = tunnel_filter::TunnelExperimentLastFrame();
        const bool b_active = experiment && frame.active && frame.result != nullptr &&
                              std::fabs(frame.stamp.toSec() - ts_CurrTime.toSec()) < 1e-3;
        json.Key("B").Open('{');
        json.Key("active").Bool(b_active).Key("tunnel_ms").Num(tunnel_ms, 3);
        if (b_active) {
            const tunnel_filter::FrameResult& r = *frame.result;
            json.Key("ground").Open('[').Int(static_cast<int>(r.ground.source))
                .Str(r.ground_fit.reject_reason).Num(r.ground.plane.a, 5)
                .Num(r.ground.plane.b, 5).Num(r.ground.plane.c, 5).Num(r.ground.plane.d, 5)
                .Num(r.ground_reference_height, 4).Num(r.ground.plane.TiltDeg(), 4)
                .Int(r.ground_fit.candidates).Int(r.ground_fit.inliers)
                .Num(r.ground_fit.inlier_ratio, 4).Int(r.ground.consecutive_failures)
                .Bool(r.ground_fallback_used)
                .Num(r.ground_fit.found ? r.ground_fit.height_at_origin : NAN, 4)
                .Int(r.ground_fit.refined_candidates).Int(r.ground_fit.prior_source)
                .Num(r.ground_fit.prior_height, 4)
                .Bool(r.ground_fit.refit_used).Int(r.ground_fit.refit_candidates)
                .Num(r.ground_fit.refit_angle_deg, 3).Num(r.ground_fit.refit_height_delta_m, 4)
                .Close(']');
            json.Key("counts").Open('[').Int(r.input_points).Int(r.ceiling_removed)
                .Int(r.ground_removed).Int(r.wall_removed).Int(r.kept_points)
                .Int(static_cast<long long>(r.clusters.size())).Int(r.protected_points)
                .Close(']');
            json.Key("motion").Open('[').Bool(r.motion.valid).Num(r.motion.dt_s, 4)
                .Num(r.motion.dyaw_rad * 57.29578f, 3).Num(r.motion.dx_m, 3)
                .Num(r.motion.dy_m, 3).Close(']');
            json.Key("protect").Open('[').Bool(frame.protection_available)
                .Num(frame.yolo_offset_ms, 2).Int(frame.yolo_boxes).Int(frame.protect_boxes)
                .Close(']');
            json.Key("walls").Open('[');
            for (const auto& w : r.walls) {
                json.Open('{');
                json.Key("cand").Int(w.candidates).Key("fit").Bool(w.fit_valid)
                    .Key("rm").Bool(w.removal_allowed).Key("persist").Int(w.track.persist)
                    .Key("reason").Str(w.first_attempt.reject_reason)
                    .Key("removed").Int(w.removed).Key("prot_skip").Int(w.protected_skipped)
                    .Key("amb_skip").Int(w.ambiguous_skipped);
                json.Key("first").Open('[').Num(w.first_attempt.lateral_distance, 3)
                    .Num(w.first_attempt.tilt_deg, 2).Num(w.first_attempt.span, 2)
                    .Int(w.first_attempt.inliers).Num(w.first_attempt.inlier_ratio, 3)
                    .Num(w.first_attempt.vertical_span, 2)
                    .Num(w.first_attempt.beyond_fraction, 3)
                    .Int(w.first_attempt.occupied_bins).Close(']');
                json.Key("segs").Open('[');
                for (const auto& s : w.segments) {
                    json.Open('[').Num(s.plane.a, 5).Num(s.plane.b, 5).Num(s.plane.c, 5)
                        .Num(s.plane.d, 4).Num(s.tangent_x, 5).Num(s.tangent_y, 5)
                        .Num(s.s_min, 2).Num(s.s_max, 2).Int(s.inliers)
                        .Num(s.inlier_ratio, 3).Num(s.vertical_span, 2)
                        .Num(s.beyond_fraction, 3).Num(s.tilt_deg, 2).Bool(s.step)
                        .Int(static_cast<long long>(s.intervals.size())).Num(s.span, 2)
                        .Bool(s.continuation).Close(']');
                }
                json.Close(']');
                // att: [a, b, c, d, tilt, vspan, beyond, span, inliers, pool_support, reason]
                json.Key("att").Open('[');
                for (const auto& s : w.attempts) {
                    json.Open('[').Num(s.plane.a, 5).Num(s.plane.b, 5).Num(s.plane.c, 5)
                        .Num(s.plane.d, 4).Num(s.tilt_deg, 2).Num(s.vertical_span, 2)
                        .Num(s.beyond_fraction, 3).Num(s.span, 2).Int(s.inliers)
                        .Int(s.pool_support).Str(s.reject_reason).Close(']');
                }
                json.Close(']');
                json.Close('}');
            }
            json.Close(']');
            json.Key("objs");
            WriteObjectsB(json, r);
            json.Key("t").Open('[').Num(r.timings.level_ms, 3).Num(r.timings.ceiling_ms, 3)
                .Num(r.timings.ground_fit_ms, 3).Num(r.timings.ground_remove_ms, 3)
                .Num(r.timings.wall_fit_ms, 3).Num(r.timings.wall_remove_ms, 3)
                .Num(r.timings.cluster_ms, 3).Num(r.timings.obb_ms, 3)
                .Num(r.timings.obb_filter_ms, 3).Num(r.timings.total_ms, 3)
                .Num(frame.protection_ms, 3).Num(frame.publish_ms, 3).Num(frame.total_ms, 3)
                .Close(']');
        }
        json.Close('}');

        // --- 평가 대상(정적 차량/사람) 주변 점/객체 (월드 -> 센서 좌표)
        std::vector<TargetLocal> local(targets.size());
        for (std::size_t k = 0; k < targets.size() && has_ego; ++k) {
            const double dx = targets[k].wx - ego.x;
            const double dy = targets[k].wy - ego.y;
            const double c = std::cos(ego.heading_rad);
            const double s = std::sin(ego.heading_rad);
            local[k].lx = static_cast<float>(c * dx + s * dy - st_VehicleSpec.st_LiDARPos.f32_X);
            local[k].ly = static_cast<float>(-s * dx + c * dy - st_VehicleSpec.st_LiDARPos.f32_Y);
            const double local_yaw = targets[k].yaw - ego.heading_rad;
            local[k].cos_yaw = static_cast<float>(std::cos(local_yaw));
            local[k].sin_yaw = static_cast<float>(std::sin(local_yaw));
            local[k].valid = true;
        }
        const auto in_target = [&](const pcl::PointXYZI& p, std::size_t k) {
            if (!local[k].Contains(targets[k], p.x, p.y, 0.0f)) return false;
            const float ground = ref.ok ? ref.plane.HeightAt(p.x, p.y) : -1.7f;
            const float z_road = p.z - ground;
            return z_road >= targets[k].z_min_road && z_road <= targets[k].z_max_road;
        };
        auto count_points = [&](const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                const std::vector<int>* indices, std::size_t k) {
            int count = 0;
            if (indices) {
                for (const int i : *indices) count += in_target(cloud.points[static_cast<std::size_t>(i)], k);
            } else {
                for (const auto& p : cloud.points) count += in_target(p, k);
            }
            return count;
        };
        json.Key("targets").Open('{');
        for (std::size_t k = 0; k < targets.size(); ++k) {
            json.Key(targets[k].name).Open('{');
            json.Key("local").Open('[').Num(local[k].lx, 2).Num(local[k].ly, 2).Close(']');
            if (local[k].valid) {
                json.Key("A_pts").Int(count_points(a_level, nullptr, k));
                // 기존 추적 객체 중 점 3개 이상이 대상 박스 안에 있는 것
                json.Key("A_obj").Open('[');
                for (const auto& o : st_LiDAR.vec_Objects) {
                    if (o.s32_MissCount != 0 || o.s32_Id < 0) continue;
                    int inside = 0;
                    for (const auto& p : o.pcl_ClusterPoints.points) inside += in_target(p, k);
                    if (inside >= 3) {
                        json.Open('[').Int(o.s32_Id).Str(o.str_ObjectClass)
                            .Num(std::max(o.f32_OBB_Width, o.f32_OBB_Height), 2)
                            .Num(o.f32_OBB_CenterX, 2).Num(o.f32_OBB_CenterY, 2)
                            .Int(static_cast<long long>(o.pcl_ClusterPoints.size()))
                            .Int(inside).Close(']');
                    }
                }
                json.Close(']');
                if (b_active) {
                    const auto& r = *frame.result;
                    json.Key("B_pts").Int(count_points(*r.leveled, &r.kept_indices, k));
                    json.Key("B_obj").Open('[');
                    for (std::size_t oi = 0; oi < r.objects.size(); ++oi) {
                        const auto& o = r.objects[oi];
                        int inside = 0;
                        for (const int index : r.clusters[oi].indices) {
                            inside += in_target(r.leveled->points[static_cast<std::size_t>(index)], k);
                        }
                        if (inside >= 3) {
                            json.Open('[').Int(static_cast<long long>(oi)).Num(o.obb.length, 2)
                                .Num(o.obb.cx, 2).Num(o.obb.cy, 2).Int(o.point_count)
                                .Bool(o.is_protected).Bool(o.flags.removed).Int(inside)
                                .Num(o.obb.width, 2).Close(']');
                        }
                    }
                    json.Close(']');
                }
            }
            json.Close('}');
        }
        json.Close('}');

        // --- sweep 설정 (같은 입력/보호 정보, 독립 상태)
        if (!sweeps.empty()) {
            json.Key("sweep").Open('{');
            for (auto& config : sweeps) {
                if (!b_active) {
                    if (!denied) config.pipeline->Reset();
                    continue;
                }
                const auto sweep_start = Clock::now();
                const bool use_protect = config.protect && !config.camera_fail;
                const tunnel_filter::FrameResult& r = config.pipeline->Process(
                    *frame.input, frame.roll_rad, frame.pitch_rad,
                    use_protect ? &frame.protect_class : nullptr,
                    config.camera_fail || config.protect ? &frame.ambiguous : nullptr,
                    config.camera_fail ? false
                                       : (config.protect ? frame.protection_available : true),
                    frame.result != nullptr ? &frame.result->motion : nullptr);
                const double sweep_ms = ElapsedMs(sweep_start);
                json.Key(config.name).Open('{');
                json.Key("g").Open('[').Int(static_cast<int>(r.ground.source))
                    .Num(r.ground_reference_height, 4).Num(r.ground.plane.TiltDeg(), 3)
                    .Bool(r.ground_fallback_used).Close(']');
                json.Key("c").Open('[').Int(r.input_points).Int(r.ceiling_removed)
                    .Int(r.ground_removed).Int(r.wall_removed).Int(r.kept_points)
                    .Int(static_cast<long long>(r.clusters.size())).Close(']');
                json.Key("w").Open('[').Bool(r.walls[0].fit_valid).Bool(r.walls[0].removal_allowed)
                    .Bool(r.walls[1].fit_valid).Bool(r.walls[1].removal_allowed).Close(']');
                // 기준 지면 띠 안 잔여점, 천장 임계 위 잔여점
                int resid = 0, above = 0;
                for (const int i : r.kept_indices) {
                    const auto& p = r.leveled->points[static_cast<std::size_t>(i)];
                    if (ref.ok && std::hypot(p.x, p.y) <= 30.0f &&
                        std::fabs(ref.plane.SignedDistance(p.x, p.y, p.z)) <= 0.15f) ++resid;
                    if (p.z > config.params.ceiling_max_z_lidar_m) ++above;
                }
                json.Key("q").Open('[').Int(resid).Int(above).Num(sweep_ms, 3).Close(']');
                json.Key("objs");
                WriteObjectsB(json, r);
                json.Key("tg").Open('{');
                for (std::size_t k = 0; k < targets.size(); ++k) {
                    if (!local[k].valid) continue;
                    int present = 0;
                    int removed_present = 0;
                    int best_inside = 0;
                    float best_length = 0.0f;
                    for (std::size_t oi = 0; oi < r.objects.size(); ++oi) {
                        const auto& o = r.objects[oi];
                        int inside = 0;
                        for (const int index : r.clusters[oi].indices) {
                            inside += in_target(r.leveled->points[static_cast<std::size_t>(index)], k);
                        }
                        if (inside < 3) continue;
                        if (o.flags.removed) { ++removed_present; continue; }
                        ++present;
                        if (inside > best_inside) {
                            best_inside = inside;
                            best_length = o.obb.length;
                        }
                    }
                    json.Key(targets[k].name).Open('[')
                        .Int(count_points(*r.leveled, &r.kept_indices, k))
                        .Int(present).Int(best_inside).Num(best_length, 2)
                        .Int(removed_present).Close(']');
                }
                json.Close('}');
                json.Close('}');
            }
            json.Close('}');
        }
        json.Close('}');
        frames_out << json.str() << '\n';

        // --- 점군 덤프 (대표 프레임 이미지용)
        if (dump_clouds) {
            const std::int32_t index = frame_index;
            const double stamp = scan_stamp;
            const std::int32_t n_a = static_cast<std::int32_t>(a_level.points.size());
            const pcl::PointCloud<pcl::PointXYZI>& b_cloud =
                b_active ? *frame.result->leveled : *leveled_input;
            const std::int32_t n_b = static_cast<std::int32_t>(b_cloud.points.size());
            clouds_out.write(reinterpret_cast<const char*>(&index), sizeof(index));
            clouds_out.write(reinterpret_cast<const char*>(&stamp), sizeof(stamp));
            clouds_out.write(reinterpret_cast<const char*>(&n_a), sizeof(n_a));
            clouds_out.write(reinterpret_cast<const char*>(&n_b), sizeof(n_b));
            for (const auto& p : a_level.points) {
                const float xyz[3] = {p.x, p.y, p.z};
                clouds_out.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
            }
            for (const auto& p : b_cloud.points) {
                const float xyz[3] = {p.x, p.y, p.z};
                clouds_out.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
            }
            for (std::int32_t i = 0; i < n_b; ++i) {
                const std::uint8_t label = b_active ? frame.result->labels[static_cast<std::size_t>(i)] : 255;
                const std::int16_t cluster = b_active
                    ? static_cast<std::int16_t>(frame.result->cluster_id[static_cast<std::size_t>(i)]) : -1;
                const std::uint8_t protect = b_active && !frame.protect_class.empty()
                    ? frame.protect_class[static_cast<std::size_t>(i)] : 0;
                clouds_out.write(reinterpret_cast<const char*>(&label), 1);
                clouds_out.write(reinterpret_cast<const char*>(&cluster), 2);
                clouds_out.write(reinterpret_cast<const char*>(&protect), 1);
            }
        }
        ++frame_index;
    };

    const auto run_start = Clock::now();
    for (const rosbag::MessageInstance& m : view) {
        const double record_time = m.getTime().toSec();
        // 처리 지연이 지난 스캔부터 처리한다 (그 사이의 GPS/IMU/YOLO가 먼저 반영됨).
        while (!lidar_queue.empty() &&
               lidar_queue.front().second->header.stamp.toSec() + processing_delay <= record_time) {
            auto item = lidar_queue.front();
            lidar_queue.pop_front();
            if (scheduled(item.second)) process_lidar(item.second, item.first);
        }
        const std::string& topic = m.getTopic();
        if (topic == "/gps") {
            auto msg = m.instantiate<morai_msgs::GPSMessage>();
            if (msg) GPSParser_Callback(msg);
        } else if (topic == "/imu") {
            auto msg = m.instantiate<sensor_msgs::Imu>();
            if (msg) IMUParser_Callback(msg);
        } else if (topic == "/Ego_topic") {
            auto msg = m.instantiate<morai_msgs::EgoVehicleStatus>();
            if (msg) CANParser_Callback(msg);
        } else if (topic == "/YOLO/BBox") {
            auto msg = m.instantiate<Msg::Yolo_bbox_arr>();
            if (msg) {
                YoloCallback(msg);
                if (experiment) {
                    std::vector<BBox> boxes;
                    for (const auto& d : msg->yolo_bbox_arr) {
                        BBox box;
                        box.st_Min_pt.f32_X = d.xMin;
                        box.st_Min_pt.f32_Y = d.yMin;
                        box.st_Max_pt.f32_X = d.xMax;
                        box.st_Max_pt.f32_Y = d.yMax;
                        box.str_class = d.classes;
                        box.str_raw_class = d.classes;
                        boxes.push_back(box);
                    }
                    tunnel_filter::TunnelExperimentIngestYolo(msg->header.stamp, boxes);
                }
            }
        } else if (topic == "/camera/image/compressed") {
            auto msg = m.instantiate<sensor_msgs::CompressedImage>();
            if (msg && !msg->header.stamp.isZero()) {
                BufferFusionCameraFrame(msg->header.stamp, msg->data);
            }
        } else if (topic == "/velodyne_points") {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg) lidar_queue.emplace_back(record_time, msg);
        }
    }
    while (!lidar_queue.empty()) {
        auto item = lidar_queue.front();
        lidar_queue.pop_front();
        if (scheduled(item.second)) process_lidar(item.second, item.first);
    }
    bag.close();
    const double run_ms = ElapsedMs(run_start);

    std::ofstream summary(out_dir + "/summary.json");
    summary << "{\"frames\":" << frame_index << ",\"replica_mismatch\":" << replica_mismatch
            << ",\"experiment_enabled\":" << (experiment ? "true" : "false")
            << ",\"sweeps\":" << sweeps.size() << ",\"run_ms\":" << run_ms
            << ",\"processing_delay_s\":" << processing_delay
            << ",\"only_stamps\":" << only_stamps.size()
            << ",\"skipped_scans\":" << skipped_scans << "}\n";
    std::printf("processed %d frames (replica mismatch %d, experiment=%d, sweeps=%zu) in %.1f s\n",
                frame_index, replica_mismatch, experiment ? 1 : 0, sweeps.size(), run_ms / 1000.0);
    return 0;
}
