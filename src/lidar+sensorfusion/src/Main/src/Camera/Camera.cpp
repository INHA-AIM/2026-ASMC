#include "Camera.hpp"
#include <Eigen/Geometry>


UDPReceiver::UDPReceiver(UdpConfig &st_UdpConfig)
 {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        cerr << "Socket creation failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Let the receiver periodically observe stop_flag_ even when no camera
    // packets arrive. Without this timeout, Ctrl+C hangs in pthread_join().
    struct timeval receive_timeout = {};
    receive_timeout.tv_usec = 100000;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO,
                   &receive_timeout, sizeof(receive_timeout)) < 0) {
        cerr << "Failed to set UDP receive timeout: " << strerror(errno) << endl;
        exit(EXIT_FAILURE);
    }

    // A 1280x720 JPEG arrives as several near-65 KB UDP datagrams in a burst.
    // The Linux default (about 212 KB) is too small when WSL scheduling is
    // delayed, so request a larger queue. Linux may cap this at rmem_max, but
    // even the capped/doubled value is safer than the default.
    int receive_buffer_size = 4 * 1024 * 1024;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF,
                   &receive_buffer_size, sizeof(receive_buffer_size)) < 0) {
        cerr << "Failed to enlarge UDP receive buffer: " << strerror(errno) << endl;
    }

    server_addr_ = {};
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(st_UdpConfig.s32_Port);
    server_addr_.sin_addr.s_addr = inet_addr(st_UdpConfig.str_Ip.c_str());

    if (bind(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        cerr << "Bind failed : " << strerror(errno) << endl;
        exit(EXIT_FAILURE);
    }

    if (pthread_mutex_init(&image_mutex_, nullptr) != 0) {
        cerr << "Mutex init failed" << endl;
        exit(EXIT_FAILURE);
    }

    cout << "[INFO] UDP Receiver listening on " << st_UdpConfig.str_Ip << " : " << st_UdpConfig.s32_Port << endl;
}

UDPReceiver::~UDPReceiver() {
    stop();
    close(socket_fd_);
    pthread_mutex_destroy(&image_mutex_);
}

void UDPReceiver::start() {
    stop_flag_.store(false);
    if (pthread_create(&receiver_thread_, nullptr, &UDPReceiver::receiver_thread_function, this) != 0) {
        cerr << "Failed to create pthread" << endl;
        exit(EXIT_FAILURE);
    }
    thread_started_.store(true);
}

void UDPReceiver::stop() {
    if (!thread_started_.exchange(false)) {
        return;
    }
    stop_flag_.store(true);
    pthread_join(receiver_thread_, nullptr);
}

void* UDPReceiver::receiver_thread_function(void* context) {
    ((UDPReceiver*)context)->receive_loop();
    return nullptr;
}

boost::optional<vector<uint8_t>> UDPReceiver::get_image_data() {
    pthread_mutex_lock(&image_mutex_);

    if (completed_image_data_.empty()) {
        pthread_mutex_unlock(&image_mutex_);
        return boost::none;
    }

    // Consume each completed frame exactly once. Previously the same JPEG was
    // returned at every 30 Hz polling tick, causing duplicate ROS/YOLO frames.
    vector<uint8_t> data_copy = std::move(completed_image_data_);
    completed_image_data_.clear();
    pthread_mutex_unlock(&image_mutex_);
    return data_copy;
}

void UDPReceiver::receive_loop() {

    ImagePacket st_Packet{};
    vector<uint8_t> image_chunk_buffer;
    int32 last_chunk_index = 0;
    bool assembling_frame = false;

    while (!stop_flag_.load()) {

        int n = recvfrom(socket_fd_, (char*)&st_Packet, sizeof(ImagePacket), 0, NULL, NULL);

        if (n == static_cast<int>(sizeof(ImagePacket))) {
            if (strncmp(st_Packet.c8_Header, "MOR", 3) == 0) {

                // ImageChunk has trailing alignment padding on this platform,
                // while u8_Data contains only the packed wire payload.
                ImageChunk st_Image{};
                memcpy(&st_Image, st_Packet.u8_Data, sizeof(st_Packet.u8_Data));

                if (st_Image.s32_Size <= 0 ||
                    st_Image.s32_Size > c_CAMERA_DATA_SIZE) {
                    image_chunk_buffer.clear();
                    assembling_frame = false;
                    ROS_WARN_THROTTLE(2.0, "Discarding camera packet with invalid JPEG chunk size: %d",
                                      st_Image.s32_Size);
                    continue;
                }

                const bool starts_jpeg =
                    st_Image.s32_Size >= 2 &&
                    st_Image.u8_JpegData[0] == 0xff &&
                    st_Image.u8_JpegData[1] == 0xd8;

                // A JPEG SOI always starts a new frame. This also recovers when
                // the previous frame's final UDP datagram was lost.
                if (starts_jpeg) {
                    image_chunk_buffer.clear();
                    assembling_frame = true;
                    last_chunk_index = st_Image.s32_Index - 1;
                }

                if (!assembling_frame) {
                    continue;
                }

                // Never publish a JPEG with a missing or reordered middle
                // datagram. libjpeg otherwise displays the missing rows black.
                if (st_Image.s32_Index != last_chunk_index + 1) {
                    image_chunk_buffer.clear();
                    assembling_frame = false;
                    ROS_WARN_THROTTLE(2.0,
                                      "Dropped incomplete camera frame: expected chunk %d, received %d",
                                      last_chunk_index + 1, st_Image.s32_Index);
                    continue;
                }

                image_chunk_buffer.insert(image_chunk_buffer.end(),
                    st_Image.u8_JpegData,
                    st_Image.u8_JpegData + st_Image.s32_Size
                );
                last_chunk_index = st_Image.s32_Index;

                if (strncmp(st_Image.c8_Tail, "EI", 2) == 0) {
                    const bool valid_jpeg =
                        image_chunk_buffer.size() >= 4 &&
                        image_chunk_buffer[0] == 0xff &&
                        image_chunk_buffer[1] == 0xd8 &&
                        image_chunk_buffer[image_chunk_buffer.size() - 2] == 0xff &&
                        image_chunk_buffer[image_chunk_buffer.size() - 1] == 0xd9;

                    if (valid_jpeg && image_chunk_buffer.size() <= c_CAMERA_JPEG_SIZE) {
                        pthread_mutex_lock(&image_mutex_);
                        completed_image_data_ = std::move(image_chunk_buffer);
                        pthread_mutex_unlock(&image_mutex_);
                    } else {
                        ROS_WARN_THROTTLE(2.0,
                                          "Discarding invalid/oversized camera JPEG (%zu bytes)",
                                          image_chunk_buffer.size());
                    }
                    image_chunk_buffer.clear();
                    assembling_frame = false;
                }
            }
        } else if (n > 0) {
            ROS_WARN_THROTTLE(2.0, "Discarding camera UDP datagram with unexpected size: %d", n);
        }
    }
}

namespace
{
Eigen::Matrix3d RotationVehicleFromSensor(double roll_deg,
                                          double pitch_deg,
                                          double yaw_deg)
{
    const double roll = roll_deg * M_PI / 180.0;
    const double pitch = pitch_deg * M_PI / 180.0;
    const double yaw = yaw_deg * M_PI / 180.0;
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Matrix4d VehicleFromSensor(double x, double y, double z,
                                  double roll_deg, double pitch_deg,
                                  double yaw_deg)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) =
        RotationVehicleFromSensor(roll_deg, pitch_deg, yaw_deg);
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    return transform;
}
}  // namespace

void InitProjectionMatrix(CAMERA &st_Camera, const YAML::Node& config)
{
    const YAML::Node camera = config["Vehicle"]["Camera"];
    const YAML::Node lidar = config["Vehicle"]["LiDAR"];

    double image_width = camera["ImageWidth"].as<double>();
    double image_height = camera["ImageHeight"].as<double>();
    double horizontal_fov_deg = camera["HorizontalFOV"].as<double>();
    double fx = camera["fx"].as<double>();
    double fy = camera["fy"].as<double>();
    double cx = camera["cx"].as<double>();
    double cy = camera["cy"].as<double>();
    double camera_x = camera["x"].as<double>();
    double camera_y = camera["y"].as<double>();
    double camera_z = camera["z"].as<double>();
    double camera_roll_deg = camera["roll"].as<double>();
    double camera_pitch_deg = camera["pitch"].as<double>();
    double camera_yaw_deg = camera["yaw"].as<double>();
    double lidar_x = lidar["x"].as<double>();
    double lidar_y = lidar["y"].as<double>();
    double lidar_z = lidar["z"].as<double>();
    double lidar_roll_deg = lidar["roll"].as<double>();
    double lidar_pitch_deg = lidar["pitch"].as<double>();
    double lidar_yaw_deg = lidar["yaw"].as<double>();

    // setting.yaml is the source of truth; private ROS params are optional
    // fine-calibration overrides.
    ros::param::param("~fusion_image_width", image_width, image_width);
    ros::param::param("~fusion_image_height", image_height, image_height);
    ros::param::param("~fusion_horizontal_fov_deg", horizontal_fov_deg,
                      horizontal_fov_deg);
    ros::param::param("~fusion_fx", fx, fx);
    ros::param::param("~fusion_fy", fy, fy);
    ros::param::param("~fusion_cx", cx, cx);
    ros::param::param("~fusion_cy", cy, cy);
    ros::param::param("~fusion_camera_x", camera_x, camera_x);
    ros::param::param("~fusion_camera_y", camera_y, camera_y);
    ros::param::param("~fusion_camera_z", camera_z, camera_z);
    ros::param::param("~fusion_camera_roll_deg", camera_roll_deg, camera_roll_deg);
    ros::param::param("~fusion_camera_pitch_deg", camera_pitch_deg, camera_pitch_deg);
    ros::param::param("~fusion_camera_yaw_deg", camera_yaw_deg, camera_yaw_deg);
    ros::param::param("~fusion_lidar_x", lidar_x, lidar_x);
    ros::param::param("~fusion_lidar_y", lidar_y, lidar_y);
    ros::param::param("~fusion_lidar_z", lidar_z, lidar_z);
    ros::param::param("~fusion_lidar_roll_deg", lidar_roll_deg, lidar_roll_deg);
    ros::param::param("~fusion_lidar_pitch_deg", lidar_pitch_deg, lidar_pitch_deg);
    ros::param::param("~fusion_lidar_yaw_deg", lidar_yaw_deg, lidar_yaw_deg);

    if (fx <= 0.0) {
        fx = image_width / (2.0 * std::tan(horizontal_fov_deg * M_PI / 360.0));
    }
    if (fy <= 0.0) {
        fy = fx;
    }

    st_Camera.params.intrinsic_matrix_90_fov = (Eigen::Matrix<double, 3, 4>() <<
        fx, 0,  cx, 0,
        0,  fy, cy, 0,
        0,  0,  1,  0).finished();

    // Translate a point from the LiDAR origin to the camera origin. Both MORAI
    // sensor positions use the vehicle frame (x forward, y left, z up).
    const Eigen::Matrix4d vehicle_from_camera = VehicleFromSensor(
        camera_x, camera_y, camera_z,
        camera_roll_deg, camera_pitch_deg, camera_yaw_deg);
    const Eigen::Matrix4d vehicle_from_lidar = VehicleFromSensor(
        lidar_x, lidar_y, lidar_z,
        lidar_roll_deg, lidar_pitch_deg, lidar_yaw_deg);

    // Full 6DoF transform:
    // T_camera_mount_lidar = inverse(T_vehicle_camera) * T_vehicle_lidar.
    st_Camera.params.sensor_fusion_transform =
        vehicle_from_camera.inverse() * vehicle_from_lidar;

    // BEV(Bird's Eye View)용 변환 행렬
    st_Camera.params.bird_eye_view_transform = vehicle_from_camera.inverse();

    // Camera mount (x forward, y left, z up) -> OpenCV optical
    // (x right, y down, z forward).
    Eigen::Matrix4d camera_mount_to_optical = Eigen::Matrix4d::Identity();
    camera_mount_to_optical.block<3, 3>(0, 0) =
        (Eigen::Matrix3d() <<
             0, -1,  0,
             0,  0, -1,
             1,  0,  0).finished();

    st_Camera.params.sensor_fusion_projection_matrix =
        st_Camera.params.intrinsic_matrix_90_fov *
        camera_mount_to_optical *
        st_Camera.params.sensor_fusion_transform;

    st_Camera.params.bird_eye_view_projection_matrix =
        st_Camera.params.intrinsic_matrix_90_fov *
        camera_mount_to_optical *
        st_Camera.params.bird_eye_view_transform;

    ROS_INFO("Fusion intrinsics: %.0fx%.0f HFOV=%.2f fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
             image_width, image_height, horizontal_fov_deg, fx, fy, cx, cy);
    ROS_INFO("Fusion camera pose: xyz=(%.3f, %.3f, %.3f)m rpy=(%.3f, %.3f, %.3f)deg",
             camera_x, camera_y, camera_z,
             camera_roll_deg, camera_pitch_deg, camera_yaw_deg);
    ROS_INFO("Fusion LiDAR pose: xyz=(%.3f, %.3f, %.3f)m rpy=(%.3f, %.3f, %.3f)deg",
             lidar_x, lidar_y, lidar_z,
             lidar_roll_deg, lidar_pitch_deg, lidar_yaw_deg);
    ROS_INFO_STREAM("T_camera_mount_lidar:\n"
                    << st_Camera.params.sensor_fusion_transform);
    ROS_INFO_STREAM("P_opencv_lidar:\n"
                    << st_Camera.params.sensor_fusion_projection_matrix);

    // BEV를 위한 월드 좌표계 포인트 설정
    st_Camera.params.world_points_3d = {
            {st_Camera.params.world_points_vector[0], st_Camera.params.world_points_vector[2], 0.0f},
            {st_Camera.params.world_points_vector[0], st_Camera.params.world_points_vector[3], 0.0f},
            {st_Camera.params.world_points_vector[1], st_Camera.params.world_points_vector[3], 0.0f},
            {st_Camera.params.world_points_vector[1], st_Camera.params.world_points_vector[2], 0.0f}
    };

    // BEV 이미지의 목적지 좌표
    st_Camera.params.destination_points = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(st_Camera.params.moving_average_filter_size), 0.0f),
        cv::Point2f(static_cast<float>(st_Camera.params.moving_average_filter_size), static_cast<float>(st_Camera.params.moving_average_filter_size)),
        cv::Point2f(0.0f, static_cast<float>(st_Camera.params.moving_average_filter_size))
    };

    st_Camera.params.initial_roll_rad = 0.0;
    st_Camera.params.initial_pitch_rad = 0.0;
    st_Camera.params.is_initial_transform_calculated = false; // IPM 계산 여부 플래그 초기화
}


void ProcessCameraStream(const RawImageFrame& front_frame, const RawImageFrame& traffic_light_frame, CAMERA &st_Camera, GPS_IMU &st_GPSIMU)
{
    cv::Mat encoded1, encoded2;

    pthread_mutex_lock(&st_SensorData.st_MutexCamera);
    if (front_frame.s32_Num <= 0 || front_frame.s32_Num > c_CAMERA_JPEG_SIZE)
    {
        pthread_mutex_unlock(&st_SensorData.st_MutexCamera);
        return;
    }

    encoded1 = cv::Mat(1,
                       front_frame.s32_Num,
                       CV_8UC1,
                       const_cast<char*>(front_frame.arc_Buffer)).clone();

    if (traffic_light_frame.s32_Num > 0 &&
        traffic_light_frame.s32_Num <= c_CAMERA_JPEG_SIZE)
    {
        encoded2 = cv::Mat(1,
                           traffic_light_frame.s32_Num,
                           CV_8UC1,
                           const_cast<char*>(traffic_light_frame.arc_Buffer)).clone();
    }
    pthread_mutex_unlock(&st_SensorData.st_MutexCamera);

    st_Camera.bgr_image = cv::imdecode(encoded1, cv::IMREAD_COLOR);

    if (st_Camera.bgr_image.empty()) {
        std::cout << "[DEBUG] Front camera image is empty after imdecode!" << std::endl;
    }

    if (!encoded2.empty())
    {
        st_Camera.traffic_light_image = cv::imdecode(encoded2, cv::IMREAD_COLOR);
        st_Camera.is_traffic_light_camera_available = true;
    }

    IdentifyTrafficLight(st_Camera);
    FindStopLine(st_Camera, st_GPSIMU);
    LaneDetectionProcess(st_Camera);
}
