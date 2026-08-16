#pragma once
#include <Global/Global.hpp>
#include <boost/optional.hpp>
#include <netinet/in.h>
#include <arpa/inet.h>

extern CAMERA st_Camera;
extern SENSOR_DATA st_SensorData;

// ====================== Camera.cpp ====================== //
class UDPReceiver {
public:
    // UDP -> UdpConfig 로 수정
    UDPReceiver(UdpConfig &st_UdpConfig);
    ~UDPReceiver();

    void start();
    void stop();
    boost::optional<vector<uint8_t>> get_image_data();

private:
    static void* receiver_thread_function(void* context);
    void receive_loop();

    int socket_fd_;
    struct sockaddr_in server_addr_;

    pthread_t receiver_thread_;
    pthread_mutex_t image_mutex_;
    std::atomic<bool> stop_flag_{false};
    std::atomic<bool> thread_started_{false};

    vector<uint8_t> completed_image_data_;
};

void InitProjectionMatrix(CAMERA &st_Camera, const YAML::Node& config);
void ProcessCameraStream(const RawImageFrame& front_frame, const RawImageFrame& traffic_light_frame, CAMERA &st_Camera, GPS_IMU &st_GPSIMU);

// ====================== TrafficLight.cpp ====================== //
void IdentifyTrafficLight(CAMERA& st_Camera);

// ====================== StopLine.cpp ========================== //
void FindStopLine(CAMERA& st_Camera, GPS_IMU &st_GPSIMU);

// ====================== LaneDetection.cpp ====================== //
void LaneDetectionProcess(CAMERA& st_Camera); 
