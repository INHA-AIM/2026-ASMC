#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <netinet/in.h>

#include <Global/Global.hpp>
#include <yaml-cpp/yaml.h>

class PlannerObjectSender {
public:
    PlannerObjectSender(const std::string& destination_ip, int destination_port,
                        std::size_t max_objects);
    ~PlannerObjectSender();

    PlannerObjectSender(const PlannerObjectSender&) = delete;
    PlannerObjectSender& operator=(const PlannerObjectSender&) = delete;

    void Send(const LiDAR& lidar, const GPS_IMU& gps_imu,
              const VehicleSpec& vehicle) const;

private:
    int socket_fd_ = -1;
    std::size_t max_objects_ = 20;
    sockaddr_in destination_{};
};

// ~planner_udp_enabled default. true reproduces production: the sender exists whenever
// setting.yaml PlannerInterface.Enabled is true. Tests and offline replays pass false.
inline constexpr bool kPlannerUdpEnabledDefault = true;
// nullptr unless planner_udp_enabled && planner_interface["Enabled"]. With
// planner_udp_enabled=false neither the sender nor its UDP socket is created, so Send() can never run.
std::unique_ptr<PlannerObjectSender> MakePlannerObjectSender(
    bool planner_udp_enabled, const YAML::Node& planner_interface);
