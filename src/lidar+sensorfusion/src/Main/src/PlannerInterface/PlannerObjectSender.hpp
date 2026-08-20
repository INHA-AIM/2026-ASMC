#pragma once

#include <cstddef>
#include <string>

#include <netinet/in.h>

#include <Global/Global.hpp>

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
