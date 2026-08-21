#include <PlannerInterface/PlannerObjectSender.hpp>

#include <PlannerInterface/PlannerObjectAdapter.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {

constexpr std::size_t kDataOffset = 38;
constexpr std::size_t kSlotSize = 106;
constexpr std::size_t kProtocolMaxObjects = 20;

template <typename T>
void WriteAt(std::vector<std::uint8_t>& packet, std::size_t offset, T value) {
    std::memcpy(packet.data() + offset, &value, sizeof(T));
}

}  // namespace

PlannerObjectSender::PlannerObjectSender(
    const std::string& destination_ip, int destination_port,
    std::size_t max_objects)
    : max_objects_(std::min(max_objects, kProtocolMaxObjects)) {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        throw std::runtime_error("PlannerObjectSender: socket() failed");
    }

    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(static_cast<std::uint16_t>(destination_port));
    if (inet_pton(AF_INET, destination_ip.c_str(), &destination_.sin_addr) != 1) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("PlannerObjectSender: invalid destination IP");
    }

    std::printf("[PlannerObjectSender] LiDAR objects -> %s:%d (max=%zu)\n",
                destination_ip.c_str(), destination_port, max_objects_);
}

PlannerObjectSender::~PlannerObjectSender() {
    if (socket_fd_ >= 0) close(socket_fd_);
}

void PlannerObjectSender::Send(const LiDAR& lidar, const GPS_IMU& gps_imu,
                               const VehicleSpec& vehicle) const {
    const std::vector<PlannerObject> objects =
        MakePlannerObjects(lidar, gps_imu, vehicle, max_objects_);

    // 빈 프레임도 한 슬롯을 보내 receiver가 이전 객체 목록을 비우게 한다.
    const std::size_t slot_count = std::max<std::size_t>(1, objects.size());
    std::vector<std::uint8_t> packet(kDataOffset + slot_count * kSlotSize, 0);
    const char header[] = "LIDAR_OBJECT_INFO";
    std::memcpy(packet.data(), header, sizeof(header) - 1);

    for (std::size_t i = 0; i < objects.size(); ++i) {
        const PlannerObject& object = objects[i];
        const std::size_t base = kDataOffset + i * kSlotSize;
        WriteAt(packet, base + 0, object.id);
        WriteAt(packet, base + 2, object.type);
        WriteAt(packet, base + 4, object.x);
        WriteAt(packet, base + 8, object.y);
        WriteAt(packet, base + 12, object.z);
        WriteAt(packet, base + 16, object.heading_deg);
        WriteAt(packet, base + 20, object.length);
        WriteAt(packet, base + 24, object.width);
        WriteAt(packet, base + 28, object.height);

        // Existing receiver consumes velocity XYZ at offsets 44/48 in km/h.
        const double heading_rad = object.heading_deg * 3.14159265358979323846 / 180.0;
        const float velocity_x = static_cast<float>(object.speed_kph * std::cos(heading_rad));
        const float velocity_y = static_cast<float>(object.speed_kph * std::sin(heading_rad));
        WriteAt(packet, base + 44, velocity_x);
        WriteAt(packet, base + 48, velocity_y);
    }

    const ssize_t sent = sendto(
        socket_fd_, packet.data(), packet.size(), 0,
        reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_));
    if (sent < 0) {
        static int error_count = 0;
        if (++error_count % 20 == 1) {
            std::fprintf(stderr, "[PlannerObjectSender] sendto failed: %s\n",
                         std::strerror(errno));
        }
    }
}
