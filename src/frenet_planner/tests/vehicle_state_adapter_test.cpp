#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "ros/state/vehicle_state_adapter.hpp"

namespace {
void Expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

morai_msgs::GPSMessage MakeGps(double stamp, double latitude) {
    morai_msgs::GPSMessage gps;
    gps.header.stamp = ros::Time(stamp);
    gps.latitude = latitude;
    gps.longitude = 127.0;
    gps.eastOffset = 0.0;
    gps.northOffset = 0.0;
    gps.status = 1;
    return gps;
}

sensor_msgs::Imu MakeImu(double stamp) {
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time(stamp);
    imu.orientation.w = 1.0;
    imu.angular_velocity.z = 0.2;
    imu.linear_acceleration.x = 0.5;
    return imu;
}
}  // namespace

int main() {
    frenet_planner_ros::StateAdapterConfig config;
    config.gps_timeout_s = 0.3;
    config.imu_timeout_s = 0.3;
    config.speed_filter_alpha = 1.0;
    frenet_planner_ros::GpsImuStateEstimator estimator(config);

    estimator.updateGps(MakeGps(10.0, 37.0));
    estimator.updateImu(MakeImu(10.0));
    Expect(!estimator.makeState(ros::Time(10.1)).valid,
           "one GPS fix must not invent speed");

    estimator.updateGps(MakeGps(10.1, 37.000001));
    estimator.updateImu(MakeImu(10.1));
    const auto state = estimator.makeState(ros::Time(10.2));
    Expect(state.valid, "fresh GPS/IMU with two fixes must be valid");
    Expect(state.source == asmc_msgs::VehicleState::SOURCE_GPS_IMU,
           "competition source must be GPS_IMU");
    Expect(state.header.frame_id == "map", "canonical frame must be map");
    Expect(state.speed > 0.0, "speed must be derived from GPS displacement");
    Expect(std::abs(state.acceleration - 0.5) < 1e-9,
           "longitudinal acceleration must come from IMU x");
    Expect(std::abs(state.yaw_rate - 0.2) < 1e-9,
           "yaw rate must come from IMU z");
    Expect(!estimator.makeState(ros::Time(10.5)).valid,
           "stale source data must produce invalid state");

    // 시계가 뒤로 점프하면 now < stamp가 되어 데이터가 미래처럼 보인다.
    // 이때 stale로 처리해 VehicleState를 invalid로 내보내면 Frenet/MPC가
    // 매 점프마다 급정지한다(실측: WSL2/Docker 환경에서 약 28초 주기로
    // wall clock -1.2초 점프). 데이터 자체는 멀쩡하므로 유효해야 한다.
    Expect(estimator.makeState(ros::Time(8.9)).valid,
           "backward clock jump must not invalidate otherwise good state");
    Expect(estimator.makeState(ros::Time(10.1)).valid,
           "state stays valid right after the clock jump recovers");

    // 시계 점프로 설명되지 않는 먼 미래 stamp는 잘못된 입력이므로 거부한다.
    Expect(!estimator.makeState(ros::Time(0.0)).valid,
           "implausibly far future stamp is still rejected");

    morai_msgs::EgoVehicleStatus ego;
    ego.header.stamp = ros::Time(20.0);
    ego.position.x = 1.0;
    ego.position.y = 2.0;
    ego.heading = 90.0;
    ego.velocity.x = 3.0;
    ego.velocity.y = 4.0;
    ego.acceleration.x = 0.7;
    const auto dev = frenet_planner_ros::ConvertEgoVehicleStatus(ego);
    Expect(dev.valid, "finite EgoStatus must convert for development");
    Expect(dev.source == asmc_msgs::VehicleState::SOURCE_EGO_STATUS,
           "development source must be EGO_STATUS");
    Expect(std::abs(dev.yaw - M_PI_2) < 1e-9,
           "Ego heading degrees must convert to radians");
    Expect(std::abs(dev.speed - 5.0) < 1e-9,
           "Ego planar velocity must convert to speed");

    ego.position.x = std::numeric_limits<double>::quiet_NaN();
    Expect(!frenet_planner_ros::ConvertEgoVehicleStatus(ego).valid,
           "non-finite EgoStatus must be invalid");
    std::cout << "vehicle_state_adapter_test: PASS\n";
    return 0;
}
