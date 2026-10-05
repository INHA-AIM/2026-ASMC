#include <Parser/SensorPoseHistory.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <thread>

namespace {

GPS_DATA Gps(double east, double north) {
    GPS_DATA gps{};
    gps.f64_E = east;
    gps.f64_N = north;
    return gps;
}

IMU_DATA Imu(double yaw) {
    IMU_DATA imu{};
    imu.f32_Yaw_rad = yaw;
    imu.b_ImuReady = true;
    return imu;
}

CAN_DATA Can(double speed_mps) {
    CAN_DATA can{};
    can.f32_CurrentVel_MPS = speed_mps;
    can.f32_CurrentVel_KPH = speed_mps * 3.6;
    return can;
}

}  // namespace

TEST(SensorPoseHistory, ExtrapolatesGpsToLidarTimestamp) {
    SensorPoseHistory history;
    history.AddGps(1000, Gps(0.0, 5.0));
    history.AddGps(1200, Gps(2.0, 5.0));
    history.AddImu(1270, Imu(0.25));
    history.AddCan(1270, Can(10.0));
    GPS_IMU pose{};
    ASSERT_TRUE(history.Lookup(1300, &pose));
    EXPECT_NEAR(2.0 + std::cos(0.25), pose.st_GPS.f64_E, 1e-6);
    EXPECT_NEAR(5.0 + std::sin(0.25), pose.st_GPS.f64_N, 1e-6);
    EXPECT_NEAR(0.25, pose.st_IMU.f32_Yaw_rad, 1e-6);
}

TEST(SensorPoseHistory, UsesPastImuEvenWhenFutureSampleIsBuffered) {
    SensorPoseHistory history;
    history.AddGps(1000, Gps(0.0, 0.0));
    history.AddGps(1200, Gps(0.0, 0.0));
    history.AddImu(1190, Imu(0.1));
    history.AddImu(1210, Imu(0.11));
    history.AddCan(1190, Can(0.0));
    GPS_IMU pose{};
    ASSERT_TRUE(history.Lookup(1200, &pose));
    EXPECT_NEAR(0.1, pose.st_IMU.f32_Yaw_rad, 1e-6);
}

TEST(SensorPoseHistory, RejectsStaleGpsInsteadOfReusingOldPose) {
    SensorPoseHistory history;
    history.AddGps(1000, Gps(0.0, 0.0));
    history.AddGps(1200, Gps(1.0, 0.0));
    history.AddImu(1550, Imu(0.0));
    history.AddCan(1550, Can(0.0));
    GPS_IMU pose{};
    std::string reason;
    EXPECT_FALSE(history.Lookup(1600, &pose, &reason));
    EXPECT_EQ("GPS is stale for LiDAR scan", reason);
}

TEST(SensorPoseHistory, RejectsImpossibleGpsHistorySpeed) {
    SensorPoseHistory history;
    history.AddGps(1000, Gps(0.0, 0.0));
    history.AddGps(1100, Gps(10.0, 0.0));
    history.AddImu(1100, Imu(0.0));
    history.AddCan(1100, Can(0.0));
    GPS_IMU pose{};
    std::string reason;
    EXPECT_FALSE(history.Lookup(1100, &pose, &reason));
    EXPECT_EQ("GPS history implies impossible speed", reason);
}

TEST(SensorPoseHistory, RejectsAlternatingImuYawSpike) {
    SensorPoseHistory history;
    std::string reason;
    EXPECT_TRUE(history.AddImu(1000, Imu(-1.53), &reason));
    EXPECT_FALSE(history.AddImu(1010, Imu(-2.55), &reason));
    EXPECT_EQ("implausible IMU yaw jump", reason);
    EXPECT_TRUE(history.AddImu(1020, Imu(-1.54), &reason));
}

TEST(SensorPoseHistory, RejectsAlternatingVehicleSpeedSpike) {
    SensorPoseHistory history;
    std::string reason;
    EXPECT_TRUE(history.AddCan(1000, Can(10.0), &reason));
    EXPECT_FALSE(history.AddCan(1001, Can(0.0), &reason));
    EXPECT_EQ("implausible vehicle-speed jump", reason);
    EXPECT_TRUE(history.AddCan(1020, Can(10.1), &reason));
}

TEST(SensorPoseHistory, WaitsForCallbackOrderJitterAtSameSourceTime) {
    SensorPoseHistory history;
    history.AddGps(1000, Gps(0.0, 0.0));
    history.AddGps(1200, Gps(1.0, 0.0));

    std::thread delayed_pose([&history]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        history.AddImu(1250, Imu(0.0));
        history.AddCan(1250, Can(1.0));
    });

    GPS_IMU pose{};
    std::string reason;
    EXPECT_TRUE(history.WaitForLookup(1250, 0.05, &pose, &reason));
    EXPECT_EQ("ready", reason);
    delayed_pose.join();
}

TEST(SensorPoseHistory, WaitTimeoutStillRejectsMissingPose) {
    SensorPoseHistory history;
    history.AddGps(1000, Gps(0.0, 0.0));
    GPS_IMU pose{};
    std::string reason;
    EXPECT_FALSE(history.WaitForLookup(1200, 0.005, &pose, &reason));
    EXPECT_EQ("no IMU at or before LiDAR scan", reason);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
