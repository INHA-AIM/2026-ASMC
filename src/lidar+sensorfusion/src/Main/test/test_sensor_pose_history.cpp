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

TEST(SensorPoseHistory, TrueEnuHeadingAndExtrapolationUseUtmGrid) {
    SensorPoseHistoryConfig config;
    config.imu_yaw_is_true_enu = true;
    SensorPoseHistory history(config);
    // True north at K-City points slightly east in UTM52N.
    auto gps = Gps(10., 20.);
    gps.f64_Latitude = 37.24;
    gps.f64_Longitude = 126.77;
    history.AddImu(1000, Imu(M_PI / 2.)); // IMU arrives before the first GPS.
    history.AddCan(1000, Can(10.));
    history.AddGps(1000, gps);
    GPS_IMU pose{}, local{};
    ASSERT_TRUE(history.Lookup(1100, &pose));
    const double expected_yaw = M_PI / 2. - 0.02356;
    EXPECT_NEAR(expected_yaw, pose.st_IMU.f32_Yaw_rad, 0.0001);
    EXPECT_NEAR(10. + std::cos(expected_yaw), pose.st_GPS.f64_E, 0.0001);
    EXPECT_NEAR(20. + std::sin(expected_yaw), pose.st_GPS.f64_N, 0.0001);
    ASSERT_TRUE(history.WaitForLocalLookup(1100, 0., &local));
    EXPECT_NEAR(M_PI / 2., local.st_IMU.f32_Yaw_rad, 1e-6);
    auto invalid = gps; invalid.f64_Latitude = invalid.f64_Longitude = 0.;
    invalid.f64_E = 1e6;
    history.AddGps(1050, invalid);
    ASSERT_TRUE(history.Lookup(1100, &pose));
    EXPECT_LT(pose.st_GPS.f64_E, 11.);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// End-to-end packet fixture is generated by scripts/signed_velocity_fixture.py.
// Production callbacks + production causal selection, with deliberately nonzero y/z.
#include <Parser/Parser.hpp>
#include <Geometry/FrameBoundary.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdlib>
#include <cstring>
TEST(CompetitionInput, PacketBridgeCallbackAndCausalSelectionPreserveSignedX) {
  const char* path=std::getenv("SIGNED_VELX_TEST_BAG");
  if (!path) GTEST_SKIP() << "Set SIGNED_VELX_TEST_BAG for packet-to-CAN integration";
  ros::Time::init();
  frame_boundary::ConfigureForTest(true,false,"signed_packet_test");
  rosbag::Bag bag(path,rosbag::bagmode::Read);
  int count=0;
  for (auto const& entry:rosbag::View(bag)) {
    auto msg=entry.instantiate<morai_msgs::EgoVehicleStatus>();
    ASSERT_TRUE(bool(msg));
    CANParser_Callback(msg);
    CAN_DATA can{};
    std::memcpy(&can,st_SensorData.st_RawCAN.arc_Buffer,sizeof(can));
    EXPECT_NEAR(msg->velocity.x,can.f32_CurrentVel_MPS,1e-6);
    EXPECT_NEAR(msg->velocity.x*3.6,can.f32_CurrentVel_KPH,1e-5);
    auto imu=boost::make_shared<sensor_msgs::Imu>();
    imu->header.stamp=msg->header.stamp;imu->orientation.w=1;
    IMUParser_Callback(imu);
    GPS_IMU local{};LocalInputSelection selected;
    ASSERT_TRUE(LookupLocalStateForLidarNs(msg->header.stamp.toNSec(),&local,&selected));
    EXPECT_NEAR(msg->velocity.x,local.st_CAN.f32_CurrentVel_MPS,1e-6);
    EXPECT_LE(selected.can_stamp_ns,msg->header.stamp.toNSec());
    EXPECT_LE(selected.imu_stamp_ns,msg->header.stamp.toNSec());
    asmc::lidar::PoseSelection no_map;no_map.reason="gps_missing";
    frame_boundary::PrepareScan(msg->header.stamp,local,st_VehicleSpec,no_map,count+1);
    EXPECT_NEAR(msg->velocity.x,frame_boundary::Current().ego_linear_base.x(),1e-6);
    if(count==2) {
      // A buffered future sample within the same millisecond must not leak into LiDAR time.
      auto future=boost::make_shared<morai_msgs::EgoVehicleStatus>(*msg);
      future->header.stamp.fromNSec(msg->header.stamp.toNSec()+500000);
      future->velocity.x=.5;CANParser_Callback(future);
      auto future_imu=boost::make_shared<sensor_msgs::Imu>(*imu);
      future_imu->header.stamp=future->header.stamp;IMUParser_Callback(future_imu);
      ASSERT_TRUE(LookupLocalStateForLidarNs(msg->header.stamp.toNSec(),&local,&selected));
      EXPECT_NEAR(0.,local.st_CAN.f32_CurrentVel_MPS,1e-6);
      EXPECT_EQ(msg->header.stamp.toNSec(),selected.can_stamp_ns);
      EXPECT_EQ(msg->header.stamp.toNSec(),selected.imu_stamp_ns);
    }

    ++count;
  }
  EXPECT_EQ(3,count);
}
