#include <gtest/gtest.h>
#include <cmath>
#include "planning_inputs/gps_transform.hpp"
#include "planning_inputs/vehicle_state_adapter.hpp"

namespace {
using frenet_planner_ros::GpsImuStateEstimator;
using frenet_planner_ros::StateAdapterConfig;
morai_msgs::GPSMessage Gps(double t, double x) {
    morai_msgs::GPSMessage gps;
    gps.header.stamp = ros::Time(t); gps.status = 1;
    gps.latitude = 37.24; gps.longitude = 126.77;
    double e, n; GpsToUtm52N(gps.latitude, gps.longitude, e, n);
    gps.eastOffset = e - x; gps.northOffset = n;
    return gps;
}
void Imu(GpsImuStateEstimator& estimator, double t) {
    sensor_msgs::Imu imu; imu.header.stamp = ros::Time(t);
    imu.orientation.w = 1.;
    estimator.updateImu(imu, ros::SteadyTime(t));
}
void Feed(GpsImuStateEstimator& estimator, double t, double x) {
    Imu(estimator, t);
    estimator.updateGps(Gps(t, x), ros::SteadyTime(t));
}
StateAdapterConfig Config() {
    ros::Time::init();
    StateAdapterConfig cfg; cfg.position_gps_correction_gain = .35;
    cfg.gps_sensor_offset_x_m = .35; cfg.gps_timeout_s = .5;
    return cfg;
}
}

TEST(GpsRecovery, ReanchorsAfterJammingAndRebuildsVelocityBaseline) {
    GpsImuStateEstimator estimator(Config());
    Feed(estimator, 10., 0.); Feed(estimator, 10.15, 1.5);
    ASSERT_TRUE(estimator.makeState(ros::SteadyTime(10.15)).valid);
    auto jam = Gps(10.2, 0.); jam.latitude = jam.longitude = 0.;
    estimator.updateGps(jam, ros::SteadyTime(10.2));
    for (int i = 4; i <= 200; ++i) Imu(estimator, 10. + .05*i);
    EXPECT_FALSE(estimator.makeState(ros::SteadyTime(20.)).valid);
    // Old IMU integration is far from this recovered fix. It must not be
    // blended into a valid pose or used as a cross-outage speed baseline.
    Feed(estimator, 20.05, 40.);
    EXPECT_FALSE(estimator.makeState(ros::SteadyTime(20.05)).valid);
    Feed(estimator, 20.20, 40.75);
    const auto state = estimator.makeState(ros::SteadyTime(20.20));
    ASSERT_TRUE(state.valid);
    EXPECT_NEAR(state.x, 40.75 - .35, .01);
    EXPECT_NEAR(state.speed, 5., .01);
}

TEST(GpsRecovery, WaitsForSourceAlignedImuEvenWhenGpsArrivesFirst) {
    GpsImuStateEstimator estimator(Config());
    Feed(estimator, 10., 0.); Feed(estimator, 10.15, 1.5);
    auto jam = Gps(10.2, 0.); jam.latitude = jam.longitude = 0.;
    estimator.updateGps(jam, ros::SteadyTime(10.2));
    estimator.updateGps(Gps(10.3, 30.), ros::SteadyTime(10.3));
    EXPECT_FALSE(estimator.makeState(ros::SteadyTime(10.3)).valid);
    Imu(estimator, 10.3);
    EXPECT_FALSE(estimator.makeState(ros::SteadyTime(10.3)).valid);
    Feed(estimator, 10.45, 30.75);
    const auto state = estimator.makeState(ros::SteadyTime(10.45));
    ASSERT_TRUE(state.valid);
    EXPECT_NEAR(state.x, 30.75 - .35, .01);
    EXPECT_NEAR(state.speed, 5., .01);
}

TEST(GpsRecovery, SilentGpsOutageAlsoInvalidatesOldBaseline) {
    GpsImuStateEstimator estimator(Config());
    Feed(estimator, 10., 0.); Feed(estimator, 10.15, 1.5);
    for (int i = 4; i <= 100; ++i) Imu(estimator, 10. + .05*i);
    Feed(estimator, 15.05, 20.);
    EXPECT_FALSE(estimator.makeState(ros::SteadyTime(15.05)).valid);
    Feed(estimator, 15.20, 20.75);
    const auto state = estimator.makeState(ros::SteadyTime(15.20));
    ASSERT_TRUE(state.valid);
    EXPECT_NEAR(state.x, 20.75 - .35, .01);
    EXPECT_NEAR(state.speed, 5., .01);
}
