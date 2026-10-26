#include <gtest/gtest.h>
#include <cmath>
#include <asmc_geometry/geodesy.hpp>
using asmc::geometry::Utm52NYawOffset;
using asmc::geometry::GpsToUtm52N;
#include "planning_inputs/vehicle_state_adapter.hpp"

namespace {
morai_msgs::GPSMessage Gps(double t, double latitude) {
    morai_msgs::GPSMessage gps;
    gps.header.stamp=ros::Time(t); gps.status=1;
    gps.latitude=latitude; gps.longitude=126.7733;
    gps.eastOffset=302595; gps.northOffset=4124145;
    return gps;
}
sensor_msgs::Imu Imu(double t, double yaw) {
    sensor_msgs::Imu imu; imu.header.stamp=ros::Time(t);
    imu.orientation.w=std::cos(yaw/2);imu.orientation.z=std::sin(yaw/2);
    return imu;
}
void Feed(frenet_planner_ros::GpsImuStateEstimator& estimator, int i,
          double yaw=M_PI/2) {
    const double t=10+.2*i;
    const ros::SteadyTime now(t);
    // IMU arrives first, including before the initial GPS grid basis is known.
    estimator.updateImu(Imu(t,yaw),now);
    estimator.updateGps(Gps(t,37.2394+.00002*i),now);
}
}

TEST(GridYaw, CentralMeridianAndKCityProjectionBasis) {
    EXPECT_NEAR(Utm52NYawOffset(37.2394,129),0,1e-8);
    // K-City is west of the zone's central meridian. True north points slightly
    // east of grid north; the measured heading discrepancy is about 1.35 deg.
    EXPECT_NEAR(Utm52NYawOffset(37.2394,126.7733)*180/M_PI,-1.348,0.01);
    EXPECT_GT(Utm52NYawOffset(37.2394,131),0);
}

TEST(GridYaw, LateFirstGpsRotatesRetainedImuAndPublishedState) {
    ros::Time::init();
    frenet_planner_ros::StateAdapterConfig cfg;
    cfg.imu_maximum_dt_s=.25;cfg.imu_yaw_is_true_enu=true;
    frenet_planner_ros::GpsImuStateEstimator fixed(cfg);
    cfg.imu_yaw_is_true_enu=false;
    frenet_planner_ros::GpsImuStateEstimator legacy(cfg);
    for(int i=0;i<4;++i){Feed(fixed,i);Feed(legacy,i);}
    const auto a=fixed.makeState(ros::SteadyTime(10.6));
    const auto b=legacy.makeState(ros::SteadyTime(10.6));
    ASSERT_TRUE(a.valid);ASSERT_TRUE(b.valid);
    EXPECT_NEAR(a.yaw,M_PI/2+Utm52NYawOffset(37.23946,126.7733),1e-9);
    EXPECT_NEAR(b.yaw,M_PI/2,1e-9);
    EXPECT_NEAR(a.header.stamp.toSec(),10.6,1e-9);
    EXPECT_GT(a.speed,5);
}

TEST(GridYaw, JammingSentinelCannotPoisonGridOrPositionHistory) {
    ros::Time::init();
    frenet_planner_ros::StateAdapterConfig cfg;
    cfg.imu_maximum_dt_s=.25;cfg.imu_yaw_is_true_enu=true;
    frenet_planner_ros::GpsImuStateEstimator estimator(cfg);
    for(int i=0;i<4;++i)Feed(estimator,i);
    auto zero=Gps(10.8,0);zero.longitude=0;
    estimator.updateGps(zero,ros::SteadyTime(10.8));
    EXPECT_FALSE(estimator.makeState(ros::SteadyTime(10.8)).valid);
    Feed(estimator,5);Feed(estimator,6);
    const auto state=estimator.makeState(ros::SteadyTime(11.2));
    ASSERT_TRUE(state.valid);
    EXPECT_LT(std::abs(state.x),2000);EXPECT_LT(std::abs(state.y),2000);
    EXPECT_NEAR(state.yaw,M_PI/2+Utm52NYawOffset(37.23952,126.7733),1e-8);
}

TEST(GridYaw, WrapsCorrectedOrientation) {
    ros::Time::init();
    frenet_planner_ros::StateAdapterConfig cfg;
    cfg.imu_maximum_dt_s=.25;cfg.imu_yaw_is_true_enu=true;
    frenet_planner_ros::GpsImuStateEstimator estimator(cfg);
    for(int i=0;i<4;++i)Feed(estimator,i,-179*M_PI/180);
    const auto state=estimator.makeState(ros::SteadyTime(10.6));
    ASSERT_TRUE(state.valid);
    EXPECT_GT(state.yaw,3.0);EXPECT_LE(state.yaw,M_PI);
}
