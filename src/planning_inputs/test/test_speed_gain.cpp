#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "planning_inputs/gps_transform.hpp"
#include "planning_inputs/vehicle_state_adapter.hpp"

namespace {
using frenet_planner_ros::StateAdapterConfig;
std::vector<double> Replay(StateAdapterConfig cfg, double cruise, bool brake) {
    ros::Time::init();
    cfg.imu_yaw_is_true_enu=true;
    frenet_planner_ros::GpsImuStateEstimator estimator(cfg);
    std::vector<double> speeds;
    double position=0.;
    double e0,n0,e1,n1;
    GpsToUtm52N(37.24,126.77,e0,n0);
    GpsToUtm52N(37.24001,126.77,e1,n1);
    const double metres_per_degree=std::hypot(e1-e0,n1-n0)/1e-5;
    for(int i=0;i<=480;++i) {
        const double t=i*.025, stamp=10.+t;
        const double v=brake?std::max(0.,cruise-5.*std::max(0.,t-5.)):cruise;
        const double a=brake && t>=5. && t<5.+cruise/5.?-5.:0.;
        position+=v*.025;
        sensor_msgs::Imu imu;imu.header.stamp=ros::Time(stamp);
        imu.orientation.w=imu.orientation.z=std::sqrt(.5);
        imu.linear_acceleration.x=a;
        estimator.updateImu(imu,ros::SteadyTime(stamp));
        if(i%8==0) {
            morai_msgs::GPSMessage gps;gps.header.stamp=ros::Time(stamp);gps.status=1;
            // Alternate position noise; no noise once physically stopped.
            const double noise=v>0.?((i/8)%2?.10:-.10):0.;
            gps.latitude=37.24+(position+noise)/metres_per_degree;
            gps.longitude=126.77;gps.eastOffset=302595;gps.northOffset=4124145;
            estimator.updateGps(gps,ros::SteadyTime(stamp));
        }
        speeds.push_back(estimator.makeState(ros::SteadyTime(stamp)).speed);
    }
    return speeds;
}
double Error(const std::vector<double>& values,double truth) {
    double sum=0.;
    for(unsigned i=160;i<values.size();++i)sum+=std::pow(values[i]-truth,2);
    return std::sqrt(sum/(values.size()-160));
}
}

TEST(SpeedGain, ReducesHighSpeedGpsNoiseWithoutChangingImuPrediction) {
    StateAdapterConfig cfg;const auto fixed=Replay(cfg,16.,false);
    cfg.gps_gain_reference_speed_mps=5.;
    const auto scheduled=Replay(cfg,16.,false);
    EXPECT_LT(Error(scheduled,16.),.5*Error(fixed,16.));
}
TEST(SpeedGain, KeepsExistingLowSpeedCorrection) {
    StateAdapterConfig cfg;const auto fixed=Replay(cfg,3.,false);
    cfg.gps_gain_reference_speed_mps=5.;
    EXPECT_EQ(fixed,Replay(cfg,3.,false));
}
TEST(SpeedGain, RestoresCorrectionWhileBrakingAndSettlesAtRest) {
    StateAdapterConfig cfg;cfg.gps_gain_reference_speed_mps=5.;
    const auto scheduled=Replay(cfg,15.,true);
    EXPECT_LT(scheduled[360],.2); // 1 s after the physical stop.
    EXPECT_NEAR(scheduled.back(),0.,.02);
}
TEST(SpeedGain, InvalidReferenceDisablesScheduling) {
    StateAdapterConfig cfg;const auto fixed=Replay(cfg,16.,false);
    cfg.gps_gain_reference_speed_mps=NAN;
    EXPECT_EQ(fixed,Replay(cfg,16.,false));
}
