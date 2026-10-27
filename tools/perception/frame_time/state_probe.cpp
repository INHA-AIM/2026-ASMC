// Offline arrival-ordered probe of unmodified production estimators. No ROS master.
#ifdef BRIDGE_PROBE
#include <asmc_bridge/vehicle_state_adapter.hpp>
#else
#include <planning_inputs/vehicle_state_adapter.hpp>
#endif
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace frenet_planner_ros;
StateAdapterConfig Config(const std::string& file) {
  const auto y=YAML::LoadFile(file); StateAdapterConfig c;
  // Same YAML fields as the production nodes, required rather than silently defaulted.
  c.gps_timeout_s=y["timeouts"]["gps"].as<double>();
  c.imu_timeout_s=y["timeouts"]["imu"].as<double>();
#define SPEED(member,key) c.member=y["speed"][key].as<double>()
  SPEED(speed_window_duration_s,"window_duration");
  SPEED(minimum_speed_baseline_s,"minimum_baseline"); SPEED(maximum_speed_mps,"maximum_mps");
  SPEED(gps_correction_gain,"gps_correction_gain"); SPEED(gps_innovation_limit_mps,"gps_innovation_limit_mps");
  SPEED(imu_acceleration_filter_alpha,"imu_acceleration_filter_alpha");
  SPEED(imu_acceleration_deadband_mps2,"imu_acceleration_deadband_mps2");
  SPEED(imu_acceleration_min_mps2,"imu_acceleration_min_mps2");
  SPEED(imu_acceleration_max_mps2,"imu_acceleration_max_mps2"); SPEED(imu_maximum_dt_s,"imu_maximum_dt");
  SPEED(stationary_duration_s,"stationary_duration"); SPEED(stationary_distance_m,"stationary_distance");
  SPEED(stationary_speed_threshold_mps,"stationary_speed_threshold_mps");
  SPEED(stationary_acceleration_threshold_mps2,"stationary_acceleration_threshold_mps2");
#ifndef BRIDGE_PROBE
  c.gps_sensor_offset_x_m=y["gps_sensor_offset"]["x"].as<double>();
  c.gps_sensor_offset_y_m=y["gps_sensor_offset"]["y"].as<double>();
  c.position_gps_correction_gain=y["position"]["gps_correction_gain"].as<double>();
  c.imu_yaw_is_true_enu=y["orientation"]["imu_yaw_is_true_enu"].as<bool>();
  SPEED(gps_gain_reference_speed_mps,"gps_gain_reference_speed_mps"); SPEED(gps_gain_minimum,"gps_gain_minimum");
#endif
#undef SPEED
  return c;
}
int main(int argc,char** argv) { try {
  if(argc!=4)throw std::runtime_error("usage: state_probe BAG CONFIG NEW.jsonl");
  if(std::ifstream(argv[3]).good())throw std::runtime_error("output already exists");
  ros::Time::init(); GpsImuStateEstimator estimator(Config(argv[2]));
  rosbag::Bag bag(argv[1],rosbag::bagmode::Read);
  const auto first=rosbag::View(bag).getBeginTime().toNSec();
  rosbag::View view(bag,rosbag::TopicQuery({"/gps","/imu","/velodyne_points"}));
  std::ofstream out(argv[3]); if(!out)throw std::runtime_error("output open failed");
  out<<std::setprecision(17);std::uint64_t seq=0,last_gps=0,last_imu=0;
  for(const auto& m:view) {
    const auto arrival=m.getTime().toNSec();ros::Time::setNow(m.getTime());
    ros::SteadyTime logical;logical.fromNSec(arrival-first+1000000000ULL);
    if(m.getTopic()=="/gps") {auto p=m.instantiate<morai_msgs::GPSMessage>();if(!p)throw std::runtime_error("GPS type");estimator.updateGps(*p,logical);last_gps=p->header.stamp.toNSec();}
    else if(m.getTopic()=="/imu") {auto p=m.instantiate<sensor_msgs::Imu>();if(!p)throw std::runtime_error("IMU type");estimator.updateImu(*p,logical);last_imu=p->header.stamp.toNSec();}
    else {auto p=m.instantiate<sensor_msgs::PointCloud2>();if(!p)throw std::runtime_error("cloud type");
      const auto s=estimator.makeState(logical);
      out<<"{\"seq\":"<<++seq<<",\"scan_ns\":"<<p->header.stamp.toNSec()<<",\"arrival_ns\":"<<arrival
         <<",\"state_ns\":"<<s.header.stamp.toNSec()<<",\"valid\":"<<(s.valid?"true":"false")
         <<",\"latest_gps_ns\":"<<last_gps<<",\"latest_imu_ns\":"<<last_imu;
      // Invalid numeric placeholders must not enter error statistics.
      if(s.valid)out<<",\"x\":"<<s.x<<",\"y\":"<<s.y<<",\"yaw\":"<<s.yaw<<",\"speed\":"<<s.speed;
      out<<"}\n";
    }
  }return out?0:1;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;} }
