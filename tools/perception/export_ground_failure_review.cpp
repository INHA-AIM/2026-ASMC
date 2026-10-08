// Review-only exporter: call the actual production filter with the ROS fixture.
// No sensor/UDP/control workers are started. Each policy uses a fresh process.
#include <Geometry/FrameBoundary.hpp>
#include <Camera/Camera.hpp>
#include <Lidar/ImuRansac/ImuRansac.hpp>
#include <ros/package.h>
#include <fstream>
#include <iomanip>

extern VehicleSpec st_VehicleSpec;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
void Points(std::ostream& f,const Cloud& cloud) {
  f << '[';
  for(std::size_t i=0;i<cloud.size();++i) {
    if(i) f << ',';
    const auto& p=cloud[i]; f << '[' << p.x << ',' << p.y << ',' << p.z << ']';
  }
  f << ']';
}
int main(int argc,char** argv) {
  if(argc!=3) return 2;
  const bool common=std::string(argv[1])=="common";
  ros::init(argc,argv,"ground_failure_review");
  ros::NodeHandle node("~");
  node.setParam("enable_common_geometry",common);node.setParam("lidar_level_roll",true);
  st_VehicleSpec.st_LiDARPos={.8f,0,1.45f};
  st_VehicleSpec.st_LiDARRotationDeg={0,0,0};st_VehicleSpec.st_GPSPos={.35f,0,1.25f};
  CAMERA camera{};
  InitProjectionMatrix(camera,YAML::LoadFile(ros::package::getPath("Main")+"/setting.yaml"));
  frame_boundary::Configure(node,st_VehicleSpec,camera);
  GPS_IMU local{};local.st_IMU.b_ImuReady=true;
  local.st_CAN.f32_CurrentVel_MPS=2.;local.st_CAN.f32_CurrentVel_KPH=7.2;
  asmc::lidar::PoseSelection no_map;
  ros::Time::setNow(ros::Time(75.));
  frame_boundary::PrepareScan(ros::Time(75.),local,st_VehicleSpec,no_map);
  frame_boundary::StoreLeveling(asmc::lidar::Rot::Identity());
  Cloud::Ptr ground(new Cloud),input(new Cloud),output(new Cloud);
  for(int x=0;x<8;++x)for(int y=0;y<8;++y) {
    pcl::PointXYZI p;p.x=5.f+x*.3f;p.y=-1.f+y*.3f;p.z=-1.5f;ground->push_back(p);
  }
  NormalVectorFiltering(ground,output,local);
  const auto initial=CurrentGroundPlaneSnapshot();
  if(!initial.valid) return 3;
  ros::Time::setNow(ros::Time(75.1));local.st_IMU.f32_Roll_rad=10*M_PI/180.;
  frame_boundary::PrepareScan(ros::Time(75.1),local,st_VehicleSpec,no_map);
  frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(local.st_IMU.f32_Roll_rad,0,
      asmc::lidar::Rot::Identity()));
  for(int i=0;i<24;++i) {
    pcl::PointXYZI p;p.x=8.f+i*.02f;p.y=.2f;p.z=-1.5f;input->push_back(p);
  }
  // Deliberately identical to the test: points already occupy filter coordinates.
  // No CorrectingPoints call: roll is context metadata, not a synthetic raw scan.
  NormalVectorFiltering(input,output,local);
  const auto current=CurrentGroundPlaneSnapshot();
  if(output->size()!=(common?24U:0U) || current.valid==common) return 4;
  std::ofstream f(argv[2]);f << std::setprecision(17);
  f << "{\"policy\":\"" << (common?"common_preserve_unknown":"legacy_previous_plane_fallback")
    << "\",\"actual_filter\":true,\"independent_object_GT\":false,\"roll_context_deg\":10,"
       "\"points_already_in_filter_frame\":true,\"previous_plane\":[";
  for(int i=0;i<4;++i){if(i)f<<',';f<<initial.coefficients[i];}
  f << "],\"current_plane_valid\":" << (current.valid?"true":"false")
    << ",\"initial_ground\":";Points(f,*ground);
  f << ",\"input\":";Points(f,*input);f << ",\"output\":";Points(f,*output);f << "}\n";
  return f.good()?0:5;
}
