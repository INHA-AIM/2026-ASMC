// Only pure point/vector adapter functions run; no sensors, tracker or publisher.
#include "io.hpp"
#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <iostream>
// Original Global.cpp has unrelated globals/ROS functions. These two arithmetic
// helpers reproduce its first two functions exactly; adapter source is unedited.
float64 Deg2Rad(const float64& v){return v*M_PI/180.0;}
float32 Deg2Rad(const float32& v){return v*M_PI/180.0f;}
using namespace asmc::lidar_eval;
int main() {
  std::string line;
  while(std::getline(std::cin,line))try {
    const auto n=YAML::Load(line);Json out;out["input_id"]=n["input_id"].as<std::string>();
    const auto legacy=n["legacy_snapshot"];GPS_IMU imu{};VehicleSpec vehicle{};
    const Vec angles=vec(legacy["ego_rpy_rad"]),gps=vec(legacy["gps_map"]);
    imu.st_IMU.f32_Roll_rad=angles.x();imu.st_IMU.f32_Pitch_rad=angles.y();imu.st_IMU.f32_Yaw_rad=angles.z();
    imu.st_GPS.f64_E=gps.x();imu.st_GPS.f64_N=gps.y();imu.st_GPS.f64_U=gps.z();
    const Vec mount=vec(legacy["mount"]),antenna=vec(legacy["gps_in_base"]),mount_deg=vec(legacy["mount_rpy_deg"]);
    vehicle.st_LiDARPos={float(mount.x()),float(mount.y()),float(mount.z())};
    vehicle.st_GPSPos={float(antenna.x()),float(antenna.y()),float(antenna.z())};
    vehicle.st_LiDARRotationDeg={float(mount_deg.x()),float(mount_deg.y()),float(mount_deg.z())};
    Json::Array results;
    for(const auto& v:n["objects"]) {
      const auto o=object(v);auto convert=[&](const Vec& p){auto a=TransformLidarPointToMap({p.x(),p.y(),p.z()},imu,vehicle);return vector_json(Vec(a.x,a.y,a.z));};
      Json item;item["id"]=o.id;item["local_center_as_published"]=vector_json(o.center);
      item["map_center"]=convert(o.center);Json::Array corners;
      for(const auto& p:o.corners)corners.push_back(convert(p));item["map_corners"]=corners;
      auto a=RotateLidarVectorToMap({o.velocity.x(),o.velocity.y(),o.velocity.z()},imu,vehicle);
      item["map_vector"]=vector_json(Vec(a.x,a.y,a.z));results.push_back(item);
    }
    out["objects"]=results;std::cout<<out.dump()<<'\n';
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
