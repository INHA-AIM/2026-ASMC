#include <asmc_lidar/geometry/camera_calibration.hpp>
#include <cassert>
#include <iostream>
using namespace asmc::lidar;
int main(int argc,char** argv) {
  if(argc==2) {
    for(const auto& e:camera::load_calibrations(argv[1]))std::cout<<e.first<<" "<<e.second.id<<"\n";
    return 0;
  }
  camera::Calibration k;k.frame_id="camera_front";k.id="fixture";k.width=1280;k.height=720;
  k.intrinsic<<640,0,640,0,640,360,0,0,1;
  Context c;c.frame=Frame::Base;c.time={1000000000,5000000000,5000000000,"ros","fixture","steady"};
  auto p=camera::project(k,c,1000000000,Vec(10,0,0));
  assert(p.valid && p.u==640 && p.v==360 && !p.time_compensated);
  assert(camera::project(k,c,1000000000,Vec(-1,0,0)).reason=="behind_camera");
  assert(camera::project(k,c,1000000000,Vec(1,10,0)).reason=="outside_image");
  p=camera::project(k,c,1010000000,Vec(10,0,0));
  assert(p.valid && p.reason=="ok_uncompensated_time_offset" && p.image_minus_scan_ns==10000000);
  Pose movement=Pose::Identity();movement.translation()=Vec(0,1,0);
  p=camera::project(k,c,1010000000,Vec(10,0,0),movement);
  assert(p.valid && p.time_compensated && std::abs(p.u-576)<1e-10);
  c.frame=Frame::LeveledSensor;c.base_from_sensor.translation()=Vec(.8,0,1.45);
  c.base_from_sensor.linear()=rpy(.04,-.07,.11);c.calibration_source="fixture";
  c.applied_leveling=leveling_rotation(.15,.09,c.base_from_sensor.rotation());c.leveling_source="fixture";
  Vec raw(12,-2,-.8),leveled=c.applied_leveling*raw;
  p=camera::project(k,c,1000000000,leveled);
  auto matrix=camera_projection(c,k.base_from_camera,k.intrinsic);Vec q=(*matrix)*leveled.homogeneous();
  assert(p.valid && std::abs(p.u-q.x()/q.z())<1e-10 && std::abs(p.v-q.y()/q.z())<1e-10);
  movement.matrix()(0,0)=2;assert(!camera::project(k,c,1000000000,leveled,movement).valid);
  c.time.clock="steady";assert(camera::project(k,c,1000000000,leveled).reason=="invalid_time");
  std::cout<<"camera projection/time provenance passed\n";
}
