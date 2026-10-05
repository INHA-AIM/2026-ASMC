#include <asmc_lidar/tracking.hpp>
#include <cmath>
#include <stdexcept>

namespace {
void Check(bool ok){if(!ok)throw std::runtime_error("tracking contract failed");}
asmc_lidar::FrameDetection Detection(double x,double y=0){
  asmc_lidar::FrameDetection d;d.center_base={x,y,.5};d.detection_index=7;
  for(int i=0;i<8;++i)
    d.corners_base[i]={x+((i==1||i==2||i==5||i==6)?1.:-1.),
                       y+((i==2||i==3||i==6||i==7)?.5:-.5),i<4?0.:1.};
  d.source_indices={1,2,3};return d;
}
asmc_lidar::UnifiedResult Scan(std::int64_t t,bool detected,double x=10){
  asmc_lidar::UnifiedResult scan;scan.observation_ns=t;
  if(detected)scan.detections.push_back(Detection(x));return scan;
}
asmc_lidar::CameraObservation Camera(std::int64_t t){
  asmc_lidar::CameraObservation c;c.stamp_ns=t;c.valid=true;c.frame_id="Camera-1";
  c.width=1280;c.height=720;c.boxes.push_back({580,360,700,500,"car"});return c;
}
}
int main(){
  using namespace asmc_lidar;
  TrackingEngine engine;
  const auto t=1000000000LL;
  auto a=engine.Update(Scan(t,true),0);
  Check(a.size()==1&&a[0].id==1&&a[0].observed&&a[0].geometry.detection_index==7);
  Check(!a[0].relative_velocity_valid&&!a[0].absolute_velocity_valid&&!a[0].dynamic_valid);
  EgoTwist twist;twist.valid=true;twist.linear_base.x()=5;
  auto b=engine.Update(Scan(t+100000000,true,9.5),0,twist);
  Check(b.size()==1&&b[0].id==1&&b[0].age==2&&b[0].relative_velocity_valid);
  Check(b[0].absolute_velocity_valid&&b[0].absolute_velocity_base.allFinite());
  auto missing=engine.Update(Scan(t+200000000,false),0,twist);
  Check(missing.size()==1&&missing[0].id==1&&!missing[0].observed&&missing[0].miss_count==1);
  Check(missing[0].geometry.source_indices.empty());
  for(int i=2;i<=25;++i)missing=engine.Update(Scan(t+(i+1)*100000000LL,false),0,twist);
  Check(missing.empty());

  // GPS/map presence is not an input to this local tracker. One camera result
  // cannot be counted five times at 15 Hz, even when the same box is reused.
  engine.Reset();
  auto camera=Camera(t);auto first=engine.Update(Scan(t,true),0,EgoTwist{},&camera);
  Check(first[0].classification=="unknown");
  auto duplicate=engine.Update(Scan(t+10000000,true),0,EgoTwist{},&camera);
  Check(duplicate[0].classification=="unknown");
  for(int i=1;i<=4;++i){
    camera=Camera(t+10000000+i*10000000LL);
    first=engine.Update(Scan(camera.stamp_ns,true),0,EgoTwist{},&camera);
  }
  Check(first.size()==1&&first[0].id==1&&first[0].classification=="car");
  Check(first[0].camera_class=="car"&&!first[0].absolute_velocity_valid);
  bool regressed=false;
  try{engine.Update(Scan(camera.stamp_ns,true),0);}catch(const std::invalid_argument&){regressed=true;}
  Check(regressed);
  return 0;
}
