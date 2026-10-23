#include <asmc_lidar/fusion/camera_observations.hpp>
#include <cassert>
#include <iostream>
#include <limits>
#include <thread>
using namespace asmc::lidar;
using namespace asmc::lidar::camera;
constexpr Ns T=1000000000, W=5000000000;
Calibrations calibration() {
  Calibrations c;
  for(const std::string frame:{"camera_front","camera_left","camera_right"}) {
    Calibration k;k.frame_id=frame;k.id="calibration_"+frame;k.width=1280;k.height=720;c[frame]=k;
  }return c;
}
Status status(Ns t=T,std::uint64_t g=1,std::string session="session1",unsigned mode=2) {
  return {t,0,session,"yolo",g,static_cast<std::uint8_t>(mode)};
}
camera::Observation frame(Ns t=T,std::string view="camera_front",bool empty=false) {
  camera::Observation o;o.header={42,t,view};o.width=1280;o.height=720;o.valid=true;
  o.session="session1";o.generation=1;o.calibration_id="calibration_"+view;o.model_id="model";o.config_id="config";
  if(!empty)o.boxes.push_back({1,2,30,40,"vehicle","bus",.9f,true});return o;
}
Candidate first(Buffer& b,Ns t=T,Ns w=W,std::string view="camera_front") {
  return b.query(t,0,t,w).views.at(view).candidates.at(0);
}
int main() {
  {
    Buffer b(calibration());assert(b.observe(frame(),T,W)=="status_missing");
    assert(b.status(status(),T,W)=="status_accepted");
    assert(b.observe(frame(),T,W)=="accepted");auto candidate=first(b);
    assert(candidate.record->observation.header.sequence==42);
    assert(candidate.record->observation.boxes[0].source_class_name=="bus");
    assert(b.current(candidate,T,W));assert(b.observe(frame(),T,W)=="duplicate_or_old");
    auto q=b.query(T+10,20,T,W);assert(q.views.at("camera_front").candidates[0].image_minus_scan_ns==-10);
    assert(b.query(T+30,20,T,W).views.at("camera_front").reason=="no_time_match");
    assert(b.query(T,0,T,W).views.at("camera_left").reason=="missing");
    auto invalid=frame();invalid.valid=false;invalid.width=invalid.height=0;
    assert(b.observe(invalid,T,W)=="invalid");assert(!b.current(candidate,T,W));
    assert(b.query(T,0,T,W).views.at("camera_front").reason=="invalid");
    assert(b.observe(frame(T+1,"camera_front",true),T+1,W+1)=="accepted");
    assert(b.query(T+1,0,T+1,W+1).views.at("camera_front").reason=="empty");
    assert(b.observe(frame(T+1,"camera_left"),T+1,W+1)=="accepted");
    auto left=first(b,T+1,W+1,"camera_left");invalid.header.stamp_ns=T+1;
    b.observe(invalid,T+1,W+1);assert(b.current(left,T+1,W+1));
  }
  {
    Buffer b(calibration());b.status(status(),T,W);b.observe(frame(),T,W);auto old=first(b);
    assert(b.status(status(T+1,2,"session1",0),T+1,W+1)=="status_accepted");
    assert(!b.current(old,T+1,W+1));assert(b.observe(frame(T+1),T+1,W+1)=="standby");
    b.status(status(T+2,3,"session1",1),T+2,W+2);
    assert(b.query(T+2,0,T+2,W+2).views.at("camera_front").reason=="prepare");
    b.status(status(T+3,1,"session2"),T+3,W+3);
    assert(b.observe(frame(T+3),T+3,W+3)=="epoch_mismatch");
    assert(b.status(status(T+4),T+4,W+4)=="retired_session");
    auto o=frame(T+4);o.session="session2";assert(b.observe(o,T+4,W+4)=="accepted");
    auto good=first(b,T+4,W+4);assert(b.current(good,T+4,W+4));
    o.header.stamp_ns++;o.model_id="different";
    assert(b.observe(o,T+5,W+5)=="identity_conflict");assert(!b.current(good,T+5,W+5));
    assert(b.query(T+5,0,T+5,W+5).views.at("camera_front").reason=="identity_conflict");
  }
  {
    Buffer b(calibration());b.status(status(),T,W);
    for(int i=0;i<10;++i)assert(b.observe(frame(T+i),T+i,W+i)=="accepted");
    assert(b.sizes().at("camera_front")==8);assert(b.counters().at("capacity_drop")==2);
    assert(b.observe(frame(T+1),T+10,W+10)=="duplicate_or_old");
    auto bad=frame(T+11);bad.boxes[0].xmax=2000;
    assert(b.observe(bad,T+11,W+11)=="bad_box");
    bad=frame(T+12);bad.boxes[0].xmin=std::numeric_limits<float>::quiet_NaN();
    assert(b.observe(bad,T+12,W+12)=="bad_box");
    bad=frame(T+13);bad.calibration_id="wrong";
    assert(b.observe(bad,T+13,W+13)=="calibration_mismatch");
    bad=frame(T+14,"unknown");assert(b.observe(bad,T+14,W+14)=="unknown_camera");
    assert(b.observe(frame(T+100000000),T,W)=="source_stale_or_cutoff");
  }
  {
    Buffer b(calibration());b.status(status(),T,W);b.observe(frame(),T,W);auto candidate=first(b);
    // Paused ROS /clock does not keep inputs alive indefinitely.
    assert(!b.current(candidate,T,W+350000001));
    assert(b.query(T,0,T,W+350000001).views.at("camera_front").reason=="status_stale");
    b.status(status(T),T,W+350000002);
    assert(!b.current(candidate,T,W+350000002));
    assert(b.query(T,0,T,W+350000002).views.at("camera_front").candidates.empty());
    auto cutoff=status(T+1,2);cutoff.cutoff_ns=T+1;b.status(cutoff,T+1,W+350000003);
    auto o=frame(T+1);o.generation=2;
    assert(b.observe(o,T+1,W+350000003)=="source_stale_or_cutoff");
  }
  {
    Buffer b(calibration());b.status(status(),T,W);
    std::thread producer([&]{for(int i=0;i<2000;++i)b.observe(frame(T+i),T+i,W+i);});
    std::thread reader([&]{for(int i=0;i<2000;++i){b.query(T+i,10000,T+i,W+i);b.counters();}});
    producer.join();reader.join();assert(b.sizes().at("camera_front")<=8);
    auto q=b.query(T+1999,2000,T+1999,W+1999);b.close();
    if(!q.views.at("camera_front").candidates.empty())assert(!b.current(q.views.at("camera_front").candidates[0],T+1999,W+1999));
    assert(b.observe(frame(),T,W)=="closed");assert(b.sizes().at("camera_front")==0);
  }
  std::cout<<"camera buffer state/time/identity/bounds/concurrency passed\n";
}
