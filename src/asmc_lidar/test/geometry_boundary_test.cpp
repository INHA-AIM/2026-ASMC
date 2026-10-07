#include <asmc_lidar/projection.hpp>
#include <asmc_lidar/bounded_pose.hpp>
#include <iostream>
#include <stdexcept>

using namespace asmc::lidar;
void require(bool ok, const char* why) {if(!ok) throw std::runtime_error(why);}
int main() {
  Context c;
  c.time={1000000000,1000000000,1000000000,"ros","bridge_receive"};
  c.frame=Frame::LeveledSensor;
  c.base_from_sensor.translation()=Vec(.8,0,1.45);
  c.base_from_sensor.linear()=rpy(.04,-.07,.11);
  c.calibration_source="fixture"; c.leveling_source="actual_preprocess";
  c.applied_leveling=leveling_rotation(.15,.09,c.base_from_sensor.rotation());
  Pose camera=Pose::Identity(); camera.translation()=Vec(1.9,0,1.2);
  camera.linear()=rpy(.02,.035,-.03);
  Eigen::Matrix3d k; k<<640,0,640,0,640,360,0,0,1;
  const Vec raw(12,-2,-.8), leveled=c.applied_leveling*raw;
  const auto projection=camera_projection(c,camera,k);
  require(bool(projection),"projection missing");
  Context raw_context=c; raw_context.frame=Frame::Sensor;
  const auto raw_projection=camera_projection(raw_context,camera,k);
  const Vec direct=*raw_projection*raw.homogeneous();
  const Vec corrected=*projection*leveled.homogeneous();
  require((direct-corrected).norm()<1e-9,"composed projection mismatch");
  require((*raw_projection*leveled.homogeneous()-direct).norm()>.1,"missing inverse negative control");
  Context duplicate=c; duplicate.applied_leveling=c.applied_leveling*c.applied_leveling;
  require((*camera_projection(duplicate,camera,k)*leveled.homogeneous()-direct).norm()>.1,"double inverse negative control");
  Context later=c; later.applied_leveling=leveling_rotation(-.2,.1,c.base_from_sensor.rotation());
  require((*camera_projection(later,camera,k)*leveled.homogeneous()-direct).norm()>.1,"wrong scan negative control");
  require((leveling_rotation(.15,.09,Rot::Identity(),false)-rpy(0,.09,0)).norm()<1e-12,"pitch-only mismatch");
  require(((*base_from_input(c))*leveled-c.base_from_sensor*raw).norm()<1e-10,"base mismatch");
  Object object; object.center=leveled; object.long_axis=c.applied_leveling*Vec::UnitX();
  for(unsigned i=0;i<8;++i) object.corners[i]=c.applied_leveling*(raw+Vec(i&1,(i>>1)&1,(i>>2)&1));
  const auto transformed=transform(object,c); require(transformed.valid,"object transform");
  for(unsigned i=0;i<8;++i) require((transformed.base.corners[i]-c.base_from_sensor*(raw+Vec(i&1,(i>>1)&1,(i>>2)&1))).norm()<1e-10,"corner transform");

  BoundedPoseHistory history;
  PoseSample sample; sample.clock="ros"; sample.valid=true;
  sample.reference_is_gps=true; sample.gps_in_base=Vec(.35,0,1.25);
  sample.map_from_reference.translation()=Vec(100,200,5);
  for(Ns t : {200000000LL,400000000LL,600000000LL}) {
    sample.stamp_ns=sample.arrival_ns=t; require(history.append(sample).empty(),"append");
  }
  Observation time{900000000,900000000,900000000,"ros","bridge_receive"};
  const auto edge=history.at(time,Rot::Identity(),Vec(10,0,0));
  require(edge.valid && std::abs(edge.map_from_base.translation().x()-102.65)<1e-10,"300ms inclusive prediction");
  ++time.stamp_ns; ++time.receipt_ns; ++time.processing_ns;
  require(history.at(time,Rot::Identity(),Vec::Zero()).reason=="gps_stale","300ms exceeded");
  sample.stamp_ns=sample.arrival_ns=800000000; sample.valid=false; history.append(sample);
  require(history.at(time,Rot::Identity(),Vec::Zero()).reason=="gps_invalid","invalid barrier");
  sample.valid=true;
  for(Ns t : {1000000000LL,1200000000LL,1400000000LL}) {
    sample.stamp_ns=sample.arrival_ns=t; history.append(sample);
    time={t,t,t,"ros","bridge_receive"};
    require(history.at(time,Rot::Identity(),Vec::Zero()).valid==(t==1400000000),"recovery three samples");
  }
  sample.stamp_ns=1300000000; sample.arrival_ns=1500000000; sample.valid=false; history.append(sample);
  time={1500000000,1500000000,1500000000,"ros","bridge_receive"};
  require(history.at(time,Rot::Identity(),Vec::Zero()).valid,"late old invalid barrier");
  sample.stamp_ns=sample.arrival_ns=2000000000; sample.valid=true; history.append(sample);
  time={2000000000,2000000000,2000000000,"ros","bridge_receive"};
  require(history.at(time,Rot::Identity(),Vec::Zero()).reason=="gps_recovering","silence recovery");
  BoundedPoseHistory lever_history;
  sample.valid=true;sample.gps_in_base=Vec(1,0,0);sample.reference_is_gps=true;
  sample.map_from_reference.linear()=rpy(0,0,.2);
  for(Ns t:{2200000000LL,2400000000LL,2600000000LL}) {
    sample.stamp_ns=sample.arrival_ns=t;lever_history.append(sample);
  }
  time={2700000000LL,2700000000LL,2700000000LL,"ros","bridge_receive"};
  const auto lever=lever_history.at(time,rpy(0,0,.5),Vec::Zero());
  require(lever.valid && (lever.map_from_base.translation()-
      (sample.map_from_reference.translation()-rpy(0,0,.2)*Vec(1,0,0))).norm()<1e-10,
      "antenna lever used later scan attitude");
  std::cout<<"geometry and bounded pose boundary tests passed\n";
}
