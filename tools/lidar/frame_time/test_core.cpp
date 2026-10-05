#include "core.hpp"
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace asmc::lidar_eval;
static void check(bool value){if(!value)throw std::runtime_error("assertion failed");}
static void close(const Vec& a,const Vec& b){check((a-b).norm()<1e-6);}
static Observation time_at(Ns stamp=100500000000LL) {
  return {stamp,stamp+1000000,stamp+120000000,"ros_receive","bridge_receive_not_ray_time"};
}
static PoseSample sample(Ns stamp) {
  PoseSample p;p.stamp_ns=stamp;p.arrival_ns=stamp+1000000;p.clock="ros_receive";p.valid=true;
  p.map_from_reference.linear()=rpy(.12,-.07,.61);p.map_from_reference.translation()=Vec(12,-7,1);return p;
}
static Context context() {
  Context c;c.time=time_at();c.calibration_source="v3_sha";
  c.base_from_sensor.translation()=Vec(.8,0,1.45);
  c.twist_arrival_ns=c.time.receipt_ns;
  c.map_pose.query_stamp_ns=c.time.stamp_ns;c.map_pose.clock=c.time.clock;
  c.map_pose.valid=true;c.map_pose.reason="valid";c.map_pose.map_from_base=sample(c.time.stamp_ns).map_from_reference;return c;
}
static Object obj() {
  Object o;o.id=42;o.classification="unknown";o.center=Vec(5,1.2,-1.7);
  for(int i=0;i<8;++i)o.corners[i]=o.center+Vec(i&1?1.:-1.,i&2?.9:-.9,i&4?.75:-.75);
  return o;
}
int main() {
  const std::vector<std::pair<std::string,std::function<void()>>> tests={
    {"mount_center_and_corners",[]{auto c=context();auto o=obj();auto r=transform(o,c);check(r.valid);close(r.base.center,Vec(5.8,1.2,-.25));for(int i=0;i<8;++i)close(r.base.corners[i],o.corners[i]+Vec(.8,0,1.45));}},
    {"rpy_nonzero_mount_and_roundtrip",[]{auto c=context();c.base_from_sensor.linear()=rpy(.13,-.17,.21);auto o=obj();auto r=transform(o,c);auto t=c.map_pose.map_from_base*c.base_from_sensor;for(int i=0;i<8;++i)close(t.inverse()*r.map->corners[i],o.corners[i]);close(t.inverse()*r.map->center,o.center);}},
    {"undo_exact_roll_pitch_leveling",[]{auto c=context();auto o=obj();const auto reference=transform(o,c);c.frame=Frame::LeveledSensor;c.applied_leveling=rpy(.2,-.15,0);c.leveling_source="exact_snapshot";o.center=c.applied_leveling*o.center;o.long_axis=c.applied_leveling*o.long_axis;for(auto& p:o.corners)p=c.applied_leveling*p;auto r=transform(o,c);close(r.map->center,reference.map->center);for(int i=0;i<8;++i)close(r.map->corners[i],reference.map->corners[i]);}},
    {"pitch_only_is_distinct_from_roll_pitch",[]{auto c=context();auto o=obj();auto ref=transform(o,c);c.frame=Frame::LeveledSensor;c.applied_leveling=rpy(0,.3,0);c.leveling_source="exact_pitch";o.center=c.applied_leveling*o.center;o.long_axis=c.applied_leveling*o.long_axis;for(auto& p:o.corners)p=c.applied_leveling*p;close(transform(o,c).base.center,ref.base.center);}},
    {"unproven_leveling_rejected",[]{auto c=context();c.frame=Frame::LeveledSensor;check(!transform(obj(),c).valid);}},
    {"bad_rotation_rejected",[]{auto c=context();c.base_from_sensor.linear()(0,0)=2;check(!transform(obj(),c).valid);}},
    {"calibration_required",[]{auto c=context();c.calibration_source="";check(!transform(obj(),c).valid);}},
    {"gps_lever_arm_once",[]{auto t=time_at();auto p=sample(t.stamp_ns);Pose expected=p.map_from_reference;p.reference_is_gps=true;p.gps_in_base=Vec(.35,0,1.25);p.map_from_reference.translation()+=p.map_from_reference.rotation()*p.gps_in_base;PoseHistory h;check(h.append(p).empty());check((h.at(t).map_from_base.matrix()-expected.matrix()).norm()<1e-9);}},
    {"base_pose_no_antenna_subtraction",[]{auto t=time_at();auto p=sample(t.stamp_ns);p.gps_in_base=Vec(.35,0,1.25);PoseHistory h;h.append(p);close(h.at(t).map_from_base.translation(),p.map_from_reference.translation());}},
    {"interpolation_slerp_and_position",[]{auto t=time_at();auto a=sample(t.stamp_ns-40000000),b=sample(t.stamp_ns+40000000);a.map_from_reference=Pose::Identity();b.map_from_reference.linear()=rpy(0,0,.4);b.map_from_reference.translation()=Vec(2,4,6);PoseHistory h;h.append(a);h.append(b);auto r=h.at(t);check(r.valid);close(r.map_from_base.translation(),Vec(1,2,3));check((r.map_from_base.rotation()-rpy(0,0,.2)).norm()<1e-9);}},
    {"future_arrival_excluded",[]{auto t=time_at();PoseHistory h;auto a=sample(t.stamp_ns-10000000),b=sample(t.stamp_ns+10000000);b.arrival_ns=t.processing_ns+1;h.append(a);h.append(b);check(!h.at(t).valid);}},
    {"no_extrapolation",[]{auto t=time_at();PoseHistory h;h.append(sample(t.stamp_ns-1));check(!h.at(t).valid);}},
    {"100ms_boundary_inclusive",[]{auto t=time_at();PoseHistory h;h.append(sample(t.stamp_ns-50000000));h.append(sample(t.stamp_ns+50000000));check(h.at(t).valid);}},
    {"over100ms_rejected",[]{auto t=time_at();PoseHistory h;h.append(sample(t.stamp_ns-50000001));h.append(sample(t.stamp_ns+50000000));check(h.at(t).reason=="pose_gap_exceeded");}},
    {"invalid_pose_is_barrier",[]{auto t=time_at();PoseHistory h;auto p=sample(t.stamp_ns);p.valid=false;h.append(sample(t.stamp_ns-100));h.append(p);h.append(sample(t.stamp_ns+100));check(h.at(t).reason=="pose_invalid");}},
    {"nan_pose_rejected_local_retained",[]{auto t=time_at();auto p=sample(t.stamp_ns);p.map_from_reference.translation().x()=std::numeric_limits<double>::quiet_NaN();PoseHistory h;h.append(p);auto c=context();c.map_pose=h.at(t);auto r=transform(obj(),c);check(r.valid && !r.map);}},
    {"clock_mismatch_no_map",[]{auto t=time_at();auto p=sample(t.stamp_ns);p.clock="sim";PoseHistory h;h.append(p);check(!h.at(t).valid);}},
    {"pose_duplicate_rejected",[]{auto t=time_at();PoseHistory h;h.append(sample(t.stamp_ns));h.append(sample(t.stamp_ns));check(h.at(t).reason=="duplicate_pose_stamp");}},
    {"arrival_regression_rejected",[]{PoseHistory h;h.append(sample(100));check(h.append(sample(90))=="arrival_regression");}},
    {"nanosecond_stamp_preserved",[]{auto c=context();c.time=time_at(1790415998913611401LL);check(transform(obj(),c).time.stamp_ns==1790415998913611401LL);}},
    {"stamp_guard_zero_duplicate_regression",[]{StampGuard g;check(g.accept(time_at(0))=="zero_or_negative_stamp");check(g.accept(time_at(100)).empty());check(g.accept(time_at(100))=="duplicate_stamp");check(g.accept(time_at(99))=="regressed_stamp");check(g.accept(time_at(101)).empty());}},
    {"unknown_clock_and_processing_before_receipt",[]{auto t=time_at();t.clock="unknown";check(!validate_time(t).empty());t=time_at();t.processing_ns=t.receipt_ns-1;check(validate_time(t)=="processing_before_receipt");}},
    {"absolute_vector_no_ego_addition",[]{auto c=context();c.ego_linear_base=Vec(100,100,100);auto o=obj();o.velocity_meaning=VelocityMeaning::AbsoluteInputVector;o.velocity=Vec(3,2,1);auto r=transform(o,c);close(*r.absolute_velocity_base,o.velocity);close(*r.absolute_velocity_map,c.map_pose.map_from_base.rotation()*o.velocity);}},
    {"relative_derivative_rotation_term",[]{auto c=context();c.frame=Frame::Base;c.twist_valid=true;c.twist_stamp_ns=c.time.stamp_ns;c.twist_clock=c.time.clock;c.ego_linear_base=Vec(2,0,0);c.ego_angular_base=Vec(0,0,.1);auto o=obj();o.velocity_meaning=VelocityMeaning::BasePositionDerivative;o.center=Vec(5.8,1.2,-.25);auto r=transform(o,c);close(*r.absolute_velocity_base,Vec(1.88,.58,0));}},
    {"stationary_object_relative_derivative",[]{auto c=context();c.frame=Frame::Base;c.twist_valid=true;c.twist_stamp_ns=c.time.stamp_ns;c.twist_clock=c.time.clock;c.ego_linear_base=Vec(5,1,0);c.ego_angular_base=Vec(.1,.2,.3);auto o=obj();o.velocity_meaning=VelocityMeaning::BasePositionDerivative;o.velocity=-c.ego_linear_base-c.ego_angular_base.cross(o.center);close(*transform(o,c).absolute_velocity_base,Vec::Zero());}},
    {"unknown_velocity_not_zero",[]{auto r=transform(obj(),context());check(r.valid && !r.absolute_velocity_base && !r.absolute_velocity_map);}},
    {"map_pose_cannot_be_reused_at_wrong_time",[]{auto c=context();--c.map_pose.query_stamp_ns;auto r=transform(obj(),c);check(r.valid && !r.map);}},
    {"future_twist_not_used",[]{auto c=context();c.frame=Frame::Base;c.twist_valid=true;c.twist_stamp_ns=c.time.stamp_ns;c.twist_clock=c.time.clock;c.twist_arrival_ns=c.time.processing_ns+1;auto o=obj();o.velocity_meaning=VelocityMeaning::BasePositionDerivative;check(!transform(o,c).absolute_velocity_base);}},
    {"missing_or_stale_twist_unknown",[]{auto c=context();c.frame=Frame::Base;auto o=obj();o.velocity_meaning=VelocityMeaning::BasePositionDerivative;check(!transform(o,c).absolute_velocity_base);c.twist_valid=true;c.twist_stamp_ns=c.time.stamp_ns-1;check(!transform(o,c).absolute_velocity_base);}},
    {"leveled_derivative_not_relabelled",[]{auto o=obj();o.velocity_meaning=VelocityMeaning::BasePositionDerivative;check(transform(o,context()).velocity_reason=="derivative_requires_base_input");}},
    {"obb_shape_id_class_preserved",[]{auto c=context();auto o=obj();auto r=transform(o,c);check(r.id==42 && r.classification=="unknown");for(int i=0;i<8;++i)for(int j=0;j<8;++j)check(std::abs((o.corners[i]-o.corners[j]).norm()-(r.map->corners[i]-r.map->corners[j]).norm())<1e-6);}},
    {"point_count_intensity_roundtrip",[]{auto c=context();std::vector<Point> p;for(int i=0;i<1000;++i)p.push_back({Vec(i*.02,-i*.003,-1.8),float(i%256)});auto t=c.map_pose.map_from_base*c.base_from_sensor;auto out=transform_points(p,t);auto back=transform_points(out,t.inverse());check(out.size()==p.size());for(std::size_t i=0;i<p.size();++i){close(back[i].xyz,p[i].xyz);check(!std::memcmp(&p[i].intensity,&out[i].intensity,sizeof(float)));}}},
    {"vertical_axis_has_no_yaw",[]{auto o=obj();o.long_axis=Vec::UnitZ();check(!transform(o,context()).base.yaw);}},
    {"no_pose_does_not_drop_local_object",[]{auto c=context();c.map_pose=PoseSelection();auto r=transform(obj(),c);check(r.valid && !r.map && r.id==42);}}
  };
  int failures=0;for(const auto& [name,test]:tests)try{test();std::cout<<"PASS "<<name<<'\n';}catch(const std::exception& e){++failures;std::cerr<<"FAIL "<<name<<": "<<e.what()<<'\n';}
  std::cout<<tests.size()<<" tests, "<<failures<<" failures\n";return failures?1:0;
}
