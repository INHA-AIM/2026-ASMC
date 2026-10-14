#include "Frenet/Geometry/ObservedMotion.hpp"
#include <iostream>
#include "Frenet/Generator/CandidateCombiner.hpp"
#include "Frenet/Generator/LongitudinalGenerator.hpp"
int main(){int fail=0;auto check=[&](bool b,const char* s){std::cout<<(b?"[PASS] ":"[FAIL] ")<<s<<'\n';fail+=!b;};
TrackedObject o{};o.motion_state=ObjectMotionState::DYNAMIC;o.vx=10;
ObservedObjectMotion straight;
for(int i=0;i<6;++i){o.x=i*.8;o.heading=-1+i*.2;straight.Observe(o,100+i*.08,1);}
check(straight.yaw_rate==0,"converging OBB angle cannot turn a straight-moving car");
const auto n=straight.samples.size();straight.Observe(o,100.4,1);check(straight.samples.size()==n,"repeated sensor frame is not another observation");
ObservedObjectMotion turn;const double times[]{0,.07,.19,.30,.42};
for(double t:times){o.vx=6*std::cos(.3*t);o.vy=6*std::sin(.3*t);o.x=20*std::sin(.3*t);o.y=20*(1-std::cos(.3*t));turn.Observe(o,200+t,1);}
check(std::fabs(turn.yaw_rate-.3)<1e-6,"real turn survives irregular sensor timestamps");
ObservedObjectMotion small_change;
for(double t:times){o.vx=10*std::cos(.04*t);o.vy=10*std::sin(.04*t);o.x=10*t;o.y=0;small_change.Observe(o,250+t,1);}
check(small_change.yaw_rate==0,"small course correction cannot become a sustained long-horizon turn");
ObservedObjectMotion drift;
for(double t:times){o.vx=6*std::cos(.3*t);o.vy=6*std::sin(.3*t);o.x=6*t;o.y=3*t;drift.Observe(o,300+t,1);}
check(drift.yaw_rate==0,"direction fit without matching position motion cannot certify a turn");
o.motion_state=ObjectMotionState::LOW_THREAT;turn.Observe(o,201,1);
check(turn.yaw_rate==0&&turn.samples.empty(),"confirmed stationary object clears turn history");
o.motion_state=ObjectMotionState::DYNAMIC;turn.Observe(o,202,1);
check(turn.yaw_rate==0&&turn.samples.size()==1,"long observation gap needs fresh turn evidence");
// Recorded counterexample: impossible acceleration target must not make
// the FOLLOW longitudinal search empty when a slower safe approach exists.
FrenetState start{};start.s=100;start.s_d=3.39;start.s_dd=-.26;
PathGeneratorConfig cfg{};cfg.time_horizon={1.5,5,.5};cfg.delta_s={-3,3,1};
cfg.delta_s_dot={0,0,1};cfg.lateral_d1={0,0,1};cfg.dt=.1;
KinematicLimits limits{};limits.max_longitudinal_accel=3;limits.max_longitudinal_jerk=6;
limits.max_lateral_accel=3;limits.max_curvature=.23;limits.low_speed_fallback_v=.5;
auto nominal=GenerateFollowingCandidates(start,135.53,9.52,0,1.5,5,14.5,.5,cfg);
FilterLongitudinalByKinematicLimits(nominal,limits);
check(std::none_of(nominal.begin(),nominal.end(),[](const auto& p){return p.valid;}),
      "recorded high acceleration target has no nominal feasible candidate");
RefLine ref;for(int i=0;i<=300;++i)ref.points.push_back({double(i),0,double(i),0,0,0});
ref.length=300;ref.closed_loop=false;
TrajectoryRequest request{};request.longitudinal_mode=LongitudinalMode::FOLLOWING;
request.target_speed=14.5;request.following={135.53,9.52,0,1.5,5,.5};
auto recovered=GenerateRequestedCandidates(start,request,ref,cfg,limits,{},
    {4.635,1.9,.79},{.45,.1,8});
check(std::any_of(recovered.begin(),recovered.end(),[](const auto& p){return p.valid;}),
      "intermediate target recovers a kinematically and collision checked FOLLOW candidate");
TrackedObject blocker{};blocker.id=99;blocker.geometry_valid=true;
blocker.motion_state=ObjectMotionState::LOW_THREAT;blocker.x=100;
for(int i=0;i<8;++i)blocker.corners[i]={i%4<2?98.0:104.0,i%2?-2.0:2.0,double(i/4)};
auto blocked=GenerateRequestedCandidates(start,request,ref,cfg,limits,{blocker},
    {4.635,1.9,.79},{.45,.1,8});
check(std::none_of(blocked.begin(),blocked.end(),[](const auto& p){return p.valid;}),
      "recovered FOLLOW candidates still reject a real overlapping obstacle");
return fail?1:0;}
