#include "Frenet/frenet_planner.hpp"
#include "Frenet/Evaluation/Cost.hpp"
#include "Frenet/Generator/SpatialStop.hpp"
#include "Frenet/Generator/LateralGenerator.hpp"
#include "Frenet/Generator/CartesianRetimer.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

namespace {
int checks=0,failures=0;
void expect(bool value,const char* label) {
    ++checks;if(!value){++failures;std::cerr<<"FAIL: "<<label<<'\n';}
}
RefLine straight() {
    RefLine ref;ref.length=400;
    for(double s=0;s<=400;s+=.5)ref.points.push_back({s,0,s,0,0,0});
    return ref;
}
FrenetPath cruise(double speed,double duration=2) {
    FrenetPath p{};p.valid=true;
    for(int i=0;i<=std::lround(duration/.1);++i) {
        p.t.push_back(i*.1);p.s.push_back(100+speed*i*.1);
        p.s_d.push_back(speed);p.s_dd.push_back(0);p.s_ddd.push_back(0);
        p.d.push_back(0);p.d_d.push_back(0);p.d_dd.push_back(0);
    }
    return p;
}
TrackedObject box(int id,double x,double y,double vx=0,double vy=0) {
    TrackedObject o;o.id=id;o.x=x;o.y=y;o.vx=vx;o.vy=vy;o.speed=std::hypot(vx,vy);
    o.length=4;o.width=1.8;o.height=1;o.motion_state=ObjectMotionState::DYNAMIC;
    for(int i=0;i<4;++i) {
        const double dx=i==0||i==3?-2:2,dy=i<2?-.9:.9;
        o.corners[i]={x+dx,y+dy,0};o.corners[i+4]={x+dx,y+dy,1};
    }
    o.geometry_valid=geometry::ValidateCorners8(o.corners).valid;return o;
}
bool clear(const RefLine& ref,const FrenetPath& p,const TrackedObject& o,
           const StoppingContinuationPolicy* policy) {
    std::vector<FrenetPath> paths{p};
    FilterByCollision(paths,ref,{o},{1.9,4.635,.79},{.45,.1,8},-1,-1,0,policy);
    return paths.front().valid;
}
}
int main() {
    const auto ref=straight();const auto prefix=cruise(5);
    const StoppingContinuationPolicy policy{5,6,.23,2.6};
    FrenetPath witness;
    expect(BuildStoppingSafetyPath(prefix,ref,policy,8,witness),"construct complete braking witness");
    expect(witness.t.back()>=8&&witness.s_d.back()<1e-8,"witness reaches zero and retains the prediction horizon");
    expect(witness.s.back()<120,"stopping witness does not invent eight seconds of coasting");
    bool bounded=true;
    for(std::size_t i=prefix.t.size();i<witness.t.size();++i) {
        bounded&=witness.s_d[i]>=0&&std::fabs(witness.s_dd[i])<=5+1e-8;
        bounded&=std::fabs(witness.s_dd[i]-witness.s_dd[i-1])<=6*.1+1e-7;
    }
    expect(bounded,"continuation obeys braking, jerk and forward-motion limits");
    expect(prefix.s.back()==110&&prefix.s_d.back()==5,"proof does not modify the executable prefix");
    expect(!clear(ref,prefix,box(1,138,0),nullptr),"legacy coast rejects a distant hypothetical conflict");
    expect(clear(ref,prefix,box(1,138,0),&policy),"advance with a verified later stop remains available");
    expect(!clear(ref,prefix,box(2,108,0),&policy),"collision inside executed motion remains forbidden");
    expect(!clear(ref,prefix,box(3,118,0),&policy),"insufficient remaining braking room remains forbidden");
    expect(!clear(ref,prefix,box(4,111,4,0,-2),&policy),"near lateral intrusion remains forbidden");
    expect(!clear(ref,prefix,box(5,130,0,-10,0),&policy),"approaching traffic remains forbidden");
    auto invalid=box(6,138,0);invalid.geometry_valid=false;
    expect(!clear(ref,prefix,invalid,&policy),"invalid object geometry cannot gain a safe certificate");
    auto invalid_policy=policy;invalid_policy.jerk=0;
    expect(!BuildStoppingSafetyPath(prefix,ref,invalid_policy,8,witness),"invalid braking constraints fail closed");
    auto unsettled=prefix;unsettled.d_d.back()=.3;
    expect(!BuildStoppingSafetyPath(unsettled,ref,policy,8,witness),"unfinished time lateral motion is not replaced by a new curve");
    auto optional=policy;optional.allow_coast_alternative=true;
    expect(clear(ref,unsettled,box(7,200,0),&optional),"an existing safe continuation is not removed by an infeasible alternative stop");
    expect(clear(ref,prefix,box(1,138,0),&optional),"coast or brake policy can rescue a speculative coast collision");
    expect(!clear(ref,prefix,box(2,108,0),&optional),"alternative proof never rescues a collision in the actual prefix");
    auto spatial=SampleSpatialLateralMotion(prefix,{true,100,30,MakeQuintic(.2,.01,0,0,0,0,30)});
    expect(BuildStoppingSafetyPath(spatial,ref,policy,8,witness),"partial spatial motion continues its original curve during braking");
    const auto stop=ConvertToCartesianPath(witness,ref);
    const double q=std::min(30.0,witness.s.back()-100);
    const double expected_yaw=std::atan(EvalVel(spatial.spatial_lateral.polynomial,q));
    expect(std::fabs(stop.yaw.back()-expected_yaw)<1e-8,"zero-speed proof preserves spatial heading");

    std::vector<FrenetPath> choices{cruise(0),cruise(4),cruise(8),cruise(12)};
    EvaluateApproachCosts(choices,8,3,6,0);
    const auto* best=SelectBestPath(choices);
    expect(best&&best->s_d.back()==8,"cost prefers useful approach speed among safe candidates");
    choices[2].valid=false;best=SelectBestPath(choices);
    expect(best&&best->s_d.back()>0,"loss of preferred candidate does not force a stop");
    choices={cruise(8,1.5),cruise(8,5)};
    EvaluateApproachCosts(choices,8,3,6,0);
    expect(SelectBestPath(choices)==&choices[1],"cost normalization does not reward a short horizon by hiding its integral");
    FrenetState moving{};moving.s=100;moving.s_d=5;
    auto impossible=GenerateBrakingContinuation(moving,0,6,.1);
    expect(!impossible.valid,"braking primitive rejects invalid physical limits");
    moving.s_d=.1;moving.s_dd=-3;
    expect(!GenerateBrakingContinuation(moving,5,6,.1).valid,"unreleasable negative acceleration is not silently reset");
    FrenetPlanner planner;
    expect(planner.Init(FRENET_TEST_CONFIG),"load production constraints");
    expect(planner.SetPreparedReference(ref),"prepare isolated reference");
    const auto limits=planner.kinematic_limits();
    PathGeneratorConfig grid{};grid.dt=.1;grid.delta_s={0,0,0};grid.time_horizon={1.5,5,1.5};
    moving={};moving.s=100;
    const auto legacy=GenerateSpatialStoppingCandidates(moving,200,8,2,grid,limits);
    const auto partial=GenerateSpatialStoppingCandidates(moving,200,8,2,grid,limits,0,SpatialStopOptions{false,true});
    expect(legacy.empty()&&partial.size()==3,"partial profile sampling is explicit; existing STOP defaults stay intact");
    expect(partial.front().s_d.back()>0,"a stopped approach can regenerate an accelerating candidate");
    CartesianPath curve;
    for(int i=0;i<=240;++i) {
        const double s=i*.5;
        curve.x.push_back(100*std::sin(s/100));curve.y.push_back(100*(1-std::cos(s/100)));
        curve.yaw.push_back(s/100);curve.kappa.push_back(.01);
        curve.v.push_back(500);curve.a.push_back(500);
    }
    PathGeneratorConfig timing{};timing.dt=.1;timing.time_horizon={5,5,0};timing.delta_s_dot={0,0,0};
    CurveSpeedConfig speed_policy{20,10,.8,.5,.5};CartesianPath timed;FrenetPath physical;
    expect(RetimeCartesianGeometry(curve,10,0,20,timing,limits,speed_policy,timed,physical),
        "arbitrary geometry uses the shared physical-distance speed envelope");
    bool feasible=!timed.v.empty();
    for(std::size_t i=0;i<timed.v.size();++i)
        feasible&=timed.v[i]*timed.v[i]*std::fabs(timed.kappa[i])<=limits.max_lateral_accel+1e-6&&
            std::fabs(timed.a[i])<=limits.max_longitudinal_accel+1e-6&&
            std::fabs(physical.s_ddd[i])<=limits.max_longitudinal_jerk+1e-6;
    expect(feasible,"retimed speed, acceleration and jerk satisfy actual geometry constraints");
    expect(!timed.v.empty()&&timed.v.front()==10&&timed.v.back()<20,
        "old geometry velocity metadata cannot bypass curve-dependent speed planning");
    std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
