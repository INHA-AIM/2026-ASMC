#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Frenet/Generator/CandidateCombiner.hpp"
#include "Frenet/Pipeline/TrajectoryRequest.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

int main() {
    int failures=0;
    const auto expect=[&](bool ok,const char* text){std::cout<<(ok?"PASS ":"FAIL ")<<text<<'\n';failures+=!ok;};
    PathGeneratorConfig cfg{};
    cfg.lateral_d1={0,0,1};cfg.time_horizon={1.5,5,.5};
    cfg.delta_s={-3,3,1};cfg.delta_s_dot={-3,3,1};cfg.dt=.1;
    KinematicLimits limits{};limits.max_longitudinal_accel=3;limits.max_longitudinal_jerk=6;
    limits.max_lateral_accel=2.6;limits.max_curvature=.23;limits.low_speed_fallback_v=.5;
    limits.boundary_condition_accel_fraction=.8;limits.max_forward_accel=1.5;
    FrenetState start{};start.s=20;start.s_d=16.3402;start.s_dd=-.80178;
    const double leader=start.s+12.0928,lv=14.9401,tau=1.5,d0=5;
    auto exact=GenerateFollowingCandidates(start,leader,lv,0,tau,d0,17.8679,.3,cfg);
    FilterLongitudinalByKinematicLimits(exact,limits);
    expect(exact.size()==56&&std::none_of(exact.begin(),exact.end(),[](auto& p){return p.valid;}),
        "recorded short-gap endpoint rejects all 56 exact candidates without relaxing limits");
    auto recovered=GenerateFollowingRecoveryCandidates(start,leader,lv,0,tau,d0,17.8679,.3,cfg,limits);
    FilterLongitudinalByKinematicLimits(recovered,limits);
    bool valid=!recovered.empty(),continuous=true,safer=true;
    for(const auto& p:recovered) {
        valid=valid&&p.valid;
        continuous=continuous&&std::abs(p.s.front()-start.s)<1e-9&&
            std::abs(p.s_d.front()-start.s_d)<1e-9&&std::abs(p.s_dd.front()-start.s_dd)<1e-9;
        const double gap_end=leader+lv*p.t.back()-p.s.back();
        safer=safer&&gap_end>leader-start.s&&p.s_d.back()<start.s_d;
        for(std::size_t i=1;i<p.t.size();++i) {
            const double h=p.t[i]-p.t[i-1],j=p.s_ddd[i];
            continuous=continuous&&std::abs(p.s_d[i]-p.s_d[i-1]-p.s_dd[i-1]*h-.5*j*h*h)<1e-8;
        }
    }
    expect(valid,"gradual gap recovery obeys existing acceleration, jerk and no-reverse filters");
    expect(continuous,"recovery preserves the measured start and integrated derivatives");
    expect(safer,"recovery decelerates and increases the gap instead of forcing full recovery at T");
    RefLine ref;ref.length=300;
    for(int i=0;i<=600;++i)ref.points.push_back({i*.5,0,i*.5,0,0,0});
    TrajectoryRequest request;request.longitudinal_mode=LongitudinalMode::FOLLOWING;
    request.target_speed=17.8679;request.following={leader,lv,0,tau,d0,.3};
    const VehicleShape shape{1.9,4.635,.79};const CollisionCheckConfig collision{.3,.1,3};
    auto paths=GenerateRequestedCandidates(start,request,ref,cfg,limits,{},shape,collision);
    expect(std::any_of(paths.begin(),paths.end(),[](auto& p){return p.valid;}),
        "normal FOLLOW pipeline uses recovery when exact endpoints are unreachable");
    TrackedObject obstacle{};obstacle.id=7;obstacle.x=start.s+2;obstacle.y=0;
    obstacle.motion_state=ObjectMotionState::LOW_THREAT;obstacle.geometry_valid=true;obstacle.z_valid=true;
    for(unsigned i=0;i<8;++i)obstacle.corners[i]={start.s+(i&1?4.0:0.0),i&2?1.0:-1.0,i&4?1.5:0.0};
    paths=GenerateRequestedCandidates(start,request,ref,cfg,limits,{obstacle},shape,collision);
    expect(std::none_of(paths.begin(),paths.end(),[](auto& p){return p.valid;}),
        "recovery candidates still fail the common physical collision check");
    expect(GenerateFollowingRecoveryCandidates(start,start.s+60,lv,0,tau,d0,20,.3,cfg,limits).empty(),
        "far-gap acceleration does not enter short-gap recovery");
    expect(GenerateFollowingRecoveryCandidates(start,start.s-1,lv,0,tau,d0,20,.3,cfg,limits).empty(),
        "a rear object cannot become a recovery leader");
    auto impossible=start;impossible.s_dd=-3.1;
    expect(GenerateFollowingRecoveryCandidates(impossible,leader,lv,0,tau,d0,20,.3,cfg,limits).empty(),
        "invalid acceleration boundaries are not silently repaired");
    std::cout<<"failures="<<failures<<'\n';return failures?1:0;
}
