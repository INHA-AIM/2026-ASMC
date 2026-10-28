#include "Frenet/Generator/SpatialStop.hpp"
#include <cmath>
#include <iostream>

int main(){
    int checks=0,failures=0;const auto expect=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::cerr<<"FAIL: "<<label<<'\n';}};
    KinematicLimits limits{};limits.max_longitudinal_accel=5;limits.max_longitudinal_jerk=6;
    limits.boundary_condition_accel_fraction=.8;
    PathGeneratorConfig grid{};grid.dt=.05;grid.delta_s={0,0,0};grid.time_horizon={1,3,.5};
    FrenetState moving{};moving.s_d=8;
    const auto complete=GenerateSpatialStoppingCandidates(moving,100,8,1.5,grid,limits);
    SpatialStopOptions partial;partial.allow_partial_horizons=true;
    const auto approach=GenerateSpatialStoppingCandidates(moving,100,8,1.5,grid,limits,0,partial);
    expect(!complete.empty()&&approach.size()>complete.size(),"highway partial horizon policy preserves additional checked approach prefixes");
    FrenetState braking{};braking.s_d=4.37;braking.s_dd=-3.64;
    grid.time_horizon={1,20,1};
    SpatialStopOptions only;only.braking_only=true;
    const auto full=GenerateSpatialStoppingCandidates(braking,9.24,6.94,1.5,grid,limits,.5,only);
    expect(!full.empty(),"main braking-only policy retains a complete feasible stop");
    bool monotone=!full.empty();
    for(const auto& p:full)for(std::size_t i=0;i<p.s.size();++i){
        monotone&=p.s_dd[i]<=1e-7&&p.s[i]<=9.24+1e-6&&p.s_d[i]>=-1e-8;
        if(i)monotone&=p.s_d[i]<=p.s_d[i-1]+1e-7;
    }
    expect(monotone,"braking-only does not reopen acceleration or overshoot the target");
    only.allow_partial_horizons=true;
    const auto combined=GenerateSpatialStoppingCandidates(braking,9.24,6.94,1.5,grid,limits,.5,only);
    expect(combined.size()>full.size(),"sampling and acceleration policies are independent");
    bool safe=!combined.empty();for(const auto& p:combined)for(double a:p.s_dd)safe&=a<=1e-7;
    expect(safe,"partial sampling cannot bypass braking-only ownership");
    std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
