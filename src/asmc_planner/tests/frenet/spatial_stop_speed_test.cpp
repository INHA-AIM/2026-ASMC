#include "Frenet/Generator/SpatialStop.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
int failures=0;
void Expect(bool ok,const std::string& name) {
    std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';
    failures+=!ok;
}
KinematicLimits Limits() {
    KinematicLimits l{};
    l.max_longitudinal_accel=5;l.max_longitudinal_jerk=6;
    l.boundary_condition_accel_fraction=.8;
    return l;
}
PathGeneratorConfig Grid(double horizon=20) {
    PathGeneratorConfig g{};g.dt=.05;
    g.delta_s={0,0,0};g.time_horizon={horizon,horizon,0};return g;
}
std::vector<FrenetPath> Generate(double v,double a,double distance,
                                double cruise=11.111,double horizon=20) {
    FrenetState x{};x.s_d=v;x.s_dd=a;
    return GenerateSpatialStoppingCandidates(x,distance,cruise,1.5,Grid(horizon),Limits());
}
bool Safe(const FrenetPath& p,double target) {
    for(std::size_t i=0;i<p.s.size();++i) {
        if(!std::isfinite(p.s[i]) || p.s[i]>target+1e-6 ||
           p.s_d[i]<-1e-9 || std::fabs(p.s_dd[i])>4+1e-7 ||
           std::fabs(p.s_ddd[i])>4.8+1e-7 ||
           (i && p.s[i]<p.s[i-1]-1e-8))return false;
    }
    return true;
}
} // namespace

int main() {
    auto paths=Generate(10.85,-.4,27.4);
    Expect(!paths.empty(),"bag first STOP: feasible distance profile exists");
    if(paths.empty())return 1;
    Expect(paths[0].s_dd[10]<-1.8,"bag first STOP: strong braking starts within 0.5s");
    Expect(std::fabs(paths[0].s.back()-27.4)<1e-5 && paths[0].s_d.back()<1e-7,
           "stop endpoint is reached at zero speed without overshoot");
    Expect(Safe(paths[0],27.4),"speed, acceleration and jerk stay physically admissible");
    auto longer=Generate(10.85,-.4,27.4,11.111,5);
    auto shorter=Generate(10.85,-.4,27.4,11.111,1.5);
    bool same=!longer.empty()&&!shorter.empty();
    if(same)for(std::size_t i=0;i<shorter[0].s.size();++i)
        same &= std::fabs(shorter[0].s[i]-longer[0].s[i])<1e-10 &&
                std::fabs(shorter[0].s_dd[i]-longer[0].s_dd[i])<1e-10;
    Expect(same,"longer planning horizon cannot postpone the braking prefix");

    auto strong=Generate(6,-3.5,6);
    Expect(!strong.empty() && strong[0].s_dd.front()==-3.5,
           "achieved strong braking is preserved at the next plan boundary");
    FrenetState red_replan{};
    red_replan.s_d=4.37;red_replan.s_dd=-3.64;
    auto released=GenerateSpatialStoppingCandidates(red_replan,9.24,6.94,1.5,
        Grid(20),Limits(),.5,SpatialStopOptions{true,false});
    bool released_safe=!released.empty();
    if(released_safe) {
        const auto& path=released.front();
        released_safe=Safe(path,9.24) &&
            std::fabs(path.s.back()-9.24)<1e-4 && path.s_d.back()<1e-7;
        for(std::size_t i=0;i<path.s_d.size();++i) {
            released_safe &= path.s_dd[i]<=1e-7;
            if(i)released_safe &= path.s_d[i]<=path.s_d[i-1]+1e-7;
        }
    }
    Expect(released_safe,
           "red-stop measured braking releases within jerk bound and reaches the target without reacceleration");
    Expect(Generate(8,0,3).empty(),"physically impossible target is rejected instead of moving it");
    auto far=Generate(5,0,60,5);
    Expect(!far.empty() && std::fabs(far[0].s_d[10]-5)<1e-9,
           "distant STOP approaches at cruise rather than braking to an early halt");
    auto restart=Generate(0,0,4,5);
    Expect(!restart.empty() && restart[0].s_d[10]>.1 && Safe(restart[0],4),
           "stopped vehicle can safely approach a still-distant target");
    auto small=Generate(.1,0,.05,1);
    Expect(!small.empty() && Safe(small[0],.05),"low-speed endpoint remains forward-only");
    Expect(Generate(5,std::numeric_limits<double>::quiet_NaN(),20).empty(),
           "invalid acceleration does not generate a path");

    bool grid_safe=true;int accepted=0;
    for(double v:{0.,.2,1.,4.,8.,12.})
    for(double a:{-2.,0.,1.})
    for(double distance:{.1,2.,8.,25.,60.}) {
        auto p=Generate(v,a,distance,8);
        if(p.empty())continue;
        ++accepted;grid_safe &= Safe(p[0],distance);
        // Twenty seconds is sufficient for these bounded test distances.
        grid_safe &= std::fabs(p[0].s.back()-distance)<1e-5 && p[0].s_d.back()<1e-7;
    }
    Expect(accepted>40 && grid_safe,"state/distance grid preserves the fixed endpoint and hard limits");

    // Replan the first 50 ms repeatedly, as a controller would consume them.
    // This is a kinematic closed loop, not a MORAI vehicle model.
    bool rolled=true;double s=0,v=10.85,a=-.4;
    for(int i=0;i<600 && v>.02;++i) {
        auto p=Generate(v,a,27.4-s,11.111,1.5);
        if(p.empty()) {rolled=false;break;}
        s+=p[0].s[1];v=p[0].s_d[1];a=p[0].s_dd[1];
    }
    Expect(rolled && v<.02 && s<=27.4+1e-6 && s>27.3,
           "repeated replanning reaches the same stop instead of deferring deceleration");
    FrenetState creep{};
    auto slow=GenerateSpatialStoppingCandidates(creep,2.1,1,1.5,Grid(20),Limits(),.5);
    Expect(!slow.empty(),"distance-admissible slow restart exists");
    if(!slow.empty()) {
        Expect(*std::max_element(slow[0].s_d.begin(),slow[0].s_d.end())<=1+1e-7,
               "restart respects the low cruise ceiling");
        Expect(*std::max_element(slow[0].s_dd.begin(),slow[0].s_dd.end())<=.5+1e-7,
               "restart acceleration is independent of service braking strength");
        Expect(Safe(slow[0],2.1)&&std::fabs(slow[0].s.back()-2.1)<1e-5,
               "slow restart retains the fixed zero-speed endpoint");
    }
    return failures?1:0;
}
