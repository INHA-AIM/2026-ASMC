#include "frenet_planner/Speed/AvoidSpeedMemory.hpp"
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    int failures=0;
    auto check=[&](bool ok,const char* name){
        std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';
        if(!ok)++failures;
    };
    const std::vector<double> braking{3,2.4,1.8,1.2,.6,0,-.6,-1.2,-1.8,-2.4};
    auto seed=[&](double observed,double v,bool active,double elapsed){
        return AvoidAccelerationSeed(observed,v,3,active,braking,.1,elapsed,2.4,3);
    };
    check(std::fabs(seed(3,4.8,true,.05)-2.7)<1e-9,
          "advance accepted jerk ramp by planning period");
    check(std::fabs(seed(3,4.8,true,.5))<1e-9,
          "measured acceleration cannot restart a completed braking ramp");
    check(seed(-2,4.8,true,.05)==-2,
          "preserve stronger measured braking");
    check(seed(2,2,true,.05)==2,
          "allow acceleration below bypass speed cap");
    check(seed(3,4.8,false,.05)==3,
          "approach and reset contexts do not consume bypass history");
    check(seed(3,4.8,true,.6)==3,
          "runtime pause expires speed history");
    check(seed(3,4.8,true,0)==3,
          "invalid elapsed time does not consume history");
    check(AvoidAccelerationSeed(3,4.8,3,true,{},.1,.05,2.4,3)==3,
          "missing accepted profile falls back to observed acceleration");
    check(AvoidAccelerationSeed(3,4.8,3,true,{NAN,NAN},.1,.05,2.4,3)==3,
          "nonfinite history is ignored");
    // Repeated 20 Hz replanning with persistently positive measured acceleration
    // must finish the ramp instead of repeating its positive initial portion.
    double current=3;
    for(int i=0;i<10;++i){
        std::vector<double> profile{current};
        double a=current;
        for(int j=0;j<10;++j){a=std::max(-2.4,a-.6);profile.push_back(a);}
        current=AvoidAccelerationSeed(3,4.8,3,true,profile,.1,.05,2.4,3);
    }
    check(std::fabs(current)<1e-9,"replanning completes positive-to-zero ramp");
    return failures?1:0;
}
