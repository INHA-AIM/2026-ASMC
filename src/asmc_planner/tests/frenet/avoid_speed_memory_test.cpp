#include "Frenet/Speed/AvoidSpeedMemory.hpp"
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
    // A measured braking response can remain stronger than the accepted
    // release ramp. Re-seeding from that measurement every 50 ms must not
    // postpone release forever after the bypass cap has been reached.
    current=-2.4;
    for(int i=0;i<10;++i){
        std::vector<double> profile{current};
        double a=current;
        for(int j=0;j<10;++j){a=std::min(3.0,a+.6);profile.push_back(a);}
        current=AvoidAccelerationSeed(-2.4,3.0,4.0,true,profile,.1,.05,2.4,3,true);
    }
    check(current>=-1e-8,"replanning completes accepted brake release below bypass cap");
    const std::vector<double> releasing{-2.4,-1.8,-1.2,-.6,0,.6};
    check(AvoidAccelerationSeed(-2.4,3,4,true,releasing,.1,.05,2.4,3,true)>-2.4,
          "below-cap braking advances the accepted release rather than rewinding");
    // full_05_healthy_input, 96.244670 s: these are the unchanged measured
    // state, previous accepted acceleration samples and inter-publication time.
    const double captured=AvoidAccelerationSeed(-2.6042563929835656,
        2.9306059205077553,4.0,true,
        {-2.072632557026426,-1.4726325570263685,-.8726325570263684},
        .1,.048316240310668945,2.4,3,true);
    check(captured>-1.9&&captured<-1.7,
          "recorded over-braking advances the already accepted release without an acceleration jump");
    check(AvoidAccelerationSeed(-2.4,3,4,false,releasing,.1,.05,2.4,3,true)==-2.4,
          "PREPARE cannot consume an old bypass brake release");
    check(AvoidAccelerationSeed(-2.4,3,4,true,releasing,.1,.6,2.4,3,true)==-2.4,
          "stale accepted release cannot change observed braking");
    check(AvoidAccelerationSeed(-2.4,5,2,true,releasing,.1,.05,2.4,3,true)==-2.4,
          "a newly lower cap preserves stronger measured braking");
    check(AvoidAccelerationSeed(-2.4,3,4,true,{-.6,-1.2,-1.8},.1,.05,2.4,3,true)==-2.4,
          "a profile still increasing braking cannot authorize brake release");
    check(AvoidAccelerationSeed(-2.4,1,2,true,releasing,.1,.05,2.4,3,false)==-2.4,
          "a release from a higher previous cap requires explicit rejection by the caller");
    current=3;
    for(int i=0;i<10;++i){
        std::vector<double> profile{current};
        double a=current;
        for(int j=0;j<10;++j){a=std::max(-2.4,a-.6);profile.push_back(a);}
        current=AvoidAccelerationSeed(3,2.8,3,true,profile,.1,.05,2.4,3,false,true);
    }
    check(current<=1e-8,"dynamic bypass advances its accepted acceleration reduction before overspeed");
    check(AvoidAccelerationSeed(3,2.8,3,true,{3,2.4,1.8},.1,.05,2.4,3,false,false)==3,
          "ordinary static bypass retains its existing acceleration-seed policy");
    check(AvoidAccelerationSeed(-2.4,2.8,3,true,{3,2.4,1.8},.1,.05,2.4,3,false,true)==-2.4,
          "continuing acceleration reduction cannot weaken stronger observed braking");
    check(AvoidAccelerationSeed(3,2.8,3,true,{3,2.4,1.8},.1,.6,2.4,3,false,true)==3,
          "a runtime pause cannot revive an old acceleration reduction");
    return failures?1:0;
}
