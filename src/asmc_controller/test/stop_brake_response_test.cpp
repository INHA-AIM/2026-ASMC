#include "Util/StopBrakeResponse.hpp"
#include "Util/StopResponseScope.hpp"
#include <cmath>
#include <iostream>
#include <limits>

#ifndef STOP_RESPONSE_STANDALONE
#include <gtest/gtest.h>
TEST(StopBrakeResponseTest, ResponseAndGuards) {
#else
int main() {
#endif
#ifdef STOP_RESPONSE_STANDALONE
    int failed=0;
#endif
    auto check=[&](bool ok,const char* name){
    #ifdef STOP_RESPONSE_STANDALONE
        std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';failed+=!ok;
#else
        EXPECT_TRUE(ok)<<name;
#endif
    };
    struct Command {
        enum { KEEP=0,FOLLOW=1,MERGE=2,LANE_CHANGE=3,AVOID=4,STOP=5,EMERGENCY_STOP=6,
               MERGE_NONE=0,MERGE_ROUNDABOUT=1,MERGE_HIGHWAY=2,PREPARE=0,EXECUTE=1,CANCEL=2 };
        int maneuver_type,merge_type,execution_request;
    };
    check(ResolveStopResponseScope(Command{Command::STOP,Command::MERGE_NONE,Command::EXECUTE})==
        StopResponseScope::ORDINARY_STOP,"ordinary STOP retains response correction");
    check(ResolveStopResponseScope(Command{Command::MERGE,Command::MERGE_ROUNDABOUT,Command::PREPARE})==
        StopResponseScope::ROUNDABOUT_WAITING,"roundabout waiting shares correction in its own reset scope");
    for(int request:{Command::EXECUTE,Command::CANCEL})
        check(ResolveStopResponseScope(Command{Command::MERGE,Command::MERGE_ROUNDABOUT,request})==
            StopResponseScope::NONE,"roundabout execution and cancellation remain outside STOP correction");
    for(int mode:{Command::KEEP,Command::FOLLOW,Command::AVOID,Command::LANE_CHANGE,Command::EMERGENCY_STOP})
        check(ResolveStopResponseScope(Command{mode,Command::MERGE_NONE,Command::PREPARE})==
            StopResponseScope::NONE,"other maneuvers and fixed safety braking retain their mapping");
    check(ResolveStopResponseScope(Command{Command::MERGE,Command::MERGE_HIGHWAY,Command::PREPARE})==
        StopResponseScope::NONE,"highway MERGE preparation remains unchanged");
    check(ResolveStopResponseScope(Command{Command::STOP,Command::MERGE_ROUNDABOUT,Command::PREPARE})==
        StopResponseScope::NONE,"legacy roundabout STOP is not mistaken for ordinary STOP");
    StopBrakeResponseConfig cfg;cfg.enabled=true;
    for(double plant_gain:{.75,1.,1.5,2.}) {
        StopBrakeResponse response;
        double measured=0,request=0;
        for(int i=0;i<100;++i) {
            const double t=1+i*.05;
            response.Observe(measured,5,t,cfg);
            request=response.Compensate(-2,-5);
            response.Issued(request,t,cfg);
            measured=plant_gain*request;
        }
        check(std::fabs(measured+2)<.04,"moving brake response tracks desired deceleration across plant gains");
        check(std::fabs(response.gain()-plant_gain)<.04,"estimated gain follows the observed brake response");
        check(request<=0&&request>=-5,"correction remains a bounded brake request");
        check(response.Compensate(1,-5)==1,"positive acceleration is not corrected by the brake model");
        response.Reset();
        check(response.gain()==1&&response.Compensate(-2,-5)==-2,"reset removes prior STOP calibration");
    }
    for(double plant_gain:{.75,1.,1.5,2.}) {
        StopAccelerationResponse throttle;
        double measured=0,request=0;
        for(int i=0;i<150;++i) {
            const double t=1+i*.05;
            throttle.Observe(measured,1,t,cfg);
            request=throttle.Compensate(.5,2);
            throttle.Issued(request,t,cfg);
            measured=plant_gain*request;
        }
        check(std::fabs(measured-.5)<.04,"throttle response tracks desired acceleration independently");
        check(std::fabs(throttle.gain()-plant_gain)<.08,"throttle gain is identified from positive responses");
        check(throttle.Compensate(-2,2)==-2,"throttle observer never changes a brake request");
        const double before=throttle.gain();
        throttle.Issued(-2,9,cfg);throttle.Observe(2,1,9.05,cfg);
        check(throttle.gain()==before,"residual acceleration during brake cannot recalibrate throttle");
        throttle.Reset();
        check(throttle.gain()==1,"new maneuver resets throttle calibration");
    }
    StopBrakeResponse response;
    response.Issued(-2,1,cfg);response.Observe(-4,5,1.05,cfg);
    const double gain=response.gain();
    response.Observe(-20,5,1.05,cfg);
    response.Observe(-20,5,1.04,cfg);
    check(response.gain()==gain,"duplicate and backward timestamps cannot relearn the same response");
    response.Observe(std::numeric_limits<double>::quiet_NaN(),5,1.1,cfg);
    response.Observe(-20,0,1.15,cfg);
    check(response.gain()==gain,"nonfinite acceleration and standstill cannot change brake calibration");
    response.Issued(1,1.15,cfg);response.Observe(-20,5,1.2,cfg);
    check(response.gain()==gain,"residual braking during throttle cannot be learned as pedal response");
    for(int i=0;i<50;++i){double t=2+i*.05;response.Issued(-1,t,cfg);response.Observe(-100,5,t+.025,cfg);}
    check(response.gain()==gain,"out-of-range acceleration spikes are not learned as calibration");
    check(response.gain()<=cfg.maximum_gain&&response.Compensate(-100,-5)==-5,
          "extreme response and requested braking remain bounded");
    response.Reset();response.Issued(-2,1,cfg);response.Observe(-4,5,.9,cfg);
    check(response.gain()==1,"observation older than the issued command cannot identify its response");
#ifdef STOP_RESPONSE_STANDALONE
    return failed?1:0;
#endif
}
