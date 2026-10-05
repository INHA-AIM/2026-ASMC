#include "frenet_planner/Maneuver/Merge/RoundaboutPrediction.hpp"
#include <cmath>
#include <iostream>
int main(){
    int failures=0;
    auto expect=[&](bool value,const char* label){std::cout<<(value?"[PASS] ":"[FAIL] ")<<label<<'\n';failures+=!value;};
    RoundaboutPrediction history;
    TrackedObject object{};object.id=1;object.x=2;object.y=2;
    object.length=4.5;object.width=1.9;object.geometry_valid=true;
    object.motion_state=ObjectMotionState::DYNAMIC;
    const double times[]{0,.07,.19,.30,.42};
    for(double t:times){
        object.vx=5*std::cos(3.08+.3*t);object.vy=5*std::sin(3.08+.3*t);
        object.heading=t>0?-2:2; // Visible OBB orientation deliberately jumps.
        history.Observe({object},100+t,.6);
    }
    expect(std::fabs(history.Rate(1)-.3)<1e-6,"irregular sensor intervals and angle wrap recover physical turn rate");
    expect(history.UncertainNear(0,0)<0,"stable full vehicle history passes confidence gate");
    const double rate=history.Rate(1);
    object.vx=-5;object.vy=0;object.length=.8;
    history.Observe({object},100.42,.6);
    expect(history.Rate(1)==rate&&history.UncertainNear(0,0)<0,"reusing the same observation does not alter motion or confidence");
    history.Observe({object},100.5,.6);
    expect(history.UncertainNear(0,0)==1,"new partial vehicle geometry revokes departure confidence");
    object.motion_state=ObjectMotionState::LOW_THREAT;object.vx=object.vy=0;
    history.Observe({object},100.55,.6);
    expect(history.UncertainNear(0,0)==1,"one zero-velocity classification cannot erase a recently moving fragment");
    history.Observe({},100.6,.6);
    expect(history.UncertainNear(0,0)==1,"one missing observation cannot immediately release a recently seen vehicle");
    history.Observe({},101.6,.6);
    expect(history.UncertainNear(0,0)<0,"missing history is bounded and expires without an unbounded container");
    object.motion_state=ObjectMotionState::DYNAMIC;object.vx=-5;
    history.Observe({object},102,.6);
    expect(history.Rate(1)==0&&history.UncertainNear(0,0)==1,"a newly reacquired ID needs fresh motion history");
    history.Observe({object},101,.6);
    expect(history.Rate(1)==0&&history.UncertainNear(0,0)==1,"backward sensor time resets rather than deriving a spurious turn");
    history.Observe({object},103,.6);
    expect(history.Rate(1)==0&&history.UncertainNear(0,0)==1,"a long sensor gap requires a new bounded observation history");
    object.id=2;object.motion_state=ObjectMotionState::LOW_THREAT;object.vx=object.vy=0;
    history.Reset();history.Observe({object},200,.6);
    expect(history.UncertainNear(0,0)<0&&history.Rate(2)==0,"stationary small fixtures do not become uncertain rotating vehicles");
    object.id=3;object.vx=5;object.x=100;
    history.Observe({object},200.1,.6);
    expect(history.UncertainNear(0,0)<0,"remote partial objects do not block the local conflict area");
    object.x=2;history.Observe({object},200.2,.6);
    expect(history.UncertainNear(0,0)==3,"a nearby new moving fragment cannot authorize entry");
    history.Reset();
    expect(history.UncertainNear(0,0)<0&&history.Rate(3)==0,"mission reset clears isolated roundabout history");
    return failures?1:0;
}
