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
    auto observe=[&](double stamp,double clamp) {
        const double angle=std::atan2(object.vy,object.vx),c=std::cos(angle),sn=std::sin(angle);
        for(int i=0;i<8;++i) {
            const double dx=(i%4==0||i%4==3)?-object.length/2:object.length/2;
            const double dy=i%4<2?-object.width/2:object.width/2;
            object.corners[i]={object.x+dx*c-dy*sn,object.y+dx*sn+dy*c,double(i/4)};
        }
        history.Observe({object},stamp,clamp);
    };
    const double times[]{0,.07,.19,.30,.42};
    for(double t:times){
        object.vx=5*std::cos(3.08+.3*t);object.vy=5*std::sin(3.08+.3*t);
        object.heading=t>0?-2:2; // Visible OBB orientation deliberately jumps.
        observe(100+t,.6);
    }
    expect(std::fabs(history.Rate(1)-.3)<1e-6,"irregular sensor intervals and angle wrap recover physical turn rate");
    expect(history.UncertainNear(0,0)<0,"stable full vehicle history passes confidence gate");
    const double rate=history.Rate(1);
    object.vx=-5;object.vy=0;object.length=.8;
    observe(100.42,.6);
    expect(history.Rate(1)==rate&&history.UncertainNear(0,0)<0,"reusing the same observation does not alter motion or confidence");
    observe(100.5,.6);
    expect(history.UncertainNear(0,0)<0,"brief partial visibility retains reliable motion with a fresh full-body bound");
    auto held=history.ConservativeOccupancies({object});
    expect(held.size()==1&&held[0].length>=4.5&&held[0].width>=1.9,
        "partial visibility does not shrink the additional planning occupancy");
    object.motion_state=ObjectMotionState::LOW_THREAT;object.vx=object.vy=0;
    observe(100.55,.6);
    expect(history.UncertainNear(0,0)==1,"one zero-velocity classification cannot erase a recently moving fragment");
    history.Observe({},100.6,.6);
    expect(history.UncertainNear(0,0)==1,"one missing observation cannot immediately release a recently seen vehicle");
    history.Observe({},101.6,.6);
    expect(history.UncertainNear(0,0)<0,"missing history is bounded and expires without an unbounded container");
    object.motion_state=ObjectMotionState::DYNAMIC;object.vx=-5;
    observe(102,.6);
    expect(history.Rate(1)==0&&history.UncertainNear(0,0)==1,"a newly reacquired ID needs fresh motion history");
    observe(101,.6);
    expect(history.Rate(1)==0&&history.UncertainNear(0,0)==1,"backward sensor time resets rather than deriving a spurious turn");
    observe(103,.6);
    expect(history.Rate(1)==0&&history.UncertainNear(0,0)==1,"a long sensor gap requires a new bounded observation history");
    object.id=2;object.motion_state=ObjectMotionState::LOW_THREAT;object.vx=object.vy=0;
    history.Reset();observe(200,.6);
    expect(history.UncertainNear(0,0)<0&&history.Rate(2)==0,"stationary small fixtures do not become uncertain rotating vehicles");
    object.id=3;object.vx=5;object.x=100;
    observe(200.1,.6);
    expect(history.UncertainNear(0,0)<0,"remote partial objects do not block the local conflict area");
    object.x=2;observe(200.2,.6);
    expect(history.UncertainNear(0,0)==3,"a nearby new moving fragment cannot authorize entry");
    expect(history.UncertainNear(0,0,[](int id){return id==3;})<0,
        "explicit passed-vehicle certificate excludes only its own uncertainty gate");
    expect(!history.MovingAwayReliable(3,0,0),"new incomplete motion cannot be called reliably departing");
    history.Reset();
    expect(history.UncertainNear(0,0)<0&&history.Rate(3)==0,"mission reset clears isolated roundabout history");
    object.id=4;object.x=20;object.y=0;object.vx=5;object.vy=0;
    object.length=4.5;object.width=1.9;object.motion_state=ObjectMotionState::DYNAMIC;
    for(int i=0;i<5;++i)observe(300+i*.1,.6);
    expect(history.MovingAwayReliable(4,0,0),"stable receding velocity is distinguished from fragment orientation");
    expect(!history.MovingAwayReliable(4,40,0),"the same approaching vehicle cannot receive a passed certificate");
    history.Reset();object.id=5;object.x=5;object.y=0;object.vx=5;object.vy=0;
    object.length=4.8;object.width=2.1;object.geometry_valid=true;
    for(int i=0;i<8;++i) {object.x=5+.5*i;object.width=i%2?1.3:2.1;observe(400+i*.1,.6);}
    expect(history.UncertainNear(0,0)<0,"normal full-body width variation cannot erase reliable motion");
    auto full=history.ConservativeOccupancies({object});
    expect(full.size()==1&&full[0].width>=2.6,"maximum recent width enlarges occupancy instead of causing permanent waiting");
    object.length=.8;object.width=.5;object.x=10.3;observe(400.8,.6);
    auto fragment=history.ConservativeOccupancies({object});
    expect(fragment.size()==1&&std::fabs(fragment[0].x-9)<1e-8,
        "retained body centre follows velocity rather than jumping to visible fragment centre");
    for(int i=9;i<=18;++i){object.x=5+.5*i+1.3;observe(400+i*.1,.6);}
    expect(history.UncertainNear(0,0)==5&&history.ConservativeOccupancies({object}).empty(),
        "partial geometry cannot renew a full-body memory beyond one second");
    observe(402.5,.6);
    expect(history.UncertainNear(0,0)==5,"a sensor gap clears geometry and motion certificates");
    history.Reset();object.id=6;object.x=2;object.length=.8;object.width=.5;
    for(int i=0;i<8;++i)observe(500+i*.1,.6);
    expect(history.UncertainNear(0,0)==6&&history.ConservativeOccupancies({object}).empty(),
        "a new partial-only vehicle cannot inherit a fabricated full-body certificate");
    object.id=7;object.length=4.5;object.width=1.9;object.x=0;
    for(int i=0;i<5;++i){object.x=.5*i;observe(600+i*.1,.6);}
    object.length=.8;object.width=.5;object.y=10;observe(600.5,.6);
    expect(history.ConservativeOccupancies({object}).empty(),"inconsistent fragment location invalidates retained body use");
    object.y=0;object.length=4.5;object.width=1.9;object.geometry_valid=false;observe(600.6,.6);
    expect(history.ConservativeOccupancies({object}).empty(),"invalid current geometry cannot authorize a remembered body");
    return failures?1:0;
}
