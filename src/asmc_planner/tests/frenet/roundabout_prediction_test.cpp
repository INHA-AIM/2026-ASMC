#include "Frenet/Maneuver/Merge/RoundaboutPrediction.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
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
    // A smooth velocity-heading fit can still be wrong: position observations
    // keep moving faster while reported speed decays and invents a tight turn.
    history.Reset();object.id=20;object.length=4.5;object.width=1.9;
    object.geometry_valid=true;object.motion_state=ObjectMotionState::DYNAMIC;
    for(int i=0;i<6;++i){double q=i*.1;object.x=20+6*q;object.y=0;
        object.vx=2*std::cos(-.9*q);object.vy=2*std::sin(-.9*q);observe(700+q,1);}
    const auto disputed=history.ApproachOccupancies({object});
    expect(disputed.size()==2&&std::fabs(disputed[0].vx-6)<1e-8&&
        std::fabs(disputed[0].vy)<1e-8&&disputed[0].yaw_rate==0,
        "position-supported continued motion replaces a disputed velocity decay for approach checks");
    expect(disputed.size()==2&&disputed[1].vx==object.vx&&disputed[1].vy==object.vy&&
        std::fabs(disputed[1].yaw_rate+.9)<1e-6&&disputed[1].turn_prediction_duration<=.5,
        "the original velocity and provisional turn remain a bounded additional safety hypothesis");
    expect(history.UncertainNear(20,0)==20&&!history.MovingAwayReliable(20,0,0),
        "a disputed velocity fit cannot authorize departure or a passed-vehicle certificate");
    expect(disputed.size()==2&&disputed[0].corners[0].x==object.corners[0].x&&
        disputed[1].corners[0].y==object.corners[0].y,
        "motion refinement preserves authoritative perception corners in both hypotheses");
    observe(700.5,1);
    expect(history.ApproachOccupancies({object}).size()==2,
        "reusing an observation cannot wash away disputed motion");
    object.motion_state=ObjectMotionState::LOW_THREAT;object.vx=object.vy=0;observe(700.6,1);
    expect(history.ApproachOccupancies({object}).size()==1&&
        history.ApproachOccupancies({object})[0].yaw_rate==0,
        "a stationary classification clears the positional velocity refinement");
    history.Reset();object.id=21;object.motion_state=ObjectMotionState::DYNAMIC;
    for(int i=0;i<6;++i){double q=i*.1;object.x=20+6*std::sin(.3*q)/.3;
        object.y=6*(1-std::cos(.3*q))/.3;
        object.vx=6*std::cos(.3*q);object.vy=6*std::sin(.3*q);observe(710+q,1);}
    auto matching=object;matching.yaw_rate=history.Rate(21);
    const auto stable=history.ApproachOccupancies({matching});
    expect(stable.size()==1&&std::fabs(stable[0].yaw_rate-.3)<1e-6&&
        stable[0].vx==object.vx&&stable[0].vy==object.vy,
        "consistent real circular motion retains its original long-horizon turn prediction");
    expect(history.UncertainNear(20,0)<0,
        "consistent full-body circular history still satisfies the original confidence gate");
    object.id=22;history.Reset();
    for(int i=0;i<6;++i){object.x=20+6*i*.1+(i%2?2:-2);object.y=0;
        object.vx=2;object.vy=0;observe(720+i*.1,1);}
    auto fragment_input=object;fragment_input.yaw_rate=history.Rate(22);
    expect(history.ApproachOccupancies({fragment_input}).size()==1,
        "large fragment-centre jumps cannot fabricate a positional velocity correction");
    // A long horizontal strip from a partly visible vehicle was previously
    // mistaken for a whole body despite only 16 cm of vertical evidence.
    history.Reset();object.id=30;object.length=3.47;object.width=1.34;
    object.height=1.5;object.x=2;object.y=2;object.vx=5;object.vy=0;
    for(int j=0;j<5;++j) {
        object.x=2+.5*j;
        for(int i=0;i<8;++i) {
            object.corners[i]={object.x+(i%4==0||i%4==3?-1.735:1.735),
                object.y+(i%4<2?-.67:.67),.16*(i/4)};
        }
        history.Observe({object},800+j*.1,1);
    }
    expect(history.UncertainNear(0,0)==30&&history.ConservativeOccupancies({object}).empty(),
        "a low measured strip cannot establish a whole-body certificate even with nominal height metadata");
    const auto low_models=RoundaboutPrediction::ExpandTurnModels({object},1);
    expect(!low_models.empty()&&low_models.front().id==30&&
        low_models.front().corners[4].z==object.corners[4].z,
        "uncertified low fragments remain authoritative collision observations");
    for(int j=5;j<10;++j){object.x=2+.5*j;observe(800+j*.1,1);}
    expect(history.UncertainNear(0,0)<0&&!history.ConservativeOccupancies({object}).empty(),
        "fresh real vertical body evidence restores confidence without requiring a new ID");
    const auto rates=RoundaboutPrediction::TurnModelRates(.7,1);
    const auto negative=RoundaboutPrediction::TurnModelRates(-.7,1);
    bool interior=false,mirror=rates.size()==negative.size(),bounded=rates.size()<=13;
    for(double endpoint:{0.0,.55,.7,.85})
        bounded&=std::any_of(rates.begin(),rates.end(),[&](double value){return std::fabs(value-endpoint)<1e-9;});
    for(size_t i=0;i<rates.size();++i){
        interior|=rates[i]>.15&&rates[i]<.4;
        bounded&=rates[i]>=-1&&rates[i]<=1;
        if(i)bounded&=rates[i]-rates[i-1]<=.15+1e-9;
        if(mirror)mirror&=std::fabs(rates[i]+negative[negative.size()-1-i])<1e-9;
    }
    expect(interior&&mirror&&bounded,
        "bounded turn coverage includes moderate rotation between straight and the fitted turn in either direction");
    auto second=object;second.id=31;second.yaw_rate=-.7;object.yaw_rate=.7;
    const auto independent=RoundaboutPrediction::ExpandTurnModels({object,second},1);
    expect(independent.size()==rates.size()+negative.size(),
        "multiple vehicles add model checks linearly rather than multiplying motion combinations");
    object.vx=object.vy=0;object.yaw_rate=.7;
    expect(RoundaboutPrediction::ExpandTurnModels({object},1).size()==1,
        "stationary observations do not acquire invented motion models");
    expect(RoundaboutPrediction::TurnModelRates(NAN,1)==std::vector<double>{0},
        "nonfinite turn estimates cannot create invalid model rates");
    // Independent geometric counterexample: a moderate turn reaches the ego
    // in three seconds, while straight and fitted +/-0.15 endpoints miss it.
    object.x=object.y=0;object.vx=5;object.vy=0;object.yaw_rate=.7;
    object.motion_state=ObjectMotionState::DYNAMIC;object.geometry_valid=true;
    for(int i=0;i<8;++i)object.corners[i]={i%4==0||i%4==3?-1.735:1.735,
        i%4<2?-.67:.67,double(i/4)};
    auto moderate=object;moderate.yaw_rate=.25;
    const auto arrival=PredictTrackedObjectPose(moderate,3);
    CartesianState ego{};ego.x=arrival.x;ego.y=arrival.y;
    const auto footprint=MakeEgoFootprint(ego,{1.9,4.635,-1},0);
    bool endpoints_clear=true,intermediate_hit=false;
    for(double rate:{0.0,.55,.85}){
        auto predicted=object;predicted.yaw_rate=rate;
        endpoints_clear&=!CheckObstacleOverlap(footprint,{},predicted,3);
    }
    for(const auto& predicted:RoundaboutPrediction::ExpandTurnModels({object},1))
        intermediate_hit|=CheckObstacleOverlap(footprint,{},predicted,3);
    expect(endpoints_clear&&intermediate_hit,
        "intermediate turn models catch a physical collision missed by the old three endpoints");
    return failures?1:0;
}

