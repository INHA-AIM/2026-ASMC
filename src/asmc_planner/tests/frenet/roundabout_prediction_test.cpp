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
    auto observe=[&](double stamp,double clamp,double speed_error=1.0) {
        const double angle=std::atan2(object.vy,object.vx),c=std::cos(angle),sn=std::sin(angle);
        for(int i=0;i<8;++i) {
            const double dx=(i%4==0||i%4==3)?-object.length/2:object.length/2;
            const double dy=i%4<2?-object.width/2:object.width/2;
            object.corners[i]={object.x+dx*c-dy*sn,object.y+dx*sn+dy*c,double(i/4)};
        }
        history.Observe({object},stamp,clamp,speed_error);
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
    expect(history.ExecutionOccupancies({matching},{},1).size()==1,
        "position-consistent real circular motion keeps its original execution model");
    expect(stable.size()==1&&std::fabs(stable[0].yaw_rate-.3)<1e-6&&
        stable[0].vx==object.vx&&stable[0].vy==object.vy,
        "consistent real circular motion retains its original long-horizon turn prediction");
    expect(history.UncertainNear(20,0)<0,
        "consistent full-body circular history still satisfies the original confidence gate");
    object.x+=10;object.length=.4;object.width=.2;observe(710.6,1);
    expect(history.Rate(21)==0&&history.ConservativeOccupancies({object}).empty()&&
        history.UncertainNear(object.x,object.y)==21,
        "discontinuous same ID cannot inherit a roundabout body or departure certificate");
    object.id=22;object.length=4.5;object.width=1.9;history.Reset();
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
    // LiDAR absolute speed error and two-observation spread are different.
    // Keep direction/body certificates and independent position disagreement.
    object.id=40;object.length=4.5;object.width=1.9;object.vy=0;
    history.Reset();
    for(int i=0;i<8;++i){object.x=6*i*.1;object.y=0;object.vx=i%2?8:4;observe(900+i*.1,1);}
    expect(history.UncertainNear(0,0)==40,
        "legacy two-metre-per-second spread policy rejects noisy but straight body observations");
    history.Reset();
    for(int i=0;i<8;++i){object.x=6*i*.1;object.vx=i%2?8:4;observe(900+i*.1,1,5);}
    expect(history.UncertainNear(0,0)<0,
        "configured speed error permits observed spread without bypassing motion direction or body gates");
    auto speed_bounds=history.WithObservedSpeedRanges({object});
    expect(speed_bounds.size()==1&&speed_bounds[0].prediction_min_speed==4&&
        speed_bounds[0].prediction_max_speed==8&&speed_bounds[0].vx==8,
        "departure interval retains the observed extrema and original measured velocity");
    object.x=object.y=0;object.yaw_rate=0;
    for(int i=0;i<8;++i)object.corners[i]={i%4==0||i%4==3?-2.25:2.25,
        i%4<2?-.95:.95,double(i/4)};
    auto interval=history.WithObservedSpeedRanges({object})[0];
    ego.x=12;ego.y=0;
    const auto middle_footprint=MakeEgoFootprint(ego,{.2,.2,-1},0);
    auto slow=object;slow.vx=4;
    expect(!CheckObstacleOverlap(middle_footprint,{},slow,2)&&
        !CheckObstacleOverlap(middle_footprint,{},object,2)&&
        CheckObstacleOverlap(middle_footprint,{},interval,2),
        "continuous speed sweep catches intermediate arrival missed by both endpoint speeds");
    auto unchanged=object;unchanged.prediction_min_speed=unchanged.prediction_max_speed=8;
    expect(PredictObstacleFootprint(unchanged,2).size()==PredictObstacleFootprint(object,2).size()&&
        CheckObstacleOverlap(middle_footprint,{},unchanged,2)==CheckObstacleOverlap(middle_footprint,{},object,2),
        "zero-width speed interval preserves nominal collision prediction");
    interval.motion_state=ObjectMotionState::LOW_THREAT;
    expect(PredictObstacleFootprint(interval,2).size()==PredictObstacleFootprint(object,0).size(),
        "stationary confirmed obstacles never acquire speed uncertainty motion");
    history.Reset();
    for(int i=0;i<8;++i){object.x=6*i*.1;object.vx=i%2?14:2;observe(901+i*.1,1,5);}
    expect(history.UncertainNear(0,0)==40,
        "spread beyond the configured error allowance still revokes departure confidence");
    history.Reset();object.id=50;object.length=4.5;object.width=1.9;
    object.motion_state=ObjectMotionState::DYNAMIC;object.vx=5;object.vy=0;
    object.velocity_unavailable=true;
    for(int i=0;i<6;++i){object.x=.5*i;object.y=0;observe(1000+i*.1,1);}
    expect(history.UncertainNear(0,0)==50&&history.Rate(50)==0&&
        history.ConservativeOccupancies({object}).empty(),
        "unavailable velocity cannot be accumulated as measured motion or authorize departure");
    object.velocity_unavailable=false;history.Reset();object.id=51;
    for(int i=0;i<8;++i){object.x=.5*i;object.y=0;object.vx=6.5;object.vy=0;observe(1100+i*.1,1);}
    auto supplied_lane=std::make_shared<RefLine>();supplied_lane->length=100;
    for(int i=0;i<=100;++i)supplied_lane->points.push_back({double(i),0,double(i),0,0,0});
    const auto executed=history.ExecutionOccupancies({object},{supplied_lane},2);
    expect(executed.size()==1&&bool(executed[0].prediction_lane)&&std::fabs(executed[0].vx-5)<1e-6&&
        executed[0].prediction_min_speed<=5&&executed[0].prediction_max_speed>=6.5&&
        executed[0].corners[0].x==object.corners[0].x,
        "mapped execution retains BOTH measured and position speed bounds with authoritative OBB");
    expect(!history.ExecutionOccupancies({object},{},2)[0].prediction_lane&&
        history.ExecutionOccupancies({object},{},2)[0].vx==object.vx&&
        !history.ExecutionOccupancies({object},{supplied_lane},30)[0].prediction_lane,
        "missing or partial map cannot authorize free-space positional extrapolation");
    object.type=2;
    expect(history.ExecutionOccupancies({object},{supplied_lane},2).size()==1,
        "vehicle continuation refinement does not force pedestrians into the circulating model");
    // Short missing observations are model queries, never new sensor samples.
    {
        RoundaboutPrediction retained;
        auto car=object;car.id=601;car.type=0;car.length=4.5;car.width=1.9;
        car.motion_state=ObjectMotionState::DYNAMIC;car.velocity_unavailable=false;
        car.vx=5;car.vy=0;car.y=0;car.yaw_rate=0;
        const auto stamp_car=[&](double x) {
            car.x=x;
            for(int k=0;k<8;++k)car.corners[k]={x+(k%4==0||k%4==3?-2.25:2.25),
                k%4<2?-.95:.95,double(k/4)};
        };
        for(int i=0;i<8;++i){stamp_car(.5*i);retained.Observe({car},1000+.1*i,1,5);}
        retained.Observe({},1000.8,1,5);
        auto missing=retained.MissingDepartureOccupancies(1000.85,[](double,double){return true;},6,3,1);
        expect(missing.size()==1&&!missing[0].models.empty()&&missing[0].observations==8,
            "one missing frame retains a previously verified whole-body vehicle");
        bool aligned=false;
        if(!missing.empty())for(const auto& model:missing[0].models)
            if(std::fabs(model.yaw_rate)<1e-9)aligned|=
                std::fabs(model.x-4.25)<1e-6&&std::fabs(model.prediction_origin_offset_s-.15)<1e-6;
        expect(aligned,"missing geometry aligns from real source time to ego time exactly once");
        retained.Observe({},1000.86,1,5);
        auto ahead=retained.MissingDepartureOccupancies(1000.85,[](double,double){return true;},6,3,1);
        expect(ahead.size()==1&&!ahead[0].models.empty()&&std::fabs(ahead[0].age-.15)<1e-6,
            "slightly future array time cannot hide a missing older vehicle");
        const auto repeated=retained.MissingDepartureOccupancies(1000.9,[](double,double){return true;},6,3,1);
        expect(repeated.size()==1&&repeated[0].observations==8&&
            std::fabs(repeated[0].source_stamp-1000.7)<1e-6,
            "repeated missing queries cannot renew the source stamp or fit observations");
        stamp_car(4.5);retained.Observe({car},1000.9,1,5);
        expect(retained.MissingDepartureOccupancies(1000.9,[](double,double){return true;},6,3,1).empty(),
            "reacquisition removes the missing copy rather than duplicating the vehicle");
        auto new_id=car;new_id.id=602;retained.Observe({new_id},1001,1,5);
        expect(retained.DepartureOccupancies({new_id},[](double,double){return true;},6,3).empty(),
            "a new ID cannot inherit the prior vehicle body or motion authority");
        retained.Observe({},1001.1,1,5);
        auto mixed=retained.MissingDepartureOccupancies(1001.1,[](double,double){return true;},6,3,1);
        bool unsupported=false;for(const auto& m:mixed)if(m.id==602)unsupported=m.models.empty();
        expect(unsupported,"unverified missing geometry is explicit and cannot certify a departure");
        retained.Observe({},1016,1,5);
        expect(retained.MissingDepartureOccupancies(1016,[](double,double){return true;},6,3,1).empty(),
            "one-second history is unchanged: no missing vehicle model survives fifteen seconds");
    }
    return failures?1:0;
}

