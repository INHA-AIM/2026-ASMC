#include "Frenet/Maneuver/Merge/ProgressStoppingCertificate.hpp"
#include "Frenet/Maneuver/Merge/HighwayVelocityRecovery.hpp"
#include "Frenet/Maneuver/Merge/RoundaboutPrediction.hpp"
#include <iostream>
#include "Frenet/Maneuver/ManeuverUtils.hpp"
int main(){
 int failed=0;auto check=[&](bool b,const char* name){std::cout<<(b?"[PASS] ":"[FAIL] ")<<name<<'\n';failed+=!b;};
 CartesianPath full;for(int i=0;i<=100;++i){full.x.push_back(.3*i);full.y.push_back(0);full.yaw.push_back(0);full.kappa.push_back(0);full.v.push_back(3);full.a.push_back(0);}
 const auto stop=MakeProgressStoppingCertificate(full,.2,.1,2,4,10);
 check(stop.x.size()>=101&&stop.v.back()==0&&stop.a.back()==0,"progress includes complete stop and full prediction tail");
 check(stop.x[0]==full.x[0]&&stop.v[1]==full.v[1]&&stop.x.back()<10,"progress preserves execution prefix and stops before distant blockage");
 bool bounded=true;for(size_t i=1;i<stop.a.size();++i)bounded&=std::fabs(stop.a[i])<=2.00001&&std::fabs(stop.a[i]-stop.a[i-1])/.1<=4.00001;
 check(bounded,"prefix/stop seam respects acceleration and jerk bounds");
 auto tiny=full;for(auto* v:{&tiny.x,&tiny.y,&tiny.yaw,&tiny.kappa,&tiny.v,&tiny.a})v->resize(3);
 check(MakeProgressStoppingCertificate(tiny,.1,.1,2,4,10).x.empty(),"no certificate when supplied geometry cannot contain complete stop");
 const double latest=ProgressPrefixBeforeCollision(full,70,.2,.1,2,4);
 const auto advanced=MakeProgressStoppingCertificate(full,latest,.1,2,4,.2,.25);
 check(latest>.2&&latest<7&&!advanced.x.empty()&&advanced.v.back()==0,
     "future collision opens a physically stop-capable forward prefix instead of immediate braking only");
 check((advanced.x.size()-1)*.1<=7.3+1e-6,
     "progress prefix is derived from stopping dynamics before the nominal conflict");
 TrackedObject actor;actor.id=71;actor.geometry_valid=true;actor.length=4;actor.width=2;actor.height=1;actor.x=20;actor.motion_state=ObjectMotionState::LOW_THREAT;
 auto corners=[&](){for(int i=0;i<8;++i)actor.corners[i]={actor.x+(i%4<2?-1.:1.),(i%2?-1.:1.),double(i/4)};};corners();
 CollisionCheckConfig collision{.45,.1,8};VehicleShape shape{1.9,4.635,.79};
 check(!CartesianTrajectoryClear(full,{actor},shape,collision,.1)&&CartesianTrajectoryClear(stop,{actor},shape,collision,.1),"stopping alternative preserves collision check instead of approving blocked traversal");
 actor.motion_state=ObjectMotionState::DYNAMIC;actor.vx=-2;
 check(!CartesianTrajectoryClear(stop,{actor},shape,collision,.1),"late collision with held endpoint is still rejected");
 ObservedObjectMotion history;TrackedObject input;input.id=9;input.length=4;input.width=2;input.motion_state=ObjectMotionState::DYNAMIC;input.vx=5;
 for(int i=0;i<5;++i){input.x=.5*i;history.Observe(input,100+.1*i,1,.05,12,.8,true);}
 input.x=2.5;input.velocity_unavailable=true;input.vx=0;
 history.Observe(input,100.5,1,.05,12,.8,true);
 check(history.samples.size()==5&&history.samples.back().vx==5,"missing highway velocity does not append a fictitious measured zero");
 HighwayMergeConfig cfg;cfg.source_max_age_s=.3;cfg.motion_fit_min_span_s=.2;
 const auto recovered=RecoverHighwayVelocity(input,&history,cfg);
 check(input.velocity_unavailable&&!recovered.velocity_unavailable&&recovered.vx==5,"only derived prediction recovers fresh motion; source flag is retained");
 check(recovered.prediction_min_speed==0&&PrepareHighwayObjectPrediction(recovered,&history,cfg).prediction_min_speed==0,"recovered prediction still includes stationary occupancy after preparation");
 ObservedObjectMotion newborn;newborn.Observe(input,100.5,1,.05,12,.8,true);
 check(RecoverHighwayVelocity(input,&newborn,cfg).velocity_unavailable,"newborn occupancy cannot borrow guessed velocity");
 history.Observe(input,101,1,.05,12,.8,true);
 check(RecoverHighwayVelocity(input,&history,cfg).velocity_unavailable,"stale history cannot recover velocity");
 input.velocity_unavailable=false;input.motion_state=ObjectMotionState::LOW_THREAT;history.Observe(input,101.1,1,.05,12,.8,true);
 check(history.samples.empty(),"confirmed static occupancy still clears motion");
 RoundaboutPrediction round;auto line=std::make_shared<RefLine>();
 for(int i=0;i<=100;++i){double a=.01*i;line->points.push_back({20*std::sin(a),20*(1-std::cos(a)),20*a,a,.05,0});}line->length=20;
 TrackedObject car;car.id=16;car.type=1;car.motion_state=ObjectMotionState::DYNAMIC;car.length=4;car.width=2;car.geometry_valid=true;
 for(int i=0;i<5;++i){double t=.1*i;car.x=20*std::sin(.15*t);car.y=20*(1-std::cos(.15*t));car.vx=3*std::cos(.15*t);car.vy=3*std::sin(.15*t);for(int k=0;k<8;++k)car.corners[k]={car.x+(k%4<2?-2.:2.),car.y+(k%2?-1.:1.),double(k/4)};round.Observe({car},200+t,1);}
 auto mapped=round.MappedMotion(car,{line});
 check(bool(mapped.prediction_lane)&&mapped.x==car.x&&mapped.y==car.y,"mapped actor prediction is anchored to current observation");
 const auto p=PredictTrackedObjectPose(mapped,1);const double angle=.15*1.4;
 check(std::hypot(p.x-20*std::sin(angle),p.y-20*(1-std::cos(angle)))<.02,"future position follows mapped curve rather than noisy instantaneous turn rate");
 check(!round.MappedMotion(car,{line},10).prediction_lane,"partial map cannot certify an unprovided circulating continuation");
 PredictionInputStatus status;
 auto upper_bound=car;upper_bound.prediction_max_speed=8;
 check(!round.MappedMotion(upper_bound,{line},4,&status).prediction_lane&&
     status==PredictionInputStatus::MAP_UNCOVERED,
     "map authority must cover fastest bounded speed, not only nominal velocity");
 auto missing=car;missing.velocity_unavailable=true;
 check(!round.MappedMotion(missing,{line},1,&status).prediction_lane&&
     status==PredictionInputStatus::VELOCITY_UNAVAILABLE,
     "missing velocity is distinct from zero and cannot acquire mapped authority");
 check(InspectPredictionVelocity(missing)==PredictionInputStatus::VELOCITY_UNAVAILABLE,
     "shared prediction input contract preserves unavailable velocity");
 missing.velocity_unavailable=false;missing.vx=NAN;
 check(InspectPredictionVelocity(missing)==PredictionInputStatus::INVALID_VELOCITY,
     "nonfinite motion is an invalid model, not a stationary observation");
 auto inconsistent=car;inconsistent.vx*=3;inconsistent.vy*=3;
 check(!round.MappedMotion(inconsistent,{line},1,&status).prediction_lane&&status==PredictionInputStatus::VELOCITY_DISAGREEMENT,"map match cannot mask velocity inconsistent with position history");
 car.type=2;check(!round.MappedMotion(car,{line}).prediction_lane,"pedestrian is not forced to circulate");
 car.type=1;car.vx=-car.vx;car.vy=-car.vy;
 check(!round.MappedMotion(car,{line}).prediction_lane,"opposite motion cannot acquire circulating lane");
 return failed?1:0;
}
