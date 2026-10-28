#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

namespace {
unsigned checks=0,failures=0;
void expect(bool value,const char* name){++checks;if(!value){++failures;std::cerr<<"FAIL "<<name<<'\n';}}
RefLine line(bool curve) {
    RefLine r;
    for(int i=0;i<=1600;++i){double s=i*.5;
        r.points.push_back({curve?200*std::sin(s/200):s,
            curve?200*(1-std::cos(s/200)):0,s,curve?s/200:0,curve?.005:0,0});}
    r.length=r.points.back().s;return r;
}
TrackedObject box(int id,double x,double y,double vx,double vy,double yaw) {
    TrackedObject o;o.id=id;o.x=x;o.y=y;o.vx=vx;o.vy=vy;o.heading=yaw;
    o.length=4;o.width=1.8;o.height=1;o.speed=std::hypot(vx,vy);
    o.motion_state=ObjectMotionState::DYNAMIC;
    const double xy[][2]={{-2,-.9},{2,-.9},{2,.9},{-2,.9}};
    for(int i=0;i<4;++i){double xx=x+xy[i][0]*std::cos(yaw)-xy[i][1]*std::sin(yaw);
        double yy=y+xy[i][0]*std::sin(yaw)+xy[i][1]*std::cos(yaw);
        o.corners[i]={xx,yy,0};o.corners[i+4]={xx,yy,1};}
    o.geometry_valid=true;return o;
}
FrenetPath path(double speed,double lateral,double horizon,double dt) {
    FrenetPath p;p.valid=true;
    for(int i=0;i<=int(std::round(horizon/dt));++i){double t=i*dt;
        p.t.push_back(t);p.s.push_back(100+speed*t);p.s_d.push_back(speed);
        p.s_dd.push_back(0);p.s_ddd.push_back(0);p.d.push_back(lateral);
        p.d_d.push_back(0);p.d_dd.push_back(0);}
    return p;
}
void compare(const RefLine& ref,const std::vector<TrackedObject>& objects,
    const std::vector<FrenetPath>& input,double half,int recovery,bool braking) {
    auto direct=input,reused=input;
    const StoppingContinuationPolicy stopping{3,3,.23,2.6,true};
    const VehicleShape body{1.9,4.635,.79};const CollisionCheckConfig cfg{.45,.1,5};
    FilterByCollision(direct,ref,objects,body,cfg,-1,recovery,half,braking?&stopping:nullptr,false);
    FilterByCollision(reused,ref,objects,body,cfg,-1,recovery,half,braking?&stopping:nullptr,true);
    expect(direct.size()==reused.size(),"candidate count unchanged");
    for(std::size_t i=0;i<direct.size();++i){const auto& a=direct[i];const auto& b=reused[i];
        expect(a.valid==b.valid&&a.rejection_reason==b.rejection_reason&&
            a.collision_object_id==b.collision_object_id&&a.collision_sample_index==b.collision_sample_index&&
            a.margin_recovery_used==b.margin_recovery_used,
            "cached and original full collision decisions agree");
        expect(a.s==b.s&&a.d==b.d&&a.s_d==b.s_d,"reuse never changes trajectory samples");}
}
}
int main(){
    std::mt19937_64 random(92571);std::uniform_real_distribution<double> unit(0,1);
    for(int scene=0;scene<100;++scene){auto ref=std::make_shared<RefLine>(line(scene%2));
        std::vector<TrackedObject> objects;
        for(int j=0;j<8;++j){double station=65+90*unit(random),lateral=10*unit(random)-5;
            const auto p=Interpolate(*ref,station);double speed=22*unit(random);
            auto o=box(j%4,p.x-lateral*std::sin(p.theta),p.y+lateral*std::cos(p.theta),
                speed*std::cos(p.theta),speed*std::sin(p.theta),p.theta);
            if(j%3==0)o.motion_state=ObjectMotionState::LOW_THREAT;
            if(j%3==1){o.prediction_lane=ref;o.prediction_lane_s=station;o.prediction_lane_d=lateral;
                o.prediction_min_speed=std::max(0.,speed-.75);o.prediction_max_speed=speed+.75;}
            if(j%3==2){o.yaw_rate=.07;o.turn_prediction_duration=.3;
                o.continuation_vx=o.vx;o.continuation_vy=o.vy;}
            objects.push_back(o);}
        if(scene%11==0)objects.front().geometry_valid=false;
        std::vector<FrenetPath> candidates;
        for(double horizon:{1.5,3.,4.5})for(double d:{-.5,0.,.5})for(double v:{4.,10.,18.})
            candidates.push_back(path(v,d,horizon,.1));
        compare(*ref,objects,candidates,scene%2?1.75:0,scene%5==0?0:-1,scene%3==0);
        // New contents and reused IDs in another invocation cannot inherit a
        // prediction from the previous observation or object-vector position.
        std::reverse(objects.begin(),objects.end());
        for(auto& o:objects){o.vx=-o.vx;o.vy=-o.vy;}
        compare(*ref,objects,candidates,1.75,-1,false);
    }
    const auto ref=line(false);std::vector<TrackedObject> many;
    for(int j=0;j<100;++j)many.push_back(box(7,10000+j*10,500,3,0,0));
    compare(ref,many,{path(10,0,2,.01),path(12,0,2,.01)},0,-1,false);
    auto overlap=box(1,100,0,0,0,0);
    std::vector<FrenetPath> blocked{path(10,0,3,.1)};
    FilterByCollision(blocked,ref,{overlap},VehicleShape{1.9,4.635,.79},
        CollisionCheckConfig{.45,.1,5},-1,-1,0,nullptr,true);
    expect(!blocked.front().valid,"current physical overlap remains blocked");
    std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
