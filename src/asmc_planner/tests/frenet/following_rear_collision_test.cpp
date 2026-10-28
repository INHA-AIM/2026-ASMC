#include "Frenet/frenet_planner.hpp"
#include "Frenet/Pipeline/TrajectoryPipeline.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace {
int checks=0, failures=0;
void expect(bool ok, const char* text) {
    ++checks;
    if (!ok) {++failures;std::cerr<<"FAIL "<<text<<'\n';}
}
TrackedObject box(int id,double x,double y,double vx,double vy=0,double yaw=0,
                  double length=4,double width=1.8) {
    TrackedObject object;
    object.id=id;object.x=x;object.y=y;object.vx=vx;object.vy=vy;
    object.speed=std::hypot(vx,vy);object.heading=yaw;
    object.length=length;object.width=width;object.height=1;
    object.motion_state=ObjectMotionState::DYNAMIC;
    const double a[4][2]={{-length/2,-width/2},{length/2,-width/2},
                         {length/2,width/2},{-length/2,width/2}};
    for(int i=0;i<4;++i) {
        const double px=x+a[i][0]*std::cos(yaw)-a[i][1]*std::sin(yaw);
        const double py=y+a[i][0]*std::sin(yaw)+a[i][1]*std::cos(yaw);
        object.corners[i]={px,py,0};object.corners[i+4]={px,py,1};
    }
    object.geometry_valid=geometry::ValidateCorners8(object.corners).valid;
    return object;
}
RefLine reference(bool curved=false) {
    ReferenceConfig config{};
    config.closure_tolerance_m=.05;config.smoothing_window_m=5;
    config.resample_ds=.5;config.relax_max_iter=300;config.relax_step=.3;
    config.ego_projection_search_window=30;
    std::vector<double> x,y;
    for(int i=0;i<=80;++i) {
        const double s=i*5;
        x.push_back(curved?200*std::sin(s/200):s);
        y.push_back(curved?200*(1-std::cos(s/200)):0);
    }
    return BuildRefLine(x,y,config,.23);
}
FrenetPath path() {
    FrenetPath p;p.valid=true;
    for(int i=0;i<=15;++i) {
        p.t.push_back(i*.1);p.s.push_back(100+i*.5);
        p.d.push_back(0);p.s_d.push_back(5);
    }
    return p;
}
bool clear(const RefLine& ref,const TrackedObject& object,double half_width=1.75,
           FrenetPath p=path()) {
    std::vector<FrenetPath> paths{p};
    FilterByCollision(paths,ref,{object},VehicleShape{1.9,4.635,.79},
        CollisionCheckConfig{.45,.1,8},-1,-1,half_width);
    return paths.front().valid;
}
}
int main() {
    const auto ref=reference();
    const auto rear=box(1,80,0,10);
    expect(!clear(ref,rear,0),"strict default retains the extrapolated rear collision");
    expect(clear(ref,rear),"same-lane rear catch-up alone does not stop lane following");
    auto offset_rear=box(20,80,1.6,10);
    expect(!clear(ref,offset_rear),"unconfirmed boundary-straddling traffic keeps strict checks");
    offset_rear.prediction_lane=std::make_shared<const RefLine>(ref);
    offset_rear.prediction_lane_s=80;offset_rear.prediction_lane_d=1.6;
    expect(clear(ref,offset_rear),"confirmed same-lane rear OBB need not be wholly between approximate paint bounds");
    expect(!clear(ref,offset_rear,0),"lane-change/merge admission without following scope remains strict");
    auto adjacent=box(21,80,2.0,10);auto shifted=ref;for(auto& p:shifted.points)p.y+=3.5;
    adjacent.prediction_lane=std::make_shared<const RefLine>(shifted);adjacent.prediction_lane_s=80;
    expect(!clear(ref,adjacent),"a different mapped lane cannot claim rear-following responsibility");
    auto stopped=path();stopped.s_d.assign(stopped.s_d.size(),0);
    stopped.s.assign(stopped.s.size(),100);
    expect(clear(ref,rear,1.75,stopped),"a rear follower alone cannot prevent an ordinary stationary plan");
    expect(!clear(ref,box(2,110,0,0)),"front stationary obstacle still blocks");
    expect(!clear(ref,box(3,120,0,-5)),"oncoming traffic still blocks");
    expect(!clear(ref,box(4,110,4,2,-1)),"adjacent-lane cut-in ahead still blocks");
    expect(!clear(ref,box(5,80,4,10,-1)),"rear adjacent-lane cut-in is not a same-lane follower");
    expect(!clear(ref,box(14,80,-4,10,1)),"right-side rear cut-in also retains full checks");
    expect(!clear(ref,box(6,99,0,10)),"current physical rear overlap remains blocked");
    expect(!clear(ref,box(7,97,0,10)),"current rear safety-margin contact remains blocked");
    auto invalid=rear;invalid.geometry_valid=false;
    expect(!clear(ref,invalid),"invalid geometry is never exempted");
    auto bad_velocity=rear;bad_velocity.vx=std::numeric_limits<double>::quiet_NaN();
    expect(!clear(ref,bad_velocity),"invalid motion is never exempted");
    expect(clear(ref,box(8,80,0,10,.5)),"a rear follower moving out and away is not an incoming cut-in");
    auto reentering=box(13,80,0,10,2);
    reentering.turn_prediction_duration=2;reentering.continuation_vx=10;reentering.continuation_vy=-2;
    expect(!clear(ref,reentering),"a follower leaving then cutting back in retains full collision checks");
    auto leaving=path();
    for(int i=0;i<=15;++i) leaving.d[i]=i<=5?i*.4:(15-i)*.2;
    expect(!clear(ref,rear,1.75,leaving),"ego departure disables rear-following policy even after returning");
    auto returning=path();
    for(int i=0;i<=15;++i) returning.d[i]=-3.5*(15-i)/15;
    expect(!clear(ref,rear,1.75,returning),"KEEP cannot relax rear checks while reentering a lane");
    auto reversing=path();
    for(int i=0;i<=15;++i) {reversing.s[i]=100-i*.5;reversing.s_d[i]=-5;}
    auto fixed=box(9,80,0,10);fixed.motion_state=ObjectMotionState::LOW_THREAT;
    expect(!clear(ref,fixed,1.75,reversing),"LOW_THREAT remains a fixed physical obstacle");
    expect(!clear(ref,rear,std::numeric_limits<double>::quiet_NaN()),"invalid scope retains strict checks");
    const auto original=rear.corners;
    clear(ref,rear);
    expect(rear.vx==10 && rear.corners[0].x==original[0].x && rear.corners[7].y==original[7].y,
        "classification preserves source velocity and authoritative corners");
    const auto curve=reference(true);const auto rp=Interpolate(curve,80);
    auto turning=box(10,rp.x,rp.y,10*std::cos(rp.theta),10*std::sin(rp.theta),rp.theta);
    turning.yaw_rate=.05;
    expect(!clear(curve,turning,0),"curved-road strict baseline predicts rear catch-up");
    expect(clear(curve,turning),"rear role follows the reference corridor on curves");
    std::vector<FrenetPath> mixed{path()};
    FilterByCollision(mixed,ref,{rear,box(15,110,0,0)},VehicleShape{1.9,4.635,.79},
                      CollisionCheckConfig{.45,.1,8},-1,-1,1.75);
    expect(!mixed.front().valid && mixed.front().collision_object_id==15,
        "a rear follower never suppresses the simultaneous front obstacle");

    FrenetPlanner planner;
    expect(planner.Init(FRENET_TEST_CONFIG)&&planner.SetPreparedReference(ref),"prepare isolated core");
    const auto origin=Interpolate(ref,100);
    CartesianState ego{origin.x,origin.y,origin.theta,0,5,0};
    PlannerCommand command;command.has_target_speed=true;command.target_speed=58/3.6;
    CartesianPath output;
    planner.SetObjectObservationTime(100);
    expect(planner.Plan(ego,{rear},command,output),"production KEEP no longer fails on its rear follower");
    expect(!output.v.empty() && output.v.back()>0,"KEEP retains a forward trajectory");
    command.mode=FOLLOWING;command.target_object_id=11;
    const auto leader=box(11,140,0,5);
    planner.SetObjectObservationTime(100.1);
    expect(planner.Plan(ego,{rear,leader},command,output),"production FOLLOW checks its leader without rear starvation");
    command.mode=LANE_KEEPING;command.target_object_id=-1;
    planner.SetObjectObservationTime(100.2);
    const bool front_ok=planner.Plan(ego,{box(12,108,0,0)},command,output);
    expect(!front_ok || (!output.v.empty() && output.v.back()<ego.v),
        "a close front obstacle still requires rejection or a checked braking path");
    std::cout<<checks<<" checks, "<<failures<<" failures\n";
    return failures?1:0;
}
