#include "local/journal.hpp"
#include "fixtures/local2f_stopped_barrier.hpp"
#include "local/motion.hpp"
#include "local/obstacle_memory.hpp"
#include "local/object_prediction.hpp"
#include "local/planner.hpp"
#include "local/speed_profile.hpp"
#include "local/recovery.hpp"
#include "local/road_memory.hpp"
#include "local/reference_prior.hpp"
#include "local/semantic_reference.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
using namespace local_planning;
void expect(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
Scene empty() {
  Scene s;
  s.grid.rows = 250;
  s.grid.cols = 160;
  s.grid.probability.assign(40000, 255);
  s.grid.coverage.assign(40000, 255);
  for (int i = 0; i <= 40; ++i) {
    s.corridor.push_back({i * .5, 5, -5});
    s.home.push_back({i * .5, 0});
  }
  return s;
}
int main() {
  {
    ObjectPrediction prediction;std::vector<Obstacle> objects;
    auto sample=[](int i,const std::string& group,const std::string& name,double y=0.) {
      Obstacle o;o.id=i;o.classification=group;o.camera_class=name;
      o.source_age=.1;o.uncertainty=.5;o.motion_budget=5.;
      o.polygon={{4.5,y-1.5},{4.8,y-1.5},{4.8,y-1.2},{4.5,y-1.2}};return o;
    };
    for(int i=0;i<6;++i) {
      objects={sample(51,"static_obstacle","traffic_drum"),sample(52,"vehicle","car"),
               sample(53,"dynamic_obstacle","person"),sample(54,"static_obstacle","motorcycle")};
      prediction.apply(objects,Pose{},10.+.08*i,2.,true);
    }
    expect(objects[0].quiescent_geometry_padding && objects[0].motion_budget==2. &&
           std::abs(objects[0].uncertainty-.2)<1e-8 && objects[0].motion_sweep.empty(),
           "six rigid observations retain a nonzero age envelope without declaring static velocity");
    for(int i=1;i<4;++i)expect(!objects[i].quiescent_geometry_padding && objects[i].motion_budget==5.,
        "vehicle, vulnerable and motorcycle categories never receive quiet padding");
    objects={sample(51,"static_obstacle","traffic_drum",.2)};
    prediction.apply(objects,Pose{},10.48,2.,true);
    expect(!objects[0].quiescent_geometry_padding && objects[0].uncertainty==.5,
           "current displacement immediately releases the lower nonzero envelope");
    prediction.clear();
    for(int i=0;i<8;++i){objects={sample(51,"static_obstacle","traffic_drum")};prediction.apply(objects,Pose{},20.,2.,true);}
    expect(!objects[0].quiescent_geometry_padding,"duplicate scans never confirm quiescent geometry");
    prediction.clear();
    for(int i=0;i<6;++i){auto o=sample(51,"static_obstacle","traffic_drum");o.source_age=.16;objects={o};prediction.apply(objects,Pose{},30.+.08*i,2.,true);}
    expect(!objects[0].quiescent_geometry_padding,"aged geometry cannot reduce the past-motion envelope");
    prediction.clear();
    for(int i=0;i<6;++i){auto o=sample(51,"static_obstacle","traffic_drum");o.velocity_floor=3.;objects={o};prediction.apply(objects,Pose{},40.+.08*i,2.,true);}
    expect(objects[0].motion_budget>=3. && objects[0].uncertainty>=.3-1e-8,
           "even quiet geometry cannot undercut a valid absolute-speed magnitude");
  }
  {
    auto s=empty();s.speed=1.;s.acceleration=1.;
    Config c;c.recovery=true;c.low_speed_rate_floor=.1;c.speed_cap=10./3.6;
    Result old;old.path_source="rechecked_recovery";
    old.path={{0,0,0,0,0},{.1,0,0,.20,.1},{10,0,0,.20,10}};
    auto invalid=old;
    expect(!BuildSpeedProfile(s,c,invalid) && invalid.reason=="initial_steering_slew_infeasible",
           "fixture's long previous path is physically infeasible at the measured state");
    expect(BuildRecoveryPath(s,c,old) && BuildSpeedProfile(s,c,old),
           "an infeasible long rechecked path cannot veto today's feasible swept recovery");
    expect(old.path.back().s<10.,"replacement uses its measured shorter horizon, not the old free range");
    s.grid.probability.assign(40000,0);
    Result blocked;
    expect(!BuildRecoveryPath(s,c,blocked),"speed retry cannot create an opening in a blocked current raster");
  }
  {
    auto s=empty();Config c;
    s.measured_steering_valid=true;s.measured_steering_rad=.5;s.steering_source_age=.02;
    expect(std::abs(InitialCurvature(s,c)-c.gain*std::tan(.5)/c.wheelbase)<1e-10,
           "a stopped car with measured wheel angle does not seed every path at zero curvature");
    s.steering_source_age=.101;expect(InitialCurvature(s,c)==0.,"stale measured steering cannot seed a crawl");
    s.steering_source_age=-.01;expect(InitialCurvature(s,c)==0.,"future measured steering is rejected");
    s.steering_source_age=.02;s.speed=2.;s.yaw_rate=.08;
    expect(std::abs(InitialCurvature(s,c)-.04)<1e-10,"normal-speed curvature continues using observed yaw/speed");
  }
  {
    auto s=empty(); Config c; c.low_speed_rate_floor=.1;
    Result r; r.path={{0,0,0,0,0},{.1,0,.01,.19,.1},{4,.4,.3,.19,4}};
    expect(BuildSpeedProfile(s,c,r) && r.steering_cap>.1 && r.steering_cap<.16 &&
           r.speed_reason=="steering_slew", "tight crawl geometry receives a physical steering-speed cap");
    double previous=std::atan(c.wheelbase*r.trajectory.front()[3]/c.gain);
    for(size_t i=1;i<r.trajectory.size();++i) {
      double delta=std::atan(c.wheelbase*r.trajectory[i][3]/c.gain);
      expect(std::abs(delta-previous)<=c.max_steer_rate*.05+1e-8,
             "time sampled crawl trajectory respects calibrated steering slew"); previous=delta;
    }
    s.speed=1.; s.acceleration=1.;
    expect(!BuildSpeedProfile(s,c,r) && r.reason=="initial_steering_slew_infeasible",
           "an infeasible initial state is not hidden by instant speed clipping");
    s.speed=1.;s.acceleration=0.;
    Result future;future.path={{0,0,0,0,0},{6,0,0,0,6},{6.1,0,.01,.19,6.1},{15,1,.2,.19,15}};
    expect(BuildSpeedProfile(s,c,future),
           "a feasible straight braking interval is not rejected by the worst future steering gradient");
    Result straight; straight.path={{0,0,0,0,0},{15,0,0,0,15}};s=empty();
    expect(BuildSpeedProfile(s,c,straight) && straight.steering_cap==c.speed_cap,
           "straight normal geometry has no added steering slowdown");
  }
  {
    auto s=empty();Obstacle o;o.id=3;o.polygon={{8.,-.5},{9.,-.5},{9.,.5},{8.,.5}};s.obstacles={o};
    Config c;c.maneuver_reserve=1.2;
    Result prefix;prefix.path={{0,0,0,0,0},{4,0,0,0,4}};
    expect(BuildSpeedProfile(s,c,prefix) && std::abs(prefix.path.back().s-2.8)<1e-8 &&
           std::abs(prefix.maneuver_reserve_m-1.2)<1e-8,
           "an uncompleted approach preserves turn room rather than creeping to the obstacle");
    Result bypass;bypass.path={{0,0,0,0,0},{11,2.,0,0,11.2}};
    expect(BuildSpeedProfile(s,c,bypass) && bypass.maneuver_reserve_m==0.,
           "a verified bypass beyond the object is not shortened by approach reserve");
  }
  {
    auto s=empty();s.speed=.2;
    Pose a;s.previous.push_back(a);
    for(int i=0;i<90;++i) {
      Pose b=a;b.k=.04;double turn=.5*(a.k+b.k)*.1;
      b.x+=.1*std::cos(a.yaw+turn*.5);b.y+=.1*std::sin(a.yaw+turn*.5);
      b.yaw+=turn;b.s+=.1;s.previous.push_back(b);a=b;
    }
    Config old;old.budget_ms=.0001;old.speed_cap=10./3.6;old.recovery=false;
    auto rejected=Plan(s,old);
    old.low_speed_rate_floor=.5;auto accepted=Plan(s,old);
    expect(!rejected.valid && accepted.valid && accepted.path_source=="rechecked_previous",
           "a feasible low-speed steering transition is not tested at an artificial1m/s");
  }
  {
    ObjectPrediction history;
    auto polygon=[](double x,double y) {return geometry::Polygon2D{{x-.3,y-.3},{x+.3,y-.3},{x+.3,y+.3},{x-.3,y+.3}};};
    std::vector<Obstacle> objects;
    // Ego moves; a physically stationary person does not inherit ego motion.
    for(int i=0;i<6;++i) {
      Obstacle o;o.id=21;o.camera_class="person";o.classification="dynamic_obstacle";o.polygon=polygon(8.-.1*i,0);
      objects={o};history.apply(objects,Pose{.1*i,0,0},10.+.08*i,2.);
    }
    expect(objects[0].motion_sweep.empty(),"dynamic semantic group can be physically stationary");
    history.clear();
    for(int i=0;i<6;++i) {
      Obstacle o;o.id=22;o.camera_class="box";o.classification="static_obstacle";o.polygon=polygon(8.,2.-.08*i);
      objects={o};history.apply(objects,Pose{},10.+.08*i,2.);
    }
    expect(!objects[0].motion_sweep.empty() && objects[0].local_vy<-.9,
           "rigid displacement predicts motion even for a semantic static class");
    expect(std::abs(objects[0].polygon[0].y-1.3)<1e-8,"prediction preserves original measured polygon");
    auto scene=empty();scene.obstacles=objects;Config c;Result check;
    expect(!CheckFootprint(scene,c,Pose{5.,0,0},check) && check.rejected.count("predicted_object_sweep"),
           "future crossing sweep vetoes a presently clear footprint");
    for(int i=0;i<5;++i) {
      objects[0].polygon=polygon(8.,1.6);history.apply(objects,Pose{},10.48+.08*i,2.);
    }
    expect(objects[0].motion_sweep.empty(),"observed stopped object releases motion prediction for bypass");
    history.clear();
    for(int i=0;i<8;++i)history.apply(objects,Pose{},20.,2.);
    expect(objects[0].motion_sweep.empty(),"duplicate scans never confirm motion");
    history.clear();expect(history.size()==0,"epoch reset clears relative object motion");
    for(int i=0;i<6;++i) {
      Obstacle o;o.id=45;o.polygon=polygon(8.,2.-.08*i);
      // Full-history dimensions vary; the most recent confirmed box remains compatible.
      if(i==5)for(auto &p:o.polygon)p.x+=p.x>8.?.14:-.14;
      objects={o};history.apply(objects,Pose{},30.+.08*i,2.);
    }
    objects[0].polygon=polygon(8.,1.52);
    for(auto &p:objects[0].polygon)p.x+=p.x>8.?.24:-.24;
    history.apply(objects,Pose{},30.48,2.);
    expect(!objects[0].motion_sweep.empty(),"a bounded compatible current box keeps a recently confirmed crossing prediction");
    for(int i=0;i<4;++i) {
      objects[0].polygon=polygon(8.,1.52);history.apply(objects,Pose{},30.56+.08*i,2.);
    }
    expect(objects[0].motion_sweep.empty(),"recent prediction cannot renew itself after the object actually stops");
    history.clear();
    for(int i=0;i<5;++i) {
      Obstacle o;o.id=46;o.polygon=polygon(8.,2.);objects={o};
      history.apply(objects,Pose{},40.+.08*i,2.);
    }
    expect(objects[0].motion_sweep.empty(),"a standing crossing actor is not forecast as moving");
    for(int i=0;i<4;++i) {
      objects[0].polygon=polygon(8.,2.-.16*(i+1));
      history.apply(objects,Pose{},40.4+.08*i,2.);
    }
    expect(!objects[0].motion_sweep.empty() && objects[0].local_vy<-1.9,
           "four actual moving observations do not remain diluted by a long stationary prefix");
    history.clear();
    auto near=empty();Obstacle ob;ob.id=3;ob.polygon=polygon(2.,c.width*.5+.36);near.obstacles={ob};
    c.recovery_geometry=true;Result margin;
    expect(!CheckFootprint(near,c,Pose{},margin),"crawl retains ten centimetre minimum physical OBB margin");
    Config responsive=c;responsive.decel=2.;
    expect(BrakingDistance(3.,0.,responsive)<BrakingDistance(3.,0.,c),"responsive comfort brake shortens distance with the same jerk model");
    expect(std::atan(c.wheelbase*c.max_k/c.gain)<.6981317008,"planner maximum curvature remains below calibrated40degree steering");
  }
  {
    ObstacleMemory memory;
    Obstacle box{41,{{5.,-.4},{6.,-.4},{6.,.4},{5.,.4}},"box","static_obstacle"};
    for(int i=0;i<3;++i) {auto current=std::vector<Obstacle>{box};memory.apply(current,Pose{},10.+i*.05,10.+i*.05);}
    std::vector<Obstacle> missing;
    memory.apply(missing,Pose{1.,.2,.1},10.15,10.15);
    expect(missing.size()==1 && missing[0].retained && std::abs(missing[0].source_age-.05)<1e-8,
           "brief asset dropout retains original observation age");
    const double expected=std::cos(.1)*4.+std::sin(.1)*(-.6);
    expect(std::abs(missing[0].polygon[0].x-expected)<1e-9 &&
           std::abs(missing[0].uncertainty-.25)<1e-8,
           "retained geometry changes relative frame once and preserves motion uncertainty");
    missing.clear();memory.apply(missing,Pose{},10.31,10.31);
    expect(missing.empty() && memory.size()==0,"retention cannot become indefinite obstacle ghost");
    for(const auto &name: {"motorcycle","unknown","person","car"}) {
      memory.clear(); box.camera_class=name;
      for(int i=0;i<3;++i){auto cur=std::vector<Obstacle>{box};memory.apply(cur,Pose{},10.+i*.05,10.+i*.05);}
      missing.clear();memory.apply(missing,Pose{},10.15,10.15);
      expect(missing.empty(),"group alone never asserts an immovable asset");
    }
    memory.clear();box.camera_class="box";
    for(int i=0;i<3;++i){auto cur=std::vector<Obstacle>{box};memory.apply(cur,Pose{},10.,10.);}
    missing.clear();memory.apply(missing,Pose{},10.05,10.05);
    expect(missing.empty(),"duplicate scan cannot meet retention confirmation");
    memory.clear();
    for(int i=0;i<3;++i){auto cur=std::vector<Obstacle>{box};memory.apply(cur,Pose{},10.+i*.05,10.+i*.05);}
    box.moving=true;auto cur=std::vector<Obstacle>{box};memory.apply(cur,Pose{},10.15,10.15);
    missing.clear();memory.apply(missing,Pose{},10.20,10.20);
    expect(missing.empty(),"actual moving evidence cancels static asset retention");
    memory.clear();expect(memory.size()==0,"producer/generation reset clears planning retention");
  }

  {
    ReferencePrior prior;Config c;auto s=empty();
    prior.observe(s.home,Pose{},10.);
    const auto hint=prior.aligned(Pose{1.,.25,.04},10.5);
    expect(!hint.empty() && std::abs(hint[0][0]+std::cos(.04)+.25*std::sin(.04))<1e-10,
           "navigation prior transported once using relative odometry");
    expect(prior.aligned(Pose{},12.001).empty(),"source-aged reference expires without renewal");
    prior.observe(s.home,Pose{},10.);
    expect(prior.aligned(Pose{},12.001).empty(),"duplicate source cannot refresh navigation prior");
    expect(prior.aligned(Pose{6.1,0.,0.},11.).empty(),"reference displacement bound");
    expect(prior.aligned(Pose{0.,0.,.21},11.).empty(),"reference yaw bound");
    const auto original=s.grid.probability;
    auto a=SemanticReference(s.grid,c,s.home);
    expect(!a.empty() && std::abs(a[5][1])<1e-10,"current road respects source-aged home preference");
    expect(s.grid.probability==original,"reference hint cannot fill road");
    for (int col=0;col<160;++col) s.grid.probability[100*160+col]=0;
    auto b=SemanticReference(s.grid,c,s.home);Result check;
    expect(!b.empty() && !CheckFootprint(s,c,Pose{10.,0.,0.,0.,0.},check),
           "historical navigation hint cannot override current physical obstruction");
    prior.clear();expect(prior.aligned(Pose{},10.5).empty(),"epoch reset drops reference hint");
  }
  {
    auto s=empty();Config c;
    for(int row=0;row<250;++row) for(int col=0;col<160;++col) {
      const double y=8.-col*.1;
      if(y < -2 || y>5) s.grid.probability[row*160+col]=0;
    }
    const auto original=s.grid.probability;
    auto reference=SemanticReference(s.grid,c);
    expect(!reference.empty() && std::abs(reference[5][1]+.25)<.11,
           "semantic-only cue prefers right-side road without lane selection");
    expect(s.grid.probability==original,"semantic reference cannot fill or edit road");
    auto moved=s.grid;
    moved.rotation=GroundRotation(.01,-.03);
    moved.source_in_plan={-.3,.1,.04};
    auto aligned=SemanticReference(moved,c);
    auto expected=GridToPlan(moved,19.6,-.25);
    expect(!aligned.empty() && std::abs(aligned.back()[0]-expected[0])<1e-8 &&
           std::abs(aligned.back()[1]-expected[1])<1e-8,
           "semantic reference uses existing IMU/relative transform exactly once");
    for(int col=0;col<160;++col) s.grid.probability[100*160+col]=0;
    s.home=SemanticReference(s.grid,c);
    Result blocked;
    expect(!s.home.empty() && !CheckFootprint(s,c,Pose{10.,0.,0.,0.,0.},blocked),
           "soft reference across a mask gap cannot grant collision clearance");
    s.grid.coverage.assign(40000,128);
    expect(SemanticReference(s.grid,c).empty(),"unknown semantic area cannot create a reference");
  }
  try {
    {
      auto s=RecordedStoppedBarrier();Config c;c.recovery=true;c.budget_ms=100.;
      auto seed=Plan(s,c);expect(seed.valid && seed.recovery,"recorded recovery seed");
      s.previous=seed.path;
      // Current narrow visibility rows hide the centroid probes, but actual
      // OBB/exterior and a farther observed opening remain available.
      for(int row:{155,140,120})for(int col=0;col<160;++col)s.grid.coverage[row*160+col]=0;
      expect(!Plan(s,c).valid,"normal previous path never gains recovery privileges implicitly");
      s.previous_recovery=true;
      auto kept=Plan(s,c);
      std::cerr<<"recovery recheck: "<<kept.reason<<" source="<<kept.path_source<<" length="<<(kept.path.empty()?0:kept.path.back().s)<<"\n";
      for(const auto &v:kept.rejected)std::cerr<<v.first<<"="<<v.second<<" ";std::cerr<<"\n";
      expect(kept.valid && kept.recovery && kept.path_source=="rechecked_recovery",
             "current revalidation preserves the explicitly declared recovery policy");
      s.obstacles.push_back({99,{{4.,-4.},{4.1,-4.},{4.1,4.},{4.,4.}}});
      auto blocked=Plan(s,c);
      expect(!blocked.valid || blocked.path_source!="rechecked_recovery",
             "fresh OBB vetoes an old recovery path immediately");
      s.grid.coverage.assign(40000,0);
      expect(!Plan(s,c).valid,"past recovery cannot grant road with no current opening evidence");
    }
    {
      auto s=empty();Config c;s.speed=2.8744620747;s.acceleration=-.2951031;s.aligned_to_plan=true;
      Result r;r.path={{0,0,0,0,0},{3.8123133434,0,0,0,3.8123133434}};
      expect(!BuildSpeedProfile(s,c,r),"recorded comfort brake cannot fit the shortened current path");
      c.controlled_decel=3.;
      expect(BuildSpeedProfile(s,c,r) && r.braking_limit==3. && r.required_stopping_m<r.available_stopping_m,
             "controlled brake keeps an actually stoppable path instead of unconditional full stop");
      expect(r.comfort_stopping_m>r.available_stopping_m && r.speed_reason=="controlled_braking",
             "comfort and controlled braking are separately visible");
      const double expected=std::max(0.,SpaceSpeed(r.available_stopping_m-.1,c.accel,c.delay,c)-.35);
      expect(std::abs(r.space_cap-expected)<1e-9,"stronger current-state brake never raises the cruise governor");
      for(size_t i=1;i<r.trajectory.size();++i) {
        expect(std::abs(r.trajectory[i][5]-r.trajectory[i-1][5])<=c.jerk*.05+1e-7,
               "controlled braking trajectory keeps the original jerk bound");
        expect(r.trajectory[i][5]>=-3.-1e-8,"controlled brake stays below the controller physical bound");
      }
      s.speed=4.;s.acceleration=1.;r.path.back()={1.,0,0,0,1.};
      expect(!BuildSpeedProfile(s,c,r),"insufficient actual brake space still cannot be marked valid");
    }
    {
      auto s=RecordedStoppedBarrier();Config c;c.budget_ms=250.;c.speed_cap=10./3.6;
      expect(!Plan(s,c).valid,"recorded stopped barrier needs an explicit recovery path");
      c.recovery=true;const auto r=Plan(s,c);
      expect(r.valid && r.recovery && r.path.back().s>=.6 && r.path_source=="bounded_recovery_prefix",
             "recorded stopped barrier has a bounded braking-feasible reposition");
      BuildGridIndex(s.grid);c.recovery_geometry=true;
      for(size_t i=1;i<r.path.size();++i){Result check;
        expect(CheckSweep(s,c,r.path[i-1],r.path[i],check),
               "recorded reposition never intersects actual OBB or current road exterior");}
    }
    {
      auto s=empty(); Config c;
      for(int row=0;row<250;++row)for(int col=0;col<160;++col){
        const double y=8.-col*.1;
        if(y < -3. || y > 3. || (row==120 && col>=78 && col<=82))s.grid.probability[row*160+col]=0;
      }
      const auto original=s.grid.probability;BuildGridIndex(s.grid);
      expect(s.grid.probability==original,"exterior indexing never fills semantic holes");
      Result check;
      expect(!CheckFootprint(s,c,Pose{8.,0.,0.,0.,8.},check),"normal sweep rejects current interior nonroad");
      c.recovery_geometry=true;check=Result{};
      expect(CheckFootprint(s,c,Pose{8.,0.,0.,0.,8.},check),"recovery can score an internal occlusion separately");
      check=Result{};
      expect(!CheckFootprint(s,c,Pose{8.,2.5,0.,0.,8.},check) && check.rejected.count("recovery_exterior_nonroad"),
             "recovery cannot cross the measured road exterior");
      s.obstacles.push_back({73,{{8.,-.2},{8.1,-.2},{8.1,.2},{8.,.2}}});check=Result{};
      expect(!CheckFootprint(s,c,Pose{8.,0.,0.,0.,8.},check),"semantic occlusion never overrides a measured OBB");
    }
    {
      auto s=empty(); Config c;c.recovery=true;c.budget_ms=250.;
      // A measured far wall leaves only a short braking-feasible prefix.
      // Current road support exists beyond the nose; no arbitrary4/6m gate.
      s.grid.coverage[160*160+80]=128;
      s.obstacles.push_back({90,{{5.4,-8.},{5.5,-8.},{5.5,8.},{5.4,8.}}});
      auto r=Plan(s,c);
      expect(r.valid && r.recovery && r.path.back().x<4. && r.chosen_cap>0.,
             "short observed assisted prefix permits limited stationary repositioning");
      Config aided=c;aided.blind_assist=true;
      for(size_t i=1;i<r.path.size();++i){Result check;
        expect(CheckSweep(s,aided,r.path[i-1],r.path[i],check),
               "short recovery prefix preserves known nonroad and OBB swept veto");}
      s.grid.coverage.assign(40000,0);
      expect(!Plan(s,c).valid,"unknown-only short prefix never becomes a drivable corridor");
    }
    {
      auto s=empty(); Config c; s.speed=12./3.6; s.acceleration=.8;
      s.aligned_to_plan=true;
      const double needed=s.speed*c.delay+BrakingDistance(s.speed,s.acceleration,c);
      expect(needed>4. && needed<8., "accelerating12kph requires more than4m");
      for(bool recovery : {false,true}) {
        Result r; r.recovery=recovery;
        r.path={{0,0,0,0,0},{4.,0,0,0,4.}};
        expect(!BuildSpeedProfile(s,c,r) && r.reason=="insufficient_stopping_space",
               "recovery must retain lag and jerk stopping feasibility");
        r.path.back()={needed+.3,0,0,0,needed+.3};
        expect(BuildSpeedProfile(s,c,r),"verified longer geometry accommodates actual braking state");
        if(recovery) expect(r.chosen_cap<=3./3.6,"longer recovery does not raise crawl cap");
      }
    }
    {
      auto s=empty(); Config c; c.recovery=true; c.budget_ms=200.;
      s.speed=10./3.6; s.aligned_to_plan=true;
      // Bonnet shadow plus a genuinely blocked far row. A5.3m aided sweep
      // can stop the actual vehicle, while the old fixed4m fallback cannot.
      for(int row=137;row<220;++row) for(int col=70;col<90;++col)
        s.grid.coverage[row*160+col]=128;
      for(int col=0;col<160;++col) s.grid.probability[106*160+col]=0;
      const auto r=Plan(s,c);
      std::cerr << "braking fixture: "<<r.reason<<" valid="<<r.valid<<" recovery="<<r.recovery<<" length="<<r.path.back().s<<" required="<<r.required_stopping_m<<" x="<<r.path.back().x<<"\n";
      expect(r.valid && r.recovery && r.path.back().s<6. &&
             r.path.back().s>r.required_stopping_m,
             "braking-feasible aided path below6m must not be discarded for4m recovery");
    }
    {
      auto s=empty();Config c;c.recovery=true;c.budget_ms=200.;
      s.speed=10./3.6;s.aligned_to_plan=true;
      // Near-body unknown at6.1m truncates the strict path near2m. It is
      // neither an observed obstruction nor permission to drive through OBBs.
      s.grid.coverage[139*160+80]=128;
      auto r=Plan(s,c);
      expect(r.valid && r.recovery && r.path.back().x>=6.,
             "short stopping prefix attempts a longer bounded crawl sweep");
      s.speed=0.;
      for(int col=0;col<160;++col) s.grid.coverage[153*160+col]=128;
      for(int row=0;row<=120;++row) for(int col=0;col<160;++col)
        s.grid.probability[row*160+col]=0;
      r=Plan(s,c);
      expect(r.valid && r.recovery && r.chosen_cap>.3 && r.path.back().s>3.,
             "stationary short prefix explores crawl instead of a permanent zero target");
      for(size_t i=1;i<r.path.size();++i) {
        const auto& a=r.path[i-1];const auto& b=r.path[i];
        const double ds=std::hypot(b.x-a.x,b.y-a.y),
                     turn=std::abs(std::atan(c.wheelbase*b.k/c.gain)-std::atan(c.wheelbase*a.k/c.gain));
        expect(turn<=c.max_steer_rate*ds/std::max(1.,s.speed)+1e-8,
               "crawl generation obeys the same steering slew as normal local candidates");
      }
      s.obstacles.push_back({55,{{5.,-8.},{6.,-8.},{6.,8.},{5.,8.}}});
      r=Plan(s,c);
      if(r.valid) for(const auto& p:r.path) {
        CartesianState ego{p.x,p.y,p.yaw,p.k,0,0};
        auto body=MakeEgoFootprint(ego,VehicleShape{c.width,c.length,c.rear},0.);
        expect(geometry::ConvexPolygonDistance(body,s.obstacles[0].polygon)>0.,
               "crawl recovery never selects an actual measured OBB overlap");
      }
    }
    { // Equal geometry must not rank differently due to analytic/lattice spacing.
      auto scene=empty(); Config config;
      scene.previous={{0,.3,0,0,0},{10,.3,0,0,10}};
      auto cost=[&](double step) { double value=0.; Pose a{0,.2,0,0,0};
        for(double x=step;x<10.+1e-6;x+=step) {
          Pose b{x,.2+.01*x,0,.001*x,x};
          value+=SegmentQualityCost(scene,config,a,b,0); a=b;
        } return value; };
      expect(std::abs(cost(.2)-cost(.5))<1e-8,"common final path cost independent of segmentation");
      auto previous_plan=Plan(scene,config);
      expect(previous_plan.valid,"previous test base plan");
      scene.previous=previous_plan.path;
      scene.obstacles.push_back({77,{{6,-2},{7,-2},{7,2},{6,2}}});
      auto obstructed=Plan(scene,config);
      for(const auto &p:obstructed.path) {Result check;
        expect(CheckFootprint(scene,config,p,check),"reused path obeys new current OBB");}
    }
    {
      auto s=empty();Config c;c.budget_ms=300.;
      for(auto& p:s.home) p[1]=2.;
      auto bypass=Plan(s,c);expect(bypass.valid,"initial feasible lateral bypass");
      s.previous=bypass.path;
      for(auto& p:s.home) p[1]=0.;
      auto returned=Plan(s,c);
      expect(returned.valid && std::abs(returned.path.back().y)<.3,
             "clear road returns home instead of locking in a previous bypass offset");
      for(size_t i=1;i<returned.path.size();++i) {
        Result check;
        expect(CheckSweep(s,c,returned.path[i-1],returned.path[i],check),
               "history decay never overrides the current full-body collision sweep");
      }
    }
    {
      auto s=empty();Config c;c.recovery=true;c.budget_ms=300.;
      for(int row=0;row<250;++row) for(int col=0;col<160;++col) {
        const double x=20.-.1*row,y=8.-.1*col;
        s.grid.probability[row*160+col]=(y>=.3 && y<=4.3)?255:0;
        if(x<4.6) s.grid.coverage[row*160+col]=128;
      }
      auto r=Plan(s,c);
      expect(!r.valid,"disconnected partial road cannot override a measured exterior boundary");
      for(int row=0;row<250;++row)for(int col=0;col<160;++col){
        double y=8.-col*.1;s.grid.probability[row*160+col]=(y>=-1.4&&y<=2.6)?255:0;}
      r=Plan(s,c);
      expect(r.valid && r.recovery,"connected partial near-body road exercises recovery goals");
      for(const auto& goal:r.goals)
        expect(goal[1]>=-1.4+c.width*.5-1e-6 && goal[1]<=2.6-c.width*.5+1e-6,
               "recovery goals contain the physical body inside their observed road interval");
    }
    // Tiny overspeed must converge, not become a full-brake trough. The old
    // scalar-cap + average-acceleration loop created an unnecessary trough.
    for (double initial_a : {-1., 0., 1.}) {
      double v = 4.3, a = initial_a;
      for (int i = 0; i < 100; ++i) {
        auto next =
            AdvancePhysicalSpeed({15. / 3.6, v, a, .05}, {1., 1.5, 4.8});
        expect(std::abs(next.target_acceleration - a) <= 4.8 * .05 + 1e-8,
               "physical speed jerk bound");
        v = next.target_speed;
        a = next.target_acceleration;
        expect(v >= 4.14, "small overspeed cannot create braking trough");
      }
      expect(std::abs(v - 15. / 3.6) < 1e-8 && std::abs(a) < 1e-8,
             "physical profile settles at cap with zero acceleration");
    }
    MotionHistory h;
    expect(h.add(1, 1, .1), "first motion");
    expect(h.add(1.1, 1, .1), "second motion");
    auto m = h.at(1.05);
    expect(m && std::abs(m->x - .05) < .001, "motion interpolation");
    expect(!h.at(.99), "cannot use future sample for old state");
    auto epoch = h.epoch;
    h.add(.9, 1, 0);
    expect(h.epoch == epoch + 1, "regression resets epoch");
    expect(!h.at(1.1), "prior epoch gone");
    epoch = h.epoch;
    h.add(2, 1, 0);
    expect(h.epoch == epoch + 1, "gap resets epoch");
    MotionHistory reverse;
    reverse.add(10., -1., 0.);
    const auto reverse_epoch = reverse.epoch;
    reverse.add(10.1, -1., 0.);
    expect(reverse.epoch == reverse_epoch && reverse.at(10.1)->x < -.099,
           "reverse observations preserve signed motion and epoch");
    expect(reverse.at(10.19) && !reverse.at(10.201),
           "motion forecast bounded at100ms, beyond valid83ms IMU cadence");
    expect(!reverse.add(10.1, -1., 0.) && reverse.epoch == reverse_epoch,
           "duplicate state does not reset history");
    Grid aligned;
    aligned.rotation = GroundRotation(.1, -.04);
    aligned.source_in_plan = {-1.2, .15, -.08};
    for (double x : {-1., 3., 15.})
      for (double y : {-3., 0., 4.}) {
        const auto q = GridToPlan(aligned, x, y);
        const auto back = PlanToGrid(aligned, q[0], q[1]);
        expect(std::hypot(back[0] - x, back[1] - y) < 1e-10,
               "tilt and relative translation/yaw are inverted once");
      }
    auto r = GroundRotation(.1, -.05);
    auto normal = Rotate(r, -.1, .05, 1);
    expect(std::hypot(normal[0], normal[1]) < 1e-10,
           "plane levelling exactly once");
    Config c;
    c.budget_ms = 500.;
    Scene s = empty();
    Result check;
    expect(CheckFootprint(s, c, {}, check), "empty free scene");
    Obstacle o;
    o.id = 42;
    o.polygon = {{4, -.2}, {4.4, -.2}, {4.4, .2}, {4, .2}};
    s.obstacles.push_back(o);
    Pose p;
    p.x = 1.;
    expect(!CheckFootprint(s, c, p, check) && check.blocking_id == 42,
           "all eight-corner hull veto independent of semantic free");
    s = empty();
    int idx = std::lround((20 - 4.) / .1) * 160 + 80;
    s.grid.coverage[idx] = 0;
    check = {};
    expect(!CheckFootprint(s, c, p, check) &&
               check.rejected.count("unobserved"),
           "unknown is not free");
    // The same source nonroad cell moves behind the current footprint after
    // a 6m relative displacement; no raster resampling changes its evidence.
    s.grid.source_in_plan = {-6., 0., 0.};
    check = {};
    expect(CheckFootprint(s, c, p, check), "source cell aligned away from ego");
    s.grid.source_in_plan = {0., 0., 0.};
    c.blind_assist = true;
    check = {};
    expect(CheckFootprint(s, c, p, check) && check.assisted,
           "explicit bounded assist");
    s.grid.coverage[idx] = 255;
    s.grid.probability[idx] = 0;
    check = {};
    expect(!CheckFootprint(s, c, p, check),
           "assist cannot erase observed nonroad");
    c.blind_assist = false;
    s = empty();
    auto result = Plan(s, c);
    std::cerr << "empty valid=" << result.valid << " reason=" << result.reason
              << " length=" << result.path.back().s
              << " ms=" << result.elapsed_ms << " caps=" << result.chosen_cap
              << "\n";
    expect(result.valid, "empty scene yields kinematic path");
    expect(result.chosen_cap <= 25. / 3.6, "25 is ceiling");
    expect(result.speed_reason == "verified_stopping_space",
           "limited range limits speed");
    expect(result.path.front().x == 0 && result.path.front().y == 0,
           "start is rear axle not free-space centroid");
    for (const auto &q : result.path)
      expect(std::abs(q.k) <= c.max_k + 1e-9, "curvature limit");
    // A distant curved home lane must retain its tangent after the initial
    // connection; a straight scalar lateral target used to understeer here.
    s = empty();
    for (auto &q : s.home)
      q[1] = .005 * q[0] * q[0];
    auto curved = Plan(s, c);
    expect(curved.valid && curved.path.back().x > 12.,
           "curved family yields long path");
    const auto tail = curved.path.back();
    expect(std::abs(tail.y - .005 * tail.x * tail.x) < .15 && tail.yaw > .08,
           "family tangent and lateral position survive the transition");
    // An obstacle splits free-space: do not target the mean through its centre.
    s = empty();
    // Family paint is a navigation reference, not the physical road edge. The
    // measured free road below is wider; avoidance may cross either white line.
    for (auto &paint : s.corridor) {
      paint[1] = 1.75;
      paint[2] = -1.75;
    }
    s.obstacles.push_back({9, {{8.5, -.4}, {9.3, -.4}, {9.3, .4}, {8.5, .4}}});
    for (int row = 0; row < 250; ++row)
      for (int col = 0; col < 160; ++col)
        if (std::abs(8. - col * .1) > 3.5)
          s.grid.probability[row * 160 + col] = 0;
    auto avoid = Plan(s, c);
    std::cerr << "obstacle valid=" << avoid.valid
              << " length=" << avoid.path.back().s << " ms=" << avoid.elapsed_ms
              << " end=" << avoid.path.back().x << "," << avoid.path.back().y
              << "," << avoid.path.back().yaw << " goals=" << avoid.goals.size()
              << "\n";
    for (auto g : avoid.goals)
      std::cerr << "goal=" << g[0] << "," << g[1] << "\n";
    expect(avoid.valid && avoid.path.back().x > 10.,
           "bounded kinematic search must pass isolated central obstacle");
    for (size_t k = 1; k < avoid.path.size(); ++k) {
      Result swept;
      expect(CheckSweep(s, c, avoid.path[k - 1], avoid.path[k], swept),
             "selected path continuous collision validation");
    }
    expect(avoid.goals.size() >= 2, "split openings produce separate goals");
    bool tried_left=false,tried_right=false;
    for(const auto& candidate:avoid.candidates) {
      tried_left|=candidate.lateral_target>.1;
      tried_right|=candidate.lateral_target<-.1;
    }
    expect(tried_left && tried_right,
           "bounded first candidate window tries both bypass sides before exhausting straight transitions");
    bool beyond_paint = false;
    for (const auto &q : avoid.path)
      beyond_paint |= std::abs(q.y) + c.width * .5 > 1.75;
    expect(beyond_paint,
           "valid swept avoidance can cross paint inside measured free road");
    // A shorter unused left boundary must not truncate a right-home path.
    s = empty();
    s.corridor.resize(16);
    auto support = Plan(s, c);
    expect(support.valid && support.path.back().x > 14.,
           "paint support is not a physical stopping boundary");
    RoadMemory memory;
    auto observed = empty().grid;
    memory.observe(observed, Pose{}, 10.);
    auto hidden = observed;
    hidden.coverage.assign(40000, 128);
    hidden.probability.assign(40000, 0);
    memory.apply(hidden, Pose{1., 0, 0}, 11.);
    const int near = 150 * 160 + 80; // x5, beyond the physical bonnet
    expect(hidden.recent_road_age[near] > 0,
           "past actually observed road is aligned into near ego occlusion");
    expect(hidden.recent_road_age[50 * 160 + 80] == 0,
           "memory cannot fill distant unobserved road");
    hidden.coverage[near] = 255;
    memory.apply(hidden, Pose{1., 0, 0}, 11.);
    expect(hidden.recent_road_age[near] == 0,
           "current observed nonroad always overrides old road");
    hidden.coverage[near] = 128;
    memory.apply(hidden, Pose{1., 0, 0}, 16.);
    expect(hidden.recent_road_age[near] == 0,
           "expired evidence is not renewed");
    memory.clear();
    memory.apply(hidden, Pose{1., 0, 0}, 11.);
    expect(hidden.recent_road_age[near] == 0, "epoch reset erases road memory");
    memory.observe(observed, Pose{}, 10.);
    auto obstruction = observed;
    obstruction.probability[near] = 0;
    memory.observe(obstruction, Pose{}, 10.5);
    memory.apply(hidden, Pose{}, 11.);
    expect(hidden.recent_road_age[near] == 0,
           "newer observed obstruction blocks older free road");
    s = empty();
    s.grid.coverage[idx] = 128;
    s.grid.recent_road_age.assign(40000, 0.);
    s.grid.recent_road_age[idx] = .5;
    check = {};
    expect(CheckFootprint(s, c, p, check),
           "causal near road can support footprint");
    s.obstacles.push_back(o);
    check = {};
    expect(!CheckFootprint(s, c, p, check),
           "fresh obstacle vetoes remembered road");
    s = empty();
    s.home.resize(16); // 7.5m rear-axle path cannot stop from25km/h.
    s.speed = 25. / 3.6;
    s.acceleration = .8;
    expect(!Plan(s, c).valid,
           "positive acceleration cannot invent emergency stopping space");
    s = empty();
    s.speed = 5.;
    const int soft_edge = 160 * 160 + 94; // x4, y-1.4: outside hard envelope
    s.grid.probability[soft_edge] = 0;
    check = {};
    expect(CheckFootprint(s, c, p, check) && check.soft_margin_cells > 0,
           "speed comfort margin is a cost, not physical collision");
    s.grid.probability[160 * 160 + 91] = 0; // y-1.1: hard footprint margin
    check = {};
    expect(!CheckFootprint(s, c, p, check),
           "observed nonroad inside body and minimum margin still vetoes");
    // Reusing an old path must not carry a stale 100m arc-length into speed
    // cap.
    s = empty();
    s.previous = {{0, 0, 0, 0, 100}, {.5, 0, 0, 0, 100.5}, {1, 0, 0, 0, 101}};
    auto reuse = Plan(s, c);
    expect(reuse.path.back().s < 20 && reuse.space_cap < 25. / 3.6,
           "old arc length cannot invent stopping room");
    // A swept segment detects a narrow obstacle between clear endpoints.
    s = empty();
    s.obstacles.push_back({10, {{4.6, -.1}, {4.7, -.1}, {4.7, .1}, {4.6, .1}}});
    Result sweep;
    expect(!CheckSweep(s, c, Pose{0, 0, 0, 0, 0}, Pose{8, 0, 0, 0, 8}, sweep),
           "no tunnelling through sparse old path points");
    s = empty();
    s.age = .4;
    expect(!Plan(s, c).valid, "stale cannot plan");
    s = empty();
    s.grid.probability.assign(40000, 0);
    expect(!Plan(s, c).valid, "all blocked cannot plan");
    // The recovery experiment may relax geometric rejection, but cannot
    // invent road evidence when every cell is blocked/unknown/stale.
    c.recovery = true;
    expect(!Plan(s, c).valid, "recovery cannot invent all-blocked road");
    s = empty();
    s.grid.coverage.assign(40000, 0);
    expect(!Plan(s, c).valid, "recovery cannot invent all-unknown road");
    s = empty();
    // Near hood mask prevents strict sweep; broad visible interval ends at
    // search boundary. Integer endpoint flush must retain that component.
    for (int row = 140; row < 220; ++row)
      for (int col = 65; col < 95; ++col)
        s.grid.coverage[row * 160 + col] = 128;
    auto creep = Plan(s, c);
    expect(creep.valid && creep.recovery && creep.chosen_cap <= 3. / 3.6,
           "visible near-field recovery is explicitly capped at 3km/h");
    expect(std::abs(creep.path.back().y) < .01,
           "broad free road retains home lane instead of off-centre mean");
    // With only the bounded camera hood blind spot, distant barriers must
    // influence a full-length swept candidate, rather than a four-metre arc.
    s = empty();
    for (int row = 137; row < 220; ++row)
      for (int col = 70; col < 90; ++col)
        s.grid.coverage[row * 160 + col] = 128;
    s.obstacles.push_back({73, {{10., -1.}, {11., -1.}, {11., 1.}, {10., 1.}}});
    auto ahead = Plan(s, c);
    expect(ahead.valid && ahead.recovery && ahead.path.back().x > 11.,
           "near-field assistance keeps distant obstacle lookahead");
    Config aided = c;
    aided.blind_assist = true;
    for (size_t k = 1; k < ahead.path.size(); ++k) {
      Result check;
      expect(CheckSweep(s, aided, ahead.path[k - 1], ahead.path[k], check),
             "assisted long path never softens a known OBB collision");
    }
    s.age = .4;
    expect(!Plan(s, c).valid, "recovery cannot mask stale perception");
    c.recovery = false;
    s = empty();
    s.speed = 25. / 3.6;
    auto fast = Plan(s, c);
    expect(!fast.valid && fast.reason == "insufficient_stopping_space",
           "unsafe speed cannot be made safe by cap alone");
    std::string file =
        "/tmp/local-journal-" + std::to_string(getpid()) + ".jsonl";
    {
      Journal j(file);
      j.push("{\"test\":1}");
    }
    std::ifstream f(file);
    std::string line;
    std::getline(f, line);
    expect(line == "{\"test\":1}", "shutdown drains journal");
    bool refused = false;
    try {
      Journal j(file);
    } catch (...) {
      refused = true;
    }
    expect(refused, "journal cannot overwrite");
    std::remove(file.c_str());
    std::cout << "LOCAL-1 core regression passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
