#include "shadow_localization/anchor.hpp"
#include "shadow_localization/input_alignment.hpp"
#include "shadow_localization/input_epoch.hpp"
#include "shadow_localization/gps_mount.hpp"
#include "shadow_localization/path_memory.hpp"
#include "shadow_localization/following_policy.hpp"
#include "local/planner.hpp"
#include "Planning/ShadowApproach.hpp"
#include "Planning/JammingDetector.hpp"
#include <fstream>
#include <filesystem>
#include <unistd.h>
#include <iostream>
using namespace shadow_localization;
using local_planning::Motion;
int failures=0;
void check(bool yes,const char* text){std::cout<<(yes?"PASS ":"FAIL ")<<text<<'\n';failures+=!yes;}
void fix(Anchor& a,double t,double x=0,double yaw=0,double speed=0){
  Motion m;m.t=t;m.v=speed;a.motion(m);a.observe(m,x,0,yaw);
}
void warm(Anchor& a,double t){for(int i=0;i<=20;++i)fix(a,t+.05*i);}
int main(){
  {
    auto gap=FollowingReferenceGap(3.845,10.,7.6,3.);
    check(gap&&std::abs(*gap-9.245)<1e-9,
          "3m following means bumper spacing plus both body/reference-point offsets");
    check(!FollowingReferenceGap(3.845,10.,11.,3.)&&
          !FollowingReferenceGap(3.845,10.,7.6,NAN),
          "invalid leader geometry or unknown spacing cannot create a valid zero gap");
  }
  {
    PathMemory memory;std::string why;
    CartesianPath path;for(int i=0;i<20;++i){path.x.push_back(i*.25);path.y.push_back(0);
      path.yaw.push_back(0);path.kappa.push_back(0);path.v.push_back(i==19?0:2);path.a.push_back(0);}
    CartesianState ego{.52,0,0,0,1.5,0};memory.remember(path,1.);
    auto remaining=memory.remaining(ego,1.3,why);
    check(remaining&&remaining->x.front()==ego.x&&remaining->v.front()==ego.v&&remaining->v.back()==0,
          "path memory trims passed geometry and seeds current state without extending its terminal stop");
    check(!memory.remaining(ego,1.61,why)&&!memory.remaining(ego,.9,why),
          "path memory rejects stale and retrograde source times");
    ego.y=.36;check(!memory.remaining(ego,1.3,why),"path memory rejects lateral localization discontinuity");
    ego.y=0;ego.yaw=.3;check(!memory.remaining(ego,1.3,why),"path memory rejects heading discontinuity");
    memory.clear();check(!memory.remaining(ego,1.3,why),"epoch reset retires the previous spatial path");
  }
  check(ForwardPlanningSpeed(-.004)&&*ForwardPlanningSpeed(-.004)==0.&&
        ForwardPlanningSpeed(1.)&&*ForwardPlanningSpeed(1.)==1.,
        "near-zero rollback cannot seed a negative forward trajectory");
  check(!ForwardPlanningSpeed(-.051)&&!ForwardPlanningSpeed(NAN),
        "reverse limit and nonfinite motion remain hard rejects");
  using asmc_planner::ShadowApproachDistance;
  check(ShadowApproachDistance(30./3.6,0)==2*30./3.6,
        "main keeps its original two-second preparation distance");
  const double fast_approach=ShadowApproachDistance(30./3.6,15./3.6);
  check(fast_approach>38. && fast_approach<41.,
        "localization reserves braking distance before a faster NORMAL entry");
  check(ShadowApproachDistance(15./3.6,15./3.6,fast_approach)==fast_approach,
        "slowing cannot release and reapply its own preparation speed cap");
  const auto curved=std::filesystem::temp_directory_path()/
      ("asmc-localization-approach-"+std::to_string(getpid())+".txt");
  {std::ofstream f(curved);for(int i=0;i<=200;++i){const double t=i*.025;f<<20*std::sin(t)<<' '<<20*(1-std::cos(t))<<'\n';}}
  asmc_planner::ShadowZone zone(curved.string(),70.,90.,2.,.6);
  const double x=20*std::sin(2.),y=20*(1-std::cos(2.));
  check(!zone.Approaching(x,y,2.,40.)&&zone.ApproachingAlongRoute(x,y,2.,40.),
        "curved upstream route prepares even when the entry tangent differs");
  check(!zone.ApproachingAlongRoute(x,y,2.,20.) &&
        !zone.ApproachingAlongRoute(x,y,2.+M_PI,40.) &&
        !zone.ApproachingAlongRoute(x+100,y,2.,40.),
        "approach rejects insufficient distance, opposite heading and another road");
  std::filesystem::remove(curved);
  const auto flat_mount=GpsMountXY(.35,0,1.25,0,0,0);
  check(std::abs(flat_mount[0]-.35)<1e-12&&std::abs(flat_mount[1])<1e-12,
        "flat GPS mount preserves the existing horizontal lever arm");
  const auto bank_mount=GpsMountXY(.35,0,1.25,.1,0,0);
  check(std::abs(bank_mount[0]-.35)<1e-12&&std::abs(bank_mount[1]+1.25*std::sin(.1))<1e-12,
        "GPS height is projected laterally once on a bank");
  const auto slope_mount=GpsMountXY(.35,0,1.25,0,.1,M_PI/2);
  check(std::abs(slope_mount[0])<1e-12&&std::abs(slope_mount[1]-(.35*std::cos(.1)+1.25*std::sin(.1)))<1e-12,
        "GPS slope projection precedes grid heading rotation");
  local_planning::MotionHistory rate_motion,orientation_motion;
  for(int i=0;i<1000;++i){const double t=1.+.05*i;
    rate_motion.add(t,4.,.001);
    orientation_motion.add(t,4.,.001,0.,0.,0.);
  }
  auto rate_end=rate_motion.at(50.95),orientation_end=orientation_motion.at(50.95);
  check(rate_end&&std::abs(rate_end->yaw-.04995)<1e-8,
        "default LOCAL rate integration is unchanged");
  check(orientation_end&&std::abs(orientation_end->y)<1e-8&&
        std::abs(orientation_end->x-199.8)<1e-7,
        "measured constant IMU heading does not accumulate a gyro-rate bias");
  local_planning::MotionHistory wrap_motion;
  wrap_motion.add(1.,1.,0.,0.,0.,179.*M_PI/180.);
  wrap_motion.add(1.1,1.,0.,0.,0.,-179.*M_PI/180.);
  check(wrap_motion.at(1.1)&&std::abs(wrap_motion.at(1.1)->yaw-2.*M_PI/180.)<1e-9,
        "measured orientation unwraps across pi in the same relative frame");
  const auto before_jump=wrap_motion.epoch;
  check(!wrap_motion.add(1.12,1.,0.,0.,0.,0.)&&wrap_motion.epoch>before_jump,
        "an IMU orientation jump invalidates the integration epoch");
  local_planning::MotionHistory ordered;ordered.reorder_tolerance_s=.1;
  ordered.add(10.,1.,0.,0.,0.,0.);ordered.add(10.05,1.,0.,0.,0.,0.);
  const auto ordered_epoch=ordered.epoch;
  check(!ordered.add(10.0494,20.,1.,0.,0.,1.)&&ordered.epoch==ordered_epoch&&
        ordered.at(10.05)&&std::abs(ordered.at(10.05)->x-.05)<1e-9,
        "a measured sub-millisecond reordered packet neither resets nor changes relative pose");
  ordered.add(9.,1.,0.,0.,0.,0.);
  check(ordered.epoch>ordered_epoch&&!ordered.at(10.05),
        "large source clock regression still invalidates the prior motion epoch");
  local_planning::MotionHistory point;point.velocity_reference_offset_m=1.4;
  local_planning::MotionHistory rollback;rollback.reverse_tracking_limit_mps=.15;
  rollback.add(1.,0.,0.);const auto rollback_epoch=rollback.epoch;
  for(int i=1;i<=15;++i)rollback.add(1.+.02*i,-.07,0.);
  rollback.add(1.32,0.,0.);
  check(rollback.epoch==rollback_epoch&&rollback.at(1.32)&&
        rollback.at(1.32)->x<-.015&&rollback.at(1.32)->x>-.025,
        "bounded brake rollback is integrated with its sign without losing the anchor epoch");
  check(!rollback.add(1.34,-.2,0.)&&rollback.epoch>rollback_epoch,
        "reverse beyond the bounded localization tracker still invalidates the epoch");
  local_planning::MotionHistory generic_reverse;generic_reverse.add(1.,0.,0.);
  const auto generic_reverse_epoch=generic_reverse.epoch;
  check(!generic_reverse.add(1.02,-.07,0.)&&generic_reverse.epoch>generic_reverse_epoch,
        "generic LOCAL preserves its original reverse rejection policy");
  for(int i=0;i<=100;++i)point.add(1.+.05*i,0.,.1,0.,0.,i*.005);
  const auto point_end=point.at(6.);
  check(point_end&&std::abs(point_end->x-1.4*(std::cos(.5)-1))<1e-9&&
        std::abs(point_end->y-1.4*std::sin(.5))<1e-9,
        "velocity reference correction is the exact rigid-point displacement, once per sample");
  const auto point_future=point.at(6.025);
  check(point_future&&std::abs(point_future->x-1.4*(std::cos(.5025)-1))<1e-9&&
        std::abs(point_future->y-1.4*std::sin(.5025))<1e-9,
        "short pose extrapolation uses the same reference point");

  local_planning::MotionHistory turning;turning.velocity_turn_gain_m2ps=1.54;
  for(int i=0;i<=20;++i)turning.add(1.+.05*i,2.,.2,0.,0.,i*.01);
  const auto circle=turning.at(2.);
  // Midpoint forward integration has a known O(dt^2) circular-arc error.
  check(circle&&std::abs(circle->x-(10*std::sin(.2)+.77*(std::cos(.2)-1)))<1e-4&&
        std::abs(circle->y-(10*(1-std::cos(.2))+.77*std::sin(.2)))<1e-4,
        "speed-dependent turn correction matches an independently integrated circular path");
  check(std::abs(turning.reference_offset(4.)-.385)<1e-12&&turning.reference_offset(0)==1.54,
        "turn model scales with speed and stays bounded at standstill");
  local_planning::MotionHistory ramp;ramp.velocity_turn_gain_m2ps=1.54;
  ramp.add(1.,0.,0.,0.,0.,0.);ramp.add(1.1,2.,0.,0.,0.,0.);
  check(std::abs(ramp.at(1.1)->x-.1)<1e-12&&std::abs(ramp.at(1.1)->y)<1e-12,
        "changing speed without changing heading creates no artificial reference-point jump");
  const auto prediction=turning.at(2.025);
  check(prediction&&std::abs(prediction->y-(circle->y+2*.025*std::sin(.2025)+
        .77*(std::sin(.205)-std::sin(.2))))<1e-12,
        "short extrapolation and measured integration use the same turn model");

  Anchor a;fix(a,1);check(!a.at(1).valid,"one aligned fix is not a departure anchor");
  for(int i=1;i<20;++i)fix(a,1+.05*i);
  check(!a.at(1.95).valid,"minimum one second even with enough samples");
  fix(a,2);check(a.at(2).valid&&a.samples()>=10,"one second and ten aligned samples ready");
  check(!a.at(.9).valid,"never uses a future anchor");
  check(a.at(2.7).source=="dead_reckoning","GPS loss explicitly switches to DR");
  fix(a,3,.2);check(a.at(3).source=="recovery_verify"&&std::abs(a.at(3).x)<1e-12,"recovery candidate does not move DR pose");
  for(int i=1;i<=20;++i)fix(a,3+.05*i,.2);
  check(a.at(4).source=="recovery_blend"&&std::abs(a.at(4).x)<1e-12,"one second recovery validation before blending");
  for(int i=1;i<=10;++i)fix(a,4+.05*i,.2);
  check(std::abs(a.at(4.5).x-.1)<1e-8,"one second blend halfway preserves continuity");
  for(int i=11;i<=20;++i)fix(a,4+.05*i,.2);
  check(a.at(5).source=="gps_anchored"&&std::abs(a.at(5).x-.2)<1e-8,"validated recovery completes");
  fix(a,6,.5,0,2);check(!a.at(6).valid&&a.state=="reanchor_stop","GPS innovation over 25cm invalidates path");
  for(int i=1;i<=30;++i)fix(a,6+.05*i,.5,0,2);
  check(!a.at(7.5).valid,"moving fixes cannot silently reinitialize failed anchor");
  for(int i=0;i<=20;++i)fix(a,7.6+.05*i,.5);
  check(a.at(8.6).valid,"reinitialize only after stopped stable one second window");
  fix(a,9,.5,3*M_PI/180,1);check(!a.at(9).valid,"heading innovation over two degrees invalidates path");
  Anchor window;window.validate_recovery_window=true;warm(window,1);
  Anchor tracking_noise;tracking_noise.validate_recovery_window=true;warm(tracking_noise,1);
  fix(tracking_noise,2.2,.3);
  check(tracking_noise.state=="recovery_verify"&&tracking_noise.at(2.2).valid&&
        std::abs(tracking_noise.at(2.2).x)<1e-12,
        "a noisy tracking fix retains the last validated pose during bounded verification");
  for(int i=1;i<=6;++i)fix(tracking_noise,2.2+.2*i,.02);
  for(int i=1;i<=5;++i)fix(tracking_noise,3.4+.2*i,.02);
  check(tracking_noise.state=="gps_anchored"&&std::abs(tracking_noise.at(4.4).x-.02)<1e-8,
        "an isolated tracking outlier cannot insert a stop or an unvalidated correction");
  Anchor tracking_jump;tracking_jump.validate_recovery_window=true;warm(tracking_jump,1);
  for(int i=0;i<=5;++i)fix(tracking_jump,2.2+.2*i,.3);
  check(!tracking_jump.at(3.2).valid,"a persistent tracking candidate over 25cm still stops");
  Anchor tracking_gross;tracking_gross.validate_recovery_window=true;warm(tracking_gross,1);fix(tracking_gross,2.2,.5);
  check(!tracking_gross.at(2.2).valid,"a gross tracking jump still stops immediately");
  Anchor unvalidated;unvalidated.validate_recovery_window=true;warm(unvalidated,1);
  for(int i=1;i<=153;++i)fix(unvalidated,2.+.8*i,.01);
  check(!unvalidated.at(124.4).valid,
        "unverified sporadic GPS cannot refresh the hard dead-reckoning time budget");
  fix(window,3,.31);
  check(window.state=="recovery_verify"&&std::abs(window.at(3).x)<1e-12,
        "a noisy recovered sample does not correct the DR pose");
  for(int i=1;i<=5;++i)fix(window,3+.2*i,.19);
  check(window.state=="recovery_blend"&&window.innovation_m<.25&&std::abs(window.at(4).x)<1e-12,
        "full one-second candidate must pass the unchanged 25cm bound before blending");
  for(int i=1;i<=5;++i)fix(window,4+.2*i,.21);
  check(window.state=="gps_anchored"&&std::abs(window.at(5).x-.21)<1e-8,
        "validated noisy GPS recovery blends continuously for a full second");
  Anchor advancing;advancing.validate_recovery_window=true;warm(advancing,1);
  for(int i=0;i<=5;++i)fix(advancing,3+.2*i,.2);
  for(int i=1;i<=5;++i)fix(advancing,4+.2*i,.3);
  check(advancing.state=="gps_anchored"&&std::abs(advancing.at(5).x-.2)<1e-9,
        "blend evidence is checked against the currently published anchor; correction still ends at its validated target");
  fix(advancing,5.2,.8);
  check(!advancing.at(5.2).valid,"a genuine new GPS jump still stops after a completed blend");
  Anchor sparse_blend;sparse_blend.validate_recovery_window=true;warm(sparse_blend,1);
  for(int i=0;i<=5;++i)fix(sparse_blend,3+.2*i,.24);
  check(std::abs(sparse_blend.at(4.1).x-.024)<1e-9,
        "a validated blend interpolates continuously between 5Hz GPS updates");
  fix(sparse_blend,4.2,.4);
  check(sparse_blend.state=="recovery_blend"&&sparse_blend.innovation_m<.25,
        "next fix is compared with the blend at its measurement time, not its previous keyframe");
  const double frozen=sparse_blend.at(4.5).x;
  check(std::abs(sparse_blend.at(5.).x-frozen)<1e-9,
        "GPS loss freezes a blend at the freshness boundary instead of rolling it back");
  fix(sparse_blend,5.2,.2);
  check(std::abs(sparse_blend.at(4.1).x-.024)<1e-9,
        "a later recovery cannot alter a previously validated blend in history");
  Anchor bad_window;bad_window.validate_recovery_window=true;warm(bad_window,1);
  for(int i=0;i<=5;++i)fix(bad_window,3+.2*i,.3);
  check(!bad_window.at(4).valid&&bad_window.state=="reanchor_stop",
        "a stable recovery candidate over 25cm still stops without blending");
  Anchor gross;gross.validate_recovery_window=true;warm(gross,1);fix(gross,3,.5);
  check(!gross.at(3).valid,"gross recovered GPS jumps stop immediately even with window validation");
  Anchor unstable;unstable.validate_recovery_window=true;warm(unstable,1);
  for(int i=0;i<=8;++i)fix(unstable,3+.2*i,i%2?.3:0.);
  check(!unstable.at(4.6).valid&&unstable.state=="reanchor_stop",
        "inconsistent recovery cannot keep an unvalidated candidate alive indefinitely");
  Anchor sparse;fix(sparse,1);fix(sparse,2);check(!sparse.at(2).valid,"gapped two-fix warmup is refused");
  Anchor gps5hz;
  for(int i=0;i<9;++i)fix(gps5hz,1+.2*i);
  check(!gps5hz.at(2.6).valid,"5Hz GPS cannot anchor with only nine fixes");
  fix(gps5hz,2.8);
  check(gps5hz.at(2.8).valid,"5Hz GPS anchors at ten fixes spanning 1.8 seconds");
  for(int i=0;i<=4;++i)fix(gps5hz,4+.2*i,.1);
  check(gps5hz.state=="recovery_verify","5Hz recovery still requires a full second");
  fix(gps5hz,5,.1);
  check(gps5hz.state=="recovery_blend","5Hz recovery starts blending after one second");
  for(int i=1;i<=5;++i)fix(gps5hz,5+.2*i,.1);
  check(gps5hz.state=="gps_anchored","5Hz recovery completes its one-second blend");
  fix(gps5hz,6.2,.5,0,1);
  for(int i=0;i<9;++i)fix(gps5hz,6.4+.2*i,.5);
  check(!gps5hz.at(8).valid,"5Hz stopped reanchor also requires ten fixes");
  fix(gps5hz,8.2,.5);
  check(gps5hz.at(8.2).valid,"5Hz stopped reanchor can complete");
  Anchor jitter;for(int i=0;i<30;++i)fix(jitter,1+.05*i,i%2?1:0);
  check(!jitter.at(3).valid,"inconsistent warmup cannot average a GPS jump away");
  Anchor timeout;warm(timeout,1);check(!timeout.at(123).valid,"DR hard time budget");
  Motion far;far.t=3;far.x=251;timeout.motion(far);check(!timeout.at(3).valid,"DR hard distance budget");
  InputAlignment align;Motion p;p.t=10;p.x=5;align.pose("local/a",p);
  Motion imu;imu.t=10;imu.pitch=.1;imu.roll=.05;align.imu(imu);
  check(align.paired(10)&&align.paired(10)->pitch==.1,"raw IMU attitude replaces yaw-only odometry tilt");
  check(!align.paired(10.01),"no extrapolated future pairing");
  p.t=10.1;p.x=6;align.pose("local/a",p);check(!align.paired(10.05),"late IMU waits for actual bracketing sample");
  imu.t=10.1;imu.pitch=.2;align.imu(imu);
  auto pair=align.paired(10.05);check(pair&&std::abs(pair->x-5.5)<1e-9&&std::abs(pair->pitch-.15)<1e-9,"bounded pose/attitude interpolation");
  check(!align.pose("local/a",p),"duplicate odometry not reintegrated");
  p.t=10.05;check(!align.pose("local/a",p)&&!align.paired(10.05),"clock regression invalidates alignment epoch");
  p.t=11;align.pose("local/b",p);check(!align.pose("local/a",p),"retired motion producer cannot reclaim stream");
  InputAlignment reordered;reordered.reorder_tolerance_s=.1;
  Motion rpose;rpose.t=10;reordered.pose("local/reorder",rpose);reordered.imu(rpose);
  const auto reorder_epoch=reordered.epoch;rpose.t=9.9994;
  check(!reordered.pose("local/reorder",rpose)&&!reordered.imu(rpose)&&
        reordered.epoch==reorder_epoch&&reordered.paired(10),
        "bounded pose and IMU reorder rejection preserves the last validated pairing");
  rpose.t=9.;reordered.imu(rpose);
  check(reordered.epoch>reorder_epoch&&!reordered.paired(10),
        "raw IMU clock restart still invalidates alignment");
  imu.t=11;imu.pitch=NAN;check(!align.imu(imu),"nonfinite attitude refused");
  SampledMotion gap;Motion g;g.t=1;gap.push(g);g.t=2;gap.push(g);
  check(gap.at(1)&&gap.at(2)&&!gap.at(1.5),"exact samples survive a large gap; interpolation does not");
  InputEpoch epoch;using R=InputEpoch::Result;
  check(epoch.accept("lidar-a",1,1,"c1","k1",10)==R::accepted,"first complete scan identity");
  check(epoch.accept("lidar-a",1,1,"c1","k1",10)==R::duplicate,"duplicate scan not transformed twice");
  check(epoch.accept("lidar-a",1,2,"c2","k1",10.1)==R::changed,"configuration change invalidates scan cache");
  check(epoch.accept("lidar-a",2,1,"c2","k2",10.2)==R::changed,"generation/calibration change resets boundary");
  check(epoch.accept("lidar-a",1,99,"c1","k1",10.3)==R::rejected,"old generation rejected");
  check(epoch.accept("lidar-b",1,1,"c2","k2",10.4)==R::changed,"LiDAR restart accepted as new epoch");
  check(epoch.accept("lidar-a",5,100,"c2","k2",10.5)==R::rejected,"retired LiDAR producer ignored");
  check(epoch.accept("lidar-b",1,2,"","k2",10.6)==R::rejected,"missing configuration identity refused");
  auto r=local_planning::AttitudeRotation(0,.2);auto once=local_planning::Rotate(r,2,0,1);
  check(std::abs(once[0]-(2*std::cos(.2)+std::sin(.2)))<1e-8,"OBB tilt applied once with known pitch geometry");
  return failures?1:0;
}
