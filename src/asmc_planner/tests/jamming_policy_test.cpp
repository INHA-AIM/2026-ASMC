#include "jamming_common/speed_profile.hpp"
#include "jamming_common/geometry.hpp"
#include "jamming_common/stall_escape.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace local_planning;
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
Result straight(){Result r;for(double x=0;x<=80.;x+=.25)r.path.push_back({x,0,0,0,x});return r;}
int main(){
  Config c;Scene s;
  StallEscapeWatch watch;bool requested=false;
  for(int i=0;i<=102;++i)requested=watch.update("epoch1",i*.1,i*.1,Pose{},0.,false);
  require(requested&&watch.windows()==2,"two5s windows escalate a continuous stop");
  require(!watch.update("epoch1",10.3,10.3,Pose{},0.,true),"moving conflict keeps yielding and retires escalation");
  for(int i=104;i<=208;++i)requested=watch.update("epoch1",i*.1,i*.1,Pose{},0.,false);
  require(requested,"fresh observations can re-arm after a dynamic actor clears");
  require(!watch.update("epoch2",21.,21.,Pose{},0.,false),"new producer epoch cannot inherit a stop latch");
  for(int i=211;i<260;++i)watch.update("epoch2",i*.1,i*.1,Pose{},0.,false);
  require(!watch.update("epoch2",27.,27.,Pose{},0.,false),"a stale source gap cannot trigger forced escape");
  watch.reset();
  for(int i=0;i<=60;++i)watch.update("episodes",i*.1,i*.1,Pose{},0.,false);
  for(int i=61;i<=70;++i)watch.update("episodes",i*.1,i*.1,Pose{(i-60)*.05,0,0},.5,false);
  for(int i=71;i<=125;++i)requested=watch.update("episodes",i*.1,i*.1,Pose{.5,0,0},0.,false);
  require(requested,"two separate five-second stops survive a short unsuccessful move");
  require(c.width==1.892,"shared physical body model");
  auto r=straight();require(BuildSpeedProfile(s,c,r)&&std::abs(r.chosen_cap-15./3.6)<1e-8,"15 cruise under20 ceiling");
  c.speed_cap=10./3.6;r=straight();require(BuildSpeedProfile(s,c,r)&&std::abs(r.chosen_cap-10./3.6)<1e-8,"explicit comparison ceiling wins over cruise");
  c.speed_cap=20./3.6;r=straight();r.recovery=true;
  require(BuildSpeedProfile(s,c,r)&&std::abs(r.chosen_cap-5./3.6)<1e-8,"shared escape ceiling, no fixed3kph target");
  s.speed=25./3.6;s.acceleration=.2;r=straight();
  require(BuildSpeedProfile(s,c,r)&&std::abs(r.trajectory.front()[4]-s.speed)<1e-8,"handoff retains measured25kph rather than clipping speed");
  for(size_t i=1;i<r.trajectory.size();++i)
    require(std::abs(r.trajectory[i][4]-r.trajectory[i-1][4])<.3,"continuous braking reference");
  r=straight();r.path.resize(2);r.path.back().s=.25;r.recovery=true;
  require(!BuildSpeedProfile(s,c,r),"recovery cannot bypass actual stopping space");
  s={};s.grid.rows=400;s.grid.cols=200;s.grid.xmax=40.;s.grid.ymax=10.;
  s.grid.probability.assign(80000,255);s.grid.coverage.assign(80000,255);BuildGridIndex(s.grid);
  c.margin=.30;Obstacle o;o.id=3;o.polygon={{4.,1.35},{8.,1.35},{8.,1.45},{4.,1.45}};s.obstacles.push_back(o);
  r={};require(CheckFootprint(s,c,Pose{5,0,0,0,0},r),"1.892m body with retained30cm margin fits measured gap");
  c.width=2.160835;r={};require(!CheckFootprint(s,c,Pose{5,0,0,0,0},r),"fixture distinguishes previous SDK envelope");
  c=Config{};c.budget_ms=1000.;s={};
  r=FindStallEscape(s,c,[](const Pose&,const Pose& p,Result&){return p.s<2. || p.y<-.05;});
  require(r.valid&&r.path.back().y<-.05&&r.chosen_cap>0.&&r.chosen_cap<=5./3.6,"blocked straight path selects a checked lateral escape");
  r=FindStallEscape(s,c,[](const Pose&,const Pose&,Result&){return false;});
  require(!r.valid&&r.trajectory.empty(),"closed space is not invented as drivable");
  Obstacle moving;moving.motion_sweep={{2.,-1.},{5.,-1.},{5.,1.},{2.,1.}};s.obstacles.push_back(moving);
  r=FindStallEscape(s,c,[](const Pose&,const Pose&,Result&){return true;});
  require(!r.valid&&r.reason=="stall_escape_dynamic_yield","forward moving sweep vetoes escape even when a static checker permits it");
  s={};s.grid.rows=400;s.grid.cols=200;s.grid.xmax=20.;s.grid.ymax=10.;
  s.grid.probability.assign(80000,255);s.grid.coverage.assign(80000,255);BuildGridIndex(s.grid);
  Obstacle fixed;fixed.id=6;fixed.polygon={{8.,-.6},{10.,-.6},{10.,.6},{8.,.6}};s.obstacles.push_back(fixed);
  r=FindStallEscape(s,c,[&](const Pose& a,const Pose& b,Result& out){return CheckSweep(s,c,a,b,out);});
  require(r.valid&&r.path.back().s>6.&&std::abs(r.path.back().y)>.5,"real raster and OBB sweep find a side route around a stopped object");
  std::cout<<"PASS shared speed, transition, stopping and vehicle-margin contracts\n";
}
