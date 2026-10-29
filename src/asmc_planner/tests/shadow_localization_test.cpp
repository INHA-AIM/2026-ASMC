#include "shadow_localization/static_planner.hpp"
#include "shadow_localization/sampled_motion.hpp"
#include "shadow_localization/paint_window.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <unistd.h>
using namespace local_planning;
int failures=0;
void check(bool value,const char* name){std::cout<<(value?"PASS ":"FAIL ")<<name<<'\n';if(!value)++failures;}
Scene scene(const Motion& m,bool obstacle=true){
  Scene s;s.speed=m.v;s.yaw_rate=m.w;s.acceleration=m.a;s.age=.05;
  s.grid.rows=250;s.grid.cols=160;s.grid.probability.assign(40000,255);s.grid.coverage.assign(40000,255);
  if(!obstacle)return s;
  Obstacle o;o.id=7;o.stamp=m.t;o.classification="unknown";
  auto& q=o.observation;q.id=o.id;q.width=1.;q.length=1.;q.height=1.;q.geometry_valid=q.z_valid=true;
  q.velocity_unavailable=true; // Real blackout observations have no absolute velocity.
  const double cs=std::cos(m.yaw),sn=std::sin(m.yaw);
  q.x=cs*(60-m.x)-sn*m.y;q.y=-sn*(60-m.x)-cs*m.y;q.heading=-m.yaw;
  for(int i=0;i<8;++i){double dx=60+(i%4==0||i%4==3?-.5:.5)-m.x,dy=(i%4<2?-.5:.5)-m.y;
    q.corners[i]={cs*dx+sn*dy,-sn*dx+cs*dy,double(i/4)};
    o.polygon.push_back({q.corners[i].x,q.corners[i].y});}
  o.polygon=geometry::ComputeConvexHull(o.polygon);s.obstacles.push_back(o);return s;
}
int main(int argc,char** argv){
  shadow_localization::PaintWindow paint;
  check(!paint.update(1,true,true,false),"unknown/ordinary road gets no paint exemption");
  check(paint.update(2,true,true,true),"known shadow zone grants scoped paint exemption");
  check(paint.update(3,true,true,false)&&paint.update(5.99,true,true,false),"return tail uses one exit epoch");
  check(!paint.update(6,true,true,false),"return tail expires at three seconds");
  check(paint.update(7,true,true,true)&&!paint.update(8,false,true,true),"NORMAL selection clears shadow permission");
  check(paint.update(9,true,true,true)&&!paint.update(8,true,true,true),"time regression clears paint epoch");
  check(!paint.update(10,true,false,true),"invalid localization cannot grant paint permission");
  shadow_localization::Anchor anchor;Motion m;m.x=2;m.y=3;m.yaw=.2;
  for(int i=0;i<=20;++i){m.t=1+.05*i;anchor.motion(m);anchor.observe(m,102,203,.2);}
  check(!anchor.at(.9).valid,"future anchor is unavailable");
  auto q=anchor.at(2);auto xy=q.apply(2,3);auto local=q.inverse(xy[0],xy[1]);
  check(q.valid&&std::hypot(xy[0]-102,xy[1]-203)<1e-9&&std::hypot(local[0]-2,local[1]-3)<1e-9,"warmed SE2 preserves map/local round trip");
  check(anchor.at(3).source=="dead_reckoning","GPS loss keeps explicit DR source");
  anchor.reset();check(!anchor.at(2).valid,"motion epoch removes anchor");

  auto dir=std::filesystem::temp_directory_path()/("asmc-localization-test-"+std::to_string(getpid()));
  std::filesystem::create_directories(dir);
  {std::ofstream f(dir/"waypoints.txt");for(int x=0;x<=200;++x)f<<x<<" 0\n";}
  {std::ofstream f(dir/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-4],[200,-4],[200,4],[0,4]]}]})";}
  {std::ofstream f(dir/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[200,5]]}}]})";}
  {std::ofstream f(dir/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int y=10;y<50;++y)f<<y<<" 0 1000\n";}
  if(argc>2){
    std::ofstream road(dir/"drivable.json");road<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-1.4],[200,-1.4],[200,5.4],[0,5.4]]}]})";
    std::ofstream grid(dir/"drivable_grid.rle");grid<<"3085 7232 0.2 0 -6\n";for(int y=23;y<57;++y)grid<<y<<" 0 1000\n";
  }
  auto params=YAML::LoadFile(std::string(SHADOW_SOURCE_DIR)+"/config/frenet/params.yaml");
  params["roundabout_merge"]["enabled"]=false;params["waypoint_file"]=(dir/"waypoints.txt").string();
  params["avoid_hd_map"]["directory"]=dir.string();
  {std::ofstream f(dir/"params.yaml");f<<params;}
  FrenetPlanner native_geometry;
  check(native_geometry.Init((dir/"params.yaml").string()),"native defaults initialize independently");
  const auto original_shape=native_geometry.vehicle_shape();
  auto smaller=original_shape;smaller.width-=.01;
  check(!native_geometry.SetConservativeVehicleEnvelope(smaller)&&
        native_geometry.vehicle_shape().width==original_shape.width,
        "research envelope cannot shrink or mutate production defaults");
  shadow_localization::StaticPlanner planner((dir/"params.yaml").string());
  Config cfg;cfg.speed_cap=argc>1?std::stod(argv[1])/3.6:2.;cfg.recovery=false;cfg.blind_assist=false;
  m={};m.t=1;m.x=30;m.v=argc>1?cfg.speed_cap:1.;
  check(!planner.plan(scene(m),cfg,m,{}).valid,"no GPS anchor cannot produce map trajectory");
  shadow_localization::Transform transform;transform.valid=true;transform.source="dead_reckoning";
  auto first=planner.plan(scene(m),cfg,m,transform);
  check(planner.phase()=="KEEP","one unknown observation is not confirmed static");
  auto bad=scene(m);bad.obstacles[0].observation.geometry_valid=false;
  check(!planner.plan(bad,cfg,m,transform).valid,"invalid authoritative geometry refuses frame");
  auto displaced=m;displaced.y=.25;
  auto rasterless=scene(displaced,false);rasterless.grid={};
  check(planner.reference(rasterless,cfg,displaced,transform).valid,
        "map and observed OBB route does not require a manufactured semantic raster");
  auto occupied=rasterless;auto barrier=scene(displaced).obstacles.front();
  barrier.polygon={{-1,-2},{4,-2},{4,2},{-1,2}};occupied.obstacles.push_back(barrier);
  check(!planner.reference(occupied,cfg,displaced,transform).valid,
        "missing raster never erases an observed physical obstruction");
  auto guide=planner.reference(scene(displaced,false),cfg,displaced,transform);
  check(guide.valid&&!guide.path.empty()&&std::abs(guide.path.front().y+.25)<.02,
        "fixed map reference preserves tracking error instead of snapping guide to ego");
  shadow_localization::SampledMotion samples;samples.push(m);auto later=m;later.t+=.1;later.x+=.1;samples.push(later);
  check(samples.at(m.t+.05)&&std::abs(samples.at(m.t+.05)->x-m.x-.05)<1e-9,
        "external motion interpolates recorded position without reintegration");
  auto dynamic=scene(m);dynamic.obstacles[0].observation.motion_state=ObjectMotionState::DYNAMIC;
  dynamic.obstacles[0].observation.x=10;
  check(planner.plan(dynamic,cfg,m,transform).reason=="shadow_dynamic_not_supported","moving actor is not accepted by static-only owner");
  bool valid=false;std::set<std::string> phases;double lateral=0,min_clearance=1e9,signed_lateral=0;
  for(int i=0;i<1500;++i){
    auto s=scene(m);auto result=planner.plan(s,cfg,m,transform);phases.insert(planner.phase());
    if(!result.valid||result.trajectory.size()<2){
      for(size_t k=0;k<std::min<size_t>(15,result.path.size());++k)std::cout<<"geometry "<<k<<" s="<<result.path[k].s<<" k="<<result.path[k].k<<" yaw="<<result.path[k].yaw<<"\n";
      std::cout<<"trial failure i="<<i<<" x="<<m.x<<" phase="<<planner.phase()<<" reason="<<result.reason<<" prefix="<<(result.path.empty()?0:result.path.back().s)<<" v="<<m.v<<" y="<<m.y<<" lasty="<<(result.path.empty()?0:result.path.back().y)<<'\n';for(auto& x:result.rejected)std::cout<<x.first<<":"<<x.second<<" ";std::cout<<" blocked="<<result.blocked_x<<","<<result.blocked_y<<"\n";break;
    }
    valid=true;min_clearance=std::min(min_clearance,result.minimum_clearance);
    const auto& next=result.trajectory[1];double cs=std::cos(m.yaw),sn=std::sin(m.yaw);
    m.x+=cs*next[0]-sn*next[1];m.y+=sn*next[0]+cs*next[1];m.yaw+=next[2];m.w=next[2]/.05;m.v=next[4];m.a=next[5];m.t+=.05;
    lateral=std::max(lateral,std::abs(m.y));signed_lateral=std::abs(m.y)>std::abs(signed_lateral)?m.y:signed_lateral;
    if(m.x>80&&std::abs(m.y)<.25&&planner.phase()=="KEEP")break;
    if(m.x>110)break;
  }
  check(valid&&lateral>1.,"native static bypass causes measured lateral avoidance");
  check(phases.count("PASS")&&phases.count("RETURN")&&m.x>75&&std::abs(m.y)<.25,"static bypass passes and returns under ideal tracking");
  if(argc>2)check(signed_lateral>1.,"map boundary rejects right departure and selects left bypass");
  check(min_clearance>=cfg.margin,"published prefixes preserve local physical clearance");
  shadow_localization::StaticPlanner partial((dir/"params.yaml").string());
  shadow_localization::StaticPlanner moving_unknown((dir/"params.yaml").string());
  Motion observed{};observed.x=30.;observed.t=10.;
  for(int i=0;i<9;++i){
    observed.t=10.+.1*i;auto faces=scene(observed);auto& ob=faces.obstacles.front();
    if(i%2==0){
      ob.observation.x=59.55-observed.x;ob.observation.length=.1;
      for(auto& c:ob.observation.corners)c.x=c.x<30.?29.5:29.6;
      ob.polygon={{29.5,-.5},{29.6,-.5},{29.6,.5},{29.5,.5}};
    }
    partial.plan(faces,cfg,observed,transform);
    auto moving=scene(observed);auto& mo=moving.obstacles.front();
    mo.observation.x+=.1*i;
    for(auto& c:mo.observation.corners)c.x+=.1*i;
    for(auto& p:mo.polygon)p.x+=.1*i;
    moving_unknown.plan(moving,cfg,observed,transform);
  }
  check(partial.confirmed_rest()==1,
        "partial faces of one fixed observed volume retain positive rest evidence");
  check(moving_unknown.confirmed_rest()==0,
        "unknown velocity with uniformly translating observed support is never confirmed static");
  shadow_localization::StaticPlanner flagged_rest((dir/"params.yaml").string());
  shadow_localization::StaticPlanner flagged_moving((dir/"params.yaml").string());
  for(int i=0;i<9;++i){
    observed.t=20.+.1*i;auto fixed=scene(observed);
    fixed.obstacles.front().observation.motion_state=ObjectMotionState::DYNAMIC;
    fixed.obstacles.front().observation.velocity_unavailable=false;
    fixed.obstacles.front().observation.vx=2.;
    flagged_rest.plan(fixed,cfg,observed,transform);
    check(fixed.obstacles.front().observation.vx==2.&&
          !fixed.obstacles.front().observation.velocity_unavailable,
          "derived rest reasoning preserves the original conflicting motion estimate");
    auto shifted=fixed;auto& o=shifted.obstacles.front();o.observation.x+=.1*i;
    for(auto& c:o.observation.corners)c.x+=.1*i;
    for(auto& p:o.polygon)p.x+=.1*i;
    flagged_moving.plan(shifted,cfg,observed,transform);
  }
  check(flagged_rest.confirmed_rest()==1&&flagged_rest.rest_motion_disagreements()==1,
        "stable measured hull support can qualify rest despite a flickering producer dynamic flag");
  check(flagged_moving.confirmed_rest()==0,
        "the same semantic/motion flags never qualify a translating hull as rest");
  shadow_localization::StaticPlanner person((dir/"params.yaml").string());person.enable_dynamic(true);
  for(int i=0;i<16;++i){observed.t=30.+.1*i;auto fixed=scene(observed);fixed.obstacles.front().camera_class="person";person.plan(fixed,cfg,observed,transform);}
  check(person.confirmed_rest()==1,"stationary person qualifies for bypass from fresh geometric evidence");
  for(int i=0;i<8;++i){observed.t=31.6+.1*i;auto crossing=scene(observed);auto& ob=crossing.obstacles.front();ob.camera_class="person";
    for(auto& q:ob.polygon)q.y+=.1*(i+1);for(auto& q:ob.observation.corners)q.y+=.1*(i+1);ob.observation.y+=.1*(i+1);
    person.plan(crossing,cfg,observed,transform);}
  check(person.confirmed_rest()==0,"restarted pedestrian motion revokes stationary bypass evidence");
  shadow_localization::StaticPlanner waiting((dir/"params.yaml").string());
  Motion waiting_origin{};waiting_origin.x=30.;waiting_origin.t=1.;
  Result approach;Config waiting_cfg=cfg;waiting_cfg.speed_cap=3./3.6;
  for(int i=0;i<12;++i){
    waiting_origin.t+=.1;
    auto barrier_scene=scene(waiting_origin);
    auto& barrier=barrier_scene.obstacles.front();barrier.observation.width=8.;
    for(auto& c:barrier.observation.corners)c.y=c.y<0?-4.:4.;
    barrier.polygon={{29.5,-4},{30.5,-4},{30.5,4},{29.5,4}};
    approach=waiting.plan(barrier_scene,waiting_cfg,waiting_origin,transform);
  }
  std::cout<<"WAITING_APPROACH valid="<<approach.valid<<" phase="<<waiting.phase()<<" reason="<<approach.reason<<" cap="<<approach.chosen_cap<<" prefix="<<(approach.path.empty()?0:approach.path.back().s)<<"\n";
  check(approach.valid&&approach.chosen_cap>0&&approach.chosen_cap<=3./3.6&&
        approach.path.back().s<27.,
        "blocked native bypass can approach only on verified stopping space at crawl");
  waiting_cfg.speed_cap=15./3.6;waiting_origin.t+=.1;
  auto adaptive_scene=scene(waiting_origin);
  auto& adaptive_barrier=adaptive_scene.obstacles.front();adaptive_barrier.observation.width=8.;
  for(auto& c:adaptive_barrier.observation.corners)c.y=c.y<0?-4.:4.;
  adaptive_barrier.polygon={{29.5,-4},{30.5,-4},{30.5,4},{29.5,4}};
  auto adaptive=waiting.plan(adaptive_scene,waiting_cfg,waiting_origin,transform);
  std::cout<<"ADAPTIVE_APPROACH valid="<<adaptive.valid<<" phase="<<waiting.phase()
           <<" cap="<<adaptive.chosen_cap<<" reason="<<adaptive.reason
           <<" terminal_v="<<(adaptive.trajectory.empty()?-1:adaptive.trajectory.back()[4])<<'\n';
  check(adaptive.valid&&(waiting.phase()=="WAITING_REFERENCE"||waiting.phase()=="PREPARE")&&
        adaptive.chosen_cap>3./3.6&&adaptive.chosen_cap<=15./3.6&&
        adaptive.minimum_clearance>=waiting_cfg.margin&&
        !adaptive.trajectory.empty()&&adaptive.trajectory.back()[4]<.01,
        "verified PREPARE space can retain requested speed with a complete stopping profile");
  auto blocked=scene(m,false);blocked.grid.probability.assign(40000,0);
  check(!planner.plan(blocked,cfg,m,transform).valid,"map road cannot override observed nonroad");
  {
    shadow_localization::StaticPlanner guarded((dir/"params.yaml").string());
    guarded.enable_path_continuity(true);
    Motion current{};current.x=30;current.v=cfg.speed_cap;current.t=1.;Result primed;
    for(int i=0;i<100;++i){current.t+=.05;primed=guarded.plan(scene(current),cfg,current,transform);
      if(guarded.maneuver_active()&&primed.valid&&primed.chosen_cap>.1)break;}
    check(guarded.maneuver_active()&&primed.valid&&primed.chosen_cap>.1,
          "continuity test primes an actual committed verified bypass");
    current.t+=.05;auto unsafe=scene(current);auto intrusion=unsafe.obstacles.front();
    intrusion.id=intrusion.observation.id=99;intrusion.observation.x=1.;intrusion.observation.y=0.;
    intrusion.observation.width=8.;intrusion.observation.length=2.;
    for(size_t i=0;i<8;++i)intrusion.observation.corners[i]={i%2?2.:0.,i%4<2?-4.:4.,double(i/4)};
    intrusion.polygon={{0,-4},{2,-4},{2,4},{0,4}};unsafe.obstacles.push_back(intrusion);
    const auto stopped=guarded.plan(unsafe,cfg,current,transform);
    check(!stopped.valid&&!guarded.reused_path(),
          "previous bypass never overrides a newly observed body obstruction");
    guarded.reset_reference();
    check(!guarded.maneuver_active(),"epoch reset retires the committed maneuver and cached path together");
  }
  std::cout<<"IDEAL_TRACKING max_lateral="<<lateral<<" clearance="<<min_clearance<<" end_x="<<m.x<<" end_y="<<m.y<<" phases=";for(const auto& phase:phases)std::cout<<phase<<",";std::cout<<"\n";
  std::filesystem::remove_all(dir);
  return failures?1:0;
}
