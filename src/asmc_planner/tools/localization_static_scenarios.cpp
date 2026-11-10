// ROS/bridge-free bounded kinematic diagnostic. This is neither a physical trial
// nor a controller performance claim; every case reports its failure too.
#include "shadow_localization/static_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <deque>
#include <unistd.h>
using namespace local_planning;
struct Box {double x,y,vx=0,vy=0,width=1,length=1,start=0;std::string label="unknown";};
std::array<double,2> position(const Box& b,double time){const double t=std::max(0.,time-1-b.start);return {b.x+b.vx*t,b.y+b.vy*t};}
Scene observe(const Motion& m,const std::vector<Box>& boxes,double lag=0){
  Scene s;s.speed=m.v;s.acceleration=m.a;s.age=.05;
  const double c=std::cos(m.yaw),sn=std::sin(m.yaw);
  for(size_t id=0;id<boxes.size();++id){
    const auto& b=boxes[id];const double stamp=std::max(1.,m.t-lag);const auto at=position(b,stamp);
    Obstacle o;o.id=int(id)+1;o.stamp=stamp;o.classification=b.label;o.camera_class=b.label;
    auto& q=o.observation;q.id=o.id;q.width=b.width;q.length=b.length;q.height=1;
    q.camera_class=b.label;q.classification=b.label;
    q.geometry_valid=q.z_valid=true;q.velocity_unavailable=true;
    const double dx=at[0]-m.x,dy=at[1]-m.y;
    q.x=c*dx+sn*dy;q.y=-sn*dx+c*dy;q.heading=-m.yaw;
    for(int i=0;i<8;++i){
      const double x=dx+(i%4==0||i%4==3?-.5:.5)*b.length,y=dy+(i%4<2?-.5:.5)*b.width;
      q.corners[i]={c*x+sn*y,-sn*x+c*y,double(i/4)};
      o.polygon.push_back({q.corners[i].x,q.corners[i].y});
    }
    o.polygon=geometry::ComputeConvexHull(o.polygon);s.obstacles.push_back(o);
  }return s;
}
int main(int argc,char** argv){
  const double speed=argc>1?std::stod(argv[1]):15.;
  bool strict=false,baseline=false,dynamics=false,calibrated=false,baseline_motion=false,reduced_growth=false,rotated=false;
  std::string only_case;
  for(int i=2;i<argc;++i){const std::string flag=argv[i];
    if(flag=="--require-completion")strict=true;
    else if(flag=="--baseline-target-policy")baseline=true;
    else if(flag=="--dynamic-cases")dynamics=true;
    else if(flag=="--calibrated-plant")calibrated=true;
    else if(flag=="--baseline-motion-policy")baseline_motion=true;
    else if(flag=="--reduced-growth")reduced_growth=true;
    else if(flag=="--rotated-anchor")rotated=true;
    else if(flag.rfind("--case=",0)==0)only_case=flag.substr(7);
    else return 2;}
  if(!std::isfinite(speed)||speed<=0||speed>25)return 2;
  bool all_completed=true;
  const auto dir=std::filesystem::temp_directory_path()/
      ("asmc-localization-static-scenarios-"+std::to_string(getpid()));
  std::filesystem::create_directories(dir);
  {std::ofstream f(dir/"path.txt");for(int i=0;i<=200;++i)f<<i<<" 0\n";}
  {std::ofstream f(dir/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":0,"origin_y":-6},"drivable_areas":[{"validity":"validated","polygon":[[0,-4],[200,-4],[200,4],[0,4]]}]})";}
  {std::ofstream f(dir/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,5],[200,5]]}}]})";}
  {std::ofstream f(dir/"drivable_grid.rle");f<<"3085 7232 0.2 0 -6\n";for(int y=10;y<50;++y)f<<y<<" 0 1000\n";}
  auto params=YAML::LoadFile(std::string(SHADOW_SOURCE_DIR)+"/config/frenet/params.yaml");
  params["roundabout_merge"]["enabled"]=false;params["waypoint_file"]=(dir/"path.txt").string();
  params["avoid_hd_map"]["directory"]=dir.string();
  // Sensitivity experiment only: the production YAML and live node are not
  // changed. Keep the physical footprint and 0.45m base margin unchanged.
  if(reduced_growth)params["planner"]["collision_check"]["margin_growth_rate"]=.05;
  {std::ofstream f(dir/"params.yaml");f<<params;}
  std::vector<std::pair<std::string,std::vector<Box>>> cases={
      {"center",{{60,0}}},{"left",{{60,.75}}},{"right",{{60,-.75}}},
      {"staggered",{{60,.75},{80,-.75}}}};
  if(dynamics){
    cases.push_back({"cross_left",{{55,-4,0,1.2,.6,.6,0,"person"}}});
    cases.push_back({"cross_right",{{55,4,0,-1.2,.6,.6,0,"person"}}});
    cases.push_back({"late_cross",{{50,4,0,-.8,.6,.6,0,"person"}}});
    cases.push_back({"walking_away",{{55,0,.8,0,.6,.6,0,"person"}}});
    cases.push_back({"standing_person",{{55,0,0,0,.6,.6,0,"person"}}});
    cases.push_back({"slow_lead",{{55,0,.6,0,1.8,4.5,0,"vehicle"}}});
    cases.push_back({"oncoming_other_lane",{{75,-3,-3,0,1.8,4.5,0,"vehicle"}}});
    cases.push_back({"mixed",{{60,.75},{75,-4,0,.8,.6,.6,0,"person"}}});
    cases.push_back({"stop_go_cross",{{55,0,0,1.2,.6,.6,3,"person"}}});
  }
  if(!only_case.empty()){
    cases.erase(std::remove_if(cases.begin(),cases.end(),[&](const auto& c){return c.first!=only_case;}),cases.end());
    if(cases.empty())return 2;
  }
  for(const auto& testcase:cases){
    shadow_localization::StaticPlanner planner((dir/"params.yaml").string(),!baseline);
    planner.enable_dynamic(dynamics&&!baseline_motion);
    planner.set_paint_exemption(dynamics&&!baseline_motion);
    Config cfg;cfg.speed_cap=speed/3.6;cfg.recovery=false;cfg.blind_assist=false;
    shadow_localization::Transform transform;transform.valid=true;transform.source="dead_reckoning";
    if(rotated){transform.yaw=.37;transform.x=3.;transform.y=-2.;}
    Motion m;m.x=30;m.v=cfg.speed_cap;m.t=1;
    double steering=0.,max_steer=0.,max_lateral_accel=0.;std::deque<double> commands(3,0.);
    const bool mobile=std::any_of(testcase.second.begin(),testcase.second.end(),[](const auto& b){return b.label!="unknown"||b.vx||b.vy;});
    std::set<std::string> phases;double clearance=1e9,actual_clearance=1e9,lateral=0,stopped=0,max_stopped=0;
    std::string outcome="step_limit",reason="";int steps=0,invalid_updates=0;unsigned max_native_calls=0,max_profiles=0;
    for(;steps<1800;++steps){
      Motion odom=m;const auto local=transform.inverse(m.x,m.y);
      odom.x=local[0];odom.y=local[1];odom.yaw-=transform.yaw;
      auto result=planner.plan(observe(m,testcase.second,mobile?.05:0.),cfg,odom,transform);
      phases.insert(planner.phase());max_native_calls=std::max(max_native_calls,planner.native_calls());max_profiles=std::max(max_profiles,planner.speed_profiles());
      double next_v=0,next_a=0;
      if(!result.valid||result.trajectory.size()<2||result.path.empty()){
        ++invalid_updates;reason=result.reason;
        // An invalid decision means braking with held steering, not a new
        // accepted trajectory. Continue observing/replanning as a live plant
        // does, with every invalid update counted and physical checks active.
        next_a=std::max(-cfg.decel,m.a-cfg.jerk*.05);
        next_v=std::max(0.,m.v+(m.a+next_a)*.025);
      }else{
        clearance=std::min(clearance,result.minimum_clearance);
        const auto& next=result.trajectory[1];next_v=next[4];next_a=next[5];
        const double lookahead=std::max(2.,.8*m.v);const Pose* target=&result.path.back();
        for(const auto& point:result.path)if(std::hypot(point.x,point.y)>=lookahead){target=&point;break;}
        const double radius2=target->x*target->x+target->y*target->y;
        const double prediction=calibrated?std::max(m.v,next_v)+std::max(0.,next_a)*.15:m.v;
        const double curvature_limit=std::min(cfg.max_k,cfg.lateral_accel/std::max(.1,prediction*prediction));
        const double desired_curvature=std::clamp(radius2>1e-6?2*target->y/radius2:0.,-curvature_limit,curvature_limit);
        const double desired_steer=std::atan(cfg.wheelbase*desired_curvature/(calibrated?cfg.gain:1.));
        steering+=std::clamp(desired_steer-steering,-cfg.max_steer_rate*.05,cfg.max_steer_rate*.05);
      }
      commands.push_back(steering);const double actual_steer=calibrated?commands.front():steering;commands.pop_front();
      max_steer=std::max(max_steer,std::abs(actual_steer));
      const double actual_curvature=(calibrated?cfg.gain:1.)*std::tan(actual_steer)/cfg.wheelbase;
      max_lateral_accel=std::max(max_lateral_accel,std::pow(std::max(m.v,next_v),2)*std::abs(actual_curvature));
      const double distance=(m.v+next_v)*.025,yaw_step=distance*actual_curvature;
      m.x+=distance*std::cos(m.yaw+.5*yaw_step);m.y+=distance*std::sin(m.yaw+.5*yaw_step);
      m.yaw+=yaw_step;m.v=next_v;m.a=next_a;m.t+=.05;
      const auto body=MakeEgoFootprint(CartesianState{m.x,m.y,m.yaw,0,m.v,m.a},
          VehicleShape{cfg.width,cfg.length,cfg.rear},0.);
      if(std::any_of(body.begin(),body.end(),[](const auto& p){return std::abs(p.y)>4.||p.x<0||p.x>200;})){
        outcome="physical_map_departure";break;
      }
      for(const auto& box:testcase.second){
        const auto at=position(box,m.t);const double x=at[0],y=at[1],hx=box.length/2,hy=box.width/2;
        const std::vector<geometry::Point2D> polygon={{x-hx,y-hy},{x+hx,y-hy},{x+hx,y+hy},{x-hx,y+hy}};
        actual_clearance=std::min(actual_clearance,geometry::ConvexPolygonDistance(body,polygon));
      }
      if(actual_clearance<=cfg.margin){outcome="physical_margin_violation";break;}
      if(steps%5==0)std::cout<<"SYNTHETIC_POSE "<<testcase.first<<' '<<m.t<<' '<<m.x<<' '<<m.y<<' '<<m.yaw<<' '<<m.v<<'\n';
      lateral=std::max(lateral,std::abs(m.y));stopped=m.v<.05?stopped+.05:0.;max_stopped=std::max(max_stopped,stopped);
      if(calibrated||dynamics)std::cout<<"SYNTHETIC_TICK {\"case\":\""<<testcase.first
        <<"\",\"t\":"<<m.t<<",\"x\":"<<m.x<<",\"y\":"<<m.y<<",\"yaw\":"<<m.yaw
        <<",\"v\":"<<m.v<<",\"a\":"<<m.a<<",\"steering\":"<<actual_steer
        <<",\"plan_valid\":"<<(result.valid?"true":"false")<<",\"reason\":\""<<result.reason
        <<"\",\"phase\":\""<<planner.phase()<<"\",\"moving\":"<<planner.moving_objects()
        <<",\"pending\":"<<planner.pending_motion_objects()<<",\"native_calls\":"<<planner.native_calls()
        <<",\"profiles\":"<<planner.speed_profiles()<<",\"plan_ms\":"<<result.elapsed_ms
        <<",\"horizon_m\":"<<(result.path.empty()?0:result.path.back().s)
        <<",\"horizon_s\":"<<(result.trajectory.empty()?0:result.trajectory.back()[6])
        <<",\"transition_m\":"<<planner.transition_distance()<<",\"return_m\":"<<planner.return_distance()
        <<",\"candidates\":"<<planner.generated()<<"}\n";
      const double frontier=std::max_element(testcase.second.begin(),testcase.second.end(),[](const auto& a,const auto& b){return a.x<b.x;})->x+20;
      const bool passed=std::all_of(testcase.second.begin(),testcase.second.end(),[&](const auto& box){return m.x-position(box,m.t)[0]>cfg.rear+box.length/2+2.;});
      if(m.x>frontier&&passed&&std::abs(m.y)<.25&&!planner.maneuver_active()){
        outcome="completed";break;
      }
      if(stopped>=(mobile?8.:3.)){outcome=mobile?"stopped_8s":"stopped_3s";break;}
    }
    const bool accepted=outcome=="completed"&&actual_clearance>cfg.margin&&
        (mobile||(phases.count("PASS")&&phases.count("RETURN")))&&max_native_calls<=1&&
        double(invalid_updates)/(steps+1)<=.05&&
        (!calibrated||(max_steer<=40*M_PI/180&&max_lateral_accel<=cfg.lateral_accel+.05))&&
        (testcase.first=="staggered"&&!baseline?planner.retargets()>0:planner.retargets()==0);
    std::cout<<"SYNTHETIC_CASE {\"case\":\""<<testcase.first<<"\",\"outcome\":\""<<outcome
      <<"\",\"accepted_model\":"<<(accepted?"true":"false")
      <<",\"reason\":\""<<reason<<"\",\"requested_kph\":"<<speed
      <<",\"plant\":\""<<(calibrated?"gain_0.844_delay_0.15s_bounded_pursuit":"legacy_bounded_pursuit")<<"\""
      <<",\"dynamic_policy\":"<<(dynamics&&!baseline_motion?"true":"false")
      <<",\"margin_growth_mps\":"<<params["planner"]["collision_check"]["margin_growth_rate"].as<double>()
      <<",\"anchor_yaw_rad\":"<<transform.yaw
      <<",\"max_steer_deg\":"<<max_steer*180/M_PI<<",\"max_lateral_accel_mps2\":"<<max_lateral_accel
      <<",\"max_profiles_per_update\":"<<max_profiles
      <<",\"steps\":"<<steps<<",\"end_x_m\":"<<m.x<<",\"end_y_m\":"<<m.y
      <<",\"minimum_probe_clearance_m\":"<<clearance<<",\"actual_minimum_clearance_m\":"<<actual_clearance<<",\"max_lateral_m\":"<<lateral
      <<",\"max_stopped_s\":"<<max_stopped<<",\"max_native_calls_per_update\":"<<max_native_calls
      <<",\"passed_target_replacements\":"<<planner.retargets()
      <<",\"invalid_updates\":"<<invalid_updates<<",\"valid_update_ratio\":"<<1.-double(invalid_updates)/(steps+1)
      <<",\"sequential_targets\":"<<(!baseline?"true":"false")
      <<",\"phases\":[";bool first=true;for(const auto& phase:phases){if(!first)std::cout<<',';first=false;std::cout<<'"'<<phase<<'"';}
    std::cout<<"]}"<<std::endl;
    all_completed=all_completed&&accepted;
  }
  std::filesystem::remove_all(dir);
  return strict&&!all_completed?1:0; // Diagnostic mode still preserves failed cases.
}
