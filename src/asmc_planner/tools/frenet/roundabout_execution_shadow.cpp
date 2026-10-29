// Offline PREPARE/READY audit. Uses production input conversion; never publishes or commits.
#include "Frenet/frenet_planner.hpp"
#include "Frenet/Maneuver/Merge/RoundaboutDeparture.hpp"
#include "Parser/PlanningInput.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
int main(int argc,char** argv) {
 if(argc<3){std::cerr<<"Usage: roundabout_execution_shadow frames.json params.yaml [empty] [--inspect-id N] [--inspect-from seconds]\n";return 1;}
 int inspect_id=-1;double inspect_from=-INFINITY;bool empty=false;
 for(int i=3;i<argc;++i) {
  const std::string argument=argv[i];
  if(argument=="empty")empty=true;
  else if(argument=="--inspect-id"&&i+1<argc)inspect_id=std::stoi(argv[++i]);
  else if(argument=="--inspect-from"&&i+1<argc)inspect_from=std::stod(argv[++i]);
  else {std::cerr<<"Unknown/incomplete argument: "<<argument<<"\n";return 1;}
 }
 auto frames=YAML::LoadFile(argv[1]);
 auto cfg=YAML::LoadFile(argv[2]);RoundaboutPrediction history,last18history;TrackedObject last18;double last18stamp=-1;
 const double clamp=cfg["planner"]["object_tracking"]["yaw_rate_clamp_rad_s"].as<double>(.6);
 const double speed_error=cfg["roundabout_merge"]["speed_measurement_error_mps"].as<double>(5);
 const double partial_length=cfg["roundabout_merge"]["partial_vehicle_length_m"].as<double>(6);
 const double partial_width=cfg["roundabout_merge"]["partial_vehicle_width_m"].as<double>(3);
 const double front_gap=cfg["roundabout_merge"]["min_front_time_gap_s"].as<double>(cfg["planner"]["merge"]["min_front_time_gap"].as<double>());
 const double rear_gap=cfg["planner"]["merge"]["min_rear_time_gap"].as<double>();
 FrenetPlanner planner;if(!planner.Init(argv[2]))return 2;
 std::cout<<std::setprecision(16);
 std::cout<<"ZONE stop="<<planner.roundabout_zone().stop_s<<" conflict="<<planner.roundabout_zone().conflict_start_s<<" clear="<<planner.roundabout_zone().rear_clear_s<<"\n";
 for(auto frame:frames) {
  const double t=frame["t"].as<double>();const auto v=frame["ego"];
  const double stamp=v["stamp"]?v["stamp"].as<double>():v["header"]["stamp"].as<double>();
  const double object_stamp=frame["object_stamp"]?frame["object_stamp"].as<double>():stamp-frame["object_stamp_age"].as<double>();
  asmc_msgs::VehicleState message;message.header.frame_id="map";message.header.stamp.fromSec(stamp);
  message.valid=v["valid"].as<bool>(true);message.x=v["x"].as<double>();message.y=v["y"].as<double>();
  message.yaw=v["yaw"].as<double>();message.speed=v["speed"].as<double>();message.acceleration=v["acceleration"].as<double>();message.yaw_rate=v["yaw_rate"].as<double>(0);
  CartesianState ego{};
  if(frenet_planner_ros::ConvertVehicleState(message,ego)!=frenet_planner_ros::ConversionStatus::OK) {
   planner.RevokeRoundaboutReadiness();std::cout<<"SKIP "<<t<<" vehicle_conversion\n";continue;
  }
  planning_inputs::DetectedObjectArray array;array.header.frame_id="map";array.header.stamp.fromSec(object_stamp);
  for(auto o:frame["objects"]) {
   planning_inputs::DetectedObject obj;obj.id=o["id"].as<int>();obj.type=o["type"].as<int>();obj.motion_state=o["motion_state"].as<int>();
   obj.velocity_unavailable=o["velocity_unavailable"].as<bool>(false);
   obj.classification=o["classification"].as<std::string>("");obj.camera_class=o["camera_class"].as<std::string>("");
   obj.center.x=o["center"]["x"].as<double>();obj.center.y=o["center"]["y"].as<double>();obj.center.z=o["center"]["z"].as<double>();obj.yaw=o["yaw"].as<double>();
   obj.vx=o["vx"].as<double>();obj.vy=o["vy"].as<double>();obj.yaw_rate=o["yaw_rate"].as<double>(0);
   obj.length=o["length"].as<double>();obj.width=o["width"].as<double>();obj.height=o["height"].as<double>();
   for(int i=0;i<8;++i){auto c=o["obb_corners"][i];obj.obb_corners[i].x=c["x"].as<double>();obj.obb_corners[i].y=c["y"].as<double>();obj.obb_corners[i].z=c["z"].as<double>();}
   array.objects.push_back(obj);
  }
  std::vector<TrackedObject> objects;
  if(frenet_planner_ros::ConvertDetectedObjects(array,objects)!=frenet_planner_ros::ConversionStatus::OK) {
   planner.RevokeRoundaboutReadiness();std::cout<<"SKIP "<<t<<" object_conversion\n";continue;
  }
  if(frame["detail"].as<std::string>("")=="ego/object input is not fresh") {
   planner.RevokeRoundaboutReadiness();std::cout<<"SKIP "<<t<<" recorded_stale_input\n";continue;
  }
  if(empty)objects.clear();

  if(frame["command"]&&frame["command"]["maneuver"].as<int>(2)!=2) {
   planner.ResetRoundaboutMerge();history.Reset();last18stamp=-1;continue;
  }
  PlannerCommand cmd;cmd.mode=MERGE;cmd.merge_type=MergeType::ROUNDABOUT;
  cmd.source_link_id="A2256W000148";cmd.target_link_id="A2256W000146";cmd.has_target_speed=true;cmd.target_speed=25./3.6;
  if(frame["command"]&&frame["command"]["has_desired_speed"].as<bool>(false))cmd.target_speed=frame["command"]["desired_speed"].as<double>();
  double s=0;planner.ProjectEgoToS(ego,s);
  if(s<planner.roundabout_zone().stop_s-45||s>planner.roundabout_zone().stop_s+.5)continue;
  history.Observe(objects,object_stamp,clamp,speed_error);
  const auto on_lane=[&](double x,double y){return planner.roundabout_zone().SupportsCirculatingObservation(x,y);};
  for(const auto& object:objects)if(object.id==inspect_id){last18=object;last18history=history;last18stamp=object_stamp;}
  auto nominal=objects;for(auto& object:nominal)object.yaw_rate=history.Rate(object.id);
  nominal=history.ApproachOccupancies(nominal);
  auto bounds=history.DepartureOccupancies(objects,on_lane,partial_length,partial_width);
  auto shaped=history.ApproachOccupancies(bounds);nominal.insert(nominal.end(),shaped.begin(),shaped.end());
  nominal=RoundaboutPrediction::ExpandTurnModels(history.WithObservedSpeedRanges(nominal),clamp);
  for(auto& model:nominal)model=AdvanceObjectObservation(model,std::max(0.0,stamp-object_stamp));
  auto retained=last18stamp>=0?last18history.DepartureOccupancies({last18},on_lane,partial_length,partial_width):std::vector<TrackedObject>{};
  retained=last18history.ApproachOccupancies(retained);
  retained=RoundaboutPrediction::ExpandTurnModels(last18history.WithObservedSpeedRanges(retained),clamp);
  for(auto& model:retained)model=AdvanceObjectObservation(model,std::max(0.0,stamp-last18stamp));
  const auto conflict=Interpolate(planner.reference(),planner.roundabout_zone().conflict_start_s);
  auto missing=history.MissingDepartureOccupancies(stamp,on_lane,partial_length,partial_width,clamp,
      conflict.x,conflict.y);
  if(inspect_id>=0&&t>=inspect_from)for(const auto& m:missing)if(m.id==inspect_id)
   std::cout<<"EVIDENCE "<<t<<" samples="<<m.observations<<" motion="<<m.motion_supported<<" body="<<m.body_supported<<" models="<<m.models.size()<<" age="<<m.age
    <<" reason="<<m.reason<<" relevance_models="<<m.relevance_models.size()<<"\n";
  CartesianPath path;FrenetPlanner::MergePrepareResult result;
  bool ok=planner.PlanRoundaboutMerge(ego,objects,cmd,stamp,object_stamp,path,result);
  if(inspect_id>=0&&t>=inspect_from&&!result.candidate_path.x.empty()) {
   std::size_t index=0;
   for(const auto& candidate:planner.last_candidate_paths()) {
    const auto current=CheckRoundaboutCandidate(candidate,planner.roundabout_zone(),nominal,
        planner.vehicle_shape(),planner.collision_config(),planner.sample_dt(),front_gap,rear_gap,true);
    const auto retained_check=CheckMissingRoundaboutDeparture(candidate,planner.roundabout_zone(),missing,
        planner.vehicle_shape(),planner.collision_config(),planner.sample_dt(),front_gap,rear_gap);
    std::cout<<"CANDIDATE "<<t<<" index="<<index++<<" current_safe="<<current.safe
     <<" current_blocker="<<current.blocking_object_id<<" current_reason="<<current.rejection_reason
     <<" missing_status="<<retained_check.status<<" missing_reason="<<retained_check.reason
     <<" missing_id="<<retained_check.blocking_id<<" age="<<retained_check.age
     <<" blocking_time="<<retained_check.time<<" gap_shift="<<retained_check.shift<<"\n";
   }
   auto a=CheckRoundaboutCandidate(result.candidate_path,planner.roundabout_zone(),nominal,planner.vehicle_shape(),planner.collision_config(),planner.sample_dt(),front_gap,rear_gap,true);
   nominal.insert(nominal.end(),retained.begin(),retained.end());
   auto b=CheckRoundaboutCandidate(result.candidate_path,planner.roundabout_zone(),nominal,planner.vehicle_shape(),planner.collision_config(),planner.sample_dt(),front_gap,rear_gap,true);
   auto physical=EvaluateCartesianTrajectorySafety(result.candidate_path,retained,planner.vehicle_shape(),planner.collision_config(),planner.sample_dt());
   std::cout<<"COMPARE "<<t<<" ready="<<result.ready<<" source= "<<last18stamp<<" models= "<<retained.size()<<" age= "<<stamp-last18stamp<<" current_safe="<<a.safe<<" retained_safe="<<b.safe<<" blocker="<<b.blocking_object_id<<" physical= "<<physical.clear<<" physical_sample="<<physical.blocking_sample_index<<"\n";
  }
  std::cout<<"AUDIT "<<t<<" s="<<s<<" v="<<ego.v<<" maintenance="<<ok<<" candidate="<<result.candidate_valid<<" ready="<<result.ready<<" valid="<<result.valid<<" entry="<<result.entry_time<<" exit="<<result.exit_time<<" blocker="<<result.blocking_object_id
   <<"; entry_reason="<<RoundaboutEntryStatusName(result.entry_status)<<"; retained="<<result.retained_missing_objects<<"; unverified="<<result.unverified_missing_objects<<"; blocker_age="<<result.blocking_history_age<<"; missing_status="<<result.missing_status
   <<"; missing_reason="<<result.missing_reason<<"; checked="<<result.missing_models_checked
   <<"; cleared="<<result.missing_non_interference<<"; blocking_time="<<result.blocking_path_time
   <<"; gap_shift="<<result.blocking_gap_shift<<"; certified="<<result.departure_certified<<"\n";
 }
}
