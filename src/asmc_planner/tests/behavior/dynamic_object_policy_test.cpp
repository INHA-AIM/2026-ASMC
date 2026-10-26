#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "Frenet/frenet_planner.hpp"
#include "Behavior/Feature/DynamicObjectPolicy.hpp"
#include "Behavior/Feature/feature_builder.hpp"
#include "Behavior/Decision/behavior_decision.hpp"
#include "Execution/IntentEncoder.hpp"
using namespace behavior_planner;
TEST(FollowBoundaryRegression, FreshFasterLeadCanReleaseResidualBrakeAndRestart) {
  const std::filesystem::path root=ASMC_POLICY_SOURCE;
  auto cfg=YAML::LoadFile((root/"config/frenet/params.yaml").string());
  const auto waypoint=(root/"../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt").lexically_normal();
  cfg["waypoint_file"]=waypoint.string();cfg["avoid_hd_map"]["directory"]=(root/"config/hd_map").string();
  cfg["planner"]["visualization"]["enabled"]=false;
  const auto file=std::filesystem::temp_directory_path()/"asmc_follow_brake_restart_test.yaml";
  {std::ofstream out(file);out<<cfg;}
  std::ifstream input(waypoint);double x=0,y=0,z=0,s=0,px=0,py=0;bool first=true;
  CartesianState base;
  while(input>>x>>y>>z) {
    if(!first)s+=std::hypot(x-px,y-py);
    if(s>=448.){base.x=x;base.y=y;base.yaw=std::atan2(y-py,x-px);break;}
    px=x;py=y;first=false;
  }
  ASSERT_GE(s,448.);
  const double c=std::cos(base.yaw),sn=std::sin(base.yaw);
  auto make_lead=[&](double gap,double speed) {
    ::TrackedObject o;o.id=77;o.type=1;o.motion_state=ObjectMotionState::DYNAMIC;
    o.x=base.x+gap*c;o.y=base.y+gap*sn;o.heading=base.yaw;
    o.vx=speed*c;o.vy=speed*sn;o.speed=speed;o.width=1.9;o.length=4.5;o.height=1.8;
    o.geometry_valid=o.z_valid=true;
    for(int i=0;i<8;++i){const double dx=(i%4==0||i%4==3)?-2.25:2.25;
      const double dy=i%4<2?-.95:.95;
      o.corners[i]={o.x+c*dx-sn*dy,o.y+sn*dx+c*dy,double(i/4)*1.8};}
    return o;
  };
  PlannerCommand command;command.mode=FOLLOWING;command.target_object_id=77;
  command.has_target_speed=true;command.target_speed=25./3.6;
  // Captured standstill retained -0.637 m/s2; the old quintic immediately
  // reverses and every longitudinal candidate is rejected. A moving case
  // covers the repeated braking-memory collapse before reaching standstill.
  for(const auto state:{std::pair<double,double>{0.,-.637},{2.,-2.4}}) {
    FrenetPlanner planner;ASSERT_TRUE(planner.Init(file.string()));
    auto ego=base;ego.v=state.first;ego.a=state.second;
    planner.SetObjectObservationTime(1.,1.);
    CartesianPath path;ASSERT_TRUE(planner.Plan(ego,{make_lead(30.,4.)},command,path));
    ASSERT_GT(path.v.size(),1u);EXPECT_GE(path.v[1],ego.v-1e-6);
    EXPECT_TRUE(EvaluateCartesianTrajectorySafety(path,{make_lead(30.,4.)},
        planner.vehicle_shape(),planner.collision_config(),planner.sample_dt()).clear);
  }
  // A slower/stopped lead must still retain braking, or reject movement.
  for(double leader_speed:{0.,1.}) {
    FrenetPlanner planner;ASSERT_TRUE(planner.Init(file.string()));
    auto ego=base;ego.v=4.;ego.a=-2.4;planner.SetObjectObservationTime(1.,1.);
    CartesianPath path;const bool ok=planner.Plan(ego,{make_lead(10.,leader_speed)},command,path);
    if(ok){ASSERT_GT(path.v.size(),1u);EXPECT_LT(path.v[1],ego.v);}
    else EXPECT_TRUE(path.x.empty());
  }
}
namespace {
struct Scene : ::testing::Test {
  std::filesystem::path dir;
  AvoidRoadMap road;MGeoLoader map;RouteManager route;UrbanFeature feature;
  Scene() {
    dir=std::filesystem::temp_directory_path()/"asmc_dynamic_policy_test";
    std::filesystem::create_directories(dir);
    {std::ofstream f(dir/"drivable.json");f<<R"({"frame_id":"map","map_id":"waize_katri_2025-10-23","grid":{"origin_x":-100,"origin_y":-100},"drivable_areas":[{"validity":"validated","polygon":[[0,-2],[200,-2],[200,2],[0,2]]}]})";}
    {std::ofstream f(dir/"lines.json");f<<R"({"map_id":"kcity2025_competition_route_curation_candidate_v14","frame":{"local_origin_in_global":[302595,4124145]},"features":[{"feature_type":"lane_marking","attributes":{"pattern":"solid","functional_role":"lane_boundary_paint"},"geometry":{"coordinates":[[0,2],[200,2]]}}]})";}
    {std::ofstream f(dir/"drivable_grid.rle");f<<"3085 7232 0.2 -100 -100\n";for(int y=490;y<510;++y)f<<y<<" 500 1500\n";}
    {std::ofstream f(dir/"links.json");f<<R"([{"idx":"road","from_node_idx":"a","to_node_idx":"b","max_speed":18,"points":[[0,0],[200,0]]}])";}
    {std::ofstream f(dir/"nodes.json");f<<"[]";}
    std::string error;
    if(!road.Load((dir/"drivable.json").string(),(dir/"lines.json").string(),error)) throw std::runtime_error(error);
    if(!map.load((dir/"links.json").string(),(dir/"nodes.json").string()) || !route.initialize(map,{"road"}))throw std::runtime_error("route");
    feature.ego.x=feature.ego.s=20;feature.ego.speed=5;feature.ego.valid=true;
    feature.route.current_link_id=feature.route.route_required_link_id="road";
    feature.route.current_link_valid=true;feature.route.speed_limit=5;
  }
  ::TrackedObject box(int id,double x,double y,double vx=0,double vy=0) {
    ::TrackedObject o;o.id=id;o.type=0;o.x=x;o.y=y;o.vx=vx;o.vy=vy;
    o.motion_state=ObjectMotionState::DYNAMIC;o.geometry_valid=true;
    o.length=o.width=o.height=1;
    for(int i=0;i<8;++i)o.corners[i]={x+((i%4==0||i%4==3)?-.5:.5),y+(i%4<2?-.5:.5),double(i/4)};
    return o;
  }
  DynamicDecision decide(std::vector<::TrackedObject> objects,double age=0,bool ordinary=true,bool previous_stop=false) {
    DynamicPolicyConfig config;config.route_links={"road"};
    return DynamicObjectPolicy({&road,{2,4,1},{.3,.1,0}},config).evaluate(feature,route,objects,age,ordinary,previous_stop);
  }
};
TEST_F(Scene, CapturedCrosswalkDoesNotRestartOnOptimisticMotionWhileBodyStillOccupiesLane) {
  const auto input=YAML::LoadFile(std::string(ASMC_POLICY_SOURCE)+"/tests/fixtures/two_crosswalk_restart.yaml");
  const auto e=input["ego"],o=input["object"];
  const double yaw=e["yaw"].as<double>(),c=std::cos(yaw),s=std::sin(yaw);
  const double ex=e["x"].as<double>(),ey=e["y"].as<double>();
  auto project=[&](double x,double y){return Point2D{20+(x-ex)*c+(y-ey)*s,-(x-ex)*s+(y-ey)*c};};
  const auto p=project(o["center"]["x"].as<double>(),o["center"]["y"].as<double>());
  auto object=box(o["id"].as<int>(),p.x,p.y,
      o["vx"].as<double>()*c+o["vy"].as<double>()*s,-o["vx"].as<double>()*s+o["vy"].as<double>()*c);
  object.motion_state=static_cast<ObjectMotionState>(o["motion_state"].as<int>());
  object.length=o["length"].as<double>();object.width=o["width"].as<double>();
  for(int i=0;i<8;++i){const auto corner=o["obb_corners"][i];const auto q=project(corner["x"].as<double>(),corner["y"].as<double>());object.corners[i]={q.x,q.y,corner["z"].as<double>()};}
  feature.ego.speed=0;
  for(auto type:{ObjectType::UNKNOWN,ObjectType::PEDESTRIAN,ObjectType::VEHICLE}) {
    object.type=static_cast<int>(type);
    const auto decision=decide({object},input["observation_age_s"].as<double>(),true,true);
    EXPECT_EQ(DynamicAction::STOP,decision.action);
    EXPECT_EQ("observed_restart_occupancy",decision.reason);
  }
}
TEST_F(Scene, RestartOccupancyReleasesAfterObservedClearanceWithoutAStoredObjectLatch) {
  feature.ego.speed=0;
  auto object=box(5,28,0,3,0);
  EXPECT_EQ(DynamicAction::FOLLOW,decide({object}).action);
  EXPECT_EQ(DynamicAction::STOP,decide({object},0,true,true).action);
  for(auto& p:object.corners)p.y+=1.95;object.y+=1.95;
  // The physical body is clear, but the shared margin grows from 0.3 m
  // to 0.7 m over this four-second forecast. Its 0.45 m lateral gap must
  // therefore keep the stop. Do not relax that margin to make this pass.
  EXPECT_TRUE(road.IntersectsDrivable(geometry::ProjectCornersToXY(object.corners)));
  EXPECT_EQ(DynamicAction::STOP,decide({object},0,true,true).action);
  for(auto& p:object.corners)p.y+=.3;object.y+=.3;
  // At 2.25 m lateral offset the gap is 0.75 m, outside the full margin,
  // while part of the observed body still intersects the drivable road.
  EXPECT_TRUE(road.IntersectsDrivable(geometry::ProjectCornersToXY(object.corners)));
  EXPECT_EQ(DynamicAction::KEEP,decide({object},0,true,true).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({},0,true,true).action);
}
TEST_F(Scene, RestartOccupancyKeepsFreshnessScopeAndNativeManeuverGuards) {
  feature.ego.speed=0;auto object=box(5,28,0,3,0);
  EXPECT_EQ(DynamicAction::KEEP,decide({object},.31,true,true).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({object},0,false,true).action);
  feature.route.route_required_link_id="outside";
  EXPECT_EQ(DynamicAction::KEEP,decide({object},0,true,true).action);
}
TEST_F(Scene, UnmeasuredConflictBrakesAndKeepsLegacyOwnership) {
  for(auto motion:{ObjectMotionState::UNKNOWN}) {
    auto o=box(1,23,0);o.motion_state=motion;
    EXPECT_EQ(DynamicAction::EMERGENCY,decide({o}).action);EXPECT_EQ(1u,decide({o}).eligible);
    EXPECT_TRUE(decide({o}).evaluated_provisional_ids.empty());
  }
}
TEST_F(Scene, ExplicitUnavailableVelocityKeepsObstacleWithoutBrakingForRemoteObjects) {
  auto o=box(88,23,0); o.motion_state=ObjectMotionState::UNKNOWN;
  o.velocity_unavailable=true; o.vx=o.vy=o.speed=0.;
  auto result=decide({o});
  EXPECT_EQ(DynamicAction::EMERGENCY,result.action);
  EXPECT_TRUE(result.evaluated_provisional_ids.empty());
  o=box(88,23,8); o.motion_state=ObjectMotionState::UNKNOWN;
  o.velocity_unavailable=true; o.vx=o.vy=o.speed=0.;
  EXPECT_EQ(DynamicAction::KEEP,decide({o}).action);
}
TEST_F(Scene, UnclassifiedLowOccupancyCanOnlyNominateAFullyCheckedInLaneBypass) {
  auto object=box(98,33,0);object.type=static_cast<int>(ObjectType::UNKNOWN);
  object.motion_state=ObjectMotionState::LOW_THREAT;feature.ego.speed=3;
  EXPECT_EQ(DynamicAction::STOP,decide({object}).action);
  feature.ego.speed=1;object=box(98,30,1.6,0,.01);
  object.type=static_cast<int>(ObjectType::UNKNOWN);object.motion_state=ObjectMotionState::LOW_THREAT;
  EXPECT_EQ(DynamicAction::AVOID,decide({object}).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({object},.31).action);
  feature.route.route_required_link_id="other";
  EXPECT_EQ(DynamicAction::KEEP,decide({object}).action);
}
TEST_F(Scene, UnknownBypassOwnershipIsOnlyForTheNominatedOrFreshValidatedTarget) {
  PerceptionObjects original;behavior_planner::TrackedObject object;
  object.id=98;object.type=ObjectType::UNKNOWN;object.motion_state=MotionState::LOW_THREAT;
  original.obstacles.push_back(object);
  EXPECT_EQ(1u,DynamicObjectPolicy::legacyObjects(original,true,false,{98}).obstacles.size());
  EXPECT_TRUE(DynamicObjectPolicy::legacyObjects(original,true,false,{98},98).obstacles.empty());
  EXPECT_EQ(1u,DynamicObjectPolicy::legacyObjects(original,true,false,{},98).obstacles.size());
  EXPECT_TRUE(DynamicObjectPolicy::legacyObjects(original,true,true).obstacles.empty());
  EXPECT_EQ(1u,DynamicObjectPolicy::legacyObjects(original,false,true,{98},98).obstacles.size());
  for(auto motion:{MotionState::UNKNOWN,MotionState::DYNAMIC_PENDING}) {
    original.obstacles[0].motion_state=motion;
    EXPECT_EQ(1u,DynamicObjectPolicy::legacyObjects(original,true,true,{98},98).obstacles.size());
  }
  EXPECT_EQ(1u,original.obstacles.size());
}
TEST_F(Scene, NewActionsAreInactiveOutsideTheDynamicSection) {
  feature.route.route_required_link_id="A2256W000411";
  const auto result=decide({box(1,23,0)});
  EXPECT_EQ(DynamicAction::KEEP,result.action);
  EXPECT_EQ("outside_dynamic_section",result.reason);
}
TEST_F(Scene, DiagnosticsRetainEveryObjectGateWithoutChangingTheWinningAction) {
  feature.ego.speed=1;
  auto known=box(90,30,1.6);known.type=static_cast<int>(ObjectType::PEDESTRIAN);
  known.motion_state=ObjectMotionState::LOW_THREAT;
  auto unknown=known;unknown.id=91;unknown.type=static_cast<int>(ObjectType::UNKNOWN);
  const auto result=decide({known,unknown});
  EXPECT_EQ(DynamicAction::AVOID,result.action);
  ASSERT_EQ(2u,result.assessments.size());
  EXPECT_EQ("bypass_nomination",result.assessments[0].gate);
  EXPECT_EQ("bypass_nomination",result.assessments[1].gate);
  EXPECT_TRUE(result.assessments[0].bypass_room);
  EXPECT_FALSE(result.assessments[1].known_mobile);
  EXPECT_EQ(2u,result.evaluated_provisional_ids.size());
  auto unmeasured=known;unmeasured.motion_state=ObjectMotionState::UNKNOWN;
  const auto fallback=decide({unmeasured});
  EXPECT_EQ(DynamicAction::STOP,fallback.action);
  ASSERT_EQ(1u,fallback.assessments.size());
  EXPECT_EQ("unmeasured_observed_occupancy",fallback.assessments.front().gate);
}
TEST_F(Scene, MissingRouteScopeDoesNotEnableNewActions) {
  feature.route.route_required_link_id.clear();
  EXPECT_EQ(DynamicAction::KEEP,decide({box(1,23,0)}).action);
}
class OwnershipGreenLight final : public TrafficLightProvider {
 public:
  TrafficLightState getState(const std::string&,SignalMovement) const override {return TrafficLightState::GREEN;}
  bool hasRecentDetection(const std::string&) const override {return true;}
};
TEST_F(Scene, UnknownTargetOwnershipDoesNotEnableNewActionsBeforeEgoEntry) {
  DynamicPolicyConfig config;config.route_links={"road"};
  DynamicObjectPolicy policy({&road,{2,4,1},{.3,.1,0}},config);
  ObjectFeature object;object.id=7;object.x=60;object.length=4;object.width=1.9;
  object.valid=true;object.motion_state=MotionState::LOW_THREAT;
  EXPECT_FALSE(policy.allowsStaticAvoid(object,route));
  feature.route.route_required_link_id="before_section";
  EXPECT_EQ("outside_dynamic_section",policy.evaluate(feature,route,{},0,true).reason);
  object.type=ObjectType::STATIC_OBSTACLE;EXPECT_TRUE(policy.allowsStaticAvoid(object,route));
  object.type=ObjectType::UNKNOWN;object.valid=false;EXPECT_TRUE(policy.allowsStaticAvoid(object,route));
  object.valid=true;DynamicObjectPolicy disabled;EXPECT_TRUE(disabled.allowsStaticAvoid(object,route));
  config.route_links={"elsewhere"};
  DynamicObjectPolicy elsewhere({&road,{2,4,1},{.3,.1,0}},config);
  EXPECT_TRUE(elsewhere.allowsStaticAvoid(object,route));
}
TEST_F(Scene, EligibilityPrecedesHistoryAndCannotLeaveAGhostStaticTarget) {
  DynamicPolicyConfig config;config.route_links={"road"};
  DynamicObjectPolicy policy({&road,{2,4,1},{.3,.1,0}},config);
  FeatureBuilder builder,legacy;OwnershipGreenLight light;
  EgoState ego;ego.x=20;ego.speed=5;ego.valid=true;
  behavior_planner::TrackedObject object;object.id=7;object.x=60;object.length=4;object.width=1.9;
  object.motion_state=MotionState::LOW_THREAT;
  PerceptionObjects objects;objects.obstacles.push_back(object);
  auto allowed=[&](const ObjectFeature& o){return policy.allowsStaticAvoid(o,route);};
  EXPECT_TRUE(legacy.build(ego,objects,route,light).risk.static_obstacle_ahead);
  for(int i=0;i<5;++i) {
    ego.stamp=1+i*.1;objects.obstacles[0].id=7+i;
    const auto f=builder.build(ego,objects,route,light,allowed);
    EXPECT_FALSE(f.risk.static_obstacle_ahead);EXPECT_EQ(-1,f.risk.avoidance_object_id);
    EXPECT_EQ(ObjectType::UNKNOWN,objects.obstacles[0].type);
    EXPECT_DOUBLE_EQ(60,objects.obstacles[0].x);EXPECT_EQ(1u,objects.obstacles.size());
  }
  // There is no static-AVOID latch to resurrect an unobserved/re-ID target.
  const auto missing=builder.build(ego,PerceptionObjects{},route,light,allowed);
  EXPECT_FALSE(missing.risk.static_obstacle_ahead);
  auto emergency=missing;emergency.risk.emergency_risk=true;
  BehaviorDecision decision;const auto result=decision.decide(emergency,PlannerFeedbackData{},0);
  EXPECT_TRUE(result.hard.emergency_stop);EXPECT_DOUBLE_EQ(0,result.target.desired_speed);
  // Disabling this eligibility rule restores legacy selection immediately.
  EXPECT_TRUE(builder.build(ego,objects,route,light).risk.static_obstacle_ahead);
}
TEST_F(Scene, KnownStaticTargetKeepsItsExistingEligibilityAndHistory) {
  DynamicPolicyConfig config;config.route_links={"road"};
  DynamicObjectPolicy policy({&road,{2,4,1},{.3,.1,0}},config);
  FeatureBuilder builder;OwnershipGreenLight light;EgoState ego;ego.x=20;ego.speed=5;ego.valid=true;
  behavior_planner::TrackedObject object;object.id=7;object.x=60;object.length=4;object.width=1.9;
  object.type=ObjectType::STATIC_OBSTACLE;object.motion_state=MotionState::LOW_THREAT;
  PerceptionObjects objects;objects.obstacles.push_back(object);
  auto allowed=[&](const ObjectFeature& o){return policy.allowsStaticAvoid(o,route);};
  const auto f=builder.build(ego,objects,route,light,allowed);
  EXPECT_TRUE(f.risk.static_obstacle_ahead);EXPECT_EQ(7,f.risk.avoidance_object_id);
  EXPECT_TRUE(builder.build(ego,PerceptionObjects{},route,light,allowed).risk.static_obstacle_ahead);
}
TEST_F(Scene, CapturedStoppedTruckCannotAcquireStaticAvoidHistory) {
  const auto input=YAML::LoadFile(std::string(ASMC_POLICY_SOURCE)+"/tests/fixtures/dynamic_stopped_lead.json");
  const auto recorded=input["object"];
  behavior_planner::TrackedObject object;object.id=recorded["id"].as<int>();object.x=60;
  object.type=static_cast<ObjectType>(recorded["type"].as<int>());
  object.motion_state=static_cast<MotionState>(recorded["motion_state"].as<int>());
  object.length=recorded["length"].as<double>();object.width=recorded["width"].as<double>();
  ASSERT_EQ(ObjectType::UNKNOWN,object.type);ASSERT_EQ(MotionState::LOW_THREAT,object.motion_state);
  DynamicPolicyConfig config;config.route_links={"road"};
  DynamicObjectPolicy policy({&road,{2,4,1},{.3,.1,0}},config);
  FeatureBuilder builder;OwnershipGreenLight light;EgoState ego;ego.x=20;ego.speed=5;ego.valid=true;
  PerceptionObjects objects;objects.obstacles.push_back(object);
  EXPECT_FALSE(builder.build(ego,objects,route,light,
      [&](const ObjectFeature& o){return policy.allowsStaticAvoid(o,route);}).risk.static_obstacle_ahead);
}
TEST_F(Scene, MapConnectorDoesNotPunctureTheOrderedRouteSection) {
  feature.route.current_link_id="intersection_connector";
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({box(1,23,0)}).action);
}
TEST_F(Scene, ScopeBoundaryRestoresAllLegacyTargetsWithoutANewObservation) {
  PerceptionObjects original;
  behavior_planner::TrackedObject moving;
  moving.id=7;moving.motion_state=MotionState::DYNAMIC;moving.vx=3;
  original.vehicles.push_back(moving);original.pedestrians.push_back(moving);
  original.obstacles.push_back(moving);
  auto pending=moving;pending.id=8;pending.motion_state=MotionState::DYNAMIC_PENDING;
  original.obstacles.push_back(pending);
  const auto inside=DynamicObjectPolicy::legacyObjects(original,true);
  EXPECT_TRUE(inside.vehicles.empty());EXPECT_TRUE(inside.pedestrians.empty());
  ASSERT_EQ(1u,inside.obstacles.size());EXPECT_EQ(8,inside.obstacles[0].id);
  // Reuse exactly the same cached observation at exit; no object callback is
  // needed to restore the original semantic Follow/pedestrian safety input.
  const auto outside=DynamicObjectPolicy::legacyObjects(original,false);
  ASSERT_EQ(1u,outside.vehicles.size());ASSERT_EQ(1u,outside.pedestrians.size());
  ASSERT_EQ(2u,outside.obstacles.size());EXPECT_EQ(7,outside.vehicles[0].id);
  EXPECT_DOUBLE_EQ(3,outside.vehicles[0].vx);
  EXPECT_EQ(MotionState::DYNAMIC,outside.pedestrians[0].motion_state);
  EXPECT_EQ(2u,original.obstacles.size());
  EXPECT_EQ(1u,DynamicObjectPolicy::legacyObjects(original,true).obstacles.size());
}
TEST_F(Scene, ValidatedNativeAvoidOwnsLowPersonWithoutRemovingAnyCanonicalInput) {
  PerceptionObjects source;
  behavior_planner::TrackedObject person;person.id=81;person.type=ObjectType::PEDESTRIAN;
  person.motion_state=MotionState::LOW_THREAT;source.pedestrians.push_back(person);
  auto pending=person;pending.id=82;pending.motion_state=MotionState::DYNAMIC_PENDING;
  source.pedestrians.push_back(pending);
  auto unknown=person;unknown.id=83;unknown.motion_state=MotionState::UNKNOWN;
  source.pedestrians.push_back(unknown);
  auto fixed=person;fixed.type=ObjectType::STATIC_OBSTACLE;source.obstacles.push_back(fixed);
  EXPECT_EQ(3u,DynamicObjectPolicy::legacyObjects(source,true,false).pedestrians.size());
  EXPECT_EQ(3u,DynamicObjectPolicy::legacyObjects(source,false,true).pedestrians.size());
  const auto native=DynamicObjectPolicy::legacyObjects(source,true,true);
  ASSERT_EQ(2u,native.pedestrians.size());EXPECT_EQ(82,native.pedestrians[0].id);
  EXPECT_EQ(83,native.pedestrians[1].id);EXPECT_EQ(1u,native.obstacles.size());
  EXPECT_EQ(3u,source.pedestrians.size());
  // A failed/stale native cycle immediately returns ownership to the existing
  // person stop rule; this has no timer or remembered disappearance release.
  EXPECT_EQ(3u,DynamicObjectPolicy::legacyObjects(source,true,false).pedestrians.size());
}
TEST_F(Scene, SectionKeepsTheApproachCapAfterTheStopLineAndRestoresCruiseAtExit) {
  DynamicPolicyConfig config;config.route_links={"road"};config.section_speed_limit=25./3.6;
  DynamicObjectPolicy policy({&road,{2,4,1},{.3,.1,0}},config);
  BehaviorDecision decision;NormalCruiseConfig cruise;cruise.enabled=true;
  cruise.minimum_map_speed=40./3.6;cruise.target_speed=cruise.max_speed=58./3.6;
  ASSERT_TRUE(decision.setNormalCruiseConfig(cruise));
  feature.route.speed_limit=50./3.6;
  feature.dynamic=policy.evaluate(feature,route,{},0,true);
  ASSERT_TRUE(feature.dynamic.in_scope);
  auto result=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_EQ(BehaviorType::KEEP,result.selected_behavior);
  EXPECT_DOUBLE_EQ(25./3.6,result.target.desired_speed);
  EXPECT_DOUBLE_EQ(25./3.6,result.hard.max_speed);
  feature.route.route_required_link_id="outside";
  feature.dynamic=policy.evaluate(feature,route,{},0,true);
  EXPECT_FALSE(feature.dynamic.in_scope);
  result=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_GT(result.target.desired_speed,25./3.6);
  feature.route.route_required_link_id="road";
  feature.dynamic=policy.evaluate(feature,route,{},0,true);
  feature.risk.emergency_risk=true;
  result=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_TRUE(result.hard.emergency_stop);EXPECT_DOUBLE_EQ(0,result.target.desired_speed);
}
TEST_F(Scene, CapturedPendingCrossingBrakesBeforeDynamicConfirmation) {
  const auto input=YAML::LoadFile(std::string(ASMC_POLICY_SOURCE)+"/tests/fixtures/dynamic_pending_crossing.json");
  const auto e=input["ego"],o=input["object"];
  const double yaw=e["yaw"].as<double>(),c=std::cos(yaw),s=std::sin(yaw);
  const double ex=e["x"].as<double>(),ey=e["y"].as<double>();
  auto project=[&](double x,double y) {
    return Point2D{20+(x-ex)*c+(y-ey)*s,-(x-ex)*s+(y-ey)*c};
  };
  const auto p=project(o["center"][0].as<double>(),o["center"][1].as<double>());
  const double vx=o["velocity"][0].as<double>(),vy=o["velocity"][1].as<double>();
  auto object=box(o["id"].as<int>(),p.x,p.y,vx*c+vy*s,-vx*s+vy*c);
  object.motion_state=ObjectMotionState::DYNAMIC_PENDING;
  object.heading=o["yaw"].as<double>()-yaw;
  object.length=o["length"].as<double>();object.width=o["width"].as<double>();
  object.height=o["height"].as<double>();object.yaw_rate=o["yaw_rate"].as<double>();
  for(int i=0;i<8;++i) {
    auto corner=project(o["corners"][i][0].as<double>(),o["corners"][i][1].as<double>());
    object.corners[i]={corner.x,corner.y,o["corners"][i][2].as<double>()};
  }
  feature.ego.speed=e["speed"].as<double>();feature.route.speed_limit=feature.ego.speed;
  ASSERT_TRUE(geometry::ValidateCorners8(object.corners).valid);
  const auto decision=decide({object},input["observation_age_s"].as<double>());
  EXPECT_EQ(DynamicAction::EMERGENCY,decision.action);
  EXPECT_EQ(object.id,decision.object_id);
  EXPECT_EQ("pending_crossing_imminent",decision.reason);
}
TEST_F(Scene, CapturedDeerOccupancyBrakesBeforeAnyMotionConfirmation) {
  const auto input=YAML::LoadFile(std::string(ASMC_POLICY_SOURCE)+"/tests/fixtures/unmeasured_deer_entry.yaml");
  const auto e=input["ego"],o=input["object"];
  const double yaw=e["yaw"].as<double>(),c=std::cos(yaw),s=std::sin(yaw);
  const double ex=e["x"].as<double>(),ey=e["y"].as<double>();
  auto project=[&](double x,double y) {
    return Point2D{20+(x-ex)*c+(y-ey)*s,-(x-ex)*s+(y-ey)*c};
  };
  const auto p=project(o["center"]["x"].as<double>(),o["center"]["y"].as<double>());
  auto object=box(o["id"].as<int>(),p.x,p.y);
  object.motion_state=ObjectMotionState::UNKNOWN;
  object.heading=o["yaw"].as<double>()-yaw;
  object.length=o["length"].as<double>();object.width=o["width"].as<double>();
  object.height=o["height"].as<double>();
  for(int i=0;i<8;++i) {
    const auto corner=o["obb_corners"][i];
    const auto xy=project(corner["x"].as<double>(),corner["y"].as<double>());
    object.corners[i]={xy.x,xy.y,corner["z"].as<double>()};
  }
  feature.ego.speed=e["speed"].as<double>();feature.route.speed_limit=feature.ego.speed;
  ASSERT_TRUE(geometry::ValidateCorners8(object.corners).valid);
  const auto result=decide({object},input["observation_age_s"].as<double>());
  EXPECT_EQ(DynamicAction::EMERGENCY,result.action);
  EXPECT_EQ(object.id,result.object_id);
  EXPECT_TRUE(result.evaluated_provisional_ids.empty());
  EXPECT_EQ(ObjectMotionState::UNKNOWN,object.motion_state);
}
TEST_F(Scene, UnmeasuredOccupancyPreservesRoadFreshnessAndNativeSafetyGates) {
  auto object=box(75,24,0);object.motion_state=ObjectMotionState::UNKNOWN;
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({object}).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({object},.31).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({object},-.1).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({object},0,false).action);
  object.geometry_valid=false;EXPECT_EQ(DynamicAction::KEEP,decide({object}).action);
  object.geometry_valid=true;object=box(75,24,4,0,-3);
  object.motion_state=ObjectMotionState::UNKNOWN;
  EXPECT_EQ(DynamicAction::KEEP,decide({object}).action);
  feature.route.route_required_link_id="other";
  object=box(75,24,0);object.motion_state=ObjectMotionState::UNKNOWN;
  EXPECT_EQ(DynamicAction::KEEP,decide({object}).action);
}
TEST_F(Scene, UnmeasuredMotionNeverGrantsFollowOrAvoidAndPreservesKnownStaticPipeline) {
  auto object=box(76,24,0,1,0);object.motion_state=ObjectMotionState::UNKNOWN;
  for(int type:{0,1,3}) {
    object.type=type;const auto result=decide({object});
    EXPECT_NE(DynamicAction::FOLLOW,result.action);
    EXPECT_NE(DynamicAction::AVOID,result.action);
    EXPECT_TRUE(result.evaluated_provisional_ids.empty());
  }
  object.type=static_cast<int>(ObjectType::STATIC_OBSTACLE);
  EXPECT_EQ(DynamicAction::KEEP,decide({object}).action);
}
TEST_F(Scene, PendingCrossingCanYieldButCannotRequestFollowOrAvoid) {
  auto o=box(1,33,1.4,0,-.6);o.motion_state=ObjectMotionState::DYNAMIC_PENDING;
  EXPECT_EQ(DynamicAction::STOP,decide({o}).action);
  o.vy=-5;EXPECT_EQ(DynamicAction::KEEP,decide({o}).action);
  o=box(1,45,0,3,0);o.motion_state=ObjectMotionState::DYNAMIC_PENDING;
  EXPECT_EQ(DynamicAction::KEEP,decide({o}).action);
  o=box(1,33,4,0,-2);o.motion_state=ObjectMotionState::DYNAMIC_PENDING;
  EXPECT_EQ(DynamicAction::KEEP,decide({o}).action);
  // This object clears before ego arrival. An already overlapping object
  // must still brake even when its measured motion points away.
  o=box(1,33,1.4,0,2);o.motion_state=ObjectMotionState::DYNAMIC_PENDING;
  EXPECT_EQ(DynamicAction::KEEP,decide({o}).action);
  feature.ego.speed=1;
  o=box(1,30,1.1,0,-.1);o.motion_state=ObjectMotionState::DYNAMIC_PENDING;
  EXPECT_EQ(DynamicAction::STOP,decide({o}).action);
}
TEST_F(Scene, PendingLaneOccupancyBrakesWithoutASettledCrossingDirection) {
  auto object=box(97,33,0,.2,0);object.type=static_cast<int>(ObjectType::PEDESTRIAN);
  object.motion_state=ObjectMotionState::DYNAMIC_PENDING;feature.ego.speed=3;
  const auto decision=decide({object});
  EXPECT_EQ(DynamicAction::STOP,decision.action);
  ASSERT_EQ(1u,decision.evaluated_provisional_ids.size());
  PerceptionObjects source;behavior_planner::TrackedObject person;
  person.id=97;person.type=ObjectType::PEDESTRIAN;person.motion_state=MotionState::DYNAMIC_PENDING;
  source.pedestrians.push_back(person);
  EXPECT_TRUE(DynamicObjectPolicy::legacyObjects(source,true,false,decision.evaluated_provisional_ids).pedestrians.empty());
  EXPECT_EQ(1u,DynamicObjectPolicy::legacyObjects(source,true).pedestrians.size());
  EXPECT_TRUE(decide({object},.31).evaluated_provisional_ids.empty());
  EXPECT_EQ(ObjectMotionState::DYNAMIC_PENDING,object.motion_state);
}
TEST_F(Scene, PendingSafetyPreservesFreshnessGeometryAndNativeOwnership) {
  auto o=box(1,24,1.4,0,-2);o.motion_state=ObjectMotionState::DYNAMIC_PENDING;
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({o}).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({o},.31).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({o},-.1).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({o},0,false).action);
  o.geometry_valid=false;EXPECT_EQ(DynamicAction::KEEP,decide({o}).action);
  o.geometry_valid=true;
  o.motion_state=ObjectMotionState::UNKNOWN;EXPECT_EQ(DynamicAction::EMERGENCY,decide({o}).action);
  o.motion_state=ObjectMotionState::LOW_THREAT;EXPECT_EQ(DynamicAction::EMERGENCY,decide({o}).action);
}
TEST_F(Scene, OutsideObjectIsExcludedBeforeItsFutureEntry) {
  auto o=box(1,33,4,0,-2);
  const auto result=decide({o});EXPECT_EQ(DynamicAction::KEEP,result.action);EXPECT_EQ(1u,result.outside);
}
TEST_F(Scene, EntireFootprintIncludingBoundaryIsUsed) {
  auto o=box(1,33,2.3,0,-1);
  EXPECT_EQ(1u,decide({o}).eligible);
  o.geometry_valid=false;EXPECT_EQ(0u,decide({o}).eligible);
  EXPECT_FALSE(road.IntersectsDrivable({{10,4},{20,4},{20,5},{10,5}}));
  EXPECT_TRUE(road.IntersectsDrivable({{10,-4},{20,-4},{20,4},{10,4}}));
  EXPECT_FALSE(road.IntersectsDrivable({{NAN,0},{1,0},{1,1}}));
}
TEST_F(Scene, LargeRoadOverlappingObjectIsNotExcludedByRasterWorkBudget) {
  EXPECT_TRUE(road.IntersectsDrivable({{10,-15},{25,-15},{25,15},{10,15}}));
  EXPECT_FALSE(road.IntersectsDrivable({{10,10},{25,10},{25,40},{10,40}}));
}
TEST_F(Scene, MovingLeadFollowsAndCloseLeadBrakes) {
  EXPECT_EQ(DynamicAction::FOLLOW,decide({box(1,45,0,3,0)}).action);
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({box(1,23,0,1,0)}).action);
}
TEST_F(Scene, TiltedProjectionKeepsAllFourDynamicActions) {
  auto tilt=[](::TrackedObject object) {
    // A nonvertical box has more than four vertices in its XY projection.
    for(int i=4;i<8;++i){object.corners[i].x+=.03;object.corners[i].y+=.02;}
    EXPECT_TRUE(geometry::ValidateCorners8(object.corners).valid);
    EXPECT_EQ(6u,PredictObstacleFootprint(object,0).size());
    return object;
  };
  EXPECT_EQ(DynamicAction::FOLLOW,decide({tilt(box(1,45,0,3,0))}).action);
  EXPECT_EQ(DynamicAction::STOP,decide({tilt(box(2,33,1.4,0,-.6))}).action);
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({tilt(box(3,23,0))}).action);
  feature.ego.speed=1;
  EXPECT_EQ(DynamicAction::AVOID,decide({tilt(box(4,30,1.85,0,-.1))}).action);
}
TEST_F(Scene, CapturedNonverticalLeadIsEvaluated) {
  const auto input=YAML::LoadFile(std::string(ASMC_POLICY_SOURCE)+"/tests/fixtures/dynamic_tilted_lead.json");
  auto object=box(1,45,0);
  // Rigidly move the captured northbound observation onto this test's eastbound
  // road. Relative 3D shape and map-frame velocity are preserved.
  object.vx=input["velocity"][1].as<double>();object.vy=-input["velocity"][0].as<double>();
  object.heading=input["yaw"].as<double>()-M_PI/2;
  for(int i=0;i<8;++i) {
    object.corners[i].x=45+input["corners"][i][1].as<double>()-input["center"][1].as<double>();
    object.corners[i].y=-(input["corners"][i][0].as<double>()-input["center"][0].as<double>());
    object.corners[i].z=input["corners"][i][2].as<double>();
  }
  ASSERT_TRUE(geometry::ValidateCorners8(object.corners).valid);
  ASSERT_EQ(6u,PredictObstacleFootprint(object,.1).size());
  EXPECT_EQ(DynamicAction::FOLLOW,decide({object},.1).action);
}
TEST_F(Scene, CanonicalCornersOverrideSmallerScalarSizeForLeadGap) {
  auto object=box(1,29,0,3,0);
  for(auto& p:object.corners)p.x=29+(p.x<29?-8.5:8.5);
  ASSERT_EQ(1,object.length);
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({object}).action);
}
TEST_F(Scene, LeadReactionDelayConsumesTheRemainingBrakingReserve) {
  // Gap 2.5m, closing speed 4m/s: ignoring reaction gives 3.2m/s²;
  // after 0.3s reaction and 0.3m clearance it exceeds the 5m/s² boundary.
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({box(1,26,0,1,0)}).action);
}
TEST_F(Scene, AbsoluteSpeedCapAppliesAfterCruiseAndPreservesLowerStops) {
  feature.route.speed_limit=30;
  BehaviorDecision decision;
  ASSERT_TRUE(decision.setAbsoluteSpeedLimit(57.5/3.6));
  EXPECT_FALSE(decision.setAbsoluteSpeedLimit(NAN));
  EXPECT_FALSE(decision.setAbsoluteSpeedLimit(-1));
  NormalCruiseConfig cruise;cruise.enabled=true;
  cruise.target_speed=cruise.max_speed=20;cruise.minimum_map_speed=10;
  ASSERT_TRUE(decision.setNormalCruiseConfig(cruise));
  auto result=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_DOUBLE_EQ(57.5/3.6,result.hard.max_speed);
  EXPECT_DOUBLE_EQ(57.5/3.6,result.target.desired_speed);
  feature.risk.emergency_risk=true;
  result=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_TRUE(result.hard.emergency_stop);
  EXPECT_DOUBLE_EQ(0,result.target.desired_speed);
}
TEST_F(Scene, CrossingAtSameTimeYieldsButEarlierCrossingDoesNot) {
  auto crossing=box(1,33,1.4,0,-.6);
  EXPECT_EQ(DynamicAction::STOP,decide({crossing}).action);
  crossing.vy=-5;
  EXPECT_EQ(DynamicAction::KEEP,decide({crossing}).action);
}
TEST_F(Scene, LaneFillingSideObstructionStopsInsteadOfRequestingImpossibleAvoid) {
  feature.ego.speed=1;
  auto o=box(1,30,1.1,0,-.1);
  EXPECT_EQ(DynamicAction::STOP,decide({o}).action);
}
TEST_F(Scene, SlowEdgeConflictCanPrepareAvoidBeforePhysicalBodyOverlap) {
  feature.ego.speed=1;
  auto edge=box(21,30,1.85,0,.01);
  ASSERT_GT(edge.y,.5*(edge.width+2.0));
  // The timed shared checker includes safety margin. An edge conflict can
  // have room for an in-lane bypass before touching the physical body tube.
  EXPECT_EQ(DynamicAction::AVOID,decide({edge}).action);
  edge.motion_state=ObjectMotionState::DYNAMIC_PENDING;
  EXPECT_NE(DynamicAction::AVOID,decide({edge}).action);
  edge.motion_state=ObjectMotionState::DYNAMIC;
  feature.ego.speed=5;
  EXPECT_NE(DynamicAction::AVOID,decide({edge}).action);
}
TEST_F(Scene, LowMotionPersonUsesOccupancyWithoutClaimingStandstillOrDroppingUnknown) {
  auto person=box(90,33,0);person.type=static_cast<int>(ObjectType::PEDESTRIAN);
  person.motion_state=ObjectMotionState::LOW_THREAT;
  feature.ego.speed=3;
  const auto stop=decide({person});
  EXPECT_EQ(DynamicAction::STOP,stop.action);
  ASSERT_EQ(1u,stop.evaluated_provisional_ids.size());EXPECT_EQ(90,stop.evaluated_provisional_ids[0]);
  person.motion_state=ObjectMotionState::UNKNOWN;
  EXPECT_TRUE(decide({person}).evaluated_provisional_ids.empty());
  person.motion_state=ObjectMotionState::LOW_THREAT;
  EXPECT_TRUE(decide({person},.31).evaluated_provisional_ids.empty());
  person.geometry_valid=false;EXPECT_TRUE(decide({person}).evaluated_provisional_ids.empty());
  person.geometry_valid=true;person.vx=NAN;
  EXPECT_TRUE(decide({person}).evaluated_provisional_ids.empty());
  person.vx=0;person.type=static_cast<int>(ObjectType::STATIC_OBSTACLE);
  EXPECT_TRUE(decide({person}).evaluated_provisional_ids.empty());
}
TEST_F(Scene, StationaryOccupancyStopTargetDoesNotChaseTheEgo) {
  auto person=box(95,33,0);person.type=static_cast<int>(ObjectType::PEDESTRIAN);
  person.motion_state=ObjectMotionState::LOW_THREAT;
  feature.ego.speed=3;const auto first=decide({person});
  feature.ego.x=feature.ego.s=21;feature.ego.speed=2.5;
  const auto next=decide({person});
  ASSERT_EQ(DynamicAction::STOP,first.action);ASSERT_EQ(DynamicAction::STOP,next.action);
  EXPECT_NEAR(first.stop_s,next.stop_s,1e-9);
  EXPECT_LT(first.stop_s,person.x-.5*person.length-3);
}
TEST_F(Scene, LowVehicleLeadUsesConservativeGapFollowWithoutClaimingDynamicMotion) {
  auto lead=box(99,45,0);lead.type=static_cast<int>(ObjectType::VEHICLE);
  lead.motion_state=ObjectMotionState::LOW_THREAT;feature.ego.speed=3;
  const auto follow=decide({lead});
  ASSERT_EQ(DynamicAction::FOLLOW,follow.action);
  EXPECT_EQ(MotionState::LOW_THREAT,follow.lead.motion_state);
  EXPECT_DOUBLE_EQ(0,follow.lead.speed);
  lead.x=24;for(auto& p:lead.corners)p.x-=21;
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({lead}).action);
  lead=box(99,33,0,0,.4);lead.type=static_cast<int>(ObjectType::VEHICLE);
  lead.motion_state=ObjectMotionState::LOW_THREAT;
  EXPECT_NE(DynamicAction::FOLLOW,decide({lead}).action);
}
TEST_F(Scene, LowMotionOwnershipRunsOnceBeforeLegacyPersonRiskAndEncodesPlannedStop) {
  DynamicPolicyConfig config;config.route_links={"road"};
  DynamicObjectPolicy policy({&road,{2,4,1},{.3,.1,0}},config);
  auto canonical=box(91,33,0);canonical.type=static_cast<int>(ObjectType::PEDESTRIAN);
  canonical.motion_state=ObjectMotionState::LOW_THREAT;
  PerceptionObjects source;behavior_planner::TrackedObject semantic;
  semantic.id=91;semantic.type=ObjectType::PEDESTRIAN;semantic.motion_state=MotionState::LOW_THREAT;
  semantic.x=33;semantic.length=semantic.width=1;source.pedestrians.push_back(semantic);
  EgoState ego;ego.x=20;ego.speed=3;ego.valid=true;ego.stamp=100;
  FeatureBuilder builder;OwnershipGreenLight light;int invocations=0;
  auto prepared=builder.build(ego,source,route,light,{},
      [&](UrbanFeature& current,const PerceptionObjects& original) {
        ++invocations;current.dynamic=policy.evaluate(current,route,{canonical},0,true);
        return DynamicObjectPolicy::legacyObjects(original,true,false,current.dynamic.evaluated_provisional_ids);
      });
  EXPECT_EQ(1,invocations);EXPECT_EQ(1,prepared.frame_count);
  EXPECT_FALSE(prepared.risk.pedestrian_risk);EXPECT_EQ(DynamicAction::STOP,prepared.dynamic.action);
  prepared.risk.min_ttc=std::min(prepared.risk.min_ttc,prepared.dynamic.ttc);
  BehaviorDecision decision;const auto context=decision.decide(prepared,PlannerFeedbackData{},0);
  EXPECT_TRUE(context.hard.planned_stop);EXPECT_FALSE(context.hard.emergency_stop);
  V2CommandManager manager;
  EXPECT_EQ(asmc_msgs::BehaviorCommand::STOP,manager.update(context,nullptr,ros::Time(100),.5).maneuver_type);
  EXPECT_EQ(1u,source.pedestrians.size());EXPECT_EQ(ObjectMotionState::LOW_THREAT,canonical.motion_state);
  // Missing/invalid observation cannot claim the legacy person's protection.
  FeatureBuilder fallback;
  const auto unowned=fallback.build(ego,source,route,light);
  EXPECT_TRUE(unowned.risk.pedestrian_risk);
}
TEST_F(Scene, LowMotionRoadsidePersonDoesNotStopAReleasedSignal) {
  PerceptionObjects source;
  behavior_planner::TrackedObject person;
  person.id=799;person.type=ObjectType::PEDESTRIAN;
  person.motion_state=MotionState::LOW_THREAT;
  person.x=33;person.y=3.2;person.length=.6;person.width=.2;
  source.pedestrians.push_back(person);
  EgoState ego;ego.x=20;ego.y=0;ego.speed=4;ego.valid=true;ego.stamp=100;
  FeatureBuilder builder;OwnershipGreenLight light;
  EXPECT_FALSE(builder.build(ego,source,route,light).risk.pedestrian_risk);
  source.pedestrians.front().y=.2;
  EXPECT_TRUE(builder.build(ego,source,route,light).risk.pedestrian_risk);
  source.pedestrians.front().y=3.2;
  source.pedestrians.front().motion_state=MotionState::DYNAMIC_PENDING;
  source.pedestrians.front().vx=1.0;
  EXPECT_TRUE(builder.build(ego,source,route,light).risk.pedestrian_risk);
}
TEST_F(Scene, EmergencyOverridesFollowAcrossMultipleObjects) {
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({box(1,45,0,3,0),box(2,23,0)}).action);
}
TEST_F(Scene, EqualRiskLeadsSelectTheNearestIndependentlyOfInputOrder) {
  auto far=box(10,55,0,8,0),near=box(20,35,0,8,0);
  EXPECT_EQ(20,decide({far,near}).object_id);
  EXPECT_EQ(20,decide({near,far}).object_id);
}
TEST_F(Scene, StaleAndComplexManeuverCannotCreateDelayedHazardMemory) {
  auto object=box(1,23,0);
  EXPECT_EQ(DynamicAction::KEEP,decide({object},.4).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({object},0,false).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({}).action);
  EXPECT_EQ(DynamicAction::EMERGENCY,decide({object}).action);
  EXPECT_EQ(DynamicAction::KEEP,decide({}).action);
}
TEST_F(Scene, PlannedStopRemainsStopAndDoesNotBecomeForceEmergency) {
  feature.dynamic=decide({box(1,33,1.4,0,-.6)});
  ASSERT_EQ(DynamicAction::STOP,feature.dynamic.action);
  BehaviorDecision decision;V2CommandManager manager;
  const auto context=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_TRUE(context.hard.planned_stop);EXPECT_FALSE(context.hard.force_stop);
  auto command=manager.update(context,nullptr,ros::Time(10),.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::STOP,command.maneuver_type);EXPECT_TRUE(command.has_target_pose);
}
TEST_F(Scene, FreshAvoidRequestSurvivesLongPreparationThenCommits) {
  feature.ego.speed=1;BehaviorDecision decision;V2CommandManager manager;
  PlannerFeedbackData feedback;asmc_msgs::FrenetFeedback wire_feedback;
  asmc_msgs::BehaviorCommand command;
  // Longer than the native three-frame missing-target grace: a still-visible
  // dynamic target must keep supplying its live request during PREPARE.
  for(int i=0;i<12;++i) {
    feature.timestamp=100+i*.05;
    const bool native=DynamicObjectPolicy::usesNativePathSafety(BehaviorType::AVOID,PlannerExecutionState::WAITING);
    EXPECT_FALSE(native);
    feature.dynamic=decide({box(1,30,1.85,0,-.1)},0,!native);
    auto context=decision.decide(feature,feedback,manager.activeCommandId());
    command=manager.update(context,i?&wire_feedback:nullptr,ros::Time(feature.timestamp),.5);
    ASSERT_EQ(asmc_msgs::BehaviorCommand::AVOID,command.maneuver_type);
    wire_feedback.command_id=command.command_id;wire_feedback.execution_state=asmc_msgs::FrenetFeedback::WAITING;
    feedback.valid=true;feedback.command_id=command.command_id;feedback.execution_state=PlannerExecutionState::WAITING;
  }
  feedback.execution_state=PlannerExecutionState::READY;wire_feedback.execution_state=asmc_msgs::FrenetFeedback::READY;
  auto context=decision.decide(feature,feedback,manager.activeCommandId());
  command=manager.update(context,&wire_feedback,ros::Time(101),.5);
  EXPECT_EQ(asmc_msgs::BehaviorCommand::EXECUTE,command.execution_request);
  EXPECT_TRUE(DynamicObjectPolicy::usesNativePathSafety(BehaviorType::AVOID,PlannerExecutionState::EXECUTING));
  EXPECT_TRUE(DynamicObjectPolicy::usesNativePathSafety(BehaviorType::MERGE,PlannerExecutionState::WAITING));
  EXPECT_FALSE(DynamicObjectPolicy::usesNativePathSafety(BehaviorType::AVOID,PlannerExecutionState::COMPLETED));
}
TEST_F(Scene, ExistingCloserSignalAndEmergencyRemainAuthoritative) {
  feature.signal.has_stop_line=feature.signal.stop_line_valid=true;
  feature.signal.distance_to_stop_line=8;feature.signal.stop_line_s=28;feature.signal.stop_target_s=27;
  feature.signal.stop_line_x=27;feature.signal.light_state=TrafficLightState::RED;
  feature.dynamic=decide({box(1,45,1,0,-.1)});
  BehaviorDecision decision;auto context=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_TRUE(context.hard.planned_stop);EXPECT_DOUBLE_EQ(27,context.target.stop_x);
  feature.risk.emergency_risk=true;context=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_TRUE(context.hard.emergency_stop);
}
TEST_F(Scene, GreenSignalDoesNotStealDynamicStopTarget) {
  feature.dynamic=decide({box(1,33,1.4,0,-.6)});
  ASSERT_EQ(DynamicAction::STOP,feature.dynamic.action);
  feature.signal.has_stop_line=feature.signal.stop_line_valid=true;
  feature.signal.distance_to_stop_line=2;feature.signal.stop_line_s=22;
  feature.signal.stop_target_s=21;feature.signal.stop_line_x=21;
  feature.signal.light_state=TrafficLightState::GREEN;
  BehaviorDecision decision;const auto context=decision.decide(feature,PlannerFeedbackData{},0);
  EXPECT_TRUE(context.hard.planned_stop);EXPECT_FALSE(context.hard.force_stop);
  EXPECT_DOUBLE_EQ(feature.dynamic.stop_x,context.target.stop_x);
  EXPECT_DOUBLE_EQ(feature.dynamic.stop_s,context.target.stop_before_s);
}
TEST_F(Scene, NativeDynamicAvoidCandidateStaysWithinLaneAndChecksAllObjects) {
  {std::ofstream f(dir/"waypoints.txt");for(int i=0;i<=200;++i)f<<i<<" 0\n";}
  auto config=YAML::LoadFile(std::string(ASMC_POLICY_SOURCE)+"/config/frenet/params.yaml");
  config["roundabout_merge"]["enabled"]=false;
  config["waypoint_file"]=(dir/"waypoints.txt").string();
  config["avoid_hd_map"]["directory"]=dir.string();
  config["planner"]["collision_check"]["margin_growth_rate"]=0.0;
  {std::ofstream f(dir/"params.yaml");f<<config;}
  FrenetPlanner planner;ASSERT_TRUE(planner.Init((dir/"params.yaml").string()));
  CartesianState ego{};ego.x=30;ego.v=1;
  auto target=box(1,45,1.85,0,.01);
  PlannerCommand command{};command.mode=AVOID;command.target_object_id=1;
  command.has_target_speed=true;command.target_speed=1;
  CartesianPath path;FrenetPlanner::AvoidPrepareResult result;
  for(int i=0;i<100 && !result.candidate_valid;++i)planner.PrepareAvoid(ego,{target},command,path,result);
  if(!result.candidate_valid) {
    double smallest=1e9;int road_count=0,clear_count=0;
    for(const auto& candidate:planner.last_candidate_paths()) {
      if(candidate.x.empty())continue;
      double lateral=0;
      for(std::size_t i=0;i<candidate.x.size();++i) {
        CartesianState pose{};pose.x=candidate.x[i];pose.y=candidate.y[i];pose.yaw=candidate.yaw[i];
        for(const auto& point:MakeEgoFootprint(pose,planner.vehicle_shape(),.3))lateral=std::max(lateral,std::fabs(point.y));
      }
      smallest=std::min(smallest,lateral);
      road_count+=planner.road_map().EvaluatePath(candidate,planner.vehicle_shape(),.1).drivable;
      clear_count+=EvaluateCartesianTrajectorySafety(candidate,{target},planner.vehicle_shape(),planner.collision_config(),.1).clear;
    }
    std::cerr<<"candidate diagnostic: min expanded body="<<smallest<<" road="<<road_count<<" clear="<<clear_count<<'\n';
  }
  ASSERT_TRUE(result.candidate_valid);
  for(double y:result.candidate_path.y) EXPECT_LE(std::fabs(y),.5*(3.5-planner.vehicle_shape().width)-.3+1e-6);
  for(std::size_t i=0;i<result.candidate_path.x.size();++i) {
    CartesianState pose{};pose.x=result.candidate_path.x[i];pose.y=result.candidate_path.y[i];pose.yaw=result.candidate_path.yaw[i];
    for(const auto& point:MakeEgoFootprint(pose,planner.vehicle_shape(),.3))EXPECT_LE(std::fabs(point.y),1.75+1e-6);
  }
  auto blocker=box(2,35,-.4);blocker.length=5;blocker.width=2;
  for(int i=0;i<8;++i)blocker.corners[i]={blocker.x+((i%4==0||i%4==3)?-2.5:2.5),blocker.y+(i%4<2?-1:1),double(i/4)};
  EXPECT_FALSE(EvaluateCartesianTrajectorySafety(result.candidate_path,{blocker},planner.vehicle_shape(),planner.collision_config(),.1).clear);
}

TEST_F(Scene, EdgeDecisionConnectsToNativePrepareWithProductionCollisionMargins) {
  {std::ofstream f(dir/"waypoints.txt");for(int i=0;i<=200;++i)f<<i<<" 0\n";}
  auto config=YAML::LoadFile(std::string(ASMC_POLICY_SOURCE)+"/config/frenet/params.yaml");
  config["roundabout_merge"]["enabled"]=false;
  config["waypoint_file"]=(dir/"waypoints.txt").string();
  config["avoid_hd_map"]["directory"]=dir.string();
  {std::ofstream f(dir/"edge_params.yaml");f<<config;}
  FrenetPlanner planner;ASSERT_TRUE(planner.Init((dir/"edge_params.yaml").string()));
  feature.ego.x=feature.ego.s=30;feature.ego.speed=3;
  auto target=box(61,47,2.2);
  DynamicPolicyConfig policy_config;policy_config.route_links={"road"};
  DynamicObjectPolicy policy({&road,planner.vehicle_shape(),planner.collision_config()},policy_config);
  const auto decision=policy.evaluate(feature,route,{target},0,true);
  ASSERT_EQ(DynamicAction::AVOID,decision.action);
  CartesianState ego{};ego.x=30;ego.v=3;
  PlannerCommand command{};command.mode=AVOID;command.target_object_id=decision.object_id;
  command.has_target_speed=true;command.target_speed=5;
  CartesianPath path;FrenetPlanner::AvoidPrepareResult prepared;
  for(int i=0;i<100&&!prepared.candidate_valid;++i)
    planner.PrepareAvoid(ego,{target},command,path,prepared);
  ASSERT_TRUE(prepared.candidate_valid);
  for(int cycle=0;cycle<4;++cycle) {
    planner.PrepareAvoid(ego,{target},command,path,prepared);
    EXPECT_TRUE(prepared.candidate_valid)<<"fresh revalidation must not restart the entire sliced search";
  }
  EXPECT_TRUE(EvaluateCartesianTrajectorySafety(prepared.candidate_path,{target},
      planner.vehicle_shape(),planner.collision_config(),planner.sample_dt()).clear);
  for(std::size_t i=0;i<prepared.candidate_path.x.size();++i) {
    CartesianState pose{};pose.x=prepared.candidate_path.x[i];pose.y=prepared.candidate_path.y[i];
    pose.yaw=prepared.candidate_path.yaw[i];
    for(const auto& corner:MakeEgoFootprint(pose,planner.vehicle_shape(),planner.collision_config().safety_margin))
      EXPECT_LE(std::fabs(corner.y),1.75+1e-6);
  }
  ASSERT_TRUE(planner.avoid_in_lane_only());
  // A moving object may become LOW_THREAT while braking. Its already-bound
  // bypass must not silently acquire permission to use an adjacent lane.
  target.motion_state=ObjectMotionState::LOW_THREAT;
  planner.PrepareAvoid(ego,{target},command,path,prepared);
  EXPECT_TRUE(planner.avoid_in_lane_only());
  planner.ResetExternalAvoidance();
  EXPECT_FALSE(planner.avoid_in_lane_only());
  // Native intent tightens an unclassified LOW object's permission without
  // relabeling it or changing the default static AVOID contract elsewhere.
  target.type=static_cast<int>(ObjectType::UNKNOWN);
  command.avoid_in_lane_only=true;
  planner.PrepareAvoid(ego,{target},command,path,prepared);
  EXPECT_TRUE(planner.avoid_in_lane_only());
  planner.ResetExternalAvoidance();command.avoid_in_lane_only=false;
  planner.PrepareAvoid(ego,{target},command,path,prepared);
  EXPECT_FALSE(planner.avoid_in_lane_only());
  planner.ResetExternalAvoidance();
  target.type=static_cast<int>(ObjectType::PEDESTRIAN);
  planner.PrepareAvoid(ego,{target},command,path,prepared);
  EXPECT_TRUE(planner.avoid_in_lane_only());
  ASSERT_TRUE(prepared.candidate_valid);
  ASSERT_TRUE(planner.CommitAvoid(prepared));
  // Scalar metadata can disagree with the authoritative eight corners.
  // Re-observation must not replace a validated in-lane target with an
  // unchecked static-clearance offset outside the permitted lane.
  target.width=3;
  planner.PlanCommittedAvoid(ego,{target},command,path,prepared);
  ASSERT_TRUE(prepared.candidate_valid);
  EXPECT_LE(std::fabs(prepared.selected_target_d),
      .5*(planner.lane_width()-planner.vehicle_shape().width)-planner.collision_config().safety_margin);
}

}
