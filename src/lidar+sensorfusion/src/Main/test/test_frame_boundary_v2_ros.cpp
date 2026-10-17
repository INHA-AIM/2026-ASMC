// B0 V2 output topics over real ROS publish/subscribe (rostest Main frame_boundary_v2.test).
// The existing test_frame_boundary_ros.cpp (V1 compatibility) is not modified. Each scan here runs
// the same sequence as main.cpp TemporalThread: V1 Publish/Invalid, GateInputs, Decide, PublishV2.
#include <gtest/gtest.h>
#include <Geometry/FrameBoundaryPublisher.hpp>
#include <Geometry/OutputGate.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <asmc_lidar/projection.hpp>
#include <boost/ref.hpp>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

extern VehicleSpec st_VehicleSpec;
namespace {
using ArrayV1=asmc_msgs::LidarObjectArray;
using ArrayV2=asmc_msgs::LidarObjectArrayV2;
using output_gate::Mode;
constexpr const char* kLocalV1="/perception/lidar/objects_local";
constexpr const char* kMapV1="/perception/lidar/objects_map";
constexpr const char* kLocalV2="/lidar/local_object";
constexpr const char* kMapV2="/lidar/map_object";

template<class M> struct Inbox {
  std::mutex mutex; std::vector<M> items;
  void operator()(const boost::shared_ptr<const M>& m){std::lock_guard<std::mutex> l(mutex);items.push_back(*m);}
  std::size_t size(){std::lock_guard<std::mutex> l(mutex);return items.size();}
  M at(std::size_t i){std::lock_guard<std::mutex> l(mutex);return items.at(i);}
};
void Configure(bool enabled=true) {
  frame_boundary::ConfigureForTest(enabled,true,enabled?"test_calibration":"");
  st_VehicleSpec=VehicleSpec{};
  st_VehicleSpec.st_LiDARPos={.8f,0,1.45f}; st_VehicleSpec.st_LiDARRotationDeg={0,0,0};
  st_VehicleSpec.st_GPSPos={.35f,0,1.25f};
}
GPS_IMU Local() {
  GPS_IMU out{}; out.st_IMU.b_ImuReady=true;
  out.st_IMU.f32_Roll_rad=.02f; out.st_IMU.f32_Pitch_rad=-.015f; out.st_IMU.f32_Yaw_rad=.4f;
  out.st_CAN.f32_CurrentVel_MPS=2.; out.st_CAN.f32_CurrentVel_KPH=7.2;
  return out;
}
std::int64_t Ns(double t){return static_cast<std::int64_t>(ros::Time(t).toNSec());}
asmc::lidar::PoseSelection MapPose(double t,bool valid,const std::string& reason) {
  asmc::lidar::PoseSelection m; m.valid=valid; m.reason=reason; m.query_stamp_ns=Ns(t); m.clock="ros";
  m.map_from_base.linear()=frame_boundary::MapRotation(Local());
  m.map_from_base.translation()=asmc::lidar::Vec(100,200,0); return m;
}
LiDARObjectInfo Detection(int id,double x,double y) {
  LiDARObjectInfo out{}; out.s32_Id=id; out.s32_Age=4; out.str_ObjectClass="car"; out.str_YoloClass="car";
  out.f32_CoreX=out.f32_OBB_CenterX=x; out.f32_CoreY=out.f32_OBB_CenterY=y;
  out.f32_CoreZ=out.f32_OBB_CenterZ=-.2;
  out.f32_OBB_Width=1.; out.f32_OBB_Height=.8; out.f32_OBB_ZMin=-.5; out.f32_OBB_ZMax=.5;
  for(unsigned i=0;i<8;++i) {
    out.OBB_Corners_Body[i]={float(x+((i&1)? .5:-.5)),float(y+((i&2)? .4:-.4)),float((i&4)? .5:-.5)};
    pcl::PointXYZI p; p.x=out.OBB_Corners_Body[i].x; p.y=out.OBB_Corners_Body[i].y; p.z=out.OBB_Corners_Body[i].z;
    p.intensity=1; out.pcl_ClusterPoints.push_back(p);
  }
  out.motion_valid=true; out.s32_MotionState=static_cast<int>(MotionState::LowThreat);
  return out;
}
bool Wait(const std::function<bool()>& condition,double seconds=3.) {
  const auto end=ros::WallTime::now()+ros::WallDuration(seconds);
  while(ros::ok() && ros::WallTime::now()<end) {if(condition()) return true;ros::WallDuration(.01).sleep();}
  return condition();
}
// One scan as main.cpp runs it. pose_ready=false: PrepareScan is skipped (no context for this scan).
struct Scan { ros::Time stamp; LiDAR lidar; output_gate::Decision decision; };
Scan Step(frame_boundary::Publisher& publisher,double t,const asmc::lidar::PoseSelection& map,
          std::vector<LiDARObjectInfo> objects,std::uint64_t generation,bool pose_ready=true,
          const std::string& pose_reason="local_ready") {
  Scan scan; scan.stamp=ros::Time(t);
  if(pose_ready) {
    frame_boundary::PrepareScan(asmc::lidar::Observation{Ns(t),Ns(t),Ns(t),"ros","bridge_receive_ns",""},
        Local(),st_VehicleSpec,map,generation);
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(.02,-.015,asmc::lidar::Rot::Identity()));
    scan.lidar.frame_context=frame_boundary::Current();
    scan.lidar.vec_Objects=std::move(objects);
    publisher.Publish(scan.lidar);
  } else {
    publisher.Invalid(scan.stamp,pose_reason);
  }
  const auto gate=frame_boundary::GateInputs(pose_ready,pose_reason,pose_ready,scan.lidar,generation);
  scan.decision=publisher.PublishV2(scan.lidar,scan.stamp,output_gate::Decide(gate));
  return scan;
}
template<class M> void ExpectSameV1Fields(const M& v2,const asmc_msgs::LidarObject& v1) {
  EXPECT_EQ(v2.track_id,v1.track_id); EXPECT_EQ(v2.detection_index,v1.detection_index);
  EXPECT_EQ(v2.observation_state,v1.observation_state); EXPECT_EQ(v2.age,v1.age);
  EXPECT_EQ(v2.miss_count,v1.miss_count); EXPECT_EQ(v2.classification,v1.classification);
  EXPECT_EQ(v2.camera_class,v1.camera_class);
  EXPECT_NEAR(v2.center.x,v1.center.x,1e-9); EXPECT_NEAR(v2.center.y,v1.center.y,1e-9);
  EXPECT_NEAR(v2.center.z,v1.center.z,1e-9);
  for(unsigned i=0;i<8;++i) {
    EXPECT_NEAR(v2.obb_corners[i].x,v1.obb_corners[i].x,1e-9); EXPECT_NEAR(v2.obb_corners[i].y,v1.obb_corners[i].y,1e-9);
    EXPECT_NEAR(v2.obb_corners[i].z,v1.obb_corners[i].z,1e-9);
  }
  EXPECT_NEAR(v2.yaw,v1.yaw,1e-12); EXPECT_EQ(v2.yaw_rate_valid,v1.yaw_rate_valid);
  EXPECT_EQ(v2.yaw_rate_rad_s,v1.yaw_rate_rad_s); EXPECT_EQ(v2.length,v1.length);
  EXPECT_EQ(v2.width,v1.width); EXPECT_EQ(v2.height,v1.height);
  EXPECT_EQ(v2.relative_velocity_valid,v1.relative_velocity_valid);
  EXPECT_EQ(v2.relative_velocity_mps,v1.relative_velocity_mps);
  EXPECT_EQ(v2.absolute_velocity_valid,v1.absolute_velocity_valid);
  EXPECT_EQ(v2.absolute_velocity_mps,v1.absolute_velocity_mps);
  EXPECT_EQ(v2.source_point_count,v1.source_point_count);
  EXPECT_FALSE(v1.dynamic_valid);                       // V1 meaning unchanged
  EXPECT_EQ(v2.dynamic_valid,v2.motion_valid);          // V2 compatibility-derived
}
std::map<std::string,std::string> Values(const diagnostic_msgs::DiagnosticArray& array) {
  std::map<std::string,std::string> out;
  for(const auto& status:array.status) for(const auto& value:status.values) out[value.key]=value.value;
  return out;
}

// Production (common geometry OFF, main.launch default): no V2 topic exists at all. Runs first so
// that no earlier publisher of these topics exists in this process.
TEST(FrameBoundaryV2, CommonGeometryOffAdvertisesNoV2AndPassesDecisionThrough) {
  Configure(false);
  ros::NodeHandle node; frame_boundary::Publisher publisher(node);
  Inbox<ArrayV2> local,map;
  auto a=node.subscribe<ArrayV2>(kLocalV2,10,boost::ref(local));
  auto b=node.subscribe<ArrayV2>(kMapV2,10,boost::ref(map));
  ros::WallDuration(.5).sleep();
  EXPECT_EQ(a.getNumPublishers(),0U); EXPECT_EQ(b.getNumPublishers(),0U);
  output_gate::Inputs in; in.common_geometry=false; in.pose_ready=true; in.lidar_processed=true;
  const auto decision=output_gate::Decide(in);
  LiDAR empty;
  const auto effective=publisher.PublishV2(empty,ros::Time(5.),decision);
  EXPECT_EQ(effective.v2,Mode::kNone); EXPECT_EQ(effective.reason,"legacy_mode");
  // Pre-B0 production rule for a motion-ready frame. (Warm-up frames with common geometry OFF
  // are a known B1 gap and deliberately not asserted.)
  EXPECT_TRUE(output_gate::PlanLegacy(effective,in,true).publish);
  EXPECT_FALSE(output_gate::PlanLegacy(effective,in,true).suppress_markers);
  ros::WallDuration(.2).sleep();
  EXPECT_EQ(local.size(),0U); EXPECT_EQ(map.size(),0U);
}

// Every locally valid scan has local V2; map V2 is additional when the map pose is valid.
TEST(FrameBoundaryV2, LocalAlwaysAndMapOnlyWithValidPoseAndV1Unchanged) {
  Configure(); ros::Time::setNow(ros::Time(40.));
  ros::NodeHandle node; frame_boundary::Publisher publisher(node);
  Inbox<ArrayV1> v1_local,v1_map; Inbox<ArrayV2> v2_local,v2_map;
  auto a=node.subscribe<ArrayV1>(kLocalV1,10,boost::ref(v1_local));
  auto b=node.subscribe<ArrayV1>(kMapV1,10,boost::ref(v1_map));
  auto c=node.subscribe<ArrayV2>(kLocalV2,10,boost::ref(v2_local));
  auto d=node.subscribe<ArrayV2>(kMapV2,10,boost::ref(v2_map));
  ASSERT_TRUE(Wait([&]{return a.getNumPublishers()&&b.getNumPublishers()&&c.getNumPublishers()&&d.getNumPublishers();}));
  ros::WallDuration(.2).sleep();
  auto object=Detection(7,10,0); object.s32_MotionState=static_cast<int>(MotionState::Dynamic);
  const auto s1=Step(publisher,40.0,MapPose(40.0,true,"gps_observed"),{object},1);          // map
  const auto s2=Step(publisher,40.1,MapPose(40.1,false,"gps_invalid"),{object},2);          // GPS denied
  const auto s3=Step(publisher,40.2,{},{},3,false,"imu_input_invalid");                   // IMU invalid
  const auto s4=Step(publisher,40.3,MapPose(40.3,false,"map_speed_invalid"),{object},4);    // CAN > limit
  const auto s5=Step(publisher,40.4,MapPose(40.4,false,"gps_recovering"),{object},5);       // recovering
  const auto s6=Step(publisher,40.5,MapPose(40.5,true,"gps_bounded_prediction"),{object},6); // recovered
  EXPECT_EQ(s1.decision.v2,Mode::kBoth); EXPECT_EQ(s2.decision.v2,Mode::kLocal);
  EXPECT_EQ(s3.decision.v2,Mode::kNone); EXPECT_EQ(s3.decision.reason,"local_input_invalid:imu_input_invalid");
  EXPECT_EQ(s4.decision.v2,Mode::kNone); EXPECT_EQ(s4.decision.reason,"map_pose_invalid:map_speed_invalid");
  EXPECT_EQ(s5.decision.v2,Mode::kLocal); EXPECT_EQ(s6.decision.v2,Mode::kBoth);
  ASSERT_TRUE(Wait([&]{return v1_local.size()==6 && v1_map.size()==6 && v2_local.size()==4 && v2_map.size()==2;}));
  ros::WallDuration(.5).sleep();                                         // nothing late may arrive
  ASSERT_EQ(v2_local.size(),4U); ASSERT_EQ(v2_map.size(),2U);
  // Map-valid scans appear on both; GPS-denied scans appear only on local; invalid scans are silent.
  EXPECT_EQ(v2_map.at(0).header.stamp,ros::Time(40.0)); EXPECT_EQ(v2_map.at(1).header.stamp,ros::Time(40.5));
  EXPECT_EQ(v2_local.at(0).header.stamp,ros::Time(40.0)); EXPECT_EQ(v2_local.at(1).header.stamp,ros::Time(40.1));
  EXPECT_EQ(v2_local.at(2).header.stamp,ros::Time(40.4)); EXPECT_EQ(v2_local.at(3).header.stamp,ros::Time(40.5));
  for(std::size_t i=0;i<2;++i) {
    EXPECT_EQ(v2_map.at(i).header.frame_id,"map"); EXPECT_EQ(v2_local.at(i).header.frame_id,"base_link");
    EXPECT_EQ(int(v2_map.at(i).status),int(ArrayV2::OBSERVED)); EXPECT_EQ(int(v2_local.at(i).status),int(ArrayV2::OBSERVED));
  }
  // V1 compatibility topics keep their TARGET meaning in the same scans.
  EXPECT_EQ(int(v1_map.at(1).status),int(ArrayV1::POSE_UNAVAILABLE));
  EXPECT_EQ(int(v1_local.at(2).status),int(ArrayV1::INPUT_INVALID)); EXPECT_EQ(v1_local.at(2).reason,"imu_input_invalid");
  EXPECT_EQ(int(v1_map.at(2).status),int(ArrayV1::INPUT_INVALID));
  EXPECT_EQ(int(v1_map.at(3).status),int(ArrayV1::POSE_UNAVAILABLE));
  // V2 objects are lossless copies of the V1 objects of the same scan and frame.
  ExpectSameV1Fields(v2_map.at(0).objects.at(0),v1_map.at(0).objects.at(0));
  ExpectSameV1Fields(v2_local.at(1).objects.at(0),v1_local.at(1).objects.at(0));
  const auto first_map=v2_map.at(0);                // Inbox::at returns a copy
  const auto& o=first_map.objects.at(0);
  EXPECT_EQ(o.track_id,7); EXPECT_TRUE(o.motion_valid); EXPECT_EQ(int(o.motion_state),int(o.MOTION_DYNAMIC));
  EXPECT_TRUE(o.is_dynamic);
}

// Per-object invalid geometry: the whole V2 frame is silent (no INPUT_INVALID array, no partial frame).
TEST(FrameBoundaryV2, InvalidGeometryPublishesNothingOnV2) {
  Configure();
  ros::NodeHandle node; frame_boundary::Publisher publisher(node);
  Inbox<ArrayV2> local,map;
  auto a=node.subscribe<ArrayV2>(kLocalV2,10,boost::ref(local));
  auto b=node.subscribe<ArrayV2>(kMapV2,10,boost::ref(map));
  ASSERT_TRUE(Wait([&]{return a.getNumPublishers()&&b.getNumPublishers();}));
  ros::WallDuration(.2).sleep();
  auto bad=Detection(2,12,0); bad.f32_OBB_Width=NAN;
  const auto s1=Step(publisher,41.0,MapPose(41.0,true,"gps_observed"),{Detection(1,10,0),bad},1);
  const auto s2=Step(publisher,41.1,MapPose(41.1,false,"gps_stale"),{Detection(1,10,0),bad},2);
  EXPECT_EQ(s1.decision.v2,Mode::kNone); EXPECT_EQ(s1.decision.reason,"invalid_object_dimensions");
  EXPECT_EQ(s2.decision.v2,Mode::kNone); EXPECT_EQ(s2.decision.reason,"invalid_object_dimensions");
  const auto gate=frame_boundary::GateInputs(true,"local_ready",true,s1.lidar,1);
  EXPECT_FALSE(output_gate::PlanLegacy(s1.decision,gate,true).publish);
  EXPECT_TRUE(output_gate::PlanLegacy(s1.decision,gate,true).suppress_markers);
  // A valid scan afterwards is published normally (the rejection did not consume the V2 stamp).
  const auto s3=Step(publisher,41.2,MapPose(41.2,true,"gps_observed"),{Detection(1,10,0)},3);
  EXPECT_EQ(s3.decision.v2,Mode::kBoth);
  ASSERT_TRUE(Wait([&]{return map.size()==1 && local.size()==1;}));
  ros::WallDuration(.4).sleep();
  EXPECT_EQ(map.size(),1U); EXPECT_EQ(local.size(),1U);
  EXPECT_EQ(map.at(0).header.stamp,ros::Time(41.2));
}

// Warm-up objects reach map_v2 as UNKNOWN/invalid; legacy stays withheld frame-wide.
TEST(FrameBoundaryV2, WarmupObjectIsCarriedOnMapV2AsUnknown) {
  Configure();
  ros::NodeHandle node; frame_boundary::Publisher publisher(node);
  Inbox<ArrayV2> map; auto a=node.subscribe<ArrayV2>(kMapV2,10,boost::ref(map));
  ASSERT_TRUE(Wait([&]{return a.getNumPublishers();}));
  ros::WallDuration(.2).sleep();
  auto ready=Detection(1,10,0);
  auto warm=Detection(2,14,0); warm.motion_valid=false; warm.s32_MotionState=0;
  auto stale=Detection(3,18,0); stale.motion_valid=false; stale.s32_MotionState=static_cast<int>(MotionState::Dynamic);
  const auto s=Step(publisher,42.0,MapPose(42.0,true,"gps_observed"),{ready,warm,stale},1);
  EXPECT_EQ(s.decision.v2,Mode::kBoth);
  const auto gate=frame_boundary::GateInputs(true,"local_ready",true,s.lidar,1);
  const auto plan=output_gate::PlanLegacy(s.decision,gate,frame_boundary::LegacyMotionReady(s.lidar));
  EXPECT_FALSE(plan.publish); EXPECT_TRUE(plan.suppress_markers); EXPECT_EQ(plan.reason,"motion_state_unrepresentable");
  ASSERT_TRUE(Wait([&]{return map.size()==1;}));
  const auto array=map.at(0);
  ASSERT_EQ(array.objects.size(),3U);
  EXPECT_TRUE(array.objects[0].motion_valid); EXPECT_EQ(int(array.objects[0].motion_state),int(array.objects[0].MOTION_LOW_THREAT));
  for(unsigned i:{1U,2U}) {
    EXPECT_FALSE(array.objects[i].motion_valid);
    EXPECT_EQ(int(array.objects[i].motion_state),int(array.objects[i].MOTION_UNKNOWN));
    EXPECT_FALSE(array.objects[i].absolute_velocity_valid);   // no speed-0 static claim
    EXPECT_FALSE(array.objects[i].is_dynamic);
  }
}

// V2 has its own stamp guard (V1 Accept already consumed the stamp); duplicates and context
// mismatches publish nothing.
TEST(FrameBoundaryV2, V2StampGuardIsIndependentOfV1) {
  Configure();
  ros::NodeHandle node; frame_boundary::Publisher publisher(node);
  Inbox<ArrayV2> local,map;
  auto a=node.subscribe<ArrayV2>(kLocalV2,10,boost::ref(local));
  auto b=node.subscribe<ArrayV2>(kMapV2,10,boost::ref(map));
  ASSERT_TRUE(Wait([&]{return a.getNumPublishers()&&b.getNumPublishers();}));
  ros::WallDuration(.2).sleep();
  const auto s=Step(publisher,43.0,MapPose(43.0,true,"gps_observed"),{Detection(1,10,0)},1);   // V1 consumed 43.0
  EXPECT_EQ(s.decision.v2,Mode::kBoth);
  const auto again=publisher.PublishV2(s.lidar,s.stamp,output_gate::Decide(
      frame_boundary::GateInputs(true,"local_ready",true,s.lidar,1)));
  EXPECT_EQ(again.v2,Mode::kNone); EXPECT_EQ(again.reason,"v2_stamp_rejected");
  const auto older=publisher.PublishV2(s.lidar,ros::Time(42.9),s.decision);
  EXPECT_EQ(older.v2,Mode::kNone); EXPECT_EQ(older.reason,"v2_stamp_rejected");
  const auto zero=publisher.PublishV2(s.lidar,ros::Time(),s.decision);
  EXPECT_EQ(zero.v2,Mode::kNone);
  const auto mismatch=publisher.PublishV2(s.lidar,ros::Time(43.1),s.decision);   // context is 43.0
  EXPECT_EQ(mismatch.v2,Mode::kNone); EXPECT_EQ(mismatch.reason,"context_stamp_mismatch");
  ASSERT_TRUE(Wait([&]{return map.size()==1;}));
  ros::WallDuration(.4).sleep();
  EXPECT_EQ(map.size(),1U); EXPECT_EQ(local.size(),1U);
}

// local_v2/map_v2 selection follows pose validity only, never subscriber demand.
TEST(FrameBoundaryV2, SelectionIgnoresSubscriberDemand) {
  Configure();
  ros::NodeHandle node; frame_boundary::Publisher publisher(node);
  Inbox<ArrayV2> map; auto b=node.subscribe<ArrayV2>(kMapV2,10,boost::ref(map));   // no V1, no local_v2
  ASSERT_TRUE(Wait([&]{return b.getNumPublishers();}));
  ros::WallDuration(.2).sleep();
  const auto s1=Step(publisher,44.0,MapPose(44.0,true,"gps_observed"),{Detection(1,10,0)},1);
  EXPECT_EQ(s1.decision.v2,Mode::kBoth);
  ASSERT_TRUE(Wait([&]{return map.size()==1;}));
  ASSERT_EQ(map.at(0).objects.size(),1U);                    // map geometry without any V1 subscriber
  EXPECT_NEAR(map.at(0).objects[0].center.x,
      (s1.lidar.frame_context.map_pose.map_from_base*(*asmc::lidar::base_from_input(s1.lidar.frame_context))*
       asmc::lidar::Vec(10,0,-.2)).x(),1e-5);
  // GPS denied with nobody on local_v2: still local, and map_v2 receives nothing.
  const auto s2=Step(publisher,44.1,MapPose(44.1,false,"gps_missing"),{Detection(1,10,0)},2);
  EXPECT_EQ(s2.decision.v2,Mode::kLocal);
  b.shutdown();
  Inbox<ArrayV2> local; auto a=node.subscribe<ArrayV2>(kLocalV2,10,boost::ref(local));   // only local_v2
  ASSERT_TRUE(Wait([&]{return a.getNumPublishers();}));
  ros::WallDuration(.2).sleep();
  const auto s3=Step(publisher,44.2,MapPose(44.2,true,"gps_observed"),{Detection(1,10,0)},3);
  EXPECT_EQ(s3.decision.v2,Mode::kBoth);                      // local publishes even without map subscribers
  ros::WallDuration(.4).sleep();
  EXPECT_EQ(map.size(),1U); EXPECT_EQ(local.size(),1U);
}

// Valid inputs and zero objects: one OBSERVED_EMPTY on the active topic only, never a dummy.
TEST(FrameBoundaryV2, EmptySceneIsObservedEmptyOnActiveTopicOnly) {
  Configure();
  ros::NodeHandle node; frame_boundary::Publisher publisher(node);
  Inbox<ArrayV2> local,map;
  auto a=node.subscribe<ArrayV2>(kLocalV2,10,boost::ref(local));
  auto b=node.subscribe<ArrayV2>(kMapV2,10,boost::ref(map));
  ASSERT_TRUE(Wait([&]{return a.getNumPublishers()&&b.getNumPublishers();}));
  ros::WallDuration(.2).sleep();
  auto unassigned=Detection(-1,10,0);
  Step(publisher,45.0,MapPose(45.0,true,"gps_observed"),{},1);
  Step(publisher,45.1,MapPose(45.1,false,"gps_stale"),{unassigned},2);
  ASSERT_TRUE(Wait([&]{return map.size()==1 && local.size()==2;}));
  ros::WallDuration(.4).sleep();
  ASSERT_EQ(map.size(),1U); ASSERT_EQ(local.size(),2U);
  EXPECT_EQ(int(map.at(0).status),int(ArrayV2::OBSERVED_EMPTY)); EXPECT_TRUE(map.at(0).objects.empty());
  EXPECT_EQ(int(local.at(0).status),int(ArrayV2::OBSERVED_EMPTY)); EXPECT_TRUE(local.at(0).objects.empty());
  EXPECT_EQ(int(local.at(1).status),int(ArrayV2::OBSERVED_EMPTY)); EXPECT_TRUE(local.at(1).objects.empty());
  EXPECT_EQ(map.at(0).reason,"observed_empty"); EXPECT_EQ(local.at(0).reason,"observed_empty");
}

// The reason for a silent scan is visible in diagnostics; counters and totals add up.
TEST(FrameBoundaryV2, DiagnosticsReportReasonCountersAndLevelingCount) {
  Configure();
  ros::NodeHandle node; frame_boundary::Publisher publisher(node);
  Inbox<diagnostic_msgs::DiagnosticArray> diagnostics;
  auto a=node.subscribe<diagnostic_msgs::DiagnosticArray>("/perception/lidar/diagnostics",10,boost::ref(diagnostics));
  ASSERT_TRUE(Wait([&]{return a.getNumPublishers();}));
  Step(publisher,46.0,MapPose(46.0,true,"gps_observed"),{Detection(1,10,0)},1);                // map
  const auto denied=Step(publisher,46.1,MapPose(46.1,false,"gps_invalid"),{Detection(1,10,0)},2); // local
  Step(publisher,46.2,{},{},3,false,"speed_input_invalid");                                // none
  publisher.PublishV2(denied.lidar,denied.stamp,denied.decision);                           // stamp rejected
  // The 1 Hz report that follows the last call (counters only grow; stamp_errors=1 is final).
  std::map<std::string,std::string> values;
  ASSERT_TRUE(Wait([&]{
    if(!diagnostics.size()) return false;
    values=Values(diagnostics.at(diagnostics.size()-1));
    return values.count("v2_stamp_errors") && values["v2_stamp_errors"]=="1";
  },3.5));
  EXPECT_EQ(values.at("v2_output"),"none"); EXPECT_EQ(values.at("v2_reason"),"v2_stamp_rejected");
  EXPECT_EQ(values.at("gps_state"),"denied");
  EXPECT_EQ(values.at("v2_map_frames"),"1"); EXPECT_EQ(values.at("v2_local_frames"),"2");
  EXPECT_EQ(values.at("v2_none_frames"),"2"); EXPECT_EQ(values.at("v2_stamp_errors"),"1");
  EXPECT_EQ(values.at("leveling_store_count"),"1");
  EXPECT_EQ(values.at("leveling_source"),"Main::CorrectingPoints");
  EXPECT_EQ(values.at("legacy_output"),"waiting_for_input");   // SetLegacyReason is main.cpp's job
  EXPECT_EQ(values.count("v2_local_subscribers"),1U); EXPECT_EQ(values.count("v2_map_subscribers"),1U);
  testing::internal::CaptureStdout();
  publisher.ReportTotals();
  const std::string totals=testing::internal::GetCapturedStdout();
  EXPECT_NE(totals.find("[LIDAR_FRAME_TOTAL] processed=2 invalid=1 stamp_errors=0 source_stamp_ns="),std::string::npos)
      << totals;
  EXPECT_NE(totals.find("[LIDAR_V2_TOTAL] local=2 map=1 none=2 stamp_errors=1"),std::string::npos) << totals;
}
}  // namespace
int main(int argc,char** argv) {
  ros::init(argc,argv,"frame_boundary_v2_ros");testing::InitGoogleTest(&argc,argv);
  ros::AsyncSpinner spinner(1);spinner.start();
  const int result=RUN_ALL_TESTS();spinner.stop();return result;
}
