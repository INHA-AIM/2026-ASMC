#include <gtest/gtest.h>
#include <Geometry/FrameBoundaryPublisher.hpp>
#include <Parser/SensorPoseHistory.hpp>
#include <Parser/Parser.hpp>
#include <Camera/Camera.hpp>
#include <Lidar/Tracking/tracking_utils.hpp>
#include <ros/package.h>
#include <mutex>
#include <functional>
#include <limits>

extern VehicleSpec st_VehicleSpec;
namespace {
void Configure() {
  ros::NodeHandle node("~"); node.setParam("enable_common_geometry",true); node.setParam("lidar_level_roll",true);
  st_VehicleSpec.st_LiDARPos={.8f,0,1.45f}; st_VehicleSpec.st_LiDARRotationDeg={0,0,0};
  st_VehicleSpec.st_GPSPos={.35f,0,1.25f};
  CAMERA camera{};
  InitProjectionMatrix(camera,YAML::LoadFile(ros::package::getPath("Main")+"/setting.yaml"));
  frame_boundary::Configure(node,st_VehicleSpec,camera);
}
void Now(double time){ros::Time::setNow(ros::Time(time));}
GPS_IMU Local(double yaw=0.) {
  GPS_IMU out{}; out.st_IMU.b_ImuReady=true;
  out.st_IMU.f32_Yaw_rad=yaw; out.st_IMU.f32_Yaw_deg=yaw*180/M_PI;
  out.st_CAN.f32_CurrentVel_MPS=2.; out.st_CAN.f32_CurrentVel_KPH=7.2;
  return out;
}
LiDARObjectInfo Detection(double x,double y) {
  LiDARObjectInfo out{}; out.s32_Id=-1; out.str_ObjectClass="car"; out.str_YoloClass="car";
  out.f32_CoreX=out.f32_OBB_CenterX=x; out.f32_CoreY=out.f32_OBB_CenterY=y;
  out.f32_CoreZ=out.f32_OBB_CenterZ=-.2;
  out.f32_OBB_Width=1.; out.f32_OBB_Height=.8; out.f32_OBB_ZMin=-.5; out.f32_OBB_ZMax=.5;
  out.f32_MinX=x-.5;out.f32_MaxX=x+.5;out.f32_MinY=y-.4;out.f32_MaxY=y+.4;
  for(unsigned i=0;i<8;++i) {
    out.OBB_Corners_Body[i]={float(x+((i&1)? .5:-.5)),float(y+((i&2)? .4:-.4)),float((i&4)? .5:-.5)};
    pcl::PointXYZI p; p.x=out.OBB_Corners_Body[i].x; p.y=out.OBB_Corners_Body[i].y; p.z=out.OBB_Corners_Body[i].z;
    p.intensity=1; out.pcl_ClusterPoints.push_back(p);
  }
  return out;
}
void TrackingConfig() {
  st_Tracking.s32_NewTrackId=100; st_Tracking.b_UseMapAssociation=true;
  st_Tracking.f32_COST_W_DIST=1;st_Tracking.f32_COST_W_IOU=1;
  st_Tracking.f32_COST_THRESHOLD=3;st_Tracking.f32_COST_INF=9999;
  st_Tracking.f32_ContinuityBonus=.2;st_Tracking.f32_MergedTrackRetireDistance=.5;
  st_Tracking.f32_DuplicateIouThreshold=.35;
}
TEST(FrameBoundary, LocalReadinessDoesNotRequireGps) {
  Configure(); Now(20.);
  SensorPoseHistory history; const auto motion=Local();
  history.AddImu(20000,motion.st_IMU); history.AddCan(20000,motion.st_CAN);
  GPS_IMU resolved{}; std::string reason;
  ASSERT_TRUE(history.WaitForLocalLookup(20000,0,&resolved,&reason));
  EXPECT_EQ(reason,"local_ready"); EXPECT_FALSE(history.Lookup(20000,&resolved,&reason));
  EXPECT_EQ(history.MapAt(20000,resolved).reason,"gps_missing");
  EXPECT_FALSE(history.WaitForLocalLookup(20121,0,&resolved,&reason));
  EXPECT_TRUE(history.WaitForLocalLookup(20120,0,&resolved,&reason));
}
TEST(FrameBoundary, NewInvalidLocalSampleBlocksCachedGoodSample) {
  Configure();Now(20.);SensorPoseHistory history;auto motion=Local();
  history.AddImu(20000,motion.st_IMU);history.AddCan(20000,motion.st_CAN);
  auto invalid=motion.st_IMU;invalid.b_ImuReady=false;history.AddImu(20010,invalid);
  GPS_IMU resolved{};std::string reason;
  EXPECT_FALSE(history.WaitForLocalLookup(20020,0,&resolved,&reason));EXPECT_EQ(reason,"imu_input_invalid");
  history.AddImu(20021,motion.st_IMU);
  auto bad_speed=motion.st_CAN;bad_speed.f32_CurrentVel_MPS=NAN;history.AddCan(20022,bad_speed);
  EXPECT_FALSE(history.WaitForLocalLookup(20023,0,&resolved,&reason));EXPECT_EQ(reason,"speed_input_invalid");
  history.AddCan(20024,motion.st_CAN);
  EXPECT_TRUE(history.WaitForLocalLookup(20025,0,&resolved,&reason));
}
TEST(FrameBoundary, MapInvalidBarrierRecoveryAndSilence) {
  Configure(); SensorPoseHistory history; auto motion=Local();
  GPS_DATA gps{}; gps.f64_E=100;gps.f64_N=200;gps.f64_U=29.9;
  for(unsigned t:{20000,20200,20400}) {Now(t*.001);history.AddImu(t,motion.st_IMU);history.AddGps(t,gps);}
  Now(20.45); const auto valid=history.MapAt(20450,motion);
  ASSERT_TRUE(valid.valid); EXPECT_EQ(valid.reason,"gps_bounded_prediction");
  EXPECT_NEAR(valid.map_from_base.translation().z(),28.65,1e-6);
  Now(20.7); EXPECT_TRUE(history.MapAt(20700,motion).valid);
  Now(20.701); EXPECT_EQ(history.MapAt(20701,motion).reason,"gps_stale");
  Now(20.5); history.AddGpsInvalid(20500,20500);
  EXPECT_EQ(history.MapAt(20500,motion).reason,"gps_invalid");
  for(unsigned t:{20700,20900,21100}) {
    Now(t*.001);history.AddImu(t,motion.st_IMU);history.AddGps(t,gps);
    EXPECT_EQ(history.MapAt(t,motion).valid,t==21100);
  }
  Now(21.2);history.AddGpsInvalid(20800,21200);
  EXPECT_TRUE(history.MapAt(21200,motion).valid);
}
TEST(FrameBoundary, LocalAssociationIgnoresMapCoordinatesIncludingNaN) {
  Configure(); TrackingConfig();
  auto previous=Detection(10,1);previous.s32_Id=42;previous.s32_Age=20;
  previous.b_KfInitialized=true;previous.s32_KfInitCount=2;
  previous.Eigen_KfState_CA<<10,1,-2,0,0,0;
  previous.f32_CoreE=100;previous.f32_CoreN=200;
  std::vector<int> ids;
  for(double map_x:{100.,1000000.,std::numeric_limits<double>::quiet_NaN()}) {
    auto current=Detection(9.8,1);current.f32_CoreE=map_x;current.f32_CoreN=map_x;
    std::deque<std::vector<LiDARObjectInfo>> past{{previous}};
    std::vector<LiDARObjectInfo> detections{current};
    CompareCentroids(past,detections,.1,Local());
    ASSERT_FALSE(detections.empty());ids.push_back(detections.front().s32_Id);
    EXPECT_EQ(detections.front().str_YoloClass,"car");
  }
  EXPECT_EQ(ids,std::vector<int>({42,42,42}));
}
TEST(FrameBoundary, LocalAssociationYawPredictionUsesCurrentBodyFrame) {
  Configure();TrackingConfig();
  auto previous=Detection(10,0);previous.s32_Id=42;previous.s32_Age=20;
  previous.b_KfInitialized=true;previous.s32_KfInitCount=2;
  previous.Eigen_KfState_CA<<10,0,0,0,0,0;
  previous.f32_CoreE=NAN;previous.f32_CoreN=NAN;
  auto current=Detection(10*std::cos(.1),-10*std::sin(.1));
  current.f32_CoreE=NAN;current.f32_CoreN=NAN;
  std::deque<std::vector<LiDARObjectInfo>> past{{previous}};
  std::vector<LiDARObjectInfo> detections{current};
  CompareCentroids(past,detections,.1,Local(.1));
  ASSERT_FALSE(detections.empty());EXPECT_EQ(detections.front().s32_Id,42);
}
TEST(FrameBoundary, TrackingReexpressesRollPitchAndNonzeroMountWithoutEgoDoubleCompensation) {
  Configure();TrackingConfig();Now(25.);
  VehicleSpec vehicle=st_VehicleSpec;
  vehicle.st_LiDARRotationDeg={4.f,-3.f,8.f};
  const auto mount=frame_boundary::SensorMount(vehicle);
  const asmc::lidar::Vec fixed_in_world(12,2,1);
  auto local=Local(.2);local.st_IMU.f32_Roll_rad=.1;local.st_IMU.f32_Pitch_rad=-.08;
  asmc::lidar::PoseSelection unavailable;unavailable.reason="gps_missing";
  frame_boundary::PrepareScan(ros::Time(25.),local,vehicle,unavailable);
  const auto old_level=asmc::lidar::leveling_rotation(.1,-.08,mount.rotation());
  frame_boundary::StoreLeveling(old_level);
  const auto old_point=(old_level*mount.inverse()*(frame_boundary::MapRotation(local).transpose()*fixed_in_world)).eval();
  auto previous=Detection(old_point.x(),old_point.y());previous.f32_CoreZ=old_point.z();
  previous.s32_Id=42;previous.b_KfInitialized=true;previous.s32_KfInitCount=2;
  previous.Eigen_KfState_CA<<old_point.x(),old_point.y(),0,0,0,0;
  previous.f32_PrevYaw=.2;frame_boundary::RecordTrackingFrame(previous);
  local=Local(.28);local.st_IMU.f32_Roll_rad=-.12;local.st_IMU.f32_Pitch_rad=.06;
  frame_boundary::PrepareScan(ros::Time(25.1),local,vehicle,unavailable);
  const auto new_level=asmc::lidar::leveling_rotation(-.12,.06,mount.rotation());
  frame_boundary::StoreLeveling(new_level);
  const auto new_point=(new_level*mount.inverse()*(frame_boundary::MapRotation(local).transpose()*fixed_in_world)).eval();
  auto aligned=previous;frame_boundary::AlignTrackingFrame(aligned,.28);
  EXPECT_NEAR(aligned.Eigen_KfState_CA(0),new_point.x(),1e-5);
  EXPECT_NEAR(aligned.Eigen_KfState_CA(1),new_point.y(),1e-5);
  EXPECT_NEAR(aligned.tracking_height_input,new_point.z(),1e-5);
  // CAN=2m/s does not subtract another .2m from the relative CA state.
  auto current=Detection(new_point.x(),new_point.y());current.f32_CoreZ=new_point.z();
  current.f32_CoreE=NAN;current.f32_CoreN=NAN;
  std::deque<std::vector<LiDARObjectInfo>> past{{previous}};
  std::vector<LiDARObjectInfo> detections{current};
  CompareCentroids(past,detections,.1,local);
  ASSERT_FALSE(detections.empty());EXPECT_EQ(detections.front().s32_Id,42);
  EXPECT_NEAR(detections.front().f32_CoreX,new_point.x(),1e-5);
}
TEST(FrameBoundary, CalibrationRejectsRuntimeAndSnapshotMismatch) {
  Configure();EXPECT_TRUE(frame_boundary::MapOffsetsValid(302595.,4124145.));
  EXPECT_FALSE(frame_boundary::MapOffsetsValid(302595.1,4124145.));
  ros::NodeHandle node("~");CAMERA camera{};
  InitProjectionMatrix(camera,YAML::LoadFile(ros::package::getPath("Main")+"/setting.yaml"));
  auto wrong=st_VehicleSpec;wrong.st_LiDARPos.f32_X+=.1;
  EXPECT_THROW(frame_boundary::Configure(node,wrong,camera),std::runtime_error);
  camera.params.intrinsic_matrix_90_fov(0,0)+=1.;
  EXPECT_THROW(frame_boundary::Configure(node,st_VehicleSpec,camera),std::runtime_error);
  Configure();
}
TEST(FrameBoundary, ContinuousLocalTrackingStateIsIndependentOfGpsHistory) {
  Configure();TrackingConfig();Now(26.);
  auto simulate=[](bool denial) {
    st_Tracking.s32_NewTrackId=100;
    std::deque<std::vector<LiDARObjectInfo>> history;
    std::vector<std::vector<double>> states;
    asmc::lidar::PoseSelection map;
    for(unsigned frame=0;frame<40;++frame) {
      const double time=26.+frame*.1;Now(time);
      auto local=Local(.002*frame);
      local.st_IMU.f32_Roll_rad=.03*std::sin(frame*.1);
      local.st_IMU.f32_Pitch_rad=.02*std::cos(frame*.1);
      map.valid=!denial || frame<10 || frame>=30;
      map.reason=map.valid?"gps_bounded_prediction":"gps_stale";
      map.query_stamp_ns=ros::Time(time).toNSec();map.clock="ros";
      map.map_from_base=asmc::lidar::Pose::Identity();map.map_from_base.translation().x()=frame*.2;
      frame_boundary::PrepareScan(ros::Time(time),local,st_VehicleSpec,map,frame+1);
      frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(local.st_IMU.f32_Roll_rad,
          local.st_IMU.f32_Pitch_rad,asmc::lidar::Rot::Identity()));
      auto detection=Detection(12.-.2*frame,1.);
      detection.f32_CoreE=map.valid?100:NAN;detection.f32_CoreN=map.valid?200:NAN;
      std::vector<LiDARObjectInfo> objects{detection};
      if(history.empty()) {
        objects[0].s32_Id=42;objects[0].s32_Age=1;objects[0].b_KfInitialized=true;
        objects[0].s32_KfInitCount=2;objects[0].Eigen_KfState_CA<<12,1,-2,0,0,0;
        objects[0].f32_PrevYaw=0;
      } else CompareCentroids(history,objects,.1,local);
      EXPECT_EQ(objects.front().s32_Id,42);frame_boundary::RecordTrackingFrame(objects.front());
      const auto& o=objects.front();
      states.push_back({double(o.s32_Id),double(o.s32_Age),double(o.s32_MissCount),
          double(o.Eigen_KfState_CA(0)),double(o.Eigen_KfState_CA(1)),double(o.continuity_counter)});
      history.clear();history.push_back(objects);
    }
    return states;
  };
  const auto normal=simulate(false), denied=simulate(true);EXPECT_EQ(normal,denied);
}

TEST(FrameBoundary, ParserRejectsZeroDuplicateAndBackwardStampsBeforeTrackerInput) {
  Configure();Now(45.);
  const auto old_stamp=st_SensorData.st_RawLiDAR.u64_Timestamp;
  const auto old_count=st_SensorData.st_RawLiDAR.s32_Num;
  const auto old_generation=u64_LidarFrameGeneration;
  pthread_mutex_init(&st_SensorData.st_MutexLiDAR,nullptr);
  st_SensorData.st_RawLiDAR.u64_Timestamp=0;u64_LidarFrameGeneration=0;
  pcl::PointCloud<pcl::PointXYZI> cloud;cloud.push_back(pcl::PointXYZI());
  sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud,*message);message->header.frame_id="velodyne";
  LiDARParser_Callback(message);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.u64_Timestamp,0u);
  EXPECT_EQ(u64_LidarFrameGeneration,0u);
  message->header.stamp=ros::Time(45.);LiDARParser_Callback(message);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.u64_Timestamp,45000u);
  EXPECT_EQ(u64_LidarFrameGeneration,1u);
  message->header.stamp=ros::Time(44.9);LiDARParser_Callback(message);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.u64_Timestamp,45000u);
  message->header.stamp=ros::Time(45.);LiDARParser_Callback(message);
  EXPECT_EQ(u64_LidarFrameGeneration,1u);
  message->header.stamp=ros::Time(45.1);LiDARParser_Callback(message);
  EXPECT_EQ(st_SensorData.st_RawLiDAR.u64_Timestamp,45100u);
  EXPECT_EQ(u64_LidarFrameGeneration,2u);
  st_SensorData.st_RawLiDAR.u64_Timestamp=old_stamp;
  st_SensorData.st_RawLiDAR.s32_Num=old_count;u64_LidarFrameGeneration=old_generation;
  pthread_mutex_destroy(&st_SensorData.st_MutexLiDAR);
}

bool Wait(const std::function<bool()>& condition) {
  const auto end=ros::WallTime::now()+ros::WallDuration(3.);
  while(ros::ok() && ros::WallTime::now()<end) {if(condition()) return true;ros::WallDuration(.01).sleep();}
  return condition();
}
TEST(FrameBoundary, TypedOutputsPreserveGeometryAndDistinguishInvalid) {
  Configure();Now(30.);
  ros::NodeHandle node;frame_boundary::Publisher publisher(node);
  std::mutex mutex;std::vector<asmc_msgs::LidarObjectArray> local,map;
  auto sub_local=node.subscribe<asmc_msgs::LidarObjectArray>("/perception/lidar/objects_local",10,
      [&](const asmc_msgs::LidarObjectArray::ConstPtr& m){std::lock_guard<std::mutex> lock(mutex);local.push_back(*m);});
  auto sub_map=node.subscribe<asmc_msgs::LidarObjectArray>("/perception/lidar/objects_map",10,
      [&](const asmc_msgs::LidarObjectArray::ConstPtr& m){std::lock_guard<std::mutex> lock(mutex);map.push_back(*m);});
  ASSERT_TRUE(Wait([&]{return sub_local.getNumPublishers() && sub_map.getNumPublishers();}));
  ros::WallDuration(.1).sleep();
  asmc::lidar::PoseSelection missing;missing.reason="gps_invalid";
  frame_boundary::PrepareScan(ros::Time(30.),Local(),st_VehicleSpec,missing);
  frame_boundary::StoreLeveling(asmc::lidar::Rot::Identity());
  LiDAR lidar;lidar.frame_context=frame_boundary::Current();
  auto object=Detection(10,0);object.s32_Id=42;object.s32_Age=2;lidar.vec_Objects={object};
  EXPECT_FALSE(frame_boundary::LegacyMotionReady(lidar));
  lidar.vec_Objects[0].motion_valid=true;EXPECT_TRUE(frame_boundary::LegacyMotionReady(lidar));
  publisher.Publish(lidar);
  ASSERT_TRUE(Wait([&]{std::lock_guard<std::mutex> lock(mutex);return local.size()==1 && map.size()==1;}));
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(local[0].objects.size(),1U);EXPECT_EQ(local[0].header.frame_id,"base_link");
    EXPECT_NEAR(local[0].objects[0].center.x,10.8,1e-6);EXPECT_EQ(local[0].objects[0].track_id,42);
    EXPECT_EQ(local[0].objects[0].camera_class,"car");EXPECT_FALSE(local[0].objects[0].absolute_velocity_valid);
    EXPECT_EQ(map[0].status,map[0].POSE_UNAVAILABLE);EXPECT_TRUE(map[0].objects.empty());
  }
  Now(30.1);lidar.frame_context.time={30100000000LL,30100000000LL,30100000000LL,"ros","bridge_receive_ms"};
  lidar.vec_Objects.clear();publisher.Publish(lidar);
  ASSERT_TRUE(Wait([&]{std::lock_guard<std::mutex> lock(mutex);return local.size()==2;}));
  {
    std::lock_guard<std::mutex> lock(mutex);EXPECT_EQ(local[1].status,local[1].OBSERVED_EMPTY);
  }
  publisher.Invalid(ros::Time(30.2),"imu_stale");publisher.Invalid(ros::Time(30.2),"duplicate");
  publisher.Invalid(ros::Time(),"initial_no_input");publisher.Invalid(ros::Time(30.1),"regression");
  ASSERT_TRUE(Wait([&]{std::lock_guard<std::mutex> lock(mutex);return local.size()==3 && map.size()==3;}));
  ros::WallDuration(.1).sleep();
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(local.size(),3U);EXPECT_EQ(local.back().status,local.back().INPUT_INVALID);
  EXPECT_EQ(local.back().reason,"imu_stale");
}
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"frame_boundary_ros");testing::InitGoogleTest(&argc,argv);
  ros::AsyncSpinner spinner(1);spinner.start();
  const int result=RUN_ALL_TESTS();spinner.stop();return result;
}
