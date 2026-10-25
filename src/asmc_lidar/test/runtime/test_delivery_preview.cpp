#include <asmc_lidar/filtering/delivery_preview.hpp>
#include <gtest/gtest.h>
using namespace asmc::lidar::filtering;
namespace {
AuditDecision Base(){AuditDecision d;d.decision="keep";d.reason="drivable_overlap";d.has_spatial=true;d.spatial.available=true;d.spatial.road_ratio_grid=.6;return d;}
PointEvidence Points(){PointEvidence e;e.total=e.finite=100;e.min_base_z=.2;return e;}
DeliveryPolicy Policy(){DeliveryPolicy p;p.enabled=true;return p;}
TEST(DeliveryPreview, LargeObbCannotReplaceOutsidePointSupport){auto d=Base();auto e=Points();ApplyDeliveryPreview(d,e,Policy(),true,"unknown",.1);EXPECT_EQ(d.preview_decision,"exclude_candidate");EXPECT_EQ(d.preview_reason,"cluster_support_outside");EXPECT_EQ(d.decision,"undetermined");EXPECT_DOUBLE_EQ(d.spatial.road_ratio_grid,.6);}
TEST(DeliveryPreview, CameraDynamicProtectedOutsideAndWithoutGps){for(bool gps:{false,true}){auto d=Base();auto e=Points();ApplyDeliveryPreview(d,e,Policy(),gps,"dynamic_obstacle",.1);EXPECT_EQ(d.decision,"keep");EXPECT_TRUE(d.camera_dynamic_protected);EXPECT_EQ(d.preview_reason,"camera_dynamic_protected");}}
TEST(DeliveryPreview, InteriorSupportRequiresBothCountAndRatio) {
  for(const auto& counts:std::vector<std::array<std::size_t,3>>{
      {100,3,0},{100,3,3},{100,5,5},{20,1,1},{20,3,3},{736,8,8},{736,40,40}}) {
    auto d=Base();auto e=Points();e.total=e.finite=counts[0];e.road=counts[1];e.interior_road=counts[2];
    const bool keep=e.interior_road>=3&&double(e.interior_road)/e.total>=.05;
    ApplyDeliveryPreview(d,e,Policy(),true,"unknown",.1);
    EXPECT_EQ(d.point_preview,keep?"keep":"exclude_candidate");
    EXPECT_EQ(d.preview_decision,keep?"keep":"exclude_candidate");
    EXPECT_EQ(d.road_points,e.road);EXPECT_EQ(d.interior_road_points,e.interior_road);
  }
}
TEST(DeliveryPreview, GuardBandDoesNotInventRoadSupport){auto d=Base();auto e=Points();e.total=e.finite=e.guard=736;ApplyDeliveryPreview(d,e,Policy(),true,"unknown",.1);EXPECT_EQ(d.preview_reason,"cluster_outside_boundary_uncertain");EXPECT_EQ(d.preview_decision,"exclude_candidate");EXPECT_EQ(d.decision,"undetermined");EXPECT_EQ(d.road_points,0u);}
TEST(DeliveryPreview, PointAndObbComparisonsAreIndependent) {
  auto d=Base();auto e=Points();e.guard=100;
  ApplyDeliveryPreview(d,e,Policy(),true,"unknown",.1);
  EXPECT_EQ(d.point_preview,"exclude_candidate");EXPECT_EQ(d.obb_preview,"keep");
  d=Base();d.spatial.road_ratio_grid=0;d.decision="undetermined";e.total=e.finite=5;
  ApplyDeliveryPreview(d,e,Policy(),true,"unknown",.1);
  EXPECT_EQ(d.point_preview,"undetermined");EXPECT_EQ(d.obb_preview,"exclude_candidate");
  EXPECT_EQ(d.preview_reason,"obb_support_outside");EXPECT_EQ(d.decision,"undetermined");
  d=Base();e=Points();e.min_base_z=3;
  ApplyDeliveryPreview(d,e,Policy(),true,"unknown",3);
  EXPECT_EQ(d.point_preview,"exclude_candidate");EXPECT_EQ(d.height_preview,"exclude_candidate");
}
TEST(DeliveryPreview, MissionRegionDoesNotProtectUnknownOutsideSupport){
  for(const auto* mission:{"dynamic_obstacle","static_obstacle","roundabout"}){
    auto d=Base();d.exception_types={mission};d.reason="exception_space_protected";
    d.spatial.road_ratio_grid=0;
    ApplyDeliveryPreview(d,Points(),Policy(),true,"unknown",.1);
    EXPECT_EQ(d.preview_reason,"cluster_support_outside");
    EXPECT_EQ(d.preview_decision,"exclude_candidate");
    EXPECT_EQ(d.point_preview,"exclude_candidate");EXPECT_EQ(d.obb_preview,"exclude_candidate");
    EXPECT_EQ(d.obb_reference_reason,"exception_space_protected");
    EXPECT_EQ(d.decision,"undetermined");
    EXPECT_EQ(d.preview_protection_policy,"yolo_vehicle_dynamic_interior_v3");
  }
}
TEST(DeliveryPreview, PositiveVehicleAndDynamicProtectOutsideWhileStaticDoesNot) {
  for(const auto* group:{"unknown","vehicle","dynamic_obstacle","static_obstacle"}) {
    auto d=Base();d.exception_types={"dynamic_obstacle"};
    const bool vehicle=std::string(group)=="vehicle",dynamic=std::string(group)=="dynamic_obstacle";
    ApplyDeliveryPreview(d,Points(),Policy(),true,group,.1);
    EXPECT_EQ(d.preview_decision,vehicle||dynamic?"keep":"exclude_candidate");
    EXPECT_EQ(d.camera_dynamic_protected,dynamic);EXPECT_EQ(d.camera_vehicle_protected,vehicle);
    if(vehicle)EXPECT_EQ(d.preview_reason,"camera_vehicle_protected");
  }
}
TEST(DeliveryPreview, Unknown4059OutsideMissionIsNotProtectedByLocation){
  auto d=Base();d.exception_types={"dynamic_obstacle"};d.reason="exception_space_protected";
  d.spatial.road_ratio_grid=0;auto e=Points();e.total=e.finite=259;e.min_base_z=.029;
  ApplyDeliveryPreview(d,e,Policy(),true,"unknown",.028);
  EXPECT_EQ(d.road_points,0u);EXPECT_EQ(d.preview_decision,"exclude_candidate");
  EXPECT_FALSE(d.camera_dynamic_protected);EXPECT_EQ(d.point_preview,"exclude_candidate");
  EXPECT_EQ(d.obb_preview,"exclude_candidate");EXPECT_EQ(d.decision,"undetermined");
  // With insufficient points, do not retain a final "keep" from the old region gate.
  d=Base();d.exception_types={"dynamic_obstacle"};d.reason="exception_space_protected";
  d.spatial.road_ratio_grid=0;e.total=e.finite=5;
  ApplyDeliveryPreview(d,e,Policy(),true,"unknown",.028);
  EXPECT_EQ(d.decision,"undetermined");EXPECT_EQ(d.reason,"mission_region_is_context_only");
}
TEST(DeliveryPreview, UnknownMapOrNoGpsNotOutside){for(int kind:{0,1}){auto d=Base();auto e=Points();e.unknown=kind;ApplyDeliveryPreview(d,e,Policy(),kind==1,"unknown",.1);EXPECT_EQ(d.preview_decision,"undetermined");}}
TEST(DeliveryPreview, SparseAndNonFiniteSupportHeld){for(int kind:{0,1}){auto d=Base();auto e=Points();if(kind==0)e.total=e.finite=2;else e.finite=99;ApplyDeliveryPreview(d,e,Policy(),true,"unknown",3);EXPECT_EQ(d.preview_reason,"insufficient_cluster_support");}}
TEST(DeliveryPreview, OverheadNeedsBothLowPointAndObbClearance){auto e=Points();e.min_base_z=2.5;for(double bottom:{.1,2.5}){auto d=Base();ApplyDeliveryPreview(d,e,Policy(),true,"unknown",bottom);EXPECT_EQ(d.preview_reason,bottom>2?"overhead_clearance_candidate":"cluster_support_outside");EXPECT_EQ(d.decision,"undetermined");}}
TEST(DeliveryPreview, LowSupportPointDefeatsWrongHighObb){auto d=Base();auto e=Points();e.road=e.interior_road=10;ApplyDeliveryPreview(d,e,Policy(),true,"unknown",3);EXPECT_EQ(d.preview_reason,"cluster_drivable_support");}
TEST(DeliveryPreview, VisibleTopOfKnownStaticObstacleCannotBeOverheadDeleted){auto d=Base();auto e=Points();e.min_base_z=3;e.road=e.interior_road=30;ApplyDeliveryPreview(d,e,Policy(),true,"static_obstacle",3);EXPECT_EQ(d.preview_decision,"keep");EXPECT_EQ(d.preview_reason,"cluster_drivable_support");}
TEST(DeliveryPreview, RoofMarginIsStrictAndNoObjectMutation){auto d=Base();auto e=Points();auto p=Policy();e.min_base_z=p.roof_base_z_m+p.roof_margin_m;ApplyDeliveryPreview(d,e,p,true,"unknown",e.min_base_z);EXPECT_NE(d.preview_reason,"overhead_clearance_candidate");EXPECT_EQ(e.total,100u);EXPECT_NEAR(e.min_base_z,1.884,1e-12);}
TEST(DeliveryPreview, OnlyFreshPositiveSameEpochCameraEvidenceProtects) {
  const std::int64_t now=1000000000;
  EXPECT_TRUE(PositiveCameraEvidence(now,now-500000000,2,2,0,500,50));
  EXPECT_TRUE(PositiveCameraEvidence(now,now+50000000,2,2,0,500,50));
  EXPECT_FALSE(PositiveCameraEvidence(now,now-500000001,2,2,0,500,50));
  EXPECT_FALSE(PositiveCameraEvidence(now,now+50000001,2,2,0,500,50));
  EXPECT_FALSE(PositiveCameraEvidence(now,now,3,2,0,500,50));
  EXPECT_FALSE(PositiveCameraEvidence(now,now,2,2,1,500,50));
  EXPECT_FALSE(PositiveCameraEvidence(now,0,2,2,0,500,50));
}
TEST(DeliveryPreview, LegacyModeRemainsUnchanged){auto d=Base();DeliveryPolicy p;ApplyDeliveryPreview(d,Points(),p,true,"unknown",3);EXPECT_FALSE(d.preview_enabled);EXPECT_EQ(d.reason,"drivable_overlap");}
}  // namespace
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
