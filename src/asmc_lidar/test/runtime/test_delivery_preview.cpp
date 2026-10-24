#include <asmc_lidar/filtering/delivery_preview.hpp>
#include <gtest/gtest.h>
using namespace asmc::lidar::filtering;
namespace {
AuditDecision Base(){AuditDecision d;d.decision="keep";d.reason="drivable_overlap";d.has_spatial=true;d.spatial.available=true;d.spatial.road_ratio_grid=.6;return d;}
PointEvidence Points(){PointEvidence e;e.total=e.finite=100;e.min_base_z=.2;return e;}
DeliveryPolicy Policy(){DeliveryPolicy p;p.enabled=true;return p;}
TEST(DeliveryPreview, LargeObbCannotReplaceOutsidePointSupport){auto d=Base();auto e=Points();ApplyDeliveryPreview(d,e,Policy(),true,false,false,.1);EXPECT_EQ(d.preview_decision,"exclude_candidate");EXPECT_EQ(d.preview_reason,"cluster_support_outside");EXPECT_EQ(d.decision,"undetermined");EXPECT_DOUBLE_EQ(d.spatial.road_ratio_grid,.6);}
TEST(DeliveryPreview, CameraDynamicProtectedOutsideAndWithoutGps){for(bool gps:{false,true}){auto d=Base();auto e=Points();ApplyDeliveryPreview(d,e,Policy(),gps,true,true,.1);EXPECT_EQ(d.decision,"keep");EXPECT_TRUE(d.camera_dynamic_protected);EXPECT_EQ(d.preview_reason,"camera_dynamic_protected");}}
TEST(DeliveryPreview, RoadSupportPreservesRealObjects){for(int kind:{0,1}){auto d=Base();auto e=Points();if(kind==0)e.road=3;if(kind==1){e.total=e.finite=20;e.road=1;}ApplyDeliveryPreview(d,e,Policy(),true,false,false,.1);EXPECT_EQ(d.preview_decision,"keep");}}
TEST(DeliveryPreview, GuardBandDoesNotInventRoadSupport){auto d=Base();auto e=Points();e.total=e.finite=e.guard=736;ApplyDeliveryPreview(d,e,Policy(),true,false,false,.1);EXPECT_EQ(d.preview_reason,"cluster_outside_boundary_uncertain");EXPECT_EQ(d.preview_decision,"exclude_candidate");EXPECT_EQ(d.decision,"undetermined");EXPECT_EQ(d.road_points,0u);}
TEST(DeliveryPreview, MissionApproachProtectedBeforeEgoEntry){auto d=Base();d.exception_types={"dynamic_obstacle"};ApplyDeliveryPreview(d,Points(),Policy(),true,false,false,.1);EXPECT_EQ(d.preview_reason,"exception_space_protected");}
TEST(DeliveryPreview, UnknownMapOrNoGpsNotOutside){for(int kind:{0,1}){auto d=Base();auto e=Points();e.unknown=kind;ApplyDeliveryPreview(d,e,Policy(),kind==1,false,false,.1);EXPECT_EQ(d.preview_decision,"undetermined");}}
TEST(DeliveryPreview, SparseAndNonFiniteSupportHeld){for(int kind:{0,1}){auto d=Base();auto e=Points();if(kind==0)e.total=e.finite=2;else e.finite=99;ApplyDeliveryPreview(d,e,Policy(),true,false,false,3);EXPECT_EQ(d.preview_reason,"insufficient_cluster_support");}}
TEST(DeliveryPreview, OverheadNeedsBothLowPointAndObbClearance){auto e=Points();e.min_base_z=2.5;for(double bottom:{.1,2.5}){auto d=Base();ApplyDeliveryPreview(d,e,Policy(),true,false,false,bottom);EXPECT_EQ(d.preview_reason,bottom>2?"overhead_clearance_candidate":"cluster_support_outside");EXPECT_EQ(d.decision,"undetermined");}}
TEST(DeliveryPreview, LowSupportPointDefeatsWrongHighObb){auto d=Base();auto e=Points();e.road=10;ApplyDeliveryPreview(d,e,Policy(),true,false,false,3);EXPECT_EQ(d.preview_reason,"cluster_drivable_support");}
TEST(DeliveryPreview, VisibleTopOfKnownVehicleCannotBeOverheadDeleted){auto d=Base();auto e=Points();e.min_base_z=3;e.road=30;ApplyDeliveryPreview(d,e,Policy(),true,false,true,3);EXPECT_EQ(d.preview_decision,"keep");EXPECT_EQ(d.preview_reason,"cluster_drivable_support");}
TEST(DeliveryPreview, RoofMarginIsStrictAndNoObjectMutation){auto d=Base();auto e=Points();auto p=Policy();e.min_base_z=p.roof_base_z_m+p.roof_margin_m;ApplyDeliveryPreview(d,e,p,true,false,false,e.min_base_z);EXPECT_NE(d.preview_reason,"overhead_clearance_candidate");EXPECT_EQ(e.total,100u);EXPECT_NEAR(e.min_base_z,1.884,1e-12);}
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
TEST(DeliveryPreview, LegacyModeRemainsUnchanged){auto d=Base();DeliveryPolicy p;ApplyDeliveryPreview(d,Points(),p,true,false,false,3);EXPECT_FALSE(d.preview_enabled);EXPECT_EQ(d.reason,"drivable_overlap");}
}  // namespace
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
