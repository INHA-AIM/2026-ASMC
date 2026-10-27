#include "lane_boundary_adapter/contact_clearance.hpp"
#include <gtest/gtest.h>
using namespace lane_boundary_adapter::clearance;
static std::vector<Point> straight(double y=0.){std::vector<Point> p;for(int i=0;i<=40;++i)p.push_back({i*.25,y});return p;}
TEST(ContactClearance, EmptyAndSupported) {
 EXPECT_TRUE(pathClear(straight(),std::vector<Bounds>{},Vehicle{},15.));
 EXPECT_FALSE(pathClear(straight(),std::vector<Bounds>{},Vehicle{},12.));
 EXPECT_FALSE(pathClear({}, std::vector<Bounds>{},Vehicle{},15.));
}
TEST(ContactClearance, LaneOverlapIsNotVehicleCollision) {
 // A center-line barrier intersects both 3.5m lane strips. A 1.892m body
 // plus 0.30m lateral margin still fits in its own center corridor.
 EXPECT_TRUE(pathClear(straight(),{{8.,9.,1.5,2.0}},Vehicle{},15.));
 EXPECT_FALSE(pathClear(straight(),{{8.,9.,.8,1.5}},Vehicle{},15.));
}
TEST(ContactClearance, ContactCannotAuthorizeEarlyReturn) {
 auto p=straight(-2.);for(auto& v:p)if(v.x>8.)v.y=0.;
 EXPECT_FALSE(pathClear(p,{{5.,6.,-.3,.3}},Vehicle{},15.));
 EXPECT_TRUE(pathClear(straight(),{{25.,30.,-.3,.3}},Vehicle{},15.));
}
TEST(ContactClearance, BoundaryAndBadGeometryFailClosed) {
 EXPECT_FALSE(pathClear(straight(),{{8.,9.,1.246,2.}},Vehicle{},15.));
 EXPECT_FALSE(pathClear(straight(),{{8.,7.,2.,3.}},Vehicle{},15.));
 EXPECT_FALSE(pathClear(straight(),{{8.,9.,NAN,3.}},Vehicle{},15.));
}

TEST(ContactClearance, OcclusionFollowsRoadInsteadOfTurningEgoHeading) {
 std::vector<Polygon> occupied;
 ASSERT_TRUE(contactOccupancy({{8.,9.,-2.,-1.5}},{{0.,0.},{9.,-3.},{15.,-6.}},15.,occupied));
 // Measured rectangle is unchanged. Only the unobserved downstream band curves.
 EXPECT_DOUBLE_EQ(occupied.front()[0].y,-2.);
 EXPECT_DOUBLE_EQ(occupied.front()[2].x,9.);
 EXPECT_NEAR(occupied.back()[1].y,-5.,1e-8);
 EXPECT_NEAR(occupied.back()[2].y,-4.5,1e-8);
 EXPECT_FALSE(contactOccupancy({{8.,9.,-2.,-1.5}},{{0.,0.},{10.,-3.}},15.,occupied));
}
TEST(ContactClearance, HeadingCorrectionSatisfiesEndpointConditions) {
 const double h=1e-4;
 EXPECT_DOUBLE_EQ(startSlopeBasis(0),0.);
 EXPECT_DOUBLE_EQ(startSlopeBasis(1),0.);
 EXPECT_NEAR((startSlopeBasis(h)-startSlopeBasis(0))/h,1.,1e-6);
 EXPECT_NEAR((startSlopeBasis(1)-startSlopeBasis(1-h))/h,0.,1e-6);
 EXPECT_NEAR((startSlopeBasis(2*h)-2*startSlopeBasis(h))/h/h,0.,.004);
 EXPECT_NEAR((startSlopeBasis(1-2*h)-2*startSlopeBasis(1-h))/h/h,0.,.004);
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
