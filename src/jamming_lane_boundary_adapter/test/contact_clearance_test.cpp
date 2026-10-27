#include "lane_boundary_adapter/contact_clearance.hpp"
#include <gtest/gtest.h>
using namespace lane_boundary_adapter::clearance;
static std::vector<Point> straight(double y=0.){std::vector<Point> p;for(int i=0;i<=40;++i)p.push_back({i*.25,y});return p;}
TEST(ContactClearance, EmptyAndSupported) {
 EXPECT_TRUE(pathClear(straight(),{},Vehicle{},15.));
 EXPECT_FALSE(pathClear(straight(),{},Vehicle{},12.));
 EXPECT_FALSE(pathClear({}, {},Vehicle{},15.));
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
