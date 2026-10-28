#include "lane_boundary_adapter/contact_clearance.hpp"
#include <gtest/gtest.h>
#include "lane_boundary_adapter/drivable_clearance.hpp"
using namespace lane_boundary_adapter::clearance;
static std::vector<Point> straight(double y=0.){std::vector<Point> p;for(int i=0;i<=40;++i)p.push_back({i*.25,y});return p;}
TEST(ContactClearance, NearSampleOffsetDoesNotAuthorizeAForwardGap) {
 EXPECT_DOUBLE_EQ(contiguousSupport({{1.11392,7.7},{7.7,19.4}}),19.4);
 EXPECT_DOUBLE_EQ(contiguousSupport({{1.11392,7.7},{8.,19.4}}),7.7);
 EXPECT_DOUBLE_EQ(contiguousSupport({{1.26,19.4}}),0.);
 EXPECT_DOUBLE_EQ(nearReferenceX(.9),1.);
 EXPECT_DOUBLE_EQ(nearReferenceX(1.11392),1.11392);
 EXPECT_DOUBLE_EQ(nearReferenceX(1.25),1.25);
 EXPECT_TRUE(std::isnan(nearReferenceX(1.251)));
 EXPECT_TRUE(std::isnan(nearReferenceX(3.)));
 EXPECT_TRUE(std::isnan(nearReferenceX(NAN)));
}
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
TEST(ContactClearance, ProjectedBandDoesNotMixNearAndFarLateralExtents) {
 const Polygon p{{25.58,3.475},{24.39,.846},{18.41,.622},{19.04,2.518}};
 std::vector<Polygon> exact,box;
 ASSERT_TRUE(projectedContactOccupancy({p},{{0.,0.},{30.,0.}},20.,exact));
 ASSERT_TRUE(contactOccupancy({{18.41,25.58,.622,3.475}},{{0.,0.},{30.,0.}},20.,box));
 std::vector<Point> path;for(int i=0;i<=60;++i)path.push_back({i*.25,4.});
 EXPECT_TRUE(pathClearOfPolygons(path,exact,Vehicle{},20.));
 EXPECT_FALSE(pathClearOfPolygons(path,box,Vehicle{},20.));
 // Geometry at the middle of the real projected band is still occupied.
 for(auto& point:path)point.y=1.;
 EXPECT_FALSE(pathClearOfPolygons(path,exact,Vehicle{},20.));
}
TEST(ContactClearance, ProjectedContactKeepsUnknownDepthAndRejectsBadOrder) {
 std::vector<Polygon> occupied;
 ASSERT_TRUE(projectedContactOccupancy({Polygon{{5.,-.3},{6.,-.3},{6.,.3},{5.,.3}}},{{0.,0.},{15.,0.}},15.,occupied));
 auto path=straight(-2.);for(auto& point:path)if(point.x>8.)point.y=0.;
 EXPECT_FALSE(pathClearOfPolygons(path,occupied,Vehicle{},15.));
 EXPECT_FALSE(projectedContactOccupancy({Polygon{{5.,-.3},{6.,.3},{6.,-.3},{5.,.3}}},{{0.,0.},{15.,0.}},15.,occupied));
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
TEST(ContactClearance, ConvexHullPaddingPreservesRotationAndEveryCorner) {
 const Polygon measured{{18.282587,-1.663957},{18.022652,-1.569104},{17.592718,-2.747056},
     {17.590937,-2.758485},{17.850870,-2.853337},{18.280804,-1.675385}};
 const auto hull=paddedHull(measured,.25);
 ASSERT_TRUE(convexValid(hull));EXPECT_GT(hull.size(),4u);
 for(const auto& p:measured)for(double dx:{-.25,.25})for(double dy:{-.25,.25}){
   for(size_t i=0;i<hull.size();++i)EXPECT_GE(cross(hull[i],hull[(i+1)%hull.size()],{p.x+dx,p.y+dy}),-1e-8);
 }
 auto area=[](const Polygon& p){double a=0.;for(size_t i=0;i<p.size();++i)a+=p[i].x*p[(i+1)%p.size()].y-p[i].y*p[(i+1)%p.size()].x;return std::abs(a)*.5;};
 EXPECT_LT(area(hull),(18.282587-17.590937+.5)*(-1.569104+2.853337+.5));
 EXPECT_TRUE(overlaps(hull,{{18.,-2.},{18.1,-2.},{18.1,-1.9},{18.,-1.9}}));
 EXPECT_TRUE(overlaps({},hull));
 Polygon star;for(int i:{0,2,4,1,3})star.push_back({std::cos(i*2*3.141592653589793/5),std::sin(i*2*3.141592653589793/5)});
 EXPECT_FALSE(convexValid(star));EXPECT_TRUE(overlaps(star,hull));
 EXPECT_FALSE(pathClearOfPolygons(straight(),{Polygon{{30.,0.},{30.,1.}}},Vehicle{},15.));
 std::vector<Polygon> occupied;EXPECT_FALSE(projectedContactOccupancy({hull},{{0.,0.},{30.,0.}},20.,occupied));
}
TEST(DrivableClearance, WholeBodyAndUnknownCellsAreChecked) {
 GridSpec spec{250,160,20.,8.,.1};std::vector<uint8_t> p(40000,255),c(40000,255);DrivableClearance grid;
 ASSERT_TRUE(grid.prepare(spec,p,c,Vehicle{}));EXPECT_TRUE(grid.check(straight(),Vehicle{}).clear);
 // A narrow hole away from the path centre still intersects the body.
 p[120*160+70]=0;ASSERT_TRUE(grid.prepare(spec,p,c,Vehicle{}));EXPECT_FALSE(grid.check(straight(),Vehicle{}).clear);
 p.assign(40000,255);c[120*160+70]=0;ASSERT_TRUE(grid.prepare(spec,p,c,Vehicle{}));EXPECT_FALSE(grid.check(straight(),Vehicle{}).clear);
 EXPECT_TRUE(grid.check(straight(-3.),Vehicle{}).clear);
 EXPECT_FALSE(grid.check(straight(7.5),Vehicle{}).clear);
 EXPECT_FALSE(grid.prepare(spec,{},c,Vehicle{}));EXPECT_FALSE(grid.check(straight(),Vehicle{}).clear);
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
