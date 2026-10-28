#include "lane_boundary_adapter/dynamic_obstacle_risk.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace lane_boundary_adapter::dynamic_risk;
namespace {
const std::vector<Point> straight{{0,0},{30,0}};

TEST(DynamicObstacleRisk,AdjacentParallelActorDoesNotBlock) {
    Object actor{{5,6,3,3.5},{true,{2,0}},17};
    EXPECT_TRUE(assess(straight,{actor},2,2,{}).clear);
}

TEST(DynamicObstacleRisk,OccupiedPathBlocksRegardlessOfObjectId) {
    for(int id:{-2,0,17}) {
        Object actor{{3,4,-.25,.25},{true,{0,0}},id};
        const auto risk=assess(straight,{actor},2,2,{});
        EXPECT_FALSE(risk.clear);
        EXPECT_EQ(risk.id,id);
    }
}

TEST(DynamicObstacleRisk,CrossingActorOutsideCurrentPathBlocksFuturePath) {
    Object actor{{5,5.5,3,3.5},{true,{0,-2}},17};
    const auto risk=assess(straight,{actor},2,2,{});
    EXPECT_FALSE(risk.clear);
    EXPECT_GT(risk.ttc,0);
}

TEST(DynamicObstacleRisk,ActorMovingAwayDoesNotBlock) {
    Object actor{{5,5.5,2,2.5},{true,{0,2}},17};
    EXPECT_TRUE(assess(straight,{actor},2,2,{}).clear);
}

TEST(DynamicObstacleRisk,CrossingBetweenTimeSamplesIsDetected) {
    Config config;config.width=.1;config.margin=0;
    Object actor{{2,2.1,.29,.31},{true,{0,-12}},17};
    EXPECT_FALSE(assess(straight,{actor},0,0,config).clear);
}

TEST(DynamicObstacleRisk,UnknownMotionIsNotAssumedStationary) {
    Object actor{{2,2.5,1.8,2.0},{false,{}},17};
    EXPECT_FALSE(assess(straight,{actor},0,0,{}).clear);
    actor.motion.valid=true;
    EXPECT_TRUE(assess(straight,{actor},0,0,{}).clear);
}

TEST(DynamicObstacleRisk,SlowerArrivalCanYieldToCrossingActor) {
    Object actor{{8,8.5,3,3.5},{true,{0,-2}},17};
    EXPECT_FALSE(assess(straight,{actor},2,3,{}).clear);
    EXPECT_TRUE(assess(straight,{actor},2,.5,{}).clear);
}

TEST(DynamicObstacleMotion,EgoTranslationKeepsMeasuredShape) {
    Object actor{{10,12,-.5,.5},{true,{0,0}},17};
    auto aligned=alignToEgo(actor,.2,5,0);
    EXPECT_NEAR(aligned.box.x0,9,1e-8);
    EXPECT_NEAR(aligned.box.x1,11,1e-8);
    EXPECT_NEAR(aligned.box.x1-aligned.box.x0,2,1e-8);
}

TEST(DynamicObstacleMotion,ProducerVelocityAdvancesObjectBeforeEgoCompensation) {
    Object actor{{10,12,-.5,.5},{true,{2,1}},17};
    auto aligned=alignToEgo(actor,.2,5,0);
    EXPECT_NEAR(aligned.box.x0,9.4,1e-8);
    EXPECT_NEAR(aligned.box.y0,-.3,1e-8);
}

TEST(DynamicObstacleMotion,EgoRotationPreservesObbEdgeLength) {
    Object actor{{10,12,-.5,.5},{true,{0,0}},17};
    auto aligned=alignToEgo(actor,.2,0,1);
    ASSERT_EQ(aligned.footprint.size(),4u);
    auto a=aligned.footprint[0],b=aligned.footprint[1];
    EXPECT_NEAR(std::hypot(a.x-b.x,a.y-b.y),2,1e-8);
    EXPECT_NEAR(aligned.motion.velocity.x,0,1e-8);
}

TEST(DynamicObstacleRisk,ActualObbIsUsedInsteadOfItsLargeBoundingRectangle) {
    Config config;config.front=1;config.rear=.5;config.width=.5;config.margin=0;
    Object actor{{0,6,-1,4},{true,{0,0}},17};
    actor.footprint={{0,3},{1,4},{6,0},{5,-1}};
    EXPECT_TRUE(assess(straight,{actor},0,0,config).clear);
    actor.footprint.clear();
    EXPECT_FALSE(assess(straight,{actor},0,0,config).clear);
}

TEST(DynamicObstacleMotion,GpsDeniedEgoTranslationDoesNotMakeStaticObjectMove) {
    MotionHistory history;
    EXPECT_FALSE(history.update("producer/1",17,1,{10,0},5,0,true).valid);
    EXPECT_FALSE(history.update("producer/1",17,1.1,{9.5,0},5,0,true).valid);
    const auto motion=history.update("producer/1",17,1.2,{9,0},5,0,true);
    ASSERT_TRUE(motion.valid);
    EXPECT_NEAR(motion.velocity.x,0,1e-8);
    EXPECT_NEAR(motion.velocity.y,0,1e-8);
}

TEST(DynamicObstacleMotion,EgoRotationCompensationAndSessionSeparation) {
    MotionHistory history;
    for(int i=0;i<3;++i){
        const double angle=.05*i;
        const auto motion=history.update("producer/1",17,1+.1*i,
            {10*std::cos(angle)+2*std::sin(angle),-10*std::sin(angle)+2*std::cos(angle)},0,.5,true);
        if(i==2){ASSERT_TRUE(motion.valid);EXPECT_NEAR(std::hypot(motion.velocity.x,motion.velocity.y),0,1e-8);}
    }
    EXPECT_FALSE(history.update("producer/2",17,1.3,{10,2},0,0,true).valid);
    EXPECT_FALSE(history.update("producer/1",17,3,{10,2},0,0,true).valid);
}

TEST(FinalPathValidation,ReducedPathMustBeCheckedSeparatelyFromConvergedPath) {
    Config cfg;cfg.front=.2;cfg.rear=.2;cfg.width=.2;cfg.margin=0;
    Object actor{{4.5,5.5,-.2,.2},{true,{0,0}},17};
    const std::vector<Point> converged{{0,0},{2,2},{8,2},{10,2}};
    const std::vector<Point> reduced{{0,0},{2,0},{8,0},{10,0}};
    EXPECT_TRUE(checkStaticPath(converged,{actor},cfg).clear);
    auto hit=checkStaticPath(reduced,{actor},cfg);
    EXPECT_FALSE(hit.clear);EXPECT_EQ(hit.id,17);
}

TEST(FinalPathValidation,SweptAreaBetweenSamplesCannotSkipObject) {
    Config cfg;cfg.front=.2;cfg.rear=.2;cfg.width=.2;cfg.margin=0;
    Object actor{{4.5,5.5,-.2,.2},{true,{0,0}},17};
    EXPECT_FALSE(checkStaticPath({{0,0},{10,0}},{actor},cfg).clear);
    EXPECT_TRUE(checkStaticPath({{0,2},{10,2}},{actor},cfg).clear);
}

TEST(CrossingDecision,ParallelMotionOnSlopedRoadIsNotCrossing) {
    Object actor{{5,6,3,3.5},{true,{2,1}},17};
    EXPECT_FALSE(lateralMover(actor,{{0,0},{10,5}}));
    actor.motion.velocity={1,-2};
    EXPECT_TRUE(lateralMover(actor,{{0,0},{10,5}}));
    actor.motion.valid=false;
    EXPECT_FALSE(lateralMover(actor,{{0,0},{10,5}}));
}

TEST(DynamicObstacleRisk,InvalidGeometryOrSpeedCannotPass) {
    Object actor{{5,6,-.25,.25},{true,{0,0}},17};
    const double nan=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(assess(straight,{actor},nan,2,{}).clear);
    EXPECT_FALSE(assess({{0,0},{nan,0}},{actor},2,2,{}).clear);
    EXPECT_FALSE(assess({{0,0},{0,0}},{actor},2,2,{}).clear);
}
} // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
