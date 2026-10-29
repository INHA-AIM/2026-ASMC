#include "lane_boundary_adapter/dynamic_obstacle_risk.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace lane_boundary_adapter::dynamic_risk;

TEST(DynamicObstacleRisk,StationaryWallDoesNotGrowButStillBlocksActualContact){
    Config config;
    std::vector<Point> path{{0.,0.},{10.,0.}};
    Object wall{{-2.,18.,1.34,4.7},{true,{0.,0.},true},91};
    EXPECT_TRUE(assess(path,{wall},0.,1.,config).clear);
    wall.motion.stationary=false;
    EXPECT_FALSE(assess(path,{wall},0.,1.,config).clear);
    wall.motion={false,{0.,0.}};
    EXPECT_FALSE(assess(path,{wall},0.,1.,config).clear);
    wall.motion={true,{0.,0.},true};wall.box.y0=1.;
    EXPECT_FALSE(assess(path,{wall},0.,1.,config).clear);
}

TEST(DynamicObstacleRisk,StationaryPropBesidePathDoesNotCausePermanentResumeHold){
    Config config;config.horizon=4.;config.stationary_margin=.05;
    std::vector<Point> path{{0.,0.},{10.,0.}};
    Object sign{{4.4,4.5,1.14,1.8},{true,{0.,0.},true},1};
    EXPECT_TRUE(assess(path,{sign},0.,1.,config).clear);
    sign.motion.stationary=false;
    EXPECT_FALSE(assess(path,{sign},0.,1.,config).clear);
    sign.motion={true,{0.,-1.}};
    EXPECT_FALSE(assess(path,{sign},0.,1.,config).clear);
    sign.motion={true,{0.,0.},true};sign.box.y0=.9;
    EXPECT_FALSE(assess(path,{sign},0.,1.,config).clear);
}

TEST(DynamicObstacleMotion,StationaryEvidenceRequiresOneSecondAndResetsOnMovement){
    MotionHistory history;Motion motion;
    for(int i=0;i<=15;++i){
        motion=history.update("source",7,100.+i*.1,{10.,3.},0.,0.,true);
        if(i<10){EXPECT_FALSE(motion.stationary);}
    }
    EXPECT_TRUE(motion.stationary);
    // Slow motion must break the stationary anchor within 0.3 seconds,
    // rather than treating a single centimetre-scale shape change as motion.
    for(int i=16;i<=18;++i)
        motion=history.update("source",7,100.+i*.1,{10.,3.+.1*(i-15)},0.,0.,true);
    EXPECT_FALSE(motion.stationary);
}

TEST(DynamicObstacleMotion,ProducerVelocitySharesStationaryEvidenceDespiteShapeJitter){
    MotionHistory history;Motion motion;Point measured{};
    for(int i=0;i<25;++i)
        motion=history.update("source",7,100.+i*.1,{10.+(i%2?.1:-.1),3.},0.,0.,true,&measured);
    EXPECT_TRUE(motion.valid);EXPECT_TRUE(motion.stationary);
    EXPECT_DOUBLE_EQ(motion.velocity.x,0.);
    measured={0.,5.};
    motion=history.update("source",7,102.5,{10.,3.5},0.,0.,true,&measured);
    EXPECT_FALSE(motion.stationary);EXPECT_GT(motion.velocity.y,2.);
}

TEST(DynamicObstacleMotion,PartialVisibilityCannotMoveAnAlreadyStationaryBox){
    MotionHistory history;Motion motion;
    for(int i=0;i<20;++i){
        const auto shape=rectangle({10.,11.,3.,3.5});
        motion=history.update("source",7,100.+i*.1,{10.5,3.25},0.,0.,true,nullptr,&shape);
    }
    ASSERT_TRUE(motion.stationary);
    for(int i=1;i<=8;++i){
        const auto shape=rectangle({10.,11.+i*.05,3.,3.5});
        motion=history.update("source",7,101.9+i*.1,{10.5+i*.025,3.25},0.,0.,true,nullptr,&shape);
        EXPECT_TRUE(motion.stationary);
    }
    // Both faces then translate slowly. A cumulative reference must detect
    // this; choosing the minimum displacement anew every frame would hide it.
    for(int i=1;i<=10;++i){
        const auto shape=rectangle({10.+i*.05,11.4+i*.05,3.,3.5});
        motion=history.update("source",7,102.7+i*.1,{10.7+i*.05,3.25},0.,0.,true,nullptr,&shape);
        if(i>=4){EXPECT_FALSE(motion.stationary);}
    }
    EXPECT_GT(motion.velocity.x,.4);
}

TEST(DynamicObstacleMotion,MovingBothFacesBreaksStationarityWithoutWaitingForAnOcclusionTimer){
    MotionHistory history;Motion motion;
    for(int i=0;i<20;++i){
        const auto shape=rectangle({10.,11.,3.,3.5});
        motion=history.update("source",7,100.+i*.1,{10.5,3.25},0.,0.,true,nullptr,&shape);
    }
    ASSERT_TRUE(motion.stationary);
    const auto shape=rectangle({10.,11.,3.5,4.});
    motion=history.update("source",7,102.,{10.5,3.75},0.,0.,true,nullptr,&shape);
    EXPECT_FALSE(motion.stationary);EXPECT_GT(motion.velocity.y,2.);
}

TEST(DynamicObstacleMotion,PredictionNeverExtendsTheLastObservedDeadline){
    MotionHistory history;
    for(int i=0;i<20;++i)history.update("source",7,100.+i*.1,{10.,3.},0.,0.,true);
    const auto short_loss=history.predict("source",7,102.1,0.,true);
    ASSERT_TRUE(short_loss.valid);EXPECT_TRUE(short_loss.stationary);EXPECT_TRUE(short_loss.extrapolated);
    EXPECT_NEAR(short_loss.age,.2,1e-6);
    EXPECT_FALSE(history.predict("source",7,102.5,0.,true).valid);
    EXPECT_FALSE(history.predict("restarted",7,102.1,0.,true).valid);
    EXPECT_FALSE(history.predict("source",7,102.1,0.,false).valid);
}

TEST(DynamicObstacleMotion,PredictedMovingVelocityRotatesWithEgoRatherThanBecomingZero){
    MotionHistory history;
    for(int i=0;i<5;++i)history.update("source",7,100.+i*.1,{10.+i*.1,3.},0.,0.,true);
    const auto motion=history.predict("source",7,100.5,2.,true);
    ASSERT_TRUE(motion.valid);EXPECT_FALSE(motion.stationary);EXPECT_TRUE(motion.extrapolated);
    EXPECT_NEAR(motion.velocity.x,std::cos(.1),1e-6);
    EXPECT_NEAR(motion.velocity.y,-std::sin(.1),1e-6);
}

TEST(DynamicObstacleMotion,CertifiedProducerVelocityDoesNotNeedConsumerWarmupOrOldClockEpoch){
    MotionHistory history;Point measured{0.,-3.};
    for(double stamp:{100.,101.5}){
        const auto motion=history.update("source",7,stamp,{10.,3.},0.,0.,true,&measured);
        EXPECT_TRUE(motion.valid);EXPECT_DOUBLE_EQ(motion.velocity.y,-3.);
        EXPECT_FALSE(motion.stationary);
    }
    EXPECT_TRUE(history.predict("source",7,101.6,0.,true).valid);
}

TEST(DynamicObstacleMotion,CertifiedMotionBreaksStationarityEvenBeforeTheShapeMoves){
    MotionHistory history;Point measured{};
    const auto shape=rectangle({10.,11.,3.,3.5});
    Motion motion;
    for(int i=0;i<20;++i)motion=history.update("source",7,100.+i*.1,{10.5,3.25},0.,0.,true,&measured,&shape);
    ASSERT_TRUE(motion.stationary);
    measured={0.,1.};motion=history.update("source",7,102.,{10.5,3.25},0.,0.,true,&measured,&shape);
    EXPECT_TRUE(motion.valid);EXPECT_FALSE(motion.stationary);EXPECT_DOUBLE_EQ(motion.velocity.y,1.);
}

TEST(DynamicObstacleMotion,StaticJitterDoesNotRepeatedlyResetStoppedClassification){
    MotionHistory history;Motion motion;
    for(int i=0;i<20;++i)motion=history.update("source",7,100.+i*.1,{10.,3.},0.,0.,true);
    ASSERT_TRUE(motion.stationary);
    for(int i=20;i<35;++i){
        motion=history.update("source",7,100.+i*.1,{10.,3.+(i%2?.015:-.015)},0.,0.,true);
        EXPECT_TRUE(motion.stationary);EXPECT_LT(std::hypot(motion.velocity.x,motion.velocity.y),.1);
    }
    motion=history.update("source",7,103.5,{10.,3.5},0.,0.,true);
    EXPECT_FALSE(motion.stationary);
    motion=history.update("source",7,103.6,{10.,4.},0.,0.,true);
    EXPECT_FALSE(motion.stationary);EXPECT_GT(motion.velocity.y,4.);
}
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

TEST(DynamicObstacleRisk,PedalAccelerationCannotHideAnEarlyCrossingConflict){
    Object actor{{6.,6.5,3.,3.5},{true,{0.,-2.}},17};
    Config config;config.horizon=2.;
    config.acceleration=.8;
    EXPECT_TRUE(assess(straight,{actor},0.,3.,config).clear);
    config.acceleration=4.;
    EXPECT_FALSE(assess(straight,{actor},0.,3.,config).clear);
    config.acceleration=0.;
    EXPECT_FALSE(assess(straight,{actor},0.,3.,config).clear);
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
