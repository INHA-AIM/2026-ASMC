#include "lane_boundary_adapter/decision_guard.hpp"
#include <gtest/gtest.h>
using namespace lane_boundary_adapter::decision;

TEST(ClearEvidence,OneEmptyFrameCannotAuthorizeReturn){
    ClearEvidence evidence;
    evidence.observe(32.670,350,"producer",true,true);
    evidence.observe(32.707,350,"producer",true,true);
    EXPECT_FALSE(evidence.ready(32.707,1.));
    evidence.observe(32.744,351,"producer",true,false);
    EXPECT_FALSE(evidence.ready(35.,1.));
}
TEST(ClearEvidence,TimeWithoutNewScansIsInsufficient){
    ClearEvidence evidence;
    for(int i=0;i<50;++i)evidence.observe(1+.1*i,7,"producer",true,true);
    EXPECT_FALSE(evidence.ready(6.,1.));
    EXPECT_EQ(evidence.frames,1u);
}
TEST(ClearEvidence,ValidDistinctScansAndIntervalAreBothRequired){
    ClearEvidence evidence;
    evidence.observe(1.,1,"producer",true,true);
    evidence.observe(1.1,2,"producer",true,true);
    evidence.observe(1.2,3,"producer",true,true);
    EXPECT_FALSE(evidence.ready(1.2,1.));
    evidence.observe(2.1,4,"producer",true,true);
    EXPECT_TRUE(evidence.ready(2.1,1.));
    evidence.observe(2.2,5,"restarted",true,true);
    EXPECT_FALSE(evidence.ready(2.2,1.));
}
TEST(ClearEvidence,InvalidInputAndClockRewindResetEvidence){
    ClearEvidence evidence;
    evidence.observe(3.,1,"producer",true,true);
    evidence.observe(4.,2,"producer",false,true);
    EXPECT_EQ(evidence.frames,0u);
    evidence.observe(5.,3,"producer",true,true);
    evidence.observe(1.,4,"producer",true,true);
    EXPECT_DOUBLE_EQ(evidence.since,1.);
    EXPECT_EQ(evidence.frames,1u);
}
TEST(StopHold,ThirdBagStopCannotReleaseAtEightKph){
    StopHold hold;hold.engage();
    for(int i=0;i<30;++i){
        double now=36.+i*.05;
        hold.observeSpeed(now,8.1/3.6,true,.1);
        EXPECT_FALSE(hold.release(now,true,true,"same_path",i+1,"source",.25,1.));
    }
    EXPECT_TRUE(hold.active);
}
TEST(StopHold,PhysicalRestThenContinuousSafeEvidenceAllowsResume){
    StopHold hold;hold.engage();
    hold.observeSpeed(1.,0,true,.1);
    EXPECT_FALSE(hold.release(1.,true,true,"same",1,"source",.25,1.));
    hold.observeSpeed(1.3,0,true,.1);
    EXPECT_FALSE(hold.release(1.3,true,true,"same",2,"source",.25,1.));
    EXPECT_FALSE(hold.release(1.8,true,true,"same",3,"source",.25,1.));
    EXPECT_FALSE(hold.release(2.2,true,true,"same",4,"source",.25,1.));
    EXPECT_TRUE(hold.release(2.4,true,true,"same",5,"source",.25,1.));
    EXPECT_FALSE(hold.active);
}
TEST(StopHold,ChangingTheSelectedPathDoesNotReleaseTheStop){
    StopHold hold;hold.engage();hold.observeSpeed(1.,0,true,.1);
    hold.release(1.3,true,true,"right",1,"source",.25,1.);
    hold.release(1.8,true,true,"right",2,"source",.25,1.);
    EXPECT_FALSE(hold.release(2.4,true,true,"left",3,"source",.25,1.));
    EXPECT_FALSE(hold.release(3.,true,true,"left",4,"source",.25,1.));
    EXPECT_TRUE(hold.active);
}
TEST(StopHold,RiskReappearanceOrStaleInputCancelsClearConfirmation){
    StopHold hold;hold.engage();hold.observeSpeed(1.,0,true,.1);
    hold.release(1.3,true,true,"same",1,"source",.25,1.);
    hold.release(1.8,true,true,"same",2,"source",.25,1.);
    EXPECT_FALSE(hold.release(2.4,false,true,"same",3,"source",.25,1.));
    EXPECT_EQ(hold.clear.frames,0u);
    hold.release(2.5,true,true,"same",4,"source",.25,1.);
    EXPECT_FALSE(hold.release(3.6,true,false,"same",5,"source",.25,1.));
    EXPECT_TRUE(hold.active);
}
TEST(StopHold,SameLineIdentifierCannotHideCumulativeGeometryChange){
    StopHold hold;hold.engage();hold.observeSpeed(1.,0,true,.1);
    hold.observePath({{0.,0.},{1.,0.}});
    hold.release(1.3,true,true,"same_line",1,"source",.25,1.);
    hold.observePath({{0.,0.},{1.,.2}});
    hold.release(1.8,true,true,"same_line",2,"source",.25,1.);
    EXPECT_EQ(hold.clear.frames,2u);
    EXPECT_FALSE(hold.release(2.1,true,true,"same_line",2,"source",.25,1.));
    hold.observePath({{0.,0.},{1.,.4}});
    EXPECT_EQ(hold.clear.frames,0u);
    EXPECT_EQ(hold.clear.reset_reason,"path_geometry_changed");
    EXPECT_FALSE(hold.release(2.4,true,true,"same_line",3,"source",.25,1.));
    hold.observePath({{0.,0.},{1.,.4}});
    EXPECT_FALSE(hold.release(2.9,true,true,"same_line",4,"source",.25,1.));
    EXPECT_FALSE(hold.release(3.3,true,true,"same_line",5,"source",.25,1.));
    EXPECT_TRUE(hold.release(3.5,true,true,"same_line",6,"source",.25,1.));
}
TEST(StopHold,ForwardClockStepCannotCompleteTwoScanConfirmation){
    StopHold hold;hold.engage();hold.observeSpeed(100.,0.,true,.1);
    EXPECT_FALSE(hold.release(100.3,true,true,"same",1,"source",.25,1.));
    EXPECT_FALSE(hold.release(100.35,true,true,"same",2,"source",.25,1.));
    EXPECT_FALSE(hold.release(101.94,true,true,"same",3,"source",.25,1.));
    EXPECT_TRUE(hold.active);EXPECT_EQ(hold.clear.frames,0u);
    for(unsigned i=1;i<=24;++i)
        EXPECT_FALSE(hold.release(101.94+i*.05,true,true,"same",3+i,"source",.25,1.));
    EXPECT_TRUE(hold.release(103.24,true,true,"same",29,"source",.25,1.));
}
TEST(LineSelection,OppositeSideHasHigherChangeCost){
    EXPECT_TRUE(oppositeSide(5.,2.,4.));
    EXPECT_FALSE(oppositeSide(5.,4.5,4.));
    EXPECT_GT(lineCost(3.5,5.,4.),lineCost(5.5,5.,4.));
}
TEST(LineSelection,OppositeSideWaitsButSameSideCanRespondToNewObstacle){
    EXPECT_FALSE(canSwitchSide(5.,2.,4.,10.1,10.,1.));
    EXPECT_TRUE(canSwitchSide(5.,5.5,4.,10.1,10.,1.));
    EXPECT_TRUE(canSwitchSide(5.,2.,4.,11.1,10.,1.));
    EXPECT_FALSE(canSwitchSide(5.,2.,4.,9.,10.,1.));
}
int main(int argc,char** argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
