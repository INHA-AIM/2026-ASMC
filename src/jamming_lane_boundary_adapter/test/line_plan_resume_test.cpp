// Exercise the actual planner functions without calling node main, starting a
// ROS master, publishing control commands or connecting to the simulator.
#define main unused_lane_change_node_main
#include "../src/camera_bbox_lane_change_node.cpp"
#undef main
#include <gtest/gtest.h>

class LinePlanResumeTest : public ::testing::Test {
protected:
    void SetUp() override {
        ros::Time::init();ros::Time::setNow(ros::Time(100.));
        params=Parameters{};line_state=LineState{};dynamic_state=DynamicStopState{};
        follower_state=FollowerState{};change_state=LaneChangeState{};
        avoid_state=AvoidState{};lidar_state=LidarObstacles{};
        current_obstacles.clear();planning_obstacles.clear();line_candidate_checks.clear();
        speed_integral=0.;
        planning_ready=false;gps_return=false;line_mode_active=true;
        params.use_lidar_obstacles=true;
        follower_state.locked_lane=1;follower_state.target_ready=true;
        for(int lane=0;lane<2;++lane){
            LaneChain chain;chain.lane_index=lane;
            const double centre=lane==0?3.5:0.;
            for(double x=0;x<=20.;x+=.25){
                chain.centerline.push_back({x,centre});
                chain.left_boundary.push_back({x,centre+1.75});
                chain.right_boundary.push_back({x,centre-1.75});
            }
            change_state.lanes.push_back(chain);
        }
        line_state.target=3.75;line_state.commitment_home=4.;
        line_state.clear.observe(98.,1,"fixture",true,true);
        line_state.clear.observe(98.1,2,"fixture",true,true);
        line_state.clear.observe(98.2,3,"fixture",true,true);
        dynamic_state.hold.engage();dynamic_state.hold.observeSpeed(99.,0.,true,.1);
        refreshInputs();
    }
    void refreshInputs(){
        ego_speed.received=true;ego_speed.speed=0.;ego_speed.received_at=ros::SteadyTime::now();
        dead_reckoning.imu_received=true;dead_reckoning.imu_received_at=ros::SteadyTime::now();
        lidar_state.received=true;lidar_state.received_at=ros::SteadyTime::now();
        lidar_state.arrival_age=0.;lidar_state.stamp=ros::Time::now();lidar_state.source="fixture";
        ++lidar_state.sequence;
    }
    void addObstacle(double y0,double y1){
        LineFrame frame;ASSERT_TRUE(lineFrameAt(13.,frame));
        LineObstacle obstacle{13.,1.,lineOf(frame,y1),lineOf(frame,y0),0.,params.obstacle_confirm_frames};
        obstacle.track_id=7;line_state.obstacles.push_back(obstacle);
    }
};

TEST_F(LinePlanResumeTest,SafeOffsetTargetSurvivesRestAndResumesWithinTwoSeconds){
    // Bag episode 2: a stationary vehicle is near lane 4 while line 3.75 is
    // being confirmed. Previously the home reset changed it back every cycle.
    for(unsigned frame=0;frame<40;++frame){
        ros::Time::setNow(ros::Time(100.+frame*.05));refreshInputs();
        ASSERT_TRUE(updateLinePlan(true).empty());
        ASSERT_DOUBLE_EQ(line_state.target,3.75);
        morai_msgs::CtrlCmd command;command.velocity=3.;
        const auto reason=applyDynamicObstacleStop(command,.05,0.,false);
        if(reason.empty()){
            EXPECT_FALSE(dynamic_state.hold.active);
            EXPECT_GE(frame,20u);return;
        }
        ASSERT_EQ(reason,"dynamic_confirming_clear");
    }
    FAIL()<<"Safe path did not release within two seconds";
}

TEST_F(LinePlanResumeTest,BlockedTargetCanBeReplacedAfterPhysicalRest){
    line_state.target=4.;addObstacle(-.3,.3);
    ASSERT_TRUE(updateLinePlan(true).empty())<<path_block_reason;
    EXPECT_NE(line_state.target,4.);
    EXPECT_TRUE(dynamic_state.hold.active);
    EXPECT_TRUE(pathClearOfLineObstacles(follower_state.selected_lane.centerline));
}

TEST_F(LinePlanResumeTest,BlockedRoadReportsEachRejectedCandidateAndKeepsHold){
    addObstacle(-5.,9.);
    EXPECT_EQ(updateLinePlan(true),"no_clear_line");
    EXPECT_TRUE(dynamic_state.hold.active);
    ASSERT_GT(line_candidate_checks.size(),1u);
    for(const auto& check:line_candidate_checks){
        EXPECT_EQ(check.result,"rejected");
        EXPECT_FALSE(check.last_reason.empty());
        EXPECT_GT(check.attempts,0u);
    }
}

TEST_F(LinePlanResumeTest,CompleteSafeCurveRemainsAvailableWhenLateralClippingHitsObstacle){
    params.avoid_clearance=.05;params.obstacle_lateral_margin=.05;params.avoid_shift_length=7.;
    line_state.lane_home=4.;line_state.return_home=4.;
    for(double x=8.;x<=14.;x+=.25){
        line_state.obstacles.clear();
        LineFrame frame;ASSERT_TRUE(lineFrameAt(x,frame));
        LineObstacle obstacle{x,.4,lineOf(frame,1.75),lineOf(frame,1.25),0.,params.obstacle_confirm_frames};
        obstacle.track_id=7;line_state.obstacles.push_back(obstacle);
        std::vector<FamilyPoint2d> full,clipped;
        ASSERT_TRUE(buildLinePath(2.25,full,1.,1.,false));
        ASSERT_TRUE(buildLinePath(2.25,clipped,1.,1.,true));
        if(!pathInsideRoad(full)||!pathClearOfLineObstacles(full)||pathClearOfLineObstacles(clipped))continue;
        std::vector<FamilyPoint2d> chosen;
        ASSERT_TRUE(lineClear(2.25,chosen))<<"Obstacle at "<<x;
        EXPECT_TRUE(pathInsideRoad(chosen));EXPECT_TRUE(pathClearOfLineObstacles(chosen));
        return;
    }
    FAIL()<<"Fixture must exercise safe full curve versus colliding clipped curve";
}

TEST_F(LinePlanResumeTest,EmptyScanCannotReturnHomeBeforePassingAvoidanceMilestone){
    dynamic_state.hold.active=false;line_state.return_home=4.;
    line_state.pass_odom=12.;line_state.odom=0.;
    ASSERT_TRUE(updateLinePlan(true).empty());
    EXPECT_DOUBLE_EQ(line_state.target,3.75);
    EXPECT_DOUBLE_EQ(follower_state.target_point.x,params.avoid_lookahead_distance);
    line_state.odom=13.;
    ASSERT_TRUE(updateLinePlan(true).empty());
    EXPECT_DOUBLE_EQ(line_state.target,4.);
}

TEST_F(LinePlanResumeTest,CrossingInOldLaneCannotReplaceAValidatedAvoidanceCurve){
    line_state.target=2.25;line_state.home=4.;line_state.lane_home=4.;line_state.return_home=4.;
    current_obstacles.push_back({9.,9.5,.3,.8,true,8,"unknown",{true,{0.,-.5}}});
    std::vector<FamilyPoint2d> physical;
    ASSERT_TRUE(buildPath(*getPlanningLane(1),true,physical));
    std::vector<dynamic_risk::Point> path;for(auto p:physical)path.push_back({p.x,p.y});
    dynamic_risk::Config config;config.horizon=4.;
    EXPECT_FALSE(dynamic_risk::assess(path,riskObjects(),0.,params.target_speed_kph/3.6,config).clear);
    bool handled=false;
    EXPECT_TRUE(crossingYieldPlan(handled).empty());
    EXPECT_FALSE(handled);
    EXPECT_DOUBLE_EQ(line_state.target,2.25);
}

TEST_F(LinePlanResumeTest,NearFieldCameraTangentJoinDoesNotCreateAnUndriveableKink){
    line_state.lane_home=4.;line_state.return_home=4.;
    params.avoid_max_shoulder=0.;params.avoid_line_overrun=0.;
    params.obstacle_lateral_margin=.2;
    // Saved episode 9: a roughly 1.6 cm deviation at the near-field tangent
    // join made every centre candidate exceed steering curvature at rest.
    for(auto& lane:change_state.lanes){
        for(std::size_t i=0;i<lane.centerline.size();++i){
            const double x=lane.centerline[i].x;
            const double y=x<=1.?.02*x:(x<=1.25?.02-.04*(x-1.):.01+.02*(x-1.25));
            lane.centerline[i].y+=y;lane.left_boundary[i].y+=y;lane.right_boundary[i].y+=y;
        }
    }
    std::vector<FamilyPoint2d> path;
    ASSERT_TRUE(lineClear(4.,path))<<path_block_reason;
    EXPECT_DOUBLE_EQ(path.front().y,0.);
    EXPECT_TRUE(pathInsideRoad(path));
    EXPECT_TRUE(pathClearOfLineObstacles(path));
    // Smoothing cannot make a road-wide blockage acceptable.
    addObstacle(-5.,9.);
    EXPECT_FALSE(lineClear(4.,path));
}

TEST_F(LinePlanResumeTest,LongerConnectionCandidatesAlsoApplyToSmallOffsets){
    for(auto& lane:change_state.lanes){
        for(auto& p:lane.centerline)p.y+=.2;
        for(auto& p:lane.left_boundary)p.y+=.2;
        for(auto& p:lane.right_boundary)p.y+=.2;
    }
    std::vector<FamilyPoint2d> short_path,long_path;
    ASSERT_TRUE(buildLinePath(4.,short_path,1.,1.,false));
    ASSERT_TRUE(buildLinePath(4.,long_path,1.6,1.,false));
    ASSERT_EQ(short_path.size(),long_path.size());
    const auto index=static_cast<std::size_t>(3./params.path_sample_step);
    EXPECT_LT(long_path[index].y,short_path[index].y-.01);
    EXPECT_TRUE(pathInsideRoad(short_path));EXPECT_TRUE(pathInsideRoad(long_path));
}

TEST_F(LinePlanResumeTest,RoadBoundaryUsesActualLengthWhileObstacleMarginStillBlocks){
    params.avoid_max_shoulder=0.;params.avoid_line_overrun=0.;
    for(auto& lane:change_state.lanes)
        for(auto* points:{&lane.centerline,&lane.left_boundary,&lane.right_boundary})
            for(auto& p:*points)p.y-=.4*std::min(p.x,2.);
    std::vector<FamilyPoint2d> path;
    for(double x=0.;x<=params.path_horizon;x+=params.path_sample_step)path.push_back({x,0.});
    ASSERT_TRUE(pathInsideRoad(path))<<path_block_reason;
    // An obstacle just beyond the actual nose still needs the longitudinal
    // collision margin. Relaxing road length must not relax that check.
    LineObstacle obstacle{params.vehicle_front+.2,.1,0.,0.,0.,params.obstacle_confirm_frames};
    LineFrame frame;ASSERT_TRUE(lineFrameAt(obstacle.x,frame));
    obstacle.line_left=lineOf(frame,.1);obstacle.line_right=lineOf(frame,-.1);
    line_state.obstacles.push_back(obstacle);
    EXPECT_FALSE(pathClearOfLineObstacles({{0.,0.},{.05,0.}}));
}

TEST_F(LinePlanResumeTest,SideWallCannotDelayReturnToAClearHomeLane){
    dynamic_state.hold.active=false;line_state.target=2.25;line_state.return_home=4.;
    LineFrame frame;ASSERT_TRUE(lineFrameAt(1.,frame));
    LineObstacle wall{-1.,19.,lineOf(frame,5.2),lineOf(frame,4.6),0.,params.obstacle_confirm_frames};
    line_state.obstacles.push_back(wall);
    EXPECT_FALSE(lineObstacleBlocksHome(wall));
    ASSERT_TRUE(updateLinePlan(true).empty())<<path_block_reason;
    EXPECT_DOUBLE_EQ(line_state.target,4.);
    EXPECT_DOUBLE_EQ(line_state.pass_odom,0.);
    EXPECT_TRUE(pathClearOfLineObstacles(follower_state.selected_lane.centerline));
}

TEST_F(LinePlanResumeTest,ResumeCandidateMustPassPredictionAsWellAsCurrentGeometry){
    line_state.lane_home=4.;line_state.return_home=4.;
    current_obstacles.push_back({2.,2.5,1.8,2.,true,7,"unknown",{false,{}}});
    std::vector<FamilyPoint2d> path;
    EXPECT_FALSE(lineClear(4.,path));
    EXPECT_NE(path_block_reason.find("predicted_collision"),std::string::npos);
    current_obstacles.front().motion={true,{0.,0.},true};
    EXPECT_TRUE(lineClear(4.,path))<<path_block_reason;
}

TEST_F(LinePlanResumeTest,SaturatedPedalsCannotWindUpTheSpeedIntegral){
    params.max_accel_pedal=.15;ego_speed.speed=0.;
    for(int i=0;i<200;++i){
        morai_msgs::CtrlCmd command;applySpeedControl(command,10.,.05);
        EXPECT_DOUBLE_EQ(command.accel,.15);
    }
    EXPECT_DOUBLE_EQ(speed_integral,0.);
    ego_speed.speed=10./3.6;
    morai_msgs::CtrlCmd command;applySpeedControl(command,10.,.05);
    EXPECT_DOUBLE_EQ(command.accel,0.);
    ego_speed.speed=20./3.6;
    for(int i=0;i<200;++i){
        morai_msgs::CtrlCmd brake;applySpeedControl(brake,0.,.05);
        EXPECT_DOUBLE_EQ(brake.brake,params.max_brake_pedal);
    }
    EXPECT_DOUBLE_EQ(speed_integral,0.);
}

TEST_F(LinePlanResumeTest,UnsaturatedSpeedErrorStillUsesIntegralControl){
    params.max_accel_pedal=.15;ego_speed.speed=10./3.6-.1;
    morai_msgs::CtrlCmd command;applySpeedControl(command,10.,.05);
    EXPECT_GT(speed_integral,0.);EXPECT_GT(command.accel,params.speed_kp*.1);
    EXPECT_EQ(command.longlCmdType,1);
}

TEST_F(LinePlanResumeTest,InvalidSensorCannotBecomeEmptyFreeSpaceBeforeTheFirstStop){
    for(int sensor=0;sensor<3;++sensor){
        refreshInputs();dynamic_state.hold=lane_boundary_adapter::decision::StopHold{};
        ego_speed.speed=2.;current_obstacles.clear();
        if(sensor==0)lidar_state.received=false;
        if(sensor==1)ego_speed.received=false;
        if(sensor==2)dead_reckoning.imu_received=false;
        morai_msgs::CtrlCmd command;command.velocity=10.;
        EXPECT_EQ(applyDynamicObstacleStop(command,.05,0.,false),"dynamic_inputs_unavailable");
        EXPECT_TRUE(dynamic_state.hold.active);
    }
}

TEST_F(LinePlanResumeTest,PredictionDeadlineIncludesDeliveryLatencyAndDoesNotAccumulateTwice){
    refreshInputs();lidar_state.stamp=ros::Time(99.8);
    lidar_state.objects.push_back({5.,6.,3.,3.5,true,7,"unknown",{true,{0.,0.},true,true,.4}});
    refreshPlanningObstacles();
    ASSERT_EQ(current_obstacles.size(),1u);
    EXPECT_FALSE(current_obstacles[0].motion.valid);
    EXPECT_FALSE(current_obstacles[0].motion.stationary);
    EXPECT_NEAR(current_obstacles[0].motion.age,.6,1e-6);
    lidar_state.objects[0].motion.age=.2;
    for(int i=0;i<2;++i){
        refreshPlanningObstacles();
        ASSERT_EQ(current_obstacles.size(),1u);
        EXPECT_TRUE(current_obstacles[0].motion.valid);
        EXPECT_NEAR(current_obstacles[0].motion.age,.4,1e-6);
    }
}

int main(int argc,char** argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
