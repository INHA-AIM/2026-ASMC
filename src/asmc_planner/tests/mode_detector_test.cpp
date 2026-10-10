#include <gtest/gtest.h>
#include "Planning/PlanningMode.hpp"

TEST(PlanningMode, ExactRawGpsRuleAndRecovery) {
  morai_msgs::GPSMessage gps;
  gps.latitude = 37.0;
  gps.longitude = 126.0;
  gps.altitude = 12.0;
  EXPECT_EQ(asmc_planner::PlanningMode::NORMAL, asmc_planner::DetectPlanningMode(gps));
  gps.latitude = 0.0;
  gps.longitude = 0.0;
  gps.altitude = 0.0;
  EXPECT_EQ(asmc_planner::PlanningMode::JAMMING, asmc_planner::DetectPlanningMode(gps));
  gps.altitude = 0.01;
  EXPECT_EQ(asmc_planner::PlanningMode::NORMAL, asmc_planner::DetectPlanningMode(gps));
  gps.altitude = 0.0;
  gps.longitude = -0.01;
  EXPECT_EQ(asmc_planner::PlanningMode::NORMAL, asmc_planner::DetectPlanningMode(gps));
  gps.longitude = 0.0;
  gps.latitude = 0.01;
  EXPECT_EQ(asmc_planner::PlanningMode::NORMAL, asmc_planner::DetectPlanningMode(gps));
}
