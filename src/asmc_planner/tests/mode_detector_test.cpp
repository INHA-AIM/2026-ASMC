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

#include "Planning/JammingDetector.hpp"

namespace {
morai_msgs::GPSMessage Gps(double lat) {
  morai_msgs::GPSMessage gps;
  gps.latitude = lat; gps.longitude = lat == 0.0 ? 0.0 : 126.0; gps.altitude = lat == 0.0 ? 0.0 : 12.0;
  gps.status = lat == 0.0 ? 0 : 1;
  return gps;
}
}  // namespace

TEST(JammingDetector, ZeroEntersImmediatelyAndExitNeedsHold) {
  using asmc_planner::PlanningMode;
  asmc_planner::JammingDetector detector({0.5, 1.0, false});
  detector.OnGps(Gps(37.0), 0.0);
  EXPECT_FALSE(detector.Update(0.0));
  detector.OnGps(Gps(0.0), 0.1);
  EXPECT_TRUE(detector.Update(0.1));
  EXPECT_EQ(PlanningMode::JAMMING, detector.mode());
  EXPECT_EQ(asmc_planner::JammingReason::GPS_ZERO, detector.entry_reason());
  detector.OnGps(Gps(37.0), 0.2);
  EXPECT_FALSE(detector.Update(0.2));
  // 홀드 중 한 번이라도 0이 오면 복귀 시계가 다시 시작된다.
  detector.OnGps(Gps(0.0), 0.7);
  EXPECT_FALSE(detector.Update(0.7));
  for (double t = 0.8; t < 1.75; t += 0.1) {
    detector.OnGps(Gps(37.0), t);
    EXPECT_FALSE(detector.Update(t));
  }
  detector.OnGps(Gps(37.0), 1.85);
  EXPECT_TRUE(detector.Update(1.85));
  EXPECT_EQ(PlanningMode::NORMAL, detector.mode());
}

TEST(JammingDetector, TimeoutOnlyAfterFirstGps) {
  using asmc_planner::PlanningMode;
  asmc_planner::JammingDetector detector({0.5, 1.0, false});
  EXPECT_FALSE(detector.Update(10.0));
  detector.OnGps(Gps(37.0), 10.0);
  EXPECT_FALSE(detector.Update(10.4));
  EXPECT_TRUE(detector.Update(10.6));
  EXPECT_EQ(asmc_planner::JammingReason::GPS_TIMEOUT, detector.entry_reason());
  detector.OnGps(Gps(37.0), 11.0);
  EXPECT_FALSE(detector.Update(11.0));
  detector.OnGps(Gps(37.0), 12.0);
  EXPECT_TRUE(detector.Update(12.0));
}

TEST(JammingDetector, NoFixIsJamming) {
  asmc_planner::JammingDetector detector({0.5, 1.0, false});
  detector.OnGps(Gps(37.0), 0.0);
  EXPECT_FALSE(detector.Update(0.0));
  morai_msgs::GPSMessage no_fix = Gps(37.0);
  no_fix.status = 0;
  detector.OnGps(no_fix, 0.1);
  EXPECT_TRUE(detector.Update(0.1));
  EXPECT_EQ(asmc_planner::JammingReason::GPS_NO_FIX, detector.entry_reason());
  detector.OnGps(Gps(37.0), 0.2);
  EXPECT_FALSE(detector.Update(0.2));
  detector.OnGps(Gps(37.0), 1.2);
  EXPECT_TRUE(detector.Update(1.2));
}

TEST(JammingDetector, ZoneEntersAndLeavesWithHold) {
  asmc_planner::JammingDetector detector({0.5, 1.0, true});
  detector.OnGps(Gps(37.0), 0.0);
  EXPECT_FALSE(detector.Update(0.0));
  detector.OnZone(true);
  EXPECT_TRUE(detector.Update(0.1));
  EXPECT_EQ(asmc_planner::JammingReason::SHADOW_ZONE, detector.entry_reason());
  detector.OnZone(false);
  detector.OnGps(Gps(37.0), 0.5);
  EXPECT_FALSE(detector.Update(0.5));
  detector.OnGps(Gps(37.0), 1.0);
  EXPECT_FALSE(detector.Update(1.0));
  detector.OnGps(Gps(37.0), 1.5);
  EXPECT_TRUE(detector.Update(1.5));
}

TEST(ShadowZone, CompetitionPathStations) {
  const std::string path = std::string(ASMC_REPO_ROOT) + "/R_KR_PG_KATRI/2026_molit_comp_global_path.txt";
  asmc_planner::ShadowZone zone(path, 1877.744798966, 1996.730791531, 10.0, 1.0);
  double station = -1.0;
  // 실측 정지 위치: A126 내부(1910.7m)와 A448 접근부(1864.1m).
  EXPECT_TRUE(zone.Contains(2.754, -507.090, -2.504, &station));
  EXPECT_NEAR(1910.7, station, 1.0);
  EXPECT_FALSE(zone.Contains(40.408, -479.126, -2.561));
  // 재밍 회피로 경로 왼쪽 4.3m까지 나간 실측 위치(1934m)도 구간 안이다.
  EXPECT_TRUE(zone.Contains(-17.484, -518.841, -2.554));
  // 구간 시작점(A448 종료 29.59,-487.65) 바로 뒤는 안, 진행 방향으로 1m 앞(이전)은 밖이다.
  EXPECT_FALSE(zone.Contains(29.59 + 0.85, -487.65 + 0.53, -2.56));
  // 같은 위치라도 반대 방향이면 음영 구간으로 보지 않는다.
  EXPECT_FALSE(zone.Contains(2.754, -507.090, -2.504 + 3.14159));
}
