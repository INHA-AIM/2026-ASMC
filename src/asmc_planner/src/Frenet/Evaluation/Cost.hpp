#ifndef FRENET_COST_HPP
#define FRENET_COST_HPP

#include <vector>

#include "Frenet/Geometry/FrenetTypes.hpp"

struct CostWeights {
    double kj;      // jerk 적분항 가중치 (공통)
    double kt;      // 시간항 가중치 (공통, g(T)=T)
    double kd;      // lateral 종료위치 가중치 (kd*d1^2)
    double kd_path;
    double ks;      // longitudinal 목표위치 가중치 (following/merging/stopping, ks*delta_s^2)
    double ks_dot;  // longitudinal 목표속도 가중치 (velocity keeping, ks_dot*delta_s_dot^2)
    double klat;    // Sec.VI: C_tot = klat*C_lat + klon*C_lon
    double klon;
};

double ComputeLateralCost(const FrenetPath& path, const CostWeights& w,
                          double target_d = 0.0);
double ComputeLongitudinalCost(const FrenetPath& path, const CostWeights& w);
double ComputeTotalCost(double cost_lat, double cost_lon, const CostWeights& w);
void EvaluateCosts(std::vector<FrenetPath>& candidates, const CostWeights& w,
                   double target_d = 0.0);

// Time-normalized approach objective, separate from the validated STOP/AVOID
// weights. All candidates must pass safety first. Speed is a preference, not
// a ceiling; progress and acceleration/jerk are compared over equal units.
struct ApproachCostWeights {
    double speed = 1.0;
    double acceleration = 0.2;
    double jerk = 0.1;
    double progress = 0.2;
    double lateral = 0.2;
    double horizon = 0.05;
};
void EvaluateApproachCosts(std::vector<FrenetPath>& candidates,
    double preferred_speed, double acceleration_limit, double jerk_limit,
    double target_d, const ApproachCostWeights& weights = {});

const FrenetPath* SelectBestPath(const std::vector<FrenetPath>& candidates);

// 안전 필터를 통과한 후보 중 기동 완료시간이 가장 짧은 후보를 고른다.
// 같은 시간 후보끼리만 기존 총비용으로 비교한다.
const FrenetPath* SelectShortestValidPath(
    const std::vector<FrenetPath>& candidates);

// STOP 후보는 충돌/곡률/동역학 필터를 통과한(valid) 후보 중 목표 위치 오차가
// 가장 작은 것을 먼저 고르고, 같은 위치 후보끼리는 기존 총비용으로 비교한다.
const FrenetPath* SelectBestStoppingPath(
    const std::vector<FrenetPath>& candidates);

#endif
