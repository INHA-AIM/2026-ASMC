#include "Frenet/Evaluation/Cost.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

double ComputeLateralCost(const FrenetPath& path, const CostWeights& w,
                          double target_d) {
    const double T  = path.t.empty() ? 0.0 : path.t.back();
    const double d1 = path.d.empty() ? 0.0 : path.d.back();

    // 경로 전체에 걸친 d^2 적분(사다리꼴 적분) - kd*d1^2(끝점만)와 달리 "중간에
    // 얼마나/얼마나 오래 벌어져 있었는지"에 비용을 매긴다(위 kd_path 주석 참고).
    double d_path_integral = 0.0;
    for (size_t i = 0; i + 1 < path.d.size(); ++i) {
        const double dt = path.t[i + 1] - path.t[i];
        const double e0 = path.d[i] - target_d;
        const double e1 = path.d[i + 1] - target_d;
        d_path_integral += 0.5 * (e0 * e0 + e1 * e1) * dt;
    }

    const double terminal_error = d1 - target_d;
    return w.kj * path.jerk_cost_lat + w.kt * T + w.kd * terminal_error * terminal_error
         + w.kd_path * d_path_integral;
}

double ComputeLongitudinalCost(const FrenetPath& path, const CostWeights& w) {
    const double T = path.t.empty() ? 0.0 : path.t.back();

    return w.kj * path.jerk_cost_lon + w.kt * T
         + w.ks * path.delta_s * path.delta_s
         + w.ks_dot * path.delta_s_dot * path.delta_s_dot;
}

double ComputeTotalCost(double cost_lat, double cost_lon, const CostWeights& w) {
    return w.klat * cost_lat + w.klon * cost_lon;
}

void EvaluateCosts(std::vector<FrenetPath>& candidates, const CostWeights& w,
                   double target_d) {
    for (auto& path : candidates) {
        path.cost_lat   = ComputeLateralCost(path, w, target_d);
        path.cost_lon   = ComputeLongitudinalCost(path, w);
        path.cost_total = ComputeTotalCost(path.cost_lat, path.cost_lon, w);
    }
}

void EvaluateApproachCosts(std::vector<FrenetPath>& candidates,double preferred,
    double acceleration_limit,double jerk_limit,double target_d,
    const ApproachCostWeights& weights) {
    const double scale=std::max(1.0,preferred);
    for(auto& path:candidates) {
        path.cost_total=std::numeric_limits<double>::infinity();
        const std::size_t n=path.t.size();
        if(n<2||path.s.size()!=n||path.s_d.size()!=n||path.s_dd.size()!=n||
           path.s_ddd.size()!=n||path.d.size()!=n||preferred<=0||
           acceleration_limit<=0||jerk_limit<=0)continue;
        const double duration=path.t.back()-path.t.front();
        if(duration<=0)continue;
        double speed=0,accel=0,jerk=0,lateral=0;
        for(std::size_t i=1;i<n;++i) {
            const double dt=path.t[i]-path.t[i-1];
            const auto square=[](double x){return x*x;};
            speed+=.5*(square((path.s_d[i-1]-preferred)/scale)+square((path.s_d[i]-preferred)/scale))*dt;
            accel+=.5*(square(path.s_dd[i-1]/acceleration_limit)+square(path.s_dd[i]/acceleration_limit))*dt;
            jerk+=.5*(square(path.s_ddd[i-1]/jerk_limit)+square(path.s_ddd[i]/jerk_limit))*dt;
            lateral+=.5*(square(path.d[i-1]-target_d)+square(path.d[i]-target_d))*dt;
        }
        const double deficit=std::max(0.0,1-(path.s.back()-path.s.front())/(scale*duration));
        path.cost_lat=weights.lateral*lateral/duration;
        path.cost_lon=(weights.speed*speed+weights.acceleration*accel+weights.jerk*jerk)/duration+
            weights.progress*deficit*deficit+weights.horizon/duration;
        path.cost_total=path.cost_lat+path.cost_lon;
    }
}

const FrenetPath* SelectBestPath(const std::vector<FrenetPath>& candidates) {
    const FrenetPath* best = nullptr;

    for (const auto& path : candidates) {
        if (!path.valid) continue;
        if (best == nullptr || path.cost_total < best->cost_total) {
            best = &path;
        }
    }

    return best;
}

const FrenetPath* SelectShortestValidPath(
    const std::vector<FrenetPath>& candidates) {
    const FrenetPath* best = nullptr;
    constexpr double kTimeTieEpsilon = 1e-6;
    for (const auto& path : candidates) {
        if (!path.valid || path.t.empty()) continue;
        if (best == nullptr ||
            path.t.back() + kTimeTieEpsilon < best->t.back() ||
            (std::fabs(path.t.back() - best->t.back()) <= kTimeTieEpsilon &&
             path.cost_total < best->cost_total)) {
            best = &path;
        }
    }
    return best;
}

const FrenetPath* SelectBestStoppingPath(
    const std::vector<FrenetPath>& candidates) {
    const FrenetPath* best = nullptr;
    constexpr double kPositionEpsilon = 1e-6;
    for (const auto& path : candidates) {
        if (!path.valid) continue;
        if (best == nullptr) {
            best = &path;
            continue;
        }
        const double error = std::fabs(path.delta_s);
        const double best_error = std::fabs(best->delta_s);
        if (error + kPositionEpsilon < best_error ||
            (std::fabs(error - best_error) <= kPositionEpsilon &&
             path.cost_total < best->cost_total)) {
            best = &path;
        }
    }
    return best;
}
