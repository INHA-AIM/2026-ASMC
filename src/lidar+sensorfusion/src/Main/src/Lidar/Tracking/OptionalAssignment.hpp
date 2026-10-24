#pragma once
#include <Lidar/HungarianAlg/HungarianAlg.hpp>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace optional_assignment {
// Minimize sum(match costs) + limit/2 * (#unmatched tracks + detections).
// Existing edge gate is unchanged. No GT/class/shape prior, new solver or tuning.
// Cost-limit dummy augmentation: pinned sources and limitations in
// handoff/lidar/optional_assignment/README.md.
inline std::vector<int> Solve(const std::vector<std::vector<double>>& costs, double limit) {
    const std::size_t n = costs.size(), m = n ? costs.front().size() : 0;
    if (!std::isfinite(limit) || limit <= 0 || !std::isfinite(2*limit))
        throw std::invalid_argument("finite positive assignment limit required");
    for (const auto& row : costs)
        if (row.size() != m) throw std::invalid_argument("ragged assignment matrix");
    std::vector<int> result(n, -1);
    if (!n || !m) return result;
    std::vector<std::vector<double>> augmented(n+m, std::vector<double>(n+m, limit/2));
    for (std::size_t i=0; i<n; ++i) for (std::size_t j=0; j<m; ++j) {
        const double c = costs[i][j];
        augmented[i][j] = std::isfinite(c) && c>=0 && c<limit ? c : 2*limit;
    }
    for (std::size_t i=n; i<n+m; ++i)
        for (std::size_t j=m; j<n+m; ++j) augmented[i][j]=0;
    AssignmentProblemSolver solver;
    std::vector<int> assigned;
    solver.Solve(augmented, assigned, AssignmentProblemSolver::optimal);
    for (std::size_t i=0; i<n; ++i) {
        const int j=assigned.at(i);
        if (j>=0 && static_cast<std::size_t>(j)<m &&
            std::isfinite(costs[i][j]) && costs[i][j]>=0 && costs[i][j]<limit)
            result[i]=j;
    }
    return result;
}
}
