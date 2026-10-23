#pragma once
// Munkres optimal assignment used by the tracker.
// Ported from Main src/Lidar/HungarianAlg/HungarianAlg.{hpp,cpp} at 8f93c22e.
// Only assignmentoptimal is reachable in Main (every TMethod maps to it), so
// assignmentsuboptimal1/2 are not ported.
//
// DistMatrix[i][j]: row i = previous track, column j = detection (Main's header
// comment calling N "columns (tracks)" is mislabelled). Assignment[i] is the
// detection of track i or -1. Every row is assigned whenever N <= M, including
// gated (e.g. 9999) entries: callers must reject gated pairs themselves.

#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

class AssignmentProblemSolver {
 public:
  enum TMethod { optimal, many_forbidden_assignments, without_forbidden_assignments };
  AssignmentProblemSolver() = default;
  // Returns the total cost of the assignment (all non -1 pairs).
  double Solve(std::vector<std::vector<double>>& DistMatrix, std::vector<int>& Assignment, TMethod Method = optimal);

 private:
  void assignmentoptimal(int* assignment, double* cost, double* distMatrix, int nOfRows, int nOfColumns);
  void buildassignmentvector(int* assignment, bool* starMatrix, int nOfRows, int nOfColumns);
  void computeassignmentcost(int* assignment, double* cost, double* distMatrix, int nOfRows);
  void step2a(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
              bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step2b(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
              bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step3(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
             bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step4(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
             bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
  void step5(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
             bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
};

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
