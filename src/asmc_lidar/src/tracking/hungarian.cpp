// Munkres optimal assignment.
// Ported from Main src/Lidar/HungarianAlg/HungarianAlg.cpp at 8f93c22e. Step
// structure (mutually recursive step2a..step5), column-major indexing
// row + nOfRows*col and every comparison are kept; malloc/calloc buffers are
// replaced by zero-initialised std::vector / bool arrays.
#include <asmc_lidar/pipeline/hungarian.hpp>

#include <cfloat>
#include <memory>

namespace asmc {
namespace lidar {
namespace pipeline {

// Ported from Main HungarianAlg.cpp:13-49 (8f93c22e)
double AssignmentProblemSolver::Solve(std::vector<std::vector<double>>& DistMatrix, std::vector<int>& Assignment,
                                      TMethod Method) {
  int N = (int)DistMatrix.size();  // rows (tracks)
  // Deviation: Main reads DistMatrix[0] unconditionally (undefined for N=0; the
  // tracker never calls it then). N=0 returns an empty assignment instead.
  if (N == 0) {
    Assignment.clear();
    return 0.0;
  }
  int M = (int)DistMatrix[0].size();  // columns (detections)

  std::vector<int> assignment(N);
  std::vector<double> distIn(static_cast<std::size_t>(N) * M);

  double cost;
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < M; j++) {
      distIn[i + N * j] = DistMatrix[i][j];
    }
  }
  switch (Method) {
    case optimal: assignmentoptimal(assignment.data(), &cost, distIn.data(), N, M); break;
    case many_forbidden_assignments: assignmentoptimal(assignment.data(), &cost, distIn.data(), N, M); break;
    case without_forbidden_assignments: assignmentoptimal(assignment.data(), &cost, distIn.data(), N, M); break;
  }

  Assignment.clear();
  for (int x = 0; x < N; x++) {
    Assignment.push_back(assignment[x]);
  }
  return cost;
}

// Ported from Main HungarianAlg.cpp:53-211 (8f93c22e)
void AssignmentProblemSolver::assignmentoptimal(int* assignment, double* cost, double* distMatrixIn, int nOfRows,
                                                int nOfColumns) {
  double* distMatrixTemp;
  double* distMatrixEnd;
  double* columnEnd;
  double value;
  double minValue;
  int nOfElements;
  int minDim;
  int row;
  int col;

  // Init
  *cost = 0;
  for (row = 0; row < nOfRows; row++) {
    assignment[row] = -1;
  }

  nOfElements = nOfRows * nOfColumns;
  std::vector<double> distMatrixBuffer(static_cast<std::size_t>(nOfElements));
  double* distMatrix = distMatrixBuffer.data();
  distMatrixEnd = distMatrix + nOfElements;

  for (row = 0; row < nOfElements; row++) {
    value = distMatrixIn[row];
    // Main only comments on negative elements; no check.
    distMatrix[row] = value;
  }

  std::unique_ptr<bool[]> coveredColumnsBuffer(new bool[nOfColumns]());
  std::unique_ptr<bool[]> coveredRowsBuffer(new bool[nOfRows]());
  std::unique_ptr<bool[]> starMatrixBuffer(new bool[nOfElements]());
  std::unique_ptr<bool[]> primeMatrixBuffer(new bool[nOfElements]());
  std::unique_ptr<bool[]> newStarMatrixBuffer(new bool[nOfElements]());  // used in step4
  bool* coveredColumns = coveredColumnsBuffer.get();
  bool* coveredRows = coveredRowsBuffer.get();
  bool* starMatrix = starMatrixBuffer.get();
  bool* primeMatrix = primeMatrixBuffer.get();
  bool* newStarMatrix = newStarMatrixBuffer.get();

  /* preliminary steps */
  if (nOfRows <= nOfColumns) {
    minDim = nOfRows;
    for (row = 0; row < nOfRows; row++) {
      /* find the smallest element in the row */
      distMatrixTemp = distMatrix + row;
      minValue = *distMatrixTemp;
      distMatrixTemp += nOfRows;
      while (distMatrixTemp < distMatrixEnd) {
        value = *distMatrixTemp;
        if (value < minValue) {
          minValue = value;
        }
        distMatrixTemp += nOfRows;
      }
      /* subtract the smallest element from each element of the row */
      distMatrixTemp = distMatrix + row;
      while (distMatrixTemp < distMatrixEnd) {
        *distMatrixTemp -= minValue;
        distMatrixTemp += nOfRows;
      }
    }
    /* Steps 1 and 2a */
    for (row = 0; row < nOfRows; row++) {
      for (col = 0; col < nOfColumns; col++) {
        if (distMatrix[row + nOfRows * col] == 0) {
          if (!coveredColumns[col]) {
            starMatrix[row + nOfRows * col] = true;
            coveredColumns[col] = true;
            break;
          }
        }
      }
    }
  } else /* if(nOfRows > nOfColumns) */ {
    minDim = nOfColumns;
    for (col = 0; col < nOfColumns; col++) {
      /* find the smallest element in the column */
      distMatrixTemp = distMatrix + nOfRows * col;
      columnEnd = distMatrixTemp + nOfRows;
      minValue = *distMatrixTemp++;
      while (distMatrixTemp < columnEnd) {
        value = *distMatrixTemp++;
        if (value < minValue) {
          minValue = value;
        }
      }
      /* subtract the smallest element from each element of the column */
      distMatrixTemp = distMatrix + nOfRows * col;
      while (distMatrixTemp < columnEnd) {
        *distMatrixTemp++ -= minValue;
      }
    }
    /* Steps 1 and 2a */
    for (col = 0; col < nOfColumns; col++) {
      for (row = 0; row < nOfRows; row++) {
        if (distMatrix[row + nOfRows * col] == 0) {
          if (!coveredRows[row]) {
            starMatrix[row + nOfRows * col] = true;
            coveredColumns[col] = true;
            coveredRows[row] = true;
            break;
          }
        }
      }
    }

    for (row = 0; row < nOfRows; row++) {
      coveredRows[row] = false;
    }
  }
  /* move to step 2b */
  step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows,
         nOfColumns, minDim);
  /* compute cost and remove invalid assignments */
  computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);
}

// Ported from Main HungarianAlg.cpp:215-229 (8f93c22e)
void AssignmentProblemSolver::buildassignmentvector(int* assignment, bool* starMatrix, int nOfRows, int nOfColumns) {
  int row, col;
  for (row = 0; row < nOfRows; row++) {
    for (col = 0; col < nOfColumns; col++) {
      if (starMatrix[row + nOfRows * col]) {
        assignment[row] = col;
        break;
      }
    }
  }
}

// Ported from Main HungarianAlg.cpp:233-244 (8f93c22e)
void AssignmentProblemSolver::computeassignmentcost(int* assignment, double* cost, double* distMatrix, int nOfRows) {
  int row, col;
  for (row = 0; row < nOfRows; row++) {
    col = assignment[row];
    if (col >= 0) {
      *cost += distMatrix[row + nOfRows * col];
    }
  }
}

// Ported from Main HungarianAlg.cpp:249-269 (8f93c22e)
void AssignmentProblemSolver::step2a(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix,
                                     bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                                     int nOfColumns, int minDim) {
  bool *starMatrixTemp, *columnEnd;
  int col;
  /* cover every column containing a starred zero */
  for (col = 0; col < nOfColumns; col++) {
    starMatrixTemp = starMatrix + nOfRows * col;
    columnEnd = starMatrixTemp + nOfRows;
    while (starMatrixTemp < columnEnd) {
      if (*starMatrixTemp++) {
        coveredColumns[col] = true;
        break;
      }
    }
  }
  /* move to step 3 */
  step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows,
         nOfColumns, minDim);
}

// Ported from Main HungarianAlg.cpp:274-296 (8f93c22e)
void AssignmentProblemSolver::step2b(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix,
                                     bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                                     int nOfColumns, int minDim) {
  int col, nOfCoveredColumns;
  /* count covered columns */
  nOfCoveredColumns = 0;
  for (col = 0; col < nOfColumns; col++) {
    if (coveredColumns[col]) {
      nOfCoveredColumns++;
    }
  }
  if (nOfCoveredColumns == minDim) {
    /* algorithm finished */
    buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
  } else {
    /* move to step 3 */
    step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows,
          nOfColumns, minDim);
  }
}

// Ported from Main HungarianAlg.cpp:301-345 (8f93c22e)
void AssignmentProblemSolver::step3(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix,
                                    bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                                    int nOfColumns, int minDim) {
  bool zerosFound;
  int row, col, starCol;
  zerosFound = true;
  while (zerosFound) {
    zerosFound = false;
    for (col = 0; col < nOfColumns; col++) {
      if (!coveredColumns[col]) {
        for (row = 0; row < nOfRows; row++) {
          if ((!coveredRows[row]) && (distMatrix[row + nOfRows * col] == 0)) {
            /* prime zero */
            primeMatrix[row + nOfRows * col] = true;
            /* find starred zero in current row */
            for (starCol = 0; starCol < nOfColumns; starCol++)
              if (starMatrix[row + nOfRows * starCol]) {
                break;
              }
            if (starCol == nOfColumns) /* no starred zero found */
            {
              /* move to step 4 */
              step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows,
                    nOfRows, nOfColumns, minDim, row, col);
              return;
            } else {
              coveredRows[row] = true;
              coveredColumns[starCol] = false;
              zerosFound = true;
              break;
            }
          }
        }
      }
    }
  }
  /* move to step 5 */
  step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows,
        nOfColumns, minDim);
}

// Ported from Main HungarianAlg.cpp:350-408 (8f93c22e)
void AssignmentProblemSolver::step4(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix,
                                    bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                                    int nOfColumns, int minDim, int row, int col) {
  int n, starRow, starCol, primeRow, primeCol;
  int nOfElements = nOfRows * nOfColumns;
  /* generate temporary copy of starMatrix */
  for (n = 0; n < nOfElements; n++) {
    newStarMatrix[n] = starMatrix[n];
  }
  /* star current zero */
  newStarMatrix[row + nOfRows * col] = true;
  /* find starred zero in current column */
  starCol = col;
  for (starRow = 0; starRow < nOfRows; starRow++) {
    if (starMatrix[starRow + nOfRows * starCol]) {
      break;
    }
  }
  while (starRow < nOfRows) {
    /* unstar the starred zero */
    newStarMatrix[starRow + nOfRows * starCol] = false;
    /* find primed zero in current row */
    primeRow = starRow;
    for (primeCol = 0; primeCol < nOfColumns; primeCol++) {
      if (primeMatrix[primeRow + nOfRows * primeCol]) {
        break;
      }
    }
    /* star the primed zero */
    newStarMatrix[primeRow + nOfRows * primeCol] = true;
    /* find starred zero in current column */
    starCol = primeCol;
    for (starRow = 0; starRow < nOfRows; starRow++) {
      if (starMatrix[starRow + nOfRows * starCol]) {
        break;
      }
    }
  }
  /* use temporary copy as new starMatrix */
  /* delete all primes, uncover all rows */
  for (n = 0; n < nOfElements; n++) {
    primeMatrix[n] = false;
    starMatrix[n] = newStarMatrix[n];
  }
  for (n = 0; n < nOfRows; n++) {
    coveredRows[n] = false;
  }
  /* move to step 2a */
  step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows,
         nOfColumns, minDim);
}

// Ported from Main HungarianAlg.cpp:413-460 (8f93c22e)
void AssignmentProblemSolver::step5(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix,
                                    bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                                    int nOfColumns, int minDim) {
  double h, value;
  int row, col;
  /* find smallest uncovered element h */
  h = DBL_MAX;
  for (row = 0; row < nOfRows; row++) {
    if (!coveredRows[row]) {
      for (col = 0; col < nOfColumns; col++) {
        if (!coveredColumns[col]) {
          value = distMatrix[row + nOfRows * col];
          if (value < h) {
            h = value;
          }
        }
      }
    }
  }
  /* add h to each covered row */
  for (row = 0; row < nOfRows; row++) {
    if (coveredRows[row]) {
      for (col = 0; col < nOfColumns; col++) {
        distMatrix[row + nOfRows * col] += h;
      }
    }
  }
  /* subtract h from each uncovered column */
  for (col = 0; col < nOfColumns; col++) {
    if (!coveredColumns[col]) {
      for (row = 0; row < nOfRows; row++) {
        distMatrix[row + nOfRows * col] -= h;
      }
    }
  }
  /* move to step 3 */
  step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows,
        nOfColumns, minDim);
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
