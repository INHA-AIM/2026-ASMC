#pragma once
#include "Frenet/Common/CartesianTypes.hpp"
namespace asmc_planner {
struct FrenetResult {
  bool success;
  const CartesianPath& path;
};
template<class Calculation>
FrenetResult CalculateFrenet(const CartesianPath& path, Calculation calculate) {
  return {calculate(), path};
}
}
