// SamplingRange -> 실제 값 목록, 시간 격자 등 LateralGenerator/
// LongitudinalGenerator가 공통으로 쓰는 순수 샘플링 유틸.

#ifndef FRENET_GENERATOR_SAMPLING_UTILS_HPP
#define FRENET_GENERATOR_SAMPLING_UTILS_HPP

#include <vector>

#include "Frenet/Generator/GeneratorConfig.hpp"

// SamplingRange(min,max,step) -> 실제 후보값 목록
std::vector<double> SampleRange(const SamplingRange& r);

// 0부터 T까지 dt 간격의 시간 샘플 (끝점 T는 항상 정확히 포함)
std::vector<double> SampleTimes(double T, double dt);

#endif  // FRENET_GENERATOR_SAMPLING_UTILS_HPP
