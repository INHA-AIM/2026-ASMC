#pragma once

#include <cstddef>
#include <vector>

#include "Global/Global.hpp"

// start_time_offset [s]: path.x[0]/y[0]/... 기준 t=0에서 얼마나 앞선 시점부터
// 참조를 시작할지. trajectory 생성 시각과 현재(x0) 시각 사이의 age를 보정하는
// 용도다 (기본값 0.0이면 기존과 동일). offset이 path 길이를 넘어가면 마지막
// 점으로 clip한다 - out-of-range access가 발생하지 않는다.
ReferencePath ToReferencePath(const PlannedPath& path, double destination_dt,
                              int steps, double start_time_offset = 0.0);
