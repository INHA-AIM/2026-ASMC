// 기본적인 헤더 파일 및 planner 전역 입,출력 구조체 들어가는 곳
//
// refactor/frenet-structure-v1 R2: 실제 타입 정의는 책임별로
// Common/BehaviorTypes.hpp, Common/ObjectTypes.hpp, Common/PlannerCommand.hpp,
// Common/FrenetTypes.hpp, Common/CartesianTypes.hpp로 옮겼다. 기존 include
// 경로("frenet_planner/global/global.hpp")를 쓰는 코드가 많아 이 파일은
// 당분간 umbrella include로 남긴다 - API/동작 변화 없음.

#ifndef PLANNER_GLOBAL_HPP
#define PLANNER_GLOBAL_HPP

// 이 파일이 예전에 <vector>/<cmath>/<limits>/<stdexcept>를 직접 포함했고,
// 다수의 하위 파일이 그 transitive include에 기대어 std::sqrt/M_PI 등을
// 직접 include 없이 써 왔다. Common/*.hpp로 타입을 옮긴 뒤에도 이 표준
// 헤더들은 그대로 남겨 기존 컴파일 단위가 깨지지 않게 한다.
#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "frenet_planner/Common/BehaviorTypes.hpp"
#include "frenet_planner/Common/ObjectTypes.hpp"
#include "frenet_planner/Common/PlannerCommand.hpp"
#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/CartesianTypes.hpp"

#endif
