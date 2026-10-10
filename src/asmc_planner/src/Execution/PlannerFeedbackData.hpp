#pragma once

#include <cstdint>

namespace behavior_planner {

enum class PlannerExecutionState : uint8_t {
  IDLE = 0,
  PREPARING = 1,
  WAITING = 2,
  READY = 3,
  COMMITTED = 4,
  EXECUTING = 5,
  COMPLETED = 6,
  FAILED = 7,
};

struct PlannerFeedbackData {
  bool valid = false;
  uint32_t command_id = 0;
  PlannerExecutionState execution_state = PlannerExecutionState::IDLE;
  uint8_t result_code = 0;
};

}  // namespace behavior_planner
