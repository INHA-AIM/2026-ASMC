#pragma once
#include "Planning/Planner.hpp"
#include "Execution/RequestedIntent.hpp"
#include <mutex>
namespace asmc_planner {
template<class RuntimePort>
void Planner::ReceiveIntent(RuntimePort& port, const RequestedIntent& request) {
  // Storage only. Validation, acceptance and lifecycle remain on the existing
  // 20 Hz timer, preserving independent requested and accepted command IDs.
  std::lock_guard<std::mutex> lock(port.input_mutex_);
  port.latest_v2_command_ = request.command;
  port.has_v2_command_ = true;
}
}
