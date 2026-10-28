#pragma once
// LOCAL adapter only: commands are held between actual issue times. Sampling
// this timeline is not a future sensor observation or a watchdog extension.
#include "Global/Global.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

namespace asmc_local {
class ExecutionHistory {
  struct Event { double time; MPCControl command; };
  std::deque<Event> events_;
  bool pending_ = false;
public:
  struct Snapshot {
    std::vector<double> steering;
    MPCControl previous{};
    double elapsed_s = 0;
    bool restart_seed = false;
  };
  void Clear() { events_.clear(); pending_ = false; }
  void Pending() { pending_ = true; }
  void Issued(double time, MPCControl command) {
    if (!std::isfinite(time) || !std::isfinite(command.delta) ||
        !std::isfinite(command.accel))
      throw std::invalid_argument("nonfinite LOCAL command history");
    if (!events_.empty() && time < events_.back().time) Clear();
    if (!events_.empty() && time == events_.back().time) events_.pop_back();
    events_.push_back({time, command});
    while (events_.size() > 128 ||
           (events_.size() > 2 && time - events_[1].time > 2.))
      events_.pop_front();
    pending_ = false;
  }
  Snapshot At(double time, double dt, double delay) const {
    if (!std::isfinite(time + dt + delay) || dt <= 0 || delay < 0 ||
        delay / dt > 127)
      throw std::invalid_argument("invalid LOCAL history grid");
    Snapshot out;
    if (events_.empty()) return out;
    if (time < events_.back().time)
      throw std::invalid_argument("LOCAL steady clock reversal");
    out.previous = events_.back().command;
    out.elapsed_s = time - events_.back().time;
    out.restart_seed = pending_ || out.elapsed_s > 1.5 * dt;
    const int count = static_cast<int>(std::ceil(delay / dt)) + 1;
    // The common delay model indexes -1 at one dt before solve time.
    // Preserve first-command fallback when no earlier command is known.
    for (int i = count; i > 0; --i) {
      const double sample = time - i * dt;
      double steering = events_.front().command.delta;
      for (const auto &event : events_) {
        if (event.time > sample + 1e-9) break;
        steering = event.command.delta;
      }
      out.steering.push_back(steering);
    }
    return out;
  }
};
// g_mpc is already process-local. Only the LOCAL node calls this boundary;
// NORMAL MPCProcess and its fixed-cadence semantics remain untouched.
inline void PrepareExecution(const ExecutionHistory::Snapshot &snapshot) {
  if (snapshot.steering.empty()) return;
  g_mpc.steering_history = snapshot.steering;
  g_mpc.last_control = snapshot.previous;
  if (snapshot.restart_seed) g_mpc.warm_start.clear();
}
} // namespace asmc_local
