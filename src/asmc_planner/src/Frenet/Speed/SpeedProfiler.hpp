#ifndef FRENET_SPEED_SPEED_PROFILER_HPP
#define FRENET_SPEED_SPEED_PROFILER_HPP

// Normal-driving scalar target owner.
//
// This component deliberately knows nothing about ROS, BehaviorCommand, route
// semantics, or planner modes. A caller supplies the already-resolved raw cap.
// A lower cap is a safety constraint and becomes the generator target in the
// same cycle. Only an upward recovery is rate/acceleration/jerk limited here.
// Physical v/a/jerk continuity after a cap drop belongs to the longitudinal
// trajectory generated from the measured ego boundary state, not to this
// scalar target.
struct SpeedProfileInput {
    double raw_speed_cap = 0.0;
    double ego_speed = 0.0;
    double ego_acceleration = 0.0;
    double dt = 0.0;
};

struct SpeedProfileLimits {
    double max_acceleration = 0.0;
    double max_deceleration = 0.0;
    double max_jerk = 0.0;
};

struct SpeedProfileOutput {
    double target_speed = 0.0;
    double target_acceleration = 0.0;
    bool initialized = false;
};

// Physical velocity/acceleration transition. Unlike the scalar cap owner below,
// a lower target never teleports velocity or discards the braking ramp. LOCAL
// trajectory generation reuses the same analytical S-curve through this API.
SpeedProfileOutput AdvancePhysicalSpeed(const SpeedProfileInput& input,
                                        const SpeedProfileLimits& limits);

class SpeedProfiler {
public:
    // Computes one transition without mutating state.  Candidate-aware KEEP
    // planning can therefore try several raw caps and commit only the target
    // that actually generated the selected trajectory.
    SpeedProfileOutput Preview(const SpeedProfileInput& input,
                               const SpeedProfileLimits& limits) const;

    void Commit(const SpeedProfileOutput& output);

    SpeedProfileOutput Update(const SpeedProfileInput& input,
                              const SpeedProfileLimits& limits);

    void Reset();
    bool initialized() const { return initialized_; }
    double speed() const { return profiled_speed_; }
    double acceleration() const { return profiled_acceleration_; }

private:
    bool initialized_ = false;
    double profiled_speed_ = 0.0;
    double profiled_acceleration_ = 0.0;
};

#endif  // FRENET_SPEED_SPEED_PROFILER_HPP
