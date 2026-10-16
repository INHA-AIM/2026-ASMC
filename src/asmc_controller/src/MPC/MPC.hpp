#pragma once

#include "Global/Global.hpp"
#include <limits>

void InitMPC(const MPCParams& params);
MPCOutput MPCProcess(const MPCState& ego_state,
                     const ReferencePath& reference,
                     double acceleration_limit = std::numeric_limits<double>::quiet_NaN());
// Returns false when no usable geometry exists; caller retains scalar fallback.
bool MPCPlannerStopSteering(const MPCState& ego, double brake_pedal,
                            double& steering);
void ResetMPCOnFailSafe();
void ResetMPCForPlannerStop(double published_steering);
const MPCParams& GetMPCParams();

