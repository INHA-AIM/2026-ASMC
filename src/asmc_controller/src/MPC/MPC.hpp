#pragma once

#include "Global/Global.hpp"
#include <limits>

void InitMPC(const MPCParams& params);
MPCOutput MPCProcess(const MPCState& ego_state,
                     const ReferencePath& reference,
                     double acceleration_limit = std::numeric_limits<double>::quiet_NaN());
void ResetMPCOnFailSafe();
void ResetMPCForPlannerStop(double published_steering);
const MPCParams& GetMPCParams();

