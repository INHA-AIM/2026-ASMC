#pragma once

#include "Global/Global.hpp"

void InitMPC(const MPCParams& params);
MPCOutput MPCProcess(const MPCState& ego_state,
                     const ReferencePath& reference);
void ResetMPCOnFailSafe();
void ResetMPCForPlannerStop(double published_steering);
const MPCParams& GetMPCParams();
