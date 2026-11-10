#pragma once
#include "local/planner.hpp"
namespace local_planning {
// Bounded3km/h simulation recovery. Observed road exteriors and actual OBB
// sweeps remain hard; internal occlusion holes are explicitly soft evidence.
bool BuildRecoveryPath(const Scene &, const Config &, Result &);
}
