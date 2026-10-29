#pragma once
#include <string>
namespace roundabout_policy {
// Competition-scoped semantic prior. Only explicitly classified non-vehicles
// are excluded; an absent/unknown/unrecognized label may be an unseen vehicle.
// Called exclusively by the verified ROUNDABOUT command, never ordinary AVOID.
inline bool Keep(const std::string& group) {
  return group != "static_obstacle" && group != "dynamic_obstacle";
}
}
