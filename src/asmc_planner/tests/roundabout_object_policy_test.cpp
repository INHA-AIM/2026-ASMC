#include "Frenet/Maneuver/Merge/RoundaboutObjectPolicy.hpp"
#include <cassert>
#include <iostream>
int main() {
  using roundabout_policy::Keep;
  assert(Keep("vehicle"));assert(Keep("unknown"));assert(Keep(""));
  assert(Keep("unrecognized_future_group"));
  assert(!Keep("static_obstacle"));assert(!Keep("dynamic_obstacle"));
  std::cout<<"PASS vehicle, unknown, empty, unrecognized preserved; confirmed nonvehicle excluded\n";
}
