#ifndef FRENET_GEOMETRY_OBJECT_GEOMETRY_SYNTHESIS_HPP
#define FRENET_GEOMETRY_OBJECT_GEOMETRY_SYNTHESIS_HPP

#include <vector>

#include "frenet_planner/Common/ObjectTypes.hpp"

// object의 x/y/z/heading/width/length/height로부터 OBB corner geometry를
// 합성해 같은 object에 채워 넣는다 - 호출자가 authoritative corner
// geometry(예: perception의 obb_corners)를 제공하지 않을 때만 쓴다.
// vx/vy/id/type 등 motion 필드는 그대로 둔다.
TrackedObject MakeLegacyTrackedObject(const TrackedObject& object);

std::vector<TrackedObject> MakeLegacyTrackedObjects(
    const std::vector<TrackedObject>& objects);

#endif  // FRENET_GEOMETRY_OBJECT_GEOMETRY_SYNTHESIS_HPP
