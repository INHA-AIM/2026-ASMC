#ifndef FRENET_LEGACY_OBJECT_ADAPTER_HPP
#define FRENET_LEGACY_OBJECT_ADAPTER_HPP

#include <vector>

#include "frenet_planner/Common/ObjectTypes.hpp"

// object의 x/y/z/heading/width/length/height로부터 OBB corner geometry를
// 합성해 같은 object에 채워 넣는다(legacy UDP는 corner geometry를 직접
// 제공하지 않는다). vx/vy/id/type 등 motion 필드는 그대로 둔다.
TrackedObject MakeLegacyTrackedObject(const TrackedObject& object);

std::vector<TrackedObject> MakeLegacyTrackedObjects(
    const std::vector<TrackedObject>& objects);

#endif  // FRENET_LEGACY_OBJECT_ADAPTER_HPP
