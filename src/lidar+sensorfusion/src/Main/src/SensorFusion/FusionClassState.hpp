#pragma once

#include <Global/Global.hpp>
#include <algorithm>
#include <vector>

// Keep tracking geometry/velocity out of classification propagation. The
// unlock argument preserves the existing distinction between fusion copying
// (always unlocked) and persisting/carrying tracking history (copy the flag).
namespace fusion_state {
inline void Copy(const LiDARObjectInfo& source, LiDARObjectInfo& target,
                 bool unlock = false) {
    target.str_ObjectClass = source.str_ObjectClass;
    target.str_YoloClass = source.str_YoloClass;
    target.last_seen_class = source.last_seen_class;
    target.continuity_counter = source.continuity_counter;
    target.class_miss_counter = source.class_miss_counter;
    target.is_class_locked = unlock ? false : source.is_class_locked;
}

inline LiDARObjectInfo* Find(std::vector<LiDARObjectInfo>& objects, int32 id) {
    auto it = std::find_if(objects.begin(), objects.end(),
        [id](const LiDARObjectInfo& object) { return object.s32_Id == id; });
    return it == objects.end() ? nullptr : &*it;
}

inline void Persist(const std::vector<LiDARObjectInfo>& current,
                    std::vector<LiDARObjectInfo>& history) {
    for (const auto& object : current)
        if (auto* previous = Find(history, object.s32_Id)) Copy(object, *previous);
}
}  // namespace fusion_state
