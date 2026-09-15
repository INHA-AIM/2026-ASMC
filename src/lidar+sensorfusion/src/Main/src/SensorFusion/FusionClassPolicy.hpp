#pragma once

#include <Global/Global.hpp>

#include <algorithm>
#include <cctype>
#include <string>

namespace fusion_class {

inline std::string Normalize(std::string label) {
    std::transform(label.begin(), label.end(), label.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    if (label == "person") return "person";
    if (label == "car" || label == "truck" || label == "bus") return "car";
    return "unknown";
}

inline bool IsConfirmed(const std::string& label) {
    return label == "car" || label == "person";
}

// 정규화 없이 소문자만 맞춘 YOLO 원본 라벨. truck/bus를 보존한다.
inline std::string RawLabel(std::string label) {
    std::transform(label.begin(), label.end(), label.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    return label.empty() ? "unknown" : label;
}

inline void Observe(LiDARObjectInfo& object, const std::string& label,
                    const std::string& yolo_label,
                    int initial_confirmations, int change_confirmations) {
    const std::string observed = Normalize(label);
    object.class_miss_counter = 0;
    object.is_class_locked = false;

    if (observed == "unknown") {
        if (!IsConfirmed(object.str_ObjectClass)) {
            object.str_ObjectClass = "unknown";
            object.str_YoloClass = "unknown";
        }
        object.last_seen_class = "unknown";
        object.continuity_counter = 0;
        return;
    }
    if (object.str_ObjectClass == observed) {
        // 클래스는 그대로여도 원본 라벨은 최신 관측으로 갱신한다
        // (car로 확정된 트랙이 이번 프레임에 truck으로 보이는 경우).
        object.str_YoloClass = RawLabel(yolo_label);
        object.last_seen_class = "unknown";
        object.continuity_counter = 0;
        return;
    }
    if (object.last_seen_class == observed) {
        ++object.continuity_counter;
    } else {
        object.last_seen_class = observed;
        object.continuity_counter = 1;
    }

    const int required = IsConfirmed(object.str_ObjectClass)
        ? std::max(1, change_confirmations)
        : std::max(1, initial_confirmations);
    if (object.continuity_counter >= required) {
        object.str_ObjectClass = observed;
        object.str_YoloClass = RawLabel(yolo_label);
        object.last_seen_class = "unknown";
        object.continuity_counter = 0;
    }
}

inline void Miss(LiDARObjectInfo& object, int miss_grace) {
    ++object.class_miss_counter;
    object.last_seen_class = "unknown";
    object.continuity_counter = 0;
    object.is_class_locked = false;
    if (object.class_miss_counter > std::max(0, miss_grace)) {
        object.str_ObjectClass = "unknown";
        object.str_YoloClass = "unknown";
    }
}

}  // namespace fusion_class
