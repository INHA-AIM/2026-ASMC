// Continuous replay AFTER recorded geometric association. Tracking births and
// explicit LiDAR ground-noise recoveries are inputs; class_before is an oracle,
// never a per-frame initial condition. No GPU or ROS master is required.
#include <SensorFusion/FusionClassState.hpp>
#include <SensorFusion/FusionClassPolicy.hpp>
#include <SensorFusion/FusionTimeMatcher.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace {
using Objects = std::vector<LiDARObjectInfo>;
using Stamp = std::uint64_t;
struct Origin { Stamp lidar = 0, yolo = 0, at = 0; };
void require(bool value, const std::string& reason) {
    if (!value) throw std::runtime_error(reason);
}
LiDARObjectInfo read(const YAML::Node& o) {
    LiDARObjectInfo state{};
    state.s32_Id = o["id"].as<int>();
    state.str_ObjectClass = o["class"].as<std::string>();
    state.str_YoloClass = o["raw_class"].as<std::string>();
    state.last_seen_class = o["candidate"].as<std::string>();
    state.continuity_counter = o["confirmations"].as<int>();
    state.class_miss_counter = o["class_misses"].as<int>();
    state.s32_MissCount = o["lidar_misses"].as<int>();
    state.is_class_locked = o["locked"].as<bool>();
    return state;
}
Objects objects(const YAML::Node& rows) {
    Objects output; std::set<int> ids;
    for (const auto& row : rows) {
        auto object = read(row);
        require(ids.insert(object.s32_Id).second, "duplicate object ID");
        output.push_back(object);
    }
    return output;
}
auto state(const LiDARObjectInfo& o) {
    return std::make_tuple(o.str_ObjectClass, o.str_YoloClass, o.last_seen_class,
        o.continuity_counter, o.class_miss_counter, o.is_class_locked);
}
void equal(Objects& actual, const YAML::Node& expected, const std::string& phase) {
    auto oracle = objects(expected);
    require(actual.size() == oracle.size(), phase + " object count");
    for (const auto& object : oracle) {
        auto* candidate = fusion_state::Find(actual, object.s32_Id);
        require(candidate && state(*candidate) == state(object),
                phase + " state mismatch id=" + std::to_string(object.s32_Id));
    }
}
}

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "usage: fusion_continuous_trace_replay TRACE INITIAL CHANGE MISS_GRACE\n";
        return 2;
    }
    std::uint64_t line_number = 0;
    try {
        const int initial = std::stoi(argv[2]), change = std::stoi(argv[3]), grace = std::stoi(argv[4]);
        std::ifstream input(argv[1]); require(bool(input), "cannot open trace");
        std::map<int, LiDARObjectInfo> carried;
        std::map<int, Origin> origins;
        std::map<Stamp, Objects> pending;
        std::vector<std::int64_t> yolo;
        std::set<Stamp> consumed, published_stamps;
        std::set<int> observed;
        Objects current, history, historical;
        Stamp current_stamp = 0, selected_stamp = 0, selected_yolo = 0;
        std::uint64_t current_generation = 0;
        bool frame_open = false, class_open = false, resolved_match = false, after_checked = false;
        std::uint64_t frames = 0, states = 0, matched = 0, dropped = 0, decisions = 0,
            observations = 0, misses = 0, births = 0, resets = 0, carried_checks = 0;
        int summaries = 0;
        std::string line;
        while (std::getline(input, line)) {
            ++line_number;
            const auto row = YAML::Load(line);
            const auto event = row["event"].as<std::string>();
            const Stamp stamp = row["stamp_ns"].as<Stamp>();
            if (event == "class_track_birth" || event == "class_tracking_reset") {
                require(!frame_open, "tracking event inside fusion/publication frame");
                auto object = read(row["object"]);
                if (event == "class_track_birth") {
                    require(!carried.count(object.s32_Id), "duplicate track birth"); ++births;
                } else {
                    require(carried.count(object.s32_Id), "reset before track birth");
                    require(carried.at(object.s32_Id).str_ObjectClass == "noise" ||
                            carried.at(object.s32_Id).str_ObjectClass == "지면노이즈",
                            "reset of a non-noise previous track");
                    require(object.str_ObjectClass != "noise" && object.str_ObjectClass != "지면노이즈",
                            "reset to ground noise is not a recovery"); ++resets;
                }
                carried[object.s32_Id] = object; origins.erase(object.s32_Id);
            } else if (event == "class_lidar_ready") {
                require(!frame_open && !class_open && !resolved_match, "unfinished previous frame");
                require(stamp && !pending.count(stamp) && !published_stamps.count(stamp), "duplicate/zero LiDAR stamp");
                const auto generation = row["generation"].as<std::uint64_t>();
                require(generation > current_generation, "non-increasing processing generation");
                current_generation = generation;
                current.clear();
                for (const auto& recorded : row["objects"]) {
                    auto metadata = read(recorded);
                    require(carried.count(metadata.s32_Id), "missing track birth, cannot replay");
                    fusion_state::Copy(carried.at(metadata.s32_Id), metadata);
                    current.push_back(metadata);
                }
                equal(current, row["objects"], "tracking carry");
                carried_checks += current.size();
                current_stamp = stamp; frame_open = true; history = current;
                pending.emplace(stamp, current);
            } else if (event == "yolo_admitted") {
                require(std::find(yolo.begin(), yolo.end(), stamp) == yolo.end() && !consumed.count(stamp),
                        "duplicate admitted/consumed YOLO");
                yolo.insert(std::upper_bound(yolo.begin(), yolo.end(), stamp), stamp);
            } else if (event == "lidar_startup_discard" || event == "lidar_startup_overflow" || event == "lidar_pending_overflow") {
                require(pending.erase(stamp) == 1, "discard missing LiDAR snapshot");
            } else if (event == "yolo_expired" || event == "yolo_history_overflow") {
                auto it = std::find(yolo.begin(), yolo.end(), stamp);
                require(it != yolo.end(), "prune missing YOLO"); yolo.erase(it);
            } else if (event == "match_decision") {
                require(frame_open && !class_open && !resolved_match && pending.count(stamp), "decision without pending snapshot");
                require(!pending.empty() && pending.begin()->first == stamp, "out-of-order LiDAR decision");
                const auto recorded_yolo = row["yolo_stamps_ns"].as<std::vector<std::int64_t>>();
                require(yolo == recorded_yolo, "YOLO history admission/pruning mismatch");
                const auto decision = fusion_sync::DecideMatch(stamp, yolo,
                    row["waited_ns"].as<std::int64_t>(), 50000000, 200000000);
                const auto action = row["action"].as<std::string>();
                require(action == (decision.action == fusion_sync::MatchAction::Wait ? "wait" :
                    decision.action == fusion_sync::MatchAction::Match ? "match" : "drop"), "matcher action mismatch");
                ++decisions;
                if (decision.action == fusion_sync::MatchAction::Match) {
                    selected_stamp = stamp; selected_yolo = yolo.at(decision.yolo_index);
                    require(selected_yolo == row["selected_yolo_stamp_ns"].as<Stamp>() &&
                        decision.offset_ns == row["offset_ns"].as<std::int64_t>(), "selected pair mismatch");
                    require(consumed.insert(selected_yolo).second, "duplicate YOLO consumption");
                    yolo.erase(yolo.begin() + decision.yolo_index);
                    historical = pending.at(stamp); pending.erase(stamp); resolved_match = true;
                } else if (decision.action == fusion_sync::MatchAction::Drop) {
                    pending.erase(stamp); ++dropped;
                }
            } else if (event == "camera_decode_drop") {
                require(resolved_match && stamp == selected_stamp && !class_open, "decode drop outside selected pair");
                resolved_match = false; ++dropped;
            } else if (event == "class_before") {
                require(resolved_match && !class_open && stamp == selected_stamp &&
                    row["yolo_stamp_ns"].as<Stamp>() == selected_yolo, "unavailable association pair, cannot replay");
                for (auto& old : historical)
                    if (auto* live = fusion_state::Find(current, old.s32_Id)) fusion_state::Copy(*live, old, true);
                equal(historical, row["objects"], "historical carry");
                observed.clear(); class_open = true; after_checked = false;
            } else if (event == "class_observation" || event == "class_miss") {
                require(class_open && !after_checked && stamp == selected_yolo, "observation outside matched pair");
                const int id = row["id"].as<int>();
                auto* object = fusion_state::Find(historical, id);
                require(object && observed.insert(id).second, "missing/duplicate object observation");
                if (event == "class_observation") {
                    fusion_class::Observe(*object, row["label"].as<std::string>(), row["raw_label"].as<std::string>(), initial, change);
                    ++observations;
                } else { fusion_class::Miss(*object, grace); ++misses; }
            } else if (event == "class_after") {
                require(class_open && !after_checked && stamp == selected_stamp &&
                    row["yolo_stamp_ns"].as<Stamp>() == selected_yolo, "after outside matched pair");
                for (const auto& object : historical)
                    require(object.s32_MissCount > 0 || object.str_ObjectClass == "noise" ||
                        object.str_ObjectClass == "지면노이즈" || observed.count(object.s32_Id),
                        "missing geometric observation, cannot replay");
                equal(historical, row["objects"], "continuous class_after");
                for (const auto& old : historical) {
                    if (auto* live = fusion_state::Find(current, old.s32_Id)) {
                        fusion_state::Copy(old, *live, true);
                        origins[old.s32_Id] = {selected_stamp, selected_yolo, current_stamp};
                    }
                }
                states += historical.size(); after_checked = true; ++matched;
            } else if (event == "class_current") {
                require(class_open && after_checked && stamp == current_stamp &&
                    row["yolo_stamp_ns"].as<Stamp>() == selected_yolo, "current outside matched pair");
                equal(current, row["objects"], "transfer to current");
                fusion_state::Persist(current, history);
                class_open = resolved_match = after_checked = false;
            } else if (event == "class_publish_input") {
                require(frame_open && !class_open && !resolved_match && stamp == current_stamp, "incomplete publication frame");
                require(row["generation"].as<std::uint64_t>() == current_generation, "publication generation mismatch");
                equal(current, row["objects"], "publication input");
                equal(history, row["objects"], "persisted tracking history");
                require(row["origins"].size() == current.size(), "missing publication provenance");
                std::set<int> ids;
                for (const auto& origin : row["origins"]) {
                    const int id = origin["id"].as<int>();
                    require(fusion_state::Find(current, id) && ids.insert(id).second, "duplicate/missing provenance ID");
                    const auto expected = origins.count(id) ? origins.at(id) : Origin{};
                    require(origin["source_lidar_stamp_ns"].as<Stamp>() == expected.lidar &&
                        origin["source_yolo_stamp_ns"].as<Stamp>() == expected.yolo &&
                        origin["applied_at_lidar_stamp_ns"].as<Stamp>() == expected.at &&
                        origin["updated_in_frame"].as<bool>() == (expected.at == current_stamp), "publication provenance mismatch");
                }
                for (const auto& object : history) carried[object.s32_Id] = object;
                require(published_stamps.insert(stamp).second, "duplicate publication stamp");
                ++frames; frame_open = false;
            } else if (event == "fusion_summary") {
                ++summaries;
                require(!frame_open && !class_open && !resolved_match, "unfinished trace");
                require(row["matched"].as<std::uint64_t>() == matched && row["dropped"].as<std::uint64_t>() == dropped &&
                    row["pending_lidar"].as<std::size_t>() == pending.size() && row["pending_yolo"].as<std::size_t>() == yolo.size(), "incomplete final accounting");
            }
        }
        require(summaries == 1 && frames && matched && !frame_open && !class_open && !resolved_match, "missing complete final summary");
        std::cout << "{\"pass\":true,\"published_frames\":" << frames << ",\"matched\":" << matched
            << ",\"dropped\":" << dropped << ",\"states\":" << states << ",\"carried_checks\":" << carried_checks
            << ",\"matcher_decisions\":" << decisions << ",\"observations\":" << observations
            << ",\"misses\":" << misses << ",\"births\":" << births << ",\"noise_recoveries\":" << resets << "}\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "line " << line_number << ": " << error.what() << '\n'; return 1;
    }
}
