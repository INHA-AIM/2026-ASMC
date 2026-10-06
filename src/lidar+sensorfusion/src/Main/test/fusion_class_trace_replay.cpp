// Replay recorded IoU observations through the actual classification policy.
// This isolates state transitions from GPU delivery and LiDAR frame selection;
// it does not replace the runtime parity gate or re-run geometric association.
#include <SensorFusion/FusionClassPolicy.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <cstdint>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "usage: fusion_class_trace_replay TRACE INITIAL CHANGE MISS_GRACE\n";
        return 2;
    }
    try {
        const int initial = std::stoi(argv[2]), change = std::stoi(argv[3]), miss = std::stoi(argv[4]);
        std::ifstream input(argv[1]);
        if (!input) throw std::runtime_error("cannot open trace");
        std::map<int, LiDARObjectInfo> objects;
        bool active = false;
        std::uint64_t stamp = 0, yolo_stamp = 0, frames = 0, states = 0, observations = 0, misses = 0;
        std::uint64_t summary_matched = 0;
        int summaries = 0;
        std::string line;
        while (std::getline(input, line)) {
            const auto row = YAML::Load(line);
            const std::string event = row["event"].as<std::string>();
            if (event == "class_before") {
                if (active) throw std::runtime_error("unfinished state frame");
                active = true;
                stamp = row["stamp_ns"].as<std::uint64_t>();
                yolo_stamp = row["yolo_stamp_ns"].as<std::uint64_t>();
                objects.clear();
                for (const auto& obj : row["objects"]) {
                    LiDARObjectInfo state;
                    state.str_ObjectClass = obj["class"].as<std::string>();
                    state.str_YoloClass = obj["raw_class"].as<std::string>();
                    state.last_seen_class = obj["candidate"].as<std::string>();
                    state.continuity_counter = obj["confirmations"].as<int>();
                    state.class_miss_counter = obj["class_misses"].as<int>();
                    state.s32_MissCount = obj["lidar_misses"].as<int>();
                    if (!objects.emplace(obj["id"].as<int>(), state).second)
                        throw std::runtime_error("duplicate object ID in state frame");
                }
            } else if (event == "class_observation" || event == "class_miss") {
                if (!active || row["stamp_ns"].as<std::uint64_t>() != yolo_stamp)
                    throw std::runtime_error("observation outside its YOLO frame");
                auto& state = objects.at(row["id"].as<int>());
                if (event == "class_observation") {
                    fusion_class::Observe(state, row["label"].as<std::string>(),
                        row["raw_label"].as<std::string>(), initial, change);
                    ++observations;
                } else {
                    fusion_class::Miss(state, miss);
                    ++misses;
                }
            } else if (event == "class_after") {
                if (!active || row["stamp_ns"].as<std::uint64_t>() != stamp ||
                    row["yolo_stamp_ns"].as<std::uint64_t>() != yolo_stamp ||
                    row["objects"].size() != objects.size())
                    throw std::runtime_error("mismatched state frame");
                for (const auto& obj : row["objects"]) {
                    const auto& state = objects.at(obj["id"].as<int>());
                    if (state.str_ObjectClass != obj["class"].as<std::string>() ||
                        state.str_YoloClass != obj["raw_class"].as<std::string>() ||
                        state.last_seen_class != obj["candidate"].as<std::string>() ||
                        state.continuity_counter != obj["confirmations"].as<int>() ||
                        state.class_miss_counter != obj["class_misses"].as<int>())
                        throw std::runtime_error("classification transition mismatch at " + std::to_string(stamp));
                    ++states;
                }
                ++frames;
                active = false;
            } else if (event == "fusion_summary") {
                ++summaries;
                summary_matched = row["matched"].as<std::uint64_t>();
            }
        }
        if (active || frames == 0 || summaries != 1 || frames != summary_matched)
            throw std::runtime_error("incomplete trace accounting");
        std::cout << "{\"pass\":true,\"frames\":" << frames << ",\"object_states\":" << states
                  << ",\"observations\":" << observations << ",\"misses\":" << misses << "}\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
