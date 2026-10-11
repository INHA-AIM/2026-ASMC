#pragma once
#include <string>
#include <vector>
#include "Global/Global.hpp"
namespace behavior_planner {
using Point2D = asmc_planner::Point2D;
struct LinkInfo { std::string id, from_node_id, to_node_id; std::vector<Point2D> centerline; double max_speed=0, min_speed=0, width_start=3.5, width_end=3.5, link_length=0; bool can_move_left=false, can_move_right=false; std::string left_dst_link_id, right_dst_link_id, related_signal, road_id; int ego_lane=-1; };
}
