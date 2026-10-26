// Offline evaluation through the actual gate. Never starts a ROS node/master.
#include <planning_inputs/lidar_observation_gate.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <map>

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "usage: planning_lidar_bag_check planning-observations.bag\n"; return 2; }
  rosbag::Bag bag(argv[1], rosbag::bagmode::Read);
  planning_inputs::LidarObservationGate gate;
  uint64_t accepted=0, rejected=0, objects=0, unmeasured=0, bad_placeholder=0, gaps=0;
  double last=0., max_gap=0.; bool pose_lost=false;
  std::map<std::string, uint64_t> reasons;
  for (const auto& message : rosbag::View(bag)) {
    auto packet=message.instantiate<asmc_msgs::LidarPlanningObservation>();
    if (!packet) continue;
    planning_inputs::DetectedObjectArray output; std::string reason;
    if (!gate.Convert(*packet,message.getTime().toSec(),&output,&reason)) {
      ++rejected; ++reasons[reason];
      const auto& a=packet->observation.objects;
      pose_lost |= a.status!=a.OBSERVED && a.status!=a.OBSERVED_EMPTY;
      continue;
    }
    ++accepted;
    const double stamp=output.header.stamp.toSec();
    if (last>0. && !pose_lost) { max_gap=std::max(max_gap,stamp-last); gaps += stamp-last>.3; }
    last=stamp; pose_lost=false;
    objects+=output.objects.size();
    for (const auto& object:output.objects) if (object.velocity_unavailable) {
      ++unmeasured;
      bad_placeholder += object.motion_state != object.MOTION_UNKNOWN || object.vx != 0. || object.vy != 0.;
    }
  }
  std::cout << "{\"accepted\":"<<accepted<<",\"rejected\":"<<rejected<<",\"objects\":"<<objects
            <<",\"unmeasured_objects_retained\":"<<unmeasured<<",\"bad_unmeasured_flags\":"<<bad_placeholder
            <<",\"map_valid_max_gap_s\":"<<max_gap<<",\"map_valid_gaps_over_300ms\":"<<gaps<<",\"reasons\":{";
  bool first=true;
  for (const auto& entry:reasons) { if(!first)std::cout<<",";first=false;std::cout<<"\""<<entry.first<<"\":"<<entry.second; }
  std::cout<<"}}\n";
  return bad_placeholder || !accepted ? 1 : 0;
}
