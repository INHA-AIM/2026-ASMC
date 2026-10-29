#pragma once
#include <algorithm>
#include <string>
#include <vector>

namespace lane_boundary_adapter::obstacle_memory {
struct Obstacle {
    double x, length, line_left, line_right, odom_at;
    int hits;
    bool lidar=true;
    int track_id=-1;
    std::string source{};
    double last_seen=0;
    std::string label{};
    bool predicted=false;
};
inline double currentX(const Obstacle& o,double odom){return o.x-(odom-o.odom_at);}

// LiDAR observations are the sole geometry source. Replace the complete valid
// snapshot, never union old extents or retain IDs missing from that snapshot.
// Call only for a new valid scan. Missing/invalid input must not call this
// function with an artificial empty vector.
inline void updateLidar(std::vector<Obstacle>& stored,std::vector<Obstacle> current,
                        const std::string& source,int confirmations){
    for(auto& now:current){
        const auto old=std::find_if(stored.begin(),stored.end(),[&](const auto& o){
            return o.source==source&&o.track_id==now.track_id;
        });
        now.source=source;
        if(old!=stored.end())now.hits=std::min(old->hits+(now.predicted?0:1),confirmations);
        else now.hits=now.predicted?0:1;
    }
    stored=std::move(current);
}
} // namespace lane_boundary_adapter::obstacle_memory
