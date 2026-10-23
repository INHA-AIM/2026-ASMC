#pragma once
#include <Geometry/FrameBoundary.hpp>
#include <Lidar/Tracking/TrackTrace.hpp>
#include <map>
#include <set>

namespace point_support {
struct Memory { std::map<int,LiDARObjectInfo> observations; std::int64_t time=0; };
inline Memory& Cache(){static Memory c;return c;}
inline void Reset(){Cache()=Memory{};}
inline void Remember(const std::vector<LiDARObjectInfo>& previous) {
    auto& c=Cache();const auto now=frame_boundary::Current().time.stamp_ns;
    if(now<=c.time)c.observations.clear();
    c.time=now;std::set<int> keep;
    for(const auto& p:previous) {
        if(p.s32_KfInitCount!=1 || !p.tracking_frame_valid)continue;
        keep.insert(p.s32_Id);
        if(p.s32_MissCount==0 && p.pcl_ClusterPoints.size()>=5 && p.pcl_ClusterPoints.size()<=16)
            c.observations[p.s32_Id]=p;
    }
    for(auto it=c.observations.begin();it!=c.observations.end();)
        if(!keep.count(it->first))it=c.observations.erase(it);else ++it;
}
struct Evidence {int points=0,hits=0,unique=0,height_bins=0;double fraction=0,mean_distance=0;bool contained=false;};
inline Evidence Containment(int id,const LiDARObjectInfo& current,double yaw) {
    Evidence e;const auto found=Cache().observations.find(id);
    if(found==Cache().observations.end())return e;
    const auto& old=found->second;
    const double age=(frame_boundary::Current().time.stamp_ns-old.tracking_stamp_ns)*1e-9;
    if(!std::isfinite(age)||age<=0||age>.4||current.pcl_ClusterPoints.size()<3*old.pcl_ClusterPoints.size())return e;
    std::set<int> unique,heights;Eigen::Vector2d mean=Eigen::Vector2d::Zero();
    for(const auto& p:old.pcl_ClusterPoints) {
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return Evidence{};
        // Existing exact frame transform, zero-ground-motion support hypothesis only.
        // Never transform an already coast-aligned cloud using coast-updated metadata.
        LiDARObjectInfo q{};
        q.tracking_frame_valid=true;q.tracking_height_input=p.z;
        q.tracking_yaw_from_input=old.tracking_yaw_from_input;
        q.tracking_origin_in_yaw=old.tracking_origin_in_yaw;
        q.tracking_stamp_ns=old.tracking_stamp_ns;q.tracking_ego_speed_mps=old.tracking_ego_speed_mps;
        q.f32_PrevYaw=old.f32_PrevYaw;q.Eigen_KfState_CA.setZero();
        q.Eigen_KfState_CA(0)=p.x;q.Eigen_KfState_CA(1)=p.y;q.Eigen_KfP_CA.setZero();
        frame_boundary::AlignTrackingFrame(q,yaw,false);
        const Eigen::Vector3d x(q.Eigen_KfState_CA(0),q.Eigen_KfState_CA(1),q.tracking_height_input);
        if(!x.allFinite())return Evidence{};
        mean+=x.head<2>();++e.points;
        double best=.25*.25;int nearest=-1,index=0;
        for(const auto& n:current.pcl_ClusterPoints) {
            const double dist=(Eigen::Vector3d(n.x,n.y,n.z)-x).squaredNorm();
            if(std::isfinite(dist)&&dist<=best){best=dist;nearest=index;}++index;
        }
        if(nearest>=0){++e.hits;unique.insert(nearest);heights.insert(int(std::floor(x.z()/.2)));}
    }
    if(!e.points)return e;
    mean/=e.points;e.fraction=double(e.hits)/e.points;e.unique=unique.size();e.height_bins=heights.size();
    e.mean_distance=(mean-Eigen::Vector2d(current.f32_CoreX,current.f32_CoreY)).norm();
    e.contained=e.fraction>=.8 && e.unique>=3 && e.height_bins>=2 &&
        e.mean_distance>st_Tracking.f32_COST_THRESHOLD;
    return e;
}
inline std::vector<bool> Retire(const std::vector<LiDARObjectInfo>& previous,
                               const std::vector<LiDARObjectInfo>& current,
                               const std::vector<int>& assignment,
                               const std::vector<std::vector<double>>& costs,double yaw) {
    Remember(previous);std::vector<bool> retired(previous.size(),false);
    for(std::size_t i=0;i<previous.size();++i) {
        if(previous[i].s32_KfInitCount!=1)continue;
        for(std::size_t owner=0;owner<previous.size();++owner) {
            if(owner==i || previous[owner].s32_KfInitCount<2)continue;
            const int j=owner<assignment.size()?assignment[owner]:-1;
            if(j<0 || j>=int(current.size()) || costs[owner][j]>=st_Tracking.f32_COST_THRESHOLD)continue;
            const auto e=Containment(previous[i].s32_Id,current[j],yaw);
            track_trace::Event("support_relation",[&](std::ostream& out) {
                out<<",\"id\":"<<previous[i].s32_Id<<",\"owner_id\":"<<previous[owner].s32_Id
                   <<",\"det\":"<<current[j].u16_Idx<<",\"points\":"<<e.points<<",\"hits\":"<<e.hits
                   <<",\"unique\":"<<e.unique<<",\"height_bins\":"<<e.height_bins
                   <<",\"fraction\":"<<e.fraction<<",\"mean_distance\":"<<e.mean_distance
                   <<",\"retire\":"<<(e.contained?"true":"false");
            });
            if(e.contained){retired[i]=true;break;}
        }
    }
    return retired;
}
}
