#pragma once
#include <asmc_lidar/experimental/confirmation_candidate.hpp>
#include <asmc_lidar/experimental/observation_confirmation.hpp>
#include <asmc_lidar/tracking/track.hpp>
#include <asmc_lidar/preprocessing/ground.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
namespace asmc::lidar::pipeline {
struct TunnelNoiseConfig {
  bool enabled=false;
  bool confirmation_all_unknown=false;
  bool adaptive_confirmation=false;
  int structural_confirmation_frames=5;
  int low_flat_confirmation_frames=6;
  int confirmation_frames=3;
  int hold_frames=4;
};
inline void ValidateTunnelNoise(const TunnelNoiseConfig& c) {
  if(c.low_flat_confirmation_frames<1 || c.low_flat_confirmation_frames>30)throw std::invalid_argument("low_flat_confirmation_frames must be in [1,30]");
  if(c.structural_confirmation_frames<1 || c.structural_confirmation_frames>30)throw std::invalid_argument("structural_confirmation_frames must be in [1,30]");
  if(c.confirmation_frames<1 || c.confirmation_frames>30)throw std::invalid_argument("confirmation_frames must be in [1,30]");
  if(c.hold_frames<1 || c.hold_frames>30)throw std::invalid_argument("tunnel noise hold_frames must be in [1,30]");
}
// Causal output gate. Raw tracker history is committed BEFORE this gate: a
// hidden candidate can accumulate evidence instead of receiving a new ID.
// Frame means a processed LiDAR observation, never a web refresh/prediction.
class TunnelNoiseGate {
 public:
  struct Stats { std::map<int,int> required_frames; int candidates=0,hidden=0,hidden_predictions=0,max_run=0; };
  static int Kind(const TrackedObject& o,const GroundPlaneSnapshot& plane,bool walls) {
    if(o.camera.str_ObjectClass!="unknown" && !o.camera.str_ObjectClass.empty())return 0;
    if(o.camera.str_YoloClass!="unknown" && !o.camera.str_YoloClass.empty())return 0;
    const auto& d=o.detection;
    if(!std::isfinite(d.obb_length_m)||!std::isfinite(d.obb_width_m))return 0;
    const double L=std::max(d.obb_length_m,d.obb_width_m),W=std::min(d.obb_length_m,d.obb_width_m);
    const double H=d.f32_OBB_ZMax-d.f32_OBB_ZMin;
    if(!std::isfinite(L)||!std::isfinite(W)||!std::isfinite(H)||L<=0||W<=0||H<0)return 0;
    const double ratio=L/W;
    if(plane.valid) {
      const auto& p=plane.coefficients;
      const double top=p[0]*d.f32_OBB_CenterX+p[1]*d.f32_OBB_CenterY+p[2]*d.f32_OBB_ZMax+p[3];
      if(std::isfinite(top)&&H<=.25&&top>=-.10&&top<=.40&&ratio>=3)return 1;
    }
    // This is a measured SHAPE candidate, not a semantic wall/GT label.
    if(walls&&ratio>=6&&W<=.8&&L>=1.5&&H>=.5)return 2;
    return 0;
  }
  std::set<int> Update(const std::vector<TrackedObject>& objects,const GroundPlaneSnapshot& plane,
                      bool walls,bool active,std::int64_t stamp,const TunnelNoiseConfig& cfg,Stats& stats,const std::set<int>* exempt=nullptr,
                      const std::map<int,ConfirmationKind>* kinds=nullptr) {
    stats={};std::set<int> hidden;
    if(cfg.confirmation_all_unknown) {
      std::vector<ObservationConfirmation::Input> input;
      for(const auto& o:objects) {
        const bool known=(!o.camera.str_ObjectClass.empty()&&o.camera.str_ObjectClass!="unknown") ||
                         (!o.camera.str_YoloClass.empty()&&o.camera.str_YoloClass!="unknown");
        int required=cfg.confirmation_frames;
        if(cfg.adaptive_confirmation&&kinds) {
          const auto it=kinds->find(o.tracking.s32_Id);
          if(it!=kinds->end()&&it->second!=ConfirmationKind::General) {
            required=std::max(required,cfg.structural_confirmation_frames);
            if(it->second==ConfirmationKind::ElevatedFlatFragment)
              required=std::max(required,cfg.low_flat_confirmation_frames);
          }
        }
        input.push_back({o.tracking.s32_Id,o.tracking.s32_MissCount==0,known,required});
      }
      hidden=confirmation_.Update(input,active&&cfg.enabled,stamp,cfg.confirmation_frames);
      for(const auto& o:objects) {
        stats.required_frames[o.tracking.s32_Id]=confirmation_.Required(o.tracking.s32_Id);
        if(o.tracking.s32_MissCount==0) { stats.max_run=std::max(stats.max_run,confirmation_.Run(o.tracking.s32_Id)); ++stats.candidates; if(hidden.count(o.tracking.s32_Id))++stats.hidden; }
        else if(hidden.count(o.tracking.s32_Id))++stats.hidden_predictions;
      }
      return hidden;
    }

    if(!active||!cfg.enabled){Reset();return hidden;}
    const bool consecutive=last_stamp_>0&&stamp>last_stamp_&&stamp-last_stamp_<=250000000;
    if(!consecutive)history_.clear();
    std::map<int,State> next;
    for(const auto& o:objects) {
      const int id=o.tracking.s32_Id;if(id<0)continue;
      if(exempt&&exempt->count(id))continue;
      const int kind=Kind(o,plane,walls);
      if(!kind)continue;
      const auto old=history_.find(id);
      if(o.tracking.s32_MissCount>0) {
        if(old!=history_.end()&&old->second.kind==kind&&old->second.hidden){hidden.insert(id);++stats.hidden_predictions;next[id]=old->second;next[id].count=0;}
        continue;
      }
      State st;st.kind=kind;
      st.count=(old!=history_.end()&&old->second.kind==kind)?old->second.count+1:1;
      st.hidden=st.count<=cfg.hold_frames;next[id]=st;
      ++stats.candidates;stats.max_run=std::max(stats.max_run,st.count);
      if(st.hidden){hidden.insert(id);++stats.hidden;}
    }
    history_=std::move(next);last_stamp_=stamp;return hidden;
  }
  void Reset(){history_.clear();last_stamp_=0;confirmation_.Reset();}
 private:
  ObservationConfirmation confirmation_;
  struct State{int kind=0,count=0;bool hidden=false;};
  std::map<int,State> history_;std::int64_t last_stamp_=0;
};
}
