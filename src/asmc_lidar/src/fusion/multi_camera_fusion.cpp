#include <asmc_lidar/fusion/multi_camera_fusion.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace asmc::lidar::pipeline::fusion {
namespace {
bool Protected(const std::string& s){return s=="vehicle"||s=="dynamic_obstacle";}
bool Available(const std::string& s){return s=="ok"||s=="empty"||s=="no_time_match"||s=="missing";}
struct Pair {double iou;std::size_t box,projection;};
}
MultiCameraFusion::MultiCameraFusion(const FusionConfig& c,const camera::Calibrations& cameras)
    :config_(c),calibrations_(cameras) {
  if(cameras.empty())throw std::invalid_argument("multi-view fusion requires calibrations");
  // Reuse all existing threshold validation/clamping, without running its tracker.
  config_=FrontCameraFusion(c,cameras.begin()->second).config();
  if(config_.yolo_policy!="unique")throw std::invalid_argument("multi-view fusion requires unique observations");
  for(const auto& v:cameras) {
    if(v.first!=v.second.frame_id)throw std::invalid_argument("camera calibration key/frame mismatch");
    FrontCameraFusion validate(config_,v.second);
  }
  offset_=std::llround(config_.max_time_offset_s*1e9);
  wait_=std::llround(config_.sync_wait_timeout_s*1e9);
  window_=std::llround(config_.camera_read_window_s*1e9);
  allowed_=config_.allowed_names.empty()?fusion_adapter::MissionNames():config_.allowed_names;
}

void MultiCameraFusion::Aggregate(Scan& scan,std::vector<TrackedObject>& live,
    CameraSource& camera,std::int64_t ros,std::int64_t steady) {
  for(auto& object:live) {
    const int id=object.tracking.s32_Id;
    if(applied_scan_[id]>scan.stamp || fusion_class::IsGroundNoise(object))continue;
    std::vector<Evidence> evidence;
    for(const auto& view:scan.views) {
      auto e=view.second.evidence.find(id);
      if(e!=view.second.evidence.end()&&camera.Current(e->second.source,ros,steady))evidence.push_back(e->second);
    }
    if(evidence.empty())continue; // invalid/outside-FOV is NOT a negative observation
    auto inserted=scan.start.emplace(id,object.camera);
    TrackedObject scratch;scratch.camera=inserted.first->second;
    std::set<std::string> groups,raws;
    std::int64_t stamp=0;bool visible=false;
    for(const auto& e:evidence) {
      stamp=std::max(stamp,e.source.stamp_ns);visible|=e.visible;
      if(e.positive){groups.insert(e.group);raws.insert(e.raw);}
    }
    // Start-state reaggregation replaces the same scan unit, never increments it
    // once per camera or on every later callback. Newer units seal older units.
    scratch.camera.evidence_ids.clear();
    for(const auto& e:evidence)if(e.positive) {
      if(!scratch.camera.evidence_ids.empty())scratch.camera.evidence_ids+="; ";
      scratch.camera.evidence_ids+=e.source.frame_id+"="+e.source.identity;
    }
    scratch.camera.camera_observation_ns=stamp;
    scratch.camera.camera_observation_epoch=consumer_epoch_;
    scratch.camera.observed_class=scratch.camera.observed_raw_class="unknown";
    scratch.camera.protection_group="unknown";
    scratch.camera.group_conflict=groups.size()>1;
    if(groups.size()==1) {
      const auto raw=raws.size()==1?*raws.begin():raws.count(scratch.camera.str_YoloClass)?scratch.camera.str_YoloClass:"unknown";
      fusion_class::Observe(scratch,*groups.begin(),raw,config_.initial_confirmations,
                            config_.change_confirmations,config_.mission_profile);
      scratch.camera.protection_group=Protected(*groups.begin())?*groups.begin():"unknown";
    } else if(groups.size()>1) {
      // Keep the prior confirmed class and confirmation counters. Only unanimous
      // protection eligibility survives a vehicle/dynamic disagreement.
      if(std::all_of(groups.begin(),groups.end(),Protected))scratch.camera.protection_group="dynamic_obstacle";
    } else if(visible) fusion_class::Miss(scratch,config_.class_miss_grace);
    scratch.camera.multiview_evidence=true;
    scratch.camera.evidence_scan_ns=scan.stamp;
    if(applied_scan_[id]!=scan.stamp) {
      if(groups.size()==1)++counters_.observations;
      else if(groups.empty()&&visible)++counters_.misses_applied;
      if(scratch.camera.str_ObjectClass!=object.camera.str_ObjectClass &&
          fusion_class::IsConfirmed(scratch.camera.str_ObjectClass,config_.mission_profile))++counters_.confirmations_applied;
    }
    object.camera=std::move(scratch.camera);
    applied_scan_[id]=scan.stamp;
    latest_evidence_[id]=std::move(evidence);
  }
}

FusionReport MultiCameraFusion::Process(std::int64_t decision,std::int64_t stamp,
    const Context& context,std::vector<TrackedObject>& live,std::vector<TrackedObject>* history,
    CameraSource& camera,std::int64_t ros,std::int64_t steady) {
  FusionReport report;report.diagnostics_enabled=config_.capture_diagnostics;
  report.emitting_scan_sequence=context.scan_sequence;report.emitting_scan_stamp_ns=stamp;
  ++counters_.calls;
  const auto snapshot=camera.Read(stamp,window_,ros,steady);
  report.camera_active=snapshot.gate_active;report.camera_reason=snapshot.gate_reason;
  if(config_.capture_diagnostics){report.views=snapshot.views;for(auto& v:report.views)v.matching_implemented=calibrations_.count(v.frame_id)!=0;}
  auto record=[&](const Scan& s,const std::string& view,const std::string& action,const std::string& why)->MatchDiagnostic* {
    if(!config_.capture_diagnostics)return nullptr;
    if(report.decisions.size()>=64){++report.decisions_omitted;return nullptr;}
    MatchDiagnostic d;d.scan_sequence=s.sequence;d.scan_stamp_ns=s.stamp;d.wait_ns=decision-s.queued;
    d.camera_id=view;d.action=action;d.reason=why;report.decisions.push_back(std::move(d));return &report.decisions.back();
  };
  if(snapshot.gate_active!=active_ || snapshot.gate_epoch!=source_epoch_) {
    ++consumer_epoch_;++counters_.epoch_changes;
    pending_.clear();consumed_.clear();applied_scan_.clear();latest_evidence_.clear();
    for(auto& o:live)fusion_class::InvalidateObservation(o,consumer_epoch_);
    active_=snapshot.gate_active;source_epoch_=snapshot.gate_epoch;
  }
  if(!active_) {
    ++counters_.inactive_calls;
    for(auto& o:live)fusion_class::InvalidateObservation(o,consumer_epoch_);
    if(history)fusion_state::Persist(live,*history);
    return report;
  }
  std::set<int> alive;for(const auto& o:live)alive.insert(o.tracking.s32_Id);
  for(auto i=applied_scan_.begin();i!=applied_scan_.end();)if(!alive.count(i->first))i=applied_scan_.erase(i);else ++i;
  for(auto i=latest_evidence_.begin();i!=latest_evidence_.end();)if(!alive.count(i->first))i=latest_evidence_.erase(i);else ++i;
  if(stamp>last_scan_) {
    Scan s;s.stamp=stamp;s.queued=decision;s.sequence=context.scan_sequence;
    for(const auto& o:live)s.ids.push_back(o.tracking.s32_Id);
    for(const auto& c:calibrations_) {
      auto& v=s.views[c.first];
      v.projection_valid=LidarToImage(live,context,c.second,config_,&v.projection);
      if(!v.projection_valid){v.done=true;++counters_.projection_invalid;record(s,c.first,"drop","projection_invalid");}
    }
    // Exactly one projection per scan/view; no historical point-cloud copies.
    pending_.push_back(std::move(s));last_scan_=stamp;++counters_.queued;
  }
  for(auto& s:pending_) {
    for(auto& view:s.views) {
      auto& v=view.second;if(v.done)continue;
      auto info=std::find_if(snapshot.views.begin(),snapshot.views.end(),[&](const auto& x){return x.frame_id==view.first;});
      if(info!=snapshot.views.end()&&!Available(info->reason)) {
        v.done=true;record(s,view.first,"drop",info->reason);continue;
      }
      std::vector<const CameraFrame*> frames;
      auto found=snapshot.by_view.find(view.first);
      if(found!=snapshot.by_view.end())for(const auto& f:found->second)
        if(f.stamp_ns>0&&!consumed_.count(f.identity))frames.push_back(&f);
      std::stable_sort(frames.begin(),frames.end(),[](auto a,auto b){return a->stamp_ns<b->stamp_ns;});
      std::vector<std::int64_t> stamps;for(auto f:frames)stamps.push_back(f->stamp_ns);
      const auto choice=fusion_sync::DecideMatch(s.stamp,stamps,decision-s.queued,offset_,wait_);
      if(choice.action==fusion_sync::MatchAction::Wait){++counters_.waits;record(s,view.first,"wait","awaiting_at_or_after_scan");continue;}
      if(choice.action==fusion_sync::MatchAction::Drop){v.done=true;++counters_.dropped;++report.dropped_this_call;record(s,view.first,"drop",decision-s.queued>=wait_?"timeout_no_time_match":"outside_time_window");continue;}
      const auto& f=*frames[choice.yolo_index];
      const auto& calib=calibrations_.at(view.first);
      if(f.width!=int(calib.width)||f.height!=int(calib.height)||
          (f.record&&f.record->observation.calibration_id!=calib.id)) {
        v.done=true;record(s,view.first,"drop","calibration_mismatch");continue;
      }
      if(!camera.Current(f,ros,steady)){++counters_.not_current;record(s,view.first,"wait","observation_no_longer_current");continue;}
      auto boxes=fusion_adapter::ConvertBoxes(f.boxes,allowed_,config_.confidence_threshold);
      std::vector<Pair> pairs;
      for(std::size_t b=0;b<boxes.size();++b)for(std::size_t j=0;j<v.projection.size();++j) {
        const double iou=ProjectionIoU(boxes[b],v.projection[j]);
        if(iou>config_.iou_threshold)pairs.push_back({iou,b,j});
      }
      // Equal scores are deterministic: source box then projected object order.
      std::sort(pairs.begin(),pairs.end(),[](const Pair& a,const Pair& b){return std::make_tuple(-a.iou,a.box,a.projection)<std::make_tuple(-b.iou,b.box,b.projection);});
      if(!camera.Current(f,ros,steady)){++counters_.not_current;record(s,view.first,"wait","observation_changed_during_projection");continue;}
      auto* d=record(s,view.first,"match","matched");
      if(d) {
        d->image_stamp_ns=f.stamp_ns;d->image_minus_scan_ns=f.stamp_ns-s.stamp;d->observation_id=f.identity;
        d->calibration_id=calib.id;d->boxes_received=f.boxes.size();d->boxes_admitted=boxes.size();d->projection_valid=true;
        if(f.record){const auto& o=f.record->observation;d->session=o.session;d->generation=o.generation;d->wire_sequence=o.header.sequence;d->model_id=o.model_id;d->config_id=o.config_id;}
      }
      CameraFrame token=f;token.boxes.clear();
      for(const auto& p:v.projection) {
        const int id=s.ids.at(p.c_idx);Evidence e;e.visible=true;e.source=token;v.evidence[id]=std::move(e);
      }
      std::set<std::size_t> used_boxes,used_objects;
      for(const auto& p:pairs) {
        const auto& box=boxes[p.box];const auto& projected=v.projection[p.projection];
        if(used_boxes.count(p.box)||used_objects.count(projected.c_idx))continue;
        used_boxes.insert(p.box);used_objects.insert(projected.c_idx);
        const int id=s.ids.at(projected.c_idx);auto& e=v.evidence.at(id);
        e.positive=true;e.group=fusion_class::Normalize(box.str_class,config_.mission_profile);e.raw=fusion_class::RawLabel(box.str_raw_class);
        if(d&&d->associations.size()<128) {
          AssociationDiagnostic a;a.track_id=id;a.source_box_index=box.source_index;a.projected=true;
          a.iou=p.iou;a.group=e.group;a.raw_class=e.raw;a.reason="view_evidence";
          a.projected_box={projected.min_x,projected.min_y,projected.max_x,projected.max_y};
          a.camera_box={box.min_x,box.min_y,box.max_x,box.max_y};d->associations.push_back(std::move(a));
        } else if(d)++d->associations_omitted;
      }
      consumed_[f.identity]=f.stamp_ns;v.done=true;
      ++counters_.matched;++counters_.fresh;++report.matched_this_call;
      report.match_offsets_ns.push_back(choice.offset_ns);
    }
    Aggregate(s,live,camera,ros,steady);
    for(auto& d:report.decisions)if(d.scan_stamp_ns==s.stamp)for(auto& a:d.associations) {
      const auto* o=fusion_state::Find(live,a.track_id);
      const auto start=s.start.find(a.track_id);
      if(start!=s.start.end())a.class_before=start->second.str_ObjectClass;
      if(o)a.class_after=o->camera.str_ObjectClass;
      a.applied=o&&applied_scan_[a.track_id]==s.stamp&&!o->camera.group_conflict;
      a.reason=applied_scan_[a.track_id]>s.stamp?"older_scan_sealed":o&&o->camera.group_conflict?"group_conflict":a.applied?"aggregated_once_per_scan":"not_current";
    }
  }
  // Retain classifications but revoke fresh protection when source records
  // expire or a view invalidates. A stale YOLO label is never new evidence.
  for(auto& o:live) {
    const auto found=latest_evidence_.find(o.tracking.s32_Id);
    if(found==latest_evidence_.end())continue;
    bool positive=false,all_protected=true;std::set<std::string> groups;
    for(const auto& e:found->second)if(e.positive&&camera.Current(e.source,ros,steady)) {
      positive=true;groups.insert(e.group);all_protected&=Protected(e.group);
    }
    if(!positive){o.camera.observed_class=o.camera.observed_raw_class="unknown";o.camera.protection_group="unknown";}
    else if(!all_protected)o.camera.protection_group="unknown";
    // Do not erase a conflict just because its contradicting camera went stale.
  }
  if(history)fusion_state::Persist(live,*history);
  pending_.erase(std::remove_if(pending_.begin(),pending_.end(),[](const Scan& s){return std::all_of(s.views.begin(),s.views.end(),[](const auto& v){return v.second.done;});}),pending_.end());
  while(pending_.size()>config_.max_pending_scans){pending_.pop_front();++counters_.bound_discarded;}
  for(auto i=consumed_.begin();i!=consumed_.end();)if(i->second<stamp-window_-wait_)i=consumed_.erase(i);else ++i;
  report.pending_scans=pending_.size();return report;
}
}
