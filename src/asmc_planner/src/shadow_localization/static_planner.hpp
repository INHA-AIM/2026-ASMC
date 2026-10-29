#pragma once
#include "Frenet/frenet_planner.hpp"
#include "jamming_common/geometry.hpp"
#include "jamming_common/speed_profile.hpp"
#include "shadow_localization/anchor.hpp"
#include "shadow_localization/path_memory.hpp"
#include <memory>
#include "jamming_common/stall_escape.hpp"
#include <asmc_geometry/motion_evidence.hpp>
#include <set>

namespace shadow_localization {
// Independent, explicit experiment owner. Does not publish canonical map
// objects, change GPS validity, or alter the normal Behavior/selector state.
class StaticPlanner {
 public:
  explicit StaticPlanner(const std::string& parameters,bool sequential_targets=true);
  void reset();
  void load_wall_corridor(const std::string&);
  void set_wall_permission(bool value){wall_permission_=value;core_->SetShadowWallPermission(value);}
  bool wall_loaded()const{return !wall_overlay_.empty();}
  local_planning::Result plan(const local_planning::Scene&,
      const local_planning::Config&,const local_planning::Motion&,const Transform&);
  const std::string& phase() const {return phase_;}
  unsigned native_calls() const {return native_calls_;}
  unsigned speed_profiles() const {return speed_profiles_;}
  unsigned confirmed_rest() const {return confirmed_rest_;}
  unsigned generated() const {return generated_;}
  unsigned feasible() const {return feasible_;}
  unsigned retargets() const {return retargets_;}
  void set_paint_exemption(bool value) {
    paint_exempt_=value;core_->SetShadowPaintExemption(value);
    core_->SetShadowDynamicBypass(value&&dynamic_enabled_);
  }
  void enable_dynamic(bool value) {dynamic_enabled_=value;set_paint_exemption(paint_exempt_);}
  unsigned moving_objects() const {return moving_objects_;}
  unsigned pending_motion_objects() const {return pending_motion_objects_;}
  unsigned rest_motion_disagreements() const {return rest_motion_disagreements_;}
  void enable_path_continuity(bool value){continuity_enabled_=value;previous_path_.clear();}
  bool reused_path()const{return reused_path_;}
  const std::string& continuity_reason()const{return continuity_reason_;}
  void set_following_bumper_gap(double value){following_bumper_gap_=value;}
  int following_target()const{return following_target_;}
  double following_gap()const{return following_gap_;}
  bool maneuver_active() const {return core_->avoid_committed()&&!core_->avoid_completed();}
  const std::vector<CartesianPath>& candidates() const {return core_->last_candidate_paths();}
  const std::vector<bool>& candidate_validity() const {return core_->last_candidate_validity();}
  double transition_distance() const {return transition_distance_;}
  double return_distance() const {return return_distance_;}
  const std::string& candidate_reason() const {return candidate_reason_;}
  // Empty-scene position-following baseline: one reference sampling and one
  // speed profile. No parallel lattice/KEEP search or semantic raster scan.
  local_planning::Result reference(const local_planning::Scene&,
      const local_planning::Config&,const local_planning::Motion&,const Transform&,bool recovery=false);
  void reset_reference() { stall_escape_.reset(); reference_s_=NAN; tracks_.clear(); previous_path_.clear(); retargets_=0; core_->ResetExternalAvoidance(); }
 private:
  std::string parameters_,phase_="unavailable";
  std::unique_ptr<FrenetPlanner> core_;
  bool sequential_targets_=true;
  bool dynamic_enabled_=false;
  bool paint_exempt_=false,wall_permission_=false;
  std::vector<geometry::Polygon2D> wall_overlay_;
  asmc_geometry::motion_evidence::MotionHistory motion_evidence_;
  unsigned moving_objects_=0;
  unsigned pending_motion_objects_=0;
  unsigned rest_motion_disagreements_=0;
  bool in_plan_=false;
  bool continuity_enabled_=false,reused_path_=false;
  std::string continuity_reason_="disabled";
  PathMemory previous_path_;
  local_planning::StallEscapeWatch stall_escape_;
  local_planning::Result escape(const local_planning::Scene&,const local_planning::Config&,const local_planning::Motion&,const Transform&);
  double following_bumper_gap_=3.,following_gap_=-1.;int following_target_=-1;
  void remember_path(const local_planning::Result&,const local_planning::Motion&,const Transform&);
  std::set<int> moving_ids_;
  double reference_s_=NAN;
  unsigned native_calls_=0,speed_profiles_=0;
  unsigned confirmed_rest_=0,generated_=0,feasible_=0;
  unsigned retargets_=0;
  double transition_distance_=0,return_distance_=0;
  std::string candidate_reason_="not_requested";
  struct Sample {double t,xmin,xmax,ymin,ymax;};
  std::map<int,std::deque<Sample>> tracks_;
  bool footprint(const local_planning::Scene&, const local_planning::Config&,
      const local_planning::Pose&, local_planning::Result&,
      const local_planning::Motion&, const Transform&) const;
  bool sweep(const local_planning::Scene&, const local_planning::Config&,
      const local_planning::Pose&, const local_planning::Pose&,
      local_planning::Result&, const local_planning::Motion&, const Transform&) const;
  bool stationary(const local_planning::Obstacle&,const local_planning::Motion&);
  bool observed_velocity(int id,double& vx,double& vy) const;
  local_planning::Result verify(const CartesianPath&,const local_planning::Scene&,
      const local_planning::Config&,const local_planning::Motion&,const Transform&,bool recovery=false);
};
} // namespace shadow_localization
