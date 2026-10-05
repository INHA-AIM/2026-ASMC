#include <asmc_lidar/unified.hpp>
#include <cassert>
#include <cmath>
#include <stdexcept>
int main(){
  using namespace asmc_lidar;
  Cloud raw;GroundLiveResult ground;
  ground.prepared.leveling={1,0,0,0,1,0,0,0,1};
  for(int i=0;i<26;++i){
    pcl::PointXYZI p;p.x=4.f+.08f*i;p.y=.08f*(i%3);p.z=i<20?-.4f:-1.8f;p.intensity=float(i);
    raw.push_back(p);ground.prepared.voxel->push_back(p);
    ground.prepared.leveled->push_back(p);
    ground.prepared.raw_to_voxel.push_back(i);
  }
  auto& mask=ground.methods["jaeho"];
  for(int i=0;i<26;++i){mask.labels.push_back(i<20?2:1);mask.structural_removed.push_back(i<20?1:0);}
  Calibration cal;cal.base_from_sensor.translation()=Eigen::Vector3d(.8,0,1.45);
  const auto a=BuildUnified(raw,ground,cal,123456789);
  assert(a.frame_id=="base_link"&&a.observation_ns==123456789);
  assert(a.ground_raw_points==6&&a.retained_raw_indices.size()==20);
  assert(a.structure_candidate_raw_indices.size()==20);
  assert(a.detections.size()==1&&a.detections[0].source_indices.size()==20);
  assert(a.structure_evidence.size()==1);
  assert(a.detections[0].object_class=="unknown"&&a.detections[0].velocity_state=="unknown");
  assert(std::abs(a.detections[0].center_base.z()-1.05)<.01);
  assert(std::abs(a.detections[0].center_base.x()-5.56)<.15);
  OptionalContext c;c.source="camera";c.frame_id="base_link";c.observation_ns=123456789;
  c.valid=true;c.quality=.9;
  assert(ContextUsable(c,123456789));
  assert(!ContextUsable(c,223456790));
  c.frame_id="map";assert(!ContextUsable(c,123456789));
  const auto b=BuildUnified(raw,ground,cal,123456789,nullptr,&c);
  assert(b.map_context_status=="missing"&&b.camera_context_status=="frame_mismatch");
  assert(b.detections.size()==a.detections.size());
  assert(b.retained_raw_indices==a.retained_raw_indices);
  c.frame_id="base_link";c.observation_ns=123456789;
  const auto with_context=BuildUnified(raw,ground,cal,123456789,&c,&c);
  assert(with_context.map_context_status=="available");
  assert(with_context.detections[0].source_indices==a.detections[0].source_indices);
  assert((with_context.detections[0].center_base-a.detections[0].center_base).norm()<1e-12);
  c.observation_ns=1;
  assert(BuildUnified(raw,ground,cal,123456789,&c).map_context_status=="stale");
  // Nonzero roll/pitch and mount yaw: corners and center must both apply
  // inverse leveling and mount exactly once, not only rotate the yaw scalar.
  const Eigen::Matrix3d level=(Eigen::AngleAxisd(.12,Eigen::Vector3d::UnitX())*
      Eigen::AngleAxisd(-.08,Eigen::Vector3d::UnitY())).toRotationMatrix();
  auto rotated=ground;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)rotated.prepared.leveling[3*i+j]=float(level(i,j));
  for(std::size_t i=0;i<raw.size();++i){
    const auto& p=raw[i];const auto q=level*Eigen::Vector3d(p.x,p.y,p.z);
    rotated.prepared.leveled->at(i).x=float(q.x());
    rotated.prepared.leveled->at(i).y=float(q.y());
    rotated.prepared.leveled->at(i).z=float(q.z());
  }
  Calibration mounted=cal;
  mounted.base_from_sensor.linear()=Eigen::AngleAxisd(.31,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto tilted=BuildUnified(raw,rotated,mounted,123456789);
  assert(tilted.detections.size()==1);
  assert((tilted.detections[0].forward_base.norm()-1)<1e-5);
  for(const auto& corner:tilted.detections[0].corners_base)assert(corner.allFinite());
  // A rejected plane hypothesis cannot delete even its own support points.
  for(std::size_t i=0;i<20;++i)assert(a.retained_raw_indices[i]==i);
  mask.labels.pop_back();
  bool rejected=false;try{BuildUnified(raw,ground,cal,123456789);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
  return 0;
}
