#include <asmc_lidar/detection/camera_point_refinement.hpp>
#include <asmc_lidar/experimental/tunnel_route.hpp>
#include <asmc_lidar/geometry/projection.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/common/point_tests.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace asmc::lidar::pipeline {
namespace {
struct Pixel { float u=0,v=0,z=0;bool valid=false,wall=false; };
struct Candidate {
  pcl::PointIndices points;
  CameraPointSupport support;
  std::size_t decision=0;
  float confidence=0;
  bool steel_merge=false;
};
bool SupportedClass(const std::string& s) {
  return s=="car"||s=="truck"||s=="bus"||s=="person"||s=="pedestrian"||
    s=="bicycle"||s=="motorcycle"||s=="steel_barricade"||s=="box"||
    s=="cargo_box"||s=="bollard"||s=="traffic_cone"||s=="drum"||s=="barrel"||
    s=="traffic_drum"||s=="concrete_barrier"||s=="plastic_barrier"||
    s=="stroller"||s=="dog"||s=="deer";
}
bool OnWall(const pcl::PointXYZI& p,const std::vector<TunnelWallEvidence>& walls,float band) {
  for(const auto& w:walls) {
    if(!w.allowed||!w.valid)continue;
    const auto& a=w.plane;
    const float norm=std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    const float h=std::hypot(a[0],a[1]);
    if(norm<.5f||h<.5f||std::fabs(a[0]*p.x+a[1]*p.y+a[2]*p.z+a[3])>band*norm)continue;
    const float along=(-a[1]*p.x+a[0]*p.y)/h;
    for(const auto& interval:w.intervals)
      if(along>=interval.first&&along<=interval.second)return true;
  }
  return false;
}
std::vector<pcl::PointIndices> Components(const CloudPtr& cloud,const std::vector<int>& indices,
                                         float tolerance,int minimum) {
  if(indices.empty())return {};
  // A local tree keeps work proportional to ROI support, not full scene size.
  CloudPtr local(new Cloud);local->reserve(indices.size());
  for(int i:indices)local->push_back(cloud->at(i));
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(local);
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setInputCloud(local);ec.setSearchMethod(tree);ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(minimum);ec.setMaxClusterSize(static_cast<int>(indices.size()));
  std::vector<pcl::PointIndices> components;ec.extract(components);
  for(auto& c:components)for(auto& i:c.indices)i=indices.at(i);
  return components;
}

// Join only complementary horizontal scan ribbons of one LiDAR parent, one
// camera instance and one depth layer. Dense surfaces and independently valid
// objects are never joined by this path. Pairwise compatibility (not transitive
// chaining) and a three-ribbon cap prevent growth through background bridges.
int GroupVehicleScanlines(const CloudPtr& cloud,const std::vector<Pixel>& pixels,
    const std::vector<int>& parent,float box_width,float box_height,
    const CameraPointRefinementConfig& cfg,std::vector<pcl::PointIndices>& parts) {
  if(parts.size()<2||parts.size()>64)return 0;
  struct Ribbon {int parent=-1;float depth=0,u0=0,u1=0,v0=0,v1=0,z0=0,z1=0;bool eligible=false;};
  std::vector<Ribbon> ribbons(parts.size());
  for(std::size_t k=0;k<parts.size();++k) {
    auto& r=ribbons[k];r.u0=r.v0=r.z0=INFINITY;r.u1=r.v1=r.z1=-INFINITY;
    std::vector<float> depths;bool same=true;r.parent=parent[parts[k].indices.front()];
    for(int i:parts[k].indices) {
      const auto& p=pixels[i];r.u0=std::min(r.u0,p.u);r.u1=std::max(r.u1,p.u);
      r.v0=std::min(r.v0,p.v);r.v1=std::max(r.v1,p.v);
      r.z0=std::min(r.z0,cloud->at(i).z);r.z1=std::max(r.z1,cloud->at(i).z);
      depths.push_back(p.z);same=same&&parent[i]==r.parent;
    }
    auto middle=depths.begin()+depths.size()/2;std::nth_element(depths.begin(),middle,depths.end());r.depth=*middle;
    r.eligible=same&&r.parent>=0&&parts[k].indices.size()>=static_cast<std::size_t>(cfg.minimum_points)&&
      (r.u1-r.u0)/box_width>=.15f&&(r.v1-r.v0)/box_height<.08f&&
      r.z1-r.z0<=.25f+.005f*r.depth;
  }
  std::vector<std::vector<bool>> compatible(parts.size(),std::vector<bool>(parts.size(),false));
  // Construct at most one local search tree per ribbon, lazily after cheap gates.
  std::vector<pcl::search::KdTree<pcl::PointXYZI>::Ptr> trees(parts.size());
  for(std::size_t a=0;a<parts.size();++a)for(std::size_t b=a+1;b<parts.size();++b) {
    const auto& x=ribbons[a];const auto& y=ribbons[b];
    if(!x.eligible||!y.eligible||x.parent!=y.parent)continue;
    const float near=std::min(x.depth,y.depth);
    if(std::fabs(x.depth-y.depth)>std::min(.5f,.15f+.01f*near))continue;
    const float overlap=std::max(0.f,std::min(x.u1,y.u1)-std::max(x.u0,y.u0));
    if(overlap<.5f*std::min(x.u1-x.u0,y.u1-y.u0))continue;
    const float allowed=std::min(cfg.scanline_max_gap_m,.2f+cfg.scanline_gap_slope*near);
    if(std::max(x.z0,y.z0)-std::min(x.z1,y.z1)>allowed)continue;
    if(!trees[a]) {
      CloudPtr local(new Cloud);local->reserve(parts[a].indices.size());
      for(int i:parts[a].indices)local->push_back(cloud->at(i));
      trees[a].reset(new pcl::search::KdTree<pcl::PointXYZI>);trees[a]->setInputCloud(local);
    }
    std::vector<int> nearest(1);std::vector<float> distance(1);bool close=false;
    for(int i:parts[b].indices)if(trees[a]->nearestKSearch(cloud->at(i),1,nearest,distance)>0&&distance[0]<=allowed*allowed){close=true;break;}
    if(close)compatible[a][b]=compatible[b][a]=true;
  }
  // A component with alternatives from two depth/instance hypotheses is not
  // resolved by iteration order. Require each connected set to be a small clique.
  std::vector<bool> seen(parts.size(),false);std::vector<pcl::PointIndices> grouped;int groups=0;
  for(std::size_t a=0;a<parts.size();++a) {
    if(seen[a])continue;
    std::vector<std::size_t> set{a};seen[a]=true;
    for(std::size_t n=0;n<set.size();++n)for(std::size_t b=0;b<parts.size();++b)
      if(!seen[b]&&compatible[set[n]][b]){seen[b]=true;set.push_back(b);}
    bool clique=set.size()>=2&&set.size()<=3;
    for(auto x:set)for(auto y:set)if(x!=y&&!compatible[x][y])clique=false;
    if(!clique){for(auto k:set)grouped.push_back(std::move(parts[k]));continue;}
    pcl::PointIndices joined;
    for(auto k:set)joined.indices.insert(joined.indices.end(),parts[k].indices.begin(),parts[k].indices.end());
    grouped.push_back(std::move(joined));++groups;
  }
  parts=std::move(grouped);return groups;
}

// Whole-parent evidence avoids turning one foreground point from a background
// cluster into a new object. Geometry below is invariant to horizontal rotation.
bool SteelPair(const CloudPtr& cloud,const std::vector<pcl::PointIndices>& clusters,
               const std::vector<Pixel>& pixels,const camera::Box& box,
               const CameraPointRefinementConfig& cfg,Candidate& candidate,std::string& reason) {
  struct Leg {int parent;float x,y,z,height,depth;};
  std::vector<Leg> legs;
  for(std::size_t c=0;c<clusters.size();++c) {
    const auto& indices=clusters[c].indices;
    if(indices.size()<3||indices.size()>48)continue;
    int inside=0;float xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY,zmin=INFINITY,zmax=-INFINITY;
    float x=0,y=0,z=0;std::vector<float> depths;bool wall=false;
    for(int i:indices) {
      const auto& q=cloud->at(i);const auto& p=pixels[i];
      xmin=std::min(xmin,q.x);xmax=std::max(xmax,q.x);ymin=std::min(ymin,q.y);ymax=std::max(ymax,q.y);
      zmin=std::min(zmin,q.z);zmax=std::max(zmax,q.z);x+=q.x;y+=q.y;z+=q.z;wall=wall||p.wall;
      if(p.valid&&p.u>=box.xmin&&p.u<=box.xmax&&p.v>=box.ymin&&p.v<=box.ymax){++inside;depths.push_back(p.z);}
    }
    if(wall||inside<3||float(inside)/indices.size()<cfg.steel_minimum_ownership)continue;
    const float h=zmax-zmin;
    // Narrow supports, not an already complete neighboring barrier, curb,
    // horizontal scan ring, wall or long background structure.
    if(h<.15f||h>1.4f||std::hypot(xmax-xmin,ymax-ymin)>1.15f)continue;
    std::nth_element(depths.begin(),depths.begin()+depths.size()/2,depths.end());
    const float n=indices.size();legs.push_back({int(c),x/n,y/n,z/n,h,depths[depths.size()/2]});
  }
  if(legs.size()!=2){reason=legs.size()>2?"steel_ambiguous_parent_set":"steel_insufficient_parents";return false;}
  const auto& a=legs[0];const auto& b=legs[1];const float dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
  if(length<1.1f||length>3.4f||std::fabs(a.z-b.z)>.35f||std::fabs(a.height-b.height)>.4f){reason="steel_incompatible_supports";return false;}
  const float ax=dx/length,ay=dy/length;
  float along_min=INFINITY,along_max=-INFINITY,side_min=INFINITY,side_max=-INFINITY;
  float umin=INFINITY,umax=-INFINITY,vmin=INFINITY,vmax=-INFINITY,gap2=INFINITY;
  for(int i:clusters[a.parent].indices)for(int j:clusters[b.parent].indices) {
    const auto& p=cloud->at(i);const auto& q=cloud->at(j);
    gap2=std::min(gap2,(p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z));
  }
  if(gap2<.65f*.65f||gap2>cfg.steel_maximum_gap_m*cfg.steel_maximum_gap_m){reason="steel_gap_out_of_bounds";return false;}
  for(int parent:{a.parent,b.parent})for(int i:clusters[parent].indices) {
    const auto& p=cloud->at(i);const float along=ax*p.x+ay*p.y,side=-ay*p.x+ax*p.y;
    along_min=std::min(along_min,along);along_max=std::max(along_max,along);
    side_min=std::min(side_min,side);side_max=std::max(side_max,side);
    if(pixels[i].valid){umin=std::min(umin,pixels[i].u);umax=std::max(umax,pixels[i].u);vmin=std::min(vmin,pixels[i].v);vmax=std::max(vmax,pixels[i].v);}
  }
  if(along_max-along_min>3.8f||side_max-side_min>1.15f||
     (umax-umin)/(box.xmax-box.xmin)<.45f||(vmax-vmin)/(box.ymax-box.ymin)<.10f) {
    reason="steel_insufficient_joint_shape";return false;
  }
  candidate.points.indices=clusters[a.parent].indices;
  candidate.points.indices.insert(candidate.points.indices.end(),clusters[b.parent].indices.begin(),clusters[b.parent].indices.end());
  candidate.steel_merge=true;reason="steel_candidate";return true;
}

}  // namespace

CameraPointRefinementResult ContinueMeasuredSteelFragments(
    const CloudPtr& cloud,const std::vector<pcl::PointIndices>& clusters,
    const std::vector<SteelFragmentHistory>& history,const asmc::lidar::Context& context,
    const TunnelCameraInput* camera,const std::vector<TunnelWallEvidence>& walls,
    const CameraPointRefinementConfig& cfg,const std::vector<CameraPointSupport>* current_support) {
  CameraPointRefinementResult out;auto& report=out.report;report.enabled=cfg.steel_fragment_merge;
  const auto start=std::chrono::steady_clock::now();
  auto finish=[&](const char* reason){report.reason=reason;report.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return std::move(out);};
  if(!cfg.steel_fragment_merge||!cloud||!camera||!camera->source_active||!camera->source_current||!camera->source_current()||history.size()>8||clusters.size()>128)
    return finish("no_current_steel_history");
  struct Proposal {int a,b;CameraPointSupport support;};std::vector<Proposal> proposals;
  const auto* current=camera->low_obstacle_camera?camera->low_obstacle_camera.get():camera;
  std::optional<Projection> current_projection;
  if(current->available&&current->calibrated&&current->stamp_ns>0&&current->stamp_ns<=context.time.stamp_ns&&
     context.time.stamp_ns-current->stamp_ns<=250000000LL&&current->still_current&&current->still_current()&&
     (current->stamp_ns==context.time.stamp_ns||current->motion_compensated)) {
    auto sensor_context=context;sensor_context.frame=Frame::Sensor;
    const auto p=camera_projection(sensor_context,current->calibration.base_from_camera,current->calibration.intrinsic);
    const auto base=base_from_input(context);
    if(p&&base)current_projection=(*p)*current->image_sensor_from_scan_sensor.matrix()*(context.base_from_sensor.inverse()* *base).matrix();
  }
  for(const auto& old:history) {
    if(!old.points||old.points->size()<6||old.points->size()>96||old.support.source_class!="steel_barricade"||
       old.support.epoch!=camera->camera_epoch||old.support.identity.empty()||old.support.stamp_ns<=0||
       old.support.stamp_ns>context.time.stamp_ns||context.time.stamp_ns-old.support.stamp_ns>750000000LL)continue;
    std::vector<int> parents;
    for(std::size_t c=0;c<clusters.size();++c) {
      if(current_support&&c<current_support->size()&&current_support->at(c).box_index>=0)continue;
      const auto& points=clusters[c].indices;if(points.size()<3||points.size()>48)continue;
      float xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY,zmin=INFINITY,zmax=-INFINITY;int matches=0;bool wall=false;
      for(int i:points) {
        const auto& p=cloud->at(i);xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);zmin=std::min(zmin,p.z);zmax=std::max(zmax,p.z);
        wall=wall||OnWall(p,walls,cfg.wall_band_m);float nearest=INFINITY;
        for(const auto& q:*old.points)nearest=std::min(nearest,(p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z));
        matches+=nearest<=.2f*.2f;
      }
      if(!wall&&zmax-zmin>=.15f&&zmax-zmin<=1.4f&&std::hypot(xmax-xmin,ymax-ymin)<=1.15f&&
          matches>=3&&float(matches)/points.size()>=cfg.steel_minimum_ownership)parents.push_back(c);
    }
    if(parents.size()!=2)continue;
    const auto& a=clusters[parents[0]].indices;const auto& b=clusters[parents[1]].indices;
    bool competing=false;
    if(current_projection)for(const auto& box:current->boxes) {
      const auto& name=box.source_class_name.empty()?box.class_name:box.source_class_name;
      if(name=="steel_barricade"||!SupportedClass(name)||!box.confidence_valid||box.confidence<cfg.steel_minimum_confidence)continue;
      int overlap=0;
      for(const auto* indices:{&a,&b})for(int i:*indices){const auto& p=cloud->at(i);const Eigen::Vector3d pixel=(*current_projection)*Eigen::Vector4d(p.x,p.y,p.z,1);if(pixel.z()>.1&&pixel.x()/pixel.z()>=box.xmin&&pixel.x()/pixel.z()<=box.xmax&&pixel.y()/pixel.z()>=box.ymin&&pixel.y()/pixel.z()<=box.ymax)++overlap;}
      if(overlap>=3&&float(overlap)/(a.size()+b.size())>=.3f){competing=true;break;}
    }
    if(competing)continue;
    auto center=[&](const std::vector<int>& points){Vec v=Vec::Zero();for(int i:points){const auto& p=cloud->at(i);v+=Vec(p.x,p.y,p.z);}return Vec(v/double(points.size()));};
    const Vec ca=center(a),cb=center(b);
    if(std::hypot(ca.x()-cb.x(),ca.y()-cb.y())<1.1||std::hypot(ca.x()-cb.x(),ca.y()-cb.y())>3.4||std::abs(ca.z()-cb.z())>.35)continue;
    float gap=INFINITY;
    for(int i:a)for(int j:b){const auto& p=cloud->at(i);const auto& q=cloud->at(j);gap=std::min(gap,(p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z));}
    if(gap<.65f*.65f||gap>cfg.steel_maximum_gap_m*cfg.steel_maximum_gap_m)continue;
    bool duplicate=false;
    for(auto& p:proposals)if(p.a==parents[0]&&p.b==parents[1]){duplicate=true;if(old.support.stamp_ns>p.support.stamp_ns)p.support=old.support;break;}
    if(!duplicate)proposals.push_back({parents[0],parents[1],old.support});
  }
  std::vector<int> ownership(clusters.size(),0);
  for(const auto& p:proposals){++ownership[p.a];++ownership[p.b];}
  std::vector<bool> used(clusters.size(),false);
  for(const auto& p:proposals) {
    if(ownership[p.a]!=1||ownership[p.b]!=1)continue;
    pcl::PointIndices current=clusters[p.a];current.indices.insert(current.indices.end(),clusters[p.b].indices.begin(),clusters[p.b].indices.end());
    CameraPointDecision decision;decision.box=p.support.box_index;decision.source_class="steel_barricade";
    decision.reason="steel_measured_continuation";decision.object_points=current.indices.size();
    out.clusters.push_back(std::move(current));out.parent_clusters.push_back(-1);out.support.push_back(p.support);report.decisions.push_back(decision);
    report.image_stamp_ns=std::max(report.image_stamp_ns,p.support.stamp_ns);used[p.a]=used[p.b]=true;++report.extracted_objects;
  }
  if(report.extracted_objects==0)return finish("no_unique_measured_steel_pair");
  for(std::size_t c=0;c<clusters.size();++c)if(!used[c]){out.clusters.push_back(clusters[c]);out.parent_clusters.push_back(c);out.support.emplace_back();}
  if(!camera->source_current()){out.clusters.clear();out.parent_clusters.clear();out.support.clear();report.extracted_objects=0;return finish("camera_changed_during_continuation");}
  report.applied=true;return finish("steel_measured_continuation");
}

void ValidateCameraPointRefinement(const CameraPointRefinementConfig& c) {
  if(!std::isfinite(c.vehicle_history_max_camera_age_s)||c.vehicle_history_max_camera_age_s<=0||c.vehicle_history_max_camera_age_s>30||
      !std::isfinite(c.vehicle_history_match_m)||c.vehicle_history_match_m<.1f||c.vehicle_history_match_m>.4f)
    throw std::invalid_argument("invalid measured vehicle history parameters");
  if(!std::isfinite(c.steel_minimum_confidence)||c.steel_minimum_confidence<.4f||c.steel_minimum_confidence>1||
     !std::isfinite(c.steel_maximum_gap_m)||c.steel_maximum_gap_m<1||c.steel_maximum_gap_m>3||
     !std::isfinite(c.steel_minimum_ownership)||c.steel_minimum_ownership<.8f||c.steel_minimum_ownership>1||
     !std::isfinite(c.minimum_confidence)||c.minimum_confidence<0||c.minimum_confidence>1||
     !std::isfinite(c.max_age_s)||c.max_age_s<=0||c.max_age_s>.15f||
     !std::isfinite(c.connection_m)||c.connection_m<.1f||c.connection_m>1||
     !std::isfinite(c.angular_connection)||c.angular_connection<0||c.angular_connection>.05f||
     !std::isfinite(c.maximum_connection_m)||c.maximum_connection_m<c.connection_m||c.maximum_connection_m>1||
     !std::isfinite(c.ambiguity_depth_m)||c.ambiguity_depth_m<.1f||c.ambiguity_depth_m>3||
     !std::isfinite(c.fragment_connection_m)||c.fragment_connection_m<c.maximum_connection_m||c.fragment_connection_m>1.2f||
     !std::isfinite(c.wall_band_m)||c.wall_band_m<.01f||c.wall_band_m>.25f||
     !std::isfinite(c.scanline_gap_slope)||c.scanline_gap_slope<.01f||c.scanline_gap_slope>.05f||
     !std::isfinite(c.scanline_max_gap_m)||c.scanline_max_gap_m<.65f||c.scanline_max_gap_m>1.5f||
     c.minimum_points<3||c.maximum_box_points<c.minimum_points||c.maximum_box_points>16384||
     c.maximum_boxes<1||c.maximum_boxes>64)
    throw std::invalid_argument("invalid camera point refinement parameters");
}

static CameraPointRefinementResult RefineCameraPointsSingle(const CloudPtr& cloud,
    const std::vector<pcl::PointIndices>& clusters,const asmc::lidar::Context& context,
    const TunnelCameraInput* input,const std::vector<TunnelWallEvidence>& walls,
    const CameraPointRefinementConfig& cfg) {
  CameraPointRefinementResult out;auto& report=out.report;report.enabled=cfg.enabled||cfg.steel_fragment_merge;
  const auto start=std::chrono::steady_clock::now();
  auto finish=[&](const std::string& reason) {
    report.reason=reason;
    report.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    return std::move(out);
  };
  if(!report.enabled)return finish("disabled");
  // Reuse the runtime's causal, motion-compensated camera selection. A recent
  // timestamp without motion evidence is insufficient for geometric deletion.
  const TunnelCameraInput* cam=input;
  if(cam&&cam->low_obstacle_camera)cam=cam->low_obstacle_camera.get();
  // Sparse steel may bridge a brief inference delivery gap using an actual
  // older image and fully bounded measured motion. General geometry refinement
  // remains limited to its original 150ms policy below.
  const auto maximum_age_ns=cfg.steel_fragment_merge?250000000LL:static_cast<std::int64_t>(cfg.max_age_s*1e9);
  if(!cloud||!cam||!cam->calibrated||!cam->available||cam->identity.empty()||
     cam->stamp_ns<=0||cam->stamp_ns>context.time.stamp_ns||
     context.time.stamp_ns-cam->stamp_ns>maximum_age_ns||
     (context.time.stamp_ns!=cam->stamp_ns&&!cam->motion_compensated)||
     !pose_valid(cam->image_sensor_from_scan_sensor)||
     cam->image_width!=static_cast<int>(cam->calibration.width)||
     cam->image_height!=static_cast<int>(cam->calibration.height)||
     !cam->still_current||!cam->still_current())return finish("no_current_synchronized_camera");
  if(cam->boxes.empty())return finish("empty_camera");
  const bool general_current=cfg.enabled&&context.time.stamp_ns-cam->stamp_ns<=static_cast<std::int64_t>(cfg.max_age_s*1e9);
  if(cam->boxes.size()>static_cast<std::size_t>(cfg.maximum_boxes))return finish("box_budget");
  auto sensor_context=context;sensor_context.frame=Frame::Sensor;
  const auto p=camera_projection(sensor_context,cam->calibration.base_from_camera,cam->calibration.intrinsic);
  const auto base=base_from_input(context);
  if(!p||!base)return finish("invalid_projection");
  const Pose sensor_from_input=context.base_from_sensor.inverse()* *base;
  const Projection projection=(*p)*cam->image_sensor_from_scan_sensor.matrix()*sensor_from_input.matrix();
  report.image_stamp_ns=cam->stamp_ns;
  for(int r=0;r<3;++r)for(int c=0;c<4;++c)report.projection[4*r+c]=projection(r,c);
  // Project once per scan. Scalar arithmetic avoids per-point dynamic Eigen
  // expressions in the live debug/O0 build.
  const double p00=projection(0,0),p01=projection(0,1),p02=projection(0,2),p03=projection(0,3);
  const double p10=projection(1,0),p11=projection(1,1),p12=projection(1,2),p13=projection(1,3);
  const double p20=projection(2,0),p21=projection(2,1),p22=projection(2,2),p23=projection(2,3);
  std::vector<Pixel> pixels(cloud->size());std::vector<int> parent(cloud->size(),-1);
  for(std::size_t c=0;c<clusters.size();++c)for(int i:clusters[c].indices)parent.at(i)=c;
  for(std::size_t i=0;i<cloud->size();++i) {
    const auto& q=cloud->at(i);
    auto& v=pixels[i];v.wall=OnWall(q,walls,cfg.wall_band_m);
    const double z=p20*q.x+p21*q.y+p22*q.z+p23;
    if(!std::isfinite(z)||z<=.1||z>80)continue;
    v.u=(p00*q.x+p01*q.y+p02*q.z+p03)/z;
    v.v=(p10*q.x+p11*q.y+p12*q.z+p13)/z;v.z=z;
    v.valid=std::isfinite(v.u)&&std::isfinite(v.v)&&v.u>=0&&v.u<cam->image_width&&v.v>=0&&v.v<cam->image_height;
    if(v.valid)++report.projected_points;
  }
  std::vector<Candidate> candidates;
  std::vector<std::vector<int>> box_membership(cam->boxes.size());
  for(std::size_t b=0;b<cam->boxes.size();++b) {
    const auto& box=cam->boxes[b];
    const auto& name=box.source_class_name.empty()?box.class_name:box.source_class_name;
    const bool steel=cfg.steel_fragment_merge&&name=="steel_barricade";
    if(!general_current&&!steel)continue;
    const float minimum_confidence=steel?cfg.steel_minimum_confidence:cfg.minimum_confidence;
    if(!SupportedClass(name)||!box.confidence_valid||!std::isfinite(box.confidence)||box.confidence<minimum_confidence) {
      CameraPointDecision d;d.box=b;d.camera_id=cam->camera_id;d.source_class=name;
      d.reason=!SupportedClass(name)?"unsupported_class":"below_confidence";
      report.decisions.push_back(std::move(d));continue;
    }
    const float w=box.xmax-box.xmin,h=box.ymax-box.ymin;
    if(!std::isfinite(w)||!std::isfinite(h)||w<=0||h<=0||box.xmin<0||box.ymin<0||
       box.xmax>cam->image_width||box.ymax>cam->image_height)continue;
    const bool vehicle=name=="car"||name=="truck"||name=="bus";
    if(vehicle&&w>=cam->image_width*.6f&&.5f*(box.ymin+box.ymax)>=cam->image_height*.85f&&box.ymax>=cam->image_height*.95f)continue;
    CameraPointDecision decision;decision.box=b;decision.camera_id=cam->camera_id;decision.source_class=name;
    const auto di=report.decisions.size();report.decisions.push_back(decision);
    auto& d=report.decisions.back();
    std::vector<int> indices;std::vector<float> depths;
    for(std::size_t i=0;i<pixels.size();++i) {
      const auto& v=pixels[i];
      if(!v.valid||v.u<box.xmin||v.u>box.xmax||v.v<box.ymin||v.v>box.ymax)continue;
      box_membership[b].push_back(i);
      ++d.input_points;
      if(v.wall){++d.wall_points;continue;}
      indices.push_back(i);depths.push_back(v.z);
    }
    if(box.xmin<=1||box.ymin<=1||box.xmax>=cam->image_width-1||box.ymax>=cam->image_height-1) {
      d.reason="truncated_image_box";continue;
    }
    if(steel) {
      Candidate c;
      if(SteelPair(cloud,clusters,pixels,box,cfg,c,d.reason)) {
        bool competing_class=false;
        for(std::size_t other=0;other<cam->boxes.size();++other) {
          if(other==b)continue;
          const auto& ob=cam->boxes[other];const auto& oc=ob.source_class_name.empty()?ob.class_name:ob.source_class_name;
          if(oc=="steel_barricade"||!SupportedClass(oc)||!ob.confidence_valid||!std::isfinite(ob.confidence)||ob.confidence<cfg.steel_minimum_confidence)continue;
          int overlap=0;
          for(int i:c.points.indices){const auto& v=pixels[i];if(v.valid&&v.u>=ob.xmin&&v.u<=ob.xmax&&v.v>=ob.ymin&&v.v<=ob.ymax)++overlap;}
          if(overlap>=3&&float(overlap)/c.points.indices.size()>=.3f){competing_class=true;break;}
        }
        if(competing_class){d.reason="steel_competing_other_class";continue;}
        c.decision=di;c.confidence=box.confidence;
        c.support={cam->stamp_ns,cam->camera_epoch,static_cast<int>(b),cam->identity,name};
        std::vector<float> merged_depths;
        for(int i:c.points.indices)merged_depths.push_back(pixels[i].z);
        std::nth_element(merged_depths.begin(),merged_depths.begin()+merged_depths.size()/2,merged_depths.end());
        d.depth_m=merged_depths[merged_depths.size()/2];
        d.object_points=c.points.indices.size();candidates.push_back(std::move(c));continue;
      }
      if(!general_current||box.confidence<cfg.minimum_confidence)continue;
    }
    if(indices.size()<static_cast<std::size_t>(cfg.minimum_points)){d.reason="insufficient_nonwall_support";continue;}
    if(indices.size()>static_cast<std::size_t>(cfg.maximum_box_points)){d.reason="point_budget";continue;}
    // Adaptive tolerance is set by the near depth quartile, never by a far
    // wall's range. No fixed vehicle-length depth window is used.
    auto quartile=depths.begin()+depths.size()/4;std::nth_element(depths.begin(),quartile,depths.end());
    const float tolerance=std::min(cfg.maximum_connection_m,cfg.connection_m+cfg.angular_connection* *quartile);
    auto components=Components(cloud,indices,tolerance,3);d.components=components.size();
    if(vehicle&&cfg.vehicle_scanline_grouping)
      d.scanline_groups=GroupVehicleScanlines(cloud,pixels,parent,w,h,cfg,components);
    struct Surface {std::size_t index;float depth;};std::vector<Surface> surfaces;
    float nearest_evidence=INFINITY;
    for(std::size_t k=0;k<components.size();++k) {
      float xmin=INFINITY,ymin=INFINITY,zmin=INFINITY,xmax=-INFINITY,ymax=-INFINITY,zmax=-INFINITY;
      float umin=INFINITY,vmin=INFINITY,umax=-INFINITY,vmax=-INFINITY;depths.clear();
      for(int i:components[k].indices) {
        const auto& q=cloud->at(i);const auto& v=pixels[i];depths.push_back(v.z);
        xmin=std::min(xmin,q.x);xmax=std::max(xmax,q.x);ymin=std::min(ymin,q.y);ymax=std::max(ymax,q.y);
        zmin=std::min(zmin,q.z);zmax=std::max(zmax,q.z);
        umin=std::min(umin,v.u);umax=std::max(umax,v.u);vmin=std::min(vmin,v.v);vmax=std::max(vmax,v.v);
      }
      const auto mid=depths.begin()+depths.size()/2;std::nth_element(depths.begin(),mid,depths.end());
      const float median=*mid;nearest_evidence=std::min(nearest_evidence,median);
      if(components[k].indices.size()<static_cast<std::size_t>(cfg.minimum_points))continue;
      const float max_xy=name=="bus"||name=="truck"?22.f:vehicle?9.f:6.f;
      const float max_z=name=="bus"||name=="truck"?5.f:3.5f;
      // Broad upper bounds reject extended structures, without inventing full
      // object dimensions from a partially observed surface.
      if(std::hypot(xmax-xmin,ymax-ymin)>max_xy||zmax-zmin>max_z)continue;
      if((umax-umin)/w<.15f||(vmax-vmin)/h<.08f)continue;
      if(vehicle&&((umax-umin)/w)*((vmax-vmin)/h)<.10f)continue;
      surfaces.push_back({k,median});
    }
    if(surfaces.empty()){d.reason="no_supported_surface";continue;}
    std::sort(surfaces.begin(),surfaces.end(),[](const auto& a,const auto& b){return a.depth<b.depth;});
    if(surfaces.size()>1&&surfaces[1].depth-surfaces[0].depth<cfg.ambiguity_depth_m) {
      d.reason="ambiguous_near_surfaces";continue;
    }
    const auto& surface=surfaces.front();
    if(surface.depth-nearest_evidence>cfg.ambiguity_depth_m) {
      d.reason="unsupported_nearer_surface";continue;
    }
    // Sparse scan rings can split a roof from the main body even though the
    // original LiDAR cluster joined them. Retain nearby subthreshold fragments
    // from those SAME parents; never attach a second plausible object surface.
    // If this cannot explain the near-depth support, abstain instead of shrinking
    // the object to one scan ring. All operations remain local and bounded.
    if(components.size()>64){d.reason="fragment_budget";continue;}
    Candidate c;c.points=components[surface.index];c.decision=di;c.confidence=box.confidence;
    std::vector<bool> joined(components.size(),false),supported(components.size(),false),parents(clusters.size(),false);
    joined[surface.index]=true;for(const auto& s:surfaces)supported[s.index]=true;
    for(int i:c.points.indices)if(parent[i]>=0)parents[parent[i]]=true;
    for(int round=0;round<3;++round) {
      CloudPtr seed(new Cloud);float lo=INFINITY,hi=-INFINITY;
      for(int i:c.points.indices){seed->push_back(cloud->at(i));lo=std::min(lo,pixels[i].z);hi=std::max(hi,pixels[i].z);}
      pcl::search::KdTree<pcl::PointXYZI> tree;tree.setInputCloud(seed);
      bool changed=false;std::vector<int> neighbors(1);std::vector<float> squared(1);
      for(std::size_t k=0;k<components.size();++k) {
        if(joined[k]||supported[k])continue;
        const auto& fragment=components[k].indices;
        if(std::any_of(fragment.begin(),fragment.end(),[&](int i){return parent[i]<0||!parents[parent[i]];}))continue;
        if(std::any_of(fragment.begin(),fragment.end(),[&](int i){return pixels[i].z<lo-cfg.ambiguity_depth_m||pixels[i].z>hi+cfg.ambiguity_depth_m;}))continue;
        bool attached=false;
        for(int i:fragment)if(tree.nearestKSearch(cloud->at(i),1,neighbors,squared)>0&&
            squared[0]<=cfg.fragment_connection_m*cfg.fragment_connection_m){attached=true;break;}
        if(attached){joined[k]=true;changed=true;c.points.indices.insert(c.points.indices.end(),fragment.begin(),fragment.end());}
      }
      if(!changed)break;
    }
    float xmin=INFINITY,ymin=INFINITY,zmin=INFINITY,xmax=-INFINITY,ymax=-INFINITY,zmax=-INFINITY;
    float depth_min=INFINITY,depth_max=-INFINITY;
    for(int i:c.points.indices) {
      const auto& q=cloud->at(i);xmin=std::min(xmin,q.x);xmax=std::max(xmax,q.x);
      ymin=std::min(ymin,q.y);ymax=std::max(ymax,q.y);zmin=std::min(zmin,q.z);zmax=std::max(zmax,q.z);
      depth_min=std::min(depth_min,pixels[i].z);depth_max=std::max(depth_max,pixels[i].z);
    }
    int unexplained=0;
    for(std::size_t k=0;k<components.size();++k)if(!joined[k])for(int i:components[k].indices)
      if(parent[i]>=0&&parents[parent[i]]&&pixels[i].z>=depth_min-cfg.ambiguity_depth_m&&
          pixels[i].z<=depth_max+cfg.ambiguity_depth_m)++unexplained;
    if(unexplained>=std::max(3,static_cast<int>(.1f*c.points.indices.size()))) {
      d.reason="incomplete_surface_support";continue;
    }
    const float max_xy=name=="bus"||name=="truck"?22.f:vehicle?9.f:6.f;
    if(std::hypot(xmax-xmin,ymax-ymin)>max_xy||zmax-zmin>(name=="bus"||name=="truck"?5.f:3.5f)) {
      d.reason="extended_fragment_group";continue;
    }
    c.support={cam->stamp_ns,cam->camera_epoch,static_cast<int>(b),cam->identity,name};
    d.depth_m=surface.depth;d.object_points=c.points.indices.size();d.reason="candidate";
    candidates.push_back(std::move(c));
  }
  // Conflicting boxes cannot create duplicate measurements. Resolve by detector
  // confidence; reject the entire lower-priority candidate, never carve it up.
  std::stable_sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.confidence>b.confidence;});
  std::vector<int> owner(cloud->size(),-1);std::vector<bool> touched(clusters.size(),false);
  // A sparse leg shared by different proposed instances has no unique camera
  // owner. Confidence ranking must not decide which physical barrier owns it.
  std::vector<bool> ambiguous(candidates.size(),false);
  for(std::size_t i=0;i<candidates.size();++i)for(std::size_t j=i+1;j<candidates.size();++j) {
    if(!candidates[i].steel_merge&&!candidates[j].steel_merge)continue;
    auto a=candidates[i].points.indices,b=candidates[j].points.indices;
    std::sort(a.begin(),a.end());std::sort(b.begin(),b.end());
    if(a==b)continue;
    std::vector<int> intersection;std::set_intersection(a.begin(),a.end(),b.begin(),b.end(),std::back_inserter(intersection));
    if(!intersection.empty()){ambiguous[i]=ambiguous[j]=true;}
  }
  std::size_t candidate_index=0;
  for(auto& c:candidates) {
    auto& d=report.decisions[c.decision];
    if(ambiguous[candidate_index++]){d.reason="steel_competing_instance_ownership";continue;}
    if(std::any_of(c.points.indices.begin(),c.points.indices.end(),[&](int i){return owner[i]>=0;})) {
      d.reason="overlapping_box_conflict";continue;
    }
    const auto next=out.clusters.size();
    int original_parent=parent[c.points.indices.front()];
    for(int i:c.points.indices)if(parent[i]!=original_parent){original_parent=-1;break;}
    for(int i:c.points.indices){owner[i]=next;if(parent[i]>=0)touched[parent[i]]=true;}
    out.clusters.push_back(std::move(c.points));out.parent_clusters.push_back(original_parent);out.support.push_back(std::move(c.support));
    d.reason=c.steel_merge?"steel_fragments_joined":"extracted";++report.extracted_objects;
  }
  // Extracting a car must not erase a sparse pedestrian against the same wall.
  // Unresolved high-confidence boxes veto structural deletion in their image
  // regions. Those points remain residual LiDAR support, without an invented
  // camera class. Boxes resolved to an existing candidate need no second veto.
  std::vector<bool> resolved(cam->boxes.size(),false),wall_veto(cloud->size(),false);
  for(const auto& d:report.decisions)
    if(d.reason=="extracted"||d.reason=="steel_fragments_joined"||d.reason=="overlapping_box_conflict")resolved[d.box]=true;
  for(std::size_t b=0;b<box_membership.size();++b)if(!resolved[b])
    for(int i:box_membership[b])wall_veto[i]=true;
  for(std::size_t i=0;i<wall_veto.size();++i)if(wall_veto[i])out.wall_veto_indices.push_back(i);
  // Wall authority is independent of successful object extraction. Only touch
  // original parents supported by current qualified boxes; unresolved boxes
  // veto deletion inside their rectangles. Unrelated clusters remain byte-for-byte.
  if(general_current&&cfg.independent_wall_cleanup) {
    std::vector<int> camera_support(clusters.size(),0),removable(clusters.size(),0);
    std::vector<bool> in_any_box(cloud->size(),false);
    for(const auto& members:box_membership)for(int i:members)in_any_box[i]=true;
    for(std::size_t i=0;i<cloud->size();++i)if(parent[i]>=0) {
      if(in_any_box[i]&&!pixels[i].wall)++camera_support[parent[i]];
      if(pixels[i].wall&&!wall_veto[i]&&owner[i]<0)++removable[parent[i]];
    }
    for(std::size_t c=0;c<clusters.size();++c)
      if(camera_support[c]>=cfg.minimum_points&&removable[c]>=cfg.minimum_points)touched[c]=true;
    // Detached wall strips need a measured foreground anchor, their own tall
    // planar support and a depth gap. Never grow the object or erase non-wall
    // residuals, unresolved instances or points outside the camera field of view.
    if(cfg.detached_wall_cleanup&&report.extracted_objects>0) {
      float foreground_far=0;
      for(std::size_t i=0;i<owner.size();++i)if(owner[i]>=0)foreground_far=std::max(foreground_far,pixels[i].z);
      for(std::size_t c=0;c<clusters.size();++c) {
        if(touched[c]||clusters[c].indices.size()<10||clusters[c].indices.size()>static_cast<std::size_t>(cfg.maximum_box_points))continue;
        float lo=INFINITY,hi=-INFINITY;bool safe=true;
        for(int i:clusters[c].indices) {
          lo=std::min(lo,cloud->at(i).z);hi=std::max(hi,cloud->at(i).z);
          if(owner[i]>=0||wall_veto[i]||!pixels[i].valid||pixels[i].z<=foreground_far+cfg.ambiguity_depth_m)safe=false;
        }
        if(!safe||hi-lo<1.5f)continue;
        for(const auto& wall:walls) {
          if(!wall.allowed||!wall.valid)continue;
          const std::vector<TunnelWallEvidence> measured{wall};
          int support=0;
          for(int i:clusters[c].indices)if(OnWall(cloud->at(i),measured,cfg.wall_band_m))++support;
          if(support*20>=static_cast<int>(clusters[c].indices.size())*19){touched[c]=true;break;}
        }
      }
    }
  }
  if(report.extracted_objects==0&&std::none_of(touched.begin(),touched.end(),[](bool b){return b;}))
    return finish("no_unambiguous_object");
  for(std::size_t c=0;c<clusters.size();++c) {
    if(!touched[c]) {
      out.clusters.push_back(clusters[c]);out.parent_clusters.push_back(c);out.support.emplace_back();continue;
    }
    std::vector<int> residual;
    for(int i:clusters[c].indices) {
      if(owner[i]>=0)continue;
      if(pixels[i].wall&&!wall_veto[i])out.removed_wall_indices.push_back(i);
      else residual.push_back(i);
    }
    // Preserve unassigned non-wall points in the source cloud. Residuals below
    // the LiDAR minimum stay unclustered rather than creating singleton tracks.
    for(auto& part:Components(cloud,residual,cfg.maximum_connection_m,3)) {
      out.clusters.push_back(std::move(part));out.parent_clusters.push_back(-2);out.support.emplace_back();
    }
  }
  // Transactional application: an epoch reset during work discards the plan.
  if(!cam->still_current()) {
    out.clusters.clear();out.parent_clusters.clear();out.support.clear();out.removed_wall_indices.clear();
    report.extracted_objects=0;return finish("camera_changed_during_refinement");
  }
  report.applied=true;report.removed_wall_points=out.removed_wall_indices.size();
  return finish(report.extracted_objects?"applied":"confirmed_wall_cleanup_only");
}

CameraPointRefinementResult RefineCameraPoints(const CloudPtr& cloud,
    const std::vector<pcl::PointIndices>& clusters,const asmc::lidar::Context& context,
    const TunnelCameraInput* input,const std::vector<TunnelWallEvidence>& walls,
    const CameraPointRefinementConfig& cfg) {
  if(!input||input->refinement_views.empty())return RefineCameraPointsSingle(cloud,clusters,context,input,walls,cfg);
  CameraPointRefinementResult out;auto& report=out.report;
  report.enabled=cfg.enabled||cfg.steel_fragment_merge;
  const auto start=std::chrono::steady_clock::now();
  auto finish=[&](const std::string& reason) {
    report.reason=reason;report.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    return std::move(out);
  };
  if(!report.enabled)return finish("disabled");
  if(!cloud)return finish("no_cloud");
  // Every camera proposes a plan against the SAME measured cloud. Sequential
  // mutation would let the first camera remove evidence needed by another.
  struct Plan {const TunnelCameraInput* camera;CameraPointRefinementResult result;std::size_t decision_offset;};
  std::vector<Plan> plans;
  std::vector<bool> veto(cloud->size(),false),remove(cloud->size(),false);
  for(const auto& camera:input->refinement_views) {
    if(!camera)continue;
    Plan plan{camera.get(),RefineCameraPointsSingle(cloud,clusters,context,camera.get(),walls,cfg),report.decisions.size()};
    const auto& r=plan.result.report;
    CameraPointViewReport view;view.camera_id=camera->camera_id;view.reason=r.reason;
    view.image_stamp_ns=r.image_stamp_ns;view.projection=r.projection;view.projected_points=r.projected_points;
    report.views.push_back(std::move(view));report.projected_points+=r.projected_points;
    if(camera->camera_id=="camera_front"){report.image_stamp_ns=r.image_stamp_ns;report.projection=r.projection;}
    report.decisions.insert(report.decisions.end(),r.decisions.begin(),r.decisions.end());
    for(int i:plan.result.wall_veto_indices)veto.at(i)=true;
    if(r.applied)for(int i:plan.result.removed_wall_indices)remove.at(i)=true;
    plans.push_back(std::move(plan));
  }
  struct Proposal {std::size_t view,cluster,decision;float confidence;};
  std::vector<Proposal> proposals;
  for(std::size_t v=0;v<plans.size();++v) {
    const auto& plan=plans[v];if(!plan.result.report.applied)continue;
    for(std::size_t c=0;c<plan.result.clusters.size();++c) {
      const auto& support=plan.result.support[c];if(support.box_index<0)continue;
      for(std::size_t d=0;d<plan.result.report.decisions.size();++d) {
        const auto& decision=plan.result.report.decisions[d];
        if(decision.box==support.box_index&&(decision.reason=="extracted"||decision.reason=="steel_fragments_joined")) {
          proposals.push_back({v,c,plan.decision_offset+d,plan.camera->boxes.at(support.box_index).confidence});break;
        }
      }
    }
  }
  std::stable_sort(proposals.begin(),proposals.end(),[](const auto& a,const auto& b){return a.confidence>b.confidence;});
  std::vector<int> owner(cloud->size(),-1),parent(cloud->size(),-1);
  std::vector<bool> touched(clusters.size(),false);
  for(std::size_t c=0;c<clusters.size();++c)for(int i:clusters[c].indices)parent.at(i)=c;
  // Even a duplicate proposal that loses cross-view ownership is measured
  // foreground evidence; another view cannot erase its unshared tail as wall.
  for(const auto& proposal:proposals)
    for(int i:plans[proposal.view].result.clusters[proposal.cluster].indices)veto.at(i)=true;
  for(std::size_t i=0;i<veto.size();++i)if(veto[i])out.wall_veto_indices.push_back(i);
  for(const auto& proposal:proposals) {
    const auto& plan=plans[proposal.view];const auto& points=plan.result.clusters[proposal.cluster];
    auto& decision=report.decisions[proposal.decision];
    if(std::any_of(points.indices.begin(),points.indices.end(),[&](int i){return owner.at(i)>=0;})) {
      decision.reason="cross_view_overlap_conflict";continue;
    }
    const auto next=out.clusters.size();
    for(int i:points.indices){owner.at(i)=next;if(parent.at(i)>=0)touched[parent[i]]=true;}
    out.clusters.push_back(points);out.parent_clusters.push_back(plan.result.parent_clusters[proposal.cluster]);
    out.support.push_back(plan.result.support[proposal.cluster]);
    ++report.extracted_objects;++report.views[proposal.view].extracted_objects;report.views[proposal.view].applied=true;
  }
  // A point claimed as an object by ANY view or inside an unresolved qualified
  // box cannot be deleted by a different camera's wall plan.
  for(std::size_t i=0;i<remove.size();++i) {
    remove[i]=remove[i]&&!veto[i]&&owner[i]<0;
    if(remove[i]) {
      out.removed_wall_indices.push_back(i);if(parent[i]>=0)touched[parent[i]]=true;
      for(std::size_t v=0;v<plans.size();++v) {
        const auto& indices=plans[v].result.removed_wall_indices;
        if(std::find(indices.begin(),indices.end(),i)!=indices.end()) {
          ++report.views[v].removed_wall_points;report.views[v].applied=true;
        }
      }
    }
  }
  if(!report.extracted_objects&&out.removed_wall_indices.empty())return finish("no_unambiguous_object");
  for(std::size_t c=0;c<clusters.size();++c) {
    if(!touched[c]){out.clusters.push_back(clusters[c]);out.parent_clusters.push_back(c);out.support.emplace_back();continue;}
    std::vector<int> residual;
    for(int i:clusters[c].indices)if(owner.at(i)<0&&!remove.at(i))residual.push_back(i);
    for(auto& part:Components(cloud,residual,cfg.maximum_connection_m,3)) {
      out.clusters.push_back(std::move(part));out.parent_clusters.push_back(-2);out.support.emplace_back();
    }
  }
  // Commit all views as one transaction. A reset of any participating image
  // invalidates its veto/ownership evidence as well as its extracted objects.
  for(const auto& plan:plans)if(plan.result.report.image_stamp_ns>0&&
      (!plan.camera->still_current||!plan.camera->still_current())) {
    out.clusters.clear();out.parent_clusters.clear();out.support.clear();out.removed_wall_indices.clear();
    report.extracted_objects=0;for(auto& view:report.views){view.applied=false;view.extracted_objects=0;view.removed_wall_points=0;}
    return finish("camera_changed_during_refinement");
  }
  report.applied=true;report.removed_wall_points=out.removed_wall_indices.size();
  return finish(report.extracted_objects?"applied":"confirmed_wall_cleanup_only");
}

CameraPointRefinementResult ContinueMeasuredVehicles(const CloudPtr& cloud,
    const std::vector<pcl::PointIndices>& clusters,const std::vector<MeasuredVehicleHistory>& history,
    const std::vector<TunnelWallEvidence>& walls,const CameraPointRefinementConfig& cfg,
    const std::vector<CameraPointSupport>* current_support,const std::vector<int>* wall_veto,
    std::vector<int>* track_ids) {
  CameraPointRefinementResult out;auto& report=out.report;report.enabled=cfg.enabled&&cfg.vehicle_history_enabled;
  const auto start=std::chrono::steady_clock::now();
  auto finish=[&](const char* reason){report.reason=reason;report.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return std::move(out);};
  if(track_ids)track_ids->clear();
  if(!report.enabled||!cloud||!track_ids||history.size()>32)return finish("vehicle_history_unavailable");
  std::vector<int> parent(cloud->size(),-1),owner(cloud->size(),-1);
  std::vector<bool> blocked(cloud->size(),false),veto(cloud->size(),false),touched(clusters.size(),false);
  if(wall_veto)for(int i:*wall_veto)veto.at(i)=true;
  for(std::size_t c=0;c<clusters.size();++c)for(int i:clusters[c].indices) {
    parent.at(i)=c;if(current_support&&c<current_support->size()&&current_support->at(c).box_index>=0)blocked.at(i)=true;
  }
  struct Proposal {int id;std::vector<int> points;};std::vector<Proposal> proposals;
  std::vector<int> fresh_track_ids(clusters.size(),-1);
  for(const auto& old:history) {
    if(old.track_id<0||old.confirmations<5||!old.points||old.points->size()<10||old.points->size()>2048)continue;
    pcl::search::KdTree<pcl::PointXYZI> tree;tree.setInputCloud(old.points);
    Proposal proposal{old.track_id,{}};std::vector<bool> matched(old.points->size(),false);
    std::vector<int> near(1);std::vector<float> distance(1);std::map<int,int> fresh_matches;
    float xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY,zmin=INFINITY,zmax=-INFINITY;
    for(std::size_t i=0;i<cloud->size();++i) {
      const auto& p=cloud->at(i);
      if(!pcl::isFinite(p)||OnWall(p,walls,cfg.wall_band_m)||tree.nearestKSearch(p,1,near,distance)!=1||distance[0]>cfg.vehicle_history_match_m*cfg.vehicle_history_match_m)continue;
      if(blocked[i]){++fresh_matches[parent[i]];matched[near[0]]=true;continue;}
      proposal.points.push_back(i);matched[near[0]]=true;
      xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);zmin=std::min(zmin,p.z);zmax=std::max(zmax,p.z);
    }
    CameraPointDecision d;d.camera_id="lidar_history";d.box=old.track_id;d.source_class="car";
    d.input_points=proposal.points.size();
    const auto covered=static_cast<std::size_t>(std::count(matched.begin(),matched.end(),true));
    if(!fresh_matches.empty()) {
      const auto best=std::max_element(fresh_matches.begin(),fresh_matches.end(),[](const auto& a,const auto& b){return a.second<b.second;});
      const auto& current=current_support->at(best->first).source_class;
      if(fresh_matches.size()==1&&best->second>=10&&covered>=std::max<std::size_t>(5,old.points->size()*15/100)&&
          (current=="car"||current=="truck"||current=="bus")) {
        int& id=fresh_track_ids[best->first];id=id==-1?old.track_id:-2;
        d.reason="vehicle_history_current_camera_confirmed";d.object_points=best->second;
      } else d.reason="vehicle_history_current_camera_ownership";
    }
    else if(proposal.points.size()<10||covered<std::max<std::size_t>(5,old.points->size()*15/100))d.reason="vehicle_history_insufficient_measured_support";
    else if(std::hypot(xmax-xmin,ymax-ymin)<.4f||std::hypot(xmax-xmin,ymax-ymin)>10.f||zmax-zmin<.15f||zmax-zmin>3.5f)d.reason="vehicle_history_shape_rejected";
    else {d.reason="vehicle_history_candidate";proposals.push_back(std::move(proposal));}
    report.decisions.push_back(std::move(d));
  }
  std::vector<int> votes(cloud->size(),0);for(const auto& p:proposals)for(int i:p.points)++votes[i];
  for(const auto& p:proposals) {
    auto decision=std::find_if(report.decisions.begin(),report.decisions.end(),[&](const auto& d){return d.box==p.id;});
    if(std::any_of(p.points.begin(),p.points.end(),[&](int i){return votes[i]>1;})) {
      if(decision!=report.decisions.end())decision->reason="vehicle_history_competing_tracks";
      continue;
    }
    pcl::PointIndices selected;selected.indices=p.points;
    const int next=out.clusters.size();for(int i:p.points){owner[i]=next;if(parent[i]>=0)touched[parent[i]]=true;}
    out.clusters.push_back(std::move(selected));out.parent_clusters.push_back(-1);out.support.emplace_back();track_ids->push_back(p.id);
    if(decision!=report.decisions.end()){decision->reason="vehicle_measured_continuation";decision->object_points=p.points.size();}
    ++report.extracted_objects;
  }
  const bool fresh_associated=std::any_of(fresh_track_ids.begin(),fresh_track_ids.end(),[](int id){return id>=0;});
  if(!report.extracted_objects&&!fresh_associated)return finish("no_verified_vehicle_history");
  for(std::size_t c=0;c<clusters.size();++c) {
    if(!touched[c]){out.clusters.push_back(clusters[c]);out.parent_clusters.push_back(c);out.support.emplace_back();track_ids->push_back(fresh_track_ids[c]>=0?fresh_track_ids[c]:-1);continue;}
    std::vector<int> residual;
    for(int i:clusters[c].indices)if(owner[i]<0) {
      if(!veto[i]&&OnWall(cloud->at(i),walls,cfg.wall_band_m))out.removed_wall_indices.push_back(i);
      else residual.push_back(i);
    }
    for(auto& part:Components(cloud,residual,cfg.maximum_connection_m,3)) {
      out.clusters.push_back(std::move(part));out.parent_clusters.push_back(-2);out.support.emplace_back();track_ids->push_back(-1);
    }
  }
  report.applied=true;report.removed_wall_points=out.removed_wall_indices.size();
  return finish("vehicle_measured_continuation");
}
}
