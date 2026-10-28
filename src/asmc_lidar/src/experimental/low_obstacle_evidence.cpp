#include <asmc_lidar/experimental/low_obstacle_evidence.hpp>
#include <asmc_lidar/geometry/projection.hpp>
#include <asmc_lidar/geometry/scan_frame.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace asmc::lidar::pipeline {
std::vector<LowObstacleHint> LowObstacleEvidence(
    const Cloud& sensor, const tunnel_filter::FrameResult& f,
    const VehicleSpec& vehicle, std::int64_t stamp,
    const TunnelCameraInput* camera, const TunnelRouteConfig& cfg) {
  std::vector<LowObstacleHint> result(f.clusters.size());
  if(camera && camera->low_obstacle_camera)camera=camera->low_obstacle_camera.get();
  if(!cfg.low_obstacle_evidence || !camera || !camera->calibrated || !camera->available ||
     camera->identity.empty() || camera->stamp_ns<=0 ||
     camera->stamp_ns>stamp || stamp-camera->stamp_ns>150000000 ||
     (stamp!=camera->stamp_ns&&!camera->motion_compensated) ||
     !pose_valid(camera->image_sensor_from_scan_sensor) ||
     camera->image_width!=static_cast<int>(camera->calibration.width) ||
     camera->image_height!=static_cast<int>(camera->calibration.height) ||
     f.ground.source==tunnel_filter::PlaneSource::kUnavailable ||
     !camera->still_current || !camera->still_current())return result;
  Context context;context.frame=Frame::Sensor;
  context.base_from_sensor=ScanFrame::SensorMount(vehicle);
  context.calibration_source=camera->calibration.id;
  const auto projection=camera_projection(context,camera->calibration.base_from_camera,camera->calibration.intrinsic);
  if(!projection || f.leveled->size()!=sensor.size() || camera->boxes.size()>64)return result;
  // Compose once per scan. Avoid thousands of unoptimized Eigen transform
  // temporaries in the live O0 build; preprocessing and point values are intact.
  const Projection scan_projection=(*projection)*camera->image_sensor_from_scan_sensor.matrix();
  const double p00=scan_projection(0,0),p01=scan_projection(0,1),p02=scan_projection(0,2),p03=scan_projection(0,3);
  const double p10=scan_projection(1,0),p11=scan_projection(1,1),p12=scan_projection(1,2),p13=scan_projection(1,3);
  const double p20=scan_projection(2,0),p21=scan_projection(2,1),p22=scan_projection(2,2),p23=scan_projection(2,3);
  struct Pixel {float u=0,v=0,depth=0;bool valid=false;};
  std::vector<Pixel> pixels(sensor.size());
  for(std::size_t i=0;i<sensor.size();++i) {
    const auto& p=sensor[i];const auto& q=f.leveled->at(i);
    // Ground is never restored. Include walls/ceiling in depth competition:
    // their presence must not make a background cluster look like foreground.
    if(f.labels.at(i)==tunnel_filter::kGround || f.labels.at(i)==tunnel_filter::kInvalid ||
       q.z-f.ground.plane.HeightAt(q.x,q.y)<=cfg.params.ground_remove_distance_m)continue;
    const double hx=p00*p.x+p01*p.y+p02*p.z+p03;
    const double hy=p10*p.x+p11*p.y+p12*p.z+p13;
    const double hz=p20*p.x+p21*p.y+p22*p.z+p23;
    if(!std::isfinite(hx)||!std::isfinite(hy)||!std::isfinite(hz)||hz<=.1||hz>30)continue;
    auto& pixel=pixels[i];pixel.u=hx/hz;pixel.v=hy/hz;pixel.depth=hz;
    pixel.valid=pixel.u>=0&&pixel.u<camera->image_width&&pixel.v>=0&&pixel.v<camera->image_height;
  }
  for(const auto& box:camera->boxes) {
    const auto& name=box.source_class_name;
    if((name!="box"&&name!="cargo_box"&&name!="steel_barricade") ||
       !box.confidence_valid || !std::isfinite(box.confidence) || box.confidence<.7f ||
       !std::isfinite(box.xmin)||!std::isfinite(box.xmax)||!std::isfinite(box.ymin)||!std::isfinite(box.ymax)||
       box.xmin<0||box.ymin<0||box.xmax>camera->image_width||box.ymax>camera->image_height||
       box.xmax<=box.xmin||box.ymax<=box.ymin)continue;
    CloudPtr points(new Cloud);std::vector<int> original;
    for(std::size_t i=0;i<pixels.size();++i) {
      const auto& p=pixels[i];
      // Deliberately no bounding-box expansion.
      if(p.valid&&p.u>=box.xmin&&p.u<=box.xmax&&p.v>=box.ymin&&p.v<=box.ymax) {
        points->push_back(sensor[i]);original.push_back(i);
      }
    }
    if(points->size()<5 || points->size()>4096)continue;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(points);pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(points);ec.setSearchMethod(tree);ec.setClusterTolerance(.35);
    ec.setMinClusterSize(5);ec.setMaxClusterSize(4096);
    std::vector<pcl::PointIndices> components;ec.extract(components);
    struct Component {float depth;std::size_t index;};std::vector<Component> ordered;
    for(std::size_t k=0;k<components.size();++k) {
      std::vector<float> depths;
      for(int j:components[k].indices)depths.push_back(pixels[original[j]].depth);
      std::sort(depths.begin(),depths.end());ordered.push_back({depths[depths.size()/2],k});
    }
    if(ordered.empty())continue;
    std::sort(ordered.begin(),ordered.end(),[](const auto& a,const auto& b){return a.depth<b.depth;});
    // Multiple equally near surfaces cannot be assigned safely.
    if(ordered.size()>1 && ordered[1].depth-ordered[0].depth<.75f)continue;
    std::vector<bool> owned(sensor.size(),false);
    for(int j:components[ordered[0].index].indices)owned[original[j]]=true;
    int owner=-1;
    for(std::size_t c=0;c<f.clusters.size();++c) {
      const auto& cluster=f.clusters[c].indices;
      const auto count=components[ordered[0].index].indices.size();
      if(cluster.size()<5 || cluster.size()>2048 || count<.9*cluster.size() || count>cluster.size())continue;
      const auto overlap=std::count_if(cluster.begin(),cluster.end(),[&](int i){return owned.at(i);});
      if(static_cast<std::size_t>(overlap)==count) {
        if(owner>=0){owner=-2;break;}owner=c;
      }
    }
    if(owner<0)continue;
    const auto& object=f.objects.at(owner);const auto& flags=object.flags;
    if(flags.wall || flags.ceiling)continue;
    float xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY,zmin=INFINITY,zmax=-INFINITY;
    bool safe=true;
    for(int i:f.clusters[owner].indices) {
      if(!owned[i])continue;
      const auto& p=f.leveled->at(i);
      if(f.labels.at(i)!=tunnel_filter::kKept)safe=false;
      xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);
      zmin=std::min(zmin,p.z);zmax=std::max(zmax,p.z);
      // Both current, confirmed corridor surfaces must cover this point.
      for(const auto& side:f.walls) {
        bool covered=false;
        if(side.removal_allowed)for(const auto& wall:side.segments)
          if(wall.valid && wall.InIntervals(wall.AlongWall(p.x,p.y),0.f) &&
             wall.plane.SignedDistance(p.x,p.y,p.z)>.35f)covered=true;
        if(!covered)safe=false;
      }
    }
    if(!safe || xmax-xmin>3.5f || ymax-ymin>2.5f || zmax-zmin>2.f)continue;
    result[owner].source_class=name;
    result[owner].camera_epoch=camera->camera_epoch;
    for(int i:f.clusters[owner].indices)if(owned[i])result[owner].support.push_back(i);
  }
  // Source resets/disable during geometry work invalidate every candidate.
  if(!camera->still_current())std::fill(result.begin(),result.end(),LowObstacleHint{});
  return result;
}
}
