#include <asmc_lidar/core.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace asmc_lidar;
int main() {
  Calibration calibration;
  calibration.source="synthetic";
  calibration.base_from_sensor.translation()=Eigen::Vector3d(.8,0,1.45);
  Cloud::Ptr cloud(new Cloud);
  for(int x=0;x<30;++x)for(int y=-10;y<=10;++y) {
    pcl::PointXYZI p;p.x=1.0f+.25f*x;p.y=.2f*y;p.z=-1.8f;p.intensity=float((x+y+50)%20)/20;
    cloud->push_back(p);
  }
  for(int x=0;x<20;++x) {
    pcl::PointXYZI p;p.x=6+.03f*x;p.y=1;p.z=-.5f+.02f*x;p.intensity=.7;
    cloud->push_back(p);
  }
  FrameInput input;input.cloud=cloud;input.stamp_ns=1000000000;input.receipt_ns=1000000001;
  input.imu_valid=true;
  Processor processor(Mode::GroundReview,calibration,tunnel_filter::Params());
  auto result=processor.Process(input);
  if(result.status!="valid") {std::cerr<<result.status<<'\n';return 1;}
  assert(result.stamp_ns==input.stamp_ns);
  assert(result.raw_points==cloud->size());
  assert(result.patchwork.by_original.size()==cloud->size());
  assert(result.patchwork.ground->size()+result.patchwork.nonground->size()+
         result.patchwork.uncertain->size()==result.roi_points);
  assert(result.patchwork.reason=="indexed_complete");
  assert(std::abs(result.input_base->at(0).x-cloud->at(0).x-.8)<1e-5);
  assert(std::abs(result.input_base->at(0).z-cloud->at(0).z-1.45)<1e-5);
  assert(processor.Process(input).status=="duplicate_stamp");
  processor.Reset();
  assert(processor.Process(input).patchwork.by_original==result.patchwork.by_original);
  input.stamp_ns=0;input.receipt_ns=1;
  assert(processor.Process(input).status=="zero_or_negative_stamp");
  input.stamp_ns=1000000002;input.receipt_ns=1000000003;input.imu_valid=false;
  assert(processor.Process(input).status=="imu_missing_or_invalid");
  // The L-shape output is frame-local: it does not contain a tracker velocity.
  pcl::PointIndices cluster;
  for(std::size_t i=0;i<20;++i)cluster.indices.push_back(int(cloud->size()-20+i));
  std::vector<tunnel_filter::Obb> obbs;
  PinnedLShapeObb()(cloud,{cluster},&obbs);
  assert(obbs.size()==1&&std::isfinite(obbs[0].cx));
  std::cout<<"asmc_lidar_core_test passed\n";
}
