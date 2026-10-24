// ROI/ego-column removal and voxel grid.
// Ported from Main src/Lidar/RoiVoxel/RoiVoxel.cpp at 8f93c22e. Roi and Voxel
// have no legacy/common branch in Main; both run in the raw sensor frame.

#include <asmc_lidar/preprocessing/preprocessing.hpp>

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <cstdint>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

// Same value as glibc M_PI (Main uses M_PI); spelled out so strict -std=c++17 builds too.
constexpr double kMainPi = 3.14159265358979323846;

// Ported from Main src/Global/Global.cpp:10-13 (8f93c22e)
// Float in, double multiply/divide (M_PI is double), float out.
float32 MainDeg2Rad(const float32& f32_Deg) { return f32_Deg * kMainPi / 180.0f; }

}  // namespace

RoiClassifier::RoiClassifier(const RangeConfig& range,const RoiConfig& roi,const VehicleSpec& vehicle)
    : range_(range),roi_(roi),xmin_(-vehicle.f32_RearOverhang),
      xmax_(vehicle.f32_WheelBase+vehicle.f32_FrontOverhang),
      ymin_(-.5f*vehicle.f32_Width),ymax_(.5f*vehicle.f32_Width) {
  const float roll = MainDeg2Rad(vehicle.st_LiDARRotationDeg.f32_X);
  const float pitch = MainDeg2Rad(vehicle.st_LiDARRotationDeg.f32_Y);
  const float yaw = MainDeg2Rad(vehicle.st_LiDARRotationDeg.f32_Z);
  rotation_ =
      (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
       Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
          .toRotationMatrix();
  translation_=Eigen::Vector3f(vehicle.st_LiDARPos.f32_X,vehicle.st_LiDARPos.f32_Y,vehicle.st_LiDARPos.f32_Z);
}
RoiReason RoiClassifier::operator()(const pcl::PointXYZI& p) const {
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return RoiReason::NonFinite;
  const float r=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
  if(!(r>=range_.f32_Min&&r<=range_.f32_Max))return RoiReason::Range;
  if(!(p.x>=roi_.f32_MinX&&p.x<=roi_.f32_MaxX&&p.y>=roi_.f32_MinY&&p.y<=roi_.f32_MaxY))return RoiReason::XY;
  if(!(p.z>=roi_.f32_MinZ&&p.z<=roi_.f32_MaxZ))return RoiReason::SensorZ;
  const Eigen::Vector3f b=rotation_*Eigen::Vector3f(p.x,p.y,p.z)+translation_;
  if(b.x()>=xmin_&&b.x()<=xmax_&&b.y()>=ymin_&&b.y()<=ymax_)return RoiReason::Ego;
  return RoiReason::Keep;
}

// Ported bounds/order, shared predicate for production and debug provenance.
void Roi(const CloudPtr& input,const CloudPtr& output,const RangeConfig& range,
         const RoiConfig& roi,const VehicleSpec& vehicle) {
  const RoiClassifier classify(range,roi,vehicle);

  // Cleared first so a reused output never accumulates the previous scan.
  output->clear();
  output->points.reserve(input->points.size());

  for (const auto& pcl_Point : input->points) {
    if(classify(pcl_Point)==RoiReason::Keep)output->points.push_back(pcl_Point);
  }
  output->width = static_cast<uint32_t>(output->points.size());
  output->height = 1;
  output->is_dense = false;
}

void HeightRoi(const CloudPtr& input,const CloudPtr& output,double sensor_height,
               const HeightRoiConfig& cfg,std::vector<int>& kept) {
  output->clear();kept.clear();output->reserve(input->size());kept.reserve(input->size());
  for(std::size_t i=0;i<input->size();++i) {
    const auto& p=input->points[i];
    if(!cfg.enabled || (std::isfinite(p.z)&&p.z+sensor_height<=cfg.max_height_above_ground_m)) {
      output->push_back(p);kept.push_back(static_cast<int>(i));
    }
  }
}

// Ported from Main src/Lidar/RoiVoxel/RoiVoxel.cpp:65-70 (8f93c22e)
// Main hard-codes 0.1f on all three axes; leaf_m defaults to that value.
void Voxel(const CloudPtr& input, const CloudPtr& output, float leaf_m) {
  pcl::VoxelGrid<pcl::PointXYZI> pcl_Sor;
  pcl_Sor.setInputCloud(input);
  pcl_Sor.setLeafSize(leaf_m, leaf_m, leaf_m);
  pcl_Sor.filter(*output);
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
