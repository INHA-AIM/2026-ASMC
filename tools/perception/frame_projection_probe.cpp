// Research probe of the actual asmc_lidar core; no ROS or external dependency.
#include <asmc_lidar/projection.hpp>
#include <iomanip>
#include <iostream>
using namespace asmc::lidar;
int main() {
  double a[21];
  std::cout << std::setprecision(17);
  while (std::cin >> a[0]) {
    for (int i=1;i<21;++i) if (!(std::cin >> a[i])) return 2;
    const double rad=std::acos(-1.)/180.;
    Pose camera=Pose::Identity();
    camera.translation()=Vec(a[0],a[1],a[2]);
    camera.linear()=rpy(a[3]*rad,a[4]*rad,a[5]*rad);
    Context c; c.calibration_source="coordinate-review";
    c.base_from_sensor.translation()=Vec(a[6],a[7],a[8]);
    c.base_from_sensor.linear()=rpy(a[9]*rad,a[10]*rad,a[11]*rad);
    c.applied_leveling=leveling_rotation(a[12]*rad,a[13]*rad,c.base_from_sensor.linear());
    c.leveling_source="same-scan";
    Eigen::Matrix3d k=Eigen::Matrix3d::Identity();
    k(0,0)=a[17]; k(1,1)=a[18]; k(0,2)=a[19]; k(1,2)=a[20];
    const Vec raw(a[14],a[15],a[16]), leveled=c.applied_leveling*raw;
    const auto project=[&](const Context& ctx,const Vec& p) {
      const auto matrix=camera_projection(ctx,camera,k);
      if (!matrix) throw std::runtime_error("invalid projection context");
      const Vec h=*matrix*p.homogeneous();
      std::cout << h.x() << ' ' << h.y() << ' ' << h.z() << ' ';
    };
    project(c,raw);
    c.frame=Frame::LeveledSensor; project(c,leveled);
    c.frame=Frame::Sensor; project(c,leveled); // negative control: missing inverse
    c.frame=Frame::LeveledSensor; project(c,raw); // negative control: extra inverse
    c.applied_leveling=leveling_rotation((a[12]+5)*rad,(a[13]-4)*rad,c.base_from_sensor.linear());
    project(c,leveled); // negative control: attitude from another observation
    std::cout << '\n';
  }
}
