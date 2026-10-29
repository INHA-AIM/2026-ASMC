#pragma once
#include <cmath>

namespace shadow_localization {
// Policy time belongs to the observed motion epoch. It never extends the
// physical drivable raster, nor grants an exemption outside a known zone.
class PaintWindow {
 public:
  void reset(){last_=-1;exit_=-1;inside_=false;}
  bool update(double stamp,bool selected,bool pose_valid,bool in_known_zone){
    if(!std::isfinite(stamp)||stamp<=0||!selected||!pose_valid||
       (last_>=0&&stamp<last_)){reset();return false;}
    last_=stamp;
    if(in_known_zone){inside_=true;exit_=-1;return true;}
    if(inside_){inside_=false;exit_=stamp;}
    return exit_>=0&&stamp-exit_<3.;
  }
 private:
  double last_=-1,exit_=-1;
  bool inside_=false;
};
}  // namespace shadow_localization
