#include <patchwork/patchworkpp.h>
#include <cassert>
#include <cmath>
#include <iostream>
namespace patchwork {
struct HeightTestAccess {
  static void run(PatchWorkpp& p, std::vector<double> v) {
    p.update_elevation_[0]=v; p.update_elevation_thr();
  }
  static double threshold(const PatchWorkpp& p) { return p.params_.elevation_thr[0]; }
};
}
int main() {
  for(int mode=0;mode<3;++mode) {
    patchwork::Params p;p.sensor_height=1.8;p.height_policy=mode;p.height_trace=true;
    patchwork::PatchWorkpp a(p);
    patchwork::HeightTestAccess::run(a,{});
    assert(a.getHeight()==1.8);
    patchwork::HeightTestAccess::run(a,{-1.18});
    assert(a.getHeight()==(mode==0?0.0:1.8));
    assert(patchwork::HeightTestAccess::threshold(a)==0.0);
    patchwork::HeightTestAccess::run(a,{-1.18,-1.16});
    assert(std::abs(a.getHeight()-(mode==2?1.8:1.17))<1e-12);
    assert(patchwork::HeightTestAccess::threshold(a)<-1.0);
    const auto saved=patchwork::HeightTestAccess::threshold(a);
    patchwork::HeightTestAccess::run(a,{});
    assert(patchwork::HeightTestAccess::threshold(a)==saved);
  }
  std::cout<<"empty, singleton, supported update, fixed-height threshold learning: PASS\n";
}
