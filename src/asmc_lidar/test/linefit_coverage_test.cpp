#include <ground_segmentation/ground_segmentation.h>
#include <cassert>
#include <cmath>
#include <algorithm>

int main() {
  for (int sectors : {3, 89, 90, 91}) {
    PointCloud cloud;
    for(int sector=0; sector<sectors; ++sector) for(int bin=0; bin<45; ++bin) {
      double a=-M_PI+(sector+.5)*2*M_PI/sectors, r=2.+bin*.4;
      cloud.emplace_back(r*std::cos(a), r*std::sin(a), -1.78+(bin%13==0?.3:0));
    }
    cloud.emplace_back(2, 0, -1.78);
    std::vector<int> reference;
    for(int threads : {1, 2, 4}) {
      GroundSegmentationParams p;p.n_segments=sectors;p.n_threads=threads;
      p.n_bins=60;p.sensor_height=1.78;p.cover_all_work=true;
      GroundSegmentation model(p);std::vector<int> labels;model.segment(cloud,&labels);
      assert(std::all_of(model.fit_visits.begin(),model.fit_visits.end(),[](unsigned n){return n==1;}));
      assert(std::all_of(model.assignment_visits.begin(),model.assignment_visits.end(),[](unsigned n){return n==1;}));
      if(threads==1)reference=labels;else assert(labels==reference);
      for(std::size_t size : {0u,1u,2u,3u,5u}) {
        PointCloud tiny; for(std::size_t i=0;i<size;++i)tiny.push_back(cloud[i]);
        model.segment(tiny,&labels);assert(labels.size()==size);
        assert(std::all_of(model.assignment_visits.begin(),model.assignment_visits.end(),[](unsigned n){return n==1;}));
      }
    }
  }
  GroundSegmentationParams old;old.n_segments=90;old.n_threads=4;
  GroundSegmentation legacy(old);PointCloud cloud;cloud.resize(7);std::vector<int> labels;
  legacy.segment(cloud,&labels);
  assert(legacy.fit_visits[88]==0 && legacy.fit_visits[89]==0);
  assert(legacy.assignment_visits[6]==0); // V1 can still be reproduced, not silently replaced.
}
