#include <asmc_lidar/ground_live.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <unistd.h>

int main(int argc,char** argv) {
  assert(argc==2);
  const auto cal=asmc_lidar::LoadCalibration(argv[1]);
  asmc_lidar::Cloud cloud;
  for(int i=0;i<12000;++i) {
    const float a=float(i%360)*float(M_PI)/180;
    const float r=4.f+float((i*37)%4600)*.01f;
    pcl::PointXYZI p;p.x=r*std::cos(a);p.y=r*std::sin(a);
    p.z=-1.78f+float(i%7)*.01f;
    if(i%23==0)p.z+=.6f;
    p.intensity=float(i%256);cloud.push_back(p);
  }
  asmc_lidar::GroundComparison all;
  const auto first=all.Process(cloud,cal,.021f);
  for(const auto& name:{"jaeho","patchwork","linefit"}) {
    asmc_lidar::GroundComparison single(name);
    const auto r=single.Process(cloud,cal,.021f);
    assert(r.methods.size()==1);
    assert(r.prepared.raw_to_voxel==first.prepared.raw_to_voxel);
    assert(r.methods.at(name).labels==first.methods.at(name).labels);
    for(std::size_t i=0;i<cloud.size();++i)
      assert((r.methods.at(name).labels[i]==0)==(r.prepared.raw_to_voxel[i]<0));
  }
  asmc_lidar::GroundComparison lf("linefit");
  for(int repeat=0;repeat<20;++repeat)
    assert(lf.Process(cloud,cal,.021f).methods.at("linefit").labels==first.methods.at("linefit").labels);
  all.Reset();
  assert(all.Process(cloud,cal,.021f).methods.at("patchwork").labels==first.methods.at("patchwork").labels);
  assert(all.Process({},cal,0).prepared.raw_to_voxel.empty());
  bool failed=false;
  try {all.Process(cloud,cal,std::numeric_limits<float>::quiet_NaN());}
  catch(const std::invalid_argument&) {failed=true;}
  assert(failed);
  const auto profiles=std::filesystem::path(argv[1]).parent_path()/"ground_v2";
  for(const auto& file:std::filesystem::directory_iterator(profiles)) {
    const auto c=asmc_lidar::LoadGroundComparisonConfig(file.path());
    assert(c.sha256.size()==64);
    asmc_lidar::GroundComparison model("all","guard25",c);
    const auto x=model.ProcessPrepared(first.prepared,cloud.size());
    assert(x.methods.at("jaeho").labels==first.methods.at("jaeho").labels);
    for(const auto& name:{"patchwork","linefit"}) {
      const auto& m=x.methods.at(name);assert(m.reasons.size()==cloud.size());
      for(std::size_t i=0;i<cloud.size();++i)if(first.prepared.raw_to_voxel[i]>=0)assert(m.reasons[i]);
    }
    model.Reset();const auto y=model.Process(cloud,cal,.021f);
    for(const auto& m:x.methods){
      assert(m.second.labels==y.methods.at(m.first).labels);
      assert(m.second.reasons==y.methods.at(m.first).reasons);
    }
    if(c.linefit_cover_all)assert(x.methods.at("linefit").events.at("unvisited_sectors")==0);
  }
  const auto file=std::filesystem::temp_directory_path()/("asmc_ground_config_invalid_"+std::to_string(getpid())+".yaml");
  for(const auto& bad:{"id: test\nlinefit_distance: .nan\n", "id: test\nlinefit_threads: 0\n",
      "id: test\nunknown: 1\n", "id: test\nid: duplicate\n", "id: '../unsafe'\n"}) {
    {std::ofstream out(file);out<<bad;}
    bool rejected=false;try{asmc_lidar::LoadGroundComparisonConfig(file.string());}catch(const std::exception&){rejected=true;}
    assert(rejected);
  }
  std::filesystem::remove(file);
  return 0;
}
