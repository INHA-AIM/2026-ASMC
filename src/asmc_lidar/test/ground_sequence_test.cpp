// Repeat all frozen profiles on one prepared observation; each has independent state.
#include <asmc_lidar/ground_live.hpp>
#include <asmc_lidar/review_io.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
namespace fs=std::filesystem;
int main(int argc,char** argv)try {
  if(argc!=4)throw std::runtime_error("usage: ground_sequence_test snapshots calibration profiles_dir");
  const fs::path source=argv[1];const auto cal=asmc_lidar::LoadCalibration(argv[2]);
  std::vector<std::unique_ptr<asmc_lidar::GroundComparison>> first,second;
  std::vector<fs::path> profiles;for(auto& f:fs::directory_iterator(argv[3]))if(f.path().extension()==".yaml")profiles.push_back(f.path());
  std::sort(profiles.begin(),profiles.end());
  for(auto& p:profiles){auto c=asmc_lidar::LoadGroundComparisonConfig(p);
    const std::string method=c.id=="v1"?"all":c.id.rfind("pp_",0)==0?"patchwork":"linefit";
    first.emplace_back(new asmc_lidar::GroundComparison(method,"guard25",c));
    second.emplace_back(new asmc_lidar::GroundComparison(method,"guard25",c));}
  std::ifstream in(source);if(!in)throw std::runtime_error("missing input");std::string line,phase;std::size_t frames=0;std::int64_t last=0;
  while(std::getline(in,line)){
    auto row=YAML::Load(line);auto current=row["phase"].as<std::string>();
    if(current!=phase){for(auto& m:first)m->Reset();for(auto& m:second)m->Reset();phase=current;last=0;}
    const auto stamp=row["scan_effective_stamp_ns"].as<std::int64_t>();
    if(stamp<=last||!row["geometry_valid"].as<bool>())throw std::runtime_error("invalid observation");last=stamp;
    const auto cloud=asmc_lidar::ReadReviewCloud(source.parent_path()/row["cloud_path"].as<std::string>());
    auto prepared=asmc_lidar::PrepareGeneralGround(cloud,cal,row["pitch"].as<float>());
    for(std::size_t i=0;i<first.size();++i){
      auto a=first[i]->ProcessPrepared(prepared,cloud.size()),b=second[i]->ProcessPrepared(prepared,cloud.size());
      for(const auto& m:a.methods){const auto& n=b.methods.at(m.first);
        if(m.second.labels!=n.labels||m.second.structural_removed!=n.structural_removed||m.second.pre_guard_labels!=n.pre_guard_labels||m.second.reasons!=n.reasons||m.second.events!=n.events)
          throw std::runtime_error("nondeterministic: "+row["input_id"].as<std::string>()+" "+profiles[i].stem().string());
      }
    }
    if(++frames%250==0)std::cerr<<"verified "<<frames<<'\n';
  }
  if(!frames||profiles.size()!=14)throw std::runtime_error("empty/incomplete verification");
  std::cout<<"{\"frames\":"<<frames<<",\"profiles\":"<<profiles.size()<<",\"repeat_geometry_and_diagnostics_exact\":true}\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
