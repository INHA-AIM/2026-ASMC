#include <asmc_lidar/ground_live.hpp>
#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <set>
#include <stdexcept>

namespace asmc_lidar {
GroundComparisonConfig LoadGroundComparisonConfig(const std::string& path) {
  GroundComparisonConfig c;std::string text="id: v1\n";
  if(!path.empty()) {
    std::ifstream in(path,std::ios::binary);if(!in)throw std::runtime_error("missing comparison_config: "+path);
    text.assign(std::istreambuf_iterator<char>(in),{});
  }
  const auto y=YAML::Load(text);
  if(!y.IsMap()||!y["id"])throw std::invalid_argument("comparison_config must be map with id");
  const std::set<std::string> allowed={"id","leveling_mode","linefit_cover_all","linefit_threads","patchwork_min_points",
    "patchwork_distance","patchwork_seeds","linefit_distance","linefit_search","linefit_long"};
  std::set<std::string> seen;
  for(const auto& kv:y) {
    auto k=kv.first.as<std::string>();
    if(!allowed.count(k)||!seen.insert(k).second)throw std::invalid_argument("unknown/duplicate config key: "+k);
  }
  c.id=y["id"].as<std::string>();
  if(c.id.empty()||c.id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
    throw std::invalid_argument("unsafe config id");
  if(y["leveling_mode"])c.leveling_mode=y["leveling_mode"].as<std::string>();
  if(c.leveling_mode!="pitch_only"&&c.leveling_mode!="roll_pitch")
    throw std::invalid_argument("leveling_mode must be pitch_only|roll_pitch");
#define LOAD(field,type) if(y[#field])c.field=y[#field].as<type>()
  LOAD(linefit_cover_all,bool);LOAD(linefit_threads,int);LOAD(patchwork_min_points,int);
  LOAD(patchwork_distance,double);LOAD(patchwork_seeds,double);
  LOAD(linefit_distance,double);LOAD(linefit_search,double);LOAD(linefit_long,double);
#undef LOAD
  if(c.linefit_threads<1||c.linefit_threads>16||c.patchwork_min_points<3||c.patchwork_min_points>100)
    throw std::invalid_argument("invalid thread/minimum point setting");
  for(double v:{c.patchwork_distance,c.patchwork_seeds,c.linefit_distance,c.linefit_search,c.linefit_long})
    if(!std::isfinite(v)||v<=0||v>10)throw std::invalid_argument("invalid comparison parameter");
  unsigned char hash[SHA256_DIGEST_LENGTH];SHA256(reinterpret_cast<const unsigned char*>(text.data()),text.size(),hash);
  std::ostringstream out;for(auto b:hash)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);
  c.sha256=out.str();return c;
}
}
