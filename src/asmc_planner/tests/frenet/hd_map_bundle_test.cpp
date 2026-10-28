#include "Frenet/Map/AvoidRoadMap.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
int main() {
    namespace fs=std::filesystem;int failures=0;
    auto expect=[&](bool ok,const char* label){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<label<<'\n';failures+=!ok;};
    const auto source=fs::path(FRENET_PACKAGE_SRC_DIR).parent_path()/"config/frenet/hd_map";
    auto temporary=fs::temp_directory_path()/("hd_map_bundle_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(temporary);
    for(const auto* name:{"drivable.json","lines.json"})fs::copy_file(source/name,temporary/name);
    std::ifstream input(source/"drivable_grid.rle");std::string header;std::getline(input,header);
    const std::string rows((std::istreambuf_iterator<char>(input)),{});
    auto load=[&](const std::string& metadata){std::ofstream out(temporary/"drivable_grid.rle");out<<metadata<<rows;out.close();AvoidRoadMap road;std::string error;return road.Load((temporary/"drivable.json").string(),(temporary/"lines.json").string(),error);};
    expect(load(header+"\n"),"the exact v6 vector/raster bundle loads through the reader shared by every mode");
    expect(!load(""),"a same-sized legacy raster cannot be paired silently with v6 polygons");
    auto corrupt=header;const auto token=corrupt.find("/v1 ");if(token!=std::string::npos)corrupt[token+4]=corrupt[token+4]=='0'?'1':'0';
    expect(!load(corrupt+"\n"),"a different vector product identity refuses initialization");
    corrupt=header;corrupt.back()=corrupt.back()=='0'?'1':'0';
    expect(!load(corrupt+"\n"),"a different official NPZ identity refuses initialization");
    fs::remove_all(temporary);return failures?1:0;
}
