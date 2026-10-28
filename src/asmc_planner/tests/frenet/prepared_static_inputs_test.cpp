#include "Frenet/frenet_planner.hpp"
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace {
int failures=0;
void expect(bool value,const char* message) {
    if(!value){++failures;std::cerr<<"FAIL "<<message<<'\n';}
}
void write(const std::string& path,const std::string& data) {
    std::ofstream out(path);out<<data;
    if(!out)throw std::runtime_error("write fixture failed");
}
std::string read(const std::string& path) {
    std::ifstream in(path);if(!in)throw std::runtime_error("read fixture failed");
    return {std::istreambuf_iterator<char>(in),{}};
}
struct Fixture {
    std::string root;
    Fixture() {
        char pattern[]="/tmp/asmc-prepared-inputs-XXXXXX";
        const char* result=mkdtemp(pattern);if(!result)throw std::runtime_error("mkdtemp failed");
        root=result;
    }
    ~Fixture() {
        for(const auto* name:{"params.yaml","other.yaml","drivable.json","lines.json","drivable_grid.rle","route.txt"})
            unlink((root+"/"+name).c_str());
        rmdir(root.c_str());
    }
};
}
int main() {
    Fixture fixture;
    auto config=YAML::LoadFile(FRENET_TEST_CONFIG);
    const std::string package=FRENET_TEST_PACKAGE;
    const auto original_route=package+"/src/"+config["waypoint_file"].as<std::string>();
    const auto route=read(original_route);
    write(fixture.root+"/route.txt",route);
    for(const auto* file:{"drivable.json","lines.json","drivable_grid.rle"})
        write(fixture.root+"/"+file,read(package+"/config/frenet/hd_map/"+file));
    config["waypoint_file"]=fixture.root+"/route.txt";
    config["avoid_hd_map"]["directory"]=fixture.root;
    const auto path=fixture.root+"/params.yaml";
    const auto bytes=YAML::Dump(config);
    write(path,bytes);
    FrenetPlanner source, fresh;
    expect(source.Init(path),"load real validated map/reference");
    asmc_planner::execution::ManeuverState state;
    source.BindExecutionState(state);
    state.follow={true,777,0};
    expect(fresh.InitFromPrepared(path,source),"prepare a fresh core");
    expect(!fresh.executionSnapshot().follow.active,"do not copy source execution state");
    expect(state.follow.active&&state.follow.leader_id==777,"do not reset source bound state");
    const auto& a=source.reference();const auto& b=fresh.reference();
    expect(a.length==b.length&&a.closed_loop==b.closed_loop&&a.points.size()==b.points.size(),"same complete reference shape");
    for(std::size_t i=0;i<a.points.size()&&i<b.points.size();++i) {
        const auto& x=a.points[i];const auto& y=b.points[i];
        expect(x.x==y.x&&x.y==y.y&&x.theta==y.theta&&x.s==y.s&&x.kappa==y.kappa&&x.d_kappa==y.d_kappa,"same reference samples");
        if(i%50==0) expect(source.road_map().FootprintInside(x.x,x.y,x.theta,source.vehicle_shape())==fresh.road_map().FootprintInside(x.x,x.y,x.theta,fresh.vehicle_shape()),"same footprint queries");
    }
    expect(source.roundabout_zone().stop_s==fresh.roundabout_zone().stop_s,"same roundabout stop station");
    const double original_x=source.reference().points.front().x;
    RefLine replacement=source.reference();replacement.closed_loop=false;
    for(auto& point:replacement.points) point.x+=100;
    expect(source.SetPreparedReference(replacement),"source adopts a different local reference");
    expect(!source.roundabout_merge_enabled(),"local reference disables source roundabout");
    FrenetPlanner another;
    expect(another.InitFromPrepared(path,source),"reuse immutable inputs after source reference replacement");
    expect(another.reference().points.front().x==original_x,"source reference changes cannot mutate prepared snapshot");
    expect(another.roundabout_merge_enabled()==fresh.roundabout_merge_enabled(),"source runtime zone reset does not alter prepared zone");
    expect(!fresh.InitFromPrepared(path,source),"reuse API rejects already initialized target");
    // Each negative test keeps the original source alive. Cache hits must never
    // hide missing/corrupt data, even when the path and byte length stay fixed.
    write(path,"invalid: [");
    FrenetPlanner bad_config;expect(!bad_config.InitFromPrepared(path,source),"changed YAML is revalidated");
    write(path,bytes);
    for(const auto* file:{"drivable.json","lines.json","drivable_grid.rle","route.txt"}) {
        const auto full=fixture.root+"/"+file;const auto before=read(full);
        write(full,std::string(before.size(),'!'));
        FrenetPlanner bad;expect(!bad.InitFromPrepared(path,source),"same-size corrupted static input invalidates snapshot");
        write(full,before);
    }
    unlink((fixture.root+"/route.txt").c_str());
    FrenetPlanner missing;expect(!missing.InitFromPrepared(path,source),"missing route cannot be hidden by prepared inputs");
    write(fixture.root+"/route.txt",route);
    // Different valid config locations may use the ordinary load path.
    write(fixture.root+"/other.yaml",bytes);
    FrenetPlanner other;expect(other.InitFromPrepared(fixture.root+"/other.yaml",source),"different config path uses independent validation");
    std::cout<<"prepared_static_inputs_test failures="<<failures<<'\n';
    return failures?1:0;
}
