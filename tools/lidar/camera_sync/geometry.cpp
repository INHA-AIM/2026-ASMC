// Isolated evaluator: no subscribers, publishers, parameter server or control.
#include "reference_generated.hpp"
#include <Lidar/RoiVoxel/RoiVoxel.hpp>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <map>

VehicleSpec st_VehicleSpec{};
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;
double Ms(Clock::time_point t) {
    return std::chrono::duration<double,std::milli>(Clock::now()-t).count();
}
std::string Hash(const void* memory, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(memory);
    std::uint64_t h=14695981039346656037ULL;
    for(std::size_t i=0;i<n;++i) { h ^= b[i]; h *= 1099511628211ULL; }
    std::ostringstream s; s << std::hex << std::setw(16) << std::setfill('0') << h;
    return s.str();
}
std::string CloudHash(const Cloud& c) {
    std::vector<float> packed;
    for(const auto& p:c) packed.insert(packed.end(), {p.x,p.y,p.z,p.intensity});
    return Hash(packed.data(),packed.size()*sizeof(float));
}
Cloud::Ptr ReadCloud(const fs::path& path) {
    if(fs::file_size(path)%16) throw std::runtime_error("bad float4 point file");
    std::ifstream in(path,std::ios::binary);
    Cloud::Ptr c(new Cloud);
    float v[4];
    while(in.read(reinterpret_cast<char*>(v),sizeof(v))) {
        pcl::PointXYZI p; p.x=v[0];p.y=v[1];p.z=v[2];p.intensity=v[3];c->push_back(p);
    }
    c->width=c->size();c->height=1;c->is_dense=false;
    return c;
}
void WriteCloud(const Cloud& c,const fs::path& path) {
    std::ofstream out(path,std::ios::binary);
    for(const auto& p:c) {float v[]={p.x,p.y,p.z,p.intensity};out.write(reinterpret_cast<char*>(v),sizeof(v));}
}
template<class T> YAML::Node Sequence(const std::vector<T>& v) {
    YAML::Node n(YAML::NodeType::Sequence);for(const auto& x:v)n.push_back(x);return n;
}
void JsonString(std::ostream& out,const std::string& s) {
    out<<'"';for(unsigned char c:s) {
        if(c=='"'||c=='\\')out<<'\\'<<c;
        else if(c<32) {const auto flags=out.flags();out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c);out.flags(flags);}
        else out<<c;
    }out<<'"';
}
void Json(std::ostream& out,const YAML::Node& n,const std::string& key="") {
    if(!n||n.IsNull()) {out<<"null";return;}
    if(n.IsMap()) {
        out<<'{';bool first=true;
        for(auto pair:n) {if(!first)out<<',';first=false;
            const auto k=pair.first.as<std::string>();JsonString(out,k);out<<':';Json(out,pair.second,k);}
        out<<'}';return;
    }
    if(n.IsSequence()) {
        out<<'[';bool first=true;for(auto item:n) {if(!first)out<<',';first=false;Json(out,item,key);}out<<']';return;
    }
    const auto value=n.Scalar();
    const bool text=key=="input_id"||key=="scene"||key=="phase"||key=="voxel_path"||
                    key=="skipped_reason"||key.find("fnv1a64")!=std::string::npos;
    if(text) {JsonString(out,value);return;}
    if(value=="true"||value=="false") {out<<value;return;}
    char* end=nullptr;const double number=std::strtod(value.c_str(),&end);
    if(end!=value.c_str()+value.size()||!std::isfinite(number))
        throw std::runtime_error("invalid numeric JSON field: "+key);
    // Emit the original decimal lexeme; int64 nanoseconds never pass through a double.
    out<<value;
}

int main(int argc,char** argv) try {
    if(argc!=6) throw std::runtime_error("usage: tunnel_geometry_eval INPUT_JSONL SETTING PARAMS OUTPUT_YAML OUTPUT_CLOUD_DIR");
    const fs::path input=argv[1], cloud_dir=argv[5];
    if(fs::exists(argv[4])||fs::exists(cloud_dir))throw std::runtime_error("output already exists");
    fs::create_directories(cloud_dir);
    const auto setting=YAML::LoadFile(argv[2]); resolved_params=YAML::LoadFile(argv[3]);
    const auto v=setting["Vehicle"], l=setting["LiDAR"]["Filter"];
    st_VehicleSpec.f32_Width=v["Width"].as<float>();
    st_VehicleSpec.f32_WheelBase=v["WheelBase"].as<float>();
    st_VehicleSpec.f32_FrontOverhang=v["FrontOverhang"].as<float>();
    st_VehicleSpec.f32_RearOverhang=v["RearOverhang"].as<float>();
    st_VehicleSpec.st_LiDARPos={v["LiDAR"]["x"].as<float>(),v["LiDAR"]["y"].as<float>(),v["LiDAR"]["z"].as<float>()};
    st_VehicleSpec.st_LiDARRotationDeg={v["LiDAR"]["roll"].as<float>(),v["LiDAR"]["pitch"].as<float>(),v["LiDAR"]["yaw"].as<float>()};
    LiDAR lidar{};
    lidar.st_Range.f32_Min=l["Range"]["Min"].as<float>();lidar.st_Range.f32_Max=l["Range"]["Max"].as<float>();
    lidar.st_ROI.f32_MinX=l["ROI"]["MinX"].as<float>();lidar.st_ROI.f32_MaxX=l["ROI"]["MaxX"].as<float>();
    lidar.st_ROI.f32_MinY=l["ROI"]["MinY"].as<float>();lidar.st_ROI.f32_MaxY=l["ROI"]["MaxY"].as<float>();
    lidar.st_ROI.f32_MinZ=l["ROI"]["MinZ"].as<float>();lidar.st_ROI.f32_MaxZ=l["ROI"]["MaxZ"].as<float>();
    OfflineParam("roi_min_x",lidar.st_ROI.f32_MinX,lidar.st_ROI.f32_MinX);
    OfflineParam("roi_max_x",lidar.st_ROI.f32_MaxX,lidar.st_ROI.f32_MaxX);
    OfflineParam("roi_min_y",lidar.st_ROI.f32_MinY,lidar.st_ROI.f32_MinY);
    OfflineParam("roi_max_y",lidar.st_ROI.f32_MaxY,lidar.st_ROI.f32_MaxY);
    pinned_camera::OfflineNode node;
    const auto params=pinned_camera::LoadParams(node,tunnel_filter::Params());
    CAMERA camera{};
    camera.params.world_points_vector=setting["Camera"]["BEV"]["WorldPoints"].as<std::vector<float>>();
    camera.params.moving_average_filter_size=setting["Camera"]["BEV"]["MAF"].as<int>();
    InitProjectionMatrix(camera,setting);
    pinned_camera::ExperimentState protection;
    protection.projection=camera.params.sensor_fusion_projection_matrix;
    // Apply the exact frozen protection parameters, not tunable sweep values.
    node.param("tunnel_protect_enable",protection.protection.enable,protection.protection.enable);
    node.param("tunnel_protect_max_dt_s",protection.protection.max_dt_s,protection.protection.max_dt_s);
    pinned_camera::ParamFloat(node,"tunnel_protect_box_margin_ratio",&protection.protection.box_margin_ratio);
    pinned_camera::ParamFloat(node,"tunnel_protect_box_margin_px",&protection.protection.box_margin_px);
    pinned_camera::ParamFloat(node,"tunnel_protect_depth_vehicle_m",&protection.protection.depth_vehicle_m);
    pinned_camera::ParamFloat(node,"tunnel_protect_depth_person_m",&protection.protection.depth_person_m);
    pinned_camera::ParamFloat(node,"tunnel_protect_depth_front_margin_m",&protection.protection.depth_front_margin_m);
    pinned_camera::ParamFloat(node,"tunnel_protect_depth_ref_percentile",&protection.protection.depth_ref_percentile);
    node.param("tunnel_protect_min_box_points",protection.protection.min_box_points,protection.protection.min_box_points);
    pinned_camera::ParamFloat(node,"tunnel_protect_ambiguous_max_range_m",&protection.protection.ambiguous_max_range_m);
    std::map<std::string,std::unique_ptr<tunnel_filter::TunnelPipeline>> pipelines;
    for(const auto& mode:{"baseline","candidate","runtime_off","old_offline_no_protection"})
        pipelines[mode]=std::make_unique<tunnel_filter::TunnelPipeline>(params,pinned_camera::MakeTrackingObbFunction());
    std::string phase,line;bool anchor=false;float previous_yaw=0,previous_speed=0;std::int64_t previous_stamp=0;
    std::ifstream in(input);std::ofstream out(argv[4]);
    while(std::getline(in,line)) {
        const auto row=YAML::Load(line);YAML::Node result;
        result["input_id"]=row["input_id"].as<std::string>();result["scene"]=row["scene"].as<std::string>();
        result["measurement"]=row["measurement"].as<bool>();
        const auto current_phase=row["phase"].as<std::string>();
        if(current_phase!=phase) {for(auto& p:pipelines)p.second->Reset();anchor=false;phase=current_phase;}
        result["phase"]=phase;
        if(!row["geometry_valid"].as<bool>()) {
            result["skipped_reason"]=row["geometry_reject_reason"].as<std::string>();
            anchor=false;for(auto& p:pipelines)p.second->Reset();
            Json(out,result);out<<'\n';continue;
        }
        const auto frame_start=Clock::now();
        auto raw=ReadCloud(input.parent_path()/row["cloud_path"].as<std::string>());
        Cloud::Ptr filtered(new Cloud),voxel(new Cloud);
        if(row["already_preprocessed"].as<bool>(false)) {*voxel=*raw;}
        else {
            Roi(raw,filtered,lidar,st_VehicleSpec);
            if(params.dynamic_roi_enable) {
                auto& points=filtered->points;
                points.erase(std::remove_if(points.begin(),points.end(),[&](const auto& p) {
                    return p.x<params.roi_min_x_m||p.x>params.roi_max_x_m||p.y<params.roi_min_y_m||
                        p.y>params.roi_max_y_m||p.z<params.roi_min_z_m||p.z>params.roi_max_z_m;
                }),points.end());
                filtered->width=points.size();filtered->height=1;
            }
            Voxel(filtered,voxel);
        }
        result["preprocessing_ms"]=Ms(frame_start);result["raw_points"]=raw->size();
        result["roi_points"]=filtered->size();result["voxel_points"]=voxel->size();
        const auto voxel_hash=CloudHash(*voxel);result["frozen_voxel_fnv1a64"]=voxel_hash;
        const std::string file=result["input_id"].as<std::string>()+".bin";
        WriteCloud(*voxel,cloud_dir/file);result["voxel_path"]=file;
        const float roll=row["roll"].as<float>(),pitch=row["pitch"].as<float>(),yaw=row["yaw"].as<float>(),speed=row["speed_mps"].as<float>();
        const auto stamp=row["scan_effective_stamp_ns"].as<std::int64_t>();
        tunnel_filter::EgoMotion motion;
        if(anchor) {
            const float dt=(stamp-previous_stamp)/1e9;
            const float dyaw=std::atan2(std::sin(yaw-previous_yaw),std::cos(yaw-previous_yaw));
            const float distance=.5f*(speed+previous_speed)*dt;
            if(dt>0&&std::isfinite(dyaw)&&std::isfinite(distance)) {
                motion.valid=true;motion.dt_s=dt;motion.dyaw_rad=dyaw;
                motion.dx_m=distance*std::cos(.5f*dyaw);motion.dy_m=distance*std::sin(.5f*dyaw);
            }
        }
        anchor=true;previous_stamp=stamp;previous_yaw=yaw;previous_speed=speed;
        result["scan_stamp_ns"]=row["scan_stamp_ns"].as<std::int64_t>();result["scan_effective_stamp_ns"]=stamp;
        result["pose_evidence"]=row["pose_evidence"];
        for(auto& mode:pipelines) {
            const auto start=Clock::now();
            const bool normal=mode.first=="baseline"||mode.first=="candidate";
            const auto decision=row[normal ? mode.first : "baseline"];
            const bool available=normal&&decision["selected_yolo_index"]&&!decision["selected_yolo_index"].IsNull();
            pinned_camera::YoloFrame yolo;
            if(available) for(const auto& b:decision["boxes"]) {
                BBox box;box.st_Min_pt.f32_X=b["xmin"].as<float>();box.st_Min_pt.f32_Y=b["ymin"].as<float>();
                box.st_Max_pt.f32_X=b["xmax"].as<float>();box.st_Max_pt.f32_Y=b["ymax"].as<float>();
                box.str_class=b["class"].as<std::string>();box.str_raw_class=box.str_class;yolo.boxes.push_back(box);
            }
            std::vector<std::uint8_t> protect,ambiguous;int protected_boxes=0;
            pinned_camera::ComputeProtection(protection,*voxel,available?&yolo:nullptr,&protect,&ambiguous,&protected_boxes);
            const double protection_ms=Ms(start);
            const bool old_off=mode.first=="old_offline_no_protection";
            const auto& r=mode.second->Process(*voxel,roll,pitch,&protect,
                old_off?nullptr:&ambiguous,old_off?true:available,&motion);
            YAML::Node m;
            m["protection_available"]=old_off?true:available;m["protection_boxes"]=protected_boxes;
            m["protected_points"]=r.protected_points;m["ambiguous_points"]=std::count(ambiguous.begin(),ambiguous.end(),1);
            m["protected_indices"]=YAML::Node(YAML::NodeType::Sequence);
            for(std::size_t i=0;i<protect.size();++i)if(protect[i]!=tunnel_filter::kProtectNone)
                m["protected_indices"].push_back(static_cast<int>(i));
            m["ground_removed"]=r.ground_removed;m["wall_removed"]=r.wall_removed;m["ceiling_removed"]=r.ceiling_removed;
            m["kept_points"]=r.kept_points;m["clusters"]=r.clusters.size();
            m["final_objects"]=std::count_if(r.objects.begin(),r.objects.end(),[](const auto& o){return !o.flags.removed;});
            m["objects_removed_wall"]=r.objects_removed_wall;m["ground_source"]=static_cast<int>(r.ground.source);
            m["wall_left_removal"]=r.walls[0].removal_allowed;m["wall_right_removal"]=r.walls[1].removal_allowed;
            m["wall_ambiguous_skipped"]=r.walls[0].ambiguous_skipped+r.walls[1].ambiguous_skipped;
            std::vector<int> labels(r.labels.begin(),r.labels.end());m["labels"]=Sequence(labels);m["kept_indices"]=Sequence(r.kept_indices);
            m["leveling"]=YAML::Node(YAML::NodeType::Sequence);for(float v:r.leveling)m["leveling"].push_back(v);
            m["leveled_xyzi_fnv1a64"]=CloudHash(*r.leveled);
            m["intensity_preservation_errors"]=0;
            int intensity_errors=0;for(std::size_t i=0;i<voxel->size();++i)
                if(std::memcmp(&voxel->points[i].intensity,&r.leveled->points[i].intensity,sizeof(float)))++intensity_errors;
            m["intensity_preservation_errors"]=intensity_errors;
            m["point_count_preserved"]=voxel->size()==r.leveled->size();
            m["objects"]=YAML::Node(YAML::NodeType::Sequence);
            for(const auto& o:r.objects) {
                YAML::Node a;a["center"]=Sequence(std::vector<float>{o.obb.cx,o.obb.cy,o.obb.cz});
                a["dimensions"]=Sequence(std::vector<float>{o.obb.length,o.obb.width,o.obb.z_max-o.obb.z_min});
                a["yaw"]=o.obb.yaw;a["removed"]=o.flags.removed;a["protected"]=o.is_protected;
                a["point_count"]=o.point_count;a["protect_class"]=int(o.protect_class);
                a["corners"]=YAML::Node(YAML::NodeType::Sequence);
                for(const auto& c:o.obb.corners)a["corners"].push_back(Sequence(std::vector<float>{c.x,c.y,c.z}));
                m["objects"].push_back(a);
            }
            m["timing_ms"]["protection"]=protection_ms;m["timing_ms"]["pipeline"]=r.timings.total_ms;
            m["timing_ms"]["obb"]=r.timings.obb_ms;m["timing_ms"]["obb_filter"]=r.timings.obb_filter_ms;
            m["timing_ms"]["serialization_inclusive"]=Ms(start);
            result[mode.first]=m;
        }
        if(voxel_hash!=CloudHash(*voxel))throw std::runtime_error("shared frozen cloud mutated");
        Json(out,result);out<<'\n';
    }
    return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
