#include <asmc_lidar/tracking.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace asmc_lidar {
namespace {
constexpr double kGate=3.0,kInf=1e6,kPi=3.14159265358979323846;
double Wrap(double x){return std::remainder(x,2*kPi);}
Eigen::Vector3d RotateZ(double yaw,const Eigen::Vector3d& p){
  return {std::cos(yaw)*p.x()-std::sin(yaw)*p.y(),
          std::sin(yaw)*p.x()+std::cos(yaw)*p.y(),p.z()};
}
// Hungarian rectangular assignment, matching the original distance+IoU
// policy without importing its mutable Global.hpp or GPS-dependent tracker.
std::vector<int> Assign(const std::vector<std::vector<double>>& cost){
  const int n=cost.size();if(!n)return {};
  const int m=cost.front().size();
  std::vector<double> u(n+1),v(m+1);std::vector<int> p(m+1),way(m+1);
  for(int i=1;i<=n;++i){
    p[0]=i;int j0=0;std::vector<double> minv(m+1,kInf*10);
    std::vector<bool> used(m+1);
    do{
      used[j0]=true;const int i0=p[j0];double delta=kInf*10;int j1=0;
      for(int j=1;j<=m;++j)if(!used[j]){
        const double cur=cost[i0-1][j-1]-u[i0]-v[j];
        if(cur<minv[j]){minv[j]=cur;way[j]=j0;}
        if(minv[j]<delta){delta=minv[j];j1=j;}
      }
      for(int j=0;j<=m;++j){if(used[j]){u[p[j]]+=delta;v[j]-=delta;}
                              else minv[j]-=delta;}
      j0=j1;
    }while(p[j0]);
    do{const int j1=way[j0];p[j0]=p[j1];j0=j1;}while(j0);
  }
  std::vector<int> out(n,-1);
  for(int j=1;j<=m;++j)if(p[j])out[p[j]-1]=j-1;
  return out;
}
std::array<double,4> Bounds(const FrameDetection& d){
  std::array<double,4> b={std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  for(const auto& p:d.corners_base){b[0]=std::min(b[0],p.x());b[1]=std::min(b[1],p.y());
                                    b[2]=std::max(b[2],p.x());b[3]=std::max(b[3],p.y());}
  return b;
}
double IoU(const std::array<double,4>& a,const std::array<double,4>& b){
  const double w=std::max(0.,std::min(a[2],b[2])-std::max(a[0],b[0]));
  const double h=std::max(0.,std::min(a[3],b[3])-std::max(a[1],b[1]));
  const double ab=std::max(0.,a[2]-a[0])*std::max(0.,a[3]-a[1]);
  const double bb=std::max(0.,b[2]-b[0])*std::max(0.,b[3]-b[1]);
  const double den=ab+bb-w*h;return den>0?w*h/den:0;
}
void StabilizeYaw(FrameDetection& now,const FrameDetection& before){
  const double delta=.25*std::remainder(now.yaw_base_rad-before.yaw_base_rad,kPi);
  const double yaw=before.yaw_base_rad+delta;
  const Eigen::Vector3d axis(std::cos(yaw),std::sin(yaw),0);
  const Eigen::Vector3d side(-std::sin(yaw),std::cos(yaw),0);
  double u0=1e30,u1=-1e30,v0=1e30,v1=-1e30,z0=1e30,z1=-1e30;
  for(const auto& c:now.corners_base){u0=std::min(u0,axis.dot(c));u1=std::max(u1,axis.dot(c));
    v0=std::min(v0,side.dot(c));v1=std::max(v1,side.dot(c));z0=std::min(z0,c.z());z1=std::max(z1,c.z());}
  const double uc=(u0+u1)/2,vc=(v0+v1)/2;
  now.center_base=axis*uc+side*vc+Eigen::Vector3d(0,0,(z0+z1)/2);
  for(int k=0;k<8;++k){const double u=(k==1||k==2||k==5||k==6)?u1:u0;
    const double v=(k==2||k==3||k==6||k==7)?v1:v0;
    now.corners_base[k]=axis*u+side*v+Eigen::Vector3d(0,0,k<4?z0:z1);}
  now.forward_base=axis;now.yaw_base_rad=yaw;
}
std::vector<std::string> CameraMatches(const std::vector<FrameDetection>& geometry,
                                      const CameraObservation* camera,std::int64_t stamp){
  std::vector<std::string> labels(geometry.size(),"unknown");
  if(!camera||!camera->valid||camera->stamp_ns<=0||camera->frame_id!="Camera-1"||
     camera->width==0||camera->height==0||std::llabs(camera->stamp_ns-stamp)>50000000)return labels;
  struct Candidate{double score;std::size_t object,box;};std::vector<Candidate> candidates;
  // Autonav v3 camera 1 mount and 90 degree horizontal FOV; source is
  // ASMC_sensor_set_v3.json. This is the same vehicle->optical convention as
  // the original FusionCamera implementation. Observed OBB only.
  const Eigen::Vector3d camera_pos(1.9,0,1.2);
  const double pitch=2*kPi/180;
  const double fx=camera->width/2.,fy=fx,cx=fx,cy=camera->height/2.;
  for(std::size_t i=0;i<geometry.size();++i){
    double x0=1e30,y0=1e30,x1=-1e30,y1=-1e30;int visible=0;
    for(const auto& p:geometry[i].corners_base){
      const auto q=Eigen::AngleAxisd(-pitch,Eigen::Vector3d::UnitY())*(p-camera_pos);
      if(q.x()<=.05)continue;
      const double x=fx*(-q.y()/q.x())+cx,y=fy*(-q.z()/q.x())+cy;
      x0=std::min(x0,x);y0=std::min(y0,y);x1=std::max(x1,x);y1=std::max(y1,y);++visible;
    }
    if(!visible)continue;
    x0=std::clamp(x0,0.,double(camera->width));x1=std::clamp(x1,0.,double(camera->width));
    y0=std::clamp(y0,0.,double(camera->height));y1=std::clamp(y1,0.,double(camera->height));
    const double mx=(x0+x1)/2,my=(y0+y1)/2,w=(x1-x0)*1.3,h=(y1-y0)*1.5;
    const std::array<double,4> projected={mx-w/2,my-h/2,mx+w/2,my+h/2};
    for(std::size_t j=0;j<camera->boxes.size();++j){const auto& box=camera->boxes[j];
      const double score=IoU(projected,{box.xmin,box.ymin,box.xmax,box.ymax});
      if(score>.25)candidates.push_back({score,i,j});}
  }
  std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){
    return std::tie(a.score,a.object,a.box)>std::tie(b.score,b.object,b.box);});
  std::vector<bool> used_box(camera->boxes.size());
  for(const auto& c:candidates)if(labels[c.object]=="unknown"&&!used_box[c.box]){
    labels[c.object]=camera->boxes[c.box].label;used_box[c.box]=true;}
  return labels;
}
}  // namespace
void TrackingEngine::Reset(){tracks_.clear();last_stamp_ns_=0;last_yaw_rad_=0;next_id_=1;}
std::vector<TrackedDetection> TrackingEngine::Update(const UnifiedResult& scan,double ego_yaw,
                                      const EgoTwist& twist,const CameraObservation* camera){
  if(scan.observation_ns<=0||!std::isfinite(ego_yaw))throw std::invalid_argument("invalid tracker clock/IMU");
  if(last_stamp_ns_&&scan.observation_ns<=last_stamp_ns_)throw std::invalid_argument("nonmonotonic tracker stamp");
  const double dt=last_stamp_ns_?double(scan.observation_ns-last_stamp_ns_)*1e-9:0;
  const double delta_yaw=last_stamp_ns_?Wrap(ego_yaw-last_yaw_rad_):0;
  const auto classes=CameraMatches(scan.detections,camera,scan.observation_ns);
  for(auto& track:tracks_){
    const auto previous=track.state;
    const auto old_center=track.output.geometry.center_base;
    const auto p=RotateZ(-delta_yaw,{previous.x(),previous.y(),0});
    const auto v=RotateZ(-delta_yaw,{previous.z(),previous.w(),0});
    track.state={p.x(),p.y(),v.x(),v.y()};
    if(twist.valid)track.state.head<2>()-=twist.linear_base.head<2>()*dt;
    Eigen::Matrix4d f=Eigen::Matrix4d::Identity();f(0,2)=dt;f(1,3)=dt;
    track.state=f*track.state;track.covariance=f*track.covariance*f.transpose()+Eigen::Matrix4d::Identity()*.05;
    const Eigen::Vector3d new_center(track.state.x(),track.state.y(),old_center.z());
    for(auto& corner:track.output.geometry.corners_base)
      corner=RotateZ(-delta_yaw,corner-old_center)+new_center;
    track.output.geometry.forward_base=RotateZ(-delta_yaw,track.output.geometry.forward_base);
    track.output.geometry.yaw_base_rad=std::atan2(track.output.geometry.forward_base.y(),
                                                 track.output.geometry.forward_base.x());
    track.output.geometry.center_base=new_center;
  }
  const int n=tracks_.size(),m=scan.detections.size();
  std::vector<std::vector<double>> cost(n,std::vector<double>(m+n,kGate));
  for(int i=0;i<n;++i)for(int j=0;j<m;++j){
    const double dx=tracks_[i].state.x()-scan.detections[j].center_base.x();
    const double dy=tracks_[i].state.y()-scan.detections[j].center_base.y();
    const double dist=std::hypot(dx,dy);
    const double score=dist+1-IoU(Bounds(tracks_[i].output.geometry),Bounds(scan.detections[j]));
    cost[i][j]=score<kGate?score:kInf;
  }
  const auto assignment=Assign(cost);std::vector<bool> used(m);
  for(int i=0;i<n;++i){auto& track=tracks_[i];const int j=assignment[i];
    track.output.age++;
    if(j>=0&&j<m&&cost[i][j]<kGate){
      used[j]=true;FrameDetection geometry=scan.detections[j];
      const double previous_yaw=track.output.geometry.yaw_base_rad;
      StabilizeYaw(geometry,track.output.geometry);
      track.output.yaw_rate_valid=dt>0;
      track.output.yaw_rate_rad_s=dt>0?std::remainder(geometry.yaw_base_rad-previous_yaw,kPi)/dt:0;
      const Eigen::Vector2d z(geometry.center_base.x(),geometry.center_base.y());
      Eigen::Matrix<double,2,4> h=Eigen::Matrix<double,2,4>::Zero();h(0,0)=h(1,1)=1;
      const Eigen::Matrix2d s=h*track.covariance*h.transpose()+Eigen::Matrix2d::Identity()*.15;
      const Eigen::Matrix<double,4,2> k=track.covariance*h.transpose()*s.inverse();
      track.state+=k*(z-h*track.state);
      track.covariance=(Eigen::Matrix4d::Identity()-k*h)*track.covariance;
      track.output.geometry=std::move(geometry);track.output.observed=true;track.output.miss_count=0;
      track.output.relative_velocity_valid=true;
      const auto& label=classes[j];
      const bool new_camera=camera&&camera->valid&&camera->frame_id=="Camera-1"&&
        camera->width>0&&camera->height>0&&camera->stamp_ns>track.last_camera_stamp_ns&&
        std::llabs(camera->stamp_ns-scan.observation_ns)<=50000000;
      if(new_camera&&label!="unknown"){
        if(label==track.pending_class)++track.class_confirmations;
        else {track.pending_class=label;track.class_confirmations=1;}
        const int needed=track.output.classification=="unknown"?5:3;
        if(track.class_confirmations>=needed){track.output.camera_class=label;
          track.output.classification=(label=="car"||label=="truck"||label=="bus")?"car":
                                      label=="person"?"person":"unknown";}
        track.class_misses=0;
      }else if(new_camera){
        if(++track.class_misses>3){track.pending_class="unknown";track.class_confirmations=0;}
      }
      if(new_camera)track.last_camera_stamp_ns=camera->stamp_ns;
    }else {track.output.observed=false;track.output.yaw_rate_valid=false;++track.output.miss_count;
      track.output.geometry.source_indices.clear();
      track.output.geometry.center_base.x()=track.state.x();
      track.output.geometry.center_base.y()=track.state.y();}
    track.output.relative_velocity_base={track.state.z(),track.state.w(),0};
    track.output.absolute_velocity_valid=track.output.relative_velocity_valid&&twist.valid;
    if(track.output.absolute_velocity_valid){
      track.output.absolute_velocity_base=track.output.relative_velocity_base+twist.linear_base+
        twist.angular_base.cross(track.output.geometry.center_base);
      track.output.dynamic_valid=track.output.age>=3;
      track.output.is_dynamic=track.output.dynamic_valid&&track.output.absolute_velocity_base.head<2>().norm()>10./3.6;
    }else track.output.dynamic_valid=false;
  }
  for(int j=0;j<m;++j)if(!used[j]){Track track;track.output.id=next_id_++;
    track.output.geometry=scan.detections[j];track.output.age=1;track.output.observed=true;
    track.state.x()=scan.detections[j].center_base.x();track.state.y()=scan.detections[j].center_base.y();
    if(classes[j]!="unknown"){
      track.pending_class=classes[j];track.class_confirmations=1;
      track.last_camera_stamp_ns=camera->stamp_ns;
    }
    tracks_.push_back(std::move(track));}
  tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[](const auto& t){return t.output.miss_count>=25;}),tracks_.end());
  last_stamp_ns_=scan.observation_ns;last_yaw_rad_=ego_yaw;
  std::vector<TrackedDetection> result;result.reserve(tracks_.size());
  for(const auto& t:tracks_)result.push_back(t.output);
  return result;
}
}  // namespace asmc_lidar
