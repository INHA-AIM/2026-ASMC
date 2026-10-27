#include <asmc_lidar/experimental/observation_confirmation.hpp>
#include <asmc_lidar/experimental/confirmation_candidate.hpp>
#include <cassert>
#include <iostream>
#include <limits>
using namespace asmc::lidar::pipeline;
int main(int argc,char**) {
 if(argc==1) {
  ConfirmationEvidence e;e.length=7;e.width=.2;e.height=.1;e.ground_valid=true;e.median_ground_clearance=.05;
  assert(ClassifyConfirmation(e)==ConfirmationKind::GroundFragment);
  e.median_ground_clearance=.7;assert(ClassifyConfirmation(e)==ConfirmationKind::General);
  e.length=2;e.width=.15;e.height=.02;e.median_ground_clearance=.45;
  assert(ClassifyConfirmation(e)==ConfirmationKind::ThinHorizontalFragment);
  e.width=1;assert(ClassifyConfirmation(e)==ConfirmationKind::General);
  e.length=12;e.width=2;e.height=.18;e.median_ground_clearance=.435;
  assert(ClassifyConfirmation(e)==ConfirmationKind::ElevatedFlatFragment);
  e.length=2;e.width=1;assert(ClassifyConfirmation(e)==ConfirmationKind::General);
  e.length=7;e.width=.2;
  e.ground_valid=false;e.height=2;e.wall_valid=true;e.wall_support_fraction=.9;e.wall_axis_sine=.02;
  assert(ClassifyConfirmation(e)==ConfirmationKind::WallFragment);
  e.wall_support_fraction=.2;assert(ClassifyConfirmation(e)==ConfirmationKind::General);
  e.wall_support_fraction=.9;e.wall_axis_sine=.5;assert(ClassifyConfirmation(e)==ConfirmationKind::General);
  e.length=.6;e.width=.5;e.height=3;assert(ClassifyConfirmation(e)==ConfirmationKind::WallFragment);
  e.length=2.5;e.width=.5;e.height=3;assert(ClassifyConfirmation(e)==ConfirmationKind::WallFragment);
  e.length=1;e.width=1;e.height=1;assert(ClassifyConfirmation(e)==ConfirmationKind::General);
  e.length=std::numeric_limits<double>::quiet_NaN();assert(ClassifyConfirmation(e)==ConfirmationKind::General);
  ObservationConfirmation g;for(int i=0;i<5;++i){auto h=g.Update({{1,true,false,5},{2,true,false,3},{3,true,true,5}},true,1000000000LL+i*70000000LL,3);assert(bool(h.count(1))==(i<4));assert(bool(h.count(2))==(i<2));assert(!h.count(3));}
  ObservationConfirmation sticky;
  assert(sticky.Update({{7,true,false,5}},true,1000000000,3).count(7));
  assert(sticky.Update({{7,true,false,5}},true,1070000000,3).count(7));
  assert(sticky.Update({{7,true,false,3}},true,1140000000,3).count(7));
  assert(sticky.Update({{7,true,false,3}},true,1210000000,3).count(7));
  assert(!sticky.Update({{7,true,false,3}},true,1280000000,3).count(7));
  ObservationConfirmation reclassified;
  for(int i=0;i<3;++i)reclassified.Update({{8,true,false,3}},true,1000000000LL+i*70000000LL,3);
  for(int i=0;i<6;++i) {
    const auto h=reclassified.Update({{8,true,false,6}},true,1210000000LL+i*70000000LL,3);
    assert(bool(h.count(8))==(i<5));
  }
  assert(!reclassified.Update({{8,true,true,7}},true,1630000000LL,3).count(8));
  std::cout<<"adaptive assertions passed\n";return 0;
 }
 long long stamp;int active,n;ObservationConfirmation g;
 while(std::cin>>stamp>>active>>n) {
  std::vector<ObservationConfirmation::Input> input;std::vector<int> kinds;
  for(int i=0;i<n;++i) {
   int id,obs,known;ConfirmationEvidence e;
   std::cin>>id>>obs>>known>>e.length>>e.width>>e.height>>e.ground_valid>>e.median_ground_clearance>>e.wall_valid>>e.wall_support_fraction>>e.wall_axis_sine;
   auto k=ClassifyConfirmation(e);kinds.push_back(int(k));input.push_back({id,obs!=0,known!=0,k==ConfirmationKind::General?3:5});
  }
  auto hidden=g.Update(input,active!=0,stamp,3);std::cout<<stamp<<" "<<n;
  for(int k:kinds){std::cout<<" "<<k;}
  std::cout<<" "<<hidden.size();for(int id:hidden)std::cout<<" "<<id;std::cout<<"\n";
 }
}
