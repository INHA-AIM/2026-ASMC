#include <asmc_lidar/experimental/observation_confirmation.hpp>
#include <cassert>
#include <iostream>
#include <string>
using asmc::lidar::pipeline::ObservationConfirmation;
int main(int argc,char**) {
 if(argc==1) {
  ObservationConfirmation g;
  using I=ObservationConfirmation::Input;
  assert(g.Update({I{1,true,false}},true,1000000000,3).count(1));
  assert(g.Update({I{1,true,false}},true,1100000000,3).count(1));
  assert(!g.Update({I{1,true,false}},true,1200000000,3).count(1));
  assert(!g.Update({I{1,false,false}},true,1300000000,3).count(1));
  assert(!g.Update({I{1,true,false}},true,1400000000,3).count(1));
  assert(g.Update({I{2,true,false}},true,1500000000,3).count(2));
  assert(g.Update({I{2,false,false}},true,1600000000,3).count(2));
  assert(g.Update({I{2,true,false}},true,1700000000,3).count(2));
  assert(g.Update({I{2,true,false}},true,1800000000,3).count(2));
  assert(!g.Update({I{2,true,false}},true,1900000000,3).count(2));
  assert(g.Update({I{2,true,false}},true,2300000000,3).count(2));
  assert(g.Update({I{2,true,false}},false,2400000000,3).empty());
  assert(g.Update({I{2,true,false}},true,2500000000,3).count(2));
  assert(!g.Update({I{3,true,true}},true,2600000000,3).count(3));
  assert(g.Update({I{3,true,false}},true,2600000000,3).count(3));
  // Continuous scans with predictions cannot renew a structural confirmation.
  ObservationConfirmation expiry;
  for(int i=0;i<5;++i)expiry.Update({I{9,true,false,5}},true,1000000000LL+i*70000000LL,3);
  for(int i=1;i<=5;++i) {
    const auto h=expiry.Update({I{9,false,false,5}},true,1280000000LL+i*70000000LL,3);
    assert(bool(h.count(9))==(i>=4));
  }
  assert(expiry.Update({I{9,true,false,5}},true,1700000000LL,3).count(9));
  assert(expiry.Run(9)==1);
  for(int i=1;i<5;++i) {
    const auto h=expiry.Update({I{9,true,false,5}},true,1700000000LL+i*70000000LL,3);
    assert(bool(h.count(9))==(i<4));
  }
  // A general obstacle tolerates occlusion while the raw tracker retains its ID.
  ObservationConfirmation general;
  for(int i=0;i<3;++i)general.Update({I{10,true,false,3}},true,1000000000LL+i*70000000LL,3);
  for(int i=1;i<=5;++i)assert(!general.Update({I{10,false,false,3}},true,1140000000LL+i*70000000LL,3).count(10));
  assert(!general.Update({I{10,true,false,3}},true,1560000000LL,3).count(10));
  for(int i=1;i<=16;++i) {
    const auto h=general.Update({I{10,false,false,3}},true,1560000000LL+i*70000000LL,3);
    assert(!h.count(10));
  }
  assert(!general.Update({I{10,true,false,3}},true,2750000000LL,3).count(10));
  general.Update({},true,2820000000LL,3); // raw tracker retires the ID
  assert(general.Update({I{10,true,false,3}},true,2890000000LL,3).count(10));
  // Fresh semantic evidence is allowed immediately after an absence.
  assert(!general.Update({I{10,true,true,5}},true,2960000000LL,3).count(10));
  std::cout<<"confirmation assertions passed\n";return 0;
 }
 ObservationConfirmation g;long long stamp;int active,required,n;
 while(std::cin>>stamp>>active>>required>>n) {
  std::vector<ObservationConfirmation::Input> v;
  for(int k=0;k<n;++k){int id,observed,known;std::cin>>id>>observed>>known;v.push_back({id,observed!=0,known!=0});}
  const auto hidden=g.Update(v,active!=0,stamp,required);
  std::cout<<stamp;for(int id:hidden)std::cout<<" "<<id;std::cout<<"\n";
 }
}
