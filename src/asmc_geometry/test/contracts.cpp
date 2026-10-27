#include <asmc_geometry/geodesy.hpp>
#include <asmc_geometry/time.hpp>
#include <deque>
#include <limits>
#include <stdexcept>
#include <iostream>
using namespace asmc::geometry;
void require(bool v){if(!v)throw std::runtime_error("geometry/time contract failure");}
int main(){
 require(!ValidGpsFix(1,true,0,0,0,0));
 require(!ValidGpsFix(1,true,37,127,INFINITY,0));
 require(!ValidGpsFix(1,false,37,127,0,0));
 require(ValidGpsFix(1,true,37.24,126.77,302595,4124145));
 double e,n;GpsToUtm52N(0,129,e,n);require(std::abs(e-500000)<1e-8&&std::abs(n)<1e-8);
 require(std::abs(Utm52NYawOffset(37.24,129))<1e-8);
 require(Utm52NYawOffset(37.24,126.77)<0);
 double x,y,z;GpsToEnu(37,127,25,37,127,25,x,y,z);require(x==0&&y==0&&z==0);
 const auto ns=1793000000000000000ULL;
 require(WithinAge(ns,ns+120000000,.12));
 require(!WithinAge(ns,ns+120000001,.12));
 require(!WithinAge(ns,ns-1,.12));require(!WithinAge(0,ns,.12));
 require(!WithinAge(ns,ns,std::numeric_limits<double>::quiet_NaN()));
 std::deque<std::uint64_t> stamps{ns,ns+10,ns+20};auto key=[](auto v){return v;};
 require(LatestAtOrBefore(stamps,ns-1,key)==stamps.end());
 require(*LatestAtOrBefore(stamps,ns+10,key)==ns+10);
 require(*LatestAtOrBefore(stamps,ns+19,key)==ns+10);
 std::cout<<"shared geodesy/time contracts passed\n";
}
