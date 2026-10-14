#include "Lidar/Tracking/WarmupMotionEvidence.hpp"
#include <iostream>
int main(){int fail=0;auto check=[&](bool b,const char* s){std::cout<<(b?"[PASS] ":"[FAIL] ")<<s<<'\n';fail+=!b;};
std::deque<motion_evidence::Sample> rigid;
for(int i=0;i<3;++i)rigid.push_back({i*.1,i*1.0-2,i*1.0+2,-1,1,10,0});
auto valid=[](const auto& s){return motion_evidence::Certify(s,.35,10.0/3.6,10.0/3.6,.7);};
check(valid(rigid),"three translating OBBs and matching velocity establish provisional motion");
auto stationary=rigid;for(auto& s:stationary){s.min_x=-2;s.max_x=2;}
check(!valid(stationary),"fast noisy velocity cannot move a stationary OBB");
auto growing=rigid;growing[1].min_x=-2;growing[2].min_x=-2;
check(!valid(growing),"newly visible points growing one edge do not certify motion");
auto opposite=rigid;for(auto& s:opposite)s.vx=-10;
check(!valid(opposite),"opposite velocity direction rejects warm-up motion");
auto duplicate=rigid;duplicate[1].time=duplicate[0].time;
check(!valid(duplicate),"duplicate timestamp cannot confirm motion");
auto fast=rigid;for(auto& s:fast)s.vx=25;
check(!valid(fast),"speed disagreement preserves UNKNOWN");
auto gap=rigid;gap[2].time=1;
check(!valid(gap),"long sampling gap needs reacquisition");
auto missing=rigid;missing.pop_back();check(!valid(missing),"one or two observations stay unconfirmed");
return fail?1:0;}
