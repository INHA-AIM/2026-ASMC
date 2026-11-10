#include "Parser/RoundaboutEmergencyAuthority.hpp"
#include <cassert>
#include <iostream>
int main(){using asmc_planner::NativeOwnsRoundaboutGapEmergency;
 auto valid=[](bool round=true,bool executing=true,bool match=true,bool plan=true,bool locked=true,bool fresh=true,bool ttc=false,double age=.05,bool linked=true){return NativeOwnsRoundaboutGapEmergency(round,executing,match,plan,locked,fresh,ttc,age,linked);};
 assert(valid());assert(!valid(false));assert(!valid(true,false));assert(!valid(true,true,false));assert(!valid(true,true,true,false));assert(!valid(true,true,true,true,false));assert(!valid(true,true,true,true,true,false));assert(!valid(true,true,true,true,true,true,true));assert(!valid(true,true,true,true,true,true,false,.151));assert(!valid(true,true,true,true,true,true,false,-.01));assert(!valid(true,true,true,true,true,true,false,.05,false));
 std::cout<<"PASS native authority and TTC/stale/invalid/wrong-owner rejection\n";
}
