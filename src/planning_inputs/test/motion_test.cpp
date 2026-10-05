#include <cmath>
#include <iostream>
#include <vector>
#include "planning_inputs/MotionClassifier.hpp"
#include "planning_inputs/heading_history.hpp"
int failures=0;
void Check(bool ok,const char* name){std::cout<<(ok?"PASS ":"FAIL ")<<name<<'\n';failures+=!ok;}
MotionObject Object(double x=0,double vx=0){MotionObject o;o.s32_Id=1;o.f32_CoreE=x;o.f32_CoreN=1;o.f32_AbsVx=vx;o.min_e=x-2;o.max_e=x+2;o.min_n=0;o.max_n=2;return o;}
int main(){
 MotionClassifier classifier;std::vector<MotionObject> objects{Object()};
 classifier.Update(objects,1);Check(objects[0].s32_MotionState==0,"startup is UNKNOWN");
 for(int i=1;i<30;++i)classifier.Update(objects,1+i*.1);
 Check(objects[0].s32_MotionState==1,"stationary geometry confirms LOW_THREAT");
 for(int i=0;i<15;++i){objects[0]=Object(0,12);objects[0].max_e=2+i*.4;objects[0].f32_CoreE=i*.2;classifier.Update(objects,4+i*.1);}
 Check(objects[0].s32_MotionState==1,"one growing OBB face does not become rigid motion");
 classifier.Reset();
 for(int i=0;i<30;++i){objects[0]=Object(i*.6,6);classifier.Update(objects,10+i*.1);}
 Check(objects[0].s32_MotionState==3,"rigid translation confirms DYNAMIC");
 objects[0].s32_MissCount=4;
 for(int i=0;i<4;++i)classifier.Update(objects,14+i*.1);
 Check(objects[0].s32_MotionState==0,"long miss clears confirmed state");
 classifier.Reset();objects[0]=Object();classifier.Update(objects,20);
 Check(objects[0].s32_MotionState==0,"reset requires fresh confirmation");
 HeadingHistory headings;headings.Add(1,3.1);headings.Add(1.1,-3.1);
 Check(headings.At(1.05).has_value()&&std::abs(*headings.At(1.05))>3.1,"heading interpolation crosses pi continuously");
 Check(!headings.At(2).has_value(),"stale heading rejected");
 headings.Add(2,0);Check(!headings.At(1.5).has_value(),"large heading gap rejected");
 return failures?1:0;
}
