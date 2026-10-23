#pragma once
#include <algorithm>
#include <cmath>
// A representation change only: preserve center and exact corner coordinates.
// The next yaw EMA must refer to the long axis, just as raw L-shape output does.
namespace obb_axis {
template<class Box,class Corners>
bool Canonicalize(Box& box,Corners& xyz) {
    if (!(box.w < box.h)) return false;
    std::swap(box.w,box.h);
    constexpr double pi=3.14159265358979323846;
    box.yaw=std::remainder(box.yaw+pi/2,pi);
    const auto old2=box.corners;const auto old3=xyz;
    for(int i=0;i<4;++i) {
        box.corners[i]=old2[(i+1)%4];
        xyz[i]=old3[(i+1)%4];xyz[i+4]=old3[(i+1)%4+4];
    }
    return true;
}
}
