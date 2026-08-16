#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <Global/Global.hpp>
// 바운딩 박스 마커 생성
visualization_msgs::Marker createBoxMarker(
    const LiDARObjectInfo &o,
    int id,
    const std_msgs::Header& header
);

visualization_msgs::Marker createOBBMarker(
    const LiDARObjectInfo &o,
    int32 s32_Id,
    const std_msgs::Header& header
);

// 텍스트 마커 생성
visualization_msgs::Marker createTextMarker(
    const LiDARObjectInfo &o,
    int id,
    const std_msgs::Header& header
);

// 화살표(속도 벡터) 마커 생성
visualization_msgs::Marker createArrowMarker(
    const LiDARObjectInfo &o,
    int id,
    const std_msgs::Header& header
);

#endif // VISUALIZATION_H

