#ifndef FRAMEFILTER_HPP
#define FRAMEFILTER_HPP

#include <Global/Global.hpp>
#include "../Tracking/tracking_utils.hpp"

void FrameFilter(deque<vector<LiDARObjectInfo>>& prev_objects_queue,
                       vector<LiDARObjectInfo>& object,const GPS_IMU& st_GPSIMU, VehicleSpec &st_VehicleSpec);

#endif // FRAMEFILTER_H
