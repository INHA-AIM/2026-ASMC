#ifndef DEADRECKONING_HPP
#define DEADRECKONING_HPP

#include <Global/Global.hpp>

void DeadReckoningProcess(RAW_GPS_DATA &st_RawGPS, RAW_IMU_DATA &st_RawIMU, RAW_CAN_DATA &st_RawCAN, GPS_IMU &st_GPSIMU);

#endif