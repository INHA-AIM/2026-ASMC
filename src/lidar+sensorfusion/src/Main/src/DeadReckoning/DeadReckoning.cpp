#include "DeadReckoning.hpp"


void DeadReckoningProcess(RAW_GPS_DATA &st_RawGPS, RAW_IMU_DATA &st_RawIMU, RAW_CAN_DATA &st_RawCAN, GPS_IMU &st_GPSIMU)
{
    memcpy(&st_GPSIMU.st_GPS, &st_RawGPS.arc_Buffer, sizeof(GPS_DATA));
    memcpy(&st_GPSIMU.st_IMU, &st_RawIMU.arc_Buffer, sizeof(IMU_DATA));
    memcpy(&st_GPSIMU.st_CAN, &st_RawCAN.arc_Buffer, sizeof(CAN_DATA));
    
    // cout << "============================================" << endl;
    // cout << "Latitude : " << st_GPSIMU.st_GPS.f32_Latitude << "\n"
    // 	<< "Longitude : " << st_GPSIMU.st_GPS.f32_Longitude << "\n"
    // 	<< "Altitude : " << st_GPSIMU.st_GPS.f32_Altitude << "\n"
    // 	<< "East : " << st_GPSIMU.st_GPS.f32_E << "\n"
    // 	<< "North : " << st_GPSIMU.st_GPS.f32_N << "\n"
    // 	<< "Up : " << st_GPSIMU.st_GPS.f32_U << "\n"
    // 	<< "ACC X : " << st_GPSIMU.st_IMU.f32_AccelX << "\n" 
	// 	<< "ACC Y : " << st_GPSIMU.st_IMU.f32_AccelY << "\n"
	// 	<< "ACC Z : " << st_GPSIMU.st_IMU.f32_AccelZ << "\n"
	// 	<< "Quat X : " << st_GPSIMU.st_IMU.f32_QuatX << "\n"
	// 	<< "Quat Y : " << st_GPSIMU.st_IMU.f32_QuatY << "\n"
	// 	<< "Quat Z : " << st_GPSIMU.st_IMU.f32_QuatZ << "\n"
	// 	<< "Quat W : " << st_GPSIMU.st_IMU.f32_QuatW << "\n"
	// 	<< "Angular X : " << st_GPSIMU.st_IMU.f32_AngularVelX << "\n"
	// 	<< "Angular Y : " << st_GPSIMU.st_IMU.f32_AngularVelY << "\n"
	// 	<< "Angular Z : " << st_GPSIMU.st_IMU.f32_AngularVelZ << "\n"
	// 	<< "Roll rad : " << st_GPSIMU.st_IMU.f32_Roll_rad << "\n"
	// 	<< "Pitch rad : " << st_GPSIMU.st_IMU.f32_Pitch_rad << "\n"
	// 	<< "Yaw rad : " << st_GPSIMU.st_IMU.f32_Yaw_rad << "\n"
	// 	<< "Roll deg : " << st_GPSIMU.st_IMU.f32_Roll_deg << "\n"
	// 	<< "Pitch deg : " << st_GPSIMU.st_IMU.f32_Pitch_deg << "\n"
	// 	<< "Yaw deg : " << st_GPSIMU.st_IMU.f32_Yaw_deg << "\n"
	// 	<< "Current Vel KPH : " << st_GPSIMU.st_CAN.f32_CurrentVel_KPH << "\n"
	// 	<< "Current Vel MPS : " << st_GPSIMU.st_CAN.f32_CurrentVel_MPS << "\n"
	// 	<< "Steer deg : " << st_GPSIMU.st_CAN.f32_Steer_deg << "\n"
	// 	<< "Steer rad : " << st_GPSIMU.st_CAN.f32_Steer_rad << endl;
}