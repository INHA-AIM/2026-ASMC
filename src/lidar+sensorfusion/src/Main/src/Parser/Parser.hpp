#ifndef PARSER_HPP
#define PARSER_HPP

#include <Global/Global.hpp>
#include <DeadReckoning/DeadReckoning.hpp>
#include <Camera/Camera.hpp>

extern ros::Publisher VizPub;
extern VehicleSpec st_VehicleSpec;
extern GPS_IMU st_GPSIMU, st_GPSIMUTemporal;
extern Control st_Control, st_ControlTemporal;
extern Planning st_Planning, st_PlanningTemporal;
extern LiDAR st_LiDAR, st_LiDARTemporal;
extern CAMERA st_Camera, st_CameraTemporal;
extern GPS_REF st_GPSRef;

extern SENSOR_DATA st_SensorData;

extern pthread_cond_t st_DataTemporalCond;
extern pthread_mutex_t st_DataTemporal;

extern int32 CANHZ, LiDARHZ, IMUHZ, VisionHZ, GPSHZ, VIEWERHZ;

//==========================카메라========================== //  
void *ParseCamera(void *pst_NodeHandle);
void ParseCameraUDP(ros::NodeHandle &st_NodeHandle);
void ParseCameraROS(ros::NodeHandle &st_NodeHandle);
void CameraRosCallback(const sensor_msgs::CompressedImage::ConstPtr& data);
void YoloCallback(const Msg::Yolo_bbox_arr::ConstPtr& data);
//==========================카메라========================== //  

//==========================라이다========================== // 
void *ParseLiDAR(void *pst_NodeHandle);  //라이다
void LiDARParser_Driver_ROS(ros::NodeHandle &st_NodeHandle);                      
void LiDARParser_Callback(const sensor_msgs::PointCloud2::ConstPtr& data); //라이다 
//==========================라이다========================== // 

//==========================GPS========================== // 
void *ParseGPS(void *pst_NodeHandle);
void GPSParser_Callback(const morai_msgs::GPSMessage::ConstPtr &st_Msg);
//==========================GPS========================== // 

//==========================CAN========================== // 
void *ParseCAN(void *pst_NodeHandle);
void CANParser_Callback(const morai_msgs::EgoVehicleStatus::ConstPtr& st_Msg);
//==========================CAN========================== // 

//==========================IMU========================== // 
void *ParseIMU(void *pst_NodeHandle);
void IMUParser_Callback(const sensor_msgs::Imu::ConstPtr& st_Msg);
//==========================IMU========================== // 

//==========================Viewer========================== // 
// void *ParseViewer(void *pst_NodeHandle);
//==========================Viewer========================== // 

#endif