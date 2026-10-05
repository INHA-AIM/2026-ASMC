#include <Global/Global.hpp>

#include <pthread.h>

// myHeader is also used by the production executable, where these objects are
// owned by main.cpp. Unit-test executables provide inert instances so that a
// small clustering/tracking test does not have to start the complete runtime.
VehicleSpec st_VehicleSpec{};
Control st_Control{};
CAMERA st_Camera{};
SENSOR_DATA st_SensorData{};

int32 GPSHZ = 0;
int32 VisionHZ = 0;
int32 LiDARHZ = 0;
int32 IMUHZ = 0;
int32 CANHZ = 0;

pthread_cond_t st_DataTemporalCond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t st_DataTemporal = PTHREAD_MUTEX_INITIALIZER;
std::uint64_t u64_LidarFrameGeneration = 0;
