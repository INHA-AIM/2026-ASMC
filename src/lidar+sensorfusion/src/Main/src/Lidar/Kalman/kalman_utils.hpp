// include/filter/kalman_utils.h
#ifndef FILTER_KALMAN_UTILS_HPP
#define FILTER_KALMAN_UTILS_HPP

#include <Global/Global.hpp>

#include <Lidar/IMMFilter/IMMFilter.hpp>
//헝가리안 알고리즘 행렬 만들기 위한 위치 칼만 예측 함수(업데이트 및 보정단계 없음)
pair<float,float> predictKalmanPosition(const LiDARObjectInfo& obj, float64 f64_Dt,float ego_yaw_now);
//현재프레임에서는 존재하지 않는 트래킹 중인 객체에 대한 칼만 예측 함수(상태벡터,공분산 업데이트;
//공통 기하 모드에서는 이전 scan OBB 중심·꼭짓점·yaw도 현재 프레임으로 재표현만 하고 이동 예측은 하지 않음)
void kalman_filter_predict(LiDARObjectInfo &o, float64 f64_Dt,float ego_yaw_now);
//현재프레임에서 트래킹되고 있는 객체에 대한 칼만 필터 함수(OBB는 현재 scan 검출 그대로 두고 필터 상태만 재표현)
void kalman_filter_update(LiDARObjectInfo &obj,
                          float32 f32_MeasuredX,float32 f32_MeasuredY,
                          float32 f32_Pred_x,float32 f32_Pred_y ,
                          float64 f64_Dt,float ego_yaw_now);


#endif // FILTER_KALMAN_UTILS_H
