#include <Lidar/FrameFilter/FrameFilter.hpp>
#include <Lidar/Kalman/kalman_utils.hpp>
#include <SensorFusion/FusionStateTrace.hpp>

constexpr float Ton  = 0.8f;  // m/s
constexpr float Toff = 0.5f;  // m/s
constexpr int   N_on  = 2;
constexpr int   N_off = 3;

void FrameFilter(deque<vector<LiDARObjectInfo>>& prev_ObjectsQueue,
                 vector<LiDARObjectInfo>& vec_Object, const GPS_IMU& st_GPSIMU, VehicleSpec &st_VehicleSpec)
{
    // VehicleSpec st_VehicleSpec ; //일단 0으로 초기화 함
    // cout<<"라이다 위치:"<<st_VehicleSpec.st_LiDARPos.f32_X<<endl;
    // {
    //     LiDARObjectInfo car{};
    //     car.f32_CoreX = car.f32_CoreY = car.f32_CoreZ = 0.0f;  // 차량 기준 원점
    //     Body2ENU(st_VehicleSpec, st_GPSIMU, car);
    //     MinMax2ENU(st_VehicleSpec, st_GPSIMU, vec_Object[s32_Idx]);
    //     // ROS_INFO_THROTTLE(0.5, "[POSE] car_ENU=(%.3f, %.3f)", car.f32_CoreE, car.f32_CoreN);
    // }

    // 한 물체가 높이 방향 등으로 둘 이상 분할된 경우에는 트랙 ID를 부여하기
    // 전에 동일 XY 점유를 하나로 합친다. 그 다음 현재 관측의 map 중심을 먼저
    // 계산해야 자차가 빠르게 움직여도 이전 프레임과 같은 좌표계에서 연관된다.
    ConsolidateCurrentDetections(vec_Object);
    for (auto& object : vec_Object) {
        Body2ENU(st_VehicleSpec, st_GPSIMU, object);
        MinMax2ENU(st_VehicleSpec, st_GPSIMU, object);
    }

    if (prev_ObjectsQueue.empty()) {
        for (int32 s32_Idx = 0; s32_Idx < static_cast<int>(vec_Object.size()); ++s32_Idx) {
            if (vec_Object[s32_Idx].f32_CoreX == 0.0f &&
                vec_Object[s32_Idx].f32_CoreY == 0.0f &&
                vec_Object[s32_Idx].f32_CoreZ == 0.0f)
            {
                break;
            }
            vec_Object[s32_Idx].s32_Id           = st_Tracking.s32_NewTrackId++;
            vec_Object[s32_Idx].s32_Age          = 1;    // 첫 등장 프레임
            vec_Object[s32_Idx].s32_MissCount    = 0;
            fusion_state_trace::Initial("class_track_birth", vec_Object[s32_Idx], ts_CurrTime);
            vec_Object[s32_Idx].b_KfInitialized  = false;

            // Initialize the filter on the first observation. Previously the
            // first frame only received an ID, so the second/third-frame
            // association had no meaningful predicted state.
            kalman_filter_update(
                vec_Object[s32_Idx],
                vec_Object[s32_Idx].f32_CoreX,
                vec_Object[s32_Idx].f32_CoreY,
                vec_Object[s32_Idx].f32_CoreX,
                vec_Object[s32_Idx].f32_CoreY,
                0.1, st_GPSIMU.st_IMU.f32_Yaw_deg);

            Body2ENU(st_VehicleSpec, st_GPSIMU, vec_Object[s32_Idx]);
            MinMax2ENU(st_VehicleSpec, st_GPSIMU, vec_Object[s32_Idx]);
            vec_Object[s32_Idx].f32_PrevE = vec_Object[s32_Idx].f32_CoreE;
            vec_Object[s32_Idx].f32_PrevN = vec_Object[s32_Idx].f32_CoreN;
        }
        return;
    }

    // 전프레임 존재
    vector<LiDARObjectInfo>& prev = prev_ObjectsQueue.back();

    // 이전 프레임에 실측이 하나라도 있는지 검사
    bool b_prevAllDummy = true;
    for (int32 s32_Idx = 0; s32_Idx < static_cast<int>(prev.size()); ++s32_Idx) {
        if (!(prev[s32_Idx].f32_CoreX == 0.0f &&
              prev[s32_Idx].f32_CoreY == 0.0f &&
              prev[s32_Idx].f32_CoreZ == 0.0f))
        {
            b_prevAllDummy = false;
            break;
        }
    }

    // 이전 프레임에 실측이 하나라도 있으면 매칭 & ghost 처리
    if (!b_prevAllDummy) {
        float64 f64_Dt = (ts_CurrTime - ts_PrevTime).toSec();
        CompareCentroids(prev_ObjectsQueue, vec_Object, f64_Dt, st_GPSIMU);

        for (int32 s32_Idx = 0; s32_Idx < static_cast<int>(vec_Object.size()); ++s32_Idx) {
            Body2ENU(st_VehicleSpec, st_GPSIMU, vec_Object[s32_Idx]);
            MinMax2ENU(st_VehicleSpec, st_GPSIMU, vec_Object[s32_Idx]);
            const double E_cur  = (double)vec_Object[s32_Idx].f32_CoreE;
            const double N_cur  = (double)vec_Object[s32_Idx].f32_CoreN;
            const double E_prev = (double)vec_Object[s32_Idx].f32_PrevE;
            const double N_prev = (double)vec_Object[s32_Idx].f32_PrevN;
            //바운딩 박스 안나온거 처리 되는지 확인 로그
            // cout<<"ID:"<<vec_Object[s32_Idx].s32_Id<<endl;
            // cout<<"x크기:"<<vec_Object[s32_Idx].f32_MaxX - vec_Object[s32_Idx].f32_MinX<<endl;
            // cout<<"y크기:"<<vec_Object[s32_Idx].f32_MaxY - vec_Object[s32_Idx].f32_MinY<<endl;
            // cout<<"z크기:"<<vec_Object[s32_Idx].f32_MaxZ - vec_Object[s32_Idx].f32_MinZ<<endl;
            if (vec_Object[s32_Idx].s32_Age > 1 &&
                f64_Dt > 1e-6 &&
                std::isfinite(E_prev) && std::isfinite(N_prev))
            {
                const double vx = ((E_cur - E_prev) / f64_Dt);
                const double vy = (N_cur - N_prev) / f64_Dt;

                // ========== raw 속도 ==========
                const float32 raw_speed = (float32)std::hypot(vx, vy);
                // cout<<"요값:"<<st_GPSIMU.st_IMU.f32_Yaw_rad<<endl;
                // vec_Object[s32_Idx].f32_Velocity = raw_speed-st_GPSIMU.st_CAN.f32_CurrentVel_MPS;
                // cout<<"전체값:"<<raw_speed-st_GPSIMU.st_CAN.f32_CurrentVel_MPS<<endl;

                // // Prev 갱신
                vec_Object[s32_Idx].f32_PrevE = vec_Object[s32_Idx].f32_CoreE;
                vec_Object[s32_Idx].f32_PrevN = vec_Object[s32_Idx].f32_CoreN;
            } else {
                // 첫 프레임(AGE==1) 또는 PrevE/N 무효 → 이번 프레임은 속도 계산 스킵하고 초기화만
                vec_Object[s32_Idx].f32_PrevE = vec_Object[s32_Idx].f32_CoreE;
                vec_Object[s32_Idx].f32_PrevN = vec_Object[s32_Idx].f32_CoreN;
            }
        }
    }
    else {
        // 이전에도 완전 더미만 있었다면 매칭 자체를 건너뜁니다
        for (int32 s32_Idx = 0; s32_Idx < (int32)vec_Object.size(); ++s32_Idx) {
            if (vec_Object[s32_Idx].f32_CoreX == 0.0f &&
                vec_Object[s32_Idx].f32_CoreY == 0.0f &&
                vec_Object[s32_Idx].f32_CoreZ == 0.0f)
            {
                break;
            }
            if (vec_Object[s32_Idx].s32_Id < 0) {
                vec_Object[s32_Idx].s32_Id           = st_Tracking.s32_NewTrackId++;
                vec_Object[s32_Idx].s32_Age          = 1;
                vec_Object[s32_Idx].s32_MissCount    = 0;
                fusion_state_trace::Initial("class_track_birth", vec_Object[s32_Idx], ts_CurrTime);
                vec_Object[s32_Idx].b_KfInitialized  = false;

                kalman_filter_update(
                    vec_Object[s32_Idx],
                    vec_Object[s32_Idx].f32_CoreX,
                    vec_Object[s32_Idx].f32_CoreY,
                    vec_Object[s32_Idx].f32_CoreX,
                    vec_Object[s32_Idx].f32_CoreY,
                    0.1, st_GPSIMU.st_IMU.f32_Yaw_deg);

                Body2ENU(st_VehicleSpec, st_GPSIMU, vec_Object[s32_Idx]);
                MinMax2ENU(st_VehicleSpec, st_GPSIMU, vec_Object[s32_Idx]);
                vec_Object[s32_Idx].f32_PrevE = vec_Object[s32_Idx].f32_CoreE;
                vec_Object[s32_Idx].f32_PrevN = vec_Object[s32_Idx].f32_CoreN;
            }
        }
    }
}
