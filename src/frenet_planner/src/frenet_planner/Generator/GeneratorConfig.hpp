// 후보 궤적 샘플링 격자(grid) 설정.
//
// 논문 Sec.IV-A / V-A / V-B 는 모두 "여러 종료조건(di 또는 Δsi 또는 Δṡi)과
// 여러 종료시간 Tj 의 조합"으로 후보 집합을 만든다. 이 프로젝트에서는
// lateral과 longitudinal이 Sec.VI에서 "Tlat × Tlon 모든 조합"으로 묶이는데,
// 이때 서로 다른 T를 가진 후보끼리 묶으면 두 다항식의 정의 구간이 달라져
// 궤적을 만들 수 없다. 그래서 lateral/longitudinal이 동일한 T_j 격자를
// 공유하도록 하고(time_horizon 하나로 통일), CombineLateralLongitudinal은
// 같은 T를 가진 후보끼리만 묶는다.
//
// (min, max, step)으로 표현: 실제 후보값은 min부터 max까지 step 간격
// (parameter.yaml에서 로드될 예정, 지금은 코드에서 직접 채워도 됨)

#ifndef FRENET_GENERATOR_CONFIG_HPP
#define FRENET_GENERATOR_CONFIG_HPP

struct SamplingRange {
    double min;
    double max;
    double step;
};

struct PathGeneratorConfig {
    SamplingRange lateral_d1;      // 목표 횡방향 오프셋 d1 후보 범위 [m] (Sec.IV-A)
    double lateral_target_tolerance; // [m] 모드가 지정한 목표 d 주변 탐색 폭
    SamplingRange time_horizon;    // 종료시간 Tj 후보 범위 [s] (lateral/longitudinal 공통)
    SamplingRange delta_s;         // following/merging/stopping용 Δsi 범위 [m] (Sec.V-A)
    SamplingRange delta_s_dot;     // velocity keeping용 Δṡi 범위 [m/s] (Sec.V-B)
    double dt;                     // 최종 궤적을 샘플링할 시간 간격 [s]
};

#endif  // FRENET_GENERATOR_CONFIG_HPP
