#pragma once

// trajectory reference의 시간 정렬 - Commit E: message_converter(ROS message
// <-> core datatype 변환)에서 분리해 Reference/timing 책임으로 옮겼다.
//
// MPC는 두 개의 독립된 시간 개념을 쓴다:
//   SOURCE TIME   message.header.stamp. trajectory/vehicle_state가 실제로
//                 표현하는 planning/측정 시각(interfaces/README.md 4절).
//                 이 파일의 계산 대상이다.
//   RECEIVE TIME  ros::SteadyTime::now(). stale/missing watchdog 전용
//                 (message_converter.hpp의 IsMessageFresh).
// 이 둘을 한 식에서 섞지 않는다 - source time으로 latency를 보정하는 동안
// receive time으로 freshness를 판정하는 계층은 그대로 분리된다.
//
// ros::Time에 의존하지 않는다 - 초 단위 double만 다루므로 ROS 없이도
// 단위 테스트할 수 있다. ros::Time -> double 변환은 message_converter.cpp가
// 담당한다.
double ComputeReferenceStartOffset(double trajectory_source_time_s,
                                   double vehicle_source_time_s);
