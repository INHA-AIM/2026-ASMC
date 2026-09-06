#ifndef FRENET_LEADER_SELECTOR_HPP
#define FRENET_LEADER_SELECTOR_HPP

#include <vector>

#include "frenet_planner/Reference/Reference.hpp"
#include "frenet_planner/global/data_logger.hpp"
#include "frenet_planner/global/global.hpp"

// 장애물 하나를 참조선 기준 Frenet 좌표로 투영한다.
// out_s: 장애물의 s(참조선 arc length), out_d: 장애물의 lateral offset,
// out_s_dot: 참조선 진행 방향으로 투영한 장애물의 종방향 속도(식(7) 역산 -
// 장애물 heading이 참조선과 어긋나 있으면 raw speed와 다를 수 있음).
// 탐색 범위 제한 없이 항상 전체 참조선에서 최근접점을 찾는다(장애물은
// ego처럼 매 사이클 위치가 조금씩만 바뀐다는 연속성 가정을 할 수 없고,
// Following 탐색 거리(수십 m)가 FindClosestS의 기본 hint window(30m)보다
// 클 수 있어 hint를 안 쓰는 게 안전하다).
void ProjectObjectToFrenet(const RefLine& ref, const TrackedObject& obj,
                           double& out_s, double& out_d, double& out_s_dot);

#endif
