# 2026-10-27 F1 후속 연구 근거 선별 통합

**이번 기록은 과거 결과의 선별 이관과 후속 책임 정리다. 새 모델 실행·주행·3뷰 matching 개선 결과가 아니다.**
PR #118은 main `22577363340c83c8a69ef64b55adb1fd678bbee9`에 병합됐다. 사용자 결정에 따라
main의 변경은 유지하고 되돌리기 PR #120은 미병합 종료했다. 후속 PR은 이 main에서 시작하며
revert/reapply 커밋을 포함하지 않는다. F1 검증과 제한은 [원래 기록](2026-10-27-front-fusion-verification.md)을 유지한다.

## 보존과 선별

원본 연구 브랜치 `research/lidar-perception-20261018`의 HEAD는
`a296c0c4f14c09ff6c3f9b16372d809dc0d79b82`다. 로컬 보존 태그
`preserve/research-lidar-20261027`과 F1 태그 `preserve/pr118-reviewed-head`를 남겼다.
개발 checkout은 `2026-ASMC-work`의 `docs/lidar-baselines-and-handoff`로 통일하고,
`2026-ASMC`는 main 참조 checkout으로 유지한다. 연구 브랜치 전체를 merge하지 않는다.
실행 중인 사용자 bridge/master는 유지했다. 현재 ROS 컨테이너는 main checkout을 사용하며
양쪽 운영 소스는 동일하다. 로컬 저장 설정의 다음 ROS workspace만 work로 지정하고 compose의
host 네트워크·데이터 마운트는 유지했다. 컨테이너 재생성/재빌드를 이번 문서 작업의 완료로 주장하지 않는다.

| 원본 | 이번 선택 | 제외·이유 |
|---|---|---|
| `4da2a5b06039e5bc4226f1a8c24291493c25e9a4` | 10/18 E8/seg/PIDNet·PCL·추적 조사 갤러리/요약6개 | 옛 projection/nearest matching·무제한 all-pairs support·결과 덮어쓰기 실행기는 현 운영에 연결하지 않음 |
| `ee97ffc52164c87e66cfeefc90a8134964c5075e` | HARP/CenterPoint·seg 라벨 감사 갤러리/요약9개 | 외부 모델 의존성·GPU runner·개인 입력 경로·옛 operational 안내는 고정 revision에서만 재현 |
| `010d1436`, `a296c0c4` | 원본 브랜치에 보존 | 과거 LAN backport는 최신 main의 센서/네트워크 정본과 섞지 않음 |

총15개 파일·1,698,921bytes를 원본 blob과 바이트 동일하게 가져왔다.
[전체 manifest](../../../config/baselines/lidar_research_import_v1.json)에 원본 commit/path/blob,
현재 path/크기/SHA256와 제외한 실행기 목록을 기록했다. SHA 검사는 이 동결 근거에 한정한다.
현행 설계·인계 문서를 SHA로 동결하지 않는다. 외부 모델·bag·원본 학습 데이터는 복사하지 않는다.

- [고전/영상 후보 갤러리](../../../config/baselines/evidence/lidar_fusion_shadow_v1/index.html)
- [학습 후보/라벨 근거 갤러리](../../../config/baselines/evidence/lidar_learned_candidates_v1/index.html)
- [원본 연구 설명](https://github.com/INHAautonav/2026-ASMC/blob/ee97ffc52164c87e66cfeefc90a8134964c5075e/docs/records/camera/camera-lidar-fusion-study.md)

개인 연구 문서 `lidar-classical-baselines-2026-10.md`, `lidar-competition-next-step-2026-10-27.md`,
`lidar-perception-papers-2026-10.md`에서 합의된 후보와 적용 조건만
[팀 개선 계획](../../../src/asmc_lidar/docs/improvement-milestones.md)에 통합했다. 개인 문서를 실행 정본으로 복제하지 않는다.

## 수치를 해석할 때

10/18 연구는 동일 개발 bag208scan과 모델별624영상, RTX4060Ti8GB 환경의 한정 대조다.
독립 객체/semantic/속도 GT가 없고, 학습 후보는 다른 작업과 GPU를 공유했다. 4070SUPER나
현재 전체 파이프라인의 실시간 성능 수락 수치로 사용하지 않는다.

| 근거 | 관측·한계 | 후속 의미 |
|---|---|---|
| E8 box+geometry | box가 있는 프레임의 geometry p95 약7.19ms. point 지지는 있으나 선택 불가42건, raw 지지 없음54건 | unknown 유지·탐색 상한 필요. box 검출만으로 3D 복구했다고 볼 수 없음 |
| scan64 | box 안38점의 넓은 깊이 분포에서 선택 성분20점으로 축소 | 배경 혼입을 줄일 가능성. 원본 index/깊이 후보이며 point GT가 아님 |
| scan112 | E8 box는 있으나 raw 지지가 없는 사례 | segmentation이나 카메라 분류로 없는 LiDAR 깊이를 생성하지 않음 |
| PCL normal+plane | 준비 비용 제외 p95 약38.51ms | 유한 표면/중력·실물 보존 없이 평면 일괄 삭제를 채택하지 않음 |
| 과거 KF ID76 | 내부 상대속도58.34m/s·miss24 | 현재 판제 속도 오류로 재해석하지 않음. 새 코어에서 위치/연관/miss를 재현할 출발 사례 |
| HARP nuScenes / SemanticKITTI | 보고 total p95 약148.47 / 189.51ms, scan64 작은 box 점의 도메인 오분류 | 구조물 제거의 정답 모델로 간주하지 않음 |
| CenterPoint-Pillar | p95 약168.08ms,10sweep 학습 모델에 single scan, velocity 무효 | 경쟁 주행 기본 교체 근거가 아님 |
| seg 라벨 감사 | train 전체 mask 검증이 아니라48개 표본 확인. 완전 양성+mask metadata train186장 | detect 납품을 막지 않고 seg는 별도 GT/실행 pilot로 분리 |

옛 실행은 offline nearest image(나중 도착한 영상 포함 가능), raw 입력·첫 원본점 voxel 대표,
모델별 정밀도·크기·class가 서로 달랐다. 현 F1의 causal 선택·native 투영·생산 voxel과 별개다.
과거 JSON/HTML의 당시 기본값·다음 계획은 이력을 보존한 것이며 현재 운영 안내는 아니다.

## 읽기 전용 검증 도구

저장소 루트에서 실행한다. 표준 Python만 필요하며 모델/ROS/개인 데이터 다운로드·실행·파일 쓰기가 없다.

```bash
python3 tools/perception/review_lidar_research.py
python3 tools/perception/review_lidar_research.py --verify-git
python3 tools/perception/test_lidar_research_review.py
```

기본 명령은15개 근거의 크기/SHA를 모두 검증한 뒤 JSON 요약과 한계를 stdout으로 출력한다.
`--verify-git`는 보존 연구 history가 있을 때 원본 commit/blob/바이트까지 대조한다. shallow CI에는
원본 history가 없을 수 있으므로 기본 파일 검사와6개 CPU 회귀만 기존 LiDAR 검사 그룹에 포함한다.
별도 job·GPU·외부 모델·대용량 bag 의존성은 추가하지 않는다.

이번 검사: 원본15개 blob/바이트 일치, 누락·변조·중복·manifest 밖 summary·경로 이탈 회귀 통과.
문서 회귀17개, 문서/anchor·동결5개/history 검사, 실행 문서26 launch·89명령,
LiDAR 정본 의존성, CI 기존145개 검사 보존 확인도 통과했다. Hosted CI는 PR 결과로 별도 확인한다.
운영 코드·launch·메시지·모델·지도
변경이 없으므로 새 MORAI 수락으로 표시하지 않는다.

## 다음 연결

역할·선행 조건·완료 기준은 [M0–M5 정본](../../../src/asmc_lidar/docs/improvement-milestones.md)을 따른다.
승현의 다음 구현은 M1 3뷰 fusion이며, 필요한 M0 사례 연결만 포함한다. 재호의 PR #119 통합·
터널 수락과 M3→M4는 별도 PR로 진행한다. 같은 runtime/trace/공용 계약을 바꿀 때 서로의
관측 식별·raw support·속도 validity 회귀를 함께 확인한다.
검토 중 PR #119는 `90415a11`에서 F1 main 병합을 완료했다. 미병합이며 통합 후 회귀·실시간 수락은 별도다.
