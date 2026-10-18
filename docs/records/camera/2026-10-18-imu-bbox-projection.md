# IMU 공용 기하와 BBox lane 재투영

## 변경 범위

기준은 main `d3cef6b2`다. 작업은 `feat/camera-imu-adapter`에 분리했다.
데이터 이관·학습 분석의 다른 worktree와 실행 중인 카메라 배포본은 변경하지 않았다.

기존 BBox classifier는 family xyz 중 xy만 차로 띠 계산에 사용하고, 고정 YAML 평면의
z를 다시 붙였다. 현재 IMU family에 이 방법을 적용하면 영상 위 경계가 자세 보정과 어긋난다.

현재 경로는 다음과 같다.

1. 공용 IMU timeline과 평면 수학으로 영상 source 시각의 M1 또는 전체 M0를 한 번 선택한다.
2. 기존 family worker가 선택한 뷰별 평면과 원 JPEG header·보정 ID·추론 세대를 발행한다.
3. BBox classifier는 family의 header stamp/frame/config와 이 context를 대응한다.
4. Family xyz가 최신 뷰의 reference plane에 있는지 확인한다. 띠의 xy를 정확히 같은 전방
   영상의 평면 위에 놓고 `K(R·xyz+t)`로 투영한다. 소실선도 동일 평면으로 계산한다.
5. 정상 빈 결과와 UNKNOWN을 구분한다. 보정·영상·상태가 맞지 않으면 다른 평면으로 대체하지 않는다.

Family/BBoxFrame 최상위 Header.seq는 각 publisher가 붙인 독립 값이다. 원 JPEG의 seq는
`CameraBBoxFrame.image.header`와 `CameraGroundProjection.image_headers`에서만 대응한다.
JPEG 대기는8개/250ms, context는8개/350ms이고 최신 완성 쌍을 콜백 도착 즉시 처리한다.
미대응 만료와 더 최신 결과로 인한 대체를 별도 진단한다. 무효 최신 family를 과거 정상 결과로 숨기지 않는다.

## 공용화와 모델별 책임

| 계층 | 책임 |
|---|---|
| `runtime/sensor_timing.py` | source/receive 시간 분리, 원본 신원, session, quaternion SLERP |
| `runtime/ground_projection.py` | IMU 장착 변환·평면 수학·뷰별 시각 선택·all-or-M0; family/model-grid 의존 없음 |
| `bev/imu_projection.py` | semantic 근거의 sparse 재투영, 기존 import 호환 |
| family | 보정된 근거의 역할·폭·joint B-spline; 이 계산은 공용 IMU 코드에 넣지 않음 |
| comparison/BBox adapter | 이미 선택한 평면 재사용; IMU 재조회·이중 회전 없음 |
| YOLO/TwinLite 신경망 | 원래 픽셀·bbox·segmentation 유지; IMU는 지면 투영 단계에서만 사용 |

정본은 `asmc_camera/config/cameras_v3.yaml`, 명목 지면 거리0.36m다.
`projection_context_v1.yaml`은 이 파일의 SHA와 Python CameraCalibration의 per-view ID를 고정하며,
C++ 시작 검사와 Python 계약 시험이 일치를 확인한다. 설정 SHA는
`2d17cf63adbf2ba049d70656233ee2f34e9a4bfb43373ee76b3371aab1e4dc2a`를 유지한다.

GroundProjection의 ID는 평면을 포함한 CameraCalibration 식별자다. 기존 YOLO
CameraObjectObservation의 calibration_id는 원 센서 설정 식별자이며 계산 범위가 다르다.
두 ID를 서로 대입하지 않으며, LiDAR의 기존 관측 계약도 변경하지 않았다.

## 검증 결과

- 격리 catkin 빌드: asmc_msgs·asmc_camera·morai_msgs·lane_boundary_adapter 통과.
- 신규 공용 선택4건과 기존 runtime 회귀를 포함한123건 통과. 임시 workspace의 누락 launch 의존성,
  모델 symlink 경로를 보완한 뒤 해당18건을 재확인했다.
- BEV/IMU·피팅·comparison 회귀210건 통과. 같은 수학을 이동했으며 모델·engine은 변경하지 않았다.
- 신규 C++ 투영8건과 기존 어댑터27건 통과. 서로 다른 front/reference 평면, 잘못된 raw seq,
  설정·보정·세대 불일치, M0·기울기 제한·재시작·상한·중복 stamp를 검사했다.
- 실제 ROS 신규3건과 기존38건 통과. 정적 fixture는 초기 SHA·0.35m·context off를 명시해
  기존 golden 결과를 보존했다. 새 시험은 현행0.36m·M1 context를 사용한다.
- 공개 메시지 필드·MD5는 유지한다. CameraGroundProjection은 시각화와 BBox 재투영의 공유 계약으로 확장했다.
- 중앙 계약12건·토픽 정책·기본 문서/링크 검사·25 launch/63 명령 참조 검사 통과.
  전체 로컬 감사76건 중75건 통과,1건은 기존 main의 미션 metadata authority 상태 검사 오류다.
  변경하지 않은 main metadata로 같은 오류를 재현했으며 이번 IMU 작업에 섞어 수정하지 않았다.
  과거 구조 이전의 일회성 본문 SHA 동결 검사(`--verify-relocation`)는 이후 현행화된 문서에
  적용하지 않는다. 기본 CI 문서 검사는 통과했다.

ROS 시험은 각각 독립 master이며 제어 시험은 기존 시험 토픽만 사용했다. 실제 MORAI 제어,
카메라 모드, 날씨, 실행 중인 모델 및 rqt는 변경하지 않았다.

## 실행 중인 토픽의 읽기 전용 확인

15.02초 구독에서 family225개, context224개, 전방 YOLO224개를 수신했다.
유효 family와 대응한224 context가 모두 M1이었다. YOLO와 동일 전방 영상의 대응은163개,
약10.87Hz였다. 두 모델이 독립적으로 최신 프레임을 선택하므로 모든15Hz 결과가 일치하지는 않는다.

이 수치는 **영상 식별자의 교집합**이며 새 C++ 어댑터의 최종 출력률·지연·주행 수락이 아니다.
출력 source stamp를 바꾸거나 가까운 영상으로 대체해 수치를 높이지 않는다. 실행 시
`projection_matched_frames`, `projection_pending_superseded`, `projection_pending_expired`,
`projection_imu_corrected`, `projection_selection_reason`을 확인한다.

## 남은 경계와 적용

- 평면 선택 함수는 한 카메라만 공급해도 사용할 수 있다. 다만 현재 metadata 생산 주기는 family
  worker에 연결돼 있다. Lane이 꺼져도 작동해야 하는 미래 YOLO 지면 기능에는 별도 소비 연결이 필요하다.
- BBox의 구형 frame에는 YOLO session/generation 필드가 없다. Lane 쪽 세대와 cutoff, 정확한
  원 영상 header·freshness를 검사한다. 모든 YOLO 실행 신원을 검증했다고 주장하지 않는다.
- M2 병진/yaw 정렬·시간 누적·차체 heave는 도입하지 않았다. 최대35ms 뷰 차이와 접점 가림의 한계가 남는다.
- 현재 SHA·v3 보정을 classifier/follower/planner 기본값에 함께 연결했다. SHA 검사·0.36m·기존 제어
  정책은 유지한다. Standalone control=false와 planner가 전달하던 control=true도 그대로다.
- 초기0.35m replay는 old SHA·v2 보정·`projection_context_required:=false`를 함께 선택한다.
  Legacy 경로는 평면에서 벗어난 z를 거부한다. 현재 M0 복구와 초기 replay를 혼동하지 않는다.
- 정차/가감속/조향과 M1↔M0의 실제 RGB 차로 판정, 폐루프 차선변경 인수는
  [판제 체크리스트](../../handoffs/camera-to-planning.md#71-판제-담당의-별도-인수-작업)에 남긴다.
  이번 구현·시험은 실제 차량 제어 승인과 별개다.

구현 검증 시점에는 실행 중인 배포본을 유지했다. 이후 사용자 요청에 따른 재빌드·재시작은 아래에 기록한다.

## 후속 배포·재시작 확인

사용자 승인으로 두 코드 커밋 `71e3d3ce`, `856337dd`를 GitHub의
`feat/camera-imu-adapter`에 push했다. 현재 운영 workspace인
`/data/runtime/camera-twinlite-large-v7-e8-r1`에 카메라·메시지·BBox 어댑터 변경56개 파일을
반영하고 Release로 재빌드했다. 각 파일 SHA와 정본 보정의 manifest 일치를 확인했다.
기존 worktree·컨테이너 마운트·모델·브릿지는 유지했다.

카메라 roslaunch 한 개를 정상 종료한 뒤 같은 기본 설정으로 재시작했다. YOLO·TwinLite·ResNet은
모두 새 producer session이며 ACTIVE·loaded·pipeline_ready, 오류0을 확인했다.
공용 함수의 실제 import는 `asmc_camera.runtime.ground_projection`이고 metadata MD5도 유지된다.

재시작 후12.01초의 읽기 전용 확인 결과는 다음과 같다.

| 결과 | 수신 수 | 평균 공급 | 최대 수신 공백 |
|---|---:|---:|---:|
| 전방 YOLO | 180 | 14.98Hz | 138.7ms |
| 좌측 YOLO | 180 | 14.98Hz | 141.4ms |
| 우측 YOLO | 180 | 14.98Hz | 141.4ms |
| 유효 family | 179 | 14.90Hz | 168.0ms |
| 공유 투영 context | 180 | 14.98Hz | 141.5ms |

Context180개 모두 M1이었다. 이는 재시작 직후의 짧은 동작 확인이며 장기/폐루프 수락을
다시 수행한 결과가 아니다. 실제 배포 workspace의 공용 선택4건·기존 IMU 연결4건 및
독립 master의 BBox 투영 ROS3건도 통과했다.

기존에 켜져 있던 카메라·family만 재시작했다. 시각화·BBox 제어·판제 노드는 새로 시작하지 않았다.
시각화는 기존 `view.launch`와 별도 rqt 명령으로 열 수 있다. BBox 소비는 새로 빌드된 패키지를
source하고 실행할 때 반영된다. Planner launch 변경은 Git 브랜치에 있으며 판제 실행 전
해당 checkout 반영·빌드는 별도다. 결과/상태별 단일 생산자와 시험 노드 종료를 확인했다.

변경 전 소스·상태·환경, 배포 파일 SHA, 빌드·ROS 시험 로그와 재시작 확인은
`/data/runtime/camera-imu-adapter-20261018-r1/`의 `before.json`, `source-before.tar`,
`provenance.json`, `build.log`, `running.json`, `after.json`, `graph-final.json`에 보존했다.
