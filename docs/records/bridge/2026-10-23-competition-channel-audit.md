# Competition 입력 채널 실측과 LiDAR 이관 재정렬

2026-10-23(사용자 기준 날짜). 사용자 화면의 **Competition Vehicle Status(Sim → User),
CompetitionInfoPublisher, Host9088/Destination9099**와 현재 송신을 대조했다.
규정 원문 URL은 이번에도 도구에서 접근되지 않았다. workspace 보존 v1.2 사본 2-4·2-8을
근거로 하며 최신 원격 본문 전체를 확인했다고 주장하지 않는다.

## 실측

컨테이너 `asmc-ros-noetic`의 ROS master/bridge는 실행되지 않았다. 다른 UDP listener가 없는
9099·9798을 SO_REUSEADDR 없이 8.024초간 임시 수신했다. ROS 노드·제어 송신·시뮬 설정 변경은 없다.
현재 bridge의 `decode_vehicle_status_packet`과 실제 `publish_vehicle_status` 함수를 호출하되,
publisher는 메모리 수집 객체로 대체했다. ROS 네트워크로 발행·전달한 시험은 아니다.

| 항목 | 9099 | 9798 |
|---|---|---|
| 수신 | 323개, 40.25Hz | 322개, 40.13Hz |
| header / 전체 길이 | `#MoraiInfo$` / 181B | `#DetailEgoInfo$` / 138B |
| body | 152B | 형식상105B |
| 기존 decoder | 323/323 성공 | 0개, header mismatch로 거부 |
| 현재 역할 판단 | 사용자 UI와 함께 CompetitionInfoPublisher로 확인 | 별도 Detail 형식. 허용 Competition 속도 입력으로 전환하지 않음 |

9099 모든 표본에서 position XYZ, velocity Y/Z, acceleration XYZ는 0이었다. 이는 제공되지
않는 필드의 자리이며 **측정된 위치·횡속도·가속도**가 아니다. signed speed는
0.01791–0.07274km/h, velocity.x와의 최대 차이는 1.5e-8km/h 미만이었다.
기존 bridge는 velocity.x를 3.6으로 나눠 m/s로 채운다. 거의 정차한 표본이므로 이 검사는
필드 대응·변환 코드 확인이며 주행 속도 정확도·후진 부호·프레임 정의의 독립 검증이 아니다.

저장된 MolitComp local JSON(레포 생성본과 Windows SaveFile 사본)은 type295를9088→9099,
type787을9797→9798에 둔다. 저장 파일이 현재 UI 전체 설정과 동일하다고 입증하지 않았으며,
해당 번호의 의미를 모든 MORAI 버전에 일반화하지 않는다. UI publisher 이름+실제 endpoint+
wire 형식으로 식별한다. `#MoraiInfo$` header만으로 허용 여부를 판정하지 않는다.

## 기존 판단 정정과 유지할 경계

- `/Ego_topic`은 팀 ROS 호환 이름이다. 이번 설정에서는 허용 Competition 정보를 운반한다.
  `EgoVehicleStatus` 메시지 타입을 쓴다는 이유만으로 판정 전용 정보를 소비한다고 판단하지 않는다.
- 기존 Main의 `/Competition_topic → /Ego_topic` remap과 shadow의 `/Ego_topic` 구독을 유지한다.
  재호가 새 금지 입력을 넣었다는 판단은 철회한다.
- 9798만 Competition이라고 설명하거나 `--require-competition-status`를 모든 본선의
  필수 gate로 안내한 문구를 정정한다. 이 옵션은 현재 구현상 별도9798 경로를 검사한다.
  현재 네트워크에서 켜면 정상9099 Competition 송신과 무관하게 실패할 수 있다.
- 위치는 GPS/IMU에서 얻는다. 실제 판정 전용 Ego/Detail 채널, Object GT를 운영 fallback으로 쓰지 않는다.
- shadow는 XYZ norm으로 속도를 만들고 음수 속도를 거부한다. 현재 Y/Z=0·양의 속도에서는
  abs(x)와 같지만 부호를 잃는다. L3에서 허용 longitudinal 필드·부호·단위를 명시하고
  정지/전진/후진 시험으로 확인한다. 이번에는 decoder나 런타임을 수정하지 않았다.

## 재현 근거

run ID: `competition-channel-audit-20261023-r1`.
D 정본 상대경로: `artifacts/competition-channel-audit-20261023-r1`.
`summary.json`, `decoded.jsonl`, 첫 packet 표본 `packets.json`, `probe.py`를 보존한다.
run 등록 입력의 network snapshot은 과거 비교 자료이며 현재 로드 설정의 증명으로 쓰지 않는다.
Git에는 [작은 요약 manifest](../../../config/baselines/competition_channel_audit_20261023.yaml)만 둔다.
센서 timestamp가 사용자 날짜와 다를 수 있으므로 파일명 날짜를 source stamp에서 추정하지 않는다.

## PR #103 반영

최신 main `281349da` 위로 로컬 L0/L1 문서 커밋을 rebase했다. 이전 tip `c3629824`는
`backup/lidar-l1-pre-rebase-20261023`에 보존했다. 충돌은 최신 shadow 구현·검증 문구와
L0–L8 책임/계획을 함께 유지하도록 해결했다. 런타임·launch·모델·메시지는 바꾸지 않았다.

[L1 설계](../../../src/asmc_lidar/docs/migration-design.md)와
[작업표](../../../src/asmc_lidar/docs/migration-tasks.md)가 현재 판단의 정본이다.
2026-10-22 L1 manifest의 소스/설정/자료 SHA와 당시 해석은 과거 기록으로 보존하며,
이번 정정은 새 manifest와 문서에 기록한다. 기존 raw bag의 `/Ego_topic` 이름만으로
당시 송신 종류를 소급 확정하지 않는다.

## 이번 변경 검증

- 문서 migration/history·동결5개·runtime 문서(launch25개·literal command63개)·공용 계약 검사 통과.
- 문서/계약 Python27개 + 카메라/LiDAR 계약5개, 총32개 통과.
- 변경 문서의 상대 파일 링크(URL decoding 포함)·diff 공백 검사 통과.
- L1 기존 manifest는 backup tip과 byte 동일. main 대비 실행 소스·launch·공용 메시지·nested Main diff0.
- 임시 UDP 수신 소켓·프로세스 종료 확인, run 완료 동결. ROS 운영 노드 시작/종료 없음.
- PR #103 HEAD의 [CI](https://github.com/INHAautonav/2026-ASMC/actions/runs/39747184038)와
  [Planning integration](https://github.com/INHAautonav/2026-ASMC/actions/runs/39747184015) 성공을 조회했다.
  이는 상위 PR 근거이며 이번 문서 커밋의 hosted 실행 결과가 아니다. push/PR/병합은 하지 않는다.
