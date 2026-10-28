# 로컬 GPU 엔진 선택·등록 검증 — 2026-10-29

main `be20138a`의 E3 배포 설정에서 직접 `camera.launch`를 실행하면 시나리오 실행기의
로컬 엔진 선택이 적용되지 않았다. 또 엔진을 데이터 드라이브에 두고 작은 실행 프로필을
WSL checkout에 두면, 생성·합성 검사 후 마지막 `os.replace`가 `EXDEV`로 실패했다.

로컬 커밋 `51c9672a`에서 직접 실행도 같은 원본 SHA의 검증된 객체 엔진을 선택하고,
프로필 교체 임시 파일을 **대상 디렉터리**에 만들도록 수정했다. 로컬 캐시 검증 실패 시
PyTorch로 전환하거나 배포 engine·SHA를 덮어쓰지 않는다.

후속 `4d3bd82d`는 시나리오의 `camera_runtime: off`를 backend에 전달하고, 시나리오가
이미 선택한 엔진도 GPU·TensorRT·head를 재검사한다. 명시한 프로필이 없으면 오류를
알리며 상대 경로는 `tools/scen_runner/`로 통일했다. 새 CPU 회귀를 camera CI 그룹에 등록했다.

## 검증 근거와 범위

[기계 판독 요약](../../../config/baselines/evidence/camera_local_engine_selection_v1/summary.json)에
소스 revision·엔진 SHA·이전 live 측정과 현재 회귀를 구분했다. 원본 로그·캐시·개인 경로와
생성 engine은 커밋하지 않았다.

- 현재 회귀: 객체 선택11, 차선 선택10, 생성·원자 갱신5, 시나리오 프로필7,
  detection90, runtime124 — 합계247개 통과. 실패·GPU 오류 주입 시험의 로그를 실제 GPU 장애로 해석하지 않는다.
- CI 실행기 자체 시험8개, 문서 검사 시험17개 통과. 문서 링크·현행 launch 안내·기존 CI145개 검사 보존 점검 통과.
- 기존 실제 RTX4060Ti·TensorRT10.13.0.35 캐시도 새 선택 코드로 읽기 전용 검증했다.
  선택된 engine SHA는 `96694250e0f358f1b0d5ebfdd136b86fecf13761d2f995bbabd49fc371e17e73`이다.

앞선 `51c9672a` 실제 실행에서는 camera·LiDAR·autonomy launch를 함께 기동하고,
A126 음영→일반도로 시작점→A126 복구에서 각각15초간 토픽을 확인했다. 차량 제어 UDP만
임시 격리했다. 차선 family 약15Hz, LiDAR local 약12Hz, 정상 GPS에서 유효 map 차량 상태
50Hz·trajectory/제어 약20Hz였다. 객체 관측 약43~45Hz는 **3뷰 합계**다.
음영에서 local 검출이 유지되고 map 가용성이 제한되며 정상 위치에서 복구됐다.
순간이동 두 번의 전환을 포함해 LiDAR invalid30건이 누적됐고 정차 관측은 다시 valid였다.

이 기록은 전 구간 폐루프 주행·객체 정확도·15Hz LiDAR 입력 부하 수락이 아니다.
후속 프로필 정책 변경 후에는 CPU 회귀와 실제 캐시 판독을 확인했으며, 앞선 live 시험을
새 연속 주행 결과로 재사용하지 않는다. 실행 중인 사용자 bridge는 종료·재생성하지 않았다.

## 문서 정합성과 적용

카메라 README·배포 안내·데이터 준비·판제 인계·ownership의 현재 기본값을
E3 15종으로 맞췄다. E8 16종과 기존 실험 수치·출처는 복구·재현 기록으로 보존했다.
직접 실행/시나리오의 선택 차이, 실제 `model_id`, 해제 옵션과 저장 위치는
[GPU별 엔진 안내](../../operations/local-lane-engine.md)를 따른다.

배포 모델·메시지·토픽·판제 코드·센서·네트워크 설정은 변경하지 않았다.
개인 데이터 저장 위치와 LOCAL·AIMDrive 실험 코드도 배포 범위에 포함하지 않는다.
같은 모델의 로컬 선택 수정이므로 배포 checkpoint·engine을 다시 내려받을 필요는 없다.
사용 중인 카메라 노드는 소스 업데이트만으로 바뀌지 않으므로 다음 실행 때 재시작한다.
