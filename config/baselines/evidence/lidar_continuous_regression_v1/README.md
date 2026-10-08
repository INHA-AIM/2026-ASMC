# PR #72 연속 처리 증거

[최종 baseline](../../perception_lidar_continuous_regression_v1.yaml)과
[검증·인계 설명](../../../../docs/handoffs/perception/lidar-continuous-regression.md)을 함께 읽는다.
전체 실행·실패를 보존하며 운영 수락은 보류다. runtime C++는 `08dbb79`, 분석/시험 도구는
별도 커밋이다. 원본 manifest의 잘못된 CLI 라벨은 [정정 근거](provenance-correction.json)에 기록했다.

| 파일 | 내용 |
|---|---|
| `cpu-failed-attempt-v1.tar.gz` | tmpfs 부족으로 중단한 첫 실행의 완료/부분 상태·로그·index |
| `cpu-attempt-v2.tar.gz` | JPEG 입력이 누락됐던 20회; 클래스 수락에서 제외 |
| `cpu-final-v3-primary.tar.gz`, `cpu-final-v3-shadow.tar.gz` | 고정 JPEG/YOLO 입력을 사용한 20회 전체 상태·종료 기록·추적 진단; 공통 manifest는 중복 포함 |
| `realtime-all-18.tar.gz` | 전체 bag 18회 원본 observations·launch·매개변수·200ms 자원 기록·실패/종료 집계 |
| `synthetic-and-fixed-cases.tar.gz` | 합성 3시도(실패 포함), truth/bag·실제 처리 trace, 사전 선정 6 scan의 점 대응과 진단 그림·회귀 로그 |
| `final-checks.tar.gz` | 수정 환경으로 재실행한 HD맵·최종 회귀 로그와 기존 history 문서 오류 |
| [CPU 분석](cpu-analysis.json) | 반복 6쌍·GPS 대조 8쌍; 모든 상태를 정확 비교 |
| [roll 분석](roll-analysis.json) | 실제 점 대응 역변환의 상호 유일 중심 대응, 신규 분류 차이·미대응·속도 급변 |
| [고정 관측 검토](fixed-case-review.json) | 기존 B1a 6관측과 같은 source ns; 독립 GT 없음 |
| [압축본 SHA](archives.json) | portable 파일 SHA/크기 |
| [전체 raw 목록](raw-manifest.json) | 호스트 `artifacts/lidar-pr72`의 파일별 SHA/크기 |

큰 전체 voxel trace와 첫 실패의 부분 trace는 호스트 `artifacts/lidar-pr72`에 보존한다.
CPU portable trace에서는 `ground_points`만 생략하며 연속 전체 객체/KF/분류/매칭 기록은
유지한다. 사전 선정 6 scan의 full ground 대응은 portable 합성/사례 archive에도 들어 있다.
전체 point trace가 Git archive에 포함됐다고 주장하지 않는다. raw 목록은 생성 시점의
snapshot이며 이후 추가 분석이나 회귀 로그는 최종 baseline/portable 기록에서 확인한다.

최초 realtime report의 지연 표본 0은 분석의 ms/ns anchor 오류다. 원본을 덮어쓰지 않고
같은 observations의 original ns로 `analysis.json`을 재생성했다. observer 지연과 실제 Main
callback receipt는 다른 지표다. frozen YOLO 재생을 새 GPU 추론·전체 카메라 부하로 해석하지 않는다.

합성 첫 16프레임만 직진·자세 변화 없는 속도 truth다. 낮은 물체·부분 관측·군집 합침의
미대응은 모두 남겼다. 실제 bag의 객체 보존·속도 정확도는 독립 GT 수락이 아니다.
