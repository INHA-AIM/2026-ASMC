# YOLO11m COCO 클래스 확대 증거

- `parity.json`: mask OFF 기존 경로 3,514프레임 정확 회귀.
- `offline/`: 같은 masked 입력의 카메라/주/음영 bag 및 GT287/GT39 전체 결과 JSONL, 요약, 차이 목록.
- `offline/gallery/index.html`: 새 라벨별 고정 표본을 브라우저에서 열기. `manual-review.json`은 영상 검토이고 독립 GT 점수가 아니다.
- `matrix/`: 20회 고정 프로토콜·manifest·반복별 report·합산/안정된 공통 객체 비교.
- `matrix-raw.tar.gz`: matrix-v2의 모든 원본 입력/출력 관측, resource 200ms 표본, launch/bag/Main 로그와 report. 원본 bags/weights는 별도이며 SHA는 manifest에 있다.
- `attempts/`: 초기 잘못된 Msg source, vendor PCAP 입력 부재/timeout, 실행기 상대 경로 오류를 보존. 성공 측정으로 덮어쓰지 않았다.
- `provenance.json`: 실행 HEAD/컨테이너/설정과 파일 SHA. 비교 운영 코드 변경 없이 최종 문서·증거 커밋을 추가한다.

`matrix` 집계의 완료와 80종 운영 승격은 다르다. 기본 네 클래스와 어댑터, confidence, 모델, semantic mask, 메시지 MD5는 유지한다. 기반 Main 경계/roll은 OFF다.

주 bag 매칭 baseline 646/647, all 646/646으로 반복 하한은 유지하지만 최고 기준선과는 1쌍 차이가 있다. 안정된 공통 관측의 신규 분류 차이는 0이며 미대응 관측은 성공으로 계산하지 않았다. matched는 프레임 쌍 수이고 고유 객체/추적 recall이 아니다. 음영 bag에 새 검출은 없어 클래스 확장만으로 장애물 문제가 해결되지 않았다.

문서·사용법: `docs/handoffs/perception/camera-coco-class-expansion.md`.
