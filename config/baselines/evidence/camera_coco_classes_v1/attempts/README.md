# 보존한 실패 실행

- `outer-missing-msg.log`: nested Msg를 source하지 않은 첫 전체 회귀. 환경 수정 뒤 jamming 94개 통과, 바깥 537개 중 기존 facade 1개만 남음.
- `nested-vendor-timeout.log`: 범위를 넓힌 vendor 시험에서 18개 실패. 시험 launch가 요구하는 class/32e/vlp16 PCAP가 checkout에 없고 일부 driver가 종료해 hz/diagnostic timeout이 발생했다. Main 296개는 통과했다. vendor 코드 변경은 없으며 전체 원인을 새 카메라 코드의 회귀나 수락 예외로 처리하지 않는다.
- `matrix-v1-relative-path.log`: 첫 반복 시작 이전 fingerprint 상대 경로 오류. 실행기만 수정했으며 v2 전체 프로토콜이 원래 실패를 대체하지 않는다.

실제 측정 중 발생한 실패는 matrix의 각 report와 원본 archive에 별도로 보존한다.
