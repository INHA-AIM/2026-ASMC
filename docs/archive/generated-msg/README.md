# 과거 generated message export

> **상태:** Historical generated copies — 현재 message 정본으로 사용 금지
>
> **현재 정본:** `src/asmc_msgs/msg/BehaviorCommand.msg`,
> `src/asmc_msgs/msg/FrenetFeedback.msg`

이 디렉터리의 두 파일은 외부 검토용으로 만들었던 filename 포함 CRLF 텍스트 사본이다.
ROS build나 MD5 계산의 입력으로 사용하지 않는다.

## D2 parity 감사 — 2026-09-22

| export | archive SHA-256 | current source SHA-256 | 정규화 결과 |
|---|---|---|---|
| `BehaviorCommand.msg.txt` | `8146606c640fd089e71b1348cb806d7aca1e9fb2f19dbd2af8d1b7290da17274` | `4d1eae77eceeb23ce4e6453340e57a7e44d5b96251af88de391c9ad64e493d5b` | 동일 |
| `FrenetFeedback.msg.txt` | `caac9cd7686e17b6b58ada9796994dbbf3ff00caf35d2862c16f36974810ae8b` | `e68970875d9120c1d845de237b8b9f488b8d6f20f11de54d02291061c51b400f` | 동일 |

정규화는 첫 filename 행 제거, CRLF→LF와 마지막 newline 차이 무시만 수행했다. 필드·상수·주석
본문 차이는 없었다. 서로 다른 파일 SHA는 wrapper와 줄바꿈 형식 때문이며 message 계약 차이를
뜻하지 않는다.

외부 소비자가 없음을 확인한 뒤 D3에서 삭제할 수 있다. 그 전에도 수정은 항상 current `.msg`에
먼저 하고 ROS1 MD5·consumer 영향을 검토한다.
