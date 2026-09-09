# D7: Route/FSM 쪽이 결정한 새 전역 경로를 Frenet에 전달하는 계약(§19).
#
# Frenet은 이 메시지를 수신 -> validate -> pending reference로 보관 -> 안전한
# boundary에서 적용까지만 한다. target_link_id/link_ids를 이용해 MGeo topology를
# 스스로 탐색하거나 route를 새로 결정하지 않는다(§19-20) - 그 책임은 Route
# Manager/FSM에 있다.

std_msgs/Header header

# 이 GlobalPath가 몇 번째 갱신인지. 같은 revision을 재발행해도 안전하게
# 무시할 수 있도록(idempotent) 소비자가 direction/변경 여부를 판단하는 데
# 쓴다.
uint32 revision

# 이 경로가 속한/향하는 MGeo link. Frenet은 이 값을 스스로 탐색하지 않고
# 그대로 보존/echo한다 - 소유권은 Route Manager에 있다.
string active_link_id
string[] link_ids

# 경로 waypoints (map frame). Frenet의 BuildRefLine/ReplaceReferenceLine이
# 그대로 소비할 수 있는 x/y 시퀀스다.
geometry_msgs/Point[] waypoints
