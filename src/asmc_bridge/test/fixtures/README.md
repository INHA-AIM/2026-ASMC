# Competition Vehicle Status fixture

`competition_vehicle_status_181.hex` is a synthetic regression packet built
from the statically inspected 152-byte competition vehicle-status payload
contract. It is not a captured competition packet and contains no private
field data.

The fixture proves deterministic parsing, declared-length checks and ROS
publication tests. A live packet capture from the competition simulator is
still required before field acceptance.


## Measured MolitComp03 regression samples (2026-10-23)

`measured_competition_9099_181.hex` and `measured_detail_9798_138.hex` come from the frozen
competition-channel-audit-20261023-r1 packets.json (SHA256
33eca123d9e547d95dcf22fc6ff633d9dd052426ae92688772139bac8bb11c8b).
The former is the UI-verified CompetitionInfoPublisher sample; the latter must not satisfy
Competition acceptance. Header or legacy ROS topic names alone do not prove permission.
The older static fixture above remains a generic decoder-structure regression.
