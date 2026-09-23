# Competition Vehicle Status fixture

`competition_vehicle_status_181.hex` is a synthetic regression packet built
from the statically inspected 152-byte competition vehicle-status payload
contract. It is not a captured competition packet and contains no private
field data.

The fixture proves deterministic parsing, declared-length checks and ROS
publication tests. A live packet capture from the competition simulator is
still required before field acceptance.
