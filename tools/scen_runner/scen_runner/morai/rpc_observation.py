"""Record lifecycle RPCs without adding queries or changing their requests."""
from datetime import datetime, timezone
import time


METHODS = frozenset(('SpawnVehicle', 'SpawnPedestrian', 'SpawnObstacle', 'DestroyActor',
    'CreateVehicleSpawnPoint', 'EnableSpawnPoint', 'SetPedestrianWaypoint',
    'SetVehicleRoute', 'SetVehicleDestination', 'SetVehicleDynamicsSpeedLimit',
    'SetPause', 'SetTransform', 'SetVelocity', 'SetVehicleGear', 'GetOptionName'))


def message_record(value):
    from google.protobuf.json_format import MessageToDict
    record = MessageToDict(value, preserving_proto_field_name=True)
    # A failure status is zero and would otherwise be omitted by proto3 JSON.
    if hasattr(value, 'status'):
        record.update(status=int(value.status), description=value.description,
                      custom_message=value.custom_message)
    return record


class ObservedStub:
    def __init__(self, stub, observer, service):
        self.stub, self.observer, self.service = stub, observer, service

    def __getattr__(self, name):
        original = getattr(self.stub, name)
        if name not in METHODS:
            return original

        def call(request, *args, **kwargs):
            record = dict(service=self.service, method=name, request=message_record(request),
                          started_utc=datetime.now(timezone.utc).isoformat(),
                          timeout_s=kwargs.get('timeout'))
            started = time.monotonic()
            try:
                response = original(request, *args, **kwargs)
            except Exception as error:
                record.update(rpc_s=time.monotonic()-started, rpc_error=str(error))
                self.observer(record)
                raise
            record.update(rpc_s=time.monotonic()-started, response=message_record(response))
            self.observer(record)
            return response
        return call


def observe_link(link, observer):
    adapter = link._require()
    for name, service in (('_actor_stub', 'actor'), ('_scenario_stub', 'scenario')):
        stub = getattr(adapter, name)
        if isinstance(stub, ObservedStub):
            stub.observer = observer
        else:
            setattr(adapter, name, ObservedStub(stub, observer, service))
