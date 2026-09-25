# -*- coding: utf-8 -*-
"""Best-effort read-only MORAI environment metadata client."""

from __future__ import print_function

import sys

from data_collection.capture_grpc import CaptureGrpcError


WEATHER_NAMES = {
    0: "sunny",
    1: "cloudy",
    2: "foggy",
    3: "storm",
    4: "rainy",
    5: "snowy",
}


class MoraiEnvironmentClient(object):
    def __init__(self, address, port, proto_root, rpc_timeout_sec=1.0):
        if proto_root not in sys.path:
            sys.path.insert(0, proto_root)
        try:
            import grpc
            from morai.common.type_pb2 import Empty
            from morai.environment.environment_pb2_grpc import EnvironmentStub
            from morai.simulator.simulator_pb2_grpc import SimulatorStub
        except ImportError as exc:
            raise CaptureGrpcError("MORAI environment gRPC import failed: {}".format(exc))
        self._grpc = grpc
        self._empty = Empty
        self._timeout = float(rpc_timeout_sec)
        self.target = "{}:{}".format(address, int(port))
        self._channel = grpc.insecure_channel(self.target)
        self._environment = EnvironmentStub(self._channel)
        self._simulator = SimulatorStub(self._channel)

    def wait_ready(self, timeout_sec):
        self._grpc.channel_ready_future(self._channel).result(timeout=float(timeout_sec))

    def snapshot(self):
        empty = self._empty()
        result = {"grpc_target": self.target}
        errors = {}
        try:
            value = self._environment.GetTime(empty, timeout=self._timeout)
            result["sim_hour"] = int(value.hour)
        except Exception as exc:
            result["sim_hour"] = None
            errors["time"] = "{}: {}".format(type(exc).__name__, exc)
        try:
            value = self._environment.GetWeather(empty, timeout=self._timeout)
            result["weather_code"] = int(value.type)
            result["weather"] = WEATHER_NAMES.get(int(value.type), "unknown")
        except Exception as exc:
            result["weather"] = "unknown"
            errors["weather"] = "{}: {}".format(type(exc).__name__, exc)
        try:
            value = self._simulator.GetTimestamp(empty, timeout=self._timeout)
            result["morai_timestamp_raw"] = int(value.value)
            result["morai_timestamp_unit"] = "us"
        except Exception as exc:
            result["morai_timestamp_raw"] = None
            errors["timestamp"] = "{}: {}".format(type(exc).__name__, exc)
        try:
            value = self._simulator.GetSimulatorVersion(empty, timeout=self._timeout)
            result["simulator_version"] = str(value.value).strip() or "unknown"
        except Exception as exc:
            result["simulator_version"] = "unknown"
            errors["version"] = "{}: {}".format(type(exc).__name__, exc)
        if errors:
            result["errors"] = errors
        return result

    def close(self):
        self._channel.close()
