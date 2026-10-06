"""Selective, ROS-independent lookup over already received global messages.

Build once on a coherent snapshot; queries do not deserialize JSON or import
ROS, and return original vector objects with their map IDs and geometry.
"""
from __future__ import annotations

from collections import defaultdict
import json
import math

from asmc_hd_map.local_crop import EgoWindow, geometry_bounds, select_ego_bounds


LAYER_FIELDS = {"lanes": "lanes", "areas": "areas", "markings": "markings",
                "checkpoints": "checkpoints", "missions": "zones"}


class SemanticMapIndex:
    def __init__(self, meta, layers):
        if not meta.complete or not meta.operational or meta.header.frame_id != "map":
            raise ValueError("a complete operational map snapshot is required")
        self.build_id = meta.build_id
        self.items, self.bounds = {}, {}
        self.by_id = {}
        self.missions_by_link = defaultdict(list)
        self.speed_by_link = {}
        self.payloads = {}
        self.signals = []
        for name, message in layers.items():
            if name not in LAYER_FIELDS:
                raise ValueError("unsupported semantic layer: " + name)
            if message.build_id != meta.build_id or message.header.frame_id != "map":
                raise ValueError("mixed map identities or frames")
            if hasattr(message, "complete") and not message.complete:
                raise ValueError("global semantic layers must be complete")
            if hasattr(message, "geometry_sha256") and (
                message.geometry_sha256 != meta.geometry_sha256 or message.map_id != meta.map_id
            ):
                raise ValueError("mixed geometry identities")
            items = list(getattr(message, LAYER_FIELDS[name]))
            self.items[name] = items
            self.by_id[name] = {item.id: item for item in items}
            if len(self.by_id[name]) != len(items):
                raise ValueError("duplicate global feature IDs")
            self.bounds[name] = [self._bounds(item) for item in items]
        for mission in self.items.get("missions", []):
            payload = json.loads(mission.rule_payload_json or "{}")
            self.payloads[mission.id] = payload
            for link in mission.link_ids:
                self.missions_by_link[link].append(mission)
            if mission.status == mission.STATUS_PROVISIONAL:
                continue
            for limit in payload.get("route_speed_limits", []):
                value = float(limit["effective_limit_kph"])
                if not math.isfinite(value) or value <= 0:
                    raise ValueError("invalid effective speed limit")
                self.speed_by_link[limit["link_id"]] = limit
            if (mission.mission_type == mission.TYPE_TRAFFIC_LIGHT
                    and payload.get("boundary_only") and payload.get("traffic_light_id")):
                self.signals.append(mission)
        self.signals.sort(key=lambda m: m.route_station_start_m)

    @staticmethod
    def _bounds(item):
        point_sets = []
        for key in ("polygon", "polyline", "centerline"):
            if hasattr(item, key):
                point_sets.append([(p.x, p.y) for p in getattr(item, key)])
        if hasattr(item, "pose"):
            point_sets.append([(item.pose.position.x, item.pose.position.y)])
        return geometry_bounds(point_sets)

    def spatial(self, xy, yaw, window=EgoWindow(), layers=None):
        names = tuple(self.items) if layers is None else tuple(layers)
        if any(name not in self.items for name in names):
            raise ValueError("requested layer has not been subscribed")
        return {name: [self.items[name][i] for i in select_ego_bounds(
            self.bounds[name], xy, yaw, window
        )] for name in names}

    def active_missions(self, link_id, station_m=None):
        """Exclude provisional rules; station disambiguates partial-link spans.

        Boundary-only traffic lights must be obtained via signals_ahead, so a
        stop line is never treated as an active rule over its entire owner link.
        """
        if station_m is not None and not math.isfinite(station_m):
            raise ValueError("station must be finite")
        return [m for m in self.missions_by_link.get(link_id, [])
                if m.status != m.STATUS_PROVISIONAL
                and not self.payloads[m.id].get("boundary_only", False)
                and (station_m is None or m.route_station_start_m <= station_m < m.route_station_end_m)]

    def speed_limit(self, link_id):
        """Return min(MGeo maximum, competition cap), or None off the route.

        A high-speed exception removes the competition cap only, not MGeo's.
        All fields are in km/h; divide effective_limit_kph by 3.6 for SI.
        """
        return self.speed_by_link.get(link_id)

    def signals_ahead(self, station_m, distance_m=120.0, route_length_m=None):
        """Route-distance lookup including zero-length signal boundary zones.

        IDs join perception/simulator state; a static map cannot supply color.
        If markings were selected, return associated reviewed stop-line geometry.
        """
        if not math.isfinite(station_m) or not math.isfinite(distance_m) or distance_m < 0:
            raise ValueError("signal query requires finite station and nonnegative distance")
        if route_length_m is not None:
            self._validate_route_length(station_m, route_length_m)
        result = []
        for mission in self.signals:
            distance = mission.route_station_start_m - station_m
            if route_length_m is not None:
                if distance < 0:
                    distance += route_length_m
            if 0 <= distance <= distance_m:
                payload = self.payloads[mission.id]
                ids = payload.get("hd_map_stopline_ids", [])
                result.append(dict(traffic_light_id=payload["traffic_light_id"],
                                   distance_m=distance, stopline_ids=ids,
                                   stoplines=[self.by_id.get("markings", {})[i] for i in ids
                                              if i in self.by_id.get("markings", {})],
                                   mission=mission))
        return sorted(result, key=lambda value: value["distance_m"])

    def missions_at_station(self, station_m):
        """Official-route rules independent of current/support lane membership."""
        if not math.isfinite(station_m) or station_m < 0:
            raise ValueError("station must be finite and nonnegative")
        return [m for m in self.items.get("missions", [])
                if m.status != m.STATUS_PROVISIONAL
                and not self.payloads[m.id].get("boundary_only", False)
                and m.route_station_start_m <= station_m < m.route_station_end_m]

    def speed_at_station(self, station_m):
        if not math.isfinite(station_m) or station_m < 0:
            raise ValueError("station must be finite and nonnegative")
        return next((limit for limit in self.speed_by_link.values()
                     if limit["route_station_start_m"] <= station_m < limit["route_station_end_m"]), None)

    def mission_boundaries(self, station_m, ahead_m=200., behind_m=30., route_length_m=None):
        """All rule transitions by route distance, including zero-length signals.

        Returns original mission objects; callers join their static payloads once.
        Geometry of boundary points comes from RouteReference.point_at(station).
        """
        if not all(math.isfinite(v) for v in (station_m, ahead_m, behind_m)) or min(station_m, ahead_m, behind_m) < 0:
            raise ValueError("boundary query distances must be finite and nonnegative")
        if route_length_m is not None:
            self._validate_route_length(station_m, route_length_m)
        result = []
        for mission in self.items.get("missions", []):
            if mission.status == mission.STATUS_PROVISIONAL:
                continue
            start, end = mission.route_station_start_m, mission.route_station_end_m
            points = [(start, "point", mission.start_link_id)] if start == end else [
                (start, "enter", mission.start_link_id), (end, "exit", mission.end_link_id)]
            for station, kind, link in points:
                distance = station - station_m
                if route_length_m is not None:
                    if distance < -behind_m:
                        distance += route_length_m
                    elif distance > ahead_m:
                        distance -= route_length_m
                if -behind_m <= distance <= ahead_m:
                    result.append(dict(mission=mission, boundary_type=kind, link_id=link,
                                       route_station_m=station, distance_m=distance))
        return sorted(result, key=lambda b: (b["distance_m"], b["mission"].id, b["boundary_type"]))

    @staticmethod
    def _validate_route_length(station_m, length_m):
        if not math.isfinite(length_m) or length_m <= 0 or not 0 <= station_m <= length_m:
            raise ValueError("closed route queries require a positive length and in-range station")
