"""Stateful, ROS-independent association of a map pose with a traversable lane."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point, Polygon


STATE_UNKNOWN = 0
STATE_INSIDE = 1
STATE_BOUNDARY = 2
STATE_NEARBY = 3


def _angle_error(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass(frozen=True)
class LaneGeometry:
    lane_id: str
    link_id: str
    lane_number: int
    lane_number_scope_id: str
    polygon: Polygon
    centerline: LineString
    left_adjacent_lane_id: str = ""
    right_adjacent_lane_id: str = ""
    predecessor_lane_ids: Tuple[str, ...] = ()
    successor_lane_ids: Tuple[str, ...] = ()
    bidirectional: bool = False
    route_assigned: bool = False
    intersection: bool = False

    @classmethod
    def from_coordinates(
        cls,
        lane_id: str,
        link_id: str,
        lane_number: int,
        lane_number_scope_id: str,
        polygon_xy: Iterable[Sequence[float]],
        centerline_xy: Iterable[Sequence[float]],
        **kwargs,
    ) -> "LaneGeometry":
        polygon = Polygon([(float(p[0]), float(p[1])) for p in polygon_xy])
        centerline = LineString([(float(p[0]), float(p[1])) for p in centerline_xy])
        if not polygon.is_valid or polygon.is_empty or polygon.area <= 0.0:
            raise ValueError(f"lane {lane_id} has an invalid polygon")
        if centerline.is_empty or centerline.length <= 0.0:
            raise ValueError(f"lane {lane_id} has an invalid centerline")
        return cls(
            lane_id=str(lane_id),
            link_id=str(link_id),
            lane_number=int(lane_number),
            lane_number_scope_id=str(lane_number_scope_id),
            polygon=polygon,
            centerline=centerline,
            **kwargs,
        )


@dataclass(frozen=True)
class LaneAssociationResult:
    valid: bool
    state: int
    lane_id: str = ""
    link_id: str = ""
    lane_number: int = 0
    lane_number_scope_id: str = ""
    centerline_distance_m: float = math.inf
    signed_lateral_offset_m: float = math.nan
    polygon_clearance_m: float = -math.inf
    heading_error_rad: float = math.nan
    confidence: float = 0.0
    previous_lane_id: str = ""
    transition_pending: bool = False
    transition_candidate_lane_id: str = ""
    transition_candidate_samples: int = 0


@dataclass(frozen=True)
class _Candidate:
    lane: LaneGeometry
    inside: bool
    clearance_m: float
    center_distance_m: float
    signed_lateral_m: float
    heading_error_rad: float


class LaneAssociator:
    """Associate rear-axle poses while suppressing boundary-switch chatter.

    Lateral switches require the reference point to enter the adjacent polygon
    by ``switch_penetration_m`` for ``switch_confirmation_samples`` distinct
    pose updates. Longitudinal predecessor/successor transitions are immediate
    once the previous polygon no longer covers the reference point.
    """

    def __init__(
        self,
        lanes: Iterable[LaneGeometry],
        switch_penetration_m: float = 0.30,
        switch_confirmation_samples: int = 2,
        boundary_band_m: float = 0.15,
        nearby_distance_m: float = 2.0,
        maximum_heading_error_deg: float = 100.0,
        intersection_switch_margin_m: float = 0.75,
    ) -> None:
        self.lanes = tuple(lanes)
        if not self.lanes:
            raise ValueError("at least one lane is required")
        self.by_id = {lane.lane_id: lane for lane in self.lanes}
        if len(self.by_id) != len(self.lanes):
            raise ValueError("lane IDs must be unique")
        self.switch_penetration_m = float(switch_penetration_m)
        self.switch_confirmation_samples = int(switch_confirmation_samples)
        self.boundary_band_m = float(boundary_band_m)
        self.nearby_distance_m = float(nearby_distance_m)
        self.maximum_heading_error_rad = math.radians(maximum_heading_error_deg)
        self.intersection_switch_margin_m = float(intersection_switch_margin_m)
        if self.switch_penetration_m < 0.0 or self.switch_confirmation_samples < 1:
            raise ValueError("invalid lane-switch hysteresis")
        if (
            self.boundary_band_m < 0.0
            or self.nearby_distance_m <= 0.0
            or self.intersection_switch_margin_m < 0.0
        ):
            raise ValueError("invalid association distance")
        self.current_lane_id = ""
        self.pending_lane_id = ""
        self.pending_samples = 0

    @staticmethod
    def _centerline_metrics(
        centerline: LineString, point: Point, yaw_rad: float
    ) -> Tuple[float, float, float]:
        station = float(centerline.project(point))
        closest = centerline.interpolate(station)
        delta = min(0.5, max(centerline.length * 0.01, 0.05))
        before = centerline.interpolate(max(0.0, station - delta))
        after = centerline.interpolate(min(centerline.length, station + delta))
        tangent_x = float(after.x - before.x)
        tangent_y = float(after.y - before.y)
        norm = math.hypot(tangent_x, tangent_y)
        if norm <= 1e-9:
            raise ValueError("lane centerline has no tangent at the projected point")
        tangent_x /= norm
        tangent_y /= norm
        lane_yaw = math.atan2(tangent_y, tangent_x)
        signed_lateral = (
            tangent_x * float(point.y - closest.y)
            - tangent_y * float(point.x - closest.x)
        )
        return (
            float(point.distance(centerline)),
            signed_lateral,
            _angle_error(float(yaw_rad), lane_yaw),
        )

    def _candidate(
        self, lane: LaneGeometry, point: Point, yaw_rad: float
    ) -> Optional[_Candidate]:
        min_x, min_y, max_x, max_y = lane.polygon.bounds
        margin = self.nearby_distance_m
        if not (
            min_x - margin <= point.x <= max_x + margin
            and min_y - margin <= point.y <= max_y + margin
        ):
            return None
        center_distance, signed_lateral, heading_error = self._centerline_metrics(
            lane.centerline, point, yaw_rad
        )
        if (
            not lane.bidirectional
            and abs(heading_error) > self.maximum_heading_error_rad
        ):
            return None
        inside = bool(lane.polygon.covers(point))
        clearance = float(point.distance(lane.polygon.boundary))
        if not inside:
            clearance = -float(point.distance(lane.polygon))
        return _Candidate(
            lane=lane,
            inside=inside,
            clearance_m=clearance,
            center_distance_m=center_distance,
            signed_lateral_m=signed_lateral,
            heading_error_rad=heading_error,
        )

    @staticmethod
    def _rank(candidate: _Candidate) -> Tuple[int, float, float]:
        return (
            0 if candidate.inside else 1,
            candidate.center_distance_m,
            abs(candidate.heading_error_rad),
        )

    def _result(
        self,
        candidate: Optional[_Candidate],
        previous_lane_id: str,
        state: Optional[int] = None,
    ) -> LaneAssociationResult:
        if candidate is None:
            return LaneAssociationResult(
                valid=False,
                state=STATE_UNKNOWN,
                previous_lane_id=previous_lane_id,
                transition_pending=bool(self.pending_lane_id),
                transition_candidate_lane_id=self.pending_lane_id,
                transition_candidate_samples=self.pending_samples,
            )
        if state is None:
            if candidate.inside:
                state = (
                    STATE_BOUNDARY
                    if candidate.clearance_m <= self.boundary_band_m
                    else STATE_INSIDE
                )
            else:
                state = STATE_NEARBY
        if state == STATE_INSIDE:
            confidence = min(1.0, 0.75 + candidate.clearance_m / 2.0)
        elif state == STATE_BOUNDARY:
            confidence = 0.65
        else:
            confidence = max(
                0.15, 0.55 * (1.0 + candidate.clearance_m / self.nearby_distance_m)
            )
        lane = candidate.lane
        return LaneAssociationResult(
            valid=True,
            state=state,
            lane_id=lane.lane_id,
            link_id=lane.link_id,
            lane_number=lane.lane_number,
            lane_number_scope_id=lane.lane_number_scope_id,
            centerline_distance_m=candidate.center_distance_m,
            signed_lateral_offset_m=candidate.signed_lateral_m,
            polygon_clearance_m=candidate.clearance_m,
            heading_error_rad=candidate.heading_error_rad,
            confidence=confidence,
            previous_lane_id=previous_lane_id,
            transition_pending=bool(self.pending_lane_id),
            transition_candidate_lane_id=self.pending_lane_id,
            transition_candidate_samples=self.pending_samples,
        )

    def _reset_pending(self) -> None:
        self.pending_lane_id = ""
        self.pending_samples = 0

    def update(self, x_m: float, y_m: float, yaw_rad: float) -> LaneAssociationResult:
        values = (float(x_m), float(y_m), float(yaw_rad))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("lane association pose must be finite")
        point = Point(values[0], values[1])
        candidates = [
            candidate
            for lane in self.lanes
            for candidate in [self._candidate(lane, point, values[2])]
            if candidate is not None
        ]
        candidates.sort(key=self._rank)
        by_id = {candidate.lane.lane_id: candidate for candidate in candidates}
        inside = [candidate for candidate in candidates if candidate.inside]
        previous_lane_id = self.current_lane_id

        if not self.current_lane_id:
            if inside:
                chosen = inside[0]
                self.current_lane_id = chosen.lane.lane_id
                return self._result(chosen, "")
            if candidates and -candidates[0].clearance_m <= self.nearby_distance_m:
                chosen = candidates[0]
                self.current_lane_id = chosen.lane.lane_id
                return self._result(chosen, "", STATE_NEARBY)
            return self._result(None, "")

        current_lane = self.by_id[self.current_lane_id]
        current = by_id.get(self.current_lane_id)
        if current is not None and current.inside:
            self._reset_pending()
            return self._result(current, previous_lane_id)

        # First preserve explicit topology. When an established lane has
        # completely left the search neighbourhood, guarded-intersection
        # support polygons can overlap the official route polygon. Prefer the
        # route-assigned candidate only for that reacquisition ambiguity. A
        # fresh acquisition and normal adjacent-lane motion stay proximity-
        # based, so obstacle-manoeuvre support lanes remain selectable.
        connected_inside = [
            candidate
            for candidate in inside
            if candidate.lane.lane_id in current_lane.successor_lane_ids
            or candidate.lane.lane_id in current_lane.predecessor_lane_ids
        ]
        if connected_inside:
            best_inside = connected_inside[0]
        elif current is None:
            route_inside = [candidate for candidate in inside if candidate.lane.route_assigned]
            best_inside = route_inside[0] if route_inside else (inside[0] if inside else None)
        else:
            best_inside = inside[0] if inside else None
        if best_inside is not None:
            target_id = best_inside.lane.lane_id
            longitudinal = (
                target_id in current_lane.successor_lane_ids
                or target_id in current_lane.predecessor_lane_ids
            )
            lateral = target_id in {
                current_lane.left_adjacent_lane_id,
                current_lane.right_adjacent_lane_id,
            }
            if longitudinal:
                self.current_lane_id = target_id
                self._reset_pending()
                return self._result(best_inside, previous_lane_id)

            intersection_transfer = (
                current is not None
                and not current.inside
                and current_lane.intersection
                and best_inside.lane.intersection
                and best_inside.clearance_m > self.boundary_band_m
                and current.center_distance_m - best_inside.center_distance_m
                >= self.intersection_switch_margin_m
            )
            if intersection_transfer:
                self.current_lane_id = target_id
                self._reset_pending()
                return self._result(best_inside, previous_lane_id)

            sufficiently_inside = (
                best_inside.clearance_m >= self.switch_penetration_m
            )
            if sufficiently_inside and (lateral or current is None):
                if self.pending_lane_id == target_id:
                    self.pending_samples += 1
                else:
                    self.pending_lane_id = target_id
                    self.pending_samples = 1
                if self.pending_samples >= self.switch_confirmation_samples:
                    self.current_lane_id = target_id
                    self._reset_pending()
                    return self._result(best_inside, previous_lane_id)
            else:
                self._reset_pending()

            if current is not None and -current.clearance_m <= self.nearby_distance_m:
                return self._result(current, previous_lane_id, STATE_NEARBY)
            return self._result(best_inside, previous_lane_id, STATE_BOUNDARY)

        self._reset_pending()
        if current is not None and -current.clearance_m <= self.nearby_distance_m:
            return self._result(current, previous_lane_id, STATE_NEARBY)
        self.current_lane_id = ""
        return self._result(None, previous_lane_id)
