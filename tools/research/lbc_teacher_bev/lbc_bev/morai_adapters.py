"""Convert MORAI ROS messages to LBC BEV dynamic state types."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Set, Tuple, Union

from .dynamic_layers import PedestrianState, TrafficLightState, VehicleState
from .intersection_tables import (
    expand_intersection_phase,
    intersection_centroid,
    load_intersection_traffic_groups,
    pick_ego_traffic_group_indices,
)
from .spec import (
    BEV_OBJECT_FILTER_RADIUS_M,
    MAP_SIZE,
    PIXELS_PER_METER,
    TL_BEV_MARGIN_PX,
    TL_PHASE_MAX_DISTANCE_M,
)
from .morai_vehicle import IONIQ5
from .spec import (
    LBC_DEFAULT_PEDESTRIAN_LENGTH_M,
    LBC_DEFAULT_PEDESTRIAN_WIDTH_M,
    LBC_VEHICLE_TL_TYPES,
)

VEHICLE_TYPE_REAR_WHEEL = 1

# MORAI / 전국신호등표준데이터 LightColor (bit flags)
TL_R = 0b000000000001
TL_Y = 0b000000000100
TL_SG = 0b000000010000
TL_LG = 0b000000100000
TL_RG = 0b000001000000
TL_UTG = 0b000010000000
TL_ULG = 0b000100000000
TL_URG = 0b001000000000
TL_LLG = 0b010000000000
TL_LRG = 0b100000000000

TL_GREEN_MASK = TL_SG | TL_LG | TL_RG | TL_UTG | TL_ULG | TL_URG | TL_LLG | TL_LRG
TL_STRAIGHT_GREEN_MASK = TL_SG | TL_RG | TL_UTG | TL_URG
TL_LEFT_GREEN_MASK = TL_LG | TL_ULG | TL_LLG | TL_LRG

# morai_msgs/GetTrafficLightStatus.trafficLightStatus (Sim enum, not LightColor bitmask)
MORAI_STATUS_RED = 1
MORAI_STATUS_YELLOW = 4
MORAI_STATUS_RED_YELLOW = 5
MORAI_STATUS_GREEN = 16
MORAI_STATUS_GREEN_YELLOW = 20
MORAI_STATUS_LEFT_ARROW = 32
MORAI_STATUS_LEFT_RED = 33
MORAI_STATUS_LEFT_YELLOW = 36
MORAI_STATUS_LEFT_GREEN = 48

MORAI_TL_TYPE_RYG = 0
MORAI_TL_TYPE_RYG_LEFT = 1
MORAI_TL_TYPE_RYG_LEFT_GREEN = 2
MORAI_TL_TYPE_FLASH_YELLOW = 100

MORAI_ENUM_STATUS_VALUES = frozenset(
    {
        MORAI_STATUS_RED,
        MORAI_STATUS_YELLOW,
        MORAI_STATUS_RED_YELLOW,
        MORAI_STATUS_GREEN,
        MORAI_STATUS_GREEN_YELLOW,
        MORAI_STATUS_LEFT_ARROW,
        MORAI_STATUS_LEFT_RED,
        MORAI_STATUS_LEFT_YELLOW,
        MORAI_STATUS_LEFT_GREEN,
    }
)

# Left-turn blinker ON: one BEV color per MORAI enum (all head types).
MORAI_ENUM_LEFT_INTENT_COLOR: Dict[int, str] = {
    MORAI_STATUS_RED: "red",
    MORAI_STATUS_YELLOW: "yellow",
    MORAI_STATUS_RED_YELLOW: "yellow",
    MORAI_STATUS_GREEN: "red",
    MORAI_STATUS_GREEN_YELLOW: "yellow",
    MORAI_STATUS_LEFT_ARROW: "green",
    MORAI_STATUS_LEFT_RED: "green",
    MORAI_STATUS_LEFT_YELLOW: "green",
    MORAI_STATUS_LEFT_GREEN: "green",
}

# Blinkers off / right turn (turnSignal 0 or 2): one BEV color per MORAI enum.
MORAI_ENUM_STRAIGHT_INTENT_COLOR: Dict[int, str] = {
    MORAI_STATUS_RED: "red",
    MORAI_STATUS_YELLOW: "yellow",
    MORAI_STATUS_RED_YELLOW: "red",
    MORAI_STATUS_GREEN: "green",
    MORAI_STATUS_GREEN_YELLOW: "green",
    MORAI_STATUS_LEFT_ARROW: "red",
    MORAI_STATUS_LEFT_RED: "red",
    MORAI_STATUS_LEFT_YELLOW: "yellow",
    MORAI_STATUS_LEFT_GREEN: "green",
}

TLEncoding = Literal["enum", "bitmask"]

# morai_msgs/Lamps.turnSignal
TURN_SIGNAL_NONE = 0
TURN_SIGNAL_LEFT = 1
TURN_SIGNAL_RIGHT = 2
TURN_SIGNAL_KEEP = 3

TurnSignalMode = Literal["straight", "left", "right"]
TLTurnIntent = Literal["straight", "left"]
_BULB_LABELS = frozenset({"red", "yellow", "green"})

CatalogEntry = Tuple[float, float, float, Tuple[str, ...]]


class TurnSignalTracker:
    """
    morai_msgs/Lamps → TL display intent.

    - turnSignal 1 (left): left-turn display rules
    - turnSignal 0 / 2 (off / right): straight display rules
    """

    def __init__(self, default_mode: TurnSignalMode = "straight"):
        self._mode: TurnSignalMode = default_mode
        self._last_raw_turn_signal: int = TURN_SIGNAL_NONE

    @property
    def mode(self) -> TurnSignalMode:
        return self._mode

    @property
    def last_raw_turn_signal(self) -> int:
        return self._last_raw_turn_signal

    def update_from_lamps(self, turn_signal: int) -> TurnSignalMode:
        self._last_raw_turn_signal = int(turn_signal)
        return self.update(int(turn_signal))

    def update(self, raw_turn_signal: int) -> TurnSignalMode:
        self._mode = parse_turn_signal_mode(int(raw_turn_signal), self._mode)
        return self._mode


def effective_tl_turn_intent(mode: TurnSignalMode) -> TLTurnIntent:
    """Right turn uses the same TL filtering as straight (blinkers off)."""
    if mode == "left":
        return "left"
    return "straight"


def parse_turn_signal_mode(
    raw_turn_signal: int, previous: TurnSignalMode = "straight"
) -> TurnSignalMode:
    raw = int(raw_turn_signal)
    if raw == TURN_SIGNAL_LEFT:
        return "left"
    if raw == TURN_SIGNAL_RIGHT:
        return "straight"
    if raw == TURN_SIGNAL_KEEP:
        return previous
    return "straight"


def turn_signal_mode_from_lamps_msg(
    msg: Any, previous: TurnSignalMode = "straight"
) -> TurnSignalMode:
    if msg is None:
        return previous
    if hasattr(msg, "lamps"):
        return turn_signal_mode_from_lamps_msg(msg.lamps, previous)
    if hasattr(msg, "turnSignal"):
        return parse_turn_signal_mode(int(msg.turnSignal), previous)
    return previous


def _event_info_from_morai_event_cmd_response(resp: Any) -> Any:
    """MoraiEventCmdSrv returns ``EventInfo`` in ``response`` (not top-level)."""
    if hasattr(resp, "response"):
        return resp.response
    return resp


def poll_turn_signal_from_morai_event_cmd(
    tracker: TurnSignalTracker,
    service_proxy: Any,
    *,
    request_option: int = 0,
) -> Tuple[TurnSignalMode, int]:
    """
    Query /Service_MoraiEventCmd (MoraiEventCmdSrv) for ego turnSignal.

    Example CLI::
        rosservice call /Service_MoraiEventCmd "{request: {option: 0}}"

    Returns (resolved_mode, raw_turnSignal).
    """
    from morai_msgs.srv import MoraiEventCmdSrvRequest

    req = MoraiEventCmdSrvRequest()
    req.request.option = int(request_option)
    resp = service_proxy(req)
    event = _event_info_from_morai_event_cmd_response(resp)
    raw = int(event.lamps.turnSignal)
    mode = tracker.update_from_lamps(raw)
    return mode, raw


def setup_morai_event_cmd_turn_signal_polling(
    tracker: TurnSignalTracker,
    lock: Any,
    *,
    log_tag: str = "[LBCBEV]",
) -> Optional[Any]:
    """
    Poll ``/Service_MoraiEventCmd`` for turnSignal (do not use empty ``/Lamps_topic``).

    Returns the service proxy on success, else ``None``.
    """
    import rospy
    from morai_msgs.srv import MoraiEventCmdSrv

    svc_name = str(rospy.get_param("~morai_event_cmd_service", "/Service_MoraiEventCmd"))
    wait_s = float(rospy.get_param("~morai_event_cmd_wait_s", 10.0))
    hz = float(rospy.get_param("~morai_event_cmd_hz", 10.0))
    state: Dict[str, Any] = {"proxy": None, "last_raw": -1}

    def _poll(_event) -> None:
        if state["proxy"] is None:
            return
        try:
            with lock:
                mode, raw = poll_turn_signal_from_morai_event_cmd(
                    tracker, state["proxy"], request_option=0
                )
            if raw != state["last_raw"]:
                state["last_raw"] = raw
                rospy.loginfo(
                    "%s turnSignal=%d → TL mode=%s (0=off 1=left 2=right)",
                    log_tag,
                    raw,
                    mode,
                )
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "%s MoraiEventCmd poll failed: %s", log_tag, exc)

    try:
        rospy.loginfo("%s Waiting for %s (%.0fs)...", log_tag, svc_name, wait_s)
        rospy.wait_for_service(svc_name, timeout=wait_s)
        state["proxy"] = rospy.ServiceProxy(svc_name, MoraiEventCmdSrv)
        rospy.Timer(rospy.Duration(1.0 / max(hz, 1.0)), _poll)
        rospy.loginfo("%s Turn signal via %s (MoraiEventCmd poll)", log_tag, svc_name)
        _poll(None)
        return state["proxy"]
    except (rospy.ROSException, Exception) as exc:
        rospy.logerr(
            "%s MoraiEventCmd unavailable (%s) — straight TL only. Check: "
            'rosservice call %s "{request: {option: 0}}"',
            log_tag,
            exc,
            svc_name,
        )
        return None


def _functional_sub_types(sub_types: Tuple[str, ...]) -> Set[str]:
    return {x.lower() for x in sub_types if x and x.lower() not in _BULB_LABELS}


def _signal_head_kind(sub_types: Tuple[str, ...]) -> str:
    """Classify car signal head: left_only | straight_only | combined | unknown."""
    functional = _functional_sub_types(sub_types)
    has_left = "left" in functional
    has_straight = "straight" in functional
    if has_left and has_straight:
        return "combined"
    if has_left:
        return "left_only"
    if has_straight:
        return "straight_only"
    if not functional:
        return "straight_only"
    return "unknown"


def signal_matches_turn_mode(sub_types: Tuple[str, ...], mode: TurnSignalMode) -> bool:
    """Whether this catalog head can be shown (color may still be None)."""
    kind = _signal_head_kind(sub_types)
    if kind == "unknown":
        return False
    intent = effective_tl_turn_intent(mode)
    if intent == "left":
        if kind in ("left_only", "combined"):
            return True
        return kind == "straight_only"
    return kind in ("straight_only", "combined")


_COLOR_PRIORITY: Dict[str, int] = {"red": 1, "yellow": 2, "green": 3}


def _best_color(*colors: Optional[str]) -> Optional[str]:
    valid = [c for c in colors if c]
    if not valid:
        return None
    return max(valid, key=lambda c: _COLOR_PRIORITY[c])


def _decode_straight_aspect(status: int) -> Optional[str]:
    s = int(status)
    if s in (MORAI_STATUS_GREEN_YELLOW, MORAI_STATUS_RED_YELLOW) or (
        (s & TL_SG) and (s & TL_Y)
    ):
        return "yellow"
    if s & TL_SG:
        return "green"
    if (s & TL_R) and (s & TL_Y):
        return "yellow"
    if s & TL_Y and not (s & TL_SG):
        return "yellow"
    if s & TL_R and not (s & TL_SG):
        return "red"
    return None


def _decode_left_aspect(status: int) -> Optional[str]:
    s = int(status)
    if s in (MORAI_STATUS_LEFT_GREEN,):
        return "green"
    if s in (MORAI_STATUS_LEFT_YELLOW,):
        return "yellow"
    if s in (MORAI_STATUS_LEFT_RED,):
        return "red"
    if s & TL_LG and not (s & TL_R):
        return "green"
    if (s & TL_R) and (s & TL_Y):
        return "yellow"
    if s & TL_Y and not (s & TL_LG):
        return "yellow"
    if s & TL_R and not (s & TL_LG):
        return "red"
    return None


def _morai_bitmask_fallback_straight(s: int) -> Optional[str]:
    if s & TL_STRAIGHT_GREEN_MASK:
        return "green"
    if (s & TL_R) and (s & TL_Y):
        return "yellow"
    if s & TL_Y:
        return "yellow"
    if s & TL_R:
        return "red"
    return None


def _morai_bitmask_fallback_left(s: int) -> Optional[str]:
    if s & TL_LEFT_GREEN_MASK:
        return "green"
    if (s & TL_R) and (s & TL_Y):
        return "yellow"
    if s & TL_Y:
        return "yellow"
    if s & TL_R:
        return "red"
    return None


def _straight_color_from_bitmask(s: int) -> Optional[str]:
    return _decode_straight_aspect(s) or _morai_bitmask_fallback_straight(s)


def _left_color_from_bitmask(s: int) -> Optional[str]:
    return _decode_left_aspect(s) or _morai_bitmask_fallback_left(s)


def _combined_color_straight_intent(
    straight: Optional[str], left: Optional[str]
) -> Optional[str]:
    """Blinkers off at straight+left head: straight R/Y first; else straight G; else left G/Y."""
    if straight in ("red", "yellow"):
        return straight
    if straight == "green":
        return "green"
    return left


def _combined_color_left_intent(
    straight: Optional[str], left: Optional[str]
) -> Optional[str]:
    """Left blinker at straight+left head: left arrow first; plain straight G (16) → red."""
    if left is not None:
        return left
    if straight == "green":
        return "red"
    if straight in ("red", "yellow"):
        return straight
    return "red"


def _color_for_head_from_aspects(
    straight: Optional[str],
    left: Optional[str],
    kind: str,
    intent: TLTurnIntent,
) -> Optional[str]:
    if kind == "left_only":
        if intent == "straight":
            return None
        if left is not None:
            return left
        return "red" if intent == "left" else None

    if kind == "straight_only":
        if intent == "left":
            if left is not None:
                return left
            if straight == "green":
                return "red"
            if straight in ("red", "yellow"):
                return straight
            return "red"
        return straight

    if kind == "combined":
        if intent == "left":
            return _combined_color_left_intent(straight, left)
        return _combined_color_straight_intent(straight, left)

    return None


def _morai_bitmask_straight_intent_display(status: int) -> Optional[str]:
    """IntscnTL bitmask when blinkers off / right turn (matches enum straight table)."""
    s = int(status)
    if s <= 0:
        return None
    if s & TL_STRAIGHT_GREEN_MASK:
        return "green"
    if (s & TL_Y) and not (s & TL_SG) and not (s & TL_LG):
        return "yellow"
    if (s & TL_R) and (s & TL_Y) and not (s & TL_SG):
        return "red"
    if s & TL_R:
        return "red"
    if s & TL_LG or s & TL_LEFT_GREEN_MASK:
        return "red"
    return None


def _morai_bitmask_left_intent_display(status: int) -> Optional[str]:
    """IntscnTL bitmask colors when left-turn blinker is on (matches enum table)."""
    s = int(status)
    if s <= 0:
        return None
    if s & TL_LEFT_GREEN_MASK:
        return "green"
    if s & TL_LG:
        return "green"
    if (s & TL_Y) and not (s & TL_SG) and not (s & TL_LG):
        return "yellow"
    if (s & TL_R) and not (s & TL_SG) and not (s & TL_LG):
        return "red"
    if (s & TL_SG) and not (s & TL_LG):
        return "red"
    if (s & TL_R) and (s & TL_Y):
        return "yellow"
    return None


def _morai_bitmask_to_color_for_head(
    status: int, kind: str, intent: TLTurnIntent
) -> Optional[str]:
    s = int(status)
    if s <= 0:
        return None

    if intent == "left":
        display = _morai_bitmask_left_intent_display(s)
        if display:
            return display
    if intent == "straight":
        display = _morai_bitmask_straight_intent_display(s)
        if display:
            return display

    straight = _straight_color_from_bitmask(s)
    left = _left_color_from_bitmask(s)
    return _color_for_head_from_aspects(straight, left, kind, intent)


def _morai_enum_composite_aspects(status: int) -> Tuple[Optional[str], Optional[str]]:
    """
    MORAI GetTrafficLightStatus — (straight_aspect, left_arrow_aspect).

    - 5 Red+Yellow, 20 Green+Yellow: **left-turn** yellow (not straight).
    - 32 Left Arrow: arrow on (left green).
    - 33/36/48 Left Arrow **with** Red/Yellow/Green: ``with`` = **straight** aspect;
      left arrow treated as green when lit alongside straight R/Y/G.
    """
    s = int(status)
    if s == MORAI_STATUS_RED:
        return "red", None
    if s == MORAI_STATUS_YELLOW:
        return "yellow", None
    if s == MORAI_STATUS_GREEN:
        return "green", None
    if s in (MORAI_STATUS_RED_YELLOW, MORAI_STATUS_GREEN_YELLOW):
        return None, "yellow"
    if s == MORAI_STATUS_LEFT_ARROW:
        return None, "green"
    if s == MORAI_STATUS_LEFT_RED:
        return "red", "green"
    if s == MORAI_STATUS_LEFT_YELLOW:
        return "yellow", "green"
    if s == MORAI_STATUS_LEFT_GREEN:
        return "green", "green"
    return None, None


def _morai_enum_to_color_for_head(
    status: int, tl_type: int, kind: str, intent: TLTurnIntent
) -> Optional[str]:
    s = int(status)
    t = int(tl_type)
    if t == MORAI_TL_TYPE_FLASH_YELLOW or s == MORAI_TL_TYPE_FLASH_YELLOW:
        return "yellow"

    if intent == "left":
        display = MORAI_ENUM_LEFT_INTENT_COLOR.get(s)
        if display:
            return display
    if intent == "straight":
        display = MORAI_ENUM_STRAIGHT_INTENT_COLOR.get(s)
        if display:
            return display

    straight_c, left_c = _morai_enum_composite_aspects(s)
    return _color_for_head_from_aspects(straight_c, left_c, kind, intent)


def morai_status_to_color_for_signal(
    status: int,
    sub_types: Tuple[str, ...] = (),
    *,
    turn_mode: TurnSignalMode = "straight",
    encoding: TLEncoding = "bitmask",
    tl_type: int = 0,
) -> Optional[str]:
    """
    Map MORAI status → LBC circle color for one catalog pole.

    Left blinker: ``MORAI_ENUM_LEFT_INTENT_COLOR``.
    Off / right turn (0,2): ``MORAI_ENUM_STRAIGHT_INTENT_COLOR``.
    """
    if not signal_matches_turn_mode(sub_types, turn_mode):
        return None
    kind = _signal_head_kind(sub_types)
    if kind == "unknown":
        return None
    intent = effective_tl_turn_intent(turn_mode)
    if encoding == "enum":
        return _morai_enum_to_color_for_head(status, tl_type, kind, intent)
    return _morai_bitmask_to_color_for_head(status, kind, intent)


def _morai_status_to_color(
    status: int,
    sub_types: Tuple[str, ...] = (),
    *,
    turn_mode: TurnSignalMode = "straight",
    encoding: TLEncoding = "bitmask",
    tl_type: int = 0,
) -> Optional[str]:
    return morai_status_to_color_for_signal(
        status,
        sub_types,
        turn_mode=turn_mode,
        encoding=encoding,
        tl_type=tl_type,
    )


def _rear_axle_to_bbox_center_m(length: float) -> float:
    if abs(length - IONIQ5.length_m) < 0.35:
        return IONIQ5.rear_axle_to_bbox_center_m
    return length * 0.5


def _object_center_world(obj) -> Tuple[float, float, float, float, float]:
    ox, oy = float(obj.position.x), float(obj.position.y)
    length = float(obj.size.x)
    width = float(obj.size.y)
    yaw_deg = float(obj.heading)
    yaw_rad = math.radians(yaw_deg)
    rear_to_center = (
        _rear_axle_to_bbox_center_m(length)
        if int(getattr(obj, "type", -1)) == VEHICLE_TYPE_REAR_WHEEL
        else 0.0
    )
    cx = ox + rear_to_center * math.cos(yaw_rad)
    cy = oy + rear_to_center * math.sin(yaw_rad)
    return cx, cy, length, width, yaw_deg


def object_status_to_vehicle(obj) -> VehicleState:
    x, y, length, width, yaw_deg = _object_center_world(obj)
    return VehicleState(x=x, y=y, yaw_deg=yaw_deg, length=length, width=width)


def object_status_to_pedestrian(obj) -> PedestrianState:
    """LBC map_utils._render_walkers: CARLA bbox extent (full length/width), no extra scale."""
    x, y, length, width, yaw_deg = _object_center_world(obj)
    length = float(length) if length > 0.05 else LBC_DEFAULT_PEDESTRIAN_LENGTH_M
    width = float(width) if width > 0.05 else LBC_DEFAULT_PEDESTRIAN_WIDTH_M
    return PedestrianState(
        x=x,
        y=y,
        yaw_deg=yaw_deg,
        length=length,
        width=width,
    )


def _within_ego_radius(
    ox: float,
    oy: float,
    ego_xy: Tuple[float, float],
    radius_m: float,
) -> bool:
    if radius_m <= 0:
        return True
    return math.hypot(ox - ego_xy[0], oy - ego_xy[1]) <= radius_m


def object_status_list_to_states(
    obj_msg,
    *,
    ego_xy: Optional[Tuple[float, float]] = None,
    filter_radius_m: float = 0.0,
    max_vehicles: int = 0,
    max_pedestrians: int = 0,
) -> Tuple[List[VehicleState], List[PedestrianState]]:
    vehicles: List[VehicleState] = []
    pedestrians: List[PedestrianState] = []
    if obj_msg is None:
        return vehicles, pedestrians

    radius_m = float(filter_radius_m)
    if ego_xy is None and radius_m > 0:
        radius_m = 0.0

    def _accept(obj) -> bool:
        if radius_m <= 0 or ego_xy is None:
            return True
        ox, oy = float(obj.position.x), float(obj.position.y)
        return _within_ego_radius(ox, oy, ego_xy, radius_m)

    for obj in list(getattr(obj_msg, "npc_list", []) or []):
        if _accept(obj):
            vehicles.append(object_status_to_vehicle(obj))
    for obj in list(getattr(obj_msg, "obstacle_list", []) or []):
        if _accept(obj):
            vehicles.append(object_status_to_vehicle(obj))
    for obj in list(getattr(obj_msg, "pedestrian_list", []) or []):
        if _accept(obj):
            pedestrians.append(object_status_to_pedestrian(obj))

    if max_vehicles > 0 and len(vehicles) > max_vehicles:
        vehicles = vehicles[: int(max_vehicles)]
    if max_pedestrians > 0 and len(pedestrians) > max_pedestrians:
        pedestrians = pedestrians[: int(max_pedestrians)]
    return vehicles, pedestrians


def load_traffic_light_catalog(
    tl_json_path: Path,
    *,
    vehicle_only: bool = True,
) -> Dict[str, CatalogEntry]:
    if not tl_json_path.is_file():
        return {}
    with open(tl_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    catalog: Dict[str, CatalogEntry] = {}
    for tl in data:
        if vehicle_only and str(tl.get("type", "")) not in LBC_VEHICLE_TL_TYPES:
            continue
        idx = str(tl.get("idx", ""))
        pt = tl.get("point") or [0.0, 0.0]
        heading = float(tl.get("heading", tl.get("orientation", 0.0)))
        sub = tuple(str(s) for s in (tl.get("sub_type") or []) if s)
        catalog[idx] = (float(pt[0]), float(pt[1]), heading, sub)
    return catalog


def load_intscn_traffic_light_map(
    synced_json_path: Path,
    vehicle_signal_ids: Optional[Set[str]] = None,
) -> Dict[str, List[str]]:
    if not synced_json_path.is_file():
        return {}
    with open(synced_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    allow = vehicle_signal_ids
    out: Dict[str, List[str]] = {}
    for entry in data:
        ctrl = entry.get("intersection_controller_id")
        if not ctrl:
            continue
        signals = [str(s) for s in (entry.get("signal_id_list") or [])]
        if allow is not None:
            signals = [s for s in signals if s in allow]
        if not signals:
            continue
        out.setdefault(str(ctrl), []).extend(signals)
    for k in list(out.keys()):
        out[k] = list(dict.fromkeys(out[k]))
    return out


def _append_tl(
    out: List[TrafficLightState],
    signal_id: str,
    color: str,
    catalog: Dict[str, CatalogEntry],
) -> None:
    if signal_id not in catalog:
        return
    x, y, heading, _sub = catalog[signal_id]
    out.append(
        TrafficLightState(
            idx=signal_id, x=x, y=y, state=color, heading_deg=heading
        )
    )


def _tl_in_bev(
    wx: float,
    wy: float,
    bev_affine: Any,
    baked: Any,
    *,
    margin_px: int = 8,
) -> bool:
    from .spec import MAP_SIZE
    from .static_layers import world_to_bev_pixel_xy_from_map

    col, row = world_to_bev_pixel_xy_from_map(wx, wy, bev_affine, baked)
    return (
        -margin_px <= col < MAP_SIZE + margin_px
        and -margin_px <= row < MAP_SIZE + margin_px
    )


@dataclass
class CachedTL:
    status: int
    encoding: TLEncoding = "bitmask"
    tl_type: int = 0


class TrafficLightStateCache:
    """
    TL state cache:
    - GetTrafficLightStatus: per-signal enum (cleared each build — no time latch)
    - IntscnTL / InsnStatus: intersection **phase index** (0,1,2...) — latched
    """

    def __init__(self) -> None:
        self.entries: Dict[str, CachedTL] = {}
        self.intersection_phases: Dict[str, int] = {}
        self.global_status: Optional[CachedTL] = None
        self._traffic_groups: Dict[str, List[List[str]]] = {}

    def set_traffic_groups(self, groups: Dict[str, List[List[str]]]) -> None:
        self._traffic_groups = groups

    def _intersection_signal_ids(self, inttl: str) -> Set[str]:
        ids: Set[str] = set()
        for group in self._traffic_groups.get(inttl, []):
            ids.update(group)
        return ids

    def _all_intersection_signal_ids(self) -> Set[str]:
        ids: Set[str] = set()
        for groups in self._traffic_groups.values():
            for group in groups:
                ids.update(group)
        return ids

    def _signal_to_intersection(self) -> Dict[str, str]:
        out: Dict[str, str] = {}
        for inttl, groups in self._traffic_groups.items():
            for group in groups:
                for sig_id in group:
                    out[str(sig_id)] = str(inttl)
        return out

    def ingest(self, msg: Any) -> None:
        if msg is None:
            return

        if hasattr(msg, "trafficLightIndex") and hasattr(msg, "trafficLightStatus"):
            idx = str(msg.trafficLightIndex)
            st = int(msg.trafficLightStatus)
            tl_type = int(getattr(msg, "trafficLightType", 0))
            if st > 0:
                self.entries[idx] = CachedTL(st, "enum", tl_type)
            return

        if hasattr(msg, "intersection_index") and hasattr(msg, "intersection_status"):
            inttl = str(msg.intersection_index)
            if inttl.startswith("IntTL"):
                self._set_intersection_phase(inttl, int(msg.intersection_status))
            return

        if hasattr(msg, "idx") and hasattr(msg, "status") and not hasattr(msg, "state"):
            idx = str(msg.idx)
            st = int(msg.status)
            if idx.startswith("IntTL"):
                self._set_intersection_phase(idx, st)
            elif st > 0:
                self.entries[idx] = CachedTL(st, "bitmask", 0)
            return

        if hasattr(msg, "idx") and hasattr(msg, "state"):
            ids = list(msg.idx or [])
            states = list(msg.state or [])
            for i, raw_idx in enumerate(ids):
                if i >= len(states):
                    continue
                idx = str(raw_idx)
                st = int(states[i])
                if idx.startswith("IntTL"):
                    self._set_intersection_phase(idx, st)
                elif st > 0:
                    self.entries[idx] = CachedTL(st, "bitmask", 0)
            return

        if hasattr(msg, "light"):
            st = int(msg.light)
            if st > 0:
                self.global_status = CachedTL(st, "bitmask", 0)

    def _set_intersection_phase(self, inttl: str, phase: int) -> None:
        """Update latched intersection phase (per-signal entries are frame-scoped)."""
        self.intersection_phases[inttl] = int(phase)

    def _pick_facing_group(
        self,
        groups: List[List[str]],
        signal_catalog: Dict[str, CatalogEntry],
        ego_xy: Tuple[float, float],
        ego_yaw_deg: float,
        *,
        require_confident_approach: bool,
    ) -> Tuple[Optional[List[int]], Set[str], bool]:
        picked, confident = pick_ego_traffic_group_indices(
            ego_xy[0], ego_xy[1], float(ego_yaw_deg), groups, signal_catalog
        )
        if require_confident_approach and not confident:
            return None, set(), False
        if not picked:
            return None, set(), confident
        gi = int(picked[0])
        if gi < 0 or gi >= len(groups):
            return None, set(), confident
        return picked, {str(s) for s in groups[gi]}, confident

    def build_traffic_lights(
        self,
        signal_catalog: Dict[str, CatalogEntry],
        intscn_catalog: Optional[Dict[str, List[str]]] = None,
        *,
        turn_mode: TurnSignalMode = "straight",
        bev_affine: Any = None,
        baked: Any = None,
        filter_to_bev: bool = True,
        ego_xy: Optional[Tuple[float, float]] = None,
        ego_yaw_deg: Optional[float] = None,
        bev_margin_px: int = TL_BEV_MARGIN_PX,
        ego_facing_only: bool = True,
        require_confident_approach: bool = True,
    ) -> List[TrafficLightState]:
        del intscn_catalog
        if not signal_catalog:
            return []

        per_signal_entries = dict(self.entries)
        self.entries.clear()

        out: List[TrafficLightState] = []
        seen: Set[str] = set()

        # Cheap world cull before per-signal affine (margin keeps poles visible near edges).
        tl_world_r_m = (0.5 * MAP_SIZE + bev_margin_px) / float(PIXELS_PER_METER)

        def emit(sig_id: str, cached: CachedTL) -> None:
            if sig_id in seen or sig_id not in signal_catalog:
                return
            entry = signal_catalog[sig_id]
            if filter_to_bev and ego_xy is not None:
                dx = float(entry[0]) - float(ego_xy[0])
                dy = float(entry[1]) - float(ego_xy[1])
                if dx * dx + dy * dy > tl_world_r_m * tl_world_r_m:
                    return
            sub = entry[3] if len(entry) > 3 else ()
            color = morai_status_to_color_for_signal(
                cached.status,
                sub,
                turn_mode=turn_mode,
                encoding=cached.encoding,
                tl_type=cached.tl_type,
            )
            if not color:
                return
            if filter_to_bev and bev_affine is not None and baked is not None:
                if not _tl_in_bev(
                    entry[0], entry[1], bev_affine, baked, margin_px=bev_margin_px
                ):
                    return
            seen.add(sig_id)
            _append_tl(out, sig_id, color, signal_catalog)

        if self.global_status is not None:
            for sig_id in signal_catalog:
                emit(sig_id, self.global_status)
            return out

        sig_inttl = self._signal_to_intersection()
        merged: Dict[str, CachedTL] = {}
        near_phased_inttls: Set[str] = set()
        facing_by_inttl: Dict[str, Set[str]] = {}

        for inttl, phase in self.intersection_phases.items():
            groups = self._traffic_groups.get(inttl)
            if not groups:
                continue

            near = True
            if ego_xy is not None:
                center = intersection_centroid(groups, signal_catalog)
                if center is not None:
                    dist = math.hypot(center[0] - ego_xy[0], center[1] - ego_xy[1])
                    if dist > TL_PHASE_MAX_DISTANCE_M:
                        near = False
            if not near:
                continue

            near_phased_inttls.add(inttl)
            gi: Optional[List[int]] = None
            facing: Set[str] = set()

            if ego_facing_only and ego_xy is not None and ego_yaw_deg is not None:
                gi, facing, confident = self._pick_facing_group(
                    groups,
                    signal_catalog,
                    (ego_xy[0], ego_xy[1]),
                    float(ego_yaw_deg),
                    require_confident_approach=require_confident_approach,
                )
                if require_confident_approach and not confident:
                    facing_by_inttl[inttl] = set()
                    continue
            else:
                facing = self._intersection_signal_ids(inttl)

            facing_by_inttl[inttl] = facing

            for sig_id, bm in expand_intersection_phase(
                inttl, phase, groups, group_indices=gi
            ).items():
                if sig_id in signal_catalog:
                    merged[sig_id] = CachedTL(bm, "bitmask", 0)

        for sig_id, cached in per_signal_entries.items():
            inttl = sig_inttl.get(sig_id)
            if inttl in near_phased_inttls:
                facing = facing_by_inttl.get(inttl, set())
                if facing and sig_id not in facing:
                    continue
            merged[sig_id] = cached

        for sig_id, cached in merged.items():
            emit(sig_id, cached)

        return out


def ingest_traffic_light_msg(cache: TrafficLightStateCache, msg: Any) -> None:
    cache.ingest(msg)


def traffic_lights_from_ros_msg(
    msg: Any,
    signal_catalog: Dict[str, CatalogEntry],
    intscn_catalog: Optional[Dict[str, List[str]]] = None,
    *,
    turn_mode: TurnSignalMode = "straight",
    cache: Optional[TrafficLightStateCache] = None,
    bev_affine: Any = None,
    baked: Any = None,
    filter_to_bev: bool = True,
) -> List[TrafficLightState]:
    """One-shot parse (optionally merges into *cache* and returns full latched view)."""
    if cache is None:
        cache = TrafficLightStateCache()
        cache.ingest(msg)
    else:
        cache.ingest(msg)
    return cache.build_traffic_lights(
        signal_catalog,
        intscn_catalog,
        turn_mode=turn_mode,
        bev_affine=bev_affine,
        baked=baked,
        filter_to_bev=filter_to_bev,
    )
