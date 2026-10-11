"""Filesystem ownership aliases for frozen map source identities.

Historical records continue naming their original source identity. Live lookup
uses the sole asset copy in asmc_planner; geometry and source hashes are unchanged.
"""

_OLD = "src/behavior_planner/src/config/"
_CURRENT = "src/asmc_planner/config/behavior/"


def current_source_path(raw: str) -> str:
    return _CURRENT + raw[len(_OLD):] if raw.startswith(_OLD) else raw


def recorded_source_path(raw: str) -> str:
    return _OLD + raw[len(_CURRENT):] if raw.startswith(_CURRENT) else raw
