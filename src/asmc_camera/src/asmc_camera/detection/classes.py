"""Normalized object groups for asmc_msgs/CameraObjectBox.class_name."""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Mapping, Optional


GROUPS = ("person", "vehicle", "obstacle")


@dataclass(frozen=True)
class ClassGroups:
    person: FrozenSet[str]
    vehicle: FrozenSet[str]
    excluded: FrozenSet[str]
    default_group: str = "obstacle"

    def group_of(self, source_class_name: str) -> Optional[str]:
        """Return the group for a raw label, or None when it is excluded."""
        key = str(source_class_name).strip().lower()
        if key in self.excluded:
            return None
        if key in self.person:
            return "person"
        if key in self.vehicle:
            return "vehicle"
        return self.default_group


def _names(values: object, label: str) -> FrozenSet[str]:
    if not isinstance(values, (list, tuple)):
        raise ValueError("{} must be a list of class names".format(label))
    names = frozenset(str(value).strip().lower() for value in values)
    if "" in names:
        raise ValueError("{} contains an empty class name".format(label))
    return names


def load_class_groups(config: Mapping[str, object]) -> ClassGroups:
    """Build groups from the ``object_classes.yaml`` mapping."""
    groups = config.get("groups")
    if not isinstance(groups, Mapping) or set(groups) != {"person", "vehicle"}:
        raise ValueError("groups must define exactly person and vehicle")
    person = _names(groups["person"], "groups.person")
    vehicle = _names(groups["vehicle"], "groups.vehicle")
    excluded = _names(config.get("excluded", []), "excluded")
    overlap = (person & vehicle) | ((person | vehicle) & excluded)
    if overlap:
        raise ValueError("class names belong to more than one group: {}".format(sorted(overlap)))
    default_group = str(config.get("default_group", "obstacle"))
    if default_group not in GROUPS:
        raise ValueError("default_group must be one of {}".format(GROUPS))
    return ClassGroups(person, vehicle, excluded, default_group)
