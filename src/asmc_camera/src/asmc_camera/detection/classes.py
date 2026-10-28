"""Normalized object groups for asmc_msgs/CameraObjectBox.class_name."""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Mapping, Optional


GROUPS = ("person", "animal", "vehicle", "obstacle")


@dataclass(frozen=True)
class ClassGroups:
    person: FrozenSet[str]
    vehicle: FrozenSet[str]
    excluded: FrozenSet[str]
    default_group: str = "obstacle"
    animal: FrozenSet[str] = frozenset()
    explicit: Optional[Mapping[str, FrozenSet[str]]] = None

    def group_of(self, source_class_name: str) -> Optional[str]:
        """Return the group for a raw label, or None when it is excluded."""
        key = str(source_class_name).strip().lower()
        if self.explicit is not None:
            return next((g for g, names in self.explicit.items() if key in names), None)
        if key in self.excluded:
            return None
        if key in self.person:
            return "person"
        if key in self.animal:
            return "animal"
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
    if config.get("profile") in ("mission16_v1", "mission15_no_cone_v1"):
        from .mission import GROUPS as MISSION_GROUPS, GROUPS_NO_CONE
        mission_groups = MISSION_GROUPS if config['profile'] == 'mission16_v1' else GROUPS_NO_CONE
        expected = {g: frozenset(names) for g, names in mission_groups.items()}
        actual = {g: _names(v, g) for g, v in groups.items()} if isinstance(groups, Mapping) else {}
        if actual != expected or config.get('default_group') != 'unknown':
            raise ValueError(config['profile']+' requires the fixed approved mapping and unknown default')
        return ClassGroups(frozenset(), frozenset(), frozenset(), 'unknown', explicit=actual)
    if not isinstance(groups, Mapping) or set(groups) not in (
        {"person", "vehicle"}, {"person", "animal", "vehicle"}
    ):
        raise ValueError("groups must define person and vehicle, with optional animal")
    person = _names(groups["person"], "groups.person")
    vehicle = _names(groups["vehicle"], "groups.vehicle")
    animal = _names(groups.get("animal", []), "groups.animal")
    excluded = _names(config.get("excluded", []), "excluded")
    overlap = (person & vehicle) | (person & animal) | (vehicle & animal) | ((person | vehicle | animal) & excluded)
    if overlap:
        raise ValueError("class names belong to more than one group: {}".format(sorted(overlap)))
    default_group = str(config.get("default_group", "obstacle"))
    if default_group not in GROUPS:
        raise ValueError("default_group must be one of {}".format(GROUPS))
    return ClassGroups(person, vehicle, excluded, default_group, animal)
