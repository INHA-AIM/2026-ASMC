"""Provenance-aware HD-map alignment and transfer candidates.

The package deliberately does not import or mutate ``src/asmc_hd_map/development/won_tae_link_node``.  MGeo is
treated as an input adapter; operational map features use the canonical schema
defined by this package.
"""

from .frames import LocalMapFrame, origin_translation
from .schema import SCHEMA_VERSION, validate_canonical_document

__all__ = [
    "LocalMapFrame",
    "SCHEMA_VERSION",
    "origin_translation",
    "validate_canonical_document",
]
