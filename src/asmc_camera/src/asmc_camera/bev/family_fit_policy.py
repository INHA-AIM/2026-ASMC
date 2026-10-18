"""Immutable, per-frame width policy for the source-time IMU experiment.

The fixed-plane fallback always retains the original fitter configuration.
Only a complete three-view attitude correction may select the relaxed prior.
"""
from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping

from .shared_lane_family import DEFAULT_SHARED_FAMILY_CONFIG


@dataclass(frozen=True)
class FrameFitConfig:
    parameters: Mapping
    mode: str
    reason: str

    def audit(self):
        return dict(mode=self.mode, reason=self.reason,
                    width_prior_strength=float(self.parameters['width_prior_strength']))


class FamilyFitPolicy:
    """Construct once; selections cannot mutate this or another frame's config."""
    def __init__(self, parameters, corrected_width_prior_strength=0.0):
        baseline = dict(DEFAULT_SHARED_FAMILY_CONFIG)
        baseline.update(parameters)
        # The shared-family fitter currently accepts scalar settings only.
        # Reject mutable additions rather than falsely promising deep immutability.
        if any(not isinstance(v, (str, bool, int, float)) for v in baseline.values()):
            raise ValueError('family fit policy requires scalar parameters')
        original = float(baseline['width_prior_strength'])
        corrected = float(corrected_width_prior_strength)
        if not math.isfinite(original) or original < 0:
            raise ValueError('nominal width prior must be finite and non-negative')
        if not math.isfinite(corrected) or not 0 <= corrected <= original:
            raise ValueError('corrected width prior must be finite and within [0, nominal]')
        self.nominal = MappingProxyType(baseline)
        self.corrected = MappingProxyType(dict(baseline, width_prior_strength=corrected))

    def select(self, imu_applied, reason='disabled'):
        return FrameFitConfig(self.corrected if imu_applied else self.nominal,
                              'M1_source' if imu_applied else 'M0', reason)

    def contract(self):
        return dict(policy='all_views_source_imu_width_prior_v1',
                    nominal_width_prior_strength=float(self.nominal['width_prior_strength']),
                    corrected_width_prior_strength=float(self.corrected['width_prior_strength']),
                    fallback='original_fixed_plane_parameters')
