"""Development-only request policy. Model runtimes never import this module."""
from .state import STANDBY, PREPARE, ACTIVE


class RegionExample:
    def __init__(self, config):
        self.config = config
        self.recovery_since = None
        self.recovery_stamp = None
        self.lost = False
        self.was_shadow = False

    def decide(self, context, zones, speed, now, source_now):
        modes = {name: int(spec.get('initial_mode', STANDBY))
                 for name, spec in self.config['models'].items()}
        valid = bool(context and context.pose_valid and context.lane_valid and
                     context.route_valid and 0 <= source_now - context.pose_stamp.to_sec()
                     <= self.config['context_freshness_sec'] and
                     0 <= source_now - context.header.stamp.to_sec()
                     <= self.config['context_freshness_sec'])
        if not valid:
            self.lost = True
            self.recovery_since = self.recovery_stamp = None
            modes.update(self.config['gps_loss_modes'])
            return modes, 'missing_or_invalid_context_no_zone_inferred'
        station = context.route_station_m
        for name, spec in self.config['models'].items():
            if spec.get('always_active', False):
                modes[name] = ACTIVE
                continue
            configured = [zones[z] for z in spec.get('zone_ids', []) if z in zones]
            if len(configured) != len(spec.get('zone_ids', [])):
                return modes, 'configured_zone_unavailable'
            lead = max(self.config['prepare_lead_sec'],
                       float(spec.get('measured_prepare_p99_sec', 0.)) + 1.)
            distance = max(self.config['prepare_min_distance_m'],
                           abs(speed) * lead + self.config['prepare_margin_m'])
            modes[name] = STANDBY
            for start, end in configured:
                if start <= station <= end:
                    modes[name] = ACTIVE
                elif 0 < start - station <= distance and modes[name] != ACTIVE:
                    modes[name] = PREPARE
            if name == 'twinlite' and modes[name] == ACTIVE:
                self.was_shadow = True
        if self.lost:
            stamp = context.pose_stamp.to_nsec()
            if self.recovery_since is None:
                self.recovery_since, self.recovery_stamp = now, stamp
            elif stamp > self.recovery_stamp:
                self.recovery_stamp = stamp
            else:
                # Repeated health headers do not constitute pose recovery.
                self.recovery_since = now
            lane_spec = self.config['models']['twinlite']
            ends = [zones[z][1] for z in lane_spec.get('zone_ids', []) if z in zones]
            outside = (not self.was_shadow or any(
                station >= end + self.config['recovery_exit_margin_m'] for end in ends))
            if now - self.recovery_since < self.config['recovery_hold_sec'] or not outside:
                modes['twinlite'] = ACTIVE
            else:
                self.lost = self.was_shadow = False
        return modes, 'development_example_requests_only'
