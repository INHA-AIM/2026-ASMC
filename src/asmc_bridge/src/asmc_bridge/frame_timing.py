"""Bounded, ROS-independent validation of raw source clocks and UDP receipts.

Raw values are never repaired using ROS/receive time. Validation is independent
of the operational sensor header policy. One tracker belongs to one receiver.
"""
from collections import OrderedDict
from dataclasses import dataclass


@dataclass(frozen=True)
class PacketReceipt:
    unix_ns: int
    monotonic_ns: int
    kernel: bool = False
    ancillary_truncated: bool = False


class SourceClockTracker:
    def __init__(self, capacity=128):
        if capacity < 1:
            raise ValueError('source history capacity must be positive')
        self.capacity = capacity
        self.latest = None
        self.recent = OrderedDict()

    def observe(self, value, valid, signature=None, period=None):
        duplicate = bool(valid and value in self.recent)
        conflict = duplicate and signature is not None and self.recent[value] != signature
        regression = False
        rollover = False
        if valid and self.latest is not None and value < self.latest:
            # A backwards device tick is a wrap only near the actual hour boundary.
            rollover = bool(period and self.latest > period - 10000000 and value < 10000000)
            regression = not rollover
        if (valid and period and self.latest is not None and self.latest < 10000000
                and value > period - 10000000):
            regression = True  # delayed packet from the previous hour
        if valid and not regression:
            self.latest = value
        if valid:
            if not duplicate:
                self.recent[value] = signature
            self.recent.move_to_end(value)
            while len(self.recent) > self.capacity:
                self.recent.popitem(last=False)
        return dict(source_valid=bool(valid), source_duplicate=duplicate,
                    source_conflict=bool(conflict), source_regression=regression,
                    device_hour_rollover=rollover)

    def sec_nsec(self, sec, nsec, signature=None):
        valid = int(sec) > 0 and 0 <= int(nsec) < 1000000000
        return self.observe(int(sec) * 1000000000 + int(nsec), valid, signature)

    def device(self, ticks):
        period = 3600000000
        previous_endpoint = self.latest
        result = dict(source_valid=bool(ticks), source_duplicate=False,
                      source_conflict=False, source_regression=False,
                      device_hour_rollover=False, device_tick_duplicates=0)
        for tick in ticks:
            valid = tick is not None and 0 <= tick < period
            row = self.observe(tick, valid, period=period)
            result['source_valid'] &= row['source_valid']
            result['device_tick_duplicates'] += int(row['source_duplicate'])
            for key in ('source_regression', 'device_hour_rollover'):
                result[key] |= row[key]
        result['source_duplicate'] = bool(result['source_valid'] and ticks[-1] == previous_endpoint)
        return result


def receipt_fields(receipts):
    """Small immutable frame summary; no payloads or unbounded journals."""
    if not receipts or any(value is None for value in receipts):
        raise ValueError('each accepted packet needs its own receipt')
    first, last = receipts[0], receipts[-1]
    return dict(rx_first_unix_ns=first.unix_ns, rx_last_unix_ns=last.unix_ns,
                rx_first_monotonic_ns=first.monotonic_ns,
                rx_last_monotonic_ns=last.monotonic_ns,
                rx_userspace_fallback_packets=sum(not value.kernel for value in receipts),
                ancillary_truncated_packets=sum(value.ancillary_truncated for value in receipts),
                packet_count=len(receipts))


def publish_sensor(publisher, message):
    """Return the *serialized sensor* sequence, not this metadata's sequence.

    rospy skips serialization without connections. Its serialization mutates
    header.seq; hold the same reentrant publisher lock across observing it.
    A wire key is unavailable if there was no serialization. Failed writes may
    still serialize: a missing sensor at the consumer remains a missing join.
    """
    impl = publisher.impl
    impl.acquire()
    try:
        previous = impl.seq
        publisher.publish(message)
        valid = impl.seq == previous + 1
        return int(message.header.seq) if valid else 0, valid
    finally:
        impl.release()
