"""Thread-safe bridge health and cadence; no sensor/control policy changes."""
from collections import deque
import math
import re
import socket
import struct
import threading
import time

from .frame_timing import PacketReceipt


def latency_summary(samples):
    """Summarize an immutable snapshot, never a live receiver-owned deque."""
    values = sorted(float(value) for value in samples)
    if not values:
        return dict(samples=0, mean_ms=0., p50_ms=0., p95_ms=0., max_ms=0.)
    def percentile(percent):
        position = (len(values) - 1) * percent / 100.
        low, high = int(math.floor(position)), int(math.ceil(position))
        return values[low] if low == high else values[low] * (high - position) + values[high] * (position - low)
    return dict(samples=len(values), mean_ms=sum(values) / len(values),
                p50_ms=percentile(50), p95_ms=percentile(95), max_ms=values[-1])


class BridgeTelemetry:
    COUNTERS = ('messages', 'raw_packets', 'parse_errors', 'header_errors',
                'timestamp_fallbacks', 'timestamp_regressions', 'socket_drops',
                'rotation_boundaries', 'discarded_revolutions', 'assembly_timeouts',
                'device_regressions', 'ignored_packets', 'send_errors',
                'unsupported_packets', 'ancillary_truncations',
                'rx_timestamp_fallbacks', 'source_invalid', 'source_duplicates',
                'source_conflicts', 'source_regressions', 'device_hour_rollovers',
                'timing_errors', 'timing_messages', 'incomplete_frames')
    WINDOWS = ('processing', 'assembly', 'gap', 'packets_per_frame',
               'points_per_frame', 'wall_rotation', 'device_rotation')

    def __init__(self, clock=time.monotonic, capacity=4096):
        self.clock, self.capacity = clock, capacity
        self.started = clock()
        self.lock = threading.Lock()
        self.streams = {}
        self.previous = {}

    def register(self, name, optional=False, transmit=False, stale_after=.5):
        if not math.isfinite(stale_after) or stale_after <= 0:
            raise ValueError('stale_after must be finite and positive')
        with self.lock:
            row = dict((key, 0) for key in self.COUNTERS)
            row.update((key, deque(maxlen=self.capacity)) for key in self.WINDOWS)
            row.update(optional=optional, transmit=transmit, stale_after=stale_after,
                       last_received=None, receiver='transmit' if transmit else 'starting', receiver_error='',
                       socket_drop_supported=False, socket_rcvbuf=0,
                       socket_timestamp_option=0,
                       unsupported_protocol='', last_unsupported=None,
                       previous_wrap_wall=None, previous_wrap_device=None)
            self.streams[name] = row

    def increment(self, name, counter, amount=1):
        with self.lock:
            self.streams[name][counter] += amount

    def sample(self, name, window, value):
        with self.lock:
            self.streams[name][window].append(value)

    def message(self, name):
        now = self.clock()
        with self.lock:
            row = self.streams[name]
            if row['last_received'] is not None:
                row['gap'].append(max(0., now - row['last_received']) * 1000.)
            row['last_received'] = now
            row['messages'] += 1
            row['unsupported_protocol'] = ''
            row['last_unsupported'] = None

    def unsupported(self, name, protocol):
        with self.lock:
            row = self.streams[name]
            if not row['optional']:
                raise ValueError('required inputs cannot be treated as optional unsupported')
            row['unsupported_packets'] += 1
            row['unsupported_protocol'] = protocol[:64]
            row['last_unsupported'] = self.clock()

    def receiver(self, name, state, error=''):
        with self.lock:
            self.streams[name].update(receiver=state, receiver_error=error)

    def receiver_exited(self, name, shutting_down):
        with self.lock:
            row = self.streams[name]
            if row['receiver'] != 'failed':
                row.update(receiver='stopped' if shutting_down else 'failed',
                           receiver_error='' if shutting_down else 'receiver exited')

    def configure_socket(self, name, sock):
        supported = False
        try:
            sock.setsockopt(socket.SOL_SOCKET, 40, 1)  # Linux SO_RXQ_OVFL
            supported = hasattr(sock, 'recvmsg')
        except OSError:
            pass
        timestamp_option = 0
        if hasattr(sock, 'recvmsg'):
            # Linux NEW uses two signed 64-bit fields, including on 32-bit.
            # OLD is a native long timespec. Python 3.8 need not expose constants.
            for option in (64, 35):
                try:
                    sock.setsockopt(socket.SOL_SOCKET, option, 1)
                    timestamp_option = option
                    break
                except OSError:
                    continue
        with self.lock:
            self.streams[name].update(socket_drop_supported=supported,
                socket_timestamp_option=timestamp_option,
                socket_rcvbuf=sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))

    def receive(self, name, sock, size):
        data, address, _ = self.receive_timed(name, sock, size)
        return data, address

    def receive_timed(self, name, sock, size):
        """Receive once; preserve kernel and monotonic receipt without a new socket."""
        kernel_ns, drops, flags = None, None, 0
        if hasattr(sock, 'recvmsg'):
            data, control, flags, address = sock.recvmsg(
                size, socket.CMSG_SPACE(16) + socket.CMSG_SPACE(4))
            monotonic_ns, unix_ns = time.monotonic_ns(), time.time_ns()
            for level, kind, value in control:
                if level == socket.SOL_SOCKET and kind == 40 and len(value) >= 4:
                    drops = struct.unpack('=I', value[:4])[0]
                if level == socket.SOL_SOCKET and kind in (35, 64):
                    fmt = '=qq' if kind == 64 else '@ll'
                    length = struct.calcsize(fmt)
                    if len(value) >= length:
                        sec, nsec = struct.unpack(fmt, value[:length])
                        if sec > 0 and 0 <= nsec < 1000000000:
                            kernel_ns = sec * 1000000000 + nsec
        else:
            data, address = sock.recvfrom(size)
            monotonic_ns, unix_ns = time.monotonic_ns(), time.time_ns()
        truncated = bool(flags & socket.MSG_CTRUNC)
        if truncated:
            kernel_ns = None
        receipt = PacketReceipt(kernel_ns if kernel_ns is not None else unix_ns,
                                monotonic_ns, kernel_ns is not None, truncated)
        with self.lock:
            row = self.streams[name]
            row['raw_packets'] += 1
            row['ancillary_truncations'] += int(truncated)
            row['rx_timestamp_fallbacks'] += int(kernel_ns is None)
            if drops is not None:
                row['socket_drops'] = drops
            if flags & socket.MSG_TRUNC:
                row['parse_errors'] += 1
                return None, address, receipt
        return data, address, receipt

    def rotation(self, name, device_us):
        now = self.clock()
        with self.lock:
            row = self.streams[name]
            row['rotation_boundaries'] += 1
            if row['previous_wrap_wall'] is not None:
                row['wall_rotation'].append((now - row['previous_wrap_wall']) * 1000.)
            valid = device_us is not None and 0 <= device_us < 3600000000
            previous = row['previous_wrap_device']
            if valid and previous is not None:
                delta = (device_us - previous) % 3600000000
                if 0 < delta < 10000000:
                    row['device_rotation'].append(delta / 1000.)
                else:
                    row['device_regressions'] += 1
            row['previous_wrap_wall'] = now
            row['previous_wrap_device'] = device_us if valid else None

    def snapshot(self, channel='diagnostics'):
        # Writers hold this lock only for counter/sample updates; all sorting,
        # ROS serialization and logging happens after the bounded copy.
        with self.lock:
            now = self.clock()
            rows = {name: {key: tuple(value) if isinstance(value, deque) else value
                           for key, value in row.items()}
                    for name, row in self.streams.items()}
            previous_time, previous_rows = self.previous.get(channel, (self.started, {}))
            self.previous[channel] = (now, rows)
        interval = max(1e-6, now - previous_time)
        for name, row in rows.items():
            previous = previous_rows.get(name, {})
            for counter in self.COUNTERS:
                row[counter + '_delta'] = max(0, row[counter] - previous.get(counter, 0))
            row['interval_hz'] = row['messages_delta'] / interval
            row['raw_packet_hz'] = row['raw_packets_delta'] / interval
            row['rotation_hz'] = row['rotation_boundaries_delta'] / interval
            row['average_hz'] = row['messages'] / max(1e-6, now - self.started)
            row['last_message_age_sec'] = (-1. if row['last_received'] is None
                else max(0., now - row['last_received']))
            row['last_unsupported_age_sec'] = (-1. if row['last_unsupported'] is None
                else max(0., now - row['last_unsupported']))
            row['health'] = stream_health(row)
        return now, interval, rows


def stream_health(row):
    if row['receiver'] in ('failed', 'stopped'):
        return 'ERROR'
    if row['transmit']:
        if row.get('send_errors_delta', 0):
            return 'DEGRADED'
        return 'IDLE' if row['last_message_age_sec'] < 0 or row['last_message_age_sec'] > row['stale_after'] else 'OK'
    if row['optional'] and row.get('unsupported_protocol'):
        if any(row.get(key + '_delta', 0) for key in ('parse_errors', 'header_errors', 'socket_drops')):
            return 'INVALID'
        return ('UNSUPPORTED' if row['last_unsupported_age_sec'] <= row['stale_after']
                else 'IDLE')
    if not row['messages']:
        if row['parse_errors'] or row['header_errors']:
            return 'INVALID'
        return 'IDLE' if row['optional'] else 'WAITING'
    if row['last_message_age_sec'] > row['stale_after']:
        return 'STALE'
    if any(row.get(key + '_delta', 0) for key in (
            'parse_errors', 'header_errors', 'socket_drops', 'ancillary_truncations',
            'source_conflicts', 'timing_errors')):
        return 'DEGRADED'
    return 'OK'


def short_link(link):
    """Shorten only the known K-City ID pattern; diagnostics keeps full IDs."""
    match = re.fullmatch(r'A2256W(\d{6})', link)
    return 'A{}'.format(int(match[1])) if match else link or '-'


def console_summary(rows, interval, network=False, ego_link='', competition_link=''):
    """Render one multiline block with a name and unit for each active stream.

    Rates retain their measured interval; the console need not print that interval.
    """
    shown = set()

    def rate(name):
        shown.add(name)
        row = rows[name]
        value = 'idle' if row['health'] == 'IDLE' else '{:.1f}Hz'.format(row['interval_hz'])
        return '{}={}'.format(name, value)

    def group(names):
        return '  '.join(rate(name) for name in names if name in rows)

    if network:
        parts = ['[NETWORK]']
        rx = group(('ego', 'col', 'obj'))
        competition = rows.get('competition')
        if competition and competition['health'] not in ('IDLE', 'UNSUPPORTED'):
            rx = '  '.join(value for value in (rx, rate('competition')) if value)
        shown.add('competition')
        if rx:
            parts.append('  RX ' + rx)
        links = ['link=' + short_link(ego_link)]
        if competition_link and competition and competition['health'] in ('OK', 'DEGRADED'):
            links.append('competition_link=' + short_link(competition_link))
        if 'cmd' in rows:
            parts.append('  TX ' + rate('cmd') + '  | ' + '  '.join(links))
        else:
            parts.append('  ' + '  '.join(links))
    else:
        parts = ['[SENSORS]']
        for names in (('cam1', 'cam2', 'cam3', 'cam4'),
                      ('gps', 'imu', 'lidar', 'lidar_instance'),
                      ('sem_front', 'sem_left', 'sem_right')):
            value = group(names)
            if value:
                parts.append('  ' + value)
    other = group([name for name in sorted(rows) if name not in shown])
    if other:
        parts.append('  ' + other)
    issues = []
    for name, row in sorted(rows.items()):
        if row['health'] not in ('OK', 'IDLE', 'UNSUPPORTED'):
            description = name + ':' + row['health']
            if row['health'] == 'STALE':
                description += '({:.1f}s)'.format(row['last_message_age_sec'])
            issues.append(description)
    state = 'state=' + (','.join(issues) if issues else 'OK')
    errors = sum(row[key + '_delta'] for row in rows.values()
                 for key in ('parse_errors', 'header_errors', 'send_errors',
                             'ancillary_truncations', 'source_conflicts', 'timing_errors'))
    receivers = [row for row in rows.values() if not row['transmit']]
    drop_known = all(row['socket_drop_supported'] for row in receivers)
    drops = str(sum(row['socket_drops_delta'] for row in receivers)) if drop_known else '?'
    parts.append('  {}  | err +{}  | drop +{}'.format(state, errors, drops))
    return '\n'.join(parts)


class PeriodicReporter:
    """Steady-clock reporter that survives a failed diagnostic callback."""
    def __init__(self, callback, period, on_error):
        if not math.isfinite(period) or period <= 0:
            raise ValueError('diagnostics_period must be finite and positive')
        self.callback, self.period, self.on_error = callback, period, on_error
        self.stop_event = threading.Event()
        self.failures = 0
        self.thread = threading.Thread(target=self._run, name='bridge-diagnostics', daemon=True)
        self.thread.start()

    def tick(self):
        try:
            self.callback(None)
        except Exception as error:
            self.failures += 1
            self.on_error(error)

    def _run(self):
        deadline = time.monotonic() + self.period
        while not self.stop_event.wait(max(0., deadline - time.monotonic())):
            self.tick()
            # Skip missed periods instead of creating a catch-up burst.
            deadline += self.period
            if deadline <= time.monotonic():
                deadline = time.monotonic() + self.period

    def stop(self):
        self.stop_event.set()
        if threading.current_thread() is not self.thread:
            self.thread.join(timeout=2.)


def start_reporting(owner, namespace):
    import rospy
    owner.log_period = float(rospy.get_param('~log_period', 5.))
    period = float(rospy.get_param('~diagnostics_period', 1.))
    if not math.isfinite(owner.log_period) or owner.log_period < period:
        raise ValueError('log_period must be finite and >= diagnostics_period')
    owner._last_console = owner.telemetry.started
    owner._next_console = owner.telemetry.started + owner.log_period
    owner._health = {}
    owner.reporter = PeriodicReporter(owner._report, period,
        lambda error: rospy.logerr_throttle(5., '%s diagnostics failed; retrying: %s', namespace, error))
    rospy.on_shutdown(owner.reporter.stop)


def publish_report(owner, namespace):
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    now, interval, rows = owner.telemetry.snapshot()
    message = DiagnosticArray()
    message.header.stamp = rospy.Time.now()
    for name, row in sorted(rows.items()):
        state = row['health']
        old_state = owner._health.get(name)
        if old_state != state:
            if state in ('STALE', 'INVALID', 'ERROR', 'DEGRADED'):
                rospy.logwarn('%s %s: %s age=%.3fs errors=%d %s', namespace, name,
                    state, row['last_message_age_sec'],
                    sum(row[key] for key in ('parse_errors', 'header_errors', 'send_errors',
                                            'ancillary_truncations', 'source_conflicts', 'timing_errors')),
                    row['receiver_error'])
            elif old_state in ('STALE', 'INVALID', 'ERROR', 'DEGRADED') and state == 'OK':
                rospy.loginfo('%s %s: recovered', namespace, name)
        owner._health[name] = state
        status = DiagnosticStatus(name=namespace + '/' + name, hardware_id='morai_udp')
        status.level = (DiagnosticStatus.ERROR if state == 'ERROR' else
                        DiagnosticStatus.OK if state in ('OK', 'IDLE', 'UNSUPPORTED') else DiagnosticStatus.WARN)
        status.message = dict(OK='receiving' if not row['transmit'] else 'active',
            IDLE='no control command' if row['transmit'] else 'optional; no data',
            WAITING='no data', STALE='stale', INVALID='invalid input',
            UNSUPPORTED='optional; unsupported ' + row['unsupported_protocol'],
            DEGRADED='recent packet or send errors', ERROR='receiver failed')[state]
        if state == 'DEGRADED' and any(row[key + '_delta'] for key in (
                'ancillary_truncations', 'source_conflicts', 'timing_errors')):
            status.message = 'recent sensor timing errors'
        values = dict(runtime_profile=owner.runtime_profile, transport_profile=owner.transport_profile,
            stream_profile=owner.stream_profile, timestamp_mode=owner.timestamp_mode,
            required=str(not row['optional'] and not row['transmit']).lower(),
            optional_stream=str(row['optional']).lower(), direction='ros_to_udp' if row['transmit'] else 'udp_to_ros',
            health=state, messages=row['messages'], average_hz=row['average_hz'],
            interval_hz=row['interval_hz'], report_interval_sec=interval,
            last_message_age_sec=row['last_message_age_sec'], stale_after_sec=row['stale_after'],
            receiver_state=row['receiver'], receiver_error=row['receiver_error'],
            raw_packets=row['raw_packets'], raw_packet_hz=row['raw_packet_hz'],
            parse_errors=row['parse_errors'], header_errors=row['header_errors'],
            ignored_packets=row['ignored_packets'], send_errors=row['send_errors'],
            unsupported_packets=row['unsupported_packets'], unsupported_protocol=row['unsupported_protocol'],
            assembly_timeouts=row['assembly_timeouts'],
            last_unsupported_age_sec=row['last_unsupported_age_sec'],
            data_available=str(not row['transmit'] and row['messages'] > 0 and state in ('OK', 'DEGRADED')).lower(),
            timestamp_fallbacks=row['timestamp_fallbacks'], timestamp_regressions=row['timestamp_regressions'],
            socket_drops=row['socket_drops'], socket_drop_supported=str(row['socket_drop_supported']).lower(),
            socket_rcvbuf_bytes=row['socket_rcvbuf'], statistics_window='rolling_last_4096_samples',
            reporter_failures=owner.reporter.failures)
        if namespace == 'morai_udp_bridge':
            if name == 'ego':
                values['last_link_id'] = owner.last_link_id
            elif name == 'competition':
                values['last_link_id'] = owner.last_competition_link_id
        for key in ('socket_timestamp_option', 'ancillary_truncations', 'rx_timestamp_fallbacks',
                    'source_invalid', 'source_duplicates', 'source_conflicts', 'source_regressions',
                    'device_hour_rollovers', 'timing_errors', 'timing_messages', 'incomplete_frames'):
            values[key] = row[key]
            if key + '_delta' in row:
                values[key + '_delta'] = row[key + '_delta']
        for source, prefix in (('processing', 'processing'), ('assembly', 'assembly'), ('gap', 'output_gap')):
            summary = latency_summary(row[source])
            for key, value in summary.items():
                values[prefix + '_' + key] = value
        if name.startswith('lidar'):
            packet_counts = sorted(row['packets_per_frame'])
            count = len(packet_counts)
            values['packets_per_frame_median'] = (0. if not count else (packet_counts[(count - 1) // 2] + packet_counts[count // 2]) / 2.)
            for key in ('rotation_boundaries', 'discarded_revolutions', 'assembly_timeouts', 'device_regressions'):
                values[key] = row[key]
            values['rotation_hz'] = row['rotation_hz']
            for key in ('wall_rotation', 'device_rotation'):
                summary = latency_summary(row[key])
                values[key + '_hz'] = 1000. / summary['mean_ms'] if summary['mean_ms'] > 0 else 0.
                values[key + '_samples'] = summary['samples']
            values['points_per_frame_mean'] = sum(row['points_per_frame']) / max(1, len(row['points_per_frame']))
        status.values = [KeyValue(str(key), '{:.6f}'.format(value) if isinstance(value, float) else str(value))
                         for key, value in sorted(values.items())]
        message.status.append(status)
    owner.pub_diagnostics.publish(message)
    if now >= owner._next_console:
        _, log_interval, log_rows = owner.telemetry.snapshot('console')
        rospy.loginfo('%s', console_summary(log_rows, log_interval,
            network=namespace == 'morai_udp_bridge',
            ego_link=getattr(owner, 'last_link_id', ''),
            competition_link=getattr(owner, 'last_competition_link_id', '')))
        owner._last_console = now
        owner._next_console += (math.floor((now - owner._next_console) / owner.log_period) + 1) * owner.log_period
