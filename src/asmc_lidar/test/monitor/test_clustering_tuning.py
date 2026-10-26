#!/usr/bin/env python3
"""Control-path tests: invalid values, stale/disconnected/pending state and HTTP write guards."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import asmc_lidar_monitor_core as core
from asmc_lidar_clustering_tuning import ClusteringTuning, validate

DEFAULT = dict(NearRange=10.0, FarRange=50.0, NearEps=1.30, FarEps=1.32,
               RearNearEps=1.60, RearFarEps=1.62)


def backend():
    b = ClusteringTuning.__new__(ClusteringTuning)
    b.lock = threading.Lock()
    b.token, b.pending = 'test-session-token', None
    b.last_error = ''
    b.received = time.monotonic()
    b.status = dict(current=DEFAULT, startup=DEFAULT, enabled=True, adaptive=True, state='applied')
    b.sent = []
    b.publisher = SimpleNamespace(get_num_connections=lambda: 1, publish=b.sent.append)
    b.String = SimpleNamespace
    return b


class TuningTests(unittest.TestCase):
    def test_validation_rejects_hidden_fallbacks_and_nonfinite(self):
        self.assertEqual(validate(DEFAULT), DEFAULT)
        for delta in (dict(NearRange=0), dict(FarRange=8.5), dict(FarRange=201),
                      dict(NearEps=0), dict(FarEps=0.2), dict(NearEps=float('nan')),
                      dict(FarEps=float('inf')), dict(NearRange=True),
                      dict(RearNearEps=0), dict(RearFarEps=1.59),
                      dict(RearNearEps=float('nan')), dict(RearFarEps=float('inf'))):
            with self.assertRaises(ValueError):
                validate(dict(DEFAULT, **delta))

    def test_legacy_or_partial_four_value_request_cannot_silently_change_rear(self):
        with self.assertRaises(ValueError):
            validate({k: DEFAULT[k] for k in ('NearRange', 'FarRange', 'NearEps', 'FarEps')})
        b = backend()
        b.submit(DEFAULT)
        sent = json.loads(b.sent[0].data)
        self.assertEqual(sent['RearNearEps'], 1.60)
        self.assertEqual(sent['RearFarEps'], 1.62)

    def test_publication_is_pending_until_pipeline_ack(self):
        b = backend()
        result = b.submit(DEFAULT)
        self.assertEqual(result['state'], 'pending')
        self.assertIsNotNone(b.snapshot()['pending'])
        with self.assertRaises(ValueError):
            b.submit(DEFAULT)
        ack = dict(b.status, request_id=result['request_id'], state='waiting_for_valid_scan')
        b.on_status(SimpleNamespace(data=json.dumps(ack)))
        self.assertIsNotNone(b.snapshot()['pending'])
        ack['state'] = 'applied'
        b.on_status(SimpleNamespace(data=json.dumps(ack)))
        self.assertIsNone(b.snapshot()['pending'])

    def test_disconnected_or_stale_never_publishes(self):
        b = backend()
        b.received -= 4
        with self.assertRaises(ValueError): b.submit(DEFAULT)
        b.received = time.monotonic()
        b.publisher.get_num_connections = lambda: 0
        with self.assertRaises(ValueError): b.submit(DEFAULT)
        self.assertEqual(b.sent, [])

    def test_rejection_and_node_restart_release_unconfirmed_request(self):
        b = backend()
        r = b.submit(DEFAULT)
        b.on_status(SimpleNamespace(data=json.dumps(dict(b.status, rejected_request_id=r['request_id'], rejection_error='busy'))))
        self.assertIsNone(b.pending)
        self.assertEqual(b.last_error, 'busy')
        b.submit(DEFAULT)
        b.on_status(SimpleNamespace(data=json.dumps(dict(b.status, producer_session_id='new-session'))))
        self.assertIsNone(b.pending)
        self.assertIn('재시작', b.last_error)

    def test_http_opt_in_token_origin_and_validation(self):
        b = backend()
        with tempfile.TemporaryDirectory() as directory:
            state = core.MonitorState(core.SceneStore())
            server = core.MonitorServer('127.0.0.1', 0, state, directory, clustering_tuning=b)
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            try:
                base = 'http://127.0.0.1:%d' % server.server_port
                def post(values, token=None, origin=None):
                    headers = {'Content-Type': 'application/json'}
                    if token: headers['X-Clustering-Token'] = token
                    if origin: headers['Origin'] = origin
                    request = urllib.request.Request(base+'/api/clustering', json.dumps(values).encode(), headers, method='POST')
                    try:
                        with urllib.request.urlopen(request, timeout=2) as r: return r.status, json.load(r)
                    except urllib.error.HTTPError as r: return r.code, json.load(r)
                self.assertEqual(post(DEFAULT)[0], 400)
                self.assertEqual(post(DEFAULT, b.token, 'http://unrelated.invalid')[0], 400)
                self.assertEqual(post(dict(DEFAULT, FarEps=0.1), b.token)[0], 400)
                self.assertEqual(b.sent, [])
                status, reply = post(DEFAULT, b.token, base)
                self.assertEqual(status, 202)
                self.assertEqual(reply['state'], 'pending')
                self.assertEqual(len(b.sent), 1)
            finally:
                server.shutdown(); server.server_close(); worker.join(timeout=2)


if __name__ == '__main__': unittest.main()
