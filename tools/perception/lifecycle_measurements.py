"""Pure analysis of lifecycle traces and source-preserving output observations."""
def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * q / 100.
    low = int(position)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


def rate_report(rows, started, ended, cutoff=0):
    if ended <= started:
        raise ValueError('non-positive observation window')
    received = [(t, stamp, valid) for t, stamp, valid in rows if started <= t < ended]
    normal = [(t, stamp, valid) for t, stamp, valid in received if stamp > cutoff]
    stamps = [stamp for _, stamp, _ in normal]
    times = [t for t, _, _ in normal]
    windows = [started + 5 * i for i in range(int((ended-started + 1e-6)/5))]
    return dict(messages=len(times), hz=len(times)/(ended-started),
                valid_messages=sum(valid for _, _, valid in normal),
                excluded_old_epoch_messages=len(received)-len(normal),
                first_second=sum(t < started+1 for t in times),
                five_second_hz=[sum(a <= t < a+5 for t in times)/5 for a in windows],
                max_gap_sec=max(b-a for a,b in zip([started]+times, times+[ended])),
                first_output_sec=times[0]-started if times else None,
                duplicate_stamps=len(stamps)-len(set(stamps)),
                reversed_stamps=sum(b<a for a,b in zip(stamps,stamps[1:])))


def trace_trial(request, trace, rows=None, ended=None):
    """Use the server acceptance event, bounded by the actual RPC interval."""
    events = [r for r in trace if r.get('generation') == request['generation']]
    accepted = [r for r in events if r['event'] == 'request_accepted'
                and r['mode'] == request['mode']
                and request['call_started'] <= r['monotonic'] <= request['call_returned']]
    if len(accepted) != 1:
        raise ValueError('missing/ambiguous server acceptance for generation')
    anchor = accepted[0]['monotonic']
    events = [r for r in events if r['monotonic'] >= anchor]
    def first(kind, predicate=lambda r: True):
        return next((r['monotonic']-anchor for r in events
                     if r['event'] == kind and predicate(r)), None)
    result = dict(request, accepted_monotonic=anchor,
                  source_cutoff_ns=accepted[0]['source_cutoff_ns'],
                  first_normal_processing_sec=first('complete'),
                  actual_image_ready_sec=first('complete', lambda r: all(n >= 3 for n in r['hits'].values())),
                  status_prepared_sec=first('status', lambda r: r['prepared']),
                  pipeline_ready_sec=first('status', lambda r: r['pipeline_ready']))
    if rows is not None:
        result['outputs'] = {name: rate_report(values, anchor, ended, result['source_cutoff_ns'])
                             for name, values in rows.items()}
    return result


def trace_integrity(rows):
    return (bool(rows) and rows[-1]['event'] == 'trace_end'
            and rows[-1]['dropped'] == 0 and
            sum(r['event'] == 'trace_end' for r in rows) == 1)


def ten_hz_pass(row):
    return (row['first_second'] >= 9 and bool(row['five_second_hz'])
            and all(hz >= 9.5 for hz in row['five_second_hz'])
            and row['max_gap_sec'] <= .25 and not row['duplicate_stamps']
            and not row['reversed_stamps'])
