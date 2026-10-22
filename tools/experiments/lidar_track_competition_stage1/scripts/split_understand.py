#!/usr/bin/env python3
"""Split the understand-workflow output into readable per-section text files."""
import json, os, sys
src = sys.argv[1]
outdir = sys.argv[2]
os.makedirs(outdir, exist_ok=True)
d = json.load(open(src))['result']

def dump(name, obj):
    path = os.path.join(outdir, name)
    with open(path, 'w') as f:
        if isinstance(obj, dict):
            for k, v in obj.items():
                f.write(f'### {k}\n')
                if isinstance(v, (dict, list)):
                    f.write(json.dumps(v, ensure_ascii=False, indent=1))
                else:
                    f.write(str(v))
                f.write('\n\n')
        else:
            f.write(json.dumps(obj, ensure_ascii=False, indent=1))
    print(path, os.path.getsize(path))

for e in d['lineage']:
    dump(f"lineage_{e['exp_id']}.txt", e)
for k, v in d['topics'].items():
    if v is None:
        print('EMPTY topic', k); continue
    # flatten items
    path = os.path.join(outdir, f'topic_{k}.txt')
    with open(path, 'w') as f:
        f.write('### summary_ko\n' + v.get('summary_ko', '') + '\n\n')
        f.write('### call_order_ko\n' + str(v.get('call_order_ko', '')) + '\n\n')
        for i, it in enumerate(v.get('items', [])):
            f.write(f"### item {i}: {it.get('topic')}  [{it.get('status')}]\n")
            f.write(it.get('answer_ko', '') + '\n')
            f.write('refs: ' + '; '.join(it.get('code_refs', [])) + '\n')
            if it.get('open_questions'):
                f.write('open: ' + it['open_questions'] + '\n')
            f.write('\n')
        for i, r in enumerate(v.get('risks_or_bugs', [])):
            f.write(f"### RISK {i}: {r.get('title')} sev={r.get('severity')} conf={r.get('confidence')}\n")
            f.write(r.get('detail_ko', '') + '\n')
            f.write('refs: ' + '; '.join(r.get('code_refs', [])) + '\n\n')
        f.write('### artifacts\n' + '\n'.join(v.get('artifacts', []) or []) + '\n')
    print(path, os.path.getsize(path))
dump('reconcile.txt', d['reconcile'])
