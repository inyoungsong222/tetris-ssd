#!/usr/bin/env python3
import argparse
import csv
import glob
import os
import re
import sys

import numpy as np

PCTS = (50, 75, 90, 95, 99)
FIELDS = ['rw', 'tenant', 'ios', 'bw_MiBps', 'mean_us'] + [f'p{p}_us' for p in PCTS]


def load(path):
    with open(path, 'rb') as f:
        first = f.readline()
        ncol = first.count(b',') + 1
        data = first + f.read()
    a = np.array(data.replace(b',', b' ').split(), dtype=np.int64)
    return a.reshape(-1, ncol)


def row(rw, tenant, lat_ns, nbytes, secs):
    us = lat_ns / 1000.0
    r = {'rw': rw, 'tenant': tenant, 'ios': len(us),
         'bw_MiBps': f'{nbytes / secs / 2**20:.1f}', 'mean_us': f'{us.mean():.1f}'}
    for p, v in zip(PCTS, np.percentile(us, PCTS)):
        r[f'p{p}_us'] = f'{v:.1f}'
    return r


def main():
    ap = argparse.ArgumentParser(description='Summarize the per-I/O latency logs of one run into a CSV.')
    ap.add_argument('results', nargs='?', help='results_<TS> directory (default: the newest results_* here)')
    args = ap.parse_args()

    d = args.results
    if d is None:
        cands = sorted(glob.glob('results_*/'))
        if not cands:
            sys.exit('no results_* directory here; give the directory as an argument')
        d = cands[-1]
    logs = {}
    for p in glob.glob(os.path.join(d, 'lat_lat.*.log')):
        m = re.search(r'lat_lat\.(\d+)\.log$', p)
        if m:
            logs[int(m.group(1))] = p
    if not logs:
        sys.exit(f'no lat_lat.<n>.log in {d}')

    per = {}
    secs = 0.0
    for t in sorted(logs):
        a = load(logs[t])
        s = (a[:, 0].max() + 1) / 1000.0
        secs = max(secs, s)
        per[t] = (a, s)

    rows = []
    for code, rw in ((0, 'read'), (1, 'write')):
        lat = []
        nbytes = 0
        for t, (a, _) in per.items():
            sel = a[:, 2] == code
            lat.append(a[sel, 1])
            nbytes += int(a[sel, 3].sum())
        lat = np.concatenate(lat)
        if not len(lat):
            continue
        rows.append(row(rw, 'all', lat, nbytes, secs))
        for t, (a, s) in per.items():
            sel = a[:, 2] == code
            if sel.any():
                rows.append(row(rw, t, a[sel, 1], int(a[sel, 3].sum()), s))

    out = os.path.join(d, 'summary.csv')
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    w = csv.DictWriter(sys.stdout, fieldnames=FIELDS)
    w.writeheader()
    w.writerows(rows)
    print(f'written to {out}', file=sys.stderr)


if __name__ == '__main__':
    main()
