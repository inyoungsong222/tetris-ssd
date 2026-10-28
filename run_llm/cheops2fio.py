#!/usr/bin/env python3
import argparse
import os
import re
import subprocess
import sys
import time
from array import array

import numpy as np

PAGE = 4096
K_READ, K_WRITE = 1, 2
F_META, F_AHEAD, F_SYNC, F_FUA, F_PREFLUSH = 4, 8, 16, 32, 64

BLK_IO_TRACE_MAGIC = 0x65617400
BLK_IO_TRACE_VERSION = 0x07
BLK_TC_READ, BLK_TC_WRITE, BLK_TC_QUEUE = 1 << 0, 1 << 1, 1 << 4
BLK_TC_SHIFT = 16
BLK_TA_QUEUE = 1


def human(n):
    for u in ("B", "KiB", "MiB", "GiB", "TiB"):
        if n < 1024 or u == "TiB":
            return "%.2f %s" % (n, u)
        n /= 1024.0


def refuse_if_root_disk(major, minor):
    node = None
    link = "/sys/dev/block/%d:%d" % (major, minor)
    if os.path.exists(link):
        node = "/dev/" + os.path.basename(os.path.realpath(link))
    try:
        root = subprocess.run(["findmnt", "-no", "SOURCE", "/"],
                              capture_output=True, text=True).stdout.strip()
    except Exception:
        root = ""
    rootdisk = re.sub(r"p?\d+$", "", root) if root else ""
    if node and root and node in (root, rootdisk):
        sys.exit("--dev-major %d --dev-minor %d is %s, the root disk (%s). Refusing."
                 % (major, minor, node, root))


def rwbs_code(s):
    c = 0
    i = 0
    if len(s) > 1 and s[0] == "F" and s[1] in "RWDNF":
        c |= F_PREFLUSH
        i = 1
    op = s[i] if i < len(s) else ""
    if op == "R":
        c |= K_READ
    elif op == "W":
        c |= K_WRITE
    for ch in s[i + 1:]:
        if ch == "F":
            c |= F_FUA
        elif ch == "A":
            c |= F_AHEAD
        elif ch == "S":
            c |= F_SYNC
        elif ch == "M":
            c |= F_META
    return c


def parse_text(path):
    t = array("q")
    off = array("q")
    nb = array("I")
    code = array("B")
    memo = {}
    bad = 0
    with open(path, "r", errors="replace") as f:
        for line in f:
            p = line.split(", ")
            if len(p) != 5 or not p[0].isdigit():
                bad += 1
                continue
            try:
                ts = int(p[0])
                b = int(p[2])
                sec = int(p[3])
            except ValueError:
                bad += 1
                continue
            c = memo.get(p[1])
            if c is None:
                c = memo[p[1]] = rwbs_code(p[1])
            t.append(ts)
            off.append(sec * 512)
            nb.append(b)
            code.append(c)
    return (np.frombuffer(t, dtype=np.int64), np.frombuffer(off, dtype=np.int64),
            np.frombuffer(nb, dtype=np.uint32).astype(np.int64), np.frombuffer(code, dtype=np.uint8), bad)


def load(path, cache):
    st = os.stat(path)
    if cache and os.path.exists(cache):
        z = np.load(cache)
        if int(z["src_size"]) == st.st_size and int(z["src_mtime_ns"]) == st.st_mtime_ns:
            return z["t"], z["off"], z["nb"], z["code"], "cache"
    t0 = time.time()
    t, off, nb, code, bad = parse_text(path)
    how = "parsed in %.0f s" % (time.time() - t0)
    if cache:
        tmp = cache + ".tmp.npz"
        np.savez(tmp, t=t, off=off, nb=nb, code=code, bad=bad, back=0,
                 src_size=st.st_size, src_mtime_ns=st.st_mtime_ns)
        os.replace(tmp, cache)
        how += ", cached in " + cache
    return t, off, nb, code, how


def build_extents(p0, p1):
    order = np.argsort(p0, kind="stable")
    s = p0[order]
    e = p1[order]
    reach = np.maximum.accumulate(e)
    brk = np.empty(len(s), dtype=bool)
    brk[0] = True
    brk[1:] = s[1:] > reach[:-1]
    idx = np.flatnonzero(brk)
    ext_start = s[idx]
    ext_end = np.append(reach[idx[1:] - 1], reach[-1])
    lens = ext_end - ext_start
    new_start = np.concatenate(([0], np.cumsum(lens)[:-1]))
    return ext_start, ext_end, new_start


def emit_blktrace(path, t_ns, off, nb, is_read, major, minor):
    magic = BLK_IO_TRACE_MAGIC | BLK_IO_TRACE_VERSION
    act_rd = BLK_TA_QUEUE | ((BLK_TC_QUEUE | BLK_TC_READ) << BLK_TC_SHIFT)
    act_wr = BLK_TA_QUEUE | ((BLK_TC_QUEUE | BLK_TC_WRITE) << BLK_TC_SHIFT)
    rec = np.zeros(len(t_ns), dtype=[("magic", "=u4"), ("seq", "=u4"), ("time", "=u8"),
                                      ("sector", "=u8"), ("bytes", "=u4"), ("action", "=u4"),
                                      ("pid", "=u4"), ("device", "=u4"), ("cpu", "=u4"),
                                      ("error", "=u2"), ("pdu_len", "=u2")])
    assert rec.dtype.itemsize == 48, rec.dtype.itemsize
    rec["magic"] = magic
    rec["seq"] = np.arange(len(t_ns), dtype=np.uint32)
    rec["time"] = t_ns
    rec["sector"] = off // 512
    rec["bytes"] = nb
    rec["action"] = np.where(is_read, act_rd, act_wr)
    rec["device"] = (major << 20) | minor
    rec.tofile(path)


def main():
    ap = argparse.ArgumentParser(description="Convert the CHEOPS'25 KV cache offloading block trace into a fio blktrace iolog for one tenant.")
    ap.add_argument("--trace", required=True, help="bpftrace text (opt-6.7b-kv-offload-bs-64-ext4-bpftrace-block.txt)")
    ap.add_argument("--cache", default=None, help="parse cache (default: <trace>.npz, 'none' to disable)")
    ap.add_argument("--file-gib", type=float, required=True, help="size of the tenant file in GiB")
    ap.add_argument("--window-start", type=float, default=0.0, help="window start in trace seconds")
    ap.add_argument("--window-len", type=float, default=0.0, help="window length in seconds (0 = to the end)")
    ap.add_argument("--rotate", default="0",
                    help="phase within the window: a fraction in [0, 1), or 'i/N' for tenant i of N = (i-1)/N")
    ap.add_argument("--time-scale", type=int, default=100,
                    help="trace speed in percent; timestamps are multiplied by 100 / time-scale")
    ap.add_argument("--blktrace", required=True, help="output blktrace iolog")
    ap.add_argument("--dev-major", type=int, required=True,
                    help="device major written into the records (must not be the root disk)")
    ap.add_argument("--dev-minor", type=int, required=True, help="device minor written into the records")
    a = ap.parse_args()

    refuse_if_root_disk(a.dev_major, a.dev_minor)
    if "/" in a.rotate:
        num, den = a.rotate.split("/", 1)
        rot_frac = (float(num) - 1.0) / float(den)
    else:
        rot_frac = float(a.rotate)
    if not 0.0 <= rot_frac < 1.0:
        sys.exit("--rotate must be in [0, 1) or i/N with 1 <= i <= N: %s" % a.rotate)
    if a.time_scale < 1:
        sys.exit("--time-scale must be at least 1: %d" % a.time_scale)

    cache = None if a.cache == "none" else (a.cache or a.trace + ".npz")
    t, off, nb, code, how = load(a.trace, cache)

    kind = code & 3
    is_rw = (kind == K_READ) | (kind == K_WRITE)
    keep = is_rw & (nb != 0) & ((code & F_META) == 0)
    tk = t[keep]
    ok = off[keep]
    nk = nb[keep]
    rk = kind[keep] == K_READ
    if len(tk) == 0:
        sys.exit("no Read or Write data requests in the trace")

    p0 = ok // PAGE
    p1 = (ok + nk + PAGE - 1) // PAGE
    ext_s, ext_e, new_s = build_extents(p0, p1)
    t0 = int(tk.min())
    tr = tk - t0
    span = int(tr.max()) + 1
    w0 = int(a.window_start * 1e9)
    wl = int(a.window_len * 1e9) if a.window_len > 0 else span - w0
    if w0 >= span or wl <= 0:
        sys.exit("window outside the trace (trace %.1f s, window %.1f + %.1f s)"
                 % (span / 1e9, a.window_start, wl / 1e9))
    foot_pg = int((ext_e - ext_s).sum())
    file_pg = int(a.file_gib * (1 << 30)) // PAGE
    if foot_pg > file_pg:
        sys.exit("trace footprint %s is larger than the tenant file %s"
                 % (human(foot_pg * PAGE), human(file_pg * PAGE)))
    k = np.searchsorted(ext_s, p0, side="right") - 1
    new_off = (new_s[k] + (p0 - ext_s[k])) * PAGE
    new_nb = (p1 - p0) * PAGE

    sel = (tr >= w0) & (tr < w0 + wl)
    tw0 = tr[sel] - w0
    rot = int(round(rot_frac * wl)) % wl
    tw = (tw0 - rot) % wl if rot else tw0
    o = np.argsort(tw, kind="stable")
    tw = tw[o]
    if a.time_scale != 100:
        tw = tw * 100 // a.time_scale
    wo = new_off[sel][o]
    wn = new_nb[sel][o]
    wr = rk[sel][o]
    if len(tw) == 0:
        sys.exit("no requests in the window")

    emit_blktrace(a.blktrace, tw, wo, wn, wr, a.dev_major, a.dev_minor)

    dur = wl / 1e9 * 100.0 / a.time_scale
    rby = int(wn[wr].sum())
    wby = int(wn[~wr].sum())
    print("trace      %s (%s)" % (os.path.basename(a.trace), how))
    print("footprint  %s in a %s tenant file" % (human(foot_pg * PAGE), human(file_pg * PAGE)))
    print("window     %.1f-%.1f s, rotated by %.3f (%.1f s), replayed in %.1f s"
          % (w0 / 1e9, (w0 + wl) / 1e9, rot_frac, rot / 1e9, dur))
    print("requests   Read %d (%s, %s/s) . Write %d (%s, %s/s)"
          % (int(wr.sum()), human(rby), human(rby / dur), int((~wr).sum()), human(wby), human(wby / dur)))
    print("output     %s (%s)" % (a.blktrace, human(os.path.getsize(a.blktrace))))


if __name__ == "__main__":
    main()
