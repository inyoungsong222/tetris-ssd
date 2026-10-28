# LLM KV cache offloading latency experiment

The paper's LLM workload with 4 tenants: every tenant replays the block trace of FlexGen
offloading the KV cache of OPT-6.7B (batch 64) to an NVMe SSD, from CHEOPS '25
(Ren et al., "An I/O Characterizing Study of Offloading LLM Models and KV Caches to NVMe SSD").

| File | Purpose |
|---|---|
| `get_cheops25_trace.sh` | download the trace from the CHEOPS '25 repository |
| `cheops2fio.py` | convert the trace into one blktrace iolog per tenant |
| `llm.fio` | replay the iologs, one job per tenant |
| `lat_summary.py` | summarize the latency logs of a run as a CSV |

## Requirements

* The emulator requirements in `../README.md`: Linux 5.15 with its kernel headers, 112 GiB of
  memory reserved at 16 GiB with `memmap=112G$16G`, and five CPUs for the emulator threads.
* `git`, `curl`, `parted`, `mkfs.ext4`, and Python 3 with NumPy.
* A C compiler with the libaio headers to build fio.

All commands below run from the `run_llm/` directory of this repository.

## 1. Get the trace

```bash
./get_cheops25_trace.sh
```

This clones the CHEOPS '25 repository at the commit we used into `cheops25/repo`, checks the
Git LFS pointer, and downloads only the KV cache trace (1.0 GiB) into `cheops25/`, verified by
SHA-256.

## 2. Convert the trace

```bash
mkdir -p cheops25/iolog
for i in 1 2 3 4; do
    python3 cheops2fio.py --trace cheops25/opt-6.7b-kv-offload-bs-64-ext4-bpftrace-block.txt \
        --file-gib 21.75 --window-start 0 --window-len 120 --time-scale 60 --rotate $i/4 \
        --blktrace cheops25/iolog/t$i.blktrace --dev-major 259 --dev-minor 255
done
```

The first run parses the text trace and caches it as `cheops25/*.npz` (632 MB). The device
number written into the records is not used for the replay: `llm.fio` sends every request to
the tenant file with `replay_redirect`. The converter refuses the number of the root disk.

## 3. Build fio

`llm.fio` needs fio 3.43 with two replay options added (`replay_abs_time`,
`replay_idle_sleep`); the source is in `../fio-3.43-abstime`:

```bash
(cd ../fio-3.43-abstime && ./configure && make)
FIO=$(pwd)/../fio-3.43-abstime/fio
```

## 4. Prepare the device

Build and load the emulator module:

```bash
make -C ..
sudo insmod ../nvmev.ko memmap_start=16G memmap_size=112G cpus=3,4,5,6,7
```

Find the emulated device by its model name (`CSL_Virt_MN_*`); do not assume its `/dev`
name:

```bash
lsblk -o NAME,SIZE,MODEL
```

Split the device into four equal partitions (98 GiB in total), make an ext4 file system
on each and mount them as `$NVMEV_MNT/p1` .. `$NVMEV_MNT/p4`:

```bash
DEV=/dev/<emulated device>
export NVMEV_MNT=/mnt/nvmev
sudo parted -s $DEV mklabel gpt mkpart p1 1MiB 25088MiB mkpart p2 25088MiB 50176MiB \
    mkpart p3 50176MiB 75264MiB mkpart p4 75264MiB 100352MiB
for i in 1 2 3 4; do
    sudo mkfs.ext4 -q -F -m 0 ${DEV}p$i
    sudo mkdir -p $NVMEV_MNT/p$i && sudo mount ${DEV}p$i $NVMEV_MNT/p$i
done
```

Register each partition as a tenant (LBAs in 512-byte sectors). TETRIS identifies tenants
by these ranges; without them every I/O belongs to one tenant:

```bash
d=$(basename $DEV)
echo clear | sudo tee /proc/nvmev/tenant_register
for i in 1 2 3 4; do
    echo "$i $(cat /sys/block/$d/${d}p$i/start) $(cat /sys/block/$d/${d}p$i/size)" \
        | sudo tee /proc/nvmev/tenant_register
done
```

## 5. Run

```bash
sudo -E $FIO --section=prefill_t1 --section=prefill_t2 --section=prefill_t3 --section=prefill_t4 ../run_fio/sequential.fio
sudo -E $FIO --section=pc_t1 --section=pc_t2 --section=pc_t3 --section=pc_t4 ../run_fio/precond.fio
echo 1 | sudo tee /sys/module/nvmev/parameters/gc_accounting_reset
echo 1 | sudo tee /sys/module/nvmev/parameters/dcache_acct_reset
export TS=$(date +%Y%m%d_%H%M%S) && mkdir -p results_$TS
sudo -E $FIO llm.fio
```

The replay writes per-I/O latency logs to `results_<TS>/` (`lat_lat.<n>.log`, `n` = tenant;
about 600 MB per run). Export `TS` before the replay. `cpus_allowed` in the job files keeps fio
off the emulator CPUs (`cpus=3,4,5,6,7`); change it to match your machine.

To summarize a run as a CSV (Read and Write throughput, and latency mean, p50, p75, p90,
p95 and p99 over all tenants and per tenant), run `python3 lat_summary.py results_<TS>`;
it also writes `results_<TS>/summary.csv`.

## 6. Clean up

```bash
for i in 1 2 3 4; do sudo umount $NVMEV_MNT/p$i; done
sudo rmmod nvmev
```
