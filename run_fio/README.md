# Synthetic latency experiments

The paper's synthetic latency experiments with 4 tenants:

| File | Workload |
|---|---|
| `random.fio` | uniform random |
| `sequential.fio` | sequential |
| `precond.fio` | preconditioning, used before either workload |
| `lat_summary.py` | summarizes the latency logs of a run as a CSV |

## Setup

All commands below run from the `run_fio/` directory of this repository.

Build the emulator module and point `FIO` to fio 3.43:

```bash
make -C ..
FIO=$(command -v fio)
```

Load the module (memory reserved with `memmap=112G$16G`, see `../README.md`):

```bash
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

## Run

```bash
JOB=random.fio                  # or sequential.fio
sudo -E $FIO --section=prefill_t1 --section=prefill_t2 --section=prefill_t3 --section=prefill_t4 $JOB
sudo -E $FIO --section=pc_t1 --section=pc_t2 --section=pc_t3 --section=pc_t4 precond.fio
echo 1 | sudo tee /sys/module/nvmev/parameters/gc_accounting_reset
echo 1 | sudo tee /sys/module/nvmev/parameters/dcache_acct_reset
export TS=$(date +%Y%m%d_%H%M%S) && mkdir -p results_$TS
sudo -E $FIO --section=t1 --section=t2 --section=t3 --section=t4 $JOB
```

The measurement step writes per-I/O latency logs to `results_<TS>/` (`lat_lat.<n>.log`,
`n` = tenant; about 280 MB per run). Export `TS` before it. `cpus_allowed` in the job files
keeps fio off the emulator CPUs (`cpus=3,4,5,6,7`); change it to match your machine.

To summarize a run as a CSV (Read and Write throughput, and latency mean, p50, p75, p90,
p95 and p99 over all tenants and per tenant), run `python3 lat_summary.py results_<TS>`
(Python 3 with NumPy); it also writes `results_<TS>/summary.csv`.

## Clean up

```bash
for i in 1 2 3 4; do sudo umount $NVMEV_MNT/p$i; done
sudo rmmod nvmev
```
