# TETRIS SSD

This repository contains the SSD emulator used to evaluate TETRIS, a device-side I/O
scheduler and pSLC write placement that mitigate read stalls in a multi-tenant SSD.
It is a Linux kernel module built on [NVMeVirt](README_NVMeVirt.md) (FAST '23) and emulates a
conventional NVMe SSD with a DRAM write-back cache, a TLC/pSLC hybrid FTL and per-die
NAND queues.

## Emulated device

| Item | Value |
|---|---|
| Channels x dies per channel | 8 x 8 (64 dies) |
| Flash page | 16 KiB |
| Pages per block | TLC 112, pSLC 37 |
| Lines | 1,280 (TLC 70 %, pSLC 30 %) |
| Latency (TLC) | read 66 us, program 730 us, erase 4.8 ms |
| Latency (pSLC) | read 30 us, program 160 us, erase 3 ms |
| Channel / PCIe bandwidth | 800 MiB/s per channel / 15,750 MiB/s |
| Over-provisioning | 10 % |
| DRAM cache | `cache_mb` MiB (default 128) |

The geometry and timing are in `ssd_config.h`.

## Requirements

* Linux 5.15 with its kernel headers (tested with 5.15.0-139 on Ubuntu 20.04).
* A physical memory region reserved for the emulated storage. The paper used 112 GiB at
  16 GiB, reserved with the kernel command line `memmap=112G$16G` (in `/etc/default/grub`,
  write it as `memmap=112G\\\$16G`, then `update-grub` and reboot).
* Five CPUs for the emulator threads. The paper isolated them with `isolcpus`.

## Build

```bash
make                 # builds nvmev.ko for 4 tenants
```

## Load

`insmod` needs root. The device appears as a new NVMe namespace; check the model
(`CSL_Virt_MN_*`) with `lsblk -o NAME,MODEL` before using it.

```bash
sudo insmod nvmev.ko memmap_start=16G memmap_size=112G cpus=3,4,5,6,7
```

`cpus` lists the dispatcher CPU, the I/O worker CPUs and, last, the queue-serve CPU.

## Workloads

* `run_fio/`: 4-tenant random and sequential latency experiments (`run_fio/README.md`).
* `run_llm/`: 4-tenant LLM KV cache offloading experiment (`run_llm/README.md`).

## License

GPL-2.0 (see `LICENSE`), as NVMeVirt. The priority queue in `pqueue/` is under the
BSD 2-clause license (see `pqueue/LICENSE`).
