// SPDX-License-Identifier: GPL-2.0-only

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/sched/clock.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#ifdef CONFIG_X86
#include <asm/e820/types.h>
#include <asm/e820/api.h>
#endif

#include "nvmev.h"
#include "conv_ftl.h"
#include "cache_model.h"
#include "dma.h"

struct nvmev_dev *nvmev_vdev = NULL;

static unsigned long memmap_start = 0;
static unsigned long memmap_size = 0;

static unsigned int read_time = 1;
static unsigned int read_delay = 1;
static unsigned int read_trailing = 0;

static unsigned int write_time = 1;
static unsigned int write_delay = 1;
static unsigned int write_trailing = 0;

static unsigned int nr_io_units = 8;
static unsigned int io_unit_shift = 12;

static char *cpus;

int io_using_dma = false;

int gc_target_lines = 16;
module_param(gc_target_lines, int, 0444);
MODULE_PARM_DESC(gc_target_lines, "background GC runs while free lines are below this (default 16)");

int gc_reserve_lines = 1;
module_param(gc_reserve_lines, int, 0444);

int rawp_sh_slots = 4096;
int rawp_sh_sweep = 128;
module_param(rawp_sh_slots, int, 0444);
module_param(rawp_sh_sweep, int, 0444);
int rawp_occ_r_ppk = 900;
module_param(rawp_occ_r_ppk, int, 0444);

int gc_force_lines = 1;

int gc_retry_max = 8;
module_param(gc_retry_max, int, 0444);
MODULE_PARM_DESC(gc_retry_max, "max TLC GC retries when the TLC pool is empty before a host write is refused (default 8)");
module_param(gc_force_lines, int, 0444);
MODULE_PARM_DESC(gc_force_lines, "pool GC ignores the valid-page filter when free lines are at or below this (default 1)");

MODULE_PARM_DESC(gc_reserve_lines, "free lines usable only by GC (default 1)");
MODULE_PARM_DESC(rawp_sh_slots, "shadow staging ring size, power of two (default 4096)");
MODULE_PARM_DESC(rawp_sh_sweep, "shadow staging slots checked for expiry per acquire (default 128)");
MODULE_PARM_DESC(rawp_occ_r_ppk, "shadow staging occupancy EWMA weight, per mille (default 900)");

int gc_accounting = 0;

int pslc_raw_reserve = 1;
int pslc_raw_rebind = 1;

int pslc_prog_ns = PSLC_PROG_LATENCY;
int pslc_read_ns = PSLC_READ_LATENCY;
int pslc_erase_ns = PSLC_ERASE_LATENCY;

int tlc_line_cnt;
int slc_line_cnt;

int pslc_fold_batch = 4;

int pslc_fold_qd = 1024;

int pslc_raw_reserve_pgs = 9472;
module_param(pslc_raw_reserve_pgs, int, 0444);
MODULE_PARM_DESC(pslc_raw_reserve_pgs, "lower bound of the SLC reserve, in SLC oneshot pages (default 9472)");

int pslc_rs_alpha_ppk = 200;
module_param(pslc_rs_alpha_ppk, int, 0444);
MODULE_PARM_DESC(pslc_rs_alpha_ppk, "EWMA weight of the cost signal, per mille (default 200)");

MODULE_INFO(tenant_n, __stringify(TENANT_N));
static int tenant_n = TENANT_N;
module_param(tenant_n, int, 0444);
MODULE_PARM_DESC(tenant_n, "number of tenants set at build time with make TENANT_N=<n> (read-only)");

int pslc_rs_dwell = 3;
module_param(pslc_rs_dwell, int, 0444);
MODULE_PARM_DESC(pslc_rs_dwell, "number of windows in the least-squares slope of the read delay (default 3)");

int pslc_rs_tw3_ms = 620;
module_param(pslc_rs_tw3_ms, int, 0444);
MODULE_PARM_DESC(pslc_rs_tw3_ms, "tw3 threshold in ms: while the decision period tw3 is below it, RAW writes do not force SLC allocation (default 620)");

int pslc_rs_cmax_pgs = 66304;
module_param(pslc_rs_cmax_pgs, int, 0444);
MODULE_PARM_DESC(pslc_rs_cmax_pgs, "upper bound of the reserve cursor, in SLC oneshot pages (0=half of the SLC pool, default 66304)");

u64 pslc_adm[2], pslc_rej[2];
u64 pslc_reserve_hwm, pslc_reserve_sum, pslc_reserve_n;
u64 pslc_rawwin_n, pslc_rawwin_min;

u64 slcline_open, slcline_full, slcline_gc_victim, slcline_reclaimed;
u64 slcline_host_pgs, slcline_gc_moved, slcline_free_hwm, slcline_starved;
u64 slcline_strict_fb;

u64 slcline_raw_rebind, slcline_raw_nospace;

int op_trace = 0;

int op_trace_log2 = OPTRACE_LOG2_DEF;

int bp_hi = 112;
int bp_lo = 105;

int lunq3_qmax = 128;

int tetris_win = 32;
int tetris_alpha_pct = 20;

int tetris_prio_tn1 = 0, tetris_prio_tn2 = 0, tetris_prio_tn3 = 0;

int tetris_fthr_pct = 80;

int tetris_gc_thres_lines = 1;

u64 nvmev_bp_fetch_skipped;

int qserve_budget = 8;

int cache_mb = 128;

int cache_hit_lat_ns = 1000;
int cache_dirty_ratio = 100;
int cache_flush_batch = 8;
int flush_mode = 1;
int flush_stage_k = 2;

static int set_parse_mem_param(const char *val, const struct kernel_param *kp)
{
	unsigned long *arg = (unsigned long *)kp->arg;
	*arg = memparse(val, NULL);
	return 0;
}

static struct kernel_param_ops ops_parse_mem_param = {
	.set = set_parse_mem_param,
	.get = param_get_ulong,
};

module_param_cb(memmap_start, &ops_parse_mem_param, &memmap_start, 0444);
MODULE_PARM_DESC(memmap_start, "Reserved memory address");
module_param_cb(memmap_size, &ops_parse_mem_param, &memmap_size, 0444);
MODULE_PARM_DESC(memmap_size, "Reserved memory size");
module_param(read_time, uint, 0644);
MODULE_PARM_DESC(read_time, "Read time in nanoseconds");
module_param(read_delay, uint, 0644);
MODULE_PARM_DESC(read_delay, "Read delay in nanoseconds");
module_param(read_trailing, uint, 0644);
MODULE_PARM_DESC(read_trailing, "Read trailing in nanoseconds");
module_param(write_time, uint, 0644);
MODULE_PARM_DESC(write_time, "Write time in nanoseconds");
module_param(write_delay, uint, 0644);
MODULE_PARM_DESC(write_delay, "Write delay in nanoseconds");
module_param(write_trailing, uint, 0644);
MODULE_PARM_DESC(write_trailing, "Write trailing in nanoseconds");
module_param(nr_io_units, uint, 0444);
MODULE_PARM_DESC(nr_io_units, "Number of I/O units that operate in parallel");
module_param(io_unit_shift, uint, 0444);
MODULE_PARM_DESC(io_unit_shift, "Size of each I/O unit (2^)");
module_param(cpus, charp, 0444);
MODULE_PARM_DESC(cpus, "CPU list for process, completion(int.) threads, Seperated by Comma(,)");
static int gc_acct_reset_set(const char *val, const struct kernel_param *kp)
{
	gcacct_reset();
	return 0;
}
static const struct kernel_param_ops gc_acct_reset_ops = { .set = gc_acct_reset_set };
module_param_cb(gc_accounting_reset, &gc_acct_reset_ops, NULL, 0200);
MODULE_PARM_DESC(gc_accounting_reset, "write anything to zero the GC accounting counters and mark the measurement window start");

static int qserve_reset_set(const char *val, const struct kernel_param *kp)
{
	qserve_stats_reset();

	dcache_reset();
	return 0;
}
static const struct kernel_param_ops qserve_reset_ops = { .set = qserve_reset_set };
module_param_cb(qserve_reset, &qserve_reset_ops, NULL, 0200);
MODULE_PARM_DESC(qserve_reset, "write anything to zero the queue-serve and cache counters");

static int dcache_acct_reset_set(const char *val, const struct kernel_param *kp)
{
	dcache_reset();
	nvmev_dep_stats_reset();
	tetris_stall_reset();
	ssd_optrace_reset();
	return 0;
}
static const struct kernel_param_ops dcache_acct_reset_ops = { .set = dcache_acct_reset_set };
module_param_cb(dcache_acct_reset, &dcache_acct_reset_ops, NULL, 0200);
MODULE_PARM_DESC(dcache_acct_reset, "write anything to zero the DRAM cache counters");
module_param(pslc_prog_ns, int, 0444);
MODULE_PARM_DESC(pslc_prog_ns, "SLC page program latency in ns (default 160000)");
module_param(pslc_read_ns, int, 0444);
MODULE_PARM_DESC(pslc_read_ns, "SLC page read latency in ns (default 30000)");
module_param(pslc_erase_ns, int, 0444);
MODULE_PARM_DESC(pslc_erase_ns, "SLC block erase latency in ns (default 3000000)");
module_param(gc_accounting, int, 0444);
MODULE_PARM_DESC(gc_accounting, "1=classify the valid pages of each GC victim line by tenant (default 0)");
module_param(pslc_fold_batch, int, 0444);
MODULE_PARM_DESC(pslc_fold_batch, "SLC-to-TLC migrations started per writeback (default 4)");
module_param(pslc_fold_qd, int, 0444);
MODULE_PARM_DESC(pslc_fold_qd, "max SLC-to-TLC migrations in flight (default 1024)");
module_param(bp_hi, int, 0444);
MODULE_PARM_DESC(bp_hi, "per-die queue occupancy at which the dispatcher stops fetching (default 112)");
module_param(bp_lo, int, 0444);
MODULE_PARM_DESC(bp_lo, "per-die queue occupancy at which fetching resumes (default 105)");
module_param(lunq3_qmax, int, 0444);
module_param(tetris_win, int, 0444);
MODULE_PARM_DESC(tetris_win, "per-die window scanned by the fairness reordering (default 32)");

int tetris_wbatch = 2;
int tetris_wb_rd_max_ns = 500000;
module_param(tetris_wb_rd_max_ns, int, 0444);
MODULE_PARM_DESC(tetris_wb_rd_max_ns, "stop a write batch once the head read has waited longer than this, ns (0=never, default 500000)");

int tetris_chain = 1;

int components = 1;
module_param(components, int, 0444);
MODULE_PARM_DESC(components, "TETRIS components: 1=full, 2=FS-2+3 off, 3=FS-1+2+3 off, 4=QS-1+2 off (default 1)");

int tetris_chain_max = 112;
module_param(tetris_chain_max, int, 0644);
MODULE_PARM_DESC(tetris_chain_max, "max consecutive chain-break charges per die before yielding (default 112)");

module_param(tetris_gc_thres_lines, int, 0444);
MODULE_PARM_DESC(tetris_gc_thres_lines, "GC bursts in the scheduler run only when free lines fall below this (0=always, default 1)");
module_param(tetris_fthr_pct, int, 0444);
MODULE_PARM_DESC(tetris_fthr_pct, "fairness threshold in percent; reorder when min/max slowdown in the window is below it (default 80)");
module_param(tetris_prio_tn1, int, 0444);
MODULE_PARM_DESC(tetris_prio_tn1, "static priority of tenant 1 (default 0 = single class)");
module_param(tetris_prio_tn2, int, 0444);
MODULE_PARM_DESC(tetris_prio_tn2, "static priority of tenant 2 (default 0)");
module_param(tetris_prio_tn3, int, 0444);
MODULE_PARM_DESC(tetris_prio_tn3, "static priority of tenant 3 (default 0)");
module_param(tetris_alpha_pct, int, 0444);
MODULE_PARM_DESC(tetris_alpha_pct, "a flow is low-intensity if its share of an epoch's transactions is below this percent (default 20)");
MODULE_PARM_DESC(lunq3_qmax, "per-die queue capacity (default 128)");
module_param(qserve_budget, int, 0444);
MODULE_PARM_DESC(qserve_budget, "max ops served per die per queue-serve pass (0=unbounded, default 8)");
module_param(op_trace, int, 0444);
MODULE_PARM_DESC(op_trace, "per-NAND-op trace ring read from /proc/nvmev/op_trace: 0=off, 2=record at service (default 0)");
module_param(op_trace_log2, int, 0444);
MODULE_PARM_DESC(op_trace_log2, "op_trace ring size as log2 of entries, 16..27 (default 21)");
module_param(cache_mb, int, 0444);
MODULE_PARM_DESC(cache_mb, "DRAM write-back cache size in MiB (0=off, default 128)");
module_param(cache_hit_lat_ns, int, 0444);
MODULE_PARM_DESC(cache_hit_lat_ns, "cache hit latency in ns (default 1000)");
module_param(cache_dirty_ratio, int, 0444);
MODULE_PARM_DESC(cache_dirty_ratio, "components=3: writeback starts above this percent of dirty entries (default 100)");
module_param(cache_flush_batch, int, 0444);
MODULE_PARM_DESC(cache_flush_batch, "components=3: writebacks issued per triggering write (default 8)");
module_param(flush_stage_k, int, 0444);
MODULE_PARM_DESC(flush_stage_k, "components=1,2,4: outstanding background writeback programs per die (default 2)");

static ssize_t scanreg_proc_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off)
{
	char buf[64];
	unsigned int id;
	unsigned long long slba, nlba;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;
	if (!strncmp(buf, "clear", 5)) {
		scanreg_clear();
		return len;
	}
	if (sscanf(buf, "%u %llu %llu", &id, &slba, &nlba) == 3)
		scanreg_set((uint16_t)id, slba, nlba);
	return len;
}

static ssize_t tenantreg_proc_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off)
{
	char buf[64];
	unsigned int tn;
	unsigned long long slba, nlba;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;
	if (!strncmp(buf, "clear", 5)) {
		tenant_map_clear();
		return len;
	}
	if (sscanf(buf, "%u %llu %llu", &tn, &slba, &nlba) == 3)
		tenant_map_set((u8)tn, slba, nlba);
	return len;
}
static const struct proc_ops tenantreg_proc_fops = {
	.proc_write = tenantreg_proc_write,
	.proc_lseek = noop_llseek,
};

static const struct proc_ops scanreg_proc_fops = {
	.proc_write = scanreg_proc_write,
};

static bool nvmev_proc_dbs(void)
{
	int qid;
	int dbs_idx;
	int new_db;
	int old_db;
	bool updated = false;

	new_db = nvmev_vdev->dbs[0];
	if (new_db != nvmev_vdev->old_dbs[0]) {
		nvmev_proc_admin_sq(new_db, nvmev_vdev->old_dbs[0]);
		nvmev_vdev->old_dbs[0] = new_db;
		updated = true;
	}
	new_db = nvmev_vdev->dbs[1];
	if (new_db != nvmev_vdev->old_dbs[1]) {
		nvmev_vdev->old_dbs[1] = new_db;
		updated = true;
	}

	if (READ_ONCE(nvmev_sq_stall)) {
		nvmev_bp_fetch_skipped++;
	} else {
		for (qid = 1; qid <= nvmev_vdev->nr_sq; qid++) {
			if (nvmev_vdev->sqes[qid] == NULL)
				continue;
			dbs_idx = qid * 2;
			new_db = nvmev_vdev->dbs[dbs_idx];
			old_db = nvmev_vdev->old_dbs[dbs_idx];
			if (new_db != old_db) {
				nvmev_vdev->old_dbs[dbs_idx] =
					nvmev_proc_io_sq(qid, new_db, old_db);
				updated = true;
			}
		}
	}

	for (qid = 1; qid <= nvmev_vdev->nr_cq; qid++) {
		if (nvmev_vdev->cqes[qid] == NULL)
			continue;
		dbs_idx = qid * 2 + 1;
		new_db = nvmev_vdev->dbs[dbs_idx];
		old_db = nvmev_vdev->old_dbs[dbs_idx];
		if (new_db != old_db) {
			nvmev_proc_io_cq(qid, new_db, old_db);
			nvmev_vdev->old_dbs[dbs_idx] = new_db;
			updated = true;
		}
	}

	return updated;
}

static void bp_update(void)
{
	WRITE_ONCE(nvmev_sq_stall,
		   READ_ONCE(bp_queue_stall) | READ_ONCE(backpressure_free_line_stall) |
		   nvmev_diq_bp_eval());
}

static int nvmev_dispatcher(void *data)
{
	static unsigned long last_dispatched_time = 0;

	NVMEV_INFO("nvmev_dispatcher started on cpu %d (node %d)\n",
		   nvmev_vdev->config.cpu_nr_dispatcher,
		   cpu_to_node(nvmev_vdev->config.cpu_nr_dispatcher));

	while (!kthread_should_stop()) {
		if (nvmev_proc_bars())
			last_dispatched_time = jiffies;
		if (nvmev_proc_dbs())
			last_dispatched_time = jiffies;

		{
			unsigned int i;

			for (i = 0; i < nvmev_vdev->nr_ns; i++)
				if (nvmev_vdev->ns[i].outbox_drain)
					nvmev_vdev->ns[i].outbox_drain(&nvmev_vdev->ns[i]);
		}

		{
			unsigned int i;

			for (i = 0; i < nvmev_vdev->nr_ns; i++)
				if (nvmev_vdev->ns[i].gc_tick)
					nvmev_vdev->ns[i].gc_tick(&nvmev_vdev->ns[i]);
		}

		bp_update();

		if (CONFIG_NVMEVIRT_IDLE_TIMEOUT != 0 &&
		    time_after(jiffies, last_dispatched_time + (CONFIG_NVMEVIRT_IDLE_TIMEOUT * HZ)))
			schedule_timeout_interruptible(1);
		else
			cond_resched();
	}

	return 0;
}

static int nvmev_qserve(void *data)
{
	NVMEV_INFO("nvmev_qserve started on cpu %d (node %d)\n", smp_processor_id(),
		   cpu_to_node(smp_processor_id()));

	while (!kthread_should_stop()) {
		uint64_t t0 = cpu_clock(nvmev_vdev->config.cpu_nr_dispatcher);
		uint64_t now = t0;
		unsigned int i;

		for (i = 0; i < nvmev_vdev->nr_ns; i++) {
			struct nvmev_ns *ns = &nvmev_vdev->ns[i];

			if (ns->qserve_pass)
				ns->qserve_pass(ns, now);
		}

		cond_resched();
	}
	return 0;
}

static void NVMEV_QSERVE_INIT(struct nvmev_dev *nvmev_vdev)
{
	nvmev_vdev->nvmev_qserve = kthread_create(nvmev_qserve, NULL, "nvmev_qserve");
	if (nvmev_vdev->config.cpu_nr_qserve != -1)
		kthread_bind(nvmev_vdev->nvmev_qserve, nvmev_vdev->config.cpu_nr_qserve);
	wake_up_process(nvmev_vdev->nvmev_qserve);
}

static void NVMEV_QSERVE_FINAL(struct nvmev_dev *nvmev_vdev)
{
	if (!IS_ERR_OR_NULL(nvmev_vdev->nvmev_qserve)) {
		kthread_stop(nvmev_vdev->nvmev_qserve);
		nvmev_vdev->nvmev_qserve = NULL;
	}
}

static void NVMEV_DISPATCHER_INIT(struct nvmev_dev *nvmev_vdev)
{
	nvmev_vdev->nvmev_dispatcher = kthread_create(nvmev_dispatcher, NULL, "nvmev_dispatcher");
	if (nvmev_vdev->config.cpu_nr_dispatcher != -1)
		kthread_bind(nvmev_vdev->nvmev_dispatcher, nvmev_vdev->config.cpu_nr_dispatcher);
	wake_up_process(nvmev_vdev->nvmev_dispatcher);
}

static void NVMEV_DISPATCHER_FINAL(struct nvmev_dev *nvmev_vdev)
{
	if (!IS_ERR_OR_NULL(nvmev_vdev->nvmev_dispatcher)) {
		kthread_stop(nvmev_vdev->nvmev_dispatcher);
		nvmev_vdev->nvmev_dispatcher = NULL;
	}
}

#ifdef CONFIG_X86
static int __validate_configs_arch(void)
{
	unsigned long resv_start_bytes;
	unsigned long resv_end_bytes;

	resv_start_bytes = memmap_start;
	resv_end_bytes = resv_start_bytes + memmap_size - 1;

	if (e820__mapped_any(resv_start_bytes, resv_end_bytes, E820_TYPE_RAM) ||
	    e820__mapped_any(resv_start_bytes, resv_end_bytes, E820_TYPE_RESERVED_KERN)) {
		NVMEV_ERROR("[mem %#010lx-%#010lx] is usable, not reseved region\n",
			    (unsigned long)resv_start_bytes, (unsigned long)resv_end_bytes);
		return -EPERM;
	}

	if (!e820__mapped_any(resv_start_bytes, resv_end_bytes, E820_TYPE_RESERVED)) {
		NVMEV_ERROR("[mem %#010lx-%#010lx] is not reseved region\n",
			    (unsigned long)resv_start_bytes, (unsigned long)resv_end_bytes);
		return -EPERM;
	}
	return 0;
}
#else
static int __validate_configs_arch(void)
{
	return 0;
}
#endif

static int __validate_configs(void)
{
	if (!memmap_start) {
		NVMEV_ERROR("[memmap_start] should be specified\n");
		return -EINVAL;
	}

	if (!memmap_size) {
		NVMEV_ERROR("[memmap_size] should be specified\n");
		return -EINVAL;
	} else if (memmap_size <= MB(1)) {
		NVMEV_ERROR("[memmap_size] should be bigger than 1 MiB\n");
		return -EINVAL;
	}

	if (__validate_configs_arch()) {
		return -EPERM;
	}

	if (nr_io_units == 0 || io_unit_shift == 0) {
		NVMEV_ERROR("Need non-zero IO unit size and at least one IO unit\n");
		return -EINVAL;
	}
	if (read_time == 0) {
		NVMEV_ERROR("Need non-zero read time\n");
		return -EINVAL;
	}
	if (write_time == 0) {
		NVMEV_ERROR("Need non-zero write time\n");
		return -EINVAL;
	}

	if (pslc_raw_reserve_pgs < 0) {
		NVMEV_ERROR("pslc_raw_reserve_pgs must be >= 0 (got %d)\n", pslc_raw_reserve_pgs);
		return -EINVAL;
	}

	if (pslc_rs_cmax_pgs < 0) {
		NVMEV_ERROR("pslc_rs_cmax_pgs must be >= 0 (got %d)\n", pslc_rs_cmax_pgs);
		return -EINVAL;
	}

	if (cache_mb > 0 && SSD_PARTITIONS != 1)
		NVMEV_INFO("cache_late_bind=1 works only with pslc_policy=6, cache_lba_check=1 and SSD_PARTITIONS=1 "
			   "(got %d, %d, %d); here host writes take a PPA at write time (old behaviour)\n",
			   6, 1, SSD_PARTITIONS);

	if (cache_mb > 0 && SSD_PARTITIONS != 1) {
		NVMEV_ERROR("cache_rand_entry=1 requires SSD_PARTITIONS == 1 (got %d)\n", SSD_PARTITIONS);
		return -EINVAL;
	}
	if (cache_mb > 0) {
		if (cache_mb > 1023) {
			NVMEV_ERROR("cache_lba_check=1 requires cache_mb <= 1023 (got %d)\n", cache_mb);
			return -EINVAL;
		}

		if (FLASH_PAGE_SIZE != ONESHOT_PAGE_SIZE || FLASH_PAGE_SIZE != CE_SLOTS * 4096) {
			NVMEV_ERROR("cache_lba_check=1 requires FLASH_PAGE_SIZE == ONESHOT_PAGE_SIZE == %d KiB\n",
				    CE_SLOTS * 4);
			return -EINVAL;
		}
	}
	NVMEV_INFO("pslc cursor window: need_both=%d (default 1), window = PSLC_ADAPT_N flushes\n",
			   0);
	if (pslc_rs_alpha_ppk < 1 || pslc_rs_alpha_ppk > 1000) {
		NVMEV_ERROR("pslc_rs_alpha_ppk must be 1..1000 (got %d)\n", pslc_rs_alpha_ppk);
		return -EINVAL;
	}
	if (pslc_rs_tw3_ms < 1) {
		NVMEV_ERROR("pslc_rs_tw3_ms must be >= 1 (got %d)\n", pslc_rs_tw3_ms);
		return -EINVAL;
	}

	if (pslc_rs_dwell < 1) {
		NVMEV_ERROR("pslc_rs_dwell must be >= 1 (got %d)\n", pslc_rs_dwell);
		return -EINVAL;
	}
	if (pslc_rs_cmax_pgs > 0 && pslc_rs_cmax_pgs < pslc_raw_reserve_pgs) {
		NVMEV_ERROR("pslc_rs_cmax_pgs=%d must be >= pslc_raw_reserve_pgs=%d (upper bound below lower bound)\n",
			    pslc_rs_cmax_pgs, pslc_raw_reserve_pgs);
		return -EINVAL;
	}
	if (tetris_wb_rd_max_ns < 0) {
		NVMEV_ERROR("tetris_wb_rd_max_ns must be >= 0 (got %d)\n", tetris_wb_rd_max_ns);
		return -EINVAL;
	}

	return 0;
}

static void __print_perf_configs(void)
{
#ifdef CONFIG_NVMEV_VERBOSE
	unsigned long unit_perf_kb =
			nvmev_vdev->config.nr_io_units << (nvmev_vdev->config.io_unit_shift - 10);
	struct nvmev_config *cfg = &nvmev_vdev->config;

	NVMEV_INFO("=============== Configurations ===============\n");
	NVMEV_INFO("* IO units : %d x %d\n",
			cfg->nr_io_units, 1 << cfg->io_unit_shift);
	NVMEV_INFO("* I/O times\n");
	NVMEV_INFO("  Read     : %u + %u x + %u ns\n",
				cfg->read_delay, cfg->read_time, cfg->read_trailing);
	NVMEV_INFO("  Write    : %u + %u x + %u ns\n",
				cfg->write_delay, cfg->write_time, cfg->write_trailing);
	NVMEV_INFO("* Bandwidth\n");
	NVMEV_INFO("  Read     : %lu MiB/s\n",
			(1000000000UL / (cfg->read_time + cfg->read_delay + cfg->read_trailing)) * unit_perf_kb >> 10);
	NVMEV_INFO("  Write    : %lu MiB/s\n",
			(1000000000UL / (cfg->write_time + cfg->write_delay + cfg->write_trailing)) * unit_perf_kb >> 10);
#endif
}

static int __get_nr_entries(int dbs_idx, int queue_size)
{
	int diff = nvmev_vdev->dbs[dbs_idx] - nvmev_vdev->old_dbs[dbs_idx];
	if (diff < 0) {
		diff += queue_size;
	}
	return diff;
}

static int __proc_file_read(struct seq_file *m, void *data)
{
	const char *filename = m->private;
	struct nvmev_config *cfg = &nvmev_vdev->config;

	if (strcmp(filename, "read_times") == 0) {
		seq_printf(m, "%u + %u x + %u", cfg->read_delay, cfg->read_time,
			   cfg->read_trailing);
	} else if (strcmp(filename, "write_times") == 0) {
		seq_printf(m, "%u + %u x + %u", cfg->write_delay, cfg->write_time,
			   cfg->write_trailing);
	} else if (strcmp(filename, "io_units") == 0) {
		seq_printf(m, "%u x %u", cfg->nr_io_units, cfg->io_unit_shift);
	} else if (strcmp(filename, "stat") == 0) {
		int i;
		unsigned int nr_in_flight = 0;
		unsigned int nr_dispatch = 0;
		unsigned int nr_dispatched = 0;
		unsigned long long total_io = 0;
		for (i = 1; i <= nvmev_vdev->nr_sq; i++) {
			struct nvmev_submission_queue *sq = nvmev_vdev->sqes[i];
			if (!sq)
				continue;

			seq_printf(m, "%2d: %2u %4u %4u %4u %4u %llu\n", i,
				   __get_nr_entries(i * 2, sq->queue_size), sq->stat.nr_in_flight,
				   sq->stat.max_nr_in_flight, sq->stat.nr_dispatch,
				   sq->stat.nr_dispatched, sq->stat.total_io);

			nr_in_flight += sq->stat.nr_in_flight;
			nr_dispatch += sq->stat.nr_dispatch;
			nr_dispatched += sq->stat.nr_dispatched;
			total_io += sq->stat.total_io;

			barrier();
			sq->stat.max_nr_in_flight = 0;
		}
		seq_printf(m, "total: %u %u %u %llu\n", nr_in_flight, nr_dispatch, nr_dispatched,
			   total_io);
	}

	return 0;
}

static ssize_t __proc_file_write(struct file *file, const char __user *buf, size_t len,
				 loff_t *offp)
{
	ssize_t count = len;
	const char *filename = file->f_path.dentry->d_name.name;
	char input[128];
	unsigned int ret;
	unsigned long long *old_stat;
	struct nvmev_config *cfg = &nvmev_vdev->config;
	size_t copy_len;

	copy_len = min(len, sizeof(input) - 1);
	if (copy_from_user(input, buf, copy_len))
		return -EFAULT;
	input[copy_len] = '\0';

	if (!strcmp(filename, "read_times")) {
		ret = sscanf(input, "%u %u %u", &cfg->read_delay, &cfg->read_time,
			     &cfg->read_trailing);
	} else if (!strcmp(filename, "write_times")) {
		ret = sscanf(input, "%u %u %u", &cfg->write_delay, &cfg->write_time,
			     &cfg->write_trailing);
	} else if (!strcmp(filename, "io_units")) {
		ret = sscanf(input, "%d %d", &cfg->nr_io_units, &cfg->io_unit_shift);
		if (ret < 1)
			goto out;

		old_stat = nvmev_vdev->io_unit_stat;
		nvmev_vdev->io_unit_stat =
			kzalloc(sizeof(*nvmev_vdev->io_unit_stat) * cfg->nr_io_units, GFP_KERNEL);

		mdelay(100);
		kfree(old_stat);
	} else if (!strcmp(filename, "stat")) {
		int i;
		for (i = 1; i <= nvmev_vdev->nr_sq; i++) {
			struct nvmev_submission_queue *sq = nvmev_vdev->sqes[i];
			if (!sq)
				continue;

			memset(&sq->stat, 0x00, sizeof(sq->stat));
		}
	}

out:
	__print_perf_configs();

	return count;
}

static int __proc_file_open(struct inode *inode, struct file *file)
{
	return single_open(file, __proc_file_read, (char *)file->f_path.dentry->d_name.name);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
static const struct proc_ops proc_file_fops = {
	.proc_open = __proc_file_open,
	.proc_write = __proc_file_write,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations proc_file_fops = {
	.open = __proc_file_open,
	.write = __proc_file_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static void *optrace_seq_start(struct seq_file *m, loff_t *pos)
{
	uint64_t n = min_t(uint64_t, ssd_optrace_total(), ssd_optrace_n());
	if (!ssd_optrace_ring() || *pos >= n)
		return NULL;
	return (void *)(uintptr_t)(*pos + 1);
}

static void *optrace_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	uint64_t n = min_t(uint64_t, ssd_optrace_total(), ssd_optrace_n());
	(*pos)++;
	if (*pos >= n)
		return NULL;
	return (void *)(uintptr_t)(*pos + 1);
}

static void optrace_seq_stop(struct seq_file *m, void *v)
{
}

static int optrace_seq_show(struct seq_file *m, void *v)
{
	uint64_t i = (uintptr_t)v - 1;
	uint64_t total = ssd_optrace_total();
	uint64_t n = ssd_optrace_n();
	uint64_t base = (total > n) ? (total - n) : 0;
	struct optrace_ent *e = &ssd_optrace_ring()[(base + i) & (n - 1)];

	seq_printf(m, "%u %u %u %u %u %u %llu %llu %llu %u %u %u\n", e->host_seq, e->ch, e->lun, e->tier,
		   e->cmd, e->io_type, (unsigned long long)e->arrival,
		   (unsigned long long)e->start, (unsigned long long)e->end,
		   e->kind, e->blat_us, e->hold_ns);
	return 0;
}

static const struct seq_operations optrace_seq_ops = {
	.start = optrace_seq_start,
	.next = optrace_seq_next,
	.stop = optrace_seq_stop,
	.show = optrace_seq_show,
};

static int optrace_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &optrace_seq_ops);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
static const struct proc_ops optrace_proc_fops = {
	.proc_open = optrace_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations optrace_proc_fops = {
	.open = optrace_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

static void NVMEV_STORAGE_INIT(struct nvmev_dev *nvmev_vdev)
{
	NVMEV_INFO("Storage: %#010lx-%#010lx (%lu MiB)\n",
			nvmev_vdev->config.storage_start,
			nvmev_vdev->config.storage_start + nvmev_vdev->config.storage_size,
			BYTE_TO_MB(nvmev_vdev->config.storage_size));

	nvmev_vdev->io_unit_stat = kzalloc(
		sizeof(*nvmev_vdev->io_unit_stat) * nvmev_vdev->config.nr_io_units, GFP_KERNEL);

	nvmev_vdev->storage_mapped = memremap(nvmev_vdev->config.storage_start,
					      nvmev_vdev->config.storage_size, MEMREMAP_WB);

	if (nvmev_vdev->storage_mapped == NULL)
		NVMEV_ERROR("Failed to map storage memory.\n");

	nvmev_vdev->proc_root = proc_mkdir("nvmev", NULL);
	nvmev_vdev->proc_read_times =
		proc_create("read_times", 0664, nvmev_vdev->proc_root, &proc_file_fops);
	nvmev_vdev->proc_write_times =
		proc_create("write_times", 0664, nvmev_vdev->proc_root, &proc_file_fops);
	nvmev_vdev->proc_io_units =
		proc_create("io_units", 0664, nvmev_vdev->proc_root, &proc_file_fops);
	nvmev_vdev->proc_stat = proc_create("stat", 0444, nvmev_vdev->proc_root, &proc_file_fops);
	nvmev_vdev->proc_debug = proc_create("debug", 0664, nvmev_vdev->proc_root, &proc_file_fops);
	ssd_optrace_init();
	if (op_trace)
		proc_create("op_trace", 0444, nvmev_vdev->proc_root, &optrace_proc_fops);
	proc_create("scan_register", 0664, nvmev_vdev->proc_root, &scanreg_proc_fops);
	proc_create("tenant_register", 0664, nvmev_vdev->proc_root, &tenantreg_proc_fops);
	dcache_init();
}

static void NVMEV_STORAGE_FINAL(struct nvmev_dev *nvmev_vdev)
{
	remove_proc_entry("read_times", nvmev_vdev->proc_root);
	remove_proc_entry("write_times", nvmev_vdev->proc_root);
	remove_proc_entry("io_units", nvmev_vdev->proc_root);
	remove_proc_entry("stat", nvmev_vdev->proc_root);
	remove_proc_entry("debug", nvmev_vdev->proc_root);
	if (op_trace)
		remove_proc_entry("op_trace", nvmev_vdev->proc_root);
	remove_proc_entry("scan_register", nvmev_vdev->proc_root);
	remove_proc_entry("tenant_register", nvmev_vdev->proc_root);
	ssd_optrace_exit();
	dcache_exit();
	pslc_free();

	remove_proc_entry("nvmev", NULL);

	if (nvmev_vdev->storage_mapped)
		memunmap(nvmev_vdev->storage_mapped);

	if (nvmev_vdev->io_unit_stat)
		kfree(nvmev_vdev->io_unit_stat);
}

static bool __load_configs(struct nvmev_config *config)
{
	bool first = true;
	unsigned int cpu_nr;
	char *cpu;

	if (__validate_configs() < 0) {
		return false;
	}

	config->memmap_start = memmap_start;
	config->memmap_size = memmap_size;

	config->storage_start = memmap_start + MB(1);
	config->storage_size = memmap_size - MB(1);

	config->read_time = read_time;
	config->read_delay = read_delay;
	config->read_trailing = read_trailing;
	config->write_time = write_time;
	config->write_delay = write_delay;
	config->write_trailing = write_trailing;
	config->nr_io_units = nr_io_units;
	config->io_unit_shift = io_unit_shift;

	config->nr_io_workers = 0;
	config->cpu_nr_dispatcher = -1;

	while ((cpu = strsep(&cpus, ",")) != NULL) {
		cpu_nr = (unsigned int)simple_strtol(cpu, NULL, 10);
		if (first) {
			config->cpu_nr_dispatcher = cpu_nr;
		} else {
			config->cpu_nr_io_workers[config->nr_io_workers] = cpu_nr;
			config->nr_io_workers++;
		}
		first = false;
	}

	config->cpu_nr_qserve = -1;
	if (config->nr_io_workers > 1) {
		config->nr_io_workers--;
		config->cpu_nr_qserve = config->cpu_nr_io_workers[config->nr_io_workers];
	}

	return true;
}

static void NVMEV_NAMESPACE_INIT(struct nvmev_dev *nvmev_vdev)
{
	unsigned long long remaining_capacity = nvmev_vdev->config.storage_size;
	void *ns_addr = nvmev_vdev->storage_mapped;
	const int nr_ns = NR_NAMESPACES;
	const unsigned int disp_no = nvmev_vdev->config.cpu_nr_dispatcher;
	int i;
	unsigned long long size;

	struct nvmev_ns *ns = kzalloc(sizeof(struct nvmev_ns) * nr_ns, GFP_KERNEL);

	for (i = 0; i < nr_ns; i++) {
		if (NS_CAPACITY(i) == 0)
			size = remaining_capacity;
		else
			size = min(NS_CAPACITY(i), remaining_capacity);

		if (NS_SSD_TYPE(i) == SSD_TYPE_CONV)
			conv_init_namespace(&ns[i], i, size, ns_addr, disp_no);
		else
			BUG_ON(1);

		remaining_capacity -= size;
		ns_addr += size;
		NVMEV_INFO("ns %d/%d: size %lld MiB\n", i, nr_ns, BYTE_TO_MB(ns[i].size));
	}

	nvmev_vdev->ns = ns;
	nvmev_vdev->nr_ns = nr_ns;
	nvmev_vdev->mdts = MDTS;
}

static void NVMEV_NAMESPACE_FINAL(struct nvmev_dev *nvmev_vdev)
{
	struct nvmev_ns *ns = nvmev_vdev->ns;
	const int nr_ns = NR_NAMESPACES;
	int i;

	for (i = 0; i < nr_ns; i++) {
		if (NS_SSD_TYPE(i) == SSD_TYPE_CONV)
			conv_remove_namespace(&ns[i]);
		else
			BUG_ON(1);
	}

	kfree(ns);
	nvmev_vdev->ns = NULL;
}

static void __print_base_config(void)
{
	const char *type = "unknown";

	NVMEV_INFO("Version %x.%x for >> %s <<\n",
			(NVMEV_VERSION & 0xff00) >> 8, (NVMEV_VERSION & 0x00ff), type);
}

static int NVMeV_init(void)
{
	__print_base_config();

	if (components < 1 || components > 4) {
		NVMEV_ERROR("components takes 1..4 (got %d)\n", components);
		return -EINVAL;
	}
	flush_mode = (components == 3) ? 0 : FLUSH_ASYNC_BUDGET;
	pslc_raw_rebind = (components == 2 || components == 3) ? 0 : 1;
	pslc_raw_reserve = pslc_raw_rebind;
	tetris_chain = (components == 4) ? 0 : 1;
	tetris_wbatch = (components == 4) ? 1 : 2;

	if (lunq3_qmax <= 0)
		lunq3_qmax = 1024;
	if (lunq3_qmax > LUNQ3_QMAX_MAX) {
		NVMEV_ERROR("lunq3_qmax must be 0 < q <= %d (got %d)\n", LUNQ3_QMAX_MAX, lunq3_qmax);
		return -EINVAL;
	}
	if (bp_hi <= 0)
		bp_hi = LUNQ3_BP_HI_OF(lunq3_qmax);
	if (bp_lo <= 0)
		bp_lo = LUNQ3_BP_LO_OF(lunq3_qmax);
	if (bp_lo >= bp_hi || bp_hi > lunq3_qmax) {
		NVMEV_ERROR("bp marks must satisfy 0 < bp_lo < bp_hi <= %d (got lo=%d hi=%d)\n",
			    lunq3_qmax, bp_lo, bp_hi);
		return -EINVAL;
	}

	if (op_trace != 0 && op_trace != 2) {
		NVMEV_ERROR("op_trace takes 0 or 2 (got %d)\n", op_trace);
		return -EINVAL;
	}

	if (op_trace_log2 < 16 || op_trace_log2 > 27) {
		NVMEV_ERROR("op_trace_log2 takes 16..27 (got %d)\n", op_trace_log2);
		return -EINVAL;
	}

	if (pslc_fold_qd < 1 || pslc_fold_qd > (1 << 20)) {
		NVMEV_ERROR("pslc_fold_qd must be 1..%d (got %d)\n", 1 << 20, pslc_fold_qd);
		return -EINVAL;
	}
	if (cache_mb > 1023) {
		NVMEV_ERROR("cache_mb=%d exceeds 1023: the frame handle packs the generation into "
			    "16 bits, so a larger cache would leave dirty-flush waiters unreleased\n",
			    cache_mb);
		return -EINVAL;
	}

	if (qserve_budget < 0) {
		NVMEV_ERROR("qserve_budget must be >= 0 (0 = unbounded)\n");
		return -EINVAL;
	}

	nvmev_vdev = VDEV_INIT();
	if (!nvmev_vdev)
		return -EINVAL;

	if (!__load_configs(&nvmev_vdev->config)) {
		goto ret_err;
	}

	NVMEV_STORAGE_INIT(nvmev_vdev);

	NVMEV_NAMESPACE_INIT(nvmev_vdev);

	if (io_using_dma) {
		if (ioat_dma_chan_set("dma7chan0") != 0) {
			io_using_dma = false;
			NVMEV_ERROR("Cannot use DMA engine, Fall back to memcpy\n");
		}
	}

	if (!NVMEV_PCI_INIT(nvmev_vdev)) {
		goto ret_err;
	}

	__print_perf_configs();

	NVMEV_IO_WORKER_INIT(nvmev_vdev);
	NVMEV_QSERVE_INIT(nvmev_vdev);
	NVMEV_DISPATCHER_INIT(nvmev_vdev);

	pci_bus_add_devices(nvmev_vdev->virt_bus);

	NVMEV_INFO("Virtual NVMe device created\n");

	NVMEV_INFO("NVMeVirt FIRMWARE  TETRIS\n");

	NVMEV_INFO("NVMeVirt FIRMWARE  CACHE FAMILY=%s  (cache_mb=%dMiB flush=%s dirty_ratio=%d%% stage_k=%d)\n",
		   cache_mb == 0 ? "off" : "CFLRU", cache_mb,
		   flush_mode == FLUSH_ASYNC_BUDGET ? "async-budget" : "fixed-ratio",
		   cache_dirty_ratio, flush_stage_k);

	return 0;

ret_err:
	VDEV_FINALIZE(nvmev_vdev);
	return -EIO;
}

static void NVMeV_exit(void)
{
	int i;

	if (nvmev_vdev->virt_bus != NULL) {
		pci_stop_root_bus(nvmev_vdev->virt_bus);
		pci_remove_root_bus(nvmev_vdev->virt_bus);
	}

	NVMEV_DISPATCHER_FINAL(nvmev_vdev);
	NVMEV_QSERVE_FINAL(nvmev_vdev);
	NVMEV_IO_WORKER_FINAL(nvmev_vdev);

	NVMEV_NAMESPACE_FINAL(nvmev_vdev);
	NVMEV_STORAGE_FINAL(nvmev_vdev);

	if (io_using_dma) {
		ioat_dma_cleanup();
	}

	for (i = 0; i < nvmev_vdev->nr_sq; i++) {
		kfree(nvmev_vdev->sqes[i]);
	}

	for (i = 0; i < nvmev_vdev->nr_cq; i++) {
		kfree(nvmev_vdev->cqes[i]);
	}

	VDEV_FINALIZE(nvmev_vdev);

	NVMEV_INFO("Virtual NVMe device closed\n");
}

MODULE_LICENSE("GPL v2");
module_init(NVMeV_init);
module_exit(NVMeV_exit);
