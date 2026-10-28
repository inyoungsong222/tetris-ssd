// SPDX-License-Identifier: GPL-2.0-only

#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/sched/clock.h>
#include <linux/vmalloc.h>

#include "nvmev.h"
#include "ssd.h"

uint64_t tetris_lr_busy[TETRIS_NR_DIES];
uint8_t tetris_lr_busy_tn[TETRIS_NR_DIES];

u32 qs_pend_hwm;

u32 qs_hwm_user, qs_hwm_gc, qs_hwm_fold;
EXPORT_SYMBOL(tetris_lr_busy);
uint64_t tetris_sr_busy[TETRIS_NR_DIES];
EXPORT_SYMBOL(tetris_sr_busy);

static struct optrace_ent *optrace_ring;
static atomic64_t optrace_idx = ATOMIC64_INIT(0);

static uint64_t optrace_n;

void ssd_optrace_init(void)
{
	if (!op_trace || optrace_ring)
		return;
	optrace_n = 1ULL << op_trace_log2;
	optrace_ring = vzalloc(sizeof(struct optrace_ent) * optrace_n);
	atomic64_set(&optrace_idx, 0);
	NVMEV_INFO("op_trace: ring %s (%llu entries, %llu MiB, entry %zu B)\n",
		   optrace_ring ? "enabled" : "ALLOC FAILED", optrace_n,
		   (sizeof(struct optrace_ent) * optrace_n) >> 20, sizeof(struct optrace_ent));
}

uint64_t ssd_optrace_n(void)
{
	return optrace_n;
}

void ssd_optrace_exit(void)
{
	if (optrace_ring) {
		vfree(optrace_ring);
		optrace_ring = NULL;
	}
}

uint64_t ssd_optrace_total(void)
{
	return (uint64_t)atomic64_read(&optrace_idx);
}

void ssd_optrace_reset(void)
{
	uint64_t old;

	if (!optrace_ring)
		return;
	old = (uint64_t)atomic64_xchg(&optrace_idx, 0);
	NVMEV_INFO("op_trace: ring cleared at main-window start (dropped %llu earlier records, ring %llu entries)\n",
		   old, optrace_n);
}

void ssd_optrace_record_hit(uint32_t host_seq, uint64_t arrival, uint64_t end)
{
	struct optrace_ent *e;
	uint64_t i;

	if (likely(!op_trace) || !optrace_ring)
		return;
	i = (uint64_t)atomic64_inc_return(&optrace_idx) - 1;
	e = &optrace_ring[i & (optrace_n - 1)];
	e->arrival = arrival;
	e->start = arrival;
	e->end = end;
	e->host_seq = host_seq;
	e->ch = 255;
	e->lun = 255;
	e->tier = 0;
	e->cmd = OPTRACE_CMD_CACHE_HIT;
	e->io_type = 0;
	e->kind = 0;
	e->blat_us = 0;
	e->hold_ns = 0;
}

void ssd_optrace_record_diq(uint32_t host_seq, uint64_t was, uint64_t done, uint8_t tn, uint8_t mig)
{
	struct optrace_ent *e;
	uint64_t i;

	if (likely(!op_trace) || !optrace_ring)
		return;
	i = (uint64_t)atomic64_inc_return(&optrace_idx) - 1;
	e = &optrace_ring[i & (optrace_n - 1)];
	e->arrival = was;
	e->start = was;
	e->end = done;
	e->host_seq = host_seq;
	e->ch = 255;
	e->lun = 255;
	e->tier = mig;
	e->cmd = OPTRACE_CMD_DIQ_WAIT;
	e->io_type = 0;
	e->kind = tn;
	e->blat_us = 0;
	e->hold_ns = 0;
}

void ssd_optrace_record_host(uint32_t host_seq, uint64_t arrival, uint8_t tn, uint8_t large, uint32_t bytes)
{
	struct optrace_ent *e;
	uint64_t i;

	if (likely(!op_trace) || !optrace_ring)
		return;
	i = (uint64_t)atomic64_inc_return(&optrace_idx) - 1;
	e = &optrace_ring[i & (optrace_n - 1)];
	e->arrival = arrival;
	e->start = arrival;
	e->end = arrival;
	e->host_seq = host_seq;
	e->ch = 255;
	e->lun = 255;
	e->tier = large;
	e->cmd = OPTRACE_CMD_HOST_RD;
	e->io_type = 0;
	e->kind = tn;
	e->blat_us = (uint16_t)min_t(uint32_t, bytes >> 10, 65535);
	e->hold_ns = bytes;
}

struct optrace_ent *ssd_optrace_ring(void)
{
	return optrace_ring;
}

static inline uint64_t __get_ioclock(struct ssd *ssd)
{
	int cpu_disp = (int)ssd->cpu_nr_dispatcher;
	int cpu_qs = nvmev_vdev ? nvmev_vdev->config.cpu_nr_qserve : -1;

	if (cpu_qs < 0)
		return cpu_clock(cpu_disp);

	return cpu_clock(cpu_qs);
}

void ssd_init_params(struct ssdparams *spp, uint64_t capacity, uint32_t nparts)
{
	uint64_t blk_size, total_size;

	spp->secsz = LBA_SIZE;
	spp->secs_per_pg = 4096 / LBA_SIZE;
	spp->pgsz = spp->secsz * spp->secs_per_pg;

	spp->nchs = NAND_CHANNELS;
	spp->pls_per_lun = PLNS_PER_LUN;
	spp->luns_per_ch = LUNS_PER_NAND_CH;
	spp->cell_mode = CELL_MODE;

	NVMEV_ASSERT((spp->nchs % nparts) == 0);
	spp->nchs /= nparts;
	capacity /= nparts;

	if (BLKS_PER_PLN > 0) {
		{
			uint32_t eff;

			tlc_line_cnt = BLKS_PER_PLN_HYBRID * PROPORTION_TLC_LINE / 100;
			slc_line_cnt = BLKS_PER_PLN_HYBRID * PROPORTION_SLC_LINE / 100;
			eff = (uint32_t)tlc_line_cnt + (uint32_t)slc_line_cnt / PSLC_DENSITY_DIV;
			blk_size = DIV_ROUND_UP(capacity, (uint64_t)eff * spp->pls_per_lun *
								  spp->luns_per_ch * spp->nchs);
		}

		spp->blks_per_pl = (uint32_t)(tlc_line_cnt + slc_line_cnt);
	} else {
		NVMEV_ASSERT(BLK_SIZE > 0);
		blk_size = BLK_SIZE;
		spp->blks_per_pl = DIV_ROUND_UP(capacity, blk_size * spp->pls_per_lun *
								  spp->luns_per_ch * spp->nchs);
	}

	NVMEV_ASSERT((ONESHOT_PAGE_SIZE % spp->pgsz) == 0 && (FLASH_PAGE_SIZE % spp->pgsz) == 0);
	NVMEV_ASSERT((ONESHOT_PAGE_SIZE % FLASH_PAGE_SIZE) == 0);

	spp->pgs_per_oneshotpg = ONESHOT_PAGE_SIZE / (spp->pgsz);
	spp->oneshotpgs_per_blk = DIV_ROUND_UP(blk_size, ONESHOT_PAGE_SIZE);

	spp->pgs_per_flashpg = FLASH_PAGE_SIZE / (spp->pgsz);
	spp->flashpgs_per_blk = (ONESHOT_PAGE_SIZE / FLASH_PAGE_SIZE) * spp->oneshotpgs_per_blk;

	spp->flashpgs_slc_per_blk = spp->flashpgs_per_blk / PSLC_DENSITY_DIV;
	if (spp->flashpgs_slc_per_blk < 1)
		spp->flashpgs_slc_per_blk = 1;

	spp->pgs_per_blk = spp->pgs_per_oneshotpg * spp->oneshotpgs_per_blk;

	spp->write_unit_size = WRITE_UNIT_SIZE;

	spp->pg_4kb_rd_lat[CELL_TYPE_LSB] = NAND_4KB_READ_LATENCY_LSB;
	spp->pg_4kb_rd_lat[CELL_TYPE_MSB] = NAND_4KB_READ_LATENCY_MSB;
	spp->pg_4kb_rd_lat[CELL_TYPE_CSB] = NAND_4KB_READ_LATENCY_CSB;
	spp->pg_rd_lat[CELL_TYPE_LSB] = NAND_READ_LATENCY_LSB;
	spp->pg_rd_lat[CELL_TYPE_MSB] = NAND_READ_LATENCY_MSB;
	spp->pg_rd_lat[CELL_TYPE_CSB] = NAND_READ_LATENCY_CSB;
	spp->pg_wr_lat = NAND_PROG_LATENCY;
	spp->blk_er_lat = NAND_ERASE_LATENCY;
	spp->max_ch_xfer_size = MAX_CH_XFER_SIZE;

	spp->fw_4kb_rd_lat = FW_4KB_READ_LATENCY;
	spp->fw_rd_lat = FW_READ_LATENCY;
	spp->fw_ch_xfer_lat = FW_CH_XFER_LATENCY;
	spp->fw_wbuf_lat0 = FW_WBUF_LATENCY0;
	spp->fw_wbuf_lat1 = FW_WBUF_LATENCY1;

	spp->ch_bandwidth = NAND_CHANNEL_BANDWIDTH;
	spp->pcie_bandwidth = PCIE_BANDWIDTH;

	spp->secs_per_blk = spp->secs_per_pg * spp->pgs_per_blk;
	spp->secs_per_pl = spp->secs_per_blk * spp->blks_per_pl;
	spp->secs_per_lun = spp->secs_per_pl * spp->pls_per_lun;
	spp->secs_per_ch = spp->secs_per_lun * spp->luns_per_ch;
	spp->tt_secs = spp->secs_per_ch * spp->nchs;

	spp->pgs_per_pl = spp->pgs_per_blk * spp->blks_per_pl;
	spp->pgs_per_lun = spp->pgs_per_pl * spp->pls_per_lun;
	spp->pgs_per_ch = spp->pgs_per_lun * spp->luns_per_ch;
	spp->tt_pgs = spp->pgs_per_ch * spp->nchs;

	spp->blks_per_lun = spp->blks_per_pl * spp->pls_per_lun;
	spp->blks_per_ch = spp->blks_per_lun * spp->luns_per_ch;
	spp->tt_blks = spp->blks_per_ch * spp->nchs;

	spp->pls_per_ch = spp->pls_per_lun * spp->luns_per_ch;
	spp->tt_pls = spp->pls_per_ch * spp->nchs;

	spp->tt_luns = spp->luns_per_ch * spp->nchs;

	spp->blks_per_line = spp->tt_pls;
	spp->pgs_per_line = spp->blks_per_line * spp->pgs_per_blk;
	spp->secs_per_line = spp->pgs_per_line * spp->secs_per_pg;
	spp->tt_lines = spp->blks_per_pl;

	total_size = (unsigned long)spp->tt_luns * spp->blks_per_lun * spp->pgs_per_blk *
		     spp->secsz * spp->secs_per_pg;
	blk_size = spp->pgs_per_blk * spp->secsz * spp->secs_per_pg;

	{
		uint32_t slc_lines = (uint32_t)slc_line_cnt;
		uint32_t tlc_lines = spp->tt_lines > slc_lines ? spp->tt_lines - slc_lines : 0;
		uint64_t tlc_line_b = (uint64_t)spp->pgs_per_line * spp->pgsz;
		uint64_t slc_line_b = (uint64_t)slc_pgs_per_blk(spp) * spp->blks_per_line * spp->pgsz;
		uint64_t phys = (uint64_t)tlc_lines * tlc_line_b + (uint64_t)slc_lines * slc_line_b;

		NVMEV_INFO(
			"Total Capacity(GiB,MiB)=%llu,%llu chs=%u luns=%lu lines=%lu blk-size(MiB,KiB)=%u,%u line-size(MiB,KiB)=%lu,%lu",
			BYTE_TO_GB(phys), BYTE_TO_MB(phys), spp->nchs, spp->tt_luns,
			spp->tt_lines, BYTE_TO_MB(spp->pgs_per_blk * spp->pgsz),
			BYTE_TO_KB(spp->pgs_per_blk * spp->pgsz),
			BYTE_TO_MB(spp->pgs_per_line * spp->pgsz),
			BYTE_TO_KB(spp->pgs_per_line * spp->pgsz));
		NVMEV_INFO(
			"  capacity by mode: TLC %u lines x %lluMiB = %lluMiB | SLC %u x %lluMiB = %lluMiB"
			" | as all-TLC %lluMiB\n",
			tlc_lines, BYTE_TO_MB(tlc_line_b),
			BYTE_TO_MB((uint64_t)tlc_lines * tlc_line_b),
			slc_lines, BYTE_TO_MB(slc_line_b),
			BYTE_TO_MB((uint64_t)slc_lines * slc_line_b),
			BYTE_TO_MB(total_size));
	}

	NVMEV_INFO("  Read Latency (ns) : 4KB LSB=%u, 4KB MSB=%u, 4KB CSB=%u, LSB=%u, MSB=%u, CSB=%u\n",
			spp->pg_4kb_rd_lat[CELL_TYPE_LSB], spp->pg_4kb_rd_lat[CELL_TYPE_MSB],
			spp->pg_4kb_rd_lat[CELL_TYPE_CSB], spp->pg_rd_lat[CELL_TYPE_LSB],
			spp->pg_rd_lat[CELL_TYPE_MSB], spp->pg_rd_lat[CELL_TYPE_CSB]);
	NVMEV_INFO("  Write Latency (ns) : %u\n", spp->pg_wr_lat);
	NVMEV_INFO("  Erase Latency (ns) : %u\n", spp->blk_er_lat);
	NVMEV_INFO("  NAND CH Bandwidth (MiB/s) : %llu\n", spp->ch_bandwidth);

	NVMEV_INFO("  NAND read model : tR = page, channel = %d B (whole flash page), PCIe = request size\n",
		   spp->pgsz * spp->pgs_per_flashpg);
	NVMEV_INFO("  PCIe Bandwidth (MiB/s) : %llu\n", spp->pcie_bandwidth);
}

static void ssd_init_nand_page(struct nand_page *pg, struct ssdparams *spp)
{
	int i;
	pg->nsecs = spp->secs_per_pg;
	pg->sec = kmalloc(sizeof(nand_sec_status_t) * pg->nsecs, GFP_KERNEL);
	for (i = 0; i < pg->nsecs; i++) {
		pg->sec[i] = SEC_FREE;
	}
	pg->status = PG_FREE;
}

static void ssd_remove_nand_page(struct nand_page *pg)
{
	kfree(pg->sec);
}

static void ssd_init_nand_blk(struct nand_block *blk, struct ssdparams *spp)
{
	int i;
	blk->npgs = spp->pgs_per_blk;
	blk->pg = kmalloc(sizeof(struct nand_page) * blk->npgs, GFP_KERNEL);
	for (i = 0; i < blk->npgs; i++) {
		ssd_init_nand_page(&blk->pg[i], spp);
	}
	blk->ipc = 0;
	blk->vpc = 0;
	blk->erase_cnt = 0;
	blk->wp = 0;
}

static void ssd_remove_nand_blk(struct nand_block *blk)
{
	int i;

	for (i = 0; i < blk->npgs; i++)
		ssd_remove_nand_page(&blk->pg[i]);

	kfree(blk->pg);
}

static void ssd_init_nand_plane(struct nand_plane *pl, struct ssdparams *spp)
{
	int i;
	pl->nblks = spp->blks_per_pl;
	pl->blk = kmalloc(sizeof(struct nand_block) * pl->nblks, GFP_KERNEL);
	for (i = 0; i < pl->nblks; i++) {
		ssd_init_nand_blk(&pl->blk[i], spp);
	}
}

static void ssd_remove_nand_plane(struct nand_plane *pl)
{
	int i;

	for (i = 0; i < pl->nblks; i++)
		ssd_remove_nand_blk(&pl->blk[i]);

	kfree(pl->blk);
}

static void ssd_init_nand_lun(struct nand_lun *lun, struct ssdparams *spp)
{
	int i;
	lun->npls = spp->pls_per_lun;
	lun->pl = kmalloc(sizeof(struct nand_plane) * lun->npls, GFP_KERNEL);
	for (i = 0; i < lun->npls; i++) {
		ssd_init_nand_plane(&lun->pl[i], spp);
	}
	lun->next_lun_avail_time = 0;
	spin_lock_init(&lun->q_lock);
	memset(lun->q_subtail, 0, sizeof(lun->q_subtail));
	lun->busy = false;
}

static void ssd_remove_nand_lun(struct nand_lun *lun)
{
	int i;

	for (i = 0; i < lun->npls; i++)
		ssd_remove_nand_plane(&lun->pl[i]);

	kfree(lun->pl);
}

static void ssd_init_ch(struct ssd_channel *ch, struct ssdparams *spp)
{
	int i;
	ch->nluns = spp->luns_per_ch;
	ch->lun = kmalloc(sizeof(struct nand_lun) * ch->nluns, GFP_KERNEL);
	if (!ch->lun) {
		NVMEV_ERROR("nand_lun alloc FAILED (%zu B)\n",
			    sizeof(struct nand_lun) * ch->nluns);
		ch->perf_model = NULL;
		return;
	}
	for (i = 0; i < ch->nluns; i++) {
		ssd_init_nand_lun(&ch->lun[i], spp);
	}

	ch->perf_model = kmalloc(sizeof(struct channel_model),
				 GFP_KERNEL | __GFP_RETRY_MAYFAIL | __GFP_NOWARN);
	if (!ch->perf_model) {
		NVMEV_INFO("channel_model %zu KiB kmalloc failed (fragmentation) -> falling back to kvmalloc\n",
			   sizeof(struct channel_model) >> 10);
		ch->perf_model = kvmalloc(sizeof(struct channel_model), GFP_KERNEL);
	}
	if (!ch->perf_model) {
		NVMEV_ERROR("channel_model alloc FAILED (%zu KiB) -- out of memory\n",
			    sizeof(struct channel_model) >> 10);
		return;
	}
	chmodel_init(ch->perf_model, spp->ch_bandwidth);

	ch->perf_model->cmd_budget = 0;

	ch->perf_model->xfer_lat += (spp->fw_ch_xfer_lat * UNIT_XFER_SIZE / KB(4));
}

static void ssd_remove_ch(struct ssd_channel *ch)
{
	int i;

	kvfree(ch->perf_model);

	for (i = 0; i < ch->nluns; i++)
		ssd_remove_nand_lun(&ch->lun[i]);

	kfree(ch->lun);
}

static void ssd_init_pcie(struct ssd_pcie *pcie, struct ssdparams *spp)
{
	pcie->perf_model = kmalloc(sizeof(struct channel_model),
				   GFP_KERNEL | __GFP_RETRY_MAYFAIL | __GFP_NOWARN);
	if (!pcie->perf_model)
		pcie->perf_model = kvmalloc(sizeof(struct channel_model), GFP_KERNEL);
	if (!pcie->perf_model) {
		NVMEV_ERROR("pcie channel_model alloc FAILED\n");
		return;
	}
	chmodel_init(pcie->perf_model, spp->pcie_bandwidth);
}

static void lunq3_q_init(struct lun_queue *q);

int ssd_init(struct ssd *ssd, struct ssdparams *spp, uint32_t cpu_nr_dispatcher)
{
	uint32_t i;

	ssd->sp = *spp;

	ssd->ch = kmalloc(sizeof(struct ssd_channel) * spp->nchs, GFP_KERNEL);
	if (!ssd->ch) {
		NVMEV_ERROR("ssd_channel array alloc FAILED\n");
		return -ENOMEM;
	}

	{
		size_t nlun = (size_t)spp->nchs * spp->luns_per_ch;
		size_t lq3_sz = sizeof(struct lun_queue) * nlun;
		size_t ib_sz = sizeof(struct lunq3_inbox) * nlun;
		size_t ob_sz = sizeof(struct lunq3_outbox);

		NVMEV_INFO("lunq3 alloc: qmax=%d lq3=%zu MiB inbox=%zu MiB outbox=%zu KiB\n",
			   lunq3_qmax, lq3_sz >> 20, ib_sz >> 20, ob_sz >> 10);

		ssd->lq3 = vzalloc(lq3_sz);
		ssd->inbox = vzalloc(ib_sz);
		ssd->outbox = vzalloc(ob_sz);
		if (!ssd->lq3 || !ssd->inbox || !ssd->outbox)
			NVMEV_ERROR("lunq3 alloc FAILED (lq3=%p inbox=%p outbox=%p) -- lower LUNQ3_QMAX_MAX\n",
				    ssd->lq3, ssd->inbox, ssd->outbox);
	}
	if (ssd->lq3) {
		uint32_t ch, lun;

		for (ch = 0; ch < spp->nchs; ch++)
			for (lun = 0; lun < spp->luns_per_ch; lun++) {
				spin_lock_init(&ssd->lq3[ch][lun].lock);

				lunq3_q_init(&ssd->lq3[ch][lun]);
			}
	}
	for (i = 0; i < spp->nchs; i++) {
		ssd_init_ch(&(ssd->ch[i]), spp);

		if (!ssd->ch[i].perf_model) {
			NVMEV_ERROR("ssd_init_ch(%u) FAILED -- device not created\n", i);
			return -ENOMEM;
		}
	}

	ssd->cpu_nr_dispatcher = cpu_nr_dispatcher;

	ssd->pcie = kmalloc(sizeof(struct ssd_pcie), GFP_KERNEL);
	if (!ssd->pcie) {
		NVMEV_ERROR("ssd_pcie alloc FAILED\n");
		return -ENOMEM;
	}
	ssd_init_pcie(ssd->pcie, spp);
	if (!ssd->pcie->perf_model) {
		NVMEV_ERROR("pcie perf_model alloc FAILED -- device not created\n");
		return -ENOMEM;
	}

	return 0;
}

void ssd_remove(struct ssd *ssd)
{
	uint32_t i;

	if (ssd->pcie) {
		kvfree(ssd->pcie->perf_model);
		kfree(ssd->pcie);
	}

	for (i = 0; i < ssd->sp.nchs; i++) {
		ssd_remove_ch(&(ssd->ch[i]));
	}

	vfree(ssd->lq3);
	vfree(ssd->inbox);
	vfree(ssd->outbox);
	kfree(ssd->ch);
}

uint64_t ssd_advance_pcie(struct ssd *ssd, uint64_t request_time, uint64_t length)
{
	struct channel_model *perf_model = ssd->pcie->perf_model;
	return chmodel_request(perf_model, request_time, length, -1, NULL);
}

uint64_t ssd_advance_write_buffer(struct ssd *ssd, uint64_t request_time, uint64_t length)
{
	uint64_t nsecs_latest = request_time;
	struct ssdparams *spp = &ssd->sp;

	nsecs_latest += spp->fw_wbuf_lat0;
	nsecs_latest += spp->fw_wbuf_lat1 * DIV_ROUND_UP(length, KB(4));

	nsecs_latest = ssd_advance_pcie(ssd, nsecs_latest, length);

	return nsecs_latest;
}

#ifndef TETRIS_GC_DBG
#define TETRIS_GC_DBG 0
#endif

#define SCANREG_MAX 64
struct scanreg_ent { uint16_t id; uint64_t lo, hi; };
static struct scanreg_ent scanreg_tbl[SCANREG_MAX];
static int scanreg_n;
static DEFINE_SPINLOCK(scanreg_lock);

void scanreg_set(uint16_t id, uint64_t start_lba, uint64_t nlba)
{
	unsigned long fl;
	int i, free = -1;

	if (!id || !nlba)
		return;
	spin_lock_irqsave(&scanreg_lock, fl);
	for (i = 0; i < scanreg_n; i++) {
		if (scanreg_tbl[i].id == id) {
			scanreg_tbl[i].lo = start_lba;
			scanreg_tbl[i].hi = start_lba + nlba;
			spin_unlock_irqrestore(&scanreg_lock, fl);
			return;
		}
		if (!scanreg_tbl[i].id && free < 0)
			free = i;
	}
	if (free < 0 && scanreg_n < SCANREG_MAX)
		free = scanreg_n++;
	if (free >= 0) {
		scanreg_tbl[free].id = id;
		scanreg_tbl[free].lo = start_lba;
		scanreg_tbl[free].hi = start_lba + nlba;
	}
	spin_unlock_irqrestore(&scanreg_lock, fl);
}

void scanreg_clear(void)
{
	unsigned long fl;

	spin_lock_irqsave(&scanreg_lock, fl);
	memset(scanreg_tbl, 0, sizeof(scanreg_tbl));
	scanreg_n = 0;
	spin_unlock_irqrestore(&scanreg_lock, fl);
}

static inline uint64_t lunq_xfer_est(struct ssdparams *spp, uint64_t bytes)
{
	return spp->ch_bandwidth ? (bytes * 1000000000ULL) / (spp->ch_bandwidth << 20) : 0;
}

static inline uint64_t nand_rd_ch_bytes(const struct ssdparams *spp, uint64_t xfer)
{
	uint64_t pg;

	pg = (uint64_t)spp->pgsz * spp->pgs_per_flashpg;
	return xfer > pg ? xfer : pg;
}

u32 tfl_rdp[TETRIS_NR_DIES];

void tfl_reset(void)
{
	int d;

	for (d = 0; d < TETRIS_NR_DIES; d++) {
		WRITE_ONCE(tfl_rdp[d], 0);
	}
}

u64 qs_inbox_wait_rd, qs_inbox_wait_wr, qs_inbox_wait_er;

u64 nvmev_die_end_out;

u64 e8_over_n, e8_over_sum, e8_over_max, e8_under_n, e8_under_sum, e8_under_max;
u64 qs_pub, qs_drained_done, qs_outbox_full, qs_inbox_dropped, qs_forced_ahead, qs_budget_hit,
   qs_bp_passes, qs_bp_enter, qs_bp_run_max, qs_bp_cur;

int bp_queue_stall;
int backpressure_free_line_stall;
int nvmev_sq_stall;

void qserve_stats_reset(void)
{
	qs_pend_hwm = 0;
	qs_pub = qs_drained_done = qs_outbox_full = qs_inbox_dropped = 0;
	qs_inbox_wait_rd = qs_inbox_wait_wr = qs_inbox_wait_er = 0;
	e8_over_n = e8_over_sum = e8_over_max = 0;
	e8_under_n = e8_under_sum = e8_under_max = 0;
	qs_forced_ahead = qs_budget_hit = 0;
	qs_bp_passes = qs_bp_enter = qs_bp_run_max = qs_bp_cur = 0;
	nvmev_p4_relink = nvmev_p4_badsq = nvmev_p4_stale_owner = 0;
	nvmev_p4_badcopy = nvmev_p4_lost_cmpl = nvmev_p4_sq_stall = 0;
	nvmev_bp_fetch_skipped = 0;

	nvmev_dep_stats_reset();
	tetris_reset();
	tt_reset();
	cr_reset();
	tfl_reset();
}

u64 tetris_sd_wait[2][SK_N], tetris_sd_max[2][SK_N];

u64 pslc_rs_r_sum, pslc_rs_n, pslc_rs_r_hwm;

u64 dw_n, dw_sum, dw_sq;
u64 dw_min = ~0ULL;
u64 tw_sum;
u64 tw_min = ~0ULL;
static u64 dw_last_us, tw_last_us;

static u32 pslc_rs_k;
static u64 pslc_rs_t0_ns;
static s64 pslc_rs_st, pslc_rs_sd, pslc_rs_std, pslc_rs_stt;
static s64 pslc_rs_slope;
u64 pslc_rs_up_n, pslc_rs_dn_n, pslc_rs_hold_n;

#define PSLC_RS_NOISE_US   565ULL
#define PSLC_RS_TW_MIN_MS  100ULL
#define PSLC_RS_TW_MAX_MS  20000ULL
u64 pslc_rs_tw_min = ~0ULL;
u64 pslc_rs_tw_last;
s64 pslc_rs_noise_last;

bool pslc_rs_tw3_heavy;

static u64 pslc_rs_lvl_prev;
static bool pslc_rs_lvl_valid;
static s64 pslc_rs_dlvl;
u64 pslc_rs_undo_n;

u64 pslc_rs_probe_n;

static u32 pslc_rs_rbar_ppk;
static bool pslc_rs_rbar_valid;
static int pslc_rs_dir = 1;
u64 pslc_rs_flips, pslc_rs_rbar_sum, pslc_rs_steps;

static bool pslc_rs_cost_valid;
u64 tetris_sd_wait_tn[2][SK_N][TENANT_N + 1];
u64 tetris_sd_chnl[2], tetris_sd_chnl_max[2], tetris_sd_chnl_n[2];
u64 tt_stream_op, tt_stream_rd, tt_chain_same_stream, tt_chain_diff_stream;
u64 tt_cr_rd_n, tt_cr_cont_n;
u64 tt_cr_run_hist[8], tt_cr_run_n, tt_cr_run_sum, tt_cr_run_max;

u64 tt_cr_rd_n_tn[TETRIS_TN_MAX], tt_cr_cont_n_tn[TETRIS_TN_MAX];
u64 tt_cr_run_n_tn[TETRIS_TN_MAX], tt_cr_run_sum_tn[TETRIS_TN_MAX];
u64 tt_cr_break_gap_tn[TETRIS_TN_MAX][TETRIS_TN_MAX], tt_cr_break_wr_tn[TETRIS_TN_MAX];

u64 tt_cr_break_blk_tn[TETRIS_TN_MAX];

void tetris_stall_reset(void)
{
	tt_wb_open = 0;
	tt_wb_continue = 0;
	tt_wb_abort_rd = 0;
	tt_wb_abort_bp = 0;

	memset(tetris_sd_wait, 0, sizeof(tetris_sd_wait));
	memset(tetris_sd_max, 0, sizeof(tetris_sd_max));
	memset(tetris_sd_wait_tn, 0, sizeof(tetris_sd_wait_tn));
	memset(tetris_sd_chnl, 0, sizeof(tetris_sd_chnl));
	memset(tetris_sd_chnl_max, 0, sizeof(tetris_sd_chnl_max));
	memset(tetris_sd_chnl_n, 0, sizeof(tetris_sd_chnl_n));
	memset(tetris_sd_cache, 0, sizeof(tetris_sd_cache));
	memset(tetris_sd_cache_max, 0, sizeof(tetris_sd_cache_max));
	memset(tetris_sd_nand, 0, sizeof(tetris_sd_nand));
	memset(tetris_sd_nand_max, 0, sizeof(tetris_sd_nand_max));
	memset(tetris_sd_cnt, 0, sizeof(tetris_sd_cnt));
	memset(tetris_sd_queued_sum, 0, sizeof(tetris_sd_queued_sum));
	memset(tetris_sd_queued_max, 0, sizeof(tetris_sd_queued_max));
	tt_stream_op = tt_stream_rd = tt_chain_same_stream = tt_chain_diff_stream = 0;
	tt_cr_rd_n = tt_cr_cont_n = 0;
	memset(tt_cr_run_hist, 0, sizeof(tt_cr_run_hist));
	tt_cr_run_n = tt_cr_run_sum = tt_cr_run_max = 0;
	memset(tt_cr_rd_n_tn, 0, sizeof(tt_cr_rd_n_tn));
	memset(tt_cr_cont_n_tn, 0, sizeof(tt_cr_cont_n_tn));
	memset(tt_cr_run_n_tn, 0, sizeof(tt_cr_run_n_tn));
	memset(tt_cr_run_sum_tn, 0, sizeof(tt_cr_run_sum_tn));
	memset(tt_cr_break_gap_tn, 0, sizeof(tt_cr_break_gap_tn));

	memset(tt_cr_break_blk_tn, 0, sizeof(tt_cr_break_blk_tn));
	memset(tt_cr_break_wr_tn, 0, sizeof(tt_cr_break_wr_tn));
	lunq3_cring_drop = 0;
}
u64 tetris_sd_cache[2], tetris_sd_nand[2], tetris_sd_cnt[2];

u64 tetris_sd_queued_sum[2], tetris_sd_queued_max[2], tetris_sd_cache_max[2], tetris_sd_nand_max[2];

static inline uint8_t lunq3_kind_of(struct nand_cmd *ncmd)
{
	if (ncmd->migrate)
		return (ncmd->cmd == NAND_ERASE) ? SK_FOLD_CNZ : SK_MIGRATE;
	if (ncmd->fold || ncmd->type == FOLD_IO)
		return SK_FOLD;
	if (ncmd->cmd == NAND_ERASE)
		return SK_ERASE;
	if (ncmd->type != USER_IO)
		return SK_GC;
	if (ncmd->cmd == NAND_WRITE)
		return SK_HOST_WR;
	return ncmd->is_small_read ? SK_SMALL_RD : SK_LARGE_RD;
}

u64 er_cnt[2], er_ns[2], er_migr_cnt[2], er_migr_ns[2];
u64 lunq3_cring_drop;

static inline void lunq3_mark(struct lun_queue *q, uint8_t kind, uint64_t end, uint64_t dur)
{
	uint64_t start = end > dur ? end - dur : 0;

	while (q->ch != q->ct && q->cring[q->ch & (LUNQ3_CRING - 1)].end <= start)
		q->ch++;
	if (q->ct - q->ch >= LUNQ3_CRING) {
		q->ch++;
		lunq3_cring_drop++;
	}
	q->cring[q->ct & (LUNQ3_CRING - 1)].start = start;
	q->cring[q->ct & (LUNQ3_CRING - 1)].end = end;
	q->cring[q->ct & (LUNQ3_CRING - 1)].kind = kind;
	q->ct++;
}

#if TETRIS_STALL_DECOMP

static void lunq3_attr_resid_ro(struct lun_queue *q, uint64_t a, int L, uint8_t tn)
{
	uint64_t cur = a, charged = 0;
	uint32_t i, ch = READ_ONCE(q->ch), ct = READ_ONCE(q->ct);

	if (tn > TENANT_N)
		tn = 0;

	while (ch != ct && q->cring[ch & (LUNQ3_CRING - 1)].end <= a)
		ch++;

	for (i = ch; i != ct; i++) {
		uint8_t k = q->cring[i & (LUNQ3_CRING - 1)].kind;
		uint64_t st = q->cring[i & (LUNQ3_CRING - 1)].start;
		uint64_t e = q->cring[i & (LUNQ3_CRING - 1)].end, d;

		if (e <= cur)
			continue;
		if (st > cur) {
			tetris_sd_wait[L][SK_MIXED] += st - cur;
			tetris_sd_wait_tn[L][SK_MIXED][tn] += st - cur;
			charged += st - cur;
			cur = st;
		}
		d = e - cur;
		tetris_sd_wait[L][k] += d;
		tetris_sd_wait_tn[L][k][tn] += d;
		if (d > tetris_sd_max[L][k])
			tetris_sd_max[L][k] = d;
		cur = e;
		charged += d;
	}
}

void lunq3_attr_resid_at(struct ssd *ssd, struct ppa *ppa, uint64_t arr, int L, uint64_t dur,
			 uint8_t tn)
{
	struct lun_queue *q;
	u64 queued;

	if (!ssd->lq3)
		return;
	q = &ssd->lq3[ppa->g.ch][ppa->g.lun];

	lunq3_attr_resid_ro(q, arr, L, tn);

	queued = READ_ONCE(q->queued_ns);
	tetris_sd_queued_sum[L] += queued;
	if (queued > tetris_sd_queued_max[L])
		tetris_sd_queued_max[L] = queued;

	tetris_sd_nand[L] += dur;
	if (dur > tetris_sd_nand_max[L])
		tetris_sd_nand_max[L] = dur;
	tetris_sd_cnt[L]++;
}
#endif

static inline struct lun_queue *get_lq3(struct ssd *ssd, struct ppa *ppa)
{
	return &ssd->lq3[ppa->g.ch][ppa->g.lun];
}

static void lunq3_q_init(struct lun_queue *q)
{
	int k;

	BUILD_BUG_ON(LUNQ3_QMAX_MAX >= LUNQ3_NIL);
	q->fifo_head = q->fifo_tail = LUNQ3_NIL;
	for (k = 0; k < LUNQ3_QMAX_MAX; k++) {
		q->prv[k] = LUNQ3_NIL;
		q->nxt[k] = (k + 1 < LUNQ3_QMAX_MAX) ? (uint16_t)(k + 1) : LUNQ3_NIL;
	}
	q->free_head = 0;

	{
		int w;

		for (w = 0; w < TETRIS_NQ; w++) {
			q->fq[w].head = q->fq[w].tail = LUNQ3_NIL;
			q->fq[w].n = 0;
		}
		for (k = 0; k < LUNQ3_QMAX_MAX; k++) {
			q->fq_which[k] = TETRIS_NQ;
			q->fq_next[k] = q->fq_prev[k] = LUNQ3_NIL;
		}
	}

	memset(q->gc_numwr, 0, sizeof(q->gc_numwr));
	q->chain_debt = 0;
	q->wbatch_left = 0;
	q->wbatch_gc_done = 0;
}

static inline int lunq3_slot_alloc(struct lun_queue *q)
{
	int i = q->free_head;

	if (i == LUNQ3_NIL) {
		return -1;
	}
	q->free_head = q->nxt[i];
	return i;
}

static inline void lunq3_slot_free(struct lun_queue *q, int i)
{
	q->nxt[i] = q->free_head;
	q->prv[i] = LUNQ3_NIL;
	q->free_head = (uint16_t)i;
}

static inline void lunq3_fifo_add_tail(struct lun_queue *q, int i)
{
	NVMEV_ASSERT(i >= 0 && i < LUNQ3_QMAX_MAX);
	q->nxt[i] = LUNQ3_NIL;
	q->prv[i] = q->fifo_tail;
	if (q->fifo_tail != LUNQ3_NIL)
		q->nxt[q->fifo_tail] = (uint16_t)i;
	else
		q->fifo_head = (uint16_t)i;
	q->fifo_tail = (uint16_t)i;
}

static inline void lunq3_fifo_del(struct lun_queue *q, int i)
{
	NVMEV_ASSERT(i >= 0 && i < LUNQ3_QMAX_MAX);
	if (q->prv[i] != LUNQ3_NIL)
		q->nxt[q->prv[i]] = q->nxt[i];
	else
		q->fifo_head = q->nxt[i];
	if (q->nxt[i] != LUNQ3_NIL)
		q->prv[q->nxt[i]] = q->prv[i];
	else
		q->fifo_tail = q->prv[i];
}

static inline int lunq3_fifo_head(const struct lun_queue *q)
{
	return q->fifo_head == LUNQ3_NIL ? -1 : (int)q->fifo_head;
}

static inline int lunq3_fifo_next(const struct lun_queue *q, int i)
{
	return q->nxt[i] == LUNQ3_NIL ? -1 : (int)q->nxt[i];
}

#define lunq3_for_each(q, i) \
	for ((i) = lunq3_fifo_head(q); (i) >= 0; (i) = lunq3_fifo_next((q), (i)))

static inline enum tetris_q tetris_q_of(const struct lunq3_op *o)
{
	if (o->op_type == USER_IO)
		return o->op_cmd == NAND_READ ? TETRIS_RDQ : TETRIS_WRQ;

	return o->op_cmd == NAND_READ ? TETRIS_GC_RDQ : TETRIS_GC_WRQ;
}

static inline u8 tetris_flow_of(const struct lunq3_op *o)
{
	return conv_qserve_tenant_of_ppa((struct ppa *)&o->ppa);
}

static u64 tetris_alone_done[8][8][TETRIS_TN_MAX][2];

#define TT_CLS_RD 0
#define TT_CLS_WR 1
#define TT_CLS_BG 2

static inline void lunq3_fifo_insert_after(struct lun_queue *q, int pos, int i)
{
	if (pos < 0) {
		q->prv[i] = LUNQ3_NIL;
		q->nxt[i] = q->fifo_head;
		if (q->fifo_head != LUNQ3_NIL)
			q->prv[q->fifo_head] = (uint16_t)i;
		else
			q->fifo_tail = (uint16_t)i;
		q->fifo_head = (uint16_t)i;
		return;
	}
	q->prv[i] = (uint16_t)pos;
	q->nxt[i] = q->nxt[pos];
	if (q->nxt[pos] != LUNQ3_NIL)
		q->prv[q->nxt[pos]] = (uint16_t)i;
	else
		q->fifo_tail = (uint16_t)i;
	q->nxt[pos] = (uint16_t)i;
}

static inline void tetris_q_push(struct lun_queue *q, int slot)
{
	enum tetris_q w = tetris_q_of(&q->pend[slot]);
	struct tetris_queue *fq = &q->fq[w];

	q->fq_which[slot] = (uint8_t)w;
	q->fq_next[slot] = LUNQ3_NIL;
	q->fq_prev[slot] = fq->tail;
	if (fq->tail != LUNQ3_NIL)
		q->fq_next[fq->tail] = (uint16_t)slot;
	else
		fq->head = (uint16_t)slot;
	fq->tail = (uint16_t)slot;
	fq->n++;
	q->fq_ver++;
}

static inline void tetris_q_erase(struct lun_queue *q, int slot)
{
	uint8_t w = q->fq_which[slot];
	struct tetris_queue *fq;
	uint16_t pv, nx;

	if (w >= TETRIS_NQ)
		return;
	fq = &q->fq[w];
	pv = q->fq_prev[slot];
	nx = q->fq_next[slot];
	if (pv != LUNQ3_NIL)
		q->fq_next[pv] = nx;
	else
		fq->head = nx;
	if (nx != LUNQ3_NIL)
		q->fq_prev[nx] = pv;
	else
		fq->tail = pv;
	fq->n--;
	q->fq_which[slot] = TETRIS_NQ;
	q->fq_next[slot] = q->fq_prev[slot] = LUNQ3_NIL;
	q->fq_ver++;
}

static inline int tt_budget_init(void)
{
	return INT_MAX;
}

static u64 lunq3_gseq_next;

static int lunq3_push(struct lun_queue *q, const struct lunq3_op *src)
{
	int i = lunq3_slot_alloc(q);

	NVMEV_ASSERT(i >= 0);
	q->pend[i] = *src;

	if (q->pend[i].bypass_n == 0) {
		q->pend[i].bypass_n = tt_budget_init();
	}
	q->pend[i].seq = q->seq++;

	q->pend[i].gseq = lunq3_gseq_next++;

	q->queued_ns += q->pend[i].dur;
	q->n++;
	if (q->pend[i].op_type == USER_IO && q->pend[i].op_cmd == NAND_READ) {
		q->host_rd_n++;
	}

	if (q->pend[i].op_type != USER_IO && q->pend[i].op_cmd == NAND_READ)
		q->gc_rd_n++;
	tetris_q_push(q, i);

	{
		struct lunq3_op *o = &q->pend[i];
		u32 ch = o->ppa.g.ch, lun = o->ppa.g.lun;
		u8 tn = tetris_flow_of(o);
		int rw = (o->op_cmd == NAND_READ) ? 0 : 1;

		if (ch < 8 && lun < 8 && tn < TETRIS_TN_MAX) {
			u64 *tl = &tetris_alone_done[ch][lun][tn][rw];
			u64 base = *tl > o->arr ? *tl : o->arr;
			u64 w = base > o->arr ? base - o->arr : 0;

			o->tetris_alone_ns = w > 0xFFFFFFFFULL ? 0xFFFFFFFFU : (u32)w;
			*tl = base + o->dur;
		} else {
			o->tetris_alone_ns = 0;
		}
	}

	lunq3_fifo_add_tail(q, i);
	return i;
}

static void lunq3_pop(struct lun_queue *q, int i)
{
	if (i != lunq3_fifo_head(q))
		;
	q->queued_ns -= q->pend[i].dur;
	if (q->pend[i].op_type == USER_IO && q->pend[i].op_cmd == NAND_READ) {
		NVMEV_ASSERT(q->host_rd_n > 0);
		q->host_rd_n--;
	}
	if (q->pend[i].op_type != USER_IO && q->pend[i].op_cmd == NAND_READ) {
		NVMEV_ASSERT(q->gc_rd_n > 0);
		q->gc_rd_n--;
	}
	q->n--;
	tetris_q_erase(q, i);
	lunq3_fifo_del(q, i);
	lunq3_slot_free(q, i);
}

static inline void ssd_optrace_record_serve(const struct lunq3_op *o, uint64_t start, uint64_t end,
					    uint64_t die_free)
{
	struct optrace_ent *e;
	uint64_t i;
	uint64_t hold;
	uint64_t blat;

	if (likely(op_trace != 2) || !optrace_ring)
		return;
	i = (uint64_t)atomic64_inc_return(&optrace_idx) - 1;
	e = &optrace_ring[i & (optrace_n - 1)];
	e->arrival = o->arr;
	e->start = start;
	e->end = end;
	e->host_seq = o->host_seq;
	e->ch = (uint8_t)o->ppa.g.ch;
	e->lun = (uint8_t)o->ppa.g.lun;
	e->tier = o->pslc;
	e->cmd = o->op_cmd;
	e->io_type = o->op_type;
	e->kind = o->kind;
	blat = o->base_lat / 1000;
	e->blat_us = blat > 65535 ? 65535 : (uint16_t)blat;
	hold = die_free > start ? die_free - start : 0;
	e->hold_ns = hold > 0xffffffffULL ? 0xffffffffU : (uint32_t)hold;
}

static struct {
	uint64_t *ring;
	uint32_t head, tail, mask;
	uint32_t resident, inflight, limit;

	uint32_t reserve_cur;
	uint32_t raw_win;
	uint32_t win_n;

	uint32_t pgs_per_slc_blk;
	uint32_t fold_since_erase;
	uint32_t slc_blk;
	bool slc_blk_valid;
} pslc;

#define PSLC_DL_MAX 1024
static uint64_t pslc_dl[PSLC_DL_MAX];
static uint32_t pslc_dl_n;

static uint32_t pslc_est_n;

void pslc_note_slc_block(uint32_t blk)
{
	pslc.slc_blk = blk;
	pslc.slc_blk_valid = true;
}

#define PSLC_ADAPT_N 8192

uint32_t pslc_reserve_pgs(void)
{
	if (pslc_raw_reserve)
		return pslc.reserve_cur;
	return 0;
}

uint32_t pslc_mig_legacy_pgs(void)
{
	uint32_t per_line;

	if (slc_line_cnt <= 0 || !pslc.limit)
		return 0;
	per_line = pslc.limit / (uint32_t)slc_line_cnt;
	return per_line * (uint32_t)(gc_reserve_lines + 1);
}

void pslc_free(void)
{
	vfree(pslc.ring);
	memset(&pslc, 0, sizeof(pslc));
}

void pslc_set_pool_pages(uint32_t pages)
{
	pslc.limit = pages;

	pslc.reserve_cur = (uint32_t)max(0, pslc_raw_reserve_pgs);
}

uint32_t pslc_rs_cmax(void)
{
	return pslc_rs_cmax_pgs > 0 ? (u32)pslc_rs_cmax_pgs : pslc.limit / 2;
}

static inline bool ppa_is_slc(struct ssd *ssd, struct ppa *ppa)
{
	if (!ppa || ppa->ppa == UNMAPPED_PPA)
		return false;
	return get_blk(ssd, ppa)->slc_mode;
}

bool nvmev_ppa_is_slc(struct ssd *ssd, struct ppa *ppa)
{
	return ppa_is_slc(ssd, ppa);
}

uint64_t ssd_program_ns(struct ssd *ssd, struct ppa *ppa)
{
	return ppa_is_slc(ssd, ppa) ? (uint64_t)pslc_prog_ns : (uint64_t)ssd->sp.pg_wr_lat;
}

void pslc_fold_observe(uint64_t done, bool from_nand)
{
	if (pslc_dl_n >= PSLC_DL_MAX) {
		return;
	}
	pslc_dl[pslc_dl_n++] = done;
	if (from_nand && pslc_est_n) {
		pslc_est_n--;
	}
}

void pslc_window_reset(void)
{
	pslc.win_n = 0;
	pslc.raw_win = 0;
	pslc_rawwin_n = pslc_rawwin_min = 0;
	pslc_reserve_sum = pslc_reserve_n = pslc_reserve_hwm = 0;

	pslc_rs_r_sum = pslc_rs_n = pslc_rs_r_hwm = 0;

	pslc_rs_rbar_ppk = 0;
	pslc_rs_rbar_valid = false;
	pslc_rs_dir = 1;
	pslc_rs_flips = pslc_rs_rbar_sum = pslc_rs_steps = 0;
	pslc_rs_cost_valid = false;
	pslc_rs_k = 0;
	pslc_rs_t0_ns = 0;
	pslc_rs_st = pslc_rs_sd = pslc_rs_std = pslc_rs_stt = 0;
	pslc_rs_slope = 0;
	pslc_rs_up_n = pslc_rs_dn_n = pslc_rs_hold_n = 0;
	pslc_rs_tw_min = ~0ULL;
	pslc_rs_tw_last = 0;
	pslc_rs_noise_last = 0;

	pslc_rs_tw3_heavy = false;
	pslc_rs_lvl_prev = 0;
	pslc_rs_lvl_valid = false;
	pslc_rs_dlvl = 0;
	pslc_rs_undo_n = 0;
	pslc_rs_probe_n = 0;

	pslc.reserve_cur = (uint32_t)max(0, pslc_raw_reserve_pgs);

	slcline_open = slcline_full = slcline_gc_victim = slcline_reclaimed = 0;
	slcline_host_pgs = slcline_gc_moved = slcline_free_hwm = slcline_starved = 0;
	slcline_strict_fb = 0;

	memset(la_take, 0, sizeof(la_take));
	memset(la_fail, 0, sizeof(la_fail));
	la_edge = la_calls = 0;
	la_lo_after = ~0ULL;
	cr_free_lo = ~0ULL;
	memset(la_freehist, 0, sizeof(la_freehist));
	memset(cyc_alloc, 0, sizeof(cyc_alloc));
	BUILD_BUG_ON(RAWK_SLOTS < 3);
	slcline_raw_rebind = slcline_raw_nospace = 0;
}

static u32 pslc_rs_deriv(u64 d_us, u32 cmax)
{
	u32 m;
	u32 step;
	u64 t_ms;
	u64 lvl;
	u64 tw;
	s64 num, den;
	s64 nxt;
	s64 noise;
	bool heavy;

	m = pslc_rs_dwell > 0 ? (u32)pslc_rs_dwell : 1;
	if (!pslc_rs_k)
		pslc_rs_t0_ns = diq_last_done;
	t_ms = diq_last_done > pslc_rs_t0_ns
	     ? (diq_last_done - pslc_rs_t0_ns) / 1000000 : 0;
	pslc_rs_k++;
	pslc_rs_st  += (s64)t_ms;
	pslc_rs_sd  += (s64)d_us;
	pslc_rs_std += (s64)t_ms * (s64)d_us;
	pslc_rs_stt += (s64)t_ms * (s64)t_ms;
	if (pslc_rs_k < m)
		return pslc.reserve_cur;

	num = (s64)m * pslc_rs_std - pslc_rs_st * pslc_rs_sd;
	den = (s64)m * pslc_rs_stt - pslc_rs_st * pslc_rs_st;
	pslc_rs_slope = den > 0 ? num * 1000 / den : 0;

	tw = t_ms;
	if (tw < PSLC_RS_TW_MIN_MS) {
		tw = PSLC_RS_TW_MIN_MS;
	} else if (tw > PSLC_RS_TW_MAX_MS) {
		tw = PSLC_RS_TW_MAX_MS;
	}
	noise = (s64)((PSLC_RS_NOISE_US * 1000ULL + tw / 2) / tw);
	pslc_rs_noise_last = noise;
	pslc_rs_tw_last = t_ms;
	if (t_ms < pslc_rs_tw_min)
		pslc_rs_tw_min = t_ms;

	heavy = t_ms < (u64)pslc_rs_tw3_ms;
	pslc_rs_tw3_heavy = heavy;
	if (heavy)
		;
	else
		;

	step = pslc_mig_legacy_pgs() / 2;
	if (!step)
		step = 1;

	lvl = (u64)(pslc_rs_sd / (s64)m);
	pslc_rs_dlvl = pslc_rs_lvl_valid ? (s64)lvl - (s64)pslc_rs_lvl_prev : 0;

	nxt = (s64)pslc.reserve_cur;

	if (den <= 0) {
		pslc_rs_hold_n++;
	} else if (pslc_rs_slope > noise) {
		nxt += step;
		pslc_rs_up_n++;
	} else if (pslc_rs_slope < -noise) {
		nxt -= step;
		pslc_rs_dn_n++;
	} else {
		pslc_rs_hold_n++;
	}
	if (nxt < (s64)pslc_raw_reserve_pgs)
		nxt = pslc_raw_reserve_pgs;
	if (nxt > (s64)cmax)
		nxt = cmax;
	if (nxt != (s64)pslc.reserve_cur) {
		pslc.reserve_cur = (u32)nxt;
		pslc_rs_steps++;
	}
	pslc_rs_lvl_prev = lvl;
	pslc_rs_lvl_valid = true;
	pslc_rs_k = 0;
	pslc_rs_st = pslc_rs_sd = pslc_rs_std = pslc_rs_stt = 0;
	return (u32)nxt;
}

void pslc_note_flush(bool raw)
{
	if (!pslc.limit)
		return;

	pslc.win_n++;
	if (raw)
		pslc.raw_win++;
	if (pslc.win_n >= PSLC_ADAPT_N) {
		uint32_t raw_win = pslc.raw_win;
		uint32_t want = clamp_t(uint32_t, raw_win, (uint32_t)pslc_raw_reserve_pgs,
					pslc.limit / 2);

		if (raw_win <= (uint32_t)pslc_raw_reserve_pgs)
			;
		else if (raw_win >= pslc.limit / 2)
			;
		else
			;
		pslc_rawwin_n++;

		if (pslc_rawwin_n == 1 || raw_win < pslc_rawwin_min)
			pslc_rawwin_min = raw_win;

		{
			u64 _dt, _nt, _tw, _dw;

			nvmev_diq_dw_sample(&_dt, &_nt, &_tw);
			if (_nt && _tw) {
				_dw = _dt / _nt / 1000;
				_tw = _tw / 1000;
				dw_last_us = _dw;
				tw_last_us = _tw;
				dw_n++;
				dw_sum += _dw;
				dw_sq  += _dw * _dw;
				if (_dw < dw_min)
					dw_min = _dw;
				tw_sum += _tw;
				if (_tw < tw_min)
					tw_min = _tw;
			}
		}

		{
			u64 d_mig, n_mig, d_oth, n_oth, t_ns;
			u64 m_mig, m_oth;
			u32 r_ppk, want2, cmax;

			nvmev_diq_mig_window4(&d_mig, &n_mig, &d_oth, &n_oth, &t_ns);
			cmax = pslc_rs_cmax();

			if (t_ns < 1000000 ||
			    (!n_mig && !n_oth)) {
				pslc.raw_win = 0;
				pslc.win_n = 0;
				return;
			}

			m_mig = n_mig ? d_mig / n_mig : 0;
			m_oth = n_oth ? d_oth / n_oth : 0;

			r_ppk = (n_mig && n_oth && m_mig > m_oth)
				? (u32)min_t(u64, 1000000, (m_mig - m_oth) / 1000)
				: 0;

			if (!pslc_rs_rbar_valid) {
				pslc_rs_rbar_ppk = r_ppk;
				pslc_rs_rbar_valid = true;
			} else {
				pslc_rs_rbar_ppk = (u32)(((u64)pslc_rs_rbar_ppk *
							  (1000 - pslc_rs_alpha_ppk) +
							  (u64)r_ppk * pslc_rs_alpha_ppk) / 1000);
			}
			pslc_rs_rbar_sum += pslc_rs_rbar_ppk;

			want2 = pslc_rs_deriv((d_mig + d_oth) / (n_mig + n_oth) / 1000,
					      cmax);
			pslc_rs_r_sum += r_ppk;
			pslc_rs_n++;
			if (r_ppk > pslc_rs_r_hwm)
				pslc_rs_r_hwm = r_ppk;
			if (pslc.reserve_cur > pslc_reserve_hwm)
				pslc_reserve_hwm = pslc.reserve_cur;
			pslc_reserve_sum += pslc.reserve_cur;
			pslc_reserve_n++;
			pslc.raw_win = 0;
			pslc.win_n = 0;
			return;
		}

		if (want > pslc.reserve_cur)
			pslc.reserve_cur = want;
		else
			pslc.reserve_cur -= (pslc.reserve_cur - want) / 4;
		if (pslc.reserve_cur > pslc_reserve_hwm)
			pslc_reserve_hwm = pslc.reserve_cur;
		pslc_reserve_sum += pslc.reserve_cur;
		pslc_reserve_n++;

		pslc.raw_win = 0;
		pslc.win_n = 0;
	}
}

void pslc_fold_tick(struct ssd *ssd, uint64_t now)
{
	int budget = pslc_fold_batch;
	uint32_t target, headroom;

	if (!pslc.ring)
		return;

	{
		uint32_t k = 0;

		while (k < pslc_dl_n) {
			if (pslc_dl[k] > now) {
				k++; continue;
			}
			pslc_dl[k] = pslc_dl[--pslc_dl_n];
			if (pslc.inflight) {
				pslc.head++;
				pslc.inflight--;
				if (pslc.resident)
					pslc.resident--;
			}
		}
	}

	headroom = 1;
	target = pslc.limit > headroom ? pslc.limit - headroom : 0;

	while (pslc.resident > target && pslc.inflight < (uint32_t)pslc_fold_qd && budget-- > 0) {
		uint32_t idx = pslc.head + pslc.inflight;
		struct ppa p;

		if (idx == pslc.tail)
			break;
		p.ppa = pslc.ring[idx & pslc.mask];

		{
			bool valid = get_pg(ssd, &p)->status == PG_VALID;
			uint64_t t = now;
			uint32_t unit = ssd->sp.pgsz * ssd->sp.pgs_per_oneshotpg;

			if (valid) {
				struct nand_cmd fr = {
					.type = FOLD_IO, .cmd = NAND_READ, .pslc = true, .stime = now,
					.interleave_pci_dma = false, .xfer_size = unit, .ppa = &p,
				};
				struct nand_cmd fw = {
					.type = FOLD_IO, .cmd = NAND_WRITE,
					.interleave_pci_dma = false, .xfer_size = unit, .ppa = &p,
				};
				t = ssd_advance_nand(ssd, &fr);
				fw.stime = t;
				t = ssd_advance_nand(ssd, &fw);
			}
			if (++pslc.fold_since_erase >= pslc.pgs_per_slc_blk && pslc.slc_blk_valid) {
				struct ppa ep;
				struct nand_cmd fe = {
					.type = FOLD_IO, .cmd = NAND_ERASE, .interleave_pci_dma = false,
				};

				ep.ppa = p.ppa;
				ep.g.blk = pslc.slc_blk;
				fe.stime = t;
				fe.ppa = &ep;
				t = ssd_advance_nand(ssd, &fe);
				pslc.fold_since_erase = 0;
			}

			if (!valid) {
				pslc_fold_observe(now, false);
			} else if (pslc_est_n < PSLC_DL_MAX) {
				pslc_est_n++;
			}
			pslc.inflight++;
		}
	}
}

static uint64_t ssd_serve_read(struct ssd *ssd, const struct lunq3_op *op, uint64_t s,
			      uint64_t *chnl_end_out)
{
	struct ssdparams *spp = &ssd->sp;
	struct ppa ppa = op->ppa;
	struct ssd_channel *ch = get_ch(ssd, &ppa);
	uint64_t remaining = nand_rd_ch_bytes(spp, op->xfer_size);
	uint64_t nand_etime, chnl_stime;
	uint64_t cont_ns = 0;
	uint64_t chnl_etime = s;
	uint64_t completed_time = s;

	nand_etime = s + op->base_lat;

	chnl_stime = nand_etime;
	while (remaining) {
		uint64_t xfer = min(remaining, (uint64_t)spp->max_ch_xfer_size);

		chnl_etime = chmodel_request(ch->perf_model, chnl_stime, xfer, (int)ppa.g.lun,
					     &cont_ns);

		if (op->op_type == USER_IO && op->op_cmd == NAND_READ) {
			int cl;

			cl = TETRIS_SD_L(op->is_small_read);
			tetris_sd_chnl_n[cl]++;
			tetris_sd_chnl[cl] += cont_ns;
			if (cont_ns > tetris_sd_chnl_max[cl])
				tetris_sd_chnl_max[cl] = cont_ns;
		}

		completed_time = chnl_etime;

		remaining -= xfer;
		chnl_stime = chnl_etime;
	}

	if (chnl_end_out)
		*chnl_end_out = chnl_etime;

	nvmev_die_end_out = nand_etime;
	return completed_time;
}

static uint64_t ssd_serve_write(struct ssd *ssd, const struct lunq3_op *op, uint64_t s)
{
	struct ppa ppa = op->ppa;
	struct ssd_channel *ch = get_ch(ssd, &ppa);
	uint64_t chnl_etime, nand_stime, nand_etime;

	chnl_etime = chmodel_request(ch->perf_model, s, op->xfer_size, (int)ppa.g.lun, NULL);

	nand_stime = chnl_etime;
	nand_etime = nand_stime + op->base_lat;

	return nand_etime;
}

static uint64_t ssd_serve_erase(struct ssd *ssd, const struct lunq3_op *op, uint64_t s)
{
	uint64_t nand_etime = s + op->base_lat;

	return nand_etime;
}

static bool lunq3_inbox_push(struct lunq3_inbox *ib, const struct lunq3_op *op);
static uint64_t lunq3_defer_admit(struct ssd *ssd, struct ppa *ppa, const struct lunq3_op *op,
				  uint64_t cmd_stime);

uint64_t ssd_advance_nand(struct ssd *ssd, struct nand_cmd *ncmd)
{
	int c = ncmd->cmd;
	uint64_t cmd_stime = (ncmd->stime == 0) ? __get_ioclock(ssd) : ncmd->stime;
	uint64_t nand_stime, nand_etime;

	uint64_t chnl_stime;
	uint64_t remaining, completed_time;
	struct ssdparams *spp;
	struct ppa *ppa = ncmd->ppa;
	uint32_t cell;
	NVMEV_DEBUG(
		"SSD: %p, Enter stime: %lld, ch %d lun %d blk %d page %d command %d ppa 0x%llx\n",
		ssd, ncmd->stime, ppa->g.ch, ppa->g.lun, ppa->g.blk, ppa->g.pg, c, ppa->ppa);

	if (ppa->ppa == UNMAPPED_PPA) {
		NVMEV_ERROR("Error ppa 0x%llx\n", ppa->ppa);
		return cmd_stime;
	}

	spp = &ssd->sp;
	cell = get_cell(ssd, ppa);
	remaining = ncmd->xfer_size;

	switch (c) {
	case NAND_READ: {
		uint64_t base_read_lat;
		struct lun_queue *q3 = NULL;

		base_read_lat = spp->pg_rd_lat[cell];

		if (ncmd->pslc || ppa_is_slc(ssd, ncmd->ppa))
			base_read_lat = (uint64_t)pslc_read_ns;

		q3 = get_lq3(ssd, ppa);
		nand_stime = READ_ONCE(q3->served_until);
		if (nand_stime < cmd_stime)
			nand_stime = cmd_stime;

		{
			struct lunq3_op sop = {
				.bypass_n = tt_budget_init(),
				.arr = cmd_stime,
				.ppa = *ppa,
				.xfer_size = (uint32_t)ncmd->xfer_size,
				.host_seq = ncmd->host_seq,
				.sq_id = ncmd->sq_id,

				.dur = base_read_lat + lunq_xfer_est(spp, nand_rd_ch_bytes(spp, remaining)),
				.kind = lunq3_kind_of(ncmd),
				.is_host_rd = (ncmd->type == USER_IO) ? 1 : 0,
				.rd_bytes = (uint32_t)ncmd->xfer_size,
				.host_bytes = ncmd->host_bytes,
				.unit_seq = ncmd->host_seq,
				.op_cmd = NAND_READ,
				.op_type = (uint8_t)ncmd->type,
				.pslc = ncmd->pslc ? 1 : 0,
				.interleave_dma = ncmd->interleave_pci_dma ? 1 : 0,
				.is_small_read = ncmd->is_small_read ? 1 : 0,
				.base_lat = base_read_lat,
				.stream_id = ncmd->stream_id,
				.seq_hint = ncmd->seq_hint,

				.cache_fill = ncmd->cache_fill,
			};

			if (sop.stream_id)
				tt_stream_op++;

#if TETRIS_STALL_DECOMP

			if (ncmd->type == USER_IO)
				lunq3_attr_resid_at(ssd, ppa, cmd_stime,
						    TETRIS_SD_L(sop.is_small_read),
						    sop.dur, ncmd->tn);
#endif

			if (ncmd->type == USER_IO)
				conv_note_line_read(ppa, cmd_stime);

			sop.owner = ncmd->owner;
			completed_time = lunq3_defer_admit(ssd, ppa, &sop, cmd_stime) +
					 sop.dur;
		}

		break;
	}

	case NAND_WRITE: {
		uint64_t wr_lat = spp->pg_wr_lat;

		if (ncmd->pslc || ppa_is_slc(ssd, ncmd->ppa)) {
			wr_lat = (uint64_t)pslc_prog_ns;
		}

		{
			struct lun_queue *q3w = get_lq3(ssd, ppa);

			chnl_stime = READ_ONCE(q3w->served_until);
			if (chnl_stime < cmd_stime)
				chnl_stime = cmd_stime;
		}

		{
			struct lunq3_op sop = {
				.bypass_n = tt_budget_init(),
				.arr = cmd_stime,
				.ppa = *ppa,
				.xfer_size = (uint32_t)ncmd->xfer_size,
				.host_seq = ncmd->host_seq,
				.sq_id = ncmd->sq_id,
				.dur = wr_lat + lunq_xfer_est(spp, ncmd->xfer_size),
			.kind = lunq3_kind_of(ncmd),
			.is_host_wr = (ncmd->type == USER_IO) ? 1 : 0,
			.op_cmd = NAND_WRITE,
				.op_type = (uint8_t)ncmd->type,
				.pslc = ncmd->pslc ? 1 : 0,
				.base_lat = wr_lat,

				.cache_frame = ncmd->cache_frame,
				.raw_ppa_enc = ncmd->raw_ppa_enc,
			};

			sop.owner = ncmd->owner;
			nand_etime = lunq3_defer_admit(ssd, ppa, &sop, cmd_stime) + sop.dur;
		}
		completed_time = nand_etime;
		break;
	}

	case NAND_ERASE: {
		uint64_t er_lat = spp->blk_er_lat;

		if (ncmd->pslc || ppa_is_slc(ssd, ppa)) {
			er_lat = (uint64_t)pslc_erase_ns;
		}

		{
			int em = (ncmd->pslc || ppa_is_slc(ssd, ppa)) ? 1 : 0;

			er_cnt[em]++;
			er_ns[em] += er_lat;
			if (ncmd->migrate) {
				er_migr_cnt[em]++;
				er_migr_ns[em] += er_lat;
			}
		}

		{
			struct lun_queue *q3e = get_lq3(ssd, ppa);

			nand_stime = READ_ONCE(q3e->served_until);
			if (nand_stime < cmd_stime)
				nand_stime = cmd_stime;
		}

		{
			struct lunq3_op sop = {
				.bypass_n = tt_budget_init(),
				.arr = cmd_stime,
				.ppa = *ppa,
				.dur = er_lat,
			.kind = lunq3_kind_of(ncmd),
			.op_cmd = NAND_ERASE,
				.op_type = (uint8_t)ncmd->type,
				.base_lat = er_lat,
			};

			sop.owner = ncmd->owner;
			nand_etime = lunq3_defer_admit(ssd, ppa, &sop, cmd_stime) + sop.dur;
		}
		completed_time = nand_etime;
		break;
	}

	default:

		NVMEV_ERROR("Unsupported NAND command: 0x%x (only READ/WRITE/ERASE may be issued)\n", c);
		NVMEV_ASSERT(0 && "ssd_advance_nand: op must be NAND_READ/WRITE/ERASE -- every op must enter the die queue and take its completion from queue_serve");
		return cmd_stime;
	}

	return completed_time;
}

static bool lunq3_inbox_push(struct lunq3_inbox *ib, const struct lunq3_op *op)
{
	uint32_t tail = ib->tail;
	uint32_t used = tail - READ_ONCE(ib->head);

	if (used >= LUNQ3_INBOX_SZ) {
		return false;
	}
	ib->slot[tail & LUNQ3_INBOX_MASK] = *op;
	smp_wmb();
	WRITE_ONCE(ib->tail, tail + 1);
	return true;
}

static void lunq3_inbox_drain(struct lunq3_inbox *ib, struct lun_queue *q)
{
	uint32_t head = ib->head;
	uint32_t tail = READ_ONCE(ib->tail);

	if (head == tail)
		return;
	smp_rmb();
	while (head != tail && q->n < lunq3_qmax) {
		lunq3_push(q, &ib->slot[head & LUNQ3_INBOX_MASK]);
		head++;
	}
	WRITE_ONCE(ib->head, head);
	if ((uint32_t)q->n > qs_pend_hwm) {
		qs_pend_hwm = q->n;

		{
			int i;

			qs_hwm_user = qs_hwm_gc = qs_hwm_fold = 0;
			lunq3_for_each(q, i) {
				switch (q->pend[i].op_type) {
				case GC_IO:           qs_hwm_gc++; break;
				case FOLD_IO:         qs_hwm_fold++; break;
				default:              qs_hwm_user++; break;
				}
			}
		}
	}
}

static bool lunq3_outbox_space(struct lunq3_outbox *ob)
{
	return (ob->tail - READ_ONCE(ob->head)) < LUNQ3_OUTBOX_SZ;
}

static void lunq3_outbox_push(struct lunq3_outbox *ob, uint32_t owner, uint32_t xfer,
			      uint64_t chnl_end, uint32_t frame, uint32_t fill, uint8_t fold_prog,
			      uint64_t raw_ppa_enc, uint32_t raw_token)
{
	uint32_t tail = ob->tail;

	ob->slot[tail & LUNQ3_OUTBOX_MASK].owner = owner;
	ob->slot[tail & LUNQ3_OUTBOX_MASK].xfer = xfer;
	ob->slot[tail & LUNQ3_OUTBOX_MASK].chnl_end = chnl_end;
	ob->slot[tail & LUNQ3_OUTBOX_MASK].frame = frame;
	ob->slot[tail & LUNQ3_OUTBOX_MASK].fill = fill;
	ob->slot[tail & LUNQ3_OUTBOX_MASK].fold_prog = fold_prog;
	ob->slot[tail & LUNQ3_OUTBOX_MASK].raw_ppa_enc = raw_ppa_enc;
	ob->slot[tail & LUNQ3_OUTBOX_MASK].raw_token = raw_token;
	smp_wmb();
	WRITE_ONCE(ob->tail, tail + 1);
	qs_pub++;
}

void lunq3_outbox_drain(struct ssd *ssd,
			void (*apply)(struct ssd *, uint32_t, uint32_t, uint64_t, uint32_t, uint32_t))
{
	struct lunq3_outbox *ob = ssd->outbox;
	uint32_t head, tail;

	if (!ob)
		return;
	head = ob->head;
	tail = READ_ONCE(ob->tail);
	smp_rmb();
	while (head != tail) {
		struct lunq3_done *d = &ob->slot[head & LUNQ3_OUTBOX_MASK];

		if (unlikely(d->fold_prog))
			pslc_fold_observe(d->chnl_end, true);

		apply(ssd, d->owner, d->xfer, d->chnl_end, d->frame, d->fill);
		head++;
		qs_drained_done++;
	}
	WRITE_ONCE(ob->head, head);
}

static uint64_t lunq3_defer_admit(struct ssd *ssd, struct ppa *ppa, const struct lunq3_op *op,
				  uint64_t cmd_stime)
{
	uint32_t ch = ppa->g.ch, lun = ppa->g.lun;
	struct lun_queue *q = &ssd->lq3[ch][lun];
	uint64_t su = READ_ONCE(q->served_until);

	if (!lunq3_inbox_push(&ssd->inbox[ch][lun], op)) {
		uint32_t spun = 0;

		switch (op->op_cmd) {
		case NAND_READ:  qs_inbox_wait_rd++; break;
		case NAND_WRITE: qs_inbox_wait_wr++; break;
		default:         qs_inbox_wait_er++; break;
		}

		while (spun < LUNQ3_INBOX_SPIN_MAX &&
		       !lunq3_inbox_push(&ssd->inbox[ch][lun], op)) {
			cpu_relax();
			spun++;
		}
		if (spun >= LUNQ3_INBOX_SPIN_MAX) {
			qs_inbox_dropped++;
			return su > cmd_stime ? su : cmd_stime;
		}
	}

	if (op->owner)
		nvmev_note_issued(op->owner);
	return su > cmd_stime ? su : cmd_stime;
}

typedef int (*lunq3_pick_fn)(struct lun_queue *q, uint64_t now);

u64 cr_reads, cr_pipe_n;

u64 cr_st_n[2][CR_G_N], cr_st_sum[2][CR_G_N], cr_st_sq[2][CR_G_N],
   cr_st_min[2][CR_G_N], cr_st_max[2][CR_G_N];

static inline int cr_bucket(u32 n)
{
	if (n <= 1) return 0;
	if (n <= 2) return 1;
	if (n <= 4) return 2;
	if (n <= 8) return 3;
	if (n <= 16) return 4;
	if (n <= 32) return 5;
	if (n <= 64) return 6;
	return 7;
}

static inline void tt_cr_run_close(struct lun_queue *q, u8 breaker, int cause)
{
	u8 victim;

	if (!q->cr_run_ng)
		return;
	tt_cr_run_hist[cr_bucket(q->cr_run_ng)]++;
	tt_cr_run_n++;
	tt_cr_run_sum += q->cr_run_ng;
	if (q->cr_run_ng > tt_cr_run_max)
		tt_cr_run_max = q->cr_run_ng;
	victim = q->cr_run_tn;
	if (victim < TETRIS_TN_MAX) {
		tt_cr_run_n_tn[victim]++;
		tt_cr_run_sum_tn[victim] += q->cr_run_ng;
		if (cause == 1)
			tt_cr_break_wr_tn[victim]++;
		else if (cause == 2)
			tt_cr_break_blk_tn[victim]++;
		else if (breaker < TETRIS_TN_MAX)
			tt_cr_break_gap_tn[victim][breaker]++;
	}
	q->cr_run_ng = 0;
	q->cr_run_tn = 0;
}

static inline int cr_note_serve(struct ssd *ssd, struct lun_queue *q,
				 const struct lunq3_op *o, uint64_t s)
{
	struct ssdparams *spp = &ssd->sp;
	uint32_t prev_ct;

	u8 op_tn;
	int is_rd, cont;
	int brk_blk;

	brk_blk = 0;
	op_tn = 0;
	op_tn = conv_qserve_tenant_of_ppa((struct ppa *)&o->ppa);
	is_rd = (o->op_cmd == NAND_READ);
	if (!is_rd) {
		tt_cr_run_close(q, op_tn, 1);
		q->cr_open = 0;
		return 0;
	}

	cont = 0;
	if (q->ct != q->ch) {
		prev_ct = (q->ct - 1) & (LUNQ3_CRING - 1);
		if (q->cr_open && q->cring[prev_ct].end == s)
			cont = 1;
	}

	tt_cr_rd_n++;
	if (op_tn < TETRIS_TN_MAX)
		tt_cr_rd_n_tn[op_tn]++;
	if (!cont)
		tt_cr_run_close(q, op_tn, brk_blk ? 2 : 0);
	if (cont) {
		q->cr_run_ng++;
		tt_cr_cont_n++;
		if (op_tn < TETRIS_TN_MAX)
			tt_cr_cont_n_tn[op_tn]++;

		if (o->stream_id && o->stream_id == q->cr_stream)
			tt_chain_same_stream++;
		else
			tt_chain_diff_stream++;
	}
	q->cr_open = 1;
	q->cr_stream = o->stream_id;
	if (!q->cr_run_tn)
		q->cr_run_tn = op_tn;
	if (o->stream_id)
		tt_stream_rd++;
	q->cr_xfer = (uint32_t)nand_rd_ch_bytes(spp, o->xfer_size);
	return cont;
}

static bool ssd_qserve_one_pick(struct ssd *ssd, struct lun_queue *q, uint64_t now, bool force,
				lunq3_pick_fn pick)
{
	int i;
	uint64_t s, done;
	int cr_cont = 0;

	if (q->served_until > now && !force) {
		return false;
	}

	i = pick(q, now);
	if (i < 0) {
		return false;
	}

	if (!lunq3_outbox_space(ssd->outbox)) {
		qs_outbox_full++;
		return false;
	}
	s = q->served_until > q->pend[i].arr ? q->served_until : q->pend[i].arr;
	if (s > now && !force) {
		if (q->n > 1) {
			uint16_t nx = q->nxt[i];

			if (nx != LUNQ3_NIL && q->pend[nx].arr <= now) {
				if (q->served_until > now)
					;
				else
					;
			}
		}
		return false;
	}
	if (s > now)
		qs_forced_ahead++;

	cr_cont = cr_note_serve(ssd, q, &q->pend[i], s);

	NVMEV_ASSERT(q->pend[i].op_cmd == NAND_READ || q->pend[i].op_cmd == NAND_WRITE ||
		     q->pend[i].op_cmd == NAND_ERASE);

	switch (q->pend[i].op_cmd) {
	case NAND_WRITE:
		done = ssd_serve_write(ssd, &q->pend[i], s);
		break;
	case NAND_ERASE:
		done = ssd_serve_erase(ssd, &q->pend[i], s);
		break;
	default:
		done = ssd_serve_read(ssd, &q->pend[i], s, NULL);
		break;
	}

	{
		uint64_t die_end = (q->pend[i].op_cmd == NAND_READ) ? nvmev_die_end_out : done;
		uint64_t sd = s + q->pend[i].dur;

		if (sd > die_end) {
			uint64_t d = sd - die_end;

			e8_over_n++; e8_over_sum += d;
			if (d > e8_over_max) e8_over_max = d;
		} else if (die_end > sd) {
			uint64_t d = die_end - sd;

			e8_under_n++; e8_under_sum += d;
			if (d > e8_under_max) e8_under_max = d;
		}
	}

	if (q->pend[i].op_cmd != NAND_READ) {
		q->served_until = done;
	} else if (cr_cont) {
		uint64_t xfer_ns = done > nvmev_die_end_out ? done - nvmev_die_end_out : 0;
		uint64_t arr_ns = nvmev_die_end_out > s ? nvmev_die_end_out - s : 0;

		q->served_until = s + (arr_ns > xfer_ns ? arr_ns : xfer_ns);
	} else {
		q->served_until = done;
	}

	ssd_optrace_record_serve(&q->pend[i], s, done, q->served_until);

	lunq3_mark(q, q->pend[i].kind, q->served_until,
		   q->served_until > s ? q->served_until - s : 0);

	if (s > q->pend[i].arr) {
		u64 wt = s - q->pend[i].arr;

		u64 *n_, *sum_, *max_;

		n_   = q->pend[i].tt_overtaken ? &tetris_stv_n   : &tetris_ctl_n;
		sum_ = q->pend[i].tt_overtaken ? &tetris_stv_sum_ns : &tetris_ctl_sum_ns;
		max_ = q->pend[i].tt_overtaken ? &tetris_stv_max_ns : &tetris_ctl_max_ns;
		(*n_)++;
		*sum_ += wt;
		if (wt > *max_)
			*max_ = wt;
	}

	lunq3_outbox_push(ssd->outbox, q->pend[i].owner,
			  q->pend[i].op_cmd == NAND_READ ? q->pend[i].xfer_size : 0, done,
			  q->pend[i].cache_frame, q->pend[i].cache_fill,
			  q->pend[i].op_type == FOLD_IO && q->pend[i].op_cmd == NAND_WRITE,
			  q->pend[i].raw_ppa_enc,
			  q->pend[i].raw_ppa_enc ? q->pend[i].host_seq : 0);
	lunq3_pop(q, i);
	return true;
}

#define TETRIS_WIN_MAX  1024
#define TETRIS_LUNS_MAX 8

#define TETRIS_EPOCH_NS 10000000ULL

static uint16_t tetris_rdq[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_rdq_n[TETRIS_LUNS_MAX];
static uint16_t tetris_wrq[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_wrq_n[TETRIS_LUNS_MAX];
static uint16_t tetris_grd[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_grd_n[TETRIS_LUNS_MAX];
static uint16_t tetris_gwr[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_gwr_n[TETRIS_LUNS_MAX];

static u64      tetris_rd_sd[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_wr_sd[TETRIS_LUNS_MAX][TETRIS_WIN_MAX];

static u64      tetris_rd_sd0[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_wr_sd0[TETRIS_LUNS_MAX][TETRIS_WIN_MAX];

static u64      tetris_rd_sdx[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_wr_sdx[TETRIS_LUNS_MAX][TETRIS_WIN_MAX];

static u64      tetris_rd_sds[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_wr_sds[TETRIS_LUNS_MAX][TETRIS_WIN_MAX];
static uint8_t  tetris_rd_lo[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_wr_lo[TETRIS_LUNS_MAX][TETRIS_WIN_MAX];
static uint16_t tetris_ord0[TETRIS_LUNS_MAX][TETRIS_WIN_MAX], tetris_ordn[TETRIS_LUNS_MAX];
static uint16_t tetris_neworder[TETRIS_WIN_MAX];

static u64 tetris_ep_start, tetris_ep_rd[TETRIS_TN_MAX], tetris_ep_wr[TETRIS_TN_MAX];
static u8  tetris_lo_rd[TETRIS_TN_MAX], tetris_lo_wr[TETRIS_TN_MAX];

#define TETRIS_PRIO_MAX 8
static u32 tetris_wrr_credit[TETRIS_LUNS_MAX][2];
static u8  tetris_wrr_level[TETRIS_LUNS_MAX][2];

u64 tetris_scan_n, tetris_scan_rd, tetris_scan_wr, tetris_scan_gc;
u64 tetris_epoch_n, tetris_cls_lo_rd, tetris_cls_hi_rd, tetris_cls_lo_wr, tetris_cls_hi_wr;
u64 tetris_tn_seen[TETRIS_TN_MAX];
u64 tetris_sd_n, tetris_sd_sum_ppm, tetris_sd_max_ppm;

static uint16_t tetris_newpos[LUNQ3_QMAX_MAX];
u64 tetris_F_n, tetris_F_sum_ppm, tetris_F_min_ppm;
u64 tetris_win_full, tetris_win_clamp, tetris_ch_scan, tetris_pass_n;

u64 tetris_prio_seen[8], tetris_prio_levels;
u64 tetris_move_n, tetris_hops_sum, tetris_step1_n, tetris_step2_n;
u64 tetris_stv_n, tetris_stv_sum_ns, tetris_stv_max_ns;
u64 tetris_ctl_n, tetris_ctl_sum_ns, tetris_ctl_max_ns;

u64 tetris_pw_rd_win, tetris_pw_wr_win, tetris_gc_pair;
u64 tetris_wrr_n;

static struct lun_queue *tetris_chmemo_q[TETRIS_LUNS_MAX];
static uint64_t tetris_chmemo_now[TETRIS_LUNS_MAX];
static uint32_t tetris_chmemo_ver[TETRIS_LUNS_MAX];
static int      tetris_chmemo_slot[TETRIS_LUNS_MAX][2][TETRIS_PRIO_MAX];
static u64      tetris_chmemo_ah[TETRIS_LUNS_MAX][2][TETRIS_PRIO_MAX];
static int      tetris_chmemo_nlv[TETRIS_LUNS_MAX][2];

int tetris_free_lines = INT_MAX;

#define TT_DEP_MAX_NS 1000000000ULL
u64 tt_F_min_ppm;

static inline int tetris_prio_of(u8 tn)
{
	switch (tn) {
	case 1:  return tetris_prio_tn1;
	case 2:  return tetris_prio_tn2;
	case 3:  return tetris_prio_tn3;
	default: return 0;
	}
}

void tetris_reset(void)
{
	int i;

	tetris_scan_n = tetris_scan_rd = tetris_scan_wr = tetris_scan_gc = 0;
	tetris_epoch_n = tetris_cls_lo_rd = tetris_cls_hi_rd = tetris_cls_lo_wr = tetris_cls_hi_wr = 0;
	tetris_sd_n = tetris_sd_sum_ppm = tetris_sd_max_ppm = 0;
	tetris_F_n = tetris_F_sum_ppm = 0;
	tetris_F_min_ppm = ~0ULL;
	tetris_win_full = tetris_win_clamp = tetris_ch_scan = tetris_pass_n = 0;
	tetris_move_n = tetris_hops_sum = tetris_step1_n = tetris_step2_n = 0;
	tetris_stv_n = tetris_stv_sum_ns = tetris_stv_max_ns = 0;
	tetris_ctl_n = tetris_ctl_sum_ns = tetris_ctl_max_ns = 0;
	tetris_pw_rd_win = tetris_pw_wr_win = tetris_gc_pair = tetris_wrr_n = 0;

	memset(tetris_chmemo_q, 0, sizeof(tetris_chmemo_q));
	memset(tetris_wrr_credit, 0, sizeof(tetris_wrr_credit));
	memset(tetris_wrr_level, 0, sizeof(tetris_wrr_level));
	memset(tetris_alone_done, 0, sizeof(tetris_alone_done));
	for (i = 0; i < TETRIS_TN_MAX; i++)
		tetris_tn_seen[i] = 0;
	for (i = 0; i < 8; i++)
		tetris_prio_seen[i] = 0;
	tetris_prio_levels = 0;
	tt_F_min_ppm = ~0ULL;
}

static void tetris_epoch_tick(u64 now)
{
	u64 tot_rd = 0, tot_wr = 0;
	int t;

	if (!tetris_ep_start) {
		tetris_ep_start = now;
		return;
	}
	if (now - tetris_ep_start < TETRIS_EPOCH_NS)
		return;
	for (t = 0; t < TETRIS_TN_MAX; t++) {
		tot_rd += tetris_ep_rd[t];
		tot_wr += tetris_ep_wr[t];
	}
	for (t = 0; t < TETRIS_TN_MAX; t++) {
		tetris_lo_rd[t] = (!tot_rd || 100 * tetris_ep_rd[t] < (u64)tetris_alpha_pct * tot_rd);
		tetris_lo_wr[t] = (!tot_wr || 100 * tetris_ep_wr[t] < (u64)tetris_alpha_pct * tot_wr);
		if (tetris_lo_rd[t]) tetris_cls_lo_rd++; else tetris_cls_hi_rd++;
		if (tetris_lo_wr[t]) tetris_cls_lo_wr++; else tetris_cls_hi_wr++;
		tetris_ep_rd[t] = tetris_ep_wr[t] = 0;
	}
	tetris_ep_start = now;
	tetris_epoch_n++;
}

static inline u64 lunq3_twait_tetris(const struct lun_queue *q, const struct lunq3_op *o,
				   u64 now, u64 ahead, u64 *el, u64 *cb)
{
	u64 elapsed = now > o->arr ? now - o->arr : 0;
	u64 chip    = q->served_until > now ? q->served_until - now : 0;

	if (el)
		*el = elapsed;
	if (cb)
		*cb = chip;
	return elapsed + chip + ahead;
}

static inline void tt_twait_note(const struct lunq3_op *o, u64 tw, u64 el, u64 cb, u64 ahead)
{
	if (o->op_type != USER_IO) {
		tt_tw_gc_n++;
		tt_tw_gc_sum += tw;
		if (tw > tt_tw_gc_max)
			tt_tw_gc_max = tw;
		return;
	}
	{
		int w = (o->op_cmd == NAND_READ) ? 0 : 1;

		tt_tw_el[w] += el;
		tt_tw_cb[w] += cb;
		tt_tw_ah[w] += ahead;
	}
	if (o->op_cmd == NAND_READ) {
		tt_tw_rd_n++;
		tt_tw_rd_sum += tw;
		if (tw > tt_tw_rd_max)
			tt_tw_rd_max = tw;
	} else {
		tt_tw_wr_n++;
		tt_tw_wr_sum += tw;
		if (tw > tt_tw_wr_max)
			tt_tw_wr_max = tw;
	}
}

enum { TWC_CHAIN = 0, TWC_NOTRD, TWC_NOOPEN, TWC_NOOPEN_BLK, TWC_GAP, TWC_BLK };
enum { TWC_CLOSE_SEED = 0, TWC_CLOSE_HOST_WRITE, TWC_CLOSE_GC_WRITE, TWC_CLOSE_ERASE };

struct twait_chain {
	u64 t;
	u32 blk;
	u8  open;
	u8  close_src;
};

static inline u64 tetris_ahead_serial(const struct lunq3_op *o)
{
	u64 d;

	d = o->dur ? o->dur : o->base_lat;
	return d ? d : 1;
}

static void tetris_scan_tail(u64 s_min, u64 s_max, u64 d_min, u64 d_max, bool dep_in_chan)
{
	{
		int p, cnt = 0;

		for (p = 0; p < 8; p++)
			if (tetris_prio_seen[p])
				cnt++;
		tetris_prio_levels = cnt;
	}
	tetris_ch_scan++;

	if (s_max) {
		u64 F = (1000000ULL * s_min) / s_max;

		tetris_F_n++;
		tetris_F_sum_ppm += F;
		if (F < tetris_F_min_ppm)
			tetris_F_min_ppm = F;
	}

	if (d_max) {
		u64 F = (1000000ULL * d_min) / d_max;

		if (F < tt_F_min_ppm)
			tt_F_min_ppm = F;
	}
}

static void tetris_scan_channel_x(struct ssd *ssd, uint32_t ch, uint64_t now, bool measure_only)
{
	uint32_t lun, luns = ssd->sp.luns_per_ch;

	int win = tetris_win;
	u64 s_min = ~0ULL, s_max = 0;
	u64 d_min = ~0ULL, d_max = 0;
	bool dep_in_chan = false;

	if (win < 1)
		win = 1;
	if (win > TETRIS_WIN_MAX) {
		win = TETRIS_WIN_MAX;
		tetris_win_clamp++;
	}
	if (luns > TETRIS_LUNS_MAX)
		luns = TETRIS_LUNS_MAX;

	for (lun = 0; lun < luns; lun++) {
		struct lun_queue *q = &ssd->lq3[ch][lun];
		u64 ahead = 0;
		u64 ahead_x = 0;
		u64 ahead_s = 0;

		int d, slot = lunq3_fifo_head(q);

		int mix_first = 0;

		tetris_rdq_n[lun] = tetris_wrq_n[lun] = tetris_grd_n[lun] = tetris_gwr_n[lun] = 0;
		tetris_ordn[lun] = 0;

		for (d = 0; d < win && slot >= 0; d++, slot = lunq3_fifo_next(q, slot)) {
			struct lunq3_op *o = &q->pend[slot];
			bool is_rd = (o->op_cmd == NAND_READ);
			bool is_gc = (o->op_type != USER_IO);
			u8 tn = tetris_flow_of(o);
			u64 svc, w_shared, w_alone, sd, sd0, t_dep = 0;
			u64 tw_el = 0, tw_cb = 0;
			u64 svc_x = 0, sdx = 0;
			u64 dhat, sds = 0;

			if (tn >= TETRIS_TN_MAX)
				tn = 0;

			if (is_gc) {
				if (is_rd) tetris_grd[lun][tetris_grd_n[lun]++] = (uint16_t)slot;
				else       tetris_gwr[lun][tetris_gwr_n[lun]++] = (uint16_t)slot;
				tetris_scan_gc++;
			} else if (is_rd) {
				tetris_rd_lo[lun][tetris_rdq_n[lun]] = tetris_lo_rd[tn];
				tetris_rdq[lun][tetris_rdq_n[lun]++] = (uint16_t)slot;
				tetris_scan_rd++;

				tetris_ep_rd[tn]++;
			} else {
				tetris_wr_lo[lun][tetris_wrq_n[lun]] = tetris_lo_wr[tn];
				tetris_wrq[lun][tetris_wrq_n[lun]++] = (uint16_t)slot;
				tetris_scan_wr++;
				tetris_ep_wr[tn]++;
			}
			tetris_ord0[lun][tetris_ordn[lun]++] = (uint16_t)slot;
			tetris_scan_n++;
			tetris_tn_seen[tn]++;

			if (!is_gc && o->sq_id) {
				u32 sq = o->sq_id;
				u8  ten = conv_qserve_tenant_of_ppa(&o->ppa);

				if (sq >= TETRIS_SQ_MAX) {
					sq = TETRIS_SQ_MAX - 1;
				}
				tetris_sq_hist[sq][ten <= TENANT_N ? ten : 0]++;
			}
			{
				int pl = tetris_prio_of(tn);

				if (pl >= 0 && pl < 8)
					tetris_prio_seen[pl]++;
			}

			svc = o->dur ? o->dur : o->base_lat;
			if (!svc)
				svc = 1;
			svc_x = o->base_lat ? o->base_lat : 1;
			tt_tw_ah_x[is_rd ? 0 : 1] += ahead_x;

			dhat = tetris_ahead_serial(o);

			w_shared = lunq3_twait_tetris(q, o, now, ahead, &tw_el, &tw_cb);
			tt_twait_note(o, w_shared, tw_el, tw_cb, ahead);

			w_alone = o->tetris_alone_ns;

			if (o->op_cmd == NAND_WRITE) {
				if (o->base_lat == (uint64_t)pslc_prog_ns) {
					tt_tm_wr_slc++; tt_tm_wr_slc_ns += svc;
					if (!mix_first) mix_first = 1;
				} else {
					tt_tm_wr_tlc++; tt_tm_wr_tlc_ns += svc;
					if (!mix_first) mix_first = 2;
				}
			} else if (o->op_cmd == NAND_READ) {
				if (o->base_lat == (uint64_t)pslc_read_ns) {
					tt_tm_rd_slc++; tt_tm_rd_slc_ns += svc;
				} else {
					tt_tm_rd_tlc++; tt_tm_rd_tlc_ns += svc;
				}
			}

			if (o->op_cmd == NAND_WRITE && o->cache_frame) {
				tt_dep_scan_wr++;
				if (READ_ONCE(diq_live)) {
					struct diq_dep_adv *ad =
						&diq_dep_adv[o->cache_frame & 0xFFFFu];
					u32 f = READ_ONCE(ad->frame);
					u32 n = READ_ONCE(ad->n);

					if (f == o->cache_frame && n) {
						u64 t0 = READ_ONCE(ad->t0);
						u64 age = now > t0 ? now - t0 : 0;

						tt_dep_hit++;
						tt_dep_readers_sum += n;
						tt_dep_age_sum += age;
						if (age > tt_dep_age_max)
							tt_dep_age_max = age;

						t_dep = age;
						dep_in_chan = true;

						if (t_dep > TT_DEP_MAX_NS) {
							t_dep = TT_DEP_MAX_NS;
						}
					} else if (f) {
						tt_dep_stale++;
					}
				}
			}

			sd0 = (1000000ULL * (svc + w_shared)) / (svc + w_alone);
			{
				u64 wsx = lunq3_twait_tetris(q, o, now, ahead_x, NULL, NULL);

				sdx = (1000000ULL * (svc_x + wsx)) / (svc_x + w_alone);
			}
			{
				u64 wss = lunq3_twait_tetris(q, o, now, ahead_s, NULL, NULL);

				sds = (1000000ULL * (svc + wss)) / (svc + w_alone);
			}
			if (t_dep) {
				sd = (1000000ULL * (svc + w_shared + t_dep)) / (svc + w_alone);
			} else {
				sd = sd0;
			}

			if (o->tt_over_sd && o->bypass_n > 0) {
				o->bypass_n = 0;
			}
			tetris_sd_n++;
			tetris_sd_sum_ppm += sd0;
			if (sd0 > tetris_sd_max_ppm)
				tetris_sd_max_ppm = sd0;
			if (sd0 < s_min) s_min = sd0;
			if (sd0 > s_max) s_max = sd0;
			if (sd < d_min) d_min = sd;
			if (sd > d_max) d_max = sd;
			if (!is_gc) {
				if (is_rd) {
					tetris_rd_sd[lun][tetris_rdq_n[lun] - 1]  = sd;
					tetris_rd_sd0[lun][tetris_rdq_n[lun] - 1] = sd0;
					tetris_rd_sdx[lun][tetris_rdq_n[lun] - 1] = sdx;
					tetris_rd_sds[lun][tetris_rdq_n[lun] - 1] = sds;
				} else {
					tetris_wr_sd[lun][tetris_wrq_n[lun] - 1]  = sd;
					tetris_wr_sd0[lun][tetris_wrq_n[lun] - 1] = sd0;
					tetris_wr_sdx[lun][tetris_wrq_n[lun] - 1] = sdx;
					tetris_wr_sds[lun][tetris_wrq_n[lun] - 1] = sds;
				}
			}

			ahead += dhat;
			ahead_s += svc;
			ahead_x += svc_x;
		}
		if (d == win && slot >= 0)
			tetris_win_full++;
	}

	if (measure_only) {
		tetris_scan_tail(s_min, s_max, d_min, d_max, dep_in_chan);
		return;
	}
	for (lun = 0; lun < luns; lun++) {
		int cq, changed = 0;

		for (cq = 0; cq < 2; cq++) {
			uint16_t *Q  = cq ? tetris_wrq[lun]  : tetris_rdq[lun];
			u64      *SD = cq ? tetris_wr_sd[lun]: tetris_rd_sd[lun];
			u64     *SD0 = cq ? tetris_wr_sd0[lun]: tetris_rd_sd0[lun];
			u64     *SDX = cq ? tetris_wr_sdx[lun]: tetris_rd_sdx[lun];
			u64     *SDS = cq ? tetris_wr_sds[lun]: tetris_rd_sds[lun];
			uint8_t  *LO = cq ? tetris_wr_lo[lun]: tetris_rd_lo[lun];
			int n2 = cq ? tetris_wrq_n[lun] : tetris_rdq_n[lun];
			int a, b, b0, bx, bs, w, lo_end = 0;
			u64 mn = ~0ULL, mx = 0, mn0 = ~0ULL, mx0 = 0;
			u64 mnx = ~0ULL, mxx = 0;
			u64 mns = ~0ULL, mxs = 0;
			bool trig, trigx, trigs;

			if (n2 < 2)
				continue;

			for (a = 0; a < n2; a++) {
				if (!LO[a])
					continue;
				if (a != lo_end) {
					uint16_t ts = Q[a];
					u64 tsd = SD[a], tsd0 = SD0[a], tsdx = SDX[a], tsds = SDS[a];
					uint8_t tlo = LO[a];

					for (w = a; w > lo_end; w--) {
						Q[w] = Q[w - 1]; SD[w] = SD[w - 1];
						SD0[w] = SD0[w - 1]; SDX[w] = SDX[w - 1];
						SDS[w] = SDS[w - 1]; LO[w] = LO[w - 1];
					}
					Q[lo_end] = ts; SD[lo_end] = tsd;
					SD0[lo_end] = tsd0; SDX[lo_end] = tsdx;
					SDS[lo_end] = tsds; LO[lo_end] = tlo;
					tetris_step1_n++;
					tetris_hops_sum += a - lo_end;
					changed = 1;
				}
				lo_end++;
			}

			for (a = 0; a < n2; a++) {
				if (SD[a] < mn) mn = SD[a];
				if (SD[a] > mx) mx = SD[a];
				if (SD0[a] < mn0) mn0 = SD0[a];
				if (SD0[a] > mx0) mx0 = SD0[a];
				if (SDX[a] < mnx) mnx = SDX[a];
				if (SDX[a] > mxx) mxx = SDX[a];
				if (SDS[a] < mns) mns = SDS[a];
				if (SDS[a] > mxs) mxs = SDS[a];
			}
			trig  = mx  && (1000000ULL * mn)  / mx  < (u64)tetris_fthr_pct * 10000ULL;
			trigx = mxx && (1000000ULL * mnx) / mxx < (u64)tetris_fthr_pct * 10000ULL;
			trigs = mxs && (1000000ULL * mns) / mxs < (u64)tetris_fthr_pct * 10000ULL;

			tw_ahead_step2++;
			if (trigs != trigx)
				tw_ahead_trig++;

			tw_dhat_step2++;
			if (trig != trigs)
				tw_dhat_trig++;
			if (!trig)
				continue;
			b = b0 = bx = bs = 0;
			for (a = 1; a < n2; a++) {
				if (SD[a] > SD[b])
					b = a;
				if (SD0[a] > SD0[b0])
					b0 = a;
				if (SDX[a] > SDX[bx])
					bx = a;
				if (SDS[a] > SDS[bs])
					bs = a;
			}

			tw_ahead_pick++;
			if (bs != bx)
				tw_ahead_flip++;

			tw_dhat_pick++;
			if (b != bs)
				tw_dhat_flip++;

			{
				int lo_b = (b < lo_end) ? 0 : lo_end;

				if (b > lo_b) {
					uint16_t ts = Q[b];
					u64 tsd = SD[b], tsd0 = SD0[b], tsdx = SDX[b], tsds = SDS[b];
					uint8_t tlo = LO[b];

					for (w = b; w > lo_b; w--) {
						Q[w] = Q[w - 1]; SD[w] = SD[w - 1];
						SD0[w] = SD0[w - 1]; SDX[w] = SDX[w - 1];
						SDS[w] = SDS[w - 1]; LO[w] = LO[w - 1];
					}
					Q[lo_b] = ts; SD[lo_b] = tsd;
					SD0[lo_b] = tsd0; SDX[lo_b] = tsdx;
					SDS[lo_b] = tsds; LO[lo_b] = tlo;
					tetris_step2_n++;
					tetris_hops_sum += b - lo_b;
					changed = 1;
				}
			}
		}
		if (!changed)
			continue;

		{
			struct lun_queue *q = &ssd->lq3[ch][lun];
			int k, ri = 0, wi = 0, prev = -1;

			uint16_t *neworder = tetris_neworder;
			int need_rd = 0, need_wr = 0;

			for (k = 0; k < tetris_ordn[lun]; k++) {
				struct lunq3_op *po = &q->pend[tetris_ord0[lun][k]];

				if (po->op_type != USER_IO)
					continue;
				if (po->op_cmd == NAND_READ)
					need_rd++;
				else
					need_wr++;
			}
			if (need_rd != tetris_rdq_n[lun] || need_wr != tetris_wrq_n[lun]) {
				continue;
			}

			for (k = 0; k < tetris_ordn[lun]; k++) {
				struct lunq3_op *o = &q->pend[tetris_ord0[lun][k]];

				if (o->op_type != USER_IO)
					neworder[k] = tetris_ord0[lun][k];
				else if (o->op_cmd == NAND_READ)
					neworder[k] = tetris_rdq[lun][ri++];
				else
					neworder[k] = tetris_wrq[lun][wi++];
			}
			for (k = 0; k < tetris_ordn[lun]; k++)
				lunq3_fifo_del(q, neworder[k]);
			for (k = 0; k < tetris_ordn[lun]; k++) {
				lunq3_fifo_insert_after(q, prev, neworder[k]);
				if (neworder[k] != tetris_ord0[lun][k])
				{
					q->pend[neworder[k]].tt_overtaken = 1;

					if (q->pend[neworder[k]].bypass_n > 0 &&
					    q->pend[neworder[k]].bypass_n != INT_MAX) {
						q->pend[neworder[k]].bypass_n--;
					}
				}
				prev = neworder[k];
			}

			for (k = 0; k < tetris_ordn[lun]; k++)
				tetris_newpos[neworder[k]] = (uint16_t)k;
			for (k = 0; k < tetris_ordn[lun]; k++) {
				uint16_t slot0 = tetris_ord0[lun][k];

				if (tetris_newpos[slot0] > k) {
					if (q->pend[slot0].tt_defer < 255)
						q->pend[slot0].tt_defer++;
				}
			}

			q->fq_ver++;
			tetris_move_n++;
		}
	}

	tetris_scan_tail(s_min, s_max, d_min, d_max, dep_in_chan);
}

static int tetris_pick_slot = -1;

static int lunq3_next_tetris(struct lun_queue *q, uint64_t now)
{
	int i = tetris_pick_slot;

	(void)now;
	tetris_pick_slot = -1;
	return i;
}

static void tetris_chmemo_fill(struct lun_queue *q, int lun_i, int win, uint64_t now)
{
	int d, slot = lunq3_fifo_head(q);
	u64 acc = 0;
	int ci, pl;

	for (ci = 0; ci < 2; ci++) {
		tetris_chmemo_nlv[lun_i][ci] = 0;
		for (pl = 0; pl < TETRIS_PRIO_MAX; pl++) {
			tetris_chmemo_slot[lun_i][ci][pl] = -1;
			tetris_chmemo_ah[lun_i][ci][pl] = 0;
		}
	}
	for (d = 0; d < win && slot >= 0; d++, slot = lunq3_fifo_next(q, slot)) {
		struct lunq3_op *o = &q->pend[slot];
		int c = (o->op_type != USER_IO) ? TT_CLS_BG
					        : (o->op_cmd == NAND_READ ? TT_CLS_RD : TT_CLS_WR);

		acc += tetris_ahead_serial(o);
		if (c == TT_CLS_BG || o->arr > now)
			continue;
		ci = (c == TT_CLS_RD) ? 0 : 1;
		pl = tetris_prio_of(tetris_flow_of(o));
		if (pl < 0 || pl >= TETRIS_PRIO_MAX)
			pl = 0;
		if (tetris_chmemo_slot[lun_i][ci][pl] < 0) {
			tetris_chmemo_slot[lun_i][ci][pl] = slot;
			tetris_chmemo_ah[lun_i][ci][pl] = acc;
			tetris_chmemo_nlv[lun_i][ci]++;
		}
	}
	tetris_chmemo_q[lun_i] = q;
	tetris_chmemo_now[lun_i] = now;
	tetris_chmemo_ver[lun_i] = q->fq_ver;
}

static inline void chmemo_fill(struct lun_queue *q, int lun_i, int win, uint64_t now)
{
	tetris_chmemo_fill(q, lun_i, win, now);
}

static int tetris_class_head(struct lun_queue *q, int lun_i, int cls, int win, uint64_t now,
			   u64 *ahead_out)
{
	const int *cand;
	const u64 *cand_ah;
	int nlv, rw, lv, i, ci;

	if (ahead_out)
		*ahead_out = 0;

	if (lun_i < 0 || lun_i >= TETRIS_LUNS_MAX)
		lun_i = 0;

	if (cls != TT_CLS_RD && cls != TT_CLS_WR) {
		return -1;
	}
	if (tetris_chmemo_q[lun_i] != q || tetris_chmemo_now[lun_i] != now || tetris_chmemo_ver[lun_i] != q->fq_ver) {
		chmemo_fill(q, lun_i, win, now);
	}
	ci = (cls == TT_CLS_RD) ? 0 : 1;
	cand = tetris_chmemo_slot[lun_i][ci];
	cand_ah = tetris_chmemo_ah[lun_i][ci];
	nlv = tetris_chmemo_nlv[lun_i][ci];

	if (!nlv)
		return -1;
	if (nlv == 1) {
		for (i = 0; i < TETRIS_PRIO_MAX; i++)
			if (cand[i] >= 0) {
				if (ahead_out)
					*ahead_out = cand_ah[i];
				return cand[i];
			}
		return -1;
	}

	rw = ci;
	lv = tetris_wrr_level[lun_i][rw];
	for (i = 0; i < TETRIS_PRIO_MAX; i++) {
		int p = (lv + i) % TETRIS_PRIO_MAX;

		if (cand[p] < 0)
			continue;
		if (!tetris_wrr_credit[lun_i][rw]) {
			tetris_wrr_credit[lun_i][rw] = 1u << p;
			tetris_wrr_level[lun_i][rw] = (u8)p;
		}
		if (tetris_wrr_level[lun_i][rw] != p)
			continue;
		tetris_wrr_credit[lun_i][rw]--;
		if (!tetris_wrr_credit[lun_i][rw])
			tetris_wrr_level[lun_i][rw] = (u8)((p + 1) % TETRIS_PRIO_MAX);
		tetris_wrr_n++;
		if (ahead_out)
			*ahead_out = cand_ah[p];
		return cand[p];
	}
	for (i = 0; i < TETRIS_PRIO_MAX; i++)
		if (cand[TETRIS_PRIO_MAX - 1 - i] >= 0) {
			if (ahead_out)
				*ahead_out = cand_ah[TETRIS_PRIO_MAX - 1 - i];
			return cand[TETRIS_PRIO_MAX - 1 - i];
		}
	return -1;
}

static int tetris_gc_head(struct lun_queue *q, bool want_read, uint64_t now)
{
	struct tetris_queue *fq = &q->fq[want_read ? TETRIS_GC_RDQ : TETRIS_GC_WRQ];
	uint16_t it = fq->head;
	int guard = 0;

	while (it != LUNQ3_NIL) {
		if (q->pend[it].arr <= now) {
			return (int)it;
		}
		if (++guard > LUNQ3_QMAX_MAX) {
			break;
		}
		it = q->fq_next[it];
	}
	return -1;
}

static bool tetris_erase_blocked(struct lun_queue *q, int slot)
{
	const struct lunq3_op *x;
	uint16_t it;
	int guard;

	x = &q->pend[slot];
	if (x->op_cmd != NAND_ERASE)
		return false;
	it = q->fq[TETRIS_GC_RDQ].head;
	guard = 0;
	while (it != LUNQ3_NIL) {
		if (q->pend[it].ppa.g.blk == x->ppa.g.blk && q->pend[it].ppa.g.pl == x->ppa.g.pl)
			return true;
		if (++guard > LUNQ3_QMAX_MAX)
			break;
		it = q->fq_next[it];
	}
	return false;
}

static int tetris_gc_head_wr_rwp(struct lun_queue *q, uint64_t now)
{
	uint16_t it;
	int guard;

	it = q->fq[TETRIS_GC_WRQ].head;
	guard = 0;
	while (it != LUNQ3_NIL) {
		if (q->pend[it].arr <= now) {
			if (!tetris_erase_blocked(q, (int)it))
				return (int)it;
		}
		if (++guard > LUNQ3_QMAX_MAX) {
			break;
		}
		it = q->fq_next[it];
	}
	return -1;
}

static int tetris_gc_pick_rwp(struct lun_queue *q, uint64_t now)
{
	int gr, gw;

	gr = tetris_gc_head(q, true, now);
	if (gr >= 0 && !q->gc_last_rd)
		return gr;
	gw = tetris_gc_head_wr_rwp(q, now);
	if (gw >= 0)
		return gw;
	return gr;
}

static inline void tetris_note_idle_gc(struct lun_queue *q, int cls, bool early)
{
	if (q->gc_last_rd) {
		if (cls)
			;
		else
			;
	} else {
		if (cls)
			;
		else
			;
	}
	q->gc_last_rd = (uint8_t)cls;
}

static inline u64 tetris_pw(struct lun_queue *q, int slot, uint64_t now, u64 ahead, u64 extra)
{
	struct lunq3_op *o = &q->pend[slot];
	u64 el = now > o->arr ? now - o->arr : 0;
	u64 cb = q->served_until > now ? q->served_until - now : 0;
	u64 svc = (o->dur ? o->dur : (o->base_lat ? o->base_lat : 1)) + extra;

	return (1000000ULL * (el + cb + ahead)) / svc;
}

u64 tt_wb_open;
u64 tt_wb_continue;
u64 tt_wb_abort_rd;
u64 tt_wb_abort_bp;

static inline bool tt_wbatch_on(void)
{
	return tetris_wbatch > 1;
}

static inline bool tt_chain_on(void)
{
	return tetris_chain != 0;
}

static inline u64 tt_chain_delta(struct lun_queue *q, int rs)
{
	const struct lunq3_op *r;
	u64 tR, tX;

	if (rs < 0 || !q->cr_open)
		return 0;
	r = &q->pend[rs];
	if (r->op_cmd != NAND_READ)
		return 0;
	if (r->arr > q->served_until)
		return 0;
	tR = r->base_lat;
	tX = r->dur > r->base_lat ? r->dur - r->base_lat : 0;
	return tR < tX ? tR : tX;
}

static inline u64 tt_chain_charge(struct lun_queue *q, int rs)
{
	u64 d;

	if (!tt_chain_on())
		return 0;
	d = tt_chain_delta(q, rs);
	if (!d)
		return 0;
	if (READ_ONCE(backpressure_free_line_stall)) {
		tw_chain_yield_off++;
		return 0;
	}
	if (tetris_chain_max > 0 && q->chain_debt >= (uint16_t)tetris_chain_max) {
		tw_chain_starve_off++;
		return 0;
	}
	return d;
}

u64 tt_tm_wr_slc, tt_tm_wr_tlc, tt_tm_rd_slc, tt_tm_rd_tlc;
u64 tt_tm_wr_slc_ns, tt_tm_wr_tlc_ns, tt_tm_rd_slc_ns, tt_tm_rd_tlc_ns;

u64 tt_tw_rd_n, tt_tw_rd_sum, tt_tw_rd_max;
u64 tt_tw_wr_n, tt_tw_wr_sum, tt_tw_wr_max;
u64 tt_tw_gc_n, tt_tw_gc_sum, tt_tw_gc_max;

u64 tt_tw_el[2], tt_tw_cb[2], tt_tw_ah[2];

u64 tt_tw_ah_x[2];
u64 tw_ahead_pick, tw_ahead_flip, tw_ahead_step2, tw_ahead_trig;

u64 tt_tw_ah_s[2];
u64 tw_dhat_scan, tw_dhat_chain, tw_dhat_noopen, tw_dhat_noopen_blk, tw_dhat_gap, tw_dhat_blk;
u64 tw_dhat_save_sum, tw_dhat_pick, tw_dhat_flip, tw_dhat_step2, tw_dhat_trig;

u64 tw_chain_n, tw_chain_sum, tw_chain_flip;

u64 tetris_sq_hist[TETRIS_SQ_MAX][TENANT_N + 1];

u64 tw_chain_delayed[SK_N], tw_chain_yield_off, tw_chain_starve_off;

u64 tt_dep_scan_wr, tt_dep_hit, tt_dep_readers_sum;
u64 tt_dep_age_sum, tt_dep_age_max, tt_dep_stale;

void cr_reset(void)
{
	cr_reads = cr_pipe_n = 0;
	memset(cr_st_n, 0, sizeof(cr_st_n));
	memset(cr_st_sum, 0, sizeof(cr_st_sum));
	memset(cr_st_sq, 0, sizeof(cr_st_sq));
	memset(cr_st_max, 0, sizeof(cr_st_max));
	memset(cr_st_min, 0, sizeof(cr_st_min));
}

void tt_reset(void)
{
	tt_tm_wr_slc = tt_tm_wr_tlc = tt_tm_rd_slc = tt_tm_rd_tlc = 0;
	tt_tm_wr_slc_ns = tt_tm_wr_tlc_ns = tt_tm_rd_slc_ns = tt_tm_rd_tlc_ns = 0;
	tt_tw_rd_n = tt_tw_rd_sum = tt_tw_rd_max = 0;
	tt_tw_wr_n = tt_tw_wr_sum = tt_tw_wr_max = 0;
	tt_tw_gc_n = tt_tw_gc_sum = tt_tw_gc_max = 0;
	tt_tw_el[0] = tt_tw_el[1] = tt_tw_cb[0] = tt_tw_cb[1] = 0;
	tt_tw_ah[0] = tt_tw_ah[1] = 0;
	tt_tw_ah_x[0] = tt_tw_ah_x[1] = 0;
	tw_ahead_pick = tw_ahead_flip = tw_ahead_step2 = tw_ahead_trig = 0;
	tt_tw_ah_s[0] = tt_tw_ah_s[1] = 0;
	tw_dhat_scan = tw_dhat_chain = tw_dhat_noopen = tw_dhat_gap = tw_dhat_blk = 0;
	tw_dhat_noopen_blk = 0;
	tw_dhat_save_sum = tw_dhat_pick = tw_dhat_flip = tw_dhat_step2 = tw_dhat_trig = 0;
	tw_chain_n = tw_chain_sum = tw_chain_flip = 0;
	memset(tetris_sq_hist, 0, sizeof(tetris_sq_hist));
	tw_chain_yield_off = tw_chain_starve_off = 0;
	memset(tw_chain_delayed, 0, sizeof(tw_chain_delayed));
	tt_dep_scan_wr = tt_dep_hit = tt_dep_readers_sum = 0;
	tt_dep_age_sum = tt_dep_age_max = tt_dep_stale = 0;
}

static void tetris_gc_burst(struct ssd *ssd, struct lun_queue *q, uint64_t now, int ws)
{
	int g, want;
	bool fired = false;

	if (tetris_gc_thres_lines > 0) {
		if (READ_ONCE(tetris_free_lines) >= tetris_gc_thres_lines) {
			return;
		}
	}

	want = 1;

	for (g = 0; g < want; g++) {
		int gr = tetris_gc_head(q, true, now);
		int gw;
		bool any = false;

		gw = tetris_gc_head_wr_rwp(q, now);
		if (gr >= 0) {
			tetris_pick_slot = gr;
			if (ssd_qserve_one_pick(ssd, q, now, false, lunq3_next_tetris)) {
				any = true;
				q->gc_last_rd = 1;
			}
		}
		if (gw >= 0) {
			tetris_pick_slot = gw;
			if (ssd_qserve_one_pick(ssd, q, now, false, lunq3_next_tetris)) {
				any = true;
				q->gc_last_rd = 0;
			}
		}
		if (!any)
			break;
		tetris_gc_pair++;
		fired = true;
	}

	if (fired) {
		memset(q->gc_numwr, 0, sizeof(q->gc_numwr));
	}
}

static void tetris_scan_channel(struct ssd *ssd, uint32_t ch, uint64_t now)
{
	tetris_scan_channel_x(ssd, ch, now, false);
}

static void ssd_qserve_sweep_tetris(struct ssd *ssd, uint64_t now)
{
	uint32_t ch, lun;

	tetris_pass_n++;
	tetris_epoch_tick(now);
	for (ch = 0; ch < ssd->sp.nchs; ch++) {
		bool any = false;

		for (lun = 0; lun < ssd->sp.luns_per_ch; lun++)
			if (ssd->lq3[ch][lun].n) { any = true; break; }
		if (!any) {
			continue;
		}
		tetris_scan_channel(ssd, ch, now);
		for (lun = 0; lun < ssd->sp.luns_per_ch; lun++) {
			struct lun_queue *q = &ssd->lq3[ch][lun];
			int k, win = tetris_win;

			if (win < 1) win = 1;
			if (win > TETRIS_WIN_MAX) win = TETRIS_WIN_MAX;
			if (!q->n) {
				continue;
			}

			for (k = 0; qserve_budget <= 0 || k < qserve_budget; k++) {
				u64 ah_rd = 0, ah_wr = 0;
				int rs = tetris_class_head(q, (int)lun, TT_CLS_RD, win, now, &ah_rd);
				int ws = tetris_class_head(q, (int)lun, TT_CLS_WR, win, now, &ah_wr);
				int sel;
				bool wb_cont;
				int gc_idle_cls;
				bool gc_idle_early;

				gc_idle_cls = -1;
				gc_idle_early = false;

				wb_cont = false;

				if (q->wbatch_left && ws >= 0) {
					if (rs >= 0 && tetris_wb_rd_max_ns > 0 &&
					    now > q->pend[rs].arr &&
					    now - q->pend[rs].arr > (u64)tetris_wb_rd_max_ns) {
						q->wbatch_left = 0;
						tt_wb_abort_rd++;
					} else if (READ_ONCE(backpressure_free_line_stall)) {
						q->wbatch_left = 0;
						tt_wb_abort_bp++;
					} else {
						sel = ws;
						tetris_pw_wr_win++;
						q->wbatch_left--;
						wb_cont = true;
						tt_wb_continue++;
					}
				}
				if (wb_cont) {
				} else if (rs < 0 && ws < 0) {
					int bg;

					bg = tetris_gc_pick_rwp(q, now);
					if (bg < 0)
						break;
					sel = bg;
					gc_idle_cls = q->pend[bg].op_cmd == NAND_READ ? 1 : 0;
					gc_idle_early = tetris_erase_blocked(q, bg);
				} else if (rs < 0) {
					sel = ws;
					tetris_pw_wr_win++;
				} else if (ws < 0) {
					sel = rs;
					tetris_pw_rd_win++;
				} else {
					u64 ex = tt_chain_charge(q, rs);
					u64 pw_rd = tetris_pw(q, rs, now, ah_rd, 0);
					u64 pw_wr = tetris_pw(q, ws, now, ah_wr, ex);
					bool rd_wins = pw_rd > pw_wr;

					if (ex) {
						tw_chain_n++;
						tw_chain_sum += ex;

						if (rd_wins && pw_rd <= tetris_pw(q, ws, now, ah_wr, 0)) {
							tw_chain_flip++;

							{
								struct lunq3_op *w = &q->pend[ws];
								int k = w->kind < SK_N ? w->kind : SK_MIXED;

								tw_chain_delayed[k] += now > w->arr ? now - w->arr : 0;
							}
						}
					}
					if (rd_wins) {
						sel = rs;
						tetris_pw_rd_win++;
						if (ex)
							q->chain_debt++;
					} else {
						sel = ws;
						tetris_pw_wr_win++;
						q->chain_debt = 0;

						if (tt_wbatch_on()) {
							q->wbatch_left = (uint8_t)(tetris_wbatch - 1);
							q->wbatch_gc_done = 0;
							tt_wb_open++;
						}
					}
				}

				if (sel == ws && ws >= 0 && !q->wbatch_gc_done) {
					tetris_gc_burst(ssd, q, now, ws);
					if (tt_wbatch_on() && (q->wbatch_left || wb_cont))
						q->wbatch_gc_done = 1;
					ws = tetris_class_head(q, (int)lun, TT_CLS_WR, win, now, &ah_wr);
					if (ws < 0)
						continue;
					sel = ws;
				}

				{
					struct lunq3_op *so = &q->pend[sel];

					if (so->op_cmd != NAND_READ && so->op_type == USER_IO) {
						u8 fl = tetris_flow_of(so);

						if (fl < TETRIS_TN_MAX)
							q->gc_numwr[fl]++;
					}
				}
				tetris_pick_slot = sel;
				if (!ssd_qserve_one_pick(ssd, q, now, false, lunq3_next_tetris)) {
					tetris_pick_slot = -1;
					break;
				}
				if (gc_idle_cls >= 0)
					tetris_note_idle_gc(q, gc_idle_cls, gc_idle_early);
			}
			if (qserve_budget > 0 && k == qserve_budget)
				qs_budget_hit++;
		}
	}
}

void ssd_qserve_pass(struct ssd *ssd, uint64_t now)
{
	uint32_t ch, lun;
	uint32_t bp_max_n = 0;

	if (!ssd->lq3 || !ssd->inbox)
		return;

	for (ch = 0; ch < ssd->sp.nchs; ch++)
	for (lun = 0; lun < ssd->sp.luns_per_ch; lun++) {
		struct lun_queue *q = &ssd->lq3[ch][lun];
		struct lunq3_inbox *ib = &ssd->inbox[ch][lun];

		NVMEV_ASSERT(q->n <= lunq3_qmax);

		NVMEV_ASSERT((q->n == 0) == (q->fifo_head == LUNQ3_NIL));
		NVMEV_ASSERT((q->n == 0) == (q->fifo_tail == LUNQ3_NIL));
		lunq3_inbox_drain(ib, q);
		{
			uint32_t occ = (uint32_t)q->n + (READ_ONCE(ib->tail) - ib->head);

			if (occ > bp_max_n)
				bp_max_n = occ;
		}
	}

	if (bp_max_n >= (uint32_t)bp_hi) {
		if (!bp_queue_stall)
			qs_bp_enter++;
		WRITE_ONCE(bp_queue_stall, 1);
	} else if (bp_max_n <= (uint32_t)bp_lo) {
		WRITE_ONCE(bp_queue_stall, 0);
	}
	if (bp_queue_stall) {
		qs_bp_passes++;
		qs_bp_cur++;
	} else if (qs_bp_cur) {
		if (qs_bp_cur > qs_bp_run_max)
			qs_bp_run_max = qs_bp_cur;
		qs_bp_cur = 0;
	}

	ssd_qserve_sweep_tetris(ssd, now);
}
