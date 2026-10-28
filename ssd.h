// SPDX-License-Identifier: GPL-2.0-only

#ifndef _NVMEVIRT_SSD_H

#ifndef TENANT_N
#define TENANT_N 16
#endif

#define _NVMEVIRT_SSD_H

#include <linux/types.h>
#include "pqueue/pqueue.h"
#include "ssd_config.h"
#include "channel_model.h"

#define INVALID_LPN (~(0ULL))
#define UNMAPPED_PPA (~(0ULL))

enum {
	NAND_READ = 0,
	NAND_WRITE = 1,
	NAND_ERASE = 2,

	NAND_NOP_RETIRED = 3,
};

enum {
	USER_IO = 0,
	GC_IO = 1,

	FOLD_IO = 4,
};

enum {
	SEC_FREE = 0,
	SEC_INVALID = 1,
	SEC_VALID = 2,

	PG_FREE = 0,
	PG_INVALID = 1,
	PG_VALID = 2
};

enum { CELL_TYPE_LSB, CELL_TYPE_MSB, CELL_TYPE_CSB, MAX_CELL_TYPES };

#define TOTAL_PPA_BITS (64)
#define BLK_BITS (16)
#define PAGE_BITS (16)
#define PL_BITS (8)
#define LUN_BITS (8)
#define CH_BITS (8)
#define RSB_BITS (TOTAL_PPA_BITS - (BLK_BITS + PAGE_BITS + PL_BITS + LUN_BITS + CH_BITS))

struct ppa {
	union {
		struct {
			uint64_t pg : PAGE_BITS;
			uint64_t blk : BLK_BITS;
			uint64_t pl : PL_BITS;
			uint64_t lun : LUN_BITS;
			uint64_t ch : CH_BITS;
			uint64_t rsv : RSB_BITS;
		} g;

		struct {
			uint64_t : PAGE_BITS;
			uint64_t blk_in_ssd : BLK_BITS + PL_BITS + LUN_BITS + CH_BITS;
			uint64_t rsv : RSB_BITS;
		} h;

		uint64_t ppa;
	};
};

typedef int nand_sec_status_t;

struct nand_page {
	nand_sec_status_t *sec;
	int nsecs;
	int status;
};

struct nand_block {
	struct nand_page *pg;

	bool slc_mode;
	int npgs;
	int ipc;
	int vpc;
	int erase_cnt;
	int wp;
};

struct nand_plane {
	struct nand_block *blk;
	uint64_t next_pln_avail_time;
	int nblks;
};

#define TETRIS_SMALL_READ_BYTES	(32 * 1024)

extern int pslc_prog_ns;
extern u64 pslc_rs_r_sum, pslc_rs_n, pslc_rs_r_hwm;

extern int pslc_raw_reserve;

extern int pslc_raw_rebind;
extern int pslc_read_ns;
extern int pslc_erase_ns;

extern u64 er_cnt[2], er_ns[2];
extern u64 er_migr_cnt[2], er_migr_ns[2];
void pslc_note_slc_block(uint32_t blk);

enum { PSLC_OFF = 0, PSLC_B1_FIXED, PSLC_B2_ELASTIC_RETIRED, PSLC_RESERVE, PSLC_CONZONE,
       PSLC_TETRIS, PSLC_WGC2, PSLC_TETRIS_RAW };

extern int tlc_line_cnt, slc_line_cnt;
extern int pslc_rs_cmax_pgs;
uint32_t pslc_rs_cmax(void);
extern int pslc_rs_alpha_ppk;
extern int pslc_rs_dwell;
extern u64 pslc_rs_steps;
extern u64 pslc_rs_flips, pslc_rs_rbar_sum;

extern u64 pslc_rs_up_n, pslc_rs_dn_n, pslc_rs_hold_n;
extern int pslc_rs_tw3_ms;
extern bool pslc_rs_tw3_heavy;

extern u64 pslc_rs_tw_min;
extern u64 pslc_rs_tw_last;
extern s64 pslc_rs_noise_last;
extern u64 pslc_rs_undo_n;
extern u64 pslc_rs_probe_n;
extern int pslc_fold_batch, pslc_fold_qd;
extern u64 pslc_adm[2], pslc_rej[2];
extern u64 pslc_reserve_hwm, pslc_reserve_sum, pslc_reserve_n;

extern u64 pslc_rawwin_n, pslc_rawwin_min;
extern u64 slcline_open, slcline_full, slcline_gc_victim, slcline_reclaimed;
extern u64 slcline_host_pgs, slcline_gc_moved, slcline_free_hwm, slcline_starved;

extern u64 slcline_strict_fb;

extern u64 la_take[6][2], la_fail[6], la_edge, la_lo_after, la_calls;

extern u64 cr_free_lo;
extern u64 la_freehist[8];

extern u64 cyc_alloc[8];
struct ssd;

#define PSLC_DENSITY_DIV 3

void pslc_fold_tick(struct ssd *ssd, uint64_t now);

void pslc_set_pool_pages(uint32_t pages);

void pslc_note_flush(bool raw);

void pslc_window_reset(void);

uint64_t ssd_program_ns(struct ssd *ssd, struct ppa *ppa);

int conv_rand_pad_wl(struct ssd *ssd, struct ppa *wl);
int conv_rebind_raw_to_slc_k(struct ssd *ssd, uint64_t tag, struct ppa *cur, struct ppa *out, int rawk,
			     uint64_t *lbas);

#define VPA_BLK_BIT 0x8000u
static inline bool ppa_is_virtual(const struct ppa *p)
{
	return p->ppa != UNMAPPED_PPA && (p->g.blk & VPA_BLK_BIT);
}
#define LB_BIND_SLC 1
#define LB_BIND_RAW 2

int conv_bind_wl(struct ssd *ssd, struct ppa *vppa, int rawk, struct ppa *out, uint8_t *bmask);

void conv_bind_peek(struct ssd *ssd, const struct ppa *vppa, struct ppa *out);

#define RAWK_SLOTS 4

uint32_t pslc_reserve_pgs(void);
bool nvmev_ppa_is_slc(struct ssd *ssd, struct ppa *ppa);
bool nvmev_ppa_line_migrating(struct ssd *ssd, struct ppa *ppa);
extern u64 slcline_raw_rebind, slcline_raw_nospace;
void pslc_free(void);

extern int gc_accounting;

extern u64 gcacct_erase, gcacct_rr_erase, gcacct_lines, gcacct_vpc_sum, gcacct_move, gcacct_hostwr;
extern u64 gcacct_valid_by_tn[TENANT_N + 1];
extern u64 valid_pages_by_tenant[TENANT_N + 1];
void gcacct_reset(void);

void tenant_map_clear(void);
void tenant_map_set(u8 tn, u64 start_lba, u64 nlba);

void scanreg_set(uint16_t id, uint64_t start_lba, uint64_t nlba);
void scanreg_clear(void);

#ifndef NVME_RW_DTYPE_STREAMS
#define NVME_RW_DTYPE_STREAMS (1 << 4)
#endif
#define CDW13_DSPEC(x)       (((x) >> 16) & 0xFFFF)

extern u64 qs_inbox_wait_rd, qs_inbox_wait_wr, qs_inbox_wait_er;

extern u64 nvmev_die_end_out;
extern u64 e8_over_n, e8_over_sum, e8_over_max, e8_under_n, e8_under_sum, e8_under_max;
extern u64 diq_park, diq_resume_n, diq_full, diq_stale, diq_live, diq_hwm;

extern u64 diq_full_live;

struct diq_dep_adv {
	uint32_t frame;
	uint32_t n;
	uint64_t t0;
};
extern struct diq_dep_adv diq_dep_adv[1 << 16];
extern u64 diq_dep_pub, diq_dep_clr;
extern u64 diq_delay_n, diq_delay_sum, diq_delay_max, diq_nodelay;
extern u64 diq_delay_n_tn[TENANT_N + 1];

extern u64 diq_mig_n, diq_mig_sum, diq_last_done;
extern u64 diq_mig_sum_tn[TENANT_N + 1];

void nvmev_diq_mig_window4(u64 *d_mig, u64 *n_mig, u64 *d_oth, u64 *n_oth, u64 *t_ns);

void nvmev_diq_dw_sample(u64 *d_tot, u64 *n_tot, u64 *t_ns);

extern u64 dw_n, dw_sum, dw_sq, dw_min;
extern u64 tw_sum, tw_min;
void nvmev_diq_mig_reset(void);

uint32_t pslc_mig_legacy_pgs(void);
#define DIQ_HIST_BKTS 40
extern u64 diq_delay_hist_tn[TENANT_N + 1][DIQ_HIST_BKTS];
extern u64 diq_delay_hsum_tn[TENANT_N + 1][DIQ_HIST_BKTS];
void pslc_fold_observe(uint64_t done, bool from_nand);
extern u32 qs_hwm_user, qs_hwm_gc, qs_hwm_fold;
extern u64 qs_pub, qs_drained_done, qs_outbox_full, qs_inbox_dropped, qs_forced_ahead, qs_budget_hit,
	   qs_bp_passes, qs_bp_enter, qs_bp_run_max, qs_bp_cur;

extern int tetris_wbatch;

extern int tetris_wb_rd_max_ns;
extern u64 tt_wb_open, tt_wb_continue, tt_wb_abort_rd, tt_wb_abort_bp;

extern int bp_queue_stall, backpressure_free_line_stall;
extern int gc_target_lines, gc_reserve_lines;
extern int nvmev_sq_stall, bp_hi, bp_lo, lunq3_qmax;

void qserve_stats_reset(void);

#define LUNQ3_BP_HI_OF(q) (LUNQ3_BP_MARK < (q) ? LUNQ3_BP_MARK : (q) - (q) / 8)
#define LUNQ3_BP_LO_OF(q) (LUNQ3_BP_HI_OF(q) - LUNQ3_BP_HI_OF(q) / 16)
extern int qserve_budget;
void lunq3_outbox_drain(struct ssd *ssd,
			void (*apply)(struct ssd *, uint32_t, uint32_t, uint64_t, uint32_t, uint32_t));

enum { RDERR_DRAIN = 0, RDERR_FORCED, RDERR_PBREL, RDERR_BGFULL, RDERR_SRC_N };
struct ssd; struct ppa;
extern int op_trace;

struct optrace_ent {
	uint64_t arrival;
	uint64_t start;
	uint64_t end;
	uint32_t host_seq;
	uint8_t ch, lun, tier, cmd;
	uint8_t io_type;

	uint8_t kind;
	uint16_t blat_us;
	uint32_t hold_ns;
};
#define OPTRACE_LOG2_DEF 21
#define OPTRACE_CMD_CACHE_HIT 4

#define OPTRACE_CMD_DIQ_WAIT 5

#define OPTRACE_CMD_HOST_RD 6
extern int op_trace_log2;
void ssd_optrace_record_hit(uint32_t host_seq, uint64_t arrival, uint64_t end);
void ssd_optrace_record_diq(uint32_t host_seq, uint64_t was, uint64_t done, uint8_t tn, uint8_t mig);
void ssd_optrace_record_host(uint32_t host_seq, uint64_t arrival, uint8_t tn, uint8_t large, uint32_t bytes);
void ssd_optrace_init(void);
void ssd_optrace_exit(void);
uint64_t ssd_optrace_total(void);
uint64_t ssd_optrace_n(void);
void ssd_optrace_reset(void);
struct optrace_ent *ssd_optrace_ring(void);

#include <linux/spinlock.h>

enum { SK_SMALL_RD = 0, SK_LARGE_RD, SK_HOST_WR, SK_GC, SK_ERASE, SK_MIXED,
       SK_FOLD, SK_MIGRATE, SK_FOLD_CNZ, SK_N };
#define LUNQ3_CRING 256
extern u64 lunq3_cring_drop;

extern u64 tetris_sd_wait[2][SK_N];
extern u64 tetris_sd_max[2][SK_N];

extern u64 tetris_sd_wait_tn[2][SK_N][TENANT_N + 1];
extern u64 tetris_sd_cache[2];

extern u64 tetris_sd_chnl[2], tetris_sd_chnl_max[2], tetris_sd_chnl_n[2];
void tetris_stall_reset(void);

extern u64 tt_stream_op, tt_stream_rd, tt_chain_same_stream, tt_chain_diff_stream;

extern u64 tt_cr_rd_n, tt_cr_cont_n;

extern u64 tt_cr_run_hist[8], tt_cr_run_n, tt_cr_run_sum, tt_cr_run_max;
extern u64 tetris_sd_nand[2];
extern u64 tetris_sd_cnt[2];
extern u64 tetris_sd_queued_sum[2], tetris_sd_queued_max[2], tetris_sd_cache_max[2], tetris_sd_nand_max[2];
#define LUNQ_NLEVELS 5
#define LUNQ_RING 64
struct lunq_ent {
	uint64_t start, end;
};

struct nand_lun {
	struct nand_plane *pl;
	int npls;
	uint64_t next_lun_avail_time;
	bool busy;

	uint8_t sel78h : 1;
	uint8_t cr_open : 1;
	uint8_t cr_rsvd : 6;
	uint64_t gc_endtime;

	spinlock_t q_lock;
	uint64_t q_subtail[LUNQ_NLEVELS];
	struct lunq_ent q_ring[LUNQ_RING];
};

struct ssd_channel {
	struct nand_lun *lun;
	int nluns;
	uint64_t gc_endtime;
	struct channel_model *perf_model;
};

struct ssd_pcie {
	struct channel_model *perf_model;
};

struct nand_cmd {
	int type;
	int cmd;

	int bypass_n;
	uint64_t xfer_size;
	uint64_t stime;
	bool interleave_pci_dma;
	bool is_small_read;
	bool size_small;
	uint32_t cache_frame;

	uint32_t cache_fill;

	uint64_t raw_ppa_enc;
	bool pslc;

	bool fold;

	bool migrate;
	uint32_t host_seq;

	uint32_t owner;
	uint32_t host_bytes;

	uint16_t sq_id;
	uint16_t stream_id;

	uint8_t seq_hint;

	uint8_t tn;
	struct ppa *ppa;
};

struct ssdparams {
	int secsz;
	int secs_per_pg;
	int pgsz;
	int pgs_per_flashpg;
	int flashpgs_per_blk;

	int flashpgs_slc_per_blk;
	int pgs_per_oneshotpg;
	int oneshotpgs_per_blk;
	int pgs_per_blk;
	int blks_per_pl;
	int pls_per_lun;
	int luns_per_ch;
	int nchs;
	int cell_mode;

	int write_unit_size;

	int pg_4kb_rd_lat
		[MAX_CELL_TYPES];
	int pg_rd_lat[MAX_CELL_TYPES];
	int pg_wr_lat;
	int blk_er_lat;
	int max_ch_xfer_size;

	int fw_4kb_rd_lat;
	int fw_rd_lat;
	int fw_wbuf_lat0;
	int fw_wbuf_lat1;
	int fw_ch_xfer_lat;

	uint64_t ch_bandwidth;
	uint64_t pcie_bandwidth;

	unsigned long secs_per_blk;
	unsigned long secs_per_pl;
	unsigned long secs_per_lun;
	unsigned long secs_per_ch;
	unsigned long tt_secs;

	unsigned long pgs_per_pl;
	unsigned long pgs_per_lun;
	unsigned long pgs_per_ch;
	unsigned long tt_pgs;

	unsigned long blks_per_lun;
	unsigned long blks_per_ch;
	unsigned long tt_blks;

	unsigned long secs_per_line;
	unsigned long pgs_per_line;
	unsigned long blks_per_line;
	unsigned long tt_lines;

	unsigned long pls_per_ch;
	unsigned long tt_pls;

	unsigned long tt_luns;

	unsigned long long write_buffer_size;
};

static inline uint32_t slc_oneshotpgs_per_blk(const struct ssdparams *spp)
{
	uint32_t n = spp->oneshotpgs_per_blk / PSLC_DENSITY_DIV;

	return n ? n : 1u;
}

static inline uint32_t slc_pgs_per_blk(const struct ssdparams *spp)
{
	return slc_oneshotpgs_per_blk(spp) * spp->pgs_per_oneshotpg;
}

#define LUNQ3_BP_MARK 1024
#define LUNQ3_PAGES_PER_MAX_REQ 64
#define LUNQ3_HEADROOM (LUNQ3_PAGES_PER_MAX_REQ * 2)
#define LUNQ3_QMAX_MAX (LUNQ3_BP_MARK + LUNQ3_HEADROOM)

#define LUNQ3_NIL 0xFFFFu

enum tetris_q { TETRIS_RDQ = 0, TETRIS_WRQ, TETRIS_GC_RDQ, TETRIS_GC_WRQ, TETRIS_NQ };

struct tetris_queue {
	uint16_t head, tail;
	uint16_t n;
};
struct lunq3_op {
	uint64_t arr;
	uint64_t dur;
	uint64_t seq;
	int level;

	uint32_t cache_frame;
	uint32_t cache_fill;
	uint8_t tt_defer;

	int bypass_n;
	uint8_t tt_overtaken;

	uint8_t tt_over_sd;

	uint64_t gseq;

	uint32_t tetris_alone_ns;

	uint16_t stream_id;
	uint8_t  seq_hint;
	uint16_t sq_id;
	int is_wr;
	uint8_t is_host_wr;
	uint8_t is_host_rd;
	uint8_t saw_susp;
	uint32_t rd_bytes;
	uint32_t host_bytes;
	union {
		uint64_t rep_t;
		uint64_t raw_ppa_enc;
	};
	uint64_t unit_seq;
	uint8_t kind;

	struct ppa ppa;
	uint32_t xfer_size;
	uint32_t host_seq;
	uint32_t owner;
	uint8_t op_cmd;
	uint8_t op_type;
	uint8_t pslc;
	uint8_t interleave_dma;
	uint8_t is_small_read;
	uint64_t base_lat;
};
#define TETRIS_SQ_MAX 16

#define TETRIS_FLOW_SLOTS (TETRIS_SQ_MAX + 1)

#define SG_TAB 8
#define TETRIS_TN_MAX ((TENANT_N + 1) > TETRIS_FLOW_SLOTS ? (TENANT_N + 1) : TETRIS_FLOW_SLOTS)

struct lun_queue {
	spinlock_t lock;
	uint64_t served_until;

	uint64_t p0_last;
	uint8_t  p0_state;
	uint64_t seq;
	int n;

	uint32_t host_rd_n;

	uint32_t gc_rd_n;

	uint64_t susp_debt;
	uint32_t susp_n;
	uint8_t susp_kind;

	struct { uint64_t start, end; uint8_t kind; } cring[LUNQ3_CRING];

	uint32_t cr_run;
	uint32_t cr_selrun;
	uint32_t cr_xfer;
	uint8_t  cr_open : 1;

	uint8_t  cr_run_g : 2;
	uint8_t  cr_selrun_g : 2;
	uint8_t  cr_rsvd : 3;

	uint16_t chain_debt;

	uint8_t wbatch_left;
	uint8_t wbatch_gc_done;

	uint8_t gc_last_rd;

	uint16_t cr_stream;
	uint32_t cr_run_ng;

	uint8_t  cr_run_tn;

	uint32_t sg_last_blk, sg_last_pg;
	uint16_t sg_last_stream;
	uint8_t  sg_last_valid;
	uint8_t  sg_pred_valid;
	uint64_t sg_pred_lpn;
	uint32_t sg_tick;
	struct {
		uint16_t stream;
		uint8_t  has_stride;
		uint32_t lru;
		uint64_t last_lpn;
		int64_t  stride;
	} sg_tab[SG_TAB];

	uint32_t gc_numwr[TETRIS_TN_MAX];

	uint32_t fq_ver;

	struct tetris_queue fq[TETRIS_NQ];
	uint16_t fq_next[LUNQ3_QMAX_MAX];
	uint16_t fq_prev[LUNQ3_QMAX_MAX];
	uint8_t  fq_which[LUNQ3_QMAX_MAX];
	uint32_t ch, ct;

	uint32_t pb_debt;
	struct lunq3_op pend[LUNQ3_QMAX_MAX];

	uint16_t nxt[LUNQ3_QMAX_MAX];
	uint16_t prv[LUNQ3_QMAX_MAX];
	uint16_t fifo_head, fifo_tail;
	uint16_t free_head;

	uint64_t queued_ns;

	uint8_t ireq_credit;
	uint8_t dreq_credit;
	uint8_t cur_phase;
	uint64_t cur_ready;
	uint64_t cur_wait_from;

	uint64_t cur_start;
	struct lunq3_op cur;
};

#define LUNQ3_INBOX_SPIN_MAX (1u << 22)
#define LUNQ3_INBOX_SZ 2048
#define LUNQ3_INBOX_MASK (LUNQ3_INBOX_SZ - 1)
struct lunq3_inbox {
	struct lunq3_op slot[LUNQ3_INBOX_SZ];
	uint32_t head;
	uint32_t tail;
};

#define LUNQ3_OUTBOX_SZ 65536
#define LUNQ3_OUTBOX_MASK (LUNQ3_OUTBOX_SZ - 1)

struct lunq3_done {
	uint32_t owner;

	uint32_t xfer;
	uint64_t chnl_end;

	uint32_t frame;
	uint32_t fill;
	uint64_t raw_ppa_enc;
	uint32_t raw_token;

	uint8_t fold_prog;
};

struct lunq3_outbox {
	struct lunq3_done slot[LUNQ3_OUTBOX_SZ];
	uint32_t head;
	uint32_t tail;
};

struct ssd {
	struct ssdparams sp;
	struct ssd_channel *ch;
	struct ssd_pcie *pcie;
	unsigned int cpu_nr_dispatcher;

	struct lun_queue (*lq3)[LUNS_PER_NAND_CH];
	struct lunq3_inbox (*inbox)[LUNS_PER_NAND_CH];
	struct lunq3_outbox *outbox;
};

static inline struct ssd_channel *get_ch(struct ssd *ssd, struct ppa *ppa)
{
	return &(ssd->ch[ppa->g.ch]);
}

static inline struct nand_lun *get_lun(struct ssd *ssd, struct ppa *ppa)
{
	struct ssd_channel *ch = get_ch(ssd, ppa);
	return &(ch->lun[ppa->g.lun]);
}

#define TETRIS_NR_DIES (NAND_CHANNELS * LUNS_PER_NAND_CH)
extern uint64_t tetris_lr_busy[TETRIS_NR_DIES];

extern uint8_t tetris_lr_busy_tn[TETRIS_NR_DIES];
extern uint64_t tetris_sr_busy[TETRIS_NR_DIES];

#define TENANT_MAP_CHUNK_LPNS 256
#define TENANT_MAP_N (128 * 1024)

static inline int ppa_die_id(struct ppa *ppa)
{
	return ppa->g.ch * LUNS_PER_NAND_CH + ppa->g.lun;
}

static inline uint32_t die_occupancy(struct ssd *ssd, struct ppa *ppa)
{
	const struct lunq3_inbox *ib;
	const struct lun_queue *q;

	if (!ssd->lq3)
		return 0;
	q = &ssd->lq3[ppa->g.ch][ppa->g.lun];
	ib = &ssd->inbox[ppa->g.ch][ppa->g.lun];
	return (uint32_t)READ_ONCE(q->n) + (READ_ONCE(ib->tail) - READ_ONCE(ib->head));
}

static inline uint32_t die_host_rd_n(struct ssd *ssd, struct ppa *ppa)
{
	if (!ssd->lq3)
		return 0;
	return READ_ONCE(ssd->lq3[ppa->g.ch][ppa->g.lun].host_rd_n);
}

static inline uint64_t die_free_at_ns(struct ssd *ssd, struct ppa *ppa, uint64_t now)
{
	uint64_t su;

	if (!ssd->lq3)
		return READ_ONCE(get_lun(ssd, ppa)->next_lun_avail_time);
	su = READ_ONCE(ssd->lq3[ppa->g.ch][ppa->g.lun].served_until);
	if (su < now)
		su = now;
	return su + READ_ONCE(ssd->lq3[ppa->g.ch][ppa->g.lun].queued_ns);
}

#define TETRIS_MAX_STREAMS 32
#define TETRIS_MAX_SIDS 8
struct scan_stream {
	uint16_t id;
	uint64_t next_lpn;
	uint64_t ra_lpn;
	uint64_t last_ns;
	uint64_t cmds;
};
struct scan_sid {
	uint16_t id;
	uint8_t judged, seqok;
	uint32_t w_cmds, w_miss;
	uint64_t cmds, miss, last_ns;
	uint64_t streamed;
};
extern struct scan_stream tetris_streams[TETRIS_MAX_STREAMS];
extern struct scan_sid tetris_sids[TETRIS_MAX_SIDS];

static inline void tetris_lr_mark(struct ppa *ppa, uint64_t done, uint8_t tn)
{
	int d = ppa_die_id(ppa);

	if (done > READ_ONCE(tetris_lr_busy[d])) {
		WRITE_ONCE(tetris_lr_busy[d], done);
		WRITE_ONCE(tetris_lr_busy_tn[d], tn);
	}
}

static inline void tetris_sr_mark(struct ppa *ppa, uint64_t done)
{
	int d = ppa_die_id(ppa);

	if (done > READ_ONCE(tetris_sr_busy[d]))
		WRITE_ONCE(tetris_sr_busy[d], done);
}

static inline struct nand_plane *get_pl(struct ssd *ssd, struct ppa *ppa)
{
	struct nand_lun *lun = get_lun(ssd, ppa);
	return &(lun->pl[ppa->g.pl]);
}

static inline struct nand_block *get_blk(struct ssd *ssd, struct ppa *ppa)
{
	struct nand_plane *pl = get_pl(ssd, ppa);
	return &(pl->blk[ppa->g.blk]);
}

static inline struct nand_page *get_pg(struct ssd *ssd, struct ppa *ppa)
{
	struct nand_block *blk = get_blk(ssd, ppa);
	return &(blk->pg[ppa->g.pg]);
}

static inline uint32_t get_cell(struct ssd *ssd, struct ppa *ppa)
{
	struct ssdparams *spp = &ssd->sp;
	return (ppa->g.pg / spp->pgs_per_flashpg) % (spp->cell_mode);
}

void ssd_init_params(struct ssdparams *spp, uint64_t capacity, uint32_t nparts);
int ssd_init(struct ssd *ssd, struct ssdparams *spp, uint32_t cpu_nr_dispatcher);
void ssd_remove(struct ssd *ssd);

#define TETRIS_STALL_DECOMP 1

#define TETRIS_SD_L(small)  ((small) ? 0 : 1)

#if TETRIS_STALL_DECOMP
void lunq3_attr_resid_at(struct ssd *ssd, struct ppa *ppa, uint64_t arr, int L, uint64_t dur,
			 uint8_t tn);

extern int rawp_sh_slots;
extern int rawp_sh_sweep;

extern int rawp_occ_r_ppk;

extern u64 rawp_pick_n;

extern u64 rawp_vpc0_hit, rawp_scan_sum, mig_erase2_n, wgc2_mig_fail_n;

#define RDCNT_BKT 5
void conv_note_line_read(struct ppa *ppa, uint64_t now_ns);
extern u64 slcgc_rdcnt_n, slcgc_rdcnt_sum, slcgc_rdcnt_max, slcgc_rdcnt_hist[RDCNT_BKT];
#endif

u8 conv_qserve_tenant_of_ppa(struct ppa *ppa);

extern int tetris_win, tetris_alpha_pct;

extern int gc_force_lines;
extern u64 cr_reads, cr_pipe_n;

#define CR_G_N    3

extern u64 cr_st_n[2][CR_G_N], cr_st_sum[2][CR_G_N], cr_st_sq[2][CR_G_N],
	   cr_st_min[2][CR_G_N], cr_st_max[2][CR_G_N];

void cr_reset(void);

extern int tetris_prio_tn1, tetris_prio_tn2, tetris_prio_tn3;

extern u64 tetris_scan_n, tetris_scan_rd, tetris_scan_wr, tetris_scan_gc;
extern u64 tetris_epoch_n, tetris_cls_lo_rd, tetris_cls_hi_rd, tetris_cls_lo_wr, tetris_cls_hi_wr;
extern u64 tetris_tn_seen[TETRIS_TN_MAX];

extern u64 tt_cr_rd_n_tn[TETRIS_TN_MAX], tt_cr_cont_n_tn[TETRIS_TN_MAX];
extern u64 tt_cr_run_n_tn[TETRIS_TN_MAX], tt_cr_run_sum_tn[TETRIS_TN_MAX];
extern u64 tt_cr_break_gap_tn[TETRIS_TN_MAX][TETRIS_TN_MAX], tt_cr_break_wr_tn[TETRIS_TN_MAX];

extern u64 tt_cr_break_blk_tn[TETRIS_TN_MAX];
extern int pslc_raw_reserve_pgs;
extern u64 tetris_sd_n, tetris_sd_sum_ppm, tetris_sd_max_ppm;
extern u64 tetris_F_n, tetris_F_sum_ppm, tetris_F_min_ppm;
extern u64 tetris_win_full, tetris_win_clamp, tetris_ch_scan, tetris_pass_n;
extern u64 tetris_prio_seen[8], tetris_prio_levels;
extern u64 tetris_move_n, tetris_hops_sum, tetris_step1_n, tetris_step2_n;

extern u64 tetris_stv_n, tetris_stv_sum_ns, tetris_stv_max_ns;
extern u64 tetris_ctl_n, tetris_ctl_sum_ns, tetris_ctl_max_ns;
extern int tetris_fthr_pct;

extern int tetris_gc_thres_lines;

extern int tetris_free_lines;
extern int gc_retry_max;
extern u64 wgc2_slcgc_n, wgc2_mig_n, wgc2_tlc_redirect, wgc2_gc_retry_hit, wgc2_slc_keep, wgc2_slc_floor, wgc2_gcsweep_n, wgc2_tlcgc_n;
extern u64 gcfp_n[2];

extern u32 tfl_rdp[];
void tfl_reset(void);

extern u64 tt_tm_wr_slc, tt_tm_wr_tlc, tt_tm_rd_slc, tt_tm_rd_tlc;
extern u64 tt_tm_wr_slc_ns, tt_tm_wr_tlc_ns, tt_tm_rd_slc_ns, tt_tm_rd_tlc_ns;

extern u64 tt_tw_rd_n, tt_tw_rd_sum, tt_tw_rd_max;
extern u64 tt_tw_wr_n, tt_tw_wr_sum, tt_tw_wr_max;
extern u64 tt_tw_gc_n, tt_tw_gc_sum, tt_tw_gc_max;
extern u64 tt_tw_el[2], tt_tw_cb[2], tt_tw_ah[2];

extern u64 tt_tw_ah_x[2];
extern u64 tw_ahead_pick, tw_ahead_flip, tw_ahead_step2, tw_ahead_trig;

extern u64 tt_tw_ah_s[2];
extern u64 tw_dhat_scan, tw_dhat_chain, tw_dhat_noopen, tw_dhat_noopen_blk, tw_dhat_gap, tw_dhat_blk;
extern u64 tw_dhat_save_sum, tw_dhat_pick, tw_dhat_flip, tw_dhat_step2, tw_dhat_trig;

extern u64 tw_chain_n, tw_chain_sum, tw_chain_flip;
extern u64 tw_chain_delayed[SK_N], tw_chain_yield_off, tw_chain_starve_off;
extern int tetris_chain, tetris_chain_max;

extern u64 tetris_sq_hist[TETRIS_SQ_MAX][TENANT_N + 1];

extern u64 tt_dep_scan_wr, tt_dep_hit, tt_dep_readers_sum;

extern u64 tt_F_min_ppm;

extern u64 tt_dep_age_sum, tt_dep_age_max, tt_dep_stale;
void tt_reset(void);
extern u64 tetris_pw_rd_win, tetris_pw_wr_win, tetris_gc_pair, tetris_wrr_n;
void tetris_reset(void);

uint64_t ssd_advance_nand(struct ssd *ssd, struct nand_cmd *ncmd);
uint64_t ssd_advance_pcie(struct ssd *ssd, uint64_t request_time, uint64_t length);
uint64_t ssd_advance_write_buffer(struct ssd *ssd, uint64_t request_time, uint64_t length);

void ssd_qserve_pass(struct ssd *ssd, uint64_t now);

extern u32 qs_pend_hwm;

#endif
