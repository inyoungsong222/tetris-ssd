// SPDX-License-Identifier: GPL-2.0-only

#ifndef _NVMEVIRT_CONV_FTL_H
#define _NVMEVIRT_CONV_FTL_H

#include <linux/types.h>
#include "pqueue/pqueue.h"
#include "ssd_config.h"
#include "ssd.h"

struct convparams {
	uint32_t gc_thres_lines;
	uint32_t gc_thres_lines_high;
	bool enable_gc_delay;

	double op_area_pcent;
	int pba_pcent;
};

#define GC_WRITE_POINTER_MAX_LUNS               16

enum line_mode { LINE_TLC = 0, LINE_SLC = 1 };

enum line_pool { TLC_POOL = 0, SLC_POOL = 1, LINE_POOL_N = 2 };

struct line {
	int id;
	uint8_t mode;
	int ipc;
	int vpc;
	struct list_head entry;

	size_t pos;

	uint64_t gc_pri;

	uint64_t mig_order;
	size_t mig_pos;

	uint32_t mig_slice_k;
	uint32_t mig_slice_moved;

	uint64_t free_seq;

	uint32_t rd_cnt;

	uint64_t last_rd_ns;
	uint64_t rd_gap_sum;
	uint32_t rd_gap_n;
	uint32_t rd_life_n;

	uint8_t inflight;
};

struct write_pointer {
	struct line *curline;
	uint32_t ch;
	uint32_t lun;
	uint32_t pg;
	uint32_t blk;
	uint32_t pl;

	uint32_t written;
};

struct line_mgmt {
	struct line *lines;

	struct list_head free_line_list;
	pqueue_t *victim_line_pq;
	struct list_head full_line_list;

	struct list_head pslc_line_list;

	struct list_head pslc_free_line_list;

	pqueue_t *pslc_victim_line_pq;
	struct list_head pslc_full_line_list;

	uint32_t tt_lines;
	uint32_t free_line_cnt;
	uint32_t free_slc_line_cnt;
	uint32_t victim_line_cnt;
	uint32_t full_line_cnt;
	uint32_t pslc_victim_line_cnt;
	uint32_t pslc_full_line_cnt;
};

struct write_flow_control {
	uint32_t write_credits;
	uint32_t credits_to_refill;
};

enum tetris_mig_state {
	TMIG_IDLE = 0,
	TMIG_PICKED,
	TMIG_COPY,
	TMIG_ERASE,
	TMIG_DONE,
};

enum tetris_mig_pause_reason {
	TMIG_PAUSE_NONE = 0,
	TMIG_PAUSE_PAGE_BUDGET,
	TMIG_PAUSE_OP_BUDGET,
	TMIG_PAUSE_STATE_BOUNDARY,
	TMIG_PAUSE_TEST,

	TMIG_PAUSE_LARGE_READ,
	TMIG_PAUSE_READ_QD,
	TMIG_PAUSE_MIG_WAIT,
	TMIG_PAUSE_N,
};

struct tetris_mig_job {
	u8 state;
	u8 pause_reason;
	u8 pause_latched;
	u8 reserved;
	u64 job_id;
	u32 victim_line_id;
	u32 cursor_unit;
	u32 cursor_erase;
	u32 vpc_at_start;
	u32 slices;
	u32 trigger_reason;
	u32 candidate_score;
	u32 erased_blocks;
	u64 start_seq;
	u64 start_ns;
	u64 last_run_ns;
	u64 erase_nand_ns;
	u64 pause_ns;
};

struct tetris_mig_issue_ctl {
	u64 last_eval_ns;
	u64 last_change_ns;
	u64 read_snap;
	u64 large_wait_snap;
	u64 mig_wait_snap;
	u64 large_wait_ewma_ns;
	u64 mig_wait_ewma_ns;
	u32 read_qd;
	u32 headroom;
	u32 page_budget;
	u32 op_budget;
	u8 pause_reason;
	u8 emergency;
	u8 initialized;
	u8 reserved;
};

struct tetris_capacity_ctl {
	u64 last_eval_ns;
	u64 last_change_ns;
	u64 shadow_last_change_ns;
	u64 write_snap;
	u64 raw_snap;
	u64 write_ewma_pages;
	u64 raw_ewma_pages;
	u64 target_low_sum;
	u64 target_high_sum;
	u32 target_low;
	u32 target_high;
	u32 target_low_min;
	u32 target_low_max;
	u8 active;
	u8 shadow_active;
	u8 initialized;
	u8 reserved;
};

struct conv_ftl {
	struct ssd *ssd;

	struct convparams cp;
	struct ppa *maptbl;
	uint64_t *rmap;
	struct write_pointer wp;

	struct write_pointer wp_tn[TENANT_N + 1][2];

	uint8_t *lpn_tenant;
	u64 hot_epoch_cnt;

	struct write_pointer gc_wp_tlc_tn[TENANT_N + 1];

	struct write_pointer wp_slc_UNUSED;

	struct write_pointer wp_slc_tn[TENANT_N + 1][2];

	struct write_pointer wp_rb_slc;

	struct write_pointer wp_mig_tlc;

	struct write_pointer wp_rand_tlc;
	struct write_pointer wp_rand_slc;

	struct write_pointer gc_wp_slc_tn[TENANT_N + 1];
	struct line_mgmt lm;

	pqueue_t *migrating_line_pq;

	struct line *mig_slice_line;
	uint64_t mig_slice_last_ns;
	uint64_t line_write_cnt;
	struct tetris_mig_job imig_job;
	struct tetris_mig_issue_ctl imig_ctl;
	struct tetris_capacity_ctl tcap_ctl;
	struct write_flow_control wfc;

	struct line *gc_active_line;

	int gc_active_die;

	int gc_active_pg[256];
	int gc_active_done;

	uint8_t gc_lun_order[GC_WRITE_POINTER_MAX_LUNS];
	struct line *gc_lun_order_line;
};

void conv_init_namespace(struct nvmev_ns *ns, uint32_t id, uint64_t size, void *mapped_addr,
			 uint32_t cpu_nr_dispatcher);

void conv_remove_namespace(struct nvmev_ns *ns);

bool conv_proc_nvme_io_cmd(struct nvmev_ns *ns, struct nvmev_request *req,
			   struct nvmev_result *ret);

void conv_gc_tick(struct nvmev_ns *ns);
void conv_qserve_pass(struct nvmev_ns *ns, unsigned long long now);
void conv_outbox_drain(struct nvmev_ns *ns);

struct raw_lpn_meta {
	u64 tag;
	u64 wr_ns;
	u32 wr_cmd_seq;
	u32 pending_token;
	u8  media;
	u8  intent;
	u8  first_kind;
	u8  tenant;
	u8  pred_raw;
	u8  hist_raw;

	u8  m7_armed;
	u8  m7_seen;
};

enum raw_read_kind {
	RAW_FORWARDED = 0,
	RAW_PENDING,
	RAW_COMPLETED,
	RAW_NONE,
	RAW_KIND_N
};

#endif
