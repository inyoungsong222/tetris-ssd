// SPDX-License-Identifier: GPL-2.0-only

#ifndef _NVMEVIRT_CACHE_MODEL_H
#define _NVMEVIRT_CACHE_MODEL_H

#include <linux/hashtable.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include "ssd.h"

#include "conv_ftl.h"

#define CACHE_ENTRY_BYTES (16ULL * 1024)
#define CACHE_HASH_BITS 16

#define CM_CLEAN_BKTS 16
#define FLUSH_ASYNC_BUDGET 1

#define SHSTAGE_N_DEF 4096
#define SHSTAGE_BASE (1 << 20)

#define SHSTAGE_SWEEP_DEF 128

enum { GB_SMALL = 0, GB_LARGE, GB_WRITE, GB_N };

enum { RAWK_NONE = 0, RAWK_S, RAWK_L, RAWK_N };

enum { CACHE_MISS = 0, CACHE_HIT = 1, CACHE_HIT_DIRTY = 2, CACHE_WAIT = 3 };

enum { CE_FREE = 0, CE_CLEAN, CE_DIRTY_UNISSUED, CE_DIRTY_ISSUED, CE_LOADING };

#define CE_SLOTS    4
#define CE_LBA_NONE (~0ULL)

#define CE_ALL_MASK ((uint8_t)((1u << CE_SLOTS) - 1))

struct cache_ent {
	uint64_t tag;
	uint64_t clean_at;
	struct ssd *ssd;
	struct ppa ppa;
	uint32_t host_seq;

	uint64_t dirty_ns;
	uint8_t state;
	uint8_t pf;
	uint8_t owner;
	uint8_t was_flushed;

	uint32_t gen;
	uint16_t wait_head;
	uint8_t await;
	uint8_t pblk;

	uint16_t hold_head;

	uint64_t pgkey;
	uint64_t lba[CE_SLOTS];
	uint8_t vmask;
	uint8_t fmask;

	uint8_t wload;

	uint64_t ready_at;

	uint8_t rnd;
	uint8_t rpart;
	uint8_t rpmask;
	uint8_t rfree;

	uint16_t lbhold;
	struct hlist_node pnode;
	struct hlist_node hnode;
	struct list_head lru;
	struct list_head dnode;
	struct list_head gnode;
};

static inline uint64_t ce_pgkey(const struct ppa *ppa)
{
	struct ppa k;

	k = *ppa;
	k.g.pg = ppa->g.pg - (ppa->g.pg % CE_SLOTS);
	k.g.rsv = 0;
	return k.ppa;
}

struct ce_page {
	uint64_t pgkey;
	uint64_t tag;
	uint64_t lba[CE_SLOTS];
	uint8_t live;
	uint8_t chunk;

	uint8_t rnd;
	uint8_t wl_open;
	uint8_t wl_free;
	uint8_t rpend;
};

#define DFW_NIL   0xFFFFu

#define DFW_GEN(g) ((uint32_t)((g) & 0xFFFFu))
#define DFW_POOL  32768
struct dflush_wait {
	uint16_t next;
	uint16_t _pad;
	uint32_t gen;
	uint32_t ticket;
};

struct cache_stats {
	uint64_t hits, hits_dirty, misses, bypasses;

	uint64_t hits_dirty_by[GB_N];
	uint64_t tn_hits_dirty[TENANT_N + 1];
	uint64_t evict_clean, evict_issued, evict_unissued;
	uint64_t stall_ns_sum, stall_ns_max, stall_cnt;

	uint64_t est_stall_n, est_stall_sum, est_stall_max;

	uint64_t est_stall_by[GB_N], exact_stall_by[GB_N];

	uint64_t diq_dep_unissued, diq_dep_issued;

	uint64_t flush_cmd, flush_cmd_ops, flush_cmd_max;

	uint64_t bp_hard_skip;

	uint64_t flush_rr_pick;
	uint64_t exact_stall_n, exact_stall_sum;

	uint64_t stall_own_sum, stall_q_sum;
	uint64_t stall_q_n;
	uint64_t wm_flushes, evict_flushes, coalesced;

	uint64_t evict_flush_by[GB_N];

	uint64_t raw_wait_ns_by[GB_N];
	uint64_t raw_wait_ns_tlc[GB_N];

	uint64_t raw_wait_ns_mig[GB_N];

	uint64_t redirty_clean;
	uint64_t redirty_issued;

	uint64_t tn_hit[TENANT_N + 1], tn_miss[TENANT_N + 1];

	uint64_t rdmiss_clean_hist[CM_CLEAN_BKTS];
	uint64_t rdmiss_clean_min;
	uint64_t rdmiss_clean_n;

	uint64_t exact_stall_max_by[GB_N], est_stall_max_by[GB_N];

	uint64_t bgf_calls;
	uint64_t bgf_exit_budget;
	uint64_t bgf_exit_gate;
	uint64_t bgf_exit_novic;
	uint64_t bgf_issued;

	uint64_t bgf_tail_pick, bgf_tail_none, bgf_brake_die, bgf_brake_global;

	uint64_t bgf_refresh;
	uint64_t exact_stall_tn[TENANT_N + 1], exact_stall_ns_tn[TENANT_N + 1];

	uint64_t est_stall_tn[TENANT_N + 1], est_stall_ns_tn[TENANT_N + 1];
	uint64_t tn_wr[TENANT_N + 1];
	uint64_t tn_evict[TENANT_N + 1];
	uint64_t tn_evict_by[TENANT_N + 1];

	uint64_t ghost_tn[TENANT_N + 1][TENANT_N + 1];
	uint64_t promote_clean;
	uint64_t zero_stall_hit;
	uint64_t zero_stall_miss;
	uint64_t stage_flushes;
	uint64_t bp_skip_die;
	uint64_t bp_fallback;
	uint64_t pf_ins, pf_hit;
	uint64_t pf_wait_cnt, pf_wait_ns;
	uint64_t pf_evict_inflight;
	uint64_t flush_die_full;

	uint64_t dfw_reg, dfw_rel;
	uint64_t dfw_hwm;
	uint64_t dfw_chain_max;

	uint64_t fbh_hold, fbh_now, fbh_cb, fbh_rel, fbh_full, fbh_hwm;

	uint64_t fh_raise_n, fh_raise_sum, fh_raise_max;

	uint64_t rdpick_n, rdpick_free, rdpick_fewest, rdpick_lastch, rdpick_none;

	uint64_t rdx_n, rdx_free, rdx_defer, rdx_expire, rdx_none;

	uint64_t bgf_on_rd, bgf_rd_sum;
	uint64_t flush_offread;
	uint64_t flush_onread;

	uint64_t flush_onread_tn[TENANT_N + 1][TENANT_N + 1];
	uint64_t flush_forced;

	uint64_t stage_acq, stage_stall_cnt, stage_stall_ns_sum, stage_stall_ns_max;
	uint64_t stage_grow, stage_shrink, stage_race;

	uint64_t stage_occ_sum, stage_occ_n;
	uint32_t stage_occ_hwm;
	uint32_t stage_str_hwm[TETRIS_MAX_STREAMS + 1];
	uint64_t stage_str_acq[TETRIS_MAX_STREAMS + 1];
	uint32_t stage_active_hwm;
	uint64_t stage_quota_stall;

	uint64_t ghost_hit[2][GB_N];
	uint64_t ghost_ins;

	uint64_t lba_rd_lookup;
	uint64_t lba_rd_hit, lba_rd_hit_dirty;
	uint64_t lba_rd_miss_new;
	uint64_t lba_rd_miss_part;
	uint64_t lba_rd_miss_wait;
	uint64_t lba_rd_miss_dup;

	uint64_t lba_rd_miss_issued, lba_rd_issued_exact_n, lba_rd_issued_exact_ns;
	uint64_t lba_rd_miss_nofill;
	uint64_t lba_rd_bypass;
	uint64_t lba_rd_hit_wait_n, lba_rd_hit_wait_ns;
	uint64_t lba_pg_stale;
	uint64_t lba_fill_issue;
	uint64_t lba_fill_done;
	uint64_t lba_fill_stale;
	uint64_t lba_skip_loading;
	uint64_t lba_evict_loading;
	uint64_t lba_wr_hit;
	uint64_t lba_wr_wait_issued;
	uint64_t lba_wr_wait_exact;
	uint64_t lba_wr_wait_loading;
	uint64_t lba_wr_wait_fill_exact;
	uint64_t lba_wr_clean_hit;
	uint64_t lba_wr_clean_part;
	uint64_t lba_wr_miss;
	uint64_t lba_wr_miss_tagdup;
	uint64_t lba_wr_force;
	uint64_t lba_rebind_drop;

	uint64_t lba_rd_wait_full;
	uint64_t lba_wr_wait_full;
	uint64_t lba_wr_wait_victim;
	uint64_t lba_wr_exact_victim;
	uint64_t lba_wload_done;
	uint64_t lba_wload_force;
	uint64_t lba_wload_stale;

	uint64_t lba_rand_new;
	uint64_t lba_rand_add;
	uint64_t lba_rand_wait;
	uint64_t lba_rand_ready;
	uint64_t lba_rand_drop;
	uint64_t lba_rand_pad;
	uint64_t lba_rand_late;
	uint64_t lba_rand_bgf[CE_SLOTS];
	uint64_t lba_rand_rebind;

	uint64_t lba_rand_unhash_rd;
	uint64_t lba_rand_unhash_set;
	uint64_t lba_rand_drop_pend;
	uint64_t lba_rand_unpend;
	uint64_t lba_rand_bgf_wskip;
	uint64_t lba_ready_rd_n, lba_ready_rd_ns;
	uint64_t lba_ready_wr;
	uint64_t lba_ready_flush;
};

struct cache_model {
	struct cache_ent *ents;
	uint8_t frame_handle_on;
	uint64_t nr_ents;
	uint64_t used;
	uint64_t dirty_unissued;
	uint64_t clean_cnt;
	uint64_t issued_cnt;
	uint32_t issued_die[TETRIS_NR_DIES];

	uint32_t flush_rr_die;

	uint32_t rdpick_last_ch;
	uint32_t rdpick_lun;
	uint32_t dies_at_cap;
	uint32_t head_skips;
	struct list_head lru_list, dirty_list, free_list;

	struct list_head rand_open;
	uint32_t rand_open_n;
	struct list_head dirty_die[TETRIS_NR_DIES];
	struct list_head issued_list;
	DECLARE_HASHTABLE(map, CACHE_HASH_BITS);

	DECLARE_HASHTABLE(pmap, CACHE_HASH_BITS);
	spinlock_t lock;

	uint64_t *stage_release;
	uint16_t *stage_owner;
	uint32_t stage_cap;
	uint32_t stage_cur;

	uint64_t *sh_release;
	uint32_t sh_cursor;
	uint64_t sh_drop;
	uint64_t sh_acq, sh_commit;
	uint32_t sh_occ_hwm;

	uint32_t sh_live;
	uint32_t sh_sweep;

	uint64_t occ_ewma_x100;
	uint32_t sh_n, sh_mask;
	uint64_t sh_occ_sum, sh_occ_n;
	struct cache_stats st;
};

struct cache_model *dcache_get(void);
void dcache_init(void);
void dcache_flush_done(uint32_t frame, uint64_t done,
		       void (*release)(uint32_t ticket, uint64_t done));
void dcache_exit(void);
void dcache_reset(void);

uint32_t dcache_flush_sync(uint64_t now, uint32_t owner);

enum { CW_DONE = 0, CW_WAIT = 1 };

#define CE_WAIT_FULL 0xFFFFFFFFu
int cache_read_pg(struct cache_model *c, const struct ce_page *pg, uint64_t now, bool admit,
		  uint64_t *stall_ns, bool is_small, uint8_t tn, uint32_t *dep, uint32_t *fill,
		  uint32_t *wait);

int dcache_lb_hold(uint64_t pgkey, int delta);

uint16_t conv_lb_held(uint64_t pgkey);

int cache_read_vpg(struct cache_model *c, const struct ce_page *pg, uint64_t now, uint64_t *stall_ns,
		   bool is_small, uint8_t tn, uint32_t *wait);
int cache_write_pg(struct cache_model *c, const struct ce_page *pg, uint64_t now, uint64_t load_ns,
		   struct ssd *ssd, struct ppa *ppa, uint32_t host_seq, uint64_t *stall_ns, uint8_t tn,
		   bool force, uint32_t *wait, uint32_t *own);
int dcache_fill_done(uint32_t fill, uint64_t done, void (*release)(uint32_t ticket, uint64_t done));
int dcache_wload_done(uint32_t own, uint64_t t, bool forced, uint32_t *rel,
		      void (*release)(uint32_t ticket, uint64_t done));

const char *ce_st_name(uint8_t st);

int dcache_frame_describe(uint32_t frame, char *buf, int len);

int dcache_frame_die(uint32_t frame);

int dcache_rand_drop(uint64_t pgkey, uint32_t slot, uint64_t lpn);

int dcache_rand_unpend(uint64_t pgkey, uint8_t chunk);

void dcache_dump_census(const char *why);

int cache_stage_acquire(struct cache_model *c, uint64_t now, int avoid_die, uint16_t stream_id,
			uint64_t *stall_ns);
void cache_stage_commit(struct cache_model *c, int slot, uint64_t release_at);

extern int cache_mb;
extern int cache_hit_lat_ns;
extern int cache_dirty_ratio;
extern int cache_flush_batch;
extern int flush_mode;
extern int flush_stage_k;

int cache_shadow_acquire(struct cache_model *c, uint64_t now);

static inline bool cache_admit_read(bool is_small_read, bool size_small)
{
	return true;
}

extern bool nvmev_diq_mig_hint;

#endif
