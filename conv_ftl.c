// SPDX-License-Identifier: GPL-2.0-only

#include <linux/vmalloc.h>
#include <linux/ktime.h>
#include <linux/sched/clock.h>
#include <linux/overflow.h>

#include "nvmev.h"
#include "conv_ftl.h"
#include "cache_model.h"

static atomic_t tetris_host_seq = ATOMIC_INIT(0);

static inline uint64_t conv_cache_tag(uint64_t glpn, uint32_t nr_parts, struct ssdparams *spp)
{
	return ((glpn / nr_parts) / spp->pgs_per_oneshotpg) * nr_parts + (glpn % nr_parts);
}

static inline struct line *get_line(struct conv_ftl *conv_ftl, struct ppa *ppa);
static inline uint64_t get_rmap_ent(struct conv_ftl *conv_ftl, struct ppa *ppa);
static inline struct ppa get_maptbl_ent(struct conv_ftl *conv_ftl, uint64_t lpn);

enum { LB_VFREE = 0, LB_VOPEN, LB_VCLOSED };
#define LB_NSTREAM ((TENANT_N + 1) * 2 + 1)
struct lb_vstream {
	uint32_t w;
	uint8_t n;
};
static uint32_t lb_nv;
static uint64_t *lb_vrmap;
static uint32_t *lb_viot;
static uint8_t *lb_vgen;
static uint8_t *lb_vlive;
static uint8_t *lb_vst;

#define LB_NIL 0xFFFFu
static uint8_t *lb_vpend;
static uint16_t *lb_vtk_next;
static uint16_t *lb_vtk_serve;
static uint16_t *lb_vwq_head;
static uint16_t *lb_vwq_tail;
static uint16_t *lb_vqn;
static uint8_t *lb_vused;
static uint32_t *lb_vfifo;
static uint32_t lb_vf_head, lb_vf_n;
static struct lb_vstream lb_vs[LB_NSTREAM];
static bool lb_bind_alloc;

static inline bool lb_on(void)
{
	return lb_nv != 0 && dcache_get() != NULL;
}

static inline uint32_t lb_vw(const struct ppa *p)
{
	return ((uint32_t)(p->g.blk & ~VPA_BLK_BIT) << 14) | (uint32_t)(p->g.pg >> 2);
}

static inline uint32_t lb_vslot(const struct ppa *p)
{
	return (uint32_t)(p->g.pg & (CE_SLOTS - 1));
}

static inline struct ppa lb_vppa(uint32_t w, uint32_t s)
{
	struct ppa p;

	p.ppa = 0;
	p.g.blk = VPA_BLK_BIT | (w >> 14);
	p.g.pg = ((w & 0x3FFFu) << 2) | s;
	p.g.pl = lb_vgen[w];
	return p;
}

static inline bool lb_vcur(const struct ppa *p)
{
	uint32_t w;

	w = lb_vw(p);
	return w < lb_nv && lb_vst[w] != LB_VFREE && p->g.pl == lb_vgen[w];
}

static inline uint64_t lb_vslot_lpn(const struct ppa *p, uint32_t s)
{
	if (!lb_vcur(p))
		return INVALID_LPN;
	return lb_vrmap[lb_vw(p) * CE_SLOTS + s];
}

static inline int lb_vrand_open(const struct ppa *p)
{
	struct lb_vstream *vs;

	vs = &lb_vs[LB_NSTREAM - 1];
	if (!lb_vcur(p) || !vs->n || vs->w != lb_vw(p))
		return 0;
	return CE_SLOTS - vs->n;
}

static void lb_vfree_maybe(uint32_t w)
{
	if (lb_vst[w] != LB_VCLOSED || lb_vlive[w])
		return;
	lb_vst[w] = LB_VFREE;
	lb_vfifo[(lb_vf_head + lb_vf_n) % lb_nv] = w;
	lb_vf_n++;
}

static void lb_vclose(uint32_t w)
{
	int k;

	if (lb_vst[w] != LB_VOPEN)
		return;
	for (k = 0; k < LB_NSTREAM; k++) {
		if (lb_vs[k].n && lb_vs[k].w == w)
			lb_vs[k].n = 0;
	}
	lb_vst[w] = LB_VCLOSED;
}

static uint32_t lb_valloc(uint32_t iot)
{
	uint32_t w;
	int s;

	NVMEV_ASSERT(lb_vf_n > 0 && "late bind: out of virtual wordline numbers");
	w = lb_vfifo[lb_vf_head];
	lb_vf_head = (lb_vf_head + 1) % lb_nv;
	lb_vf_n--;
	lb_vgen[w]++;
	lb_vst[w] = LB_VOPEN;
	lb_vlive[w] = 0;
	lb_viot[w] = iot;
	lb_vused[w] = 0;
	lb_vwq_head[w] = LB_NIL;
	lb_vwq_tail[w] = LB_NIL;
	lb_vqn[w] = 0;
	for (s = 0; s < CE_SLOTS; s++) {
		lb_vrmap[w * CE_SLOTS + s] = INVALID_LPN;
		lb_vpend[w * CE_SLOTS + s] = 0;
		lb_vtk_next[w * CE_SLOTS + s] = 0;
		lb_vtk_serve[w * CE_SLOTS + s] = 0;
	}
	return w;
}

static void lb_vset(const struct ppa *p, uint64_t lpn)
{
	uint32_t w;

	w = lb_vw(p);
	NVMEV_ASSERT(lb_vcur(p) && lb_vrmap[w * CE_SLOTS + lb_vslot(p)] == INVALID_LPN);
	lb_vrmap[w * CE_SLOTS + lb_vslot(p)] = lpn;
	lb_vlive[w]++;
	lb_vused[w] |= (uint8_t)(1u << lb_vslot(p));
}

static void lb_vinval(const struct ppa *p)
{
	uint32_t w;
	uint32_t i;

	w = lb_vw(p);
	i = w * CE_SLOTS + lb_vslot(p);
	if (!lb_vcur(p) || lb_vrmap[i] == INVALID_LPN) {
		return;
	}
	lb_vrmap[i] = INVALID_LPN;
	lb_vlive[w]--;
	lb_vfree_maybe(w);
}

static bool lb_in_dirty(struct conv_ftl *conv_ftl, uint64_t local_lpn, struct ppa *out)
{
	struct ppa m;

	m = get_maptbl_ent(conv_ftl, local_lpn);
	if (!ppa_is_virtual(&m) || !lb_vcur(&m))
		return false;
	if (lb_vrmap[lb_vw(&m) * CE_SLOTS + lb_vslot(&m)] != local_lpn)
		return false;
	*out = m;
	return true;
}

static bool lb_vbusy(const struct ppa *p, uint8_t mask)
{
	uint32_t i;
	uint32_t w;
	int s;

	if (!lb_vcur(p))
		return false;
	w = lb_vw(p);
	for (s = 0; s < CE_SLOTS; s++) {
		i = w * CE_SLOTS + s;
		if ((mask & (1u << s)) && (lb_vpend[i] || lb_vtk_next[i] != lb_vtk_serve[i]))
			return true;
	}
	return false;
}

static bool lb_vturn(const struct ppa *p, int s, uint16_t tk)
{
	uint32_t i;

	i = lb_vw(p) * CE_SLOTS + s;
	return !lb_vpend[i] && lb_vtk_serve[i] == tk;
}

static uint8_t lb_vlive_mask(const struct ppa *p)
{
	uint8_t m;
	uint32_t w;
	int s;

	m = 0;
	if (!lb_vcur(p))
		return 0;
	w = lb_vw(p);
	for (s = 0; s < CE_SLOTS; s++) {
		if (lb_vrmap[w * CE_SLOTS + s] != INVALID_LPN)
			m |= (uint8_t)(1u << s);
	}
	return m;
}

static void lb_vpark(const struct ppa *p, uint8_t mask)
{
	uint32_t w;
	int s;

	if (!lb_vcur(p))
		return;
	w = lb_vw(p);
	for (s = 0; s < CE_SLOTS; s++) {
		if (mask & (1u << s))
			lb_vpend[w * CE_SLOTS + s]++;
	}
}

static void lb_vunpark(const struct ppa *p, uint8_t mask)
{
	uint32_t w;
	int s;

	if (!lb_vcur(p))
		return;
	w = lb_vw(p);
	for (s = 0; s < CE_SLOTS; s++) {
		if ((mask & (1u << s)) && lb_vpend[w * CE_SLOTS + s])
			lb_vpend[w * CE_SLOTS + s]--;
	}
}

uint16_t conv_lb_held(uint64_t pgkey)
{
	struct ppa p;

	p.ppa = pgkey;
	if (!ppa_is_virtual(&p) || !lb_vcur(&p))
		return 0;
	return lb_vqn[lb_vw(&p)];
}

static int conv_rand_wl_open(struct conv_ftl *conv_ftl, const struct ppa *ppa)
{
	uint32_t oneshot;
	const struct write_pointer *wp;
	int k;

	if (ppa_is_virtual(ppa))
		return lb_vrand_open(ppa);
	oneshot = conv_ftl->ssd->sp.pgs_per_oneshotpg;
	for (k = 0; k < 2; k++) {
		wp = k ? &conv_ftl->wp_rand_slc : &conv_ftl->wp_rand_tlc;
		if (!wp->curline || (wp->pg % oneshot) == 0)
			continue;
		if (wp->blk == ppa->g.blk && wp->ch == ppa->g.ch && wp->lun == ppa->g.lun &&
		    wp->pl == ppa->g.pl && (wp->pg / oneshot) == (ppa->g.pg / oneshot))
			return (int)(oneshot - (wp->pg % oneshot));
	}
	return 0;
}

static void conv_ce_page(struct conv_ftl *conv_ftl, struct ppa *ppa, uint64_t part_lpn,
			 uint32_t nr_parts, struct ce_page *pg)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct ppa p;
	uint64_t lpn;
	uint32_t base;
	int s;

	pg->pgkey = ce_pgkey(ppa);
	pg->tag = 0;
	pg->live = 0;
	pg->chunk = 0;
	pg->rnd = 0;
	pg->rpend = 0;
	pg->wl_free = (uint8_t)conv_rand_wl_open(conv_ftl, ppa);
	pg->wl_open = pg->wl_free ? 1 : 0;
	base = ppa->g.pg - (ppa->g.pg % CE_SLOTS);
	for (s = 0; s < CE_SLOTS; s++) {
		pg->lba[s] = CE_LBA_NONE;

		if (ppa_is_virtual(ppa)) {
			lpn = lb_vslot_lpn(ppa, s);
			if (lpn == INVALID_LPN)
				continue;
			lpn = lpn * nr_parts + (part_lpn % nr_parts);
			pg->lba[s] = lpn;
			pg->live |= 1u << s;
			pg->tag = conv_cache_tag(lpn, nr_parts, spp);
			continue;
		}
		p = *ppa;
		p.g.pg = base + s;
		if (get_pg(conv_ftl->ssd, &p)->status != PG_VALID)
			continue;
		lpn = get_rmap_ent(conv_ftl, &p);
		if (lpn == INVALID_LPN)
			continue;
		lpn = lpn * nr_parts + (part_lpn % nr_parts);
		pg->lba[s] = lpn;
		pg->live |= 1u << s;
		pg->tag = conv_cache_tag(lpn, nr_parts, spp);
	}
}

static struct conv_ftl *g_conv_ftl0;
static void raw_m6_note_completed_nand(const u16 completed_tn[TENANT_N + 1],
				      u32 host_seq);
static void raw_m7_note_nand_issue(u64 chunk_lpn, u32 xfer, u32 nr_parts,
				   struct ssdparams *spp, u64 now, u32 host_seq,
				   const u16 next_nand_tn[TENANT_N + 1]);

enum m0dbg_hook_id {
	M0H_WRITE = 0,
	M0H_READ,
	M0H_NAND_ISSUE,
	M0H_NAND_DONE,
	M0H_RESET,
	M0H_SWEEP,
	M0H_M7_ISSUE,
	M0H_PREDICT,
	M0H_M6_DONE,
	M0H_WAIT,
	M0H_SHADOW,
	M0H_RPLACE,
	M0H_N
};

#define DIQ_POOL  16384
#define DIQ_NIL   0xFFFFu
struct diq_ent {
	struct nand_cmd  cmd;
	struct ppa       ppa;
	struct conv_ftl *ftl;
	uint32_t         dep;
	uint16_t         next;
	uint8_t          rd_tn;

	uint8_t          mig;

	uint16_t         raw_m6_completed_tn[TENANT_N + 1];

	uint16_t         raw_m7_next_nand_tn[TENANT_N + 1];
	u64              raw_chunk_lpn;
	u32              raw_nr_parts;

	u8               kind;
	u8               chunk_mask;

	u8               die_tag;
	u64              wr_load_ns;
	u64              wr_park_at;

	u32              wr_own;
	u16              wr_tk;
};
enum { DIQ_K_READ = 0, DIQ_K_RELOOK = 1, DIQ_K_WRITE = 2 };
static struct diq_ent diq_pool[DIQ_POOL];
static uint16_t diq_free_head = DIQ_NIL;
static uint16_t diq_head[1 << 16];
static bool     diq_ready;
u64 diq_park, diq_resume_n, diq_full, diq_stale, diq_live, diq_hwm;

static void diq_die_untag(struct diq_ent *d)
{
	if (!d->die_tag)
		return;
	d->die_tag = 0;
}

static void diq_die_tag(struct diq_ent *d, uint32_t frame)
{
	int die;

	diq_die_untag(d);
	die = dcache_frame_die(frame);
	if (die < 0 || die >= TETRIS_NR_DIES) {
		return;
	}
	d->die_tag = (u8)(die + 1);
}

static uint16_t diq_full_head = DIQ_NIL;
static uint16_t diq_full_tail = DIQ_NIL;
static uint16_t diq_full_waking = DIQ_NIL;
u64 diq_full_live;

struct diq_dep_adv diq_dep_adv[1 << 16];
u64 diq_dep_pub, diq_dep_clr;

u64 diq_delay_n, diq_delay_sum, diq_delay_max, diq_nodelay;

u64 diq_mig_n, diq_mig_sum, diq_last_done;
u64 diq_mig_sum_tn[TENANT_N + 1];
static u64 diq_mig_sum_prev, diq_mig_t_prev;
static u64 diq_mig_n_prev, diq_all_sum_prev, diq_all_n_prev;

static u64 diq_dw_sum_prev, diq_dw_n_prev, diq_dw_t_prev;

u64 diq_delay_n_tn[TENANT_N + 1];

u64 diq_delay_hist_tn[TENANT_N + 1][DIQ_HIST_BKTS];
u64 diq_delay_hsum_tn[TENANT_N + 1][DIQ_HIST_BKTS];

static u64 diq_hist_mig[DIQ_HIST_BKTS], diq_hsum_mig[DIQ_HIST_BKTS];
static u64 diq_hist_oth[DIQ_HIST_BKTS], diq_hsum_oth[DIQ_HIST_BKTS];

static inline int diq_hist_bkt(u64 ns)
{
	int b;

	b = ns ? ilog2(ns) : 0;
	if (b >= DIQ_HIST_BKTS)
		b = DIQ_HIST_BKTS - 1;
	return b;
}

static void diq_init(void)
{
	int i;

	for (i = 0; i < (1 << 16); i++)
		diq_head[i] = DIQ_NIL;
	for (i = 0; i < DIQ_POOL - 1; i++)
		diq_pool[i].next = (uint16_t)(i + 1);
	diq_pool[DIQ_POOL - 1].next = DIQ_NIL;
	diq_free_head = 0;
	diq_ready = true;
}

static uint16_t diq_alloc(void)
{
	uint16_t i;

	if (unlikely(!diq_ready))
		diq_init();
	i = diq_free_head;
	if (i == DIQ_NIL)
		return DIQ_NIL;
	diq_free_head = diq_pool[i].next;
	if (++diq_live > diq_hwm)
		diq_hwm = diq_live;
	return i;
}

static void diq_free(uint16_t i)
{
	diq_die_untag(&diq_pool[i]);
	diq_pool[i].kind = DIQ_K_READ;
	diq_pool[i].wr_own = 0;
	diq_pool[i].next = diq_free_head;
	diq_free_head = i;
	diq_live--;
}

static void diq_full_push(uint16_t sl, bool at_head)
{
	struct diq_ent *d;

	d = &diq_pool[sl];
	diq_die_untag(d);
	d->dep = CE_WAIT_FULL;
	if (at_head) {
		d->next = diq_full_head;
		diq_full_head = sl;
		if (diq_full_tail == DIQ_NIL)
			diq_full_tail = sl;
	} else {
		d->next = DIQ_NIL;
		if (diq_full_tail == DIQ_NIL)
			diq_full_head = sl;
		else
			diq_pool[diq_full_tail].next = sl;
		diq_full_tail = sl;
	}
	diq_full_live++;
}

static void diq_repark(uint16_t sl, uint32_t frame)
{
	struct diq_ent *d;

	d = &diq_pool[sl];
	if (frame == CE_WAIT_FULL) {
		diq_full_push(sl, false);
		return;
	}
	d->dep = frame;
	d->next = diq_head[frame & 0xFFFFu];
	diq_head[frame & 0xFFFFu] = sl;
	diq_die_tag(d, frame);
}

#define LB_WQ_DEP 0xFFFFFFFEu
static uint16_t lb_rdy_head = DIQ_NIL;
static uint16_t lb_rdy_tail = DIQ_NIL;
static u64 lb_wk_t[DIQ_POOL];
static bool lb_draining;

static void lb_wq_push(uint16_t sl, uint32_t w)
{
	struct diq_ent *d;

	d = &diq_pool[sl];
	diq_die_untag(d);
	d->dep = LB_WQ_DEP;
	d->next = DIQ_NIL;
	if (lb_vwq_tail[w] == DIQ_NIL)
		lb_vwq_head[w] = sl;
	else
		diq_pool[lb_vwq_tail[w]].next = sl;
	lb_vwq_tail[w] = sl;
}

static void lb_wake(uint32_t w, u64 t)
{
	uint16_t sl;

	if (w >= lb_nv || lb_vwq_head[w] == DIQ_NIL)
		return;
	for (sl = lb_vwq_head[w]; sl != DIQ_NIL; sl = diq_pool[sl].next)
		lb_wk_t[sl] = t;
	if (lb_rdy_tail == DIQ_NIL)
		lb_rdy_head = lb_vwq_head[w];
	else
		diq_pool[lb_rdy_tail].next = lb_vwq_head[w];
	lb_rdy_tail = lb_vwq_tail[w];
	lb_vwq_head[w] = DIQ_NIL;
	lb_vwq_tail[w] = DIQ_NIL;
}

static void conv_diq_relook(uint16_t self, uint64_t done);
static void conv_diq_write(uint16_t self, uint64_t done);

static void lb_drain(void)
{
	uint16_t sl;

	if (lb_draining || !lb_nv)
		return;
	lb_draining = true;
	while (lb_rdy_head != DIQ_NIL) {
		sl = lb_rdy_head;
		lb_rdy_head = diq_pool[sl].next;
		if (lb_rdy_head == DIQ_NIL)
			lb_rdy_tail = DIQ_NIL;
		diq_pool[sl].next = DIQ_NIL;
		if (diq_pool[sl].kind == DIQ_K_RELOOK)
			conv_diq_relook(sl, lb_wk_t[sl]);
		else
			conv_diq_write(sl, lb_wk_t[sl]);
	}
	lb_draining = false;
}

static uint16_t diq_full_pop(void)
{
	uint16_t sl;

	sl = diq_full_head;
	if (sl == DIQ_NIL)
		return DIQ_NIL;
	diq_full_head = diq_pool[sl].next;
	if (diq_full_head == DIQ_NIL)
		diq_full_tail = DIQ_NIL;
	diq_pool[sl].next = DIQ_NIL;
	diq_full_live--;
	return sl;
}

static uint8_t conv_rand_fifo_pend(uint64_t pgkey)
{
	struct diq_ent *d;
	uint16_t sl;
	uint32_t n;
	uint8_t m;

	m = 0;
	n = 0;
	sl = diq_full_head;
	while (sl != DIQ_NIL && n <= DIQ_POOL) {
		d = &diq_pool[sl];
		if (d->kind == DIQ_K_WRITE && (d->chunk_mask & 0x80) && ce_pgkey(&d->ppa) == pgkey)
			m |= d->chunk_mask & CE_ALL_MASK;
		sl = d->next;
		n++;
	}
	return m;
}

#define DIQ_CMD_MAX   ((1u << MDTS) + 1)
#define DIQ_BP_ON     (2 * DIQ_CMD_MAX)
#define DIQ_BP_OFF    (4 * DIQ_CMD_MAX)
static int diq_bp_on;

bool nvmev_diq_room(void)
{
	return (u64)DIQ_POOL - diq_live >= DIQ_CMD_MAX;
}

int nvmev_diq_bp_eval(void)
{
	u64 free_n;

	free_n = (u64)DIQ_POOL - diq_live;
	if (!diq_bp_on && free_n < DIQ_BP_ON) {
		diq_bp_on = 1;
	} else if (diq_bp_on && free_n >= DIQ_BP_OFF) {
		diq_bp_on = 0;
	}
	return diq_bp_on;
}

static void conv_diq_relook(uint16_t self, uint64_t done);
static void conv_diq_write(uint16_t self, uint64_t done);

void nvmev_diq_resume(uint32_t frame, uint64_t done)
{
	uint16_t *pp;
	uint16_t later;
	uint16_t sl;

	if (unlikely(!diq_ready))
		return;
	later = DIQ_NIL;
	pp = &diq_head[frame & 0xFFFFu];
	while (*pp != DIQ_NIL) {
		struct diq_ent *d = &diq_pool[*pp];
		uint16_t self = *pp;
		uint32_t owner;
		uint64_t t;

		if (d->dep != frame) {
			diq_stale++;
			pp = &d->next;
			continue;
		}
		*pp = d->next;
		diq_die_untag(d);

		if (d->kind != DIQ_K_READ) {
			d->next = later;
			later = self;
			continue;
		}

		{
			uint64_t was = d->cmd.stime;

			if (done > was) {
				uint64_t dl = done - was;
				int _hb;

				diq_delay_n++;
				diq_delay_sum += dl;

				ssd_optrace_record_diq(d->cmd.host_seq, was, done, d->rd_tn, d->mig);

				if (d->mig) {
					diq_mig_n++;
					diq_mig_sum += dl;
					if (d->rd_tn <= TENANT_N)
						diq_mig_sum_tn[d->rd_tn] += dl;
				}
				diq_last_done = done;

				_hb = diq_hist_bkt(dl);
				if (d->mig) {
					diq_hist_mig[_hb]++;
					diq_hsum_mig[_hb] += dl;
				} else {
					diq_hist_oth[_hb]++;
					diq_hsum_oth[_hb] += dl;
				}
				if (dl > diq_delay_max)
					diq_delay_max = dl;
				if (d->rd_tn <= TENANT_N) {
					diq_delay_n_tn[d->rd_tn]++;
					diq_delay_hist_tn[d->rd_tn][_hb]++;
					diq_delay_hsum_tn[d->rd_tn][_hb] += dl;
				}
			} else {
				diq_nodelay++;
			}
		}
		d->cmd.stime = done;
		owner = d->cmd.owner;
		raw_m6_note_completed_nand(d->raw_m6_completed_tn, d->cmd.host_seq);
		raw_m7_note_nand_issue(d->raw_chunk_lpn, d->cmd.xfer_size,
				       d->raw_nr_parts, &d->ftl->ssd->sp, d->cmd.stime,
				       d->cmd.host_seq, d->raw_m7_next_nand_tn);
		t = ssd_advance_nand(d->ftl->ssd, &d->cmd);

		if (!d->cmd.is_small_read)
			tetris_lr_mark(&d->ppa, t, d->rd_tn);
		else
			tetris_sr_mark(&d->ppa, t);
		{
			struct diq_dep_adv *ad = &diq_dep_adv[frame & 0xFFFFu];

			if (ad->frame == d->dep && ad->n)
				ad->n--;
			if (!ad->n)
				ad->frame = 0;
			diq_dep_clr++;
		}
		diq_free(self);
		diq_resume_n++;
		nvmev_p4_commit_token(owner);
	}

	while (later != DIQ_NIL) {
		sl = later;
		later = diq_pool[sl].next;
		if (diq_pool[sl].kind == DIQ_K_RELOOK)
			conv_diq_relook(sl, done);
		else
			conv_diq_write(sl, done);
	}
	lb_drain();
}

void nvmev_diq_full_wake(uint64_t done)
{
	struct cache_model *dc;
	uint64_t left;
	uint16_t saved;
	uint16_t sl;
	bool stop;

	dc = dcache_get();
	if (unlikely(!diq_ready) || !dc)
		return;
	stop = false;

	left = diq_full_live;
	while (!stop && left-- > 0 && diq_full_head != DIQ_NIL) {
		sl = diq_full_pop();
		saved = diq_full_waking;
		diq_full_waking = sl;
		if (diq_pool[sl].kind == DIQ_K_RELOOK)
			conv_diq_relook(sl, done);
		else
			conv_diq_write(sl, done);
		diq_full_waking = saved;
		if (diq_full_head == sl)
			stop = true;
	}
	lb_drain();
}

static void diq_dump_node(const char *why, const char *where, uint16_t sl)
{
	struct diq_ent *d;
	char b1[256];
	char b2[256];

	d = &diq_pool[sl];
	dcache_frame_describe(d->dep, b1, sizeof(b1));
	if (d->wr_own)
		dcache_frame_describe(d->wr_own, b2, sizeof(b2));
	else
		b2[0] = '\0';
}

void nvmev_diq_dump(const char *why)
{
	uint32_t i;
	uint32_t n_chain;
	uint32_t n_fifo;
	uint32_t shown;
	uint16_t sl;

	if (unlikely(!diq_ready)) {
		return;
	}
	n_chain = 0;
	shown = 0;
	for (i = 0; i < (1u << 16) && n_chain <= DIQ_POOL; i++) {
		sl = diq_head[i];
		while (sl != DIQ_NIL && n_chain <= DIQ_POOL) {
			n_chain++;
			if (shown < 32) {
				diq_dump_node(why, "chain", sl);
				shown++;
			}
			sl = diq_pool[sl].next;
		}
	}
	n_fifo = 0;
	sl = diq_full_head;
	while (sl != DIQ_NIL && n_fifo <= DIQ_POOL) {
		n_fifo++;
		if (shown < 32) {
			diq_dump_node(why, "fifo", sl);
			shown++;
		}
		sl = diq_pool[sl].next;
	}
	dcache_dump_census(why);
}

void nvmev_diq_mig_window4(u64 *d_mig, u64 *n_mig, u64 *d_oth, u64 *n_oth, u64 *t_ns)
{
	u64 dm, dt, nm, nt;

	if (!diq_mig_t_prev) {
		diq_mig_sum_prev = diq_mig_sum;
		diq_mig_n_prev = diq_mig_n;
		diq_all_sum_prev = diq_delay_sum;
		diq_all_n_prev = diq_delay_n;
		diq_mig_t_prev = diq_last_done;
		*d_mig = *n_mig = *d_oth = *n_oth = *t_ns = 0;
		return;
	}
	dm = diq_mig_sum   > diq_mig_sum_prev ? diq_mig_sum   - diq_mig_sum_prev : 0;
	nm = diq_mig_n     > diq_mig_n_prev   ? diq_mig_n     - diq_mig_n_prev   : 0;
	dt = diq_delay_sum > diq_all_sum_prev ? diq_delay_sum - diq_all_sum_prev : 0;
	nt = diq_delay_n   > diq_all_n_prev   ? diq_delay_n   - diq_all_n_prev   : 0;
	*d_mig = dm;
	*n_mig = nm;
	*d_oth = dt > dm ? dt - dm : 0;
	*n_oth = nt > nm ? nt - nm : 0;
	*t_ns  = diq_last_done > diq_mig_t_prev ? diq_last_done - diq_mig_t_prev : 0;
	diq_mig_sum_prev = diq_mig_sum;
	diq_mig_n_prev   = diq_mig_n;
	diq_all_sum_prev = diq_delay_sum;
	diq_all_n_prev   = diq_delay_n;
	diq_mig_t_prev   = diq_last_done;
}

void nvmev_diq_dw_sample(u64 *d_tot, u64 *n_tot, u64 *t_ns)
{
	u64 ds, ns;

	if (!diq_dw_t_prev) {
		diq_dw_sum_prev = diq_delay_sum;
		diq_dw_n_prev = diq_delay_n;
		diq_dw_t_prev = diq_last_done;
		*d_tot = *n_tot = *t_ns = 0;
		return;
	}
	ds = diq_delay_sum > diq_dw_sum_prev ? diq_delay_sum - diq_dw_sum_prev : 0;
	ns = diq_delay_n   > diq_dw_n_prev   ? diq_delay_n   - diq_dw_n_prev   : 0;
	*d_tot = ds;
	*n_tot = ns;
	*t_ns  = diq_last_done > diq_dw_t_prev ? diq_last_done - diq_dw_t_prev : 0;
	diq_dw_sum_prev = diq_delay_sum;
	diq_dw_n_prev   = diq_delay_n;
	diq_dw_t_prev   = diq_last_done;
}

void nvmev_diq_mig_reset(void)
{
	diq_mig_n = diq_mig_sum = diq_last_done = 0;
	memset(diq_mig_sum_tn, 0, sizeof(diq_mig_sum_tn));

	memset(diq_hist_mig, 0, sizeof(diq_hist_mig));
	memset(diq_hsum_mig, 0, sizeof(diq_hsum_mig));
	memset(diq_hist_oth, 0, sizeof(diq_hist_oth));
	memset(diq_hsum_oth, 0, sizeof(diq_hsum_oth));
	diq_mig_sum_prev = diq_mig_t_prev = 0;
	diq_mig_n_prev = diq_all_sum_prev = diq_all_n_prev = 0;

	diq_dw_sum_prev = diq_dw_n_prev = diq_dw_t_prev = 0;
	dw_n = dw_sum = dw_sq = 0;
	dw_min = ~0ULL;
	tw_sum = 0;
	tw_min = ~0ULL;
}

enum m7_tn_counter {
	M7_TN_ARM = 0,
	M7_TN_NEXT_READ,
	M7_TN_NEXT_CLEAN,
	M7_TN_NEXT_NAND,
	M7_TN_NEXT_OTHER,
	M7_TN_OVERWRITE,
	M7_TN_DISPLACED,
	M7_TN_UNLOAD_LIVE,
	M7_TN_N,
};

enum m7_arm_loss {
	M7_LOSS_TAG = 0,
	M7_LOSS_SEEN,
	M7_LOSS_SEQ,
	M7_LOSS_OVERWRITE,
	M7_LOSS_COLLISION,
	M7_LOSS_N
};

static struct raw_lpn_meta *raw_meta;

static u8  *rng_score;

static u8  *rng_reuse;

static u32 *rng_raw, *rng_tot;

enum raw_cpu_kind { RAW_CPU_LABEL = 0, RAW_CPU_NAND, RAW_CPU_WAIT, RAW_CPU_SHADOW, RAW_CPU_N };

enum raw_shadow_action {
	RAW_SH_GC = 0,
	RAW_SH_MIGRATE,
	RAW_SH_REDIRECT,
	RAW_SH_PAUSE,
	RAW_SH_N
};

enum reclaim_selector_action {
	RSEL_VPC0 = 0,
	RSEL_SLC_GC,
	RSEL_MIGRATE,
	RSEL_REDIRECT,
	RSEL_FALLBACK_GC,
	RSEL_ACTION_N,
	RSEL_NONE = -1,
};

static void raw_meta_exit(void)
{
	if (!raw_meta)
		return;
	vfree(raw_meta);
	raw_meta = NULL;
	vfree(rng_raw); vfree(rng_tot);
	rng_raw = rng_tot = NULL;
	vfree(rng_score);
	rng_score = NULL;
	vfree(rng_reuse);
	rng_reuse = NULL;
}

static void raw_m7_note_nand_issue(u64 chunk_lpn, u32 xfer, u32 nr_parts,
				   struct ssdparams *spp, u64 now, u32 host_seq,
				   const u16 next_nand_tn[TENANT_N + 1])
{
	return;
}

static void raw_m6_note_completed_nand(const u16 completed_tn[TENANT_N + 1],
				      u32 host_seq)
{
	return;
}

static struct conv_ftl *raw_find_ftl(struct ssd *ssd, u32 *part)
{
	u32 i;

	if (!g_conv_ftl0)
		return NULL;
	for (i = 0; i < SSD_PARTITIONS; i++)
		if (g_conv_ftl0[i].ssd == ssd) {
			*part = i;
			return &g_conv_ftl0[i];
		}
	return NULL;
}

static inline u8 lpn_tenant_get(struct conv_ftl *conv_ftl, u64 lpn);

static uint64_t conv_park_relook(struct conv_ftl *conv_ftl, struct nand_cmd *srd, struct ppa *ppa,
				 uint8_t rd_tn, uint64_t chunk_lpn, uint32_t nr_parts, uint8_t chunk_mask,
				 uint32_t frame);
static uint64_t conv_issue_read_chunk(struct conv_ftl *conv_ftl, struct nand_cmd *tpl,
				      struct ppa *ppa, uint32_t xfer, uint64_t chunk_lpn,
				      uint32_t nr_parts, uint8_t chunk_mask);

static uint64_t conv_read_vpa(struct conv_ftl *conv_ftl, struct nand_cmd *tpl, struct ppa *ppa, uint32_t xfer,
			      uint64_t chunk_lpn, uint32_t nr_parts, uint8_t chunk_mask)
{
	struct nand_cmd srd = *tpl;
	struct ce_page pg;
	struct ppa cur;
	uint64_t stall;
	uint64_t done;
	uint32_t lwait;
	uint8_t rd_tn;
	int rc;

	rd_tn = lpn_tenant_get(conv_ftl, chunk_lpn);
	if (rd_tn > TENANT_N)
		rd_tn = 0;
	conv_ce_page(conv_ftl, ppa, chunk_lpn, nr_parts, &pg);
	pg.chunk = chunk_mask;
	if (!(pg.live & chunk_mask)) {
		cur = get_maptbl_ent(conv_ftl, chunk_lpn / nr_parts);
		if (cur.ppa != ppa->ppa) {
			if (cur.ppa == UNMAPPED_PPA)
				return tpl->stime;

			return conv_issue_read_chunk(conv_ftl, tpl, &cur, xfer, chunk_lpn, nr_parts, chunk_mask);
		}
	}
	srd.xfer_size = xfer;
	srd.ppa = ppa;
	srd.owner = nvmev_cur_cmd;
	if (lb_vbusy(ppa, chunk_mask)) {
		return conv_park_relook(conv_ftl, &srd, ppa, rd_tn, chunk_lpn, nr_parts, chunk_mask, LB_WQ_DEP);
	}
	stall = 0;
	rc = cache_read_vpg(dcache_get(), &pg, srd.stime, &stall, tpl->is_small_read, rd_tn, &lwait);
	if (rc == CACHE_WAIT)
		return conv_park_relook(conv_ftl, &srd, ppa, rd_tn, chunk_lpn, nr_parts, chunk_mask, lwait);
	if (rc == CACHE_MISS) {
		printk_ratelimited(KERN_ERR "NVMeVirt: late bind read orphan lpn=%llu chunk=%x -- no pending write, no entry\n",
				   chunk_lpn, chunk_mask);
		return conv_park_relook(conv_ftl, &srd, ppa, rd_tn, chunk_lpn, nr_parts, chunk_mask, CE_WAIT_FULL);
	}
	done = ssd_advance_pcie(conv_ftl->ssd, srd.stime + stall + cache_hit_lat_ns, xfer);
	ssd_optrace_record_hit(srd.host_seq, srd.stime, done);
	return done;
}

static uint64_t conv_issue_read_chunk(struct conv_ftl *conv_ftl, struct nand_cmd *tpl,
				      struct ppa *ppa, uint32_t xfer, uint64_t chunk_lpn,
				      uint32_t nr_parts, uint8_t chunk_mask)
{
	struct nand_cmd srd = *tpl;
	struct cache_model *dc = dcache_get();
	u16 raw_m6_completed_tn[TENANT_N + 1] = { 0 };
	u16 raw_m7_next_nand_tn[TENANT_N + 1] = { 0 };

	uint8_t rd_tn;

	if (ppa_is_virtual(ppa))
		return conv_read_vpa(conv_ftl, tpl, ppa, xfer, chunk_lpn, nr_parts, chunk_mask);

	rd_tn = lpn_tenant_get(conv_ftl, chunk_lpn);
	if (rd_tn > TENANT_N)
		rd_tn = 0;
	srd.xfer_size = xfer;
	srd.ppa = ppa;

	srd.owner = nvmev_cur_cmd;

	if (dc) {
		uint64_t stall = 0;

		bool admit = cache_admit_read(tpl->is_small_read, tpl->size_small);

		uint32_t dep = 0;
		int cache_rc;
		uint32_t fill;
		uint32_t lwait;
		struct ce_page pg;

		conv_ce_page(conv_ftl, ppa, chunk_lpn, nr_parts, &pg);
		pg.chunk = chunk_mask;
		cache_rc = cache_read_pg(dc, &pg, srd.stime, admit, &stall, tpl->is_small_read, rd_tn,
					 &dep, &fill, &lwait);
		if (cache_rc == CACHE_WAIT)
			return conv_park_relook(conv_ftl, &srd, ppa, rd_tn, chunk_lpn, nr_parts,
						chunk_mask, lwait);
		srd.cache_fill = fill;
		if (cache_rc != CACHE_MISS) {
			uint64_t done = ssd_advance_pcie(conv_ftl->ssd,
							 srd.stime + stall + cache_hit_lat_ns, xfer);
			ssd_optrace_record_hit(srd.host_seq, srd.stime, done);

			return done;
		}

		if (dep && srd.owner) {
			uint16_t sl = diq_alloc();

			if (sl != DIQ_NIL) {
				struct diq_ent *d = &diq_pool[sl];

				d->cmd = srd;
				d->ppa = *ppa;
				d->cmd.ppa = &d->ppa;
				d->ftl = conv_ftl;
				d->rd_tn = rd_tn;

				d->mig = nvmev_diq_mig_hint ? 1 : 0;
				memcpy(d->raw_m6_completed_tn, raw_m6_completed_tn,
				       sizeof(d->raw_m6_completed_tn));
				memcpy(d->raw_m7_next_nand_tn, raw_m7_next_nand_tn,
				       sizeof(d->raw_m7_next_nand_tn));
				d->raw_chunk_lpn = chunk_lpn;
				d->raw_nr_parts = nr_parts;
				d->dep = dep;
				d->next = diq_head[dep & 0xFFFFu];
				diq_head[dep & 0xFFFFu] = sl;
				diq_die_tag(d, dep);
				diq_park++;
				{
					struct diq_dep_adv *ad = &diq_dep_adv[dep & 0xFFFFu];

					if (ad->frame != dep) {
						ad->frame = dep;
						ad->n = 0;
					}
					if (!ad->n)
						ad->t0 = srd.stime;
					ad->n++;
					diq_dep_pub++;
				}
				nvmev_note_issued(srd.owner);
				return 0;
			}

			diq_full++;
			NVMEV_ASSERT(0 && "DIQ exhausted: no slot for a host read that got a dirty victim");
		}
		srd.stime += stall;
		tetris_sd_cache[tpl->is_small_read ? 0 : 1] += stall;
		if (stall > tetris_sd_cache_max[tpl->is_small_read ? 0 : 1]) tetris_sd_cache_max[tpl->is_small_read ? 0 : 1] = stall;
		if (!admit) {
			uint64_t sstall = 0, done;
			int slot = cache_stage_acquire(dc, srd.stime, ppa_die_id(ppa),
							       srd.stream_id, &sstall);

			int shslot = cache_shadow_acquire(dc, srd.stime);

			srd.stime += sstall;
			tetris_sd_cache[tpl->is_small_read ? 0 : 1] += sstall;
			if (sstall > tetris_sd_cache_max[tpl->is_small_read ? 0 : 1]) tetris_sd_cache_max[tpl->is_small_read ? 0 : 1] = sstall;
			raw_m6_note_completed_nand(raw_m6_completed_tn, srd.host_seq);
			raw_m7_note_nand_issue(chunk_lpn, xfer, nr_parts, &conv_ftl->ssd->sp,
					       srd.stime, srd.host_seq, raw_m7_next_nand_tn);
			done = ssd_advance_nand(conv_ftl->ssd, &srd);
			cache_stage_commit(dc, slot, done);
			if (shslot >= 0)
				cache_stage_commit(dc, shslot, done);
			if (!srd.is_small_read)
				tetris_lr_mark(ppa, done, rd_tn);
			else
				tetris_sr_mark(ppa, done);
			return done;
		}
	}
	{
		uint64_t done;

		int shslot = srd.is_small_read ? -1 :
					     cache_shadow_acquire(dcache_get(), srd.stime);

		raw_m6_note_completed_nand(raw_m6_completed_tn, srd.host_seq);
		raw_m7_note_nand_issue(chunk_lpn, xfer, nr_parts, &conv_ftl->ssd->sp,
				       srd.stime, srd.host_seq, raw_m7_next_nand_tn);
		done = ssd_advance_nand(conv_ftl->ssd, &srd);
		if (shslot >= 0)
			cache_stage_commit(dcache_get(), shslot, done);

		if (!srd.is_small_read)
			tetris_lr_mark(ppa, done, rd_tn);
		else
			tetris_sr_mark(ppa, done);
		return done;
	}
}

static uint64_t conv_park_relook(struct conv_ftl *conv_ftl, struct nand_cmd *srd, struct ppa *ppa,
				 uint8_t rd_tn, uint64_t chunk_lpn, uint32_t nr_parts, uint8_t chunk_mask,
				 uint32_t frame)
{
	struct diq_ent *d;
	uint16_t sl;

	sl = srd->owner ? diq_alloc() : DIQ_NIL;
	if (sl == DIQ_NIL) {
		NVMEV_ASSERT(0 && "DIQ exhausted: no slot for a host read that met a LOADING entry");
		return 0;
	}
	d = &diq_pool[sl];
	d->cmd = *srd;
	d->ppa = *ppa;
	d->cmd.ppa = &d->ppa;
	d->ftl = conv_ftl;
	d->rd_tn = rd_tn;
	d->mig = 0;
	memset(d->raw_m6_completed_tn, 0, sizeof(d->raw_m6_completed_tn));
	memset(d->raw_m7_next_nand_tn, 0, sizeof(d->raw_m7_next_nand_tn));
	d->raw_chunk_lpn = chunk_lpn;
	d->raw_nr_parts = nr_parts;
	d->kind = DIQ_K_RELOOK;
	d->chunk_mask = chunk_mask;
	if (frame == LB_WQ_DEP) {
		lb_wq_push(sl, lb_vw(ppa));
	} else if (frame == CE_WAIT_FULL) {
		diq_full_push(sl, false);
	} else {
		d->dep = frame;
		d->next = diq_head[frame & 0xFFFFu];
		diq_head[frame & 0xFFFFu] = sl;
		diq_die_tag(d, frame);
	}
	nvmev_note_issued(srd->owner);
	return 0;
}

static void conv_diq_relook(uint16_t self, uint64_t done)
{
	struct diq_ent *d;
	struct diq_dep_adv *ad;
	struct ce_page pg;
	uint64_t now;
	uint64_t stall;
	uint64_t t;
	uint32_t owner;
	uint32_t saved;
	uint32_t dep;
	uint32_t fill;
	uint32_t lwait;
	struct ppa cur;
	bool admit;
	bool vpa;
	int rc;

	d = &diq_pool[self];
	owner = d->cmd.owner;

	now = (done > d->cmd.stime) ? done : d->cmd.stime;
	stall = 0;
	dep = 0;
	fill = 0;
	admit = cache_admit_read(d->cmd.is_small_read, d->cmd.size_small);
	vpa = ppa_is_virtual(&d->ppa);
	conv_ce_page(d->ftl, &d->ppa, d->raw_chunk_lpn, d->raw_nr_parts, &pg);
	pg.chunk = d->chunk_mask;
	if (vpa && !(pg.live & pg.chunk)) {
		cur = get_maptbl_ent(d->ftl, d->raw_chunk_lpn / d->raw_nr_parts);
		if (cur.ppa != UNMAPPED_PPA && !ppa_is_virtual(&cur)) {
			d->ppa = cur;
			vpa = false;
			conv_ce_page(d->ftl, &d->ppa, d->raw_chunk_lpn, d->raw_nr_parts, &pg);
			pg.chunk = d->chunk_mask;
		}
	}
	if (vpa && lb_vbusy(&d->ppa, pg.chunk)) {
		lb_wq_push(self, lb_vw(&d->ppa));
		return;
	}
	saved = nvmev_cur_cmd;
	nvmev_cur_cmd = owner;
	if (vpa)
		rc = cache_read_vpg(dcache_get(), &pg, now, &stall, d->cmd.is_small_read, d->rd_tn, &lwait);
	else
		rc = cache_read_pg(dcache_get(), &pg, now, admit, &stall, d->cmd.is_small_read, d->rd_tn,
				   &dep, &fill, &lwait);
	nvmev_cur_cmd = saved;
	if (vpa && rc == CACHE_MISS) {
		printk_ratelimited(KERN_ERR "NVMeVirt: late bind relook orphan lpn=%llu chunk=%x\n",
				   d->raw_chunk_lpn, d->chunk_mask);
		diq_repark(self, CE_WAIT_FULL);
		return;
	}
	if (rc == CACHE_WAIT) {
		if (lwait == CE_WAIT_FULL) {
			if (self == diq_full_waking) {
				diq_full_push(self, true);
			} else {
				diq_full_push(self, false);
			}
		} else {
			d->dep = lwait;
			d->next = diq_head[lwait & 0xFFFFu];
			diq_head[lwait & 0xFFFFu] = self;
			diq_die_tag(d, lwait);
		}
		return;
	}
	if (rc != CACHE_MISS) {
		t = ssd_advance_pcie(d->ftl->ssd, now + stall + cache_hit_lat_ns, d->cmd.xfer_size);
		ssd_optrace_record_hit(d->cmd.host_seq, d->cmd.stime, t);
		diq_free(self);
		nvmev_p4_commit_at(owner, t);
		return;
	}
	d->cmd.cache_fill = fill;
	if (dep && owner) {
		d->kind = DIQ_K_READ;
		d->cmd.stime = now;
		d->mig = nvmev_diq_mig_hint ? 1 : 0;
		d->dep = dep;
		d->next = diq_head[dep & 0xFFFFu];
		diq_head[dep & 0xFFFFu] = self;
		diq_die_tag(d, dep);
		diq_park++;

		ad = &diq_dep_adv[dep & 0xFFFFu];
		if (ad->frame != dep) {
			ad->frame = dep;
			ad->n = 0;
		}
		if (!ad->n)
			ad->t0 = now;
		ad->n++;
		diq_dep_pub++;
		return;
	}
	d->cmd.stime = now + stall;
	raw_m6_note_completed_nand(d->raw_m6_completed_tn, d->cmd.host_seq);
	raw_m7_note_nand_issue(d->raw_chunk_lpn, d->cmd.xfer_size, d->raw_nr_parts, &d->ftl->ssd->sp,
			       d->cmd.stime, d->cmd.host_seq, d->raw_m7_next_nand_tn);
	t = ssd_advance_nand(d->ftl->ssd, &d->cmd);
	if (!d->cmd.is_small_read)
		tetris_lr_mark(&d->ppa, t, d->rd_tn);
	else
		tetris_sr_mark(&d->ppa, t);
	diq_free(self);
	nvmev_p4_commit_token(owner);
}

static bool conv_park_write(struct conv_ftl *conv_ftl, struct ppa *ppa, uint64_t lpn, uint32_t nr_parts,
			    uint64_t load_ns, uint64_t now, uint32_t host_seq, uint8_t tn, uint32_t frame,
			    uint32_t own, uint8_t rflag)
{
	struct diq_ent *d;
	uint16_t sl;

	sl = nvmev_cur_cmd ? diq_alloc() : DIQ_NIL;
	if (sl == DIQ_NIL) {
		return false;
	}
	d = &diq_pool[sl];
	memset(&d->cmd, 0, sizeof(d->cmd));
	d->cmd.owner = nvmev_cur_cmd;
	d->cmd.host_seq = host_seq;
	d->ppa = *ppa;
	d->cmd.ppa = &d->ppa;
	d->ftl = conv_ftl;
	d->rd_tn = tn;
	d->mig = 0;
	d->raw_chunk_lpn = lpn;
	d->raw_nr_parts = nr_parts;
	d->kind = DIQ_K_WRITE;
	d->chunk_mask = rflag;
	d->wr_load_ns = load_ns;
	d->wr_park_at = now;
	d->wr_own = own;
	d->wr_tk = 0;

	if (ppa_is_virtual(ppa)) {
		uint8_t m;

		m = (rflag & 0x80) ? (uint8_t)(rflag & CE_ALL_MASK) : lb_vlive_mask(ppa);
		if (!(rflag & 0x80))
			d->chunk_mask = m;
		if (!(rflag & 0x40)) {
			lb_vpark(ppa, m);
			d->chunk_mask |= 0x20;
		}
	}
	if (frame == LB_WQ_DEP) {
		uint32_t w;
		int s;

		w = lb_vw(ppa);
		s = __ffs(rflag & CE_ALL_MASK);
		d->wr_tk = lb_vtk_next[w * CE_SLOTS + s]++;
		lb_vqn[w]++;
		dcache_lb_hold(ce_pgkey(ppa), +1);
		lb_wq_push(sl, w);
	} else if (frame == CE_WAIT_FULL) {
		diq_full_push(sl, false);
	} else {
		d->dep = frame;
		d->next = diq_head[frame & 0xFFFFu];
		diq_head[frame & 0xFFFFu] = sl;
		diq_die_tag(d, frame);
	}
	nvmev_note_issued(nvmev_cur_cmd);
	return true;
}

static void conv_diq_write(uint16_t self, uint64_t done)
{
	struct diq_ent *d;
	struct cache_model *dc;
	struct ce_page pg;
	uint64_t now;
	uint64_t stall;
	uint64_t rdy0;
	uint32_t owner;
	uint32_t saved;
	uint32_t lwait;
	uint32_t lown;
	bool q_out;
	int rc;
	int un;
	bool gone;

	d = &diq_pool[self];
	dc = dcache_get();
	owner = d->cmd.owner;
	q_out = false;

	now = done + d->wr_load_ns;
	if (now < d->wr_park_at)
		now = d->wr_park_at;
	if (d->wr_own) {
		uint32_t own;

		own = d->wr_own;
		if (d->chunk_mask & 0x20) {
			lb_vunpark(&d->ppa, d->chunk_mask & CE_ALL_MASK);
			lb_wake(lb_vw(&d->ppa), now);
		}
		diq_free(self);
		nvmev_p4_commit_at(owner, now);

		if (nvmev_wload_finish(own, now, false) == 0)
			;
		else
			;
		return;
	}
	if (d->chunk_mask & 0x40) {
		uint32_t vw;
		int vs;

		vw = lb_vw(&d->ppa);
		vs = __ffs(d->chunk_mask & CE_ALL_MASK);
		if (lb_vcur(&d->ppa) && !lb_vturn(&d->ppa, vs, d->wr_tk)) {
			lb_wq_push(self, vw);
			return;
		}
		if (lb_vcur(&d->ppa)) {
			lb_vtk_serve[vw * CE_SLOTS + vs]++;
			if (lb_vqn[vw])
				lb_vqn[vw]--;
		}
		d->chunk_mask &= (uint8_t)~0x40;
		q_out = true;
	}
	stall = 0;
	conv_ce_page(d->ftl, &d->ppa, d->raw_chunk_lpn, d->raw_nr_parts, &pg);

	if (d->chunk_mask & 0x80) {
		pg.rnd = 1;
		pg.chunk = d->chunk_mask & CE_ALL_MASK;
		pg.rpend = conv_rand_fifo_pend(pg.pgkey) & pg.live & ~pg.chunk;
	}

	gone = pg.rnd && !(pg.live & pg.chunk);
	if (gone) {
		if (pg.live)
			;
		else
			;
		un = dcache_rand_unpend(pg.pgkey, pg.chunk);
		if (un == 2)
			nvmev_diq_full_wake(now);
	}
	if (pg.live && !gone) {
		saved = nvmev_cur_cmd;
		nvmev_cur_cmd = owner;
		rdy0 = dc->st.lba_rand_ready;
		rc = cache_write_pg(dc, &pg, now, d->wr_load_ns, d->ftl->ssd, &d->ppa,
				    d->cmd.host_seq, &stall, d->rd_tn, false, &lwait, &lown);
		nvmev_cur_cmd = saved;
		if (q_out) {
			q_out = false;
			if (dcache_lb_hold(pg.pgkey, -1) && diq_full_live && self != diq_full_waking)
				nvmev_diq_full_wake(now);
			lb_wake(lb_vw(&d->ppa), now);
		}

		if (dc->st.lba_rand_ready != rdy0)
			nvmev_diq_full_wake(now);
		if (rc == CW_WAIT) {
			d->wr_own = lown;

			if (ppa_is_virtual(&d->ppa) && !(d->chunk_mask & 0x20)) {
				lb_vpark(&d->ppa, d->chunk_mask & CE_ALL_MASK);
				d->chunk_mask |= 0x20;
			}
			if (lwait == CE_WAIT_FULL) {
				if (self == diq_full_waking) {
					diq_full_push(self, true);
				} else {
					diq_full_push(self, false);
				}
			} else {
				d->dep = lwait;
				d->next = diq_head[lwait & 0xFFFFu];
				diq_head[lwait & 0xFFFFu] = self;
				diq_die_tag(d, lwait);
			}
			return;
		}
	}
	if (q_out) {
		if (dcache_lb_hold(pg.pgkey, -1) && diq_full_live && self != diq_full_waking)
			nvmev_diq_full_wake(now);
		lb_wake(lb_vw(&d->ppa), now);
	}
	if (d->chunk_mask & 0x20) {
		lb_vunpark(&d->ppa, d->chunk_mask & CE_ALL_MASK);
		lb_wake(lb_vw(&d->ppa), now + stall);
	}
	diq_free(self);
	nvmev_p4_commit_at(owner, now + stall);
}

static uint64_t conv_cache_write_wl(struct conv_ftl *conv_ftl, struct ppa *ppa, uint64_t lpn,
				    uint32_t nr_parts, uint64_t xfer_base, uint64_t cmd_start,
				    uint32_t host_seq, uint8_t tn, bool rnd, bool inpl)
{
	struct ce_page pg;
	uint64_t stall;
	uint64_t load_ns;
	uint64_t rdy0;
	uint32_t lwait;
	uint32_t own;
	int rc;

	stall = 0;

	load_ns = (xfer_base > cmd_start) ? xfer_base - cmd_start : 0;
	conv_ce_page(conv_ftl, ppa, lpn, nr_parts, &pg);
	if (rnd) {
		pg.rnd = 1;
		pg.chunk = (uint8_t)(1u << (ppa->g.pg % CE_SLOTS));
		pg.rpend = conv_rand_fifo_pend(pg.pgkey) & pg.live & ~pg.chunk;
	}
	if (inpl && lb_vbusy(ppa, pg.chunk)) {
		if (conv_park_write(conv_ftl, ppa, lpn, nr_parts, load_ns, xfer_base, host_seq, tn, LB_WQ_DEP,
				    0, (uint8_t)(0x80 | 0x40 | pg.chunk)))
			return xfer_base;
		NVMEV_ASSERT(0 && "DIQ exhausted: no slot for a queued overwrite");
		return xfer_base;
	}
	rdy0 = dcache_get()->st.lba_rand_ready;
	rc = cache_write_pg(dcache_get(), &pg, xfer_base, load_ns, conv_ftl->ssd, ppa, host_seq,
			    &stall, tn, false, &lwait, &own);

	if (dcache_get()->st.lba_rand_ready != rdy0)
		nvmev_diq_full_wake(xfer_base);
	if (rc == CW_DONE)
		return xfer_base + stall;

	if (conv_park_write(conv_ftl, ppa, lpn, nr_parts, load_ns, xfer_base, host_seq, tn, lwait, own,
			    rnd ? (uint8_t)(0x80 | pg.chunk) : 0))
		return xfer_base;

	NVMEV_ASSERT(0 && "DIQ exhausted: no slot for a host write");
	return xfer_base;
}

static inline bool last_pg_in_wordline(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;

	return (ppa->g.pg % spp->pgs_per_oneshotpg) == (spp->pgs_per_oneshotpg - 1);
}

static inline uint32_t free_lines_total(struct conv_ftl *conv_ftl)
{
	return conv_ftl->lm.free_line_cnt + conv_ftl->lm.free_slc_line_cnt;
}

static inline bool should_gc_high(struct conv_ftl *conv_ftl)
{
	return free_lines_total(conv_ftl) <= conv_ftl->cp.gc_thres_lines_high;
}

static inline struct ppa get_maptbl_ent(struct conv_ftl *conv_ftl, uint64_t lpn)
{
	return conv_ftl->maptbl[lpn];
}

static inline void set_maptbl_ent(struct conv_ftl *conv_ftl, uint64_t lpn, struct ppa *ppa)
{
	NVMEV_ASSERT(lpn < conv_ftl->ssd->sp.tt_pgs);
	conv_ftl->maptbl[lpn] = *ppa;
}

static uint64_t ppa2pgidx(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	uint64_t pgidx;

	NVMEV_DEBUG_VERBOSE("%s: ch:%d, lun:%d, pl:%d, blk:%d, pg:%d\n", __func__,
			ppa->g.ch, ppa->g.lun, ppa->g.pl, ppa->g.blk, ppa->g.pg);

	pgidx = ppa->g.ch * spp->pgs_per_ch + ppa->g.lun * spp->pgs_per_lun +
		ppa->g.pl * spp->pgs_per_pl + ppa->g.blk * spp->pgs_per_blk + ppa->g.pg;

	NVMEV_ASSERT(pgidx < spp->tt_pgs);

	return pgidx;
}

static inline uint64_t get_rmap_ent(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	uint64_t pgidx = ppa2pgidx(conv_ftl, ppa);

	return conv_ftl->rmap[pgidx];
}

static inline void set_rmap_ent(struct conv_ftl *conv_ftl, uint64_t lpn, struct ppa *ppa)
{
	uint64_t pgidx = ppa2pgidx(conv_ftl, ppa);

	conv_ftl->rmap[pgidx] = lpn;
}

static inline int line_pool_of(const struct line *l)
{
	return (l->mode == LINE_SLC) ? SLC_POOL : TLC_POOL;
}

static inline struct list_head *pool_free_list(struct conv_ftl *f, int pool)
{
	return &f->lm.free_line_list;
}

static inline struct list_head *pool_full_list(struct conv_ftl *f, int pool)
{
	return (pool == SLC_POOL) ? &f->lm.pslc_full_line_list : &f->lm.full_line_list;
}

static inline pqueue_t *pool_victim_pq(struct conv_ftl *f, int pool)
{
	return (pool == SLC_POOL) ? f->lm.pslc_victim_line_pq : f->lm.victim_line_pq;
}

static inline uint32_t *pool_free_cnt(struct conv_ftl *f, int pool)
{
	return (pool == SLC_POOL) ? &f->lm.free_slc_line_cnt : &f->lm.free_line_cnt;
}

static inline uint32_t *pool_victim_cnt(struct conv_ftl *f, int pool)
{
	return (pool == SLC_POOL) ? &f->lm.pslc_victim_line_cnt : &f->lm.victim_line_cnt;
}

static inline uint32_t *pool_full_cnt(struct conv_ftl *f, int pool)
{
	return (pool == SLC_POOL) ? &f->lm.pslc_full_line_cnt : &f->lm.full_line_cnt;
}

u64 pool_reject[LINE_POOL_N];
u64 pool_borrow[LINE_POOL_N];
u64 gc_borrow[LINE_POOL_N];
u64 gc_nodest[LINE_POOL_N];

static u64 line_free_seq;
u64 rsv_block_pool[LINE_POOL_N];
u64 pool_host_pgs[LINE_POOL_N];
u64 mig_trig, mig_lines, mig_pgs, mig_free_skip, mig_noline;

u64 rb_stop_rsv;

u64 rb_own_room_pgs;

u64 wgc2_slcgc_n;
u64 wgc2_mig_n;
u64 wgc2_tlc_redirect;
u64 wgc2_gc_retry_hit;
u64 wgc2_slc_keep;

u64 wgc2_slc_floor;
u64 wgc2_gcsweep_n;
u64 wgc2_tlcgc_n;
u64 mig_removed;
u64 wp_prep_skip;
int pool_full_flag[LINE_POOL_N];

static inline int mig_line_cmp_pri(pqueue_pri_t next, pqueue_pri_t curr)
{
	return (next > curr);
}

static inline pqueue_pri_t mig_line_get_pri(void *a) { return ((struct line *)a)->mig_order; }

static inline void mig_line_set_pri(void *a, pqueue_pri_t pri)
{
	((struct line *)a)->mig_order = pri;
}

static inline size_t mig_line_get_pos(void *a) { return ((struct line *)a)->mig_pos; }

static inline void mig_line_set_pos(void *a, size_t pos) { ((struct line *)a)->mig_pos = pos; }

static inline int victim_line_cmp_pri(pqueue_pri_t next, pqueue_pri_t curr)
{
	return (next > curr);
}

static inline pqueue_pri_t victim_line_get_pri(void *a)
{
	return ((struct line *)a)->gc_pri;
}

static inline void victim_line_set_pri(void *a, pqueue_pri_t pri)
{
	((struct line *)a)->gc_pri = pri;
}

static inline size_t victim_line_get_pos(void *a)
{
	return ((struct line *)a)->pos;
}

static inline void victim_line_set_pos(void *a, size_t pos)
{
	((struct line *)a)->pos = pos;
}

static inline void consume_write_credit(struct conv_ftl *conv_ftl)
{
	conv_ftl->wfc.write_credits--;
}

static void foreground_gc(struct conv_ftl *conv_ftl, bool tick);

u64 cr_free_lo = ~0ULL;
u64 la_freehist[8];

#define CYC_NB 8
u64 cyc_alloc[CYC_NB];
static u64 cyc_mark_alloc;
static int cyc_mark_b = -1;

static inline int cyc_bucket(u64 c)
{
	return c <= 64 ? 0 : c <= 128 ? 1 : c <= 256 ? 2 : c <= 512 ? 3 :
	       c <= 1024 ? 4 : c <= 2048 ? 5 : c <= 4096 ? 6 : 7;
}

u64 gcacct_erase, gcacct_rr_erase, gcacct_lines, gcacct_vpc_sum, gcacct_move, gcacct_hostwr;

u64 gcacct_lines_slc, gcacct_vpc_sum_slc, gcacct_cap_sum_slc;
u64 gcacct_lines_tlc, gcacct_vpc_sum_tlc, gcacct_cap_sum_tlc;

u64 gcfp_n[2];

u64 slcgc_trig, slcgc_novictim, slcgc_to_tlc, slcgc_to_slc, slcgc_freed;

u32 bp_fl_cur;

int bp_fl_free_lo = INT_MAX;

u64 raw_fallthru;
#define RAWK_SLOTS 4
u64 rawk_fallthru[RAWK_SLOTS];

u64 gcacct_valid_by_tn[TENANT_N + 1];

u64 valid_pages_by_tenant[TENANT_N + 1];

enum validtn_dec_path { VALIDTN_DEC_INVALID = 0, VALIDTN_DEC_ERASE, VALIDTN_DEC_N };
static inline int lpn_to_tenant(uint64_t lpn);
static inline u8 lpn_tenant_get(struct conv_ftl *conv_ftl, u64 lpn);
static inline void lpn_tenant_set(struct conv_ftl *conv_ftl, u64 lpn, u8 tn);
static void validtn_dec_page(struct conv_ftl *conv_ftl, struct ppa *ppa,
			     enum validtn_dec_path path);

static void wgc2_reset(void)
{
	wgc2_slcgc_n = wgc2_mig_n = wgc2_tlcgc_n = 0;
	wgc2_tlc_redirect = wgc2_slc_keep = wgc2_slc_floor = 0;
	wgc2_gcsweep_n = wgc2_mig_fail_n = wgc2_gc_retry_hit = 0;
}

void gcacct_reset(void)
{
	wgc2_reset();

	gcacct_erase = gcacct_rr_erase = gcacct_lines = gcacct_vpc_sum = 0;
	gcacct_lines_slc = gcacct_vpc_sum_slc = gcacct_cap_sum_slc = 0;
	gcacct_lines_tlc = gcacct_vpc_sum_tlc = gcacct_cap_sum_tlc = 0;
	gcacct_move = gcacct_hostwr = 0;
	memset(gcacct_valid_by_tn, 0, sizeof(gcacct_valid_by_tn));
}

u64 tetris_fgc_ns_max;

u64 tetris_hostrd_bytes;

static inline void check_and_refill_write_credit(struct conv_ftl *conv_ftl)
{
	struct write_flow_control *wfc = &(conv_ftl->wfc);

	if (wfc->write_credits <= 0) {
		u64 after;

		foreground_gc(conv_ftl, false);
		after = (u64)conv_ftl->lm.free_line_cnt + conv_ftl->lm.free_slc_line_cnt;
		if (after < cr_free_lo)
			cr_free_lo = after;

		if (cyc_mark_b >= 0) {
			cyc_alloc[cyc_mark_b] += la_calls > cyc_mark_alloc ?
						 la_calls - cyc_mark_alloc : 0;
		}
		cyc_mark_b = cyc_bucket((u64)wfc->credits_to_refill);
		cyc_mark_alloc = la_calls;

		wfc->write_credits += wfc->credits_to_refill;
	}
}

static inline uint32_t line_pgs_per_blk(struct ssdparams *spp, const struct line *l)
{
	return (l && l->mode == LINE_SLC) ? slc_pgs_per_blk(spp) : spp->pgs_per_blk;
}

static inline int line_capacity(struct ssdparams *spp, const struct line *l)
{
	return (int)(line_pgs_per_blk(spp, l) * spp->blks_per_line);
}

static inline int host_free_lines(struct conv_ftl *f, int pool);

static inline int host_free_lines(struct conv_ftl *f, int pool)
{
	return (int)*pool_free_cnt(f, pool) - gc_reserve_lines;
}

static inline bool wp_has_room(struct conv_ftl *f, const struct write_pointer *wpp)
{
	struct ssdparams *spp = &f->ssd->sp;

	uint32_t oneshot = spp->pgs_per_oneshotpg;

	if (!wpp->curline)
		return false;

	if (((wpp->pg + 1) % oneshot) != 0)
		return true;
	if (wpp->ch + 1 != spp->nchs)
		return true;
	if (wpp->lun + 1 != spp->luns_per_ch)
		return true;
	if (wpp->pl + 1 != spp->pls_per_lun)
		return true;
	return (wpp->pg + 1) != line_pgs_per_blk(spp, wpp->curline);
}

static inline void gcacct_note_victim(struct conv_ftl *conv_ftl, const struct line *l)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	u64 cap = (u64)line_capacity(spp, l);

	gcacct_lines++;
	gcacct_vpc_sum += l->vpc;
	if (l->mode == LINE_SLC) {
		gcacct_lines_slc++;
		gcacct_vpc_sum_slc += l->vpc;
		gcacct_cap_sum_slc += cap;
	} else {
		gcacct_lines_tlc++;
		gcacct_vpc_sum_tlc += l->vpc;
		gcacct_cap_sum_tlc += cap;
	}
}

static inline uint64_t line_gc_pri(struct ssdparams *spp, const struct line *l)
{
	return (uint64_t)l->vpc;
}

static void init_lines(struct conv_ftl *conv_ftl)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct line_mgmt *lm = &conv_ftl->lm;
	int i;

	lm->tt_lines = spp->blks_per_pl;
	NVMEV_ASSERT(lm->tt_lines == spp->tt_lines);
	lm->lines = vmalloc(sizeof(struct line) * lm->tt_lines);

	INIT_LIST_HEAD(&lm->free_line_list);
	INIT_LIST_HEAD(&lm->full_line_list);
	INIT_LIST_HEAD(&lm->pslc_line_list);
	INIT_LIST_HEAD(&lm->pslc_free_line_list);

	lm->victim_line_pq = pqueue_init(spp->tt_lines, victim_line_cmp_pri, victim_line_get_pri,
					 victim_line_set_pri, victim_line_get_pos,
					 victim_line_set_pos);

	INIT_LIST_HEAD(&lm->pslc_full_line_list);
	lm->pslc_victim_line_cnt = 0;
	lm->pslc_full_line_cnt = 0;
	lm->pslc_victim_line_pq = pqueue_init(spp->tt_lines, victim_line_cmp_pri,
					      victim_line_get_pri, victim_line_set_pri,
					      victim_line_get_pos, victim_line_set_pos);
	conv_ftl->migrating_line_pq = pqueue_init(spp->tt_lines, mig_line_cmp_pri, mig_line_get_pri,
						  mig_line_set_pri, mig_line_get_pos,
						  mig_line_set_pos);
	conv_ftl->line_write_cnt = 0;

	{
		int want_slc = (slc_line_cnt > 0) ? slc_line_cnt : 0;

		if (want_slc > lm->tt_lines)
			want_slc = lm->tt_lines;
		lm->free_line_cnt = 0;
		lm->free_slc_line_cnt = 0;
		for (i = 0; i < lm->tt_lines; i++) {
			bool slc = (i < want_slc);

			lm->lines[i] = (struct line){
				.id = i,
				.mode = slc ? LINE_SLC : LINE_TLC,
				.ipc = 0,
				.vpc = 0,
				.pos = 0,
				.entry = LIST_HEAD_INIT(lm->lines[i].entry),
			};

			lm->lines[i].free_seq = ++line_free_seq;
			list_add_tail(&lm->lines[i].entry,
				      pool_free_list(conv_ftl, slc ? SLC_POOL : TLC_POOL));
			if (slc)
				lm->free_slc_line_cnt++;
			else
				lm->free_line_cnt++;

			{
				struct ssd *ssd = conv_ftl->ssd;
				uint32_t c, l, pl;

				for (c = 0; c < spp->nchs; c++)
				for (l = 0; l < spp->luns_per_ch; l++)
				for (pl = 0; pl < spp->pls_per_lun; pl++)
					ssd->ch[c].lun[l].pl[pl].blk[i].slc_mode = slc;
			}
		}
	}

	NVMEV_ASSERT(lm->free_line_cnt == (uint32_t)tlc_line_cnt);
	NVMEV_ASSERT(lm->free_line_cnt + lm->free_slc_line_cnt == lm->tt_lines);
	lm->victim_line_cnt = 0;
	lm->full_line_cnt = 0;
}

static void remove_lines(struct conv_ftl *conv_ftl)
{
	pqueue_free(conv_ftl->lm.victim_line_pq);

	pqueue_free(conv_ftl->lm.pslc_victim_line_pq);
	pqueue_free(conv_ftl->migrating_line_pq);
	vfree(conv_ftl->lm.lines);
}

static void init_write_flow_control(struct conv_ftl *conv_ftl)
{
	struct write_flow_control *wfc = &(conv_ftl->wfc);
	struct ssdparams *spp = &conv_ftl->ssd->sp;

	wfc->write_credits = spp->pgs_per_line;
	wfc->credits_to_refill = spp->pgs_per_line;

	conv_ftl->gc_active_line = NULL;
	memset(&conv_ftl->imig_job, 0, sizeof(conv_ftl->imig_job));
	conv_ftl->imig_job.state = TMIG_IDLE;
	memset(&conv_ftl->tcap_ctl, 0, sizeof(conv_ftl->tcap_ctl));
	conv_ftl->gc_lun_order_line = NULL;
}

static inline void check_addr(int a, int max)
{
	NVMEV_ASSERT(a >= 0 && a < max);
}

enum { INVSRC_HOST = 0, INVSRC_RAWMV, INVSRC_N };
static struct line *get_next_free_line_mode(struct conv_ftl *conv_ftl, bool slc, bool strict,
					   bool is_gc)
{
	struct line_mgmt *lm = &conv_ftl->lm;
	struct line *curline = NULL;

	if (!is_gc && gc_reserve_lines > 0 &&
		   (int)(lm->free_line_cnt + lm->free_slc_line_cnt) <= gc_reserve_lines) {
		if (lb_bind_alloc) {
		} else {
			return NULL;
		}
	}

	if (strict) {
		struct line *l;

		list_for_each_entry(l, &lm->free_line_list, entry)
			if (((l->mode == LINE_SLC) ? true : false) == slc) {
				curline = l;
				break;
			}

		if (!curline) {
			curline = list_first_entry_or_null(&lm->free_line_list,
							   struct line, entry);
			if (curline)
				slcline_strict_fb++;
		}
	} else {
		curline = list_first_entry_or_null(&lm->free_line_list, struct line, entry);
	}

	if (!curline) {
		if (slc)
			slcline_starved++;
		NVMEV_ERROR("No free line left in VIRT !!!! (req=%s)\n", slc ? "SLC" : "TLC");

		return NULL;
	}

	list_del_init(&curline->entry);

	if (curline->mode == LINE_SLC) {
		lm->free_slc_line_cnt--;
		slcline_open++;
	} else {
		lm->free_line_cnt--;
	}
	return curline;
}

#define IOT_TN_SHIFT 8
#define IOT_HOT_BIT  (1u << 16)

#define IOT_SLC_BIT  (1u << 17)

#define IOT_RAWL_BIT (1u << 18)
#define IOT_RAWS_BIT (1u << 19)

#define IOT_RB_BIT   (1u << 20)

#define IOT_MIG_BIT  (1u << 21)

#define IOT_RAND_BIT (1u << 22)
#define IOT_SLC(t)   (((t) & IOT_SLC_BIT) ? 1 : 0)
#define IOT_BASE(t)  ((t) & 0xFF)
#define IOT_TN(t)    ((((t) >> IOT_TN_SHIFT) & 0xFF))

enum { LA_USER_TLC = 0, LA_USER_SLC, LA_RAW_L, LA_RAW_S, LA_GC_TLC, LA_GC_SLC, LA_REQ_N };
u64 la_take[LA_REQ_N][2];
u64 la_fail[LA_REQ_N];
u64 la_edge;
u64 la_lo_after = ~0ULL;
u64 la_calls;

static inline void la_note(struct conv_ftl *c, unsigned int io_type, struct line *got)
{
	int req;
	u64 tot;

	if (IOT_BASE(io_type) != USER_IO)
		req = IOT_SLC(io_type) ? LA_GC_SLC : LA_GC_TLC;
	else if (io_type & IOT_RAWL_BIT)
		req = LA_RAW_L;
	else if (io_type & IOT_RAWS_BIT)
		req = LA_RAW_S;
	else
		req = IOT_SLC(io_type) ? LA_USER_SLC : LA_USER_TLC;

	la_calls++;
	if (!got) { la_fail[req]++; return; }
	la_take[req][got->mode == LINE_SLC ? 1 : 0]++;
	tot = (u64)c->lm.free_line_cnt + c->lm.free_slc_line_cnt;
	if (tot <= 2)
		la_edge++;
	if (tot < la_lo_after)
		la_lo_after = tot;

	la_freehist[tot == 0 ? 0 : tot == 1 ? 1 : tot == 2 ? 2 : tot == 3 ? 3 :
		    tot < 8 ? 4 : tot < 16 ? 5 : tot < 32 ? 6 : 7]++;
}

#define IOT_TN_SLC(t)  ((((t) >> IOT_TN_SHIFT) & 0xFF))

#define IOT_TN_GC(t)   (0)

static struct write_pointer *__get_wp(struct conv_ftl *ftl, uint32_t io_type)
{
	uint32_t tn = IOT_TN(io_type);
	uint32_t tn_slc = IOT_TN_SLC(io_type);
	uint32_t slc = IOT_SLC(io_type);
	uint32_t base = IOT_BASE(io_type);

	if (tn > TENANT_N)
		tn = 0;
	if (tn_slc > TENANT_N)
		tn_slc = 0;

	if (base == USER_IO) {

		if (io_type & IOT_RAND_BIT)
			return slc ? &ftl->wp_rand_slc : &ftl->wp_rand_tlc;
		if (slc)
			return &ftl->wp_slc_tn[tn_slc][0];
		return &ftl->wp_tn[tn][0];
	}

	if (base == GC_IO) {
		uint32_t gtn;

		gtn = IOT_TN_GC(io_type);
		if (gtn > TENANT_N)
			gtn = 0;

		if (!slc && (io_type & IOT_MIG_BIT))
			return &ftl->wp_mig_tlc;
		if (slc) {
			return &ftl->gc_wp_slc_tn[gtn];
			return &ftl->wp_slc_tn[0][0];
		}
		return &ftl->gc_wp_tlc_tn[gtn];
		return &ftl->wp_tn[0][0];
	}

	NVMEV_ASSERT(0);
	return NULL;
}

static inline int lb_stream(uint32_t io_tn)
{
	uint32_t tn;

	if (io_tn & IOT_RAND_BIT)
		return LB_NSTREAM - 1;
	tn = IOT_TN(io_tn);
	if (tn > TENANT_N)
		tn = 0;
	return (int)(tn * 2);
}

static struct ppa lb_vget(uint32_t io_tn)
{
	struct lb_vstream *vs;
	struct ppa p;
	uint32_t s;

	vs = &lb_vs[lb_stream(io_tn)];
	if (!vs->n)
		vs->w = lb_valloc(io_tn & ~IOT_SLC_BIT);
	s = vs->n++;
	p = lb_vppa(vs->w, s);
	if (vs->n == CE_SLOTS) {
		vs->n = 0;
		lb_vst[vs->w] = LB_VCLOSED;
	}
	return p;
}

static inline struct write_pointer *host_slc_base_wp(struct conv_ftl *f, uint32_t io_tn)
{
	if (io_tn & IOT_RAND_BIT)
		return &f->wp_rand_slc;
	return &f->wp_slc_tn[0][0];
}

static inline bool rand_split_on(void)
{
	return dcache_get() != NULL;
}

static inline bool conv_grp_full(uint64_t lpn, uint64_t s, uint64_t e)
{
	uint64_t base;

	base = lpn - (lpn % CE_SLOTS);
	return base >= s && base + CE_SLOTS - 1 <= e;
}

static bool conv_slc_grp_fits(struct conv_ftl *f, uint32_t io_tn)
{
	struct write_pointer *wp;
	int rest;

	wp = __get_wp(f, io_tn | IOT_SLC_BIT);
	if (!wp->curline)
		return false;
	rest = line_capacity(&f->ssd->sp, wp->curline) - (int)wp->written;
	if (rest > CE_SLOTS)
		return true;
	return rest == CE_SLOTS && host_free_lines(f, SLC_POOL) >= 1;
}

static inline bool line_alloc_strict(uint32_t io_type)
{
	if (!pslc_rs_tw3_heavy && io_type & IOT_RB_BIT)
		return true;
	return false;
}

static void prepare_write_pointer(struct conv_ftl *conv_ftl, uint32_t io_type)
{
	struct write_pointer *wp = __get_wp(conv_ftl, io_type);

	struct line *curline = get_next_free_line_mode(conv_ftl, IOT_SLC(io_type),
						      line_alloc_strict(io_type),
						      IOT_BASE(io_type) == GC_IO);

	la_note(conv_ftl, io_type, curline);

	NVMEV_ASSERT(wp);
	NVMEV_ASSERT(curline);

	*wp = (struct write_pointer){
		.curline = curline,
		.ch = 0,
		.lun = 0,
		.pg = 0,
		.blk = curline->id,
		.pl = 0,
		.written = 0,
	};
}

static void advance_write_pointer(struct conv_ftl *conv_ftl, uint32_t io_type)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct write_pointer *wpp = __get_wp(conv_ftl, io_type);

	NVMEV_DEBUG_VERBOSE("current wpp: ch:%d, lun:%d, pl:%d, blk:%d, pg:%d\n",
			wpp->ch, wpp->lun, wpp->pl, wpp->blk, wpp->pg);

	{
	uint32_t oneshot = spp->pgs_per_oneshotpg;

	wpp->written++;
	check_addr(wpp->pg, spp->pgs_per_blk);
	wpp->pg++;
	if ((wpp->pg % oneshot) != 0)
		goto out;

	wpp->pg -= oneshot;
	check_addr(wpp->ch, spp->nchs);
	wpp->ch++;
	if (wpp->ch != spp->nchs)
		goto out;

	wpp->ch = 0;
	check_addr(wpp->lun, spp->luns_per_ch);
	wpp->lun++;

	if (wpp->lun != spp->luns_per_ch)
		goto out;

	wpp->lun = 0;
	check_addr(wpp->pl, spp->pls_per_lun);
	wpp->pl++;
	if (wpp->pl != spp->pls_per_lun)
		goto out;

	wpp->pl = 0;

	wpp->pg += oneshot;

	if (wpp->pg != line_pgs_per_blk(spp, wpp->curline))
		goto out;

	wpp->pg = 0;

	if (wpp->curline->mode == LINE_SLC)
		slcline_full++;
	{
		int lpool = line_pool_of(wpp->curline);

		if (wpp->curline->vpc == line_capacity(spp, wpp->curline)) {
			NVMEV_ASSERT(wpp->curline->ipc == 0);
			list_add_tail(&wpp->curline->entry, pool_full_list(conv_ftl, lpool));
			(*pool_full_cnt(conv_ftl, lpool))++;
			NVMEV_DEBUG_VERBOSE("wpp: move line to full_line_list\n");
		} else {
			NVMEV_DEBUG_VERBOSE("wpp: line is moved to victim list\n");
			NVMEV_ASSERT(wpp->curline->vpc >= 0 &&
				     wpp->curline->vpc < line_capacity(spp, wpp->curline));

			NVMEV_ASSERT(wpp->curline->ipc > 0);

			wpp->curline->gc_pri = line_gc_pri(spp, wpp->curline);
			pqueue_insert(pool_victim_pq(conv_ftl, lpool), wpp->curline);
			(*pool_victim_cnt(conv_ftl, lpool))++;
		}

		if (lpool == SLC_POOL && IOT_BASE(io_type) == USER_IO) {
			wpp->curline->mig_order = ++conv_ftl->line_write_cnt;
			pqueue_insert(conv_ftl->migrating_line_pq, wpp->curline);
		}
	}

	check_addr(wpp->blk, spp->blks_per_pl);

	wpp->curline = get_next_free_line_mode(conv_ftl, IOT_SLC(io_type),
					      line_alloc_strict(io_type),
					      IOT_BASE(io_type) == GC_IO);
	la_note(conv_ftl, io_type, wpp->curline);

	wpp->written = 0;

	NVMEV_ASSERT(wpp->curline);
	NVMEV_DEBUG_VERBOSE("wpp: got new clean line %d\n", wpp->curline->id);

	wpp->blk = wpp->curline->id;
	check_addr(wpp->blk, spp->blks_per_pl);

	NVMEV_ASSERT(wpp->pg == 0);
	NVMEV_ASSERT(wpp->lun == 0);
	NVMEV_ASSERT(wpp->ch == 0);
	NVMEV_ASSERT(wpp->pl == 0);
	}
out:
	NVMEV_DEBUG_VERBOSE("advanced wpp: ch:%d, lun:%d, pl:%d, blk:%d, pg:%d (curline %d)\n",
			wpp->ch, wpp->lun, wpp->pl, wpp->blk, wpp->pg, wpp->curline->id);
}

static struct ppa get_new_page(struct conv_ftl *conv_ftl, uint32_t io_type)
{
	struct ppa ppa;
	struct write_pointer *wp = __get_wp(conv_ftl, io_type);

	ppa.ppa = 0;
	ppa.g.ch = wp->ch;
	ppa.g.lun = wp->lun;
	ppa.g.pg = wp->pg;
	ppa.g.blk = wp->blk;
	ppa.g.pl = wp->pl;

	return ppa;
}

static void lb_exit(void)
{
	vfree(lb_vrmap);
	vfree(lb_viot);
	vfree(lb_vgen);
	vfree(lb_vlive);
	vfree(lb_vst);
	vfree(lb_vfifo);
	vfree(lb_vpend);
	vfree(lb_vtk_next);
	vfree(lb_vtk_serve);
	vfree(lb_vwq_head);
	vfree(lb_vwq_tail);
	vfree(lb_vqn);
	vfree(lb_vused);
	lb_vpend = NULL;
	lb_vtk_next = NULL;
	lb_vtk_serve = NULL;
	lb_vwq_head = NULL;
	lb_vwq_tail = NULL;
	lb_vqn = NULL;
	lb_vused = NULL;
	lb_vrmap = NULL;
	lb_viot = NULL;
	lb_vgen = NULL;
	lb_vlive = NULL;
	lb_vst = NULL;
	lb_vfifo = NULL;
	lb_nv = 0;
}

static void lb_init(struct ssdparams *spp)
{
	uint64_t n;
	uint32_t w;
	int s;

	lb_exit();
	memset(lb_vs, 0, sizeof(lb_vs));
	lb_vf_head = 0;
	lb_vf_n = 0;
	if (cache_mb <= 0 || SSD_PARTITIONS != 1)
		return;

	NVMEV_ASSERT(spp->blks_per_pl < VPA_BLK_BIT && spp->pgs_per_oneshotpg == CE_SLOTS);
	n = ((uint64_t)cache_mb << 20) / CACHE_ENTRY_BYTES + DIQ_POOL + 1024;
	lb_vrmap = vmalloc(sizeof(uint64_t) * CE_SLOTS * n);
	lb_viot = vzalloc(sizeof(uint32_t) * n);
	lb_vgen = vzalloc(n);
	lb_vlive = vzalloc(n);
	lb_vst = vzalloc(n);
	lb_vfifo = vmalloc(sizeof(uint32_t) * n);
	lb_vpend = vzalloc(CE_SLOTS * n);
	lb_vtk_next = vzalloc(sizeof(uint16_t) * CE_SLOTS * n);
	lb_vtk_serve = vzalloc(sizeof(uint16_t) * CE_SLOTS * n);
	lb_vwq_head = vmalloc(sizeof(uint16_t) * n);
	lb_vwq_tail = vmalloc(sizeof(uint16_t) * n);
	lb_vqn = vzalloc(sizeof(uint16_t) * n);
	lb_vused = vzalloc(n);
	if (!lb_vrmap || !lb_viot || !lb_vgen || !lb_vlive || !lb_vst || !lb_vfifo || !lb_vpend || !lb_vtk_next ||
	    !lb_vtk_serve || !lb_vwq_head || !lb_vwq_tail || !lb_vqn || !lb_vused) {
		NVMEV_ERROR("late bind: allocation failed -- falling back to binding the PPA at write time\n");
		lb_exit();
		return;
	}
	for (w = 0; w < n; w++) {
		for (s = 0; s < CE_SLOTS; s++)
			lb_vrmap[w * CE_SLOTS + s] = INVALID_LPN;
		lb_vfifo[w] = w;
		lb_vwq_head[w] = LB_NIL;
		lb_vwq_tail[w] = LB_NIL;
	}
	lb_vf_n = (uint32_t)n;
	lb_nv = (uint32_t)n;
	NVMEV_INFO("late bind on: %u virtual wordlines (host writes get a VPA, bound to a physical wordline at flush)\n", lb_nv);
}

static void init_maptbl(struct conv_ftl *conv_ftl)
{
	int i;
	struct ssdparams *spp = &conv_ftl->ssd->sp;

	conv_ftl->maptbl = vmalloc(sizeof(struct ppa) * spp->tt_pgs);
	for (i = 0; i < spp->tt_pgs; i++) {
		conv_ftl->maptbl[i].ppa = UNMAPPED_PPA;
	}
	conv_ftl->lpn_tenant = vzalloc(spp->tt_pgs);

	lb_init(spp);
}

static void remove_maptbl(struct conv_ftl *conv_ftl)
{
	lb_exit();
	vfree(conv_ftl->lpn_tenant);
	conv_ftl->lpn_tenant = NULL;
	vfree(conv_ftl->maptbl);
}

static void init_rmap(struct conv_ftl *conv_ftl)
{
	int i;
	struct ssdparams *spp = &conv_ftl->ssd->sp;

	conv_ftl->rmap = vmalloc(sizeof(uint64_t) * spp->tt_pgs);
	for (i = 0; i < spp->tt_pgs; i++) {
		conv_ftl->rmap[i] = INVALID_LPN;
	}
}

static void remove_rmap(struct conv_ftl *conv_ftl)
{
	vfree(conv_ftl->rmap);
}

static void conv_init_ftl(struct conv_ftl *conv_ftl, struct convparams *cpp, struct ssd *ssd)
{
	conv_ftl->cp = *cpp;

	conv_ftl->ssd = ssd;

	init_maptbl(conv_ftl);

	init_rmap(conv_ftl);

	init_lines(conv_ftl);

	if (conv_ftl->lm.free_slc_line_cnt > 0) {
		struct ssdparams *spp = &conv_ftl->ssd->sp;
		u64 pgs = (u64)conv_ftl->lm.free_slc_line_cnt *
			  line_capacity(spp, &conv_ftl->lm.lines[0]) / spp->pgs_per_oneshotpg;

		pslc_set_pool_pages((uint32_t)pgs);
	}

	{
		struct write_pointer *seen[6 * (TENANT_N + 1) + 6];
		int t, h, k, n = 0;

		#define PREP_ONCE(iot_expr)                                                        \
		do {                                                                               \
			uint32_t _iot = (iot_expr);                                                \
			struct write_pointer *_wp = __get_wp(conv_ftl, _iot);                      \
			int _k;                                                                    \
                                                                                                   \
			for (_k = 0; _k < n; _k++)                                                 \
				if (seen[_k] == _wp)                                               \
					break;                                                     \
			if (_k == n) {                                                             \
				seen[n++] = _wp;                                                   \
				prepare_write_pointer(conv_ftl, _iot);                             \
			} else {                                                                   \
				wp_prep_skip++;    \
			}                                                                          \
		} while (0)

		for (t = 0; t <= TENANT_N; t++)
			for (h = 0; h < 2; h++) {
				uint32_t iot = USER_IO | (t << IOT_TN_SHIFT) |
					       (h ? IOT_HOT_BIT : 0);
				struct write_pointer *wp = __get_wp(conv_ftl, iot);

				for (k = 0; k < n; k++)
					if (seen[k] == wp)
						break;
				if (k < n)
					continue;
				seen[n++] = wp;
				prepare_write_pointer(conv_ftl, iot);
			}

	if (cache_mb > 0)
		PREP_ONCE(USER_IO | IOT_RAND_BIT);
	else
		conv_ftl->wp_rand_tlc = (struct write_pointer){ .curline = NULL };

	if (conv_ftl->lm.free_slc_line_cnt > 0) {
		for (t = 0; t <= TENANT_N; t++)
			for (h = 0; h < 2; h++)
				PREP_ONCE(USER_IO | IOT_SLC_BIT | (t << IOT_TN_SHIFT) |
					  (h ? IOT_HOT_BIT : 0));

		conv_ftl->wp_rb_slc = (struct write_pointer){ .curline = NULL };

		if (cache_mb > 0)
			PREP_ONCE(USER_IO | IOT_SLC_BIT | IOT_RAND_BIT);
		else
			conv_ftl->wp_rand_slc = (struct write_pointer){ .curline = NULL };
	} else {
		for (t = 0; t <= TENANT_N; t++)
			for (h = 0; h < 2; h++)
				conv_ftl->wp_slc_tn[t][h] =
					(struct write_pointer){ .curline = NULL };
		conv_ftl->wp_rb_slc = (struct write_pointer){ .curline = NULL };
		conv_ftl->wp_rand_slc = (struct write_pointer){ .curline = NULL };
	}

	for (t = 0; t <= TENANT_N; t++)
		PREP_ONCE(GC_IO | (t << IOT_TN_SHIFT));

	conv_ftl->wp_mig_tlc = (struct write_pointer){ .curline = NULL };

	if (conv_ftl->lm.free_slc_line_cnt > 0)
		for (t = 0; t <= TENANT_N; t++)
			PREP_ONCE(GC_IO | IOT_SLC_BIT | (t << IOT_TN_SHIFT));
	#undef PREP_ONCE
	}

	if (slc_line_cnt > 0 && conv_ftl->lm.free_slc_line_cnt > 0) {
		pslc_note_slc_block(0);
	}

	init_write_flow_control(conv_ftl);

	NVMEV_INFO("Init FTL instance with %d channels (%ld pages)\n", conv_ftl->ssd->sp.nchs,
		   conv_ftl->ssd->sp.tt_pgs);

	return;
}

static void conv_remove_ftl(struct conv_ftl *conv_ftl)
{
	remove_lines(conv_ftl);
	remove_rmap(conv_ftl);
	remove_maptbl(conv_ftl);
}

#ifndef GC_THRES_LINES
#define GC_THRES_LINES 2
#endif

static void conv_init_params(struct convparams *cpp)
{
	cpp->op_area_pcent = OP_AREA_PERCENT;
	cpp->gc_thres_lines = GC_THRES_LINES;
	cpp->gc_thres_lines_high = GC_THRES_LINES;
	cpp->enable_gc_delay = 1;
	cpp->pba_pcent = (int)((1 + cpp->op_area_pcent) * 100);
}

void conv_init_namespace(struct nvmev_ns *ns, uint32_t id, uint64_t size, void *mapped_addr,
			 uint32_t cpu_nr_dispatcher)
{
	struct ssdparams spp;
	struct convparams cpp;
	struct conv_ftl *conv_ftls;
	struct ssd *ssd;
	uint32_t i;
	const uint32_t nr_parts = SSD_PARTITIONS;

	ssd_init_params(&spp, size, nr_parts);
	conv_init_params(&cpp);

	conv_ftls = kmalloc(sizeof(struct conv_ftl) * nr_parts, GFP_KERNEL);

	for (i = 0; i < nr_parts; i++) {
		ssd = kmalloc(sizeof(struct ssd), GFP_KERNEL);

		if (!ssd || ssd_init(ssd, &spp, cpu_nr_dispatcher) != 0) {
			NVMEV_ERROR("ssd_init(part %u) FAILED -- namespace %u is not created.\n"
				    "  tt_lines=%lu (BLKS_PER_PLN=%d + slc_line_cnt=%d): the FTL tables are large;\n"
				    "  check free host RAM (free -g) and slc_line_cnt.\n",
				    i, id, spp.tt_lines, BLKS_PER_PLN, slc_line_cnt);
			kfree(ssd);
			ns->size = 0;
			ns->mapped = mapped_addr;
			ns->proc_io_cmd = NULL;
			return;
		}
		conv_init_ftl(&conv_ftls[i], &cpp, ssd);
	}

	for (i = 1; i < nr_parts; i++) {
		kfree(conv_ftls[i].ssd->pcie->perf_model);
		kfree(conv_ftls[i].ssd->pcie);

		conv_ftls[i].ssd->pcie = conv_ftls[0].ssd->pcie;
	}

	ns->id = id;
	ns->csi = NVME_CSI_NVM;
	ns->nr_parts = nr_parts;
	ns->ftls = (void *)conv_ftls;
	g_conv_ftl0 = &conv_ftls[0];

	{
		struct conv_ftl *f0 = &conv_ftls[0];
		struct ssdparams *sp0 = &f0->ssd->sp;
		uint64_t all_pgs = 0;
		uint64_t tlc_pgs = 0;
		uint64_t slc_pgs = 0;
		uint32_t li;

		for (li = 0; li < f0->lm.tt_lines; li++) {
			uint64_t cap = (uint64_t)line_capacity(sp0, &f0->lm.lines[li]);

			all_pgs += cap;
			if (f0->lm.lines[li].mode == LINE_SLC)
				slc_pgs += cap;
			else
				tlc_pgs += cap;
		}

		all_pgs *= nr_parts;
		tlc_pgs *= nr_parts;
		slc_pgs *= nr_parts;

		ns->size = (uint64_t)(all_pgs * sp0->pgsz) * (100 - (uint64_t)(cpp.pba_pcent - 100))
			   / 100;

		NVMEV_ASSERT(ns->size <= all_pgs * sp0->pgsz);

		NVMEV_ASSERT(ns->size <= size);
	}
	ns->mapped = mapped_addr;

	ns->proc_io_cmd = conv_proc_nvme_io_cmd;
	ns->qserve_pass = conv_qserve_pass;
	ns->outbox_drain = conv_outbox_drain;
	ns->gc_tick = conv_gc_tick;

	NVMEV_INFO("FTL physical space: %lld, logical space: %lld (physical/logical * 100 = %d)\n",
		   size, ns->size, cpp.pba_pcent);

	return;
}

bool nvmev_ppa_line_migrating(struct ssd *ssd, struct ppa *ppa)
{
	struct conv_ftl *f;
	struct line *l;
	u32 part;

	f = raw_find_ftl(ssd, &part);
	if (!f)
		return false;
	l = get_line(f, ppa);
	if (!l)
		return false;
	return l->mig_pos != 0;
}

void conv_remove_namespace(struct nvmev_ns *ns)
{
	struct conv_ftl *conv_ftls = (struct conv_ftl *)ns->ftls;
	const uint32_t nr_parts = SSD_PARTITIONS;
	uint32_t i;

	for (i = 1; i < nr_parts; i++) {
		conv_ftls[i].ssd->pcie = NULL;
	}

	for (i = 0; i < nr_parts; i++) {
		conv_remove_ftl(&conv_ftls[i]);
		ssd_remove(conv_ftls[i].ssd);
		kfree(conv_ftls[i].ssd);
	}

	kfree(conv_ftls);
	g_conv_ftl0 = NULL;
	ns->ftls = NULL;
	raw_meta_exit();
}

static inline bool valid_ppa(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	int ch = ppa->g.ch;
	int lun = ppa->g.lun;
	int pl = ppa->g.pl;
	int blk = ppa->g.blk;
	int pg = ppa->g.pg;

	if (ch < 0 || ch >= spp->nchs)
		return false;
	if (lun < 0 || lun >= spp->luns_per_ch)
		return false;
	if (pl < 0 || pl >= spp->pls_per_lun)
		return false;
	if (blk < 0 || blk >= spp->blks_per_pl)
		return false;
	if (pg < 0 || pg >= spp->pgs_per_blk)
		return false;

	return true;
}

static inline bool valid_lpn(struct conv_ftl *conv_ftl, uint64_t lpn)
{
	return (lpn < conv_ftl->ssd->sp.tt_pgs);
}

static inline bool mapped_ppa(struct ppa *ppa)
{
	return !(ppa->ppa == UNMAPPED_PPA);
}

static inline struct line *get_line(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	return &(conv_ftl->lm.lines[ppa->g.blk]);
}

u64 slcgc_rdcnt_n, slcgc_rdcnt_sum, slcgc_rdcnt_max, slcgc_rdcnt_hist[RDCNT_BKT];

static inline int rdcnt_bucket(u32 c)
{
	if (c == 0)
		return 0;
	if (c < 10)
		return 1;
	if (c < 100)
		return 2;
	if (c < 1000)
		return 3;
	return 4;
}

static inline void rdcnt_note(const struct line *l, u64 *n, u64 *sum, u64 *mx, u64 *hist)
{
	(*n)++;
	*sum += l->rd_cnt;
	if ((u64)l->rd_cnt > *mx)
		*mx = l->rd_cnt;
	hist[rdcnt_bucket(l->rd_cnt)]++;
}

void conv_note_line_read(struct ppa *ppa, uint64_t now_ns)
{
	struct conv_ftl *conv_ftl = g_conv_ftl0;
	struct line *l;

	if (!conv_ftl || !conv_ftl->lm.lines)
		return;
	l = get_line(conv_ftl, ppa);
	if (l->rd_cnt != U32_MAX)
		l->rd_cnt++;
	if (l->rd_life_n != U32_MAX)
		l->rd_life_n++;

	if (l->last_rd_ns && now_ns > l->last_rd_ns) {
		l->rd_gap_sum += now_ns - l->last_rd_ns;
		l->rd_gap_n++;
	}
	l->last_rd_ns = now_ns;
}

static uint32_t slc_free_pgs(struct conv_ftl *conv_ftl)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct line_mgmt *lm = &conv_ftl->lm;

	uint32_t cap, rest = 0;
	int _t, _h;

	cap = (uint32_t)line_capacity(spp, &lm->lines[0]);
	for (_t = 0; _t <= TENANT_N; _t++)
		for (_h = 0; _h < 2; _h++) {
			struct write_pointer *wp = &conv_ftl->wp_slc_tn[_t][_h];

			if (!wp->curline)
				continue;
			cap = (uint32_t)line_capacity(spp, wp->curline);
			if (wp->written < cap)
				rest += cap - wp->written;
		}

	if (conv_ftl->wp_rb_slc.curline) {
		uint32_t rcap = (uint32_t)line_capacity(spp, conv_ftl->wp_rb_slc.curline);

		if (conv_ftl->wp_rb_slc.written < rcap)
			rest += rcap - conv_ftl->wp_rb_slc.written;
	}

	if (conv_ftl->wp_rand_slc.curline) {
		uint32_t rcap = (uint32_t)line_capacity(spp, conv_ftl->wp_rand_slc.curline);

		if (conv_ftl->wp_rand_slc.written < rcap)
			rest += rcap - conv_ftl->wp_rand_slc.written;
	}
	if (!lm->free_slc_line_cnt && !rest)
		return 0;

	return (lm->free_slc_line_cnt * cap + rest) / spp->pgs_per_oneshotpg;
}

static inline bool same_oneshotpg(struct ssdparams *spp, struct ppa *a, struct ppa *b)
{
	return a->g.ch == b->g.ch && a->g.lun == b->g.lun && a->g.pl == b->g.pl &&
	       a->g.blk == b->g.blk &&
	       (a->g.pg / spp->pgs_per_oneshotpg) == (b->g.pg / spp->pgs_per_oneshotpg);
}

static void mark_page_invalid_src(struct conv_ftl *conv_ftl, struct ppa *ppa, int src)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct nand_block *blk = NULL;
	struct nand_page *pg = NULL;
	bool was_full_line = false;
	struct line *line;

	pg = get_pg(conv_ftl->ssd, ppa);
	NVMEV_ASSERT(pg->status == PG_VALID);
	pg->status = PG_INVALID;

	blk = get_blk(conv_ftl->ssd, ppa);
	NVMEV_ASSERT(blk->ipc >= 0 && blk->ipc < spp->pgs_per_blk);
	blk->ipc++;
	NVMEV_ASSERT(blk->vpc > 0 && blk->vpc <= spp->pgs_per_blk);
	blk->vpc--;

	line = get_line(conv_ftl, ppa);
	NVMEV_ASSERT(line->ipc >= 0 && line->ipc < line_capacity(spp, line));
	if (line->vpc == line_capacity(spp, line)) {
		NVMEV_ASSERT(line->ipc == 0);
		was_full_line = true;
	}
	line->ipc++;
	NVMEV_ASSERT(line->vpc > 0 && line->vpc <= line_capacity(spp, line));

	line->vpc--;
	if (line->pos)
		pqueue_change_priority(pool_victim_pq(conv_ftl, line_pool_of(line)),
				       line_gc_pri(spp, line), line);

	validtn_dec_page(conv_ftl, ppa, VALIDTN_DEC_INVALID);

	if (was_full_line && !line->inflight) {
		int lpool = line_pool_of(line);

		list_del_init(&line->entry);
		(*pool_full_cnt(conv_ftl, lpool))--;

		line->gc_pri = line_gc_pri(spp, line);
		pqueue_insert(pool_victim_pq(conv_ftl, lpool), line);
		(*pool_victim_cnt(conv_ftl, lpool))++;
	}
}

static void mark_page_invalid(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	mark_page_invalid_src(conv_ftl, ppa, INVSRC_HOST);
}

static void mark_page_valid(struct conv_ftl *conv_ftl, struct ppa *ppa, int owner)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct nand_block *blk = NULL;
	struct nand_page *pg = NULL;
	struct line *line;
	u64 lpn;
	int tn;

	pg = get_pg(conv_ftl->ssd, ppa);
	NVMEV_ASSERT(pg->status == PG_FREE);
	pg->status = PG_VALID;

	blk = get_blk(conv_ftl->ssd, ppa);
	NVMEV_ASSERT(blk->vpc >= 0 && blk->vpc < spp->pgs_per_blk);
	blk->vpc++;

	line = get_line(conv_ftl, ppa);
	NVMEV_ASSERT(line->vpc >= 0 && line->vpc < line_capacity(spp, line));
	line->vpc++;

	lpn = get_rmap_ent(conv_ftl, ppa);
	NVMEV_ASSERT(valid_lpn(conv_ftl, lpn));
	if (owner >= 0)
		lpn_tenant_set(conv_ftl, lpn, (u8)owner);
	tn = lpn_tenant_get(conv_ftl, lpn);
	valid_pages_by_tenant[tn]++;
}

int conv_rebind_raw_to_slc_k(struct ssd *ssd, uint64_t tag, struct ppa *cur, struct ppa *out, int rawk,
			     uint64_t *lbas)
{
	struct conv_ftl *conv_ftl = g_conv_ftl0;
	struct ssdparams *spp;
	uint32_t oneshot, i, moved = 0;
	uint64_t base;
	bool unit_fits;
	bool rb_guard;
	bool rb_go_all;
	uint32_t rb_iot;
	uint64_t lpn0;

	uint64_t nl[CE_SLOTS];
	int k;

	if (!cur || cur->ppa == UNMAPPED_PPA)
		return 0;
	if (!conv_ftl || !conv_ftl->ssd || conv_ftl->ssd != ssd)
		return 0;
	spp = &conv_ftl->ssd->sp;

	if (get_line(conv_ftl, cur)->mode == LINE_SLC)
		return 0;

	rb_iot = USER_IO | IOT_SLC_BIT | IOT_RB_BIT;
	lpn0 = tag * spp->pgs_per_oneshotpg;
	for (k = 0; lbas && k < CE_SLOTS; k++) {
		if (lbas[k] != CE_LBA_NONE) {
			lpn0 = lbas[k];
			break;
		}
	}
	if (lpn0 < spp->tt_pgs)
		rb_iot |= (uint32_t)lpn_tenant_get(conv_ftl, lpn0) << IOT_TN_SHIFT;

	if (!__get_wp(conv_ftl, rb_iot)->curline) {
		slcline_raw_nospace++;
		return 0;
	}
	if (slc_free_pgs(conv_ftl) == 0) {
		raw_fallthru++;
		if (rawk >= 0 && rawk < RAWK_N)
			rawk_fallthru[rawk]++;
	}

	oneshot = spp->pgs_per_oneshotpg;
	base = tag * oneshot;

	unit_fits = false;
	{
		struct write_pointer *rwp = __get_wp(conv_ftl, rb_iot);

		unit_fits = rwp->curline &&
			    line_capacity(spp, rwp->curline) - (int)rwp->written > (int)oneshot;
	}

	rb_guard = gc_reserve_lines > 0;

	rb_go_all = false;

	for (k = 0; k < CE_SLOTS; k++)
		nl[k] = CE_LBA_NONE;
	for (i = 0; i < oneshot; i++) {
		uint64_t lpn;
		struct ppa old, new;

		if (lbas && lbas[i] == CE_LBA_NONE)
			continue;
		lpn = lbas ? lbas[i] : base + i;
		if (lpn >= spp->tt_pgs)
			break;
		old = get_maptbl_ent(conv_ftl, lpn);
		if (old.ppa == UNMAPPED_PPA || !same_oneshotpg(spp, &old, cur))
			continue;

		{
			unsigned int iot = rb_iot |
					   (rawk == RAWK_L ? IOT_RAWL_BIT :
					    rawk == RAWK_S ? IOT_RAWS_BIT : 0);

		if (rb_guard && !rb_go_all && (int)*pool_free_cnt(conv_ftl, SLC_POOL) <= gc_reserve_lines) {
			if (!unit_fits) {
				rb_stop_rsv++;
				break;
			}
			rb_own_room_pgs++;
		}
		new = get_new_page(conv_ftl, iot);
		mark_page_invalid_src(conv_ftl, &old, INVSRC_RAWMV);
		set_maptbl_ent(conv_ftl, lpn, &new);
		set_rmap_ent(conv_ftl, lpn, &new);
		mark_page_valid(conv_ftl, &new, -1);
		advance_write_pointer(conv_ftl, iot);
		}
		slcline_host_pgs++;
		if (!moved)
			*out = new;
		moved++;

		if (lbas && new.h.blk_in_ssd == out->h.blk_in_ssd &&
		    new.g.pg / spp->pgs_per_flashpg == out->g.pg / spp->pgs_per_flashpg)
			nl[new.g.pg % CE_SLOTS] = lpn;
	}
	if (moved)
		slcline_raw_rebind += moved;
	if (lbas && moved) {
		for (k = 0; k < CE_SLOTS; k++)
			lbas[k] = nl[k];
	}
	return (int)moved;
}

static void mark_page_pad(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct nand_page *pg;
	struct nand_block *blk;
	struct line *line;

	pg = get_pg(conv_ftl->ssd, ppa);
	NVMEV_ASSERT(pg->status == PG_FREE);
	pg->status = PG_INVALID;
	blk = get_blk(conv_ftl->ssd, ppa);
	NVMEV_ASSERT(blk->ipc >= 0 && blk->ipc < spp->pgs_per_blk);
	blk->ipc++;
	line = get_line(conv_ftl, ppa);
	NVMEV_ASSERT(line->ipc >= 0 && line->ipc < line_capacity(spp, line));
	line->ipc++;
	set_rmap_ent(conv_ftl, INVALID_LPN, ppa);
}

int conv_rand_pad_wl(struct ssd *ssd, struct ppa *wl)
{
	struct conv_ftl *conv_ftl;
	struct write_pointer *wp;
	struct ppa p;
	uint32_t iot;
	int n;
	int k;

	conv_ftl = g_conv_ftl0;
	if (!conv_ftl || conv_ftl->ssd != ssd)
		return 0;
	if (ppa_is_virtual(wl)) {
		n = lb_vrand_open(wl);
		if (n)
			lb_vclose(lb_vw(wl));
		return n;
	}
	n = 0;
	for (k = 0; k < 2; k++) {
		wp = k ? &conv_ftl->wp_rand_slc : &conv_ftl->wp_rand_tlc;
		iot = USER_IO | IOT_RAND_BIT | (k ? IOT_SLC_BIT : 0);
		while (conv_rand_wl_open(conv_ftl, wl) && __get_wp(conv_ftl, iot) == wp &&
		       wp->curline && wp->blk == wl->g.blk && wp->ch == wl->g.ch && wp->lun == wl->g.lun &&
		       wp->pl == wl->g.pl &&
		       (wp->pg / ssd->sp.pgs_per_oneshotpg) == (wl->g.pg / ssd->sp.pgs_per_oneshotpg)) {
			p = get_new_page(conv_ftl, iot);
			mark_page_pad(conv_ftl, &p);
			advance_write_pointer(conv_ftl, iot);
			n++;
		}
	}
	return n;
}

static bool lb_want_slc(struct conv_ftl *conv_ftl, uint32_t iot0, bool count)
{
	struct write_pointer *swp;
	int hfree;
	bool room;
	bool want;

	swp = host_slc_base_wp(conv_ftl, iot0);
	hfree = host_free_lines(conv_ftl, SLC_POOL);
	room = wp_has_room(conv_ftl, swp);
	want = swp->curline && hfree >= 1 && (hfree > 1 || room);
	if (want && pslc_raw_reserve && slc_free_pgs(conv_ftl) <= pslc_reserve_pgs())
		want = false;
	if (want && !__get_wp(conv_ftl, iot0 | IOT_SLC_BIT)->curline)
		want = false;
	if (count) {
		if (swp->curline && !want)
			wgc2_tlc_redirect++;
		else if (want && room && hfree <= 1)
			wgc2_slc_keep++;
		if (swp->curline && hfree < 1)
			wgc2_slc_floor++;
	}
	return want;
}

static uint32_t lb_raw_iot(struct conv_ftl *conv_ftl, uint32_t w, int rawk)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct write_pointer *rwp;
	uint64_t lpn0;
	uint32_t rb_iot;
	bool rb_guard;
	bool unit_fits;
	int s;

	rb_iot = USER_IO | IOT_SLC_BIT | IOT_RB_BIT;
	lpn0 = INVALID_LPN;
	for (s = 0; s < CE_SLOTS; s++) {
		if (lb_vrmap[w * CE_SLOTS + s] != INVALID_LPN) {
			lpn0 = lb_vrmap[w * CE_SLOTS + s];
			break;
		}
	}
	if (lpn0 < spp->tt_pgs)
		rb_iot |= (uint32_t)lpn_tenant_get(conv_ftl, lpn0) << IOT_TN_SHIFT;
	rwp = __get_wp(conv_ftl, rb_iot);
	if (!rwp->curline) {
		slcline_raw_nospace++;
		return 0;
	}
	if (slc_free_pgs(conv_ftl) == 0) {
		raw_fallthru++;
		rawk_fallthru[rawk]++;
	}
	unit_fits = line_capacity(spp, rwp->curline) - (int)rwp->written > (int)spp->pgs_per_oneshotpg;
	rb_guard = gc_reserve_lines > 0;
	if (rb_guard && (int)*pool_free_cnt(conv_ftl, SLC_POOL) <= gc_reserve_lines) {
		if (!unit_fits) {
			rb_stop_rsv++;
			return 0;
		}
		rb_own_room_pgs += CE_SLOTS;
	}
	return rb_iot | (rawk == RAWK_L ? IOT_RAWL_BIT : rawk == RAWK_S ? IOT_RAWS_BIT : 0);
}

int conv_bind_wl(struct ssd *ssd, struct ppa *vppa, int rawk, struct ppa *out, uint8_t *bmask)
{
	struct conv_ftl *conv_ftl;
	struct write_pointer *wp;
	struct ppa p;
	struct ppa v;
	uint64_t lpn;
	uint32_t iot0;
	uint32_t iot;
	uint32_t w;
	bool stale;
	int n_live;
	int r;
	int s;

	conv_ftl = g_conv_ftl0;
	NVMEV_ASSERT(conv_ftl && conv_ftl->ssd == ssd && lb_nv);
	*bmask = 0;
	r = 0;
	w = lb_vw(vppa);
	NVMEV_ASSERT(w < lb_nv);
	stale = !lb_vcur(vppa);
	if (stale)
		;
	else
		lb_vclose(w);
	iot0 = stale ? USER_IO : lb_viot[w];
	iot = 0;
	if (!stale && rawk > RAWK_NONE && rawk < RAWK_N) {
		iot = lb_raw_iot(conv_ftl, w, rawk);
		if (iot) {
			r |= LB_BIND_RAW;
		}
	}
	if (!iot)
		iot = iot0 | (lb_want_slc(conv_ftl, iot0, true) ? IOT_SLC_BIT : 0);
	wp = __get_wp(conv_ftl, iot);
	NVMEV_ASSERT(wp->curline);
	lb_bind_alloc = true;

	while (wp->pg % CE_SLOTS) {
		p = get_new_page(conv_ftl, iot);
		mark_page_pad(conv_ftl, &p);
		advance_write_pointer(conv_ftl, iot);
	}
	n_live = 0;
	for (s = 0; s < CE_SLOTS; s++) {
		p = get_new_page(conv_ftl, iot);
		if (!s)
			*out = p;
		lpn = stale ? INVALID_LPN : lb_vrmap[w * CE_SLOTS + s];
		v = stale ? *vppa : lb_vppa(w, s);
		if (lpn != INVALID_LPN && get_maptbl_ent(conv_ftl, lpn).ppa == v.ppa) {
			set_maptbl_ent(conv_ftl, lpn, &p);
			set_rmap_ent(conv_ftl, lpn, &p);
			mark_page_valid(conv_ftl, &p, -1);
			*bmask |= 1u << s;
			n_live++;
		} else {
			mark_page_pad(conv_ftl, &p);
			if (!stale && (lb_vused[w] & (1u << s)))
				;
			else
				;
		}
		advance_write_pointer(conv_ftl, iot);
	}
	lb_bind_alloc = false;
	if (!stale) {
		for (s = 0; s < CE_SLOTS; s++)
			lb_vrmap[w * CE_SLOTS + s] = INVALID_LPN;
		lb_vlive[w] = 0;
		lb_vfree_maybe(w);
	}

	pool_host_pgs[(iot & IOT_SLC_BIT) ? SLC_POOL : TLC_POOL] += n_live;
	if (iot & IOT_SLC_BIT) {
		r |= LB_BIND_SLC;
		slcline_host_pgs += n_live;
	}
	if (r & LB_BIND_RAW)
		slcline_raw_rebind += n_live;
	return r;
}

void conv_bind_peek(struct ssd *ssd, const struct ppa *vppa, struct ppa *out)
{
	struct conv_ftl *conv_ftl;
	struct write_pointer *wp;
	uint32_t iot0;

	out->ppa = 0;
	conv_ftl = g_conv_ftl0;
	if (!conv_ftl || !lb_nv || !lb_vcur(vppa))
		return;
	iot0 = lb_viot[lb_vw(vppa)];
	wp = __get_wp(conv_ftl, iot0 | (lb_want_slc(conv_ftl, iot0, false) ? IOT_SLC_BIT : 0));
	if (!wp->curline)
		return;
	out->g.ch = wp->ch;
	out->g.lun = wp->lun;
	out->g.pl = wp->pl;
	out->g.blk = wp->blk;
	out->g.pg = wp->pg;
}

static void mark_block_free(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct nand_block *blk = get_blk(conv_ftl->ssd, ppa);
	struct nand_page *pg = NULL;
	int i;

	for (i = 0; i < spp->pgs_per_blk; i++) {
		pg = &blk->pg[i];
		NVMEV_ASSERT(pg->nsecs == spp->secs_per_pg);

		if (pg->status == PG_VALID) {
			struct ppa p = *ppa;

			p.g.pg = i;
			validtn_dec_page(conv_ftl, &p, VALIDTN_DEC_ERASE);
		}
		pg->status = PG_FREE;
	}

	NVMEV_ASSERT(blk->npgs == spp->pgs_per_blk);
	blk->ipc = 0;
	blk->vpc = 0;
	blk->erase_cnt++;
}

static bool block_has_nonfree_pages(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	struct nand_block *blk = get_blk(conv_ftl->ssd, ppa);

	return blk->vpc || blk->ipc;
}

static u8 tenant_map[TENANT_MAP_N];

static inline uint16_t rw_stream_id(const struct nvme_rw_command *rw)
{
	if (!(le16_to_cpu(rw->control) & NVME_RW_DTYPE_STREAMS))
		return 0;
	return CDW13_DSPEC(le32_to_cpu(rw->dsmgmt));
}

void tenant_map_clear(void)
{
	memset(tenant_map, 0, sizeof(tenant_map));
}

void tenant_map_set(u8 tn, u64 start_lba, u64 nlba)
{
	u64 c0, c1, c;

	if (!tn || tn >= TENANT_N + 1 || !nlba)
		return;
	c0 = (start_lba / 8) / TENANT_MAP_CHUNK_LPNS;
	c1 = ((start_lba + nlba - 1) / 8) / TENANT_MAP_CHUNK_LPNS;
	if (c1 >= TENANT_MAP_N)
		c1 = TENANT_MAP_N - 1;
	for (c = c0; c <= c1; c++) {
		tenant_map[c] = tn;
	}
}

static inline int lpn_to_tenant(uint64_t lpn)
{
	u64 c = lpn / TENANT_MAP_CHUNK_LPNS;

	return c < TENANT_MAP_N ? tenant_map[c] : 0;
}

static inline u8 lpn_tenant_get(struct conv_ftl *conv_ftl, u64 lpn)
{
	return conv_ftl->lpn_tenant[lpn];
}

static inline void lpn_tenant_set(struct conv_ftl *conv_ftl, u64 lpn, u8 tn)
{
	NVMEV_ASSERT(tn <= TENANT_N);
	conv_ftl->lpn_tenant[lpn] = tn;
}

static void validtn_dec_page(struct conv_ftl *conv_ftl, struct ppa *ppa,
			     enum validtn_dec_path path)
{
	u64 lpn = get_rmap_ent(conv_ftl, ppa);
	int tn;

	if (!valid_lpn(conv_ftl, lpn)) {
		tn = 0;
	} else {
		tn = lpn_tenant_get(conv_ftl, lpn);
	}
	if (valid_pages_by_tenant[tn])
		valid_pages_by_tenant[tn]--;
	else {
	}
}

static void gcacct_scan_victim(struct conv_ftl *conv_ftl, struct line *line)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct ppa ppa = { 0 };
	int ch, lun, pl, pg, seen = 0;
	u64 per_tn[TENANT_N + 1] = { 0 };

	ppa.g.blk = line->id;
	for (ch = 0; ch < spp->nchs; ch++)
		for (lun = 0; lun < spp->luns_per_ch; lun++)
			for (pl = 0; pl < spp->pls_per_lun; pl++) {
				ppa.g.ch = ch; ppa.g.lun = lun; ppa.g.pl = pl;
				for (pg = 0; pg < spp->pgs_per_blk; pg++) {
					struct nand_page *p;

					ppa.g.pg = pg;
					p = get_pg(conv_ftl->ssd, &ppa);
					if (p->status != PG_VALID)
						continue;
					{
						u64 lpn = get_rmap_ent(conv_ftl, &ppa);

						if (valid_lpn(conv_ftl, lpn))
							per_tn[lpn_tenant_get(conv_ftl, lpn)]++;
						else
							per_tn[0]++;
					}
				}
			}
	gcacct_valid_by_tn[0] += per_tn[0];
	for (ch = 1; ch <= TENANT_N; ch++) {
		gcacct_valid_by_tn[ch] += per_tn[ch];
		if (per_tn[ch])
			seen++;
	}
	if (seen <= 1)
		;
	else
		;
}

static bool in_migration;
static struct tetris_mig_job *in_migration_job;

static bool imig_source_current(struct conv_ftl *conv_ftl, struct ppa *src)
{
	u64 lpn = get_rmap_ent(conv_ftl, src);
	struct ppa cur;

	if (!valid_lpn(conv_ftl, lpn))
		return false;
	cur = get_maptbl_ent(conv_ftl, lpn);
	return cur.ppa == src->ppa;
}

static void imig_note_copy_op(struct conv_ftl *conv_ftl, int cmd, u64 bytes)
{
	if (!in_migration_job)
		return;
}

static uint64_t gc_write_page_iot(struct conv_ftl *conv_ftl, struct ppa *old_ppa, uint32_t io_type)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct convparams *cpp = &conv_ftl->cp;
	struct ppa new_ppa;
	uint64_t lpn = get_rmap_ent(conv_ftl, old_ppa);
	u8 gtn;

	NVMEV_ASSERT(valid_lpn(conv_ftl, lpn));

	gtn = lpn_tenant_get(conv_ftl, lpn);
	if (gtn <= TENANT_N)
		io_type |= ((uint32_t)gtn << IOT_TN_SHIFT);

	if (in_migration && !IOT_SLC(io_type)) {
		io_type |= IOT_MIG_BIT;
		NVMEV_ASSERT(conv_ftl->wp_mig_tlc.curline &&
			     "migration TLC write pointer is closed (migration outside its init condition)");
	}

	if (get_line(conv_ftl, old_ppa)->mode == LINE_SLC) {
		slcline_gc_moved++;
	}
	new_ppa = get_new_page(conv_ftl, io_type);

	set_maptbl_ent(conv_ftl, lpn, &new_ppa);

	set_rmap_ent(conv_ftl, lpn, &new_ppa);

	mark_page_valid(conv_ftl, &new_ppa, -1);

	advance_write_pointer(conv_ftl, io_type);

	if (cpp->enable_gc_delay && last_pg_in_wordline(conv_ftl, &new_ppa)) {
		struct nand_cmd gcw = {
			.type = GC_IO,
			.cmd = NAND_WRITE,
			.migrate = in_migration,
			.stime = 0,
			.xfer_size = spp->pgsz * spp->pgs_per_oneshotpg,
			.interleave_pci_dma = false,
			.ppa = &new_ppa,

			.fold = get_line(conv_ftl, old_ppa)->mode == LINE_SLC,
		};

		imig_note_copy_op(conv_ftl, NAND_WRITE, gcw.xfer_size);
		ssd_advance_nand(conv_ftl->ssd, &gcw);
	}

	return 0;
}

static struct line *select_victim_line_pool(struct conv_ftl *conv_ftl, bool force, int pool)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct line *victim_line = NULL;

	victim_line = pqueue_peek(pool_victim_pq(conv_ftl, pool));
	if (!victim_line) {
		return NULL;
	}

	if (!force && (victim_line->vpc > (line_capacity(spp, victim_line) / 8))) {
		return NULL;
	}

	pqueue_pop(pool_victim_pq(conv_ftl, pool));
	victim_line->pos = 0;
	(*pool_victim_cnt(conv_ftl, pool))--;
	victim_line->inflight = 1;
	if (victim_line->mode == LINE_SLC) {
		slcline_gc_victim++;

		rdcnt_note(victim_line, &slcgc_rdcnt_n, &slcgc_rdcnt_sum,
			   &slcgc_rdcnt_max, slcgc_rdcnt_hist);
	}

	gcacct_note_victim(conv_ftl, victim_line);
	if (gc_accounting)
		gcacct_scan_victim(conv_ftl, victim_line);

	return victim_line;
}

static int clean_one_flashpg_iot(struct conv_ftl *conv_ftl, struct ppa *ppa, uint32_t iot);
static void mark_line_free(struct conv_ftl *conv_ftl, struct ppa *ppa);

static int clean_one_flashpg_iot(struct conv_ftl *conv_ftl, struct ppa *ppa, uint32_t iot)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct convparams *cpp = &conv_ftl->cp;
	struct nand_page *pg_iter = NULL;
	int cnt = 0, i = 0;
	struct ppa ppa_copy = *ppa;

	for (i = 0; i < spp->pgs_per_flashpg; i++) {
		pg_iter = get_pg(conv_ftl->ssd, &ppa_copy);

		if (in_migration_job) {
			if (pg_iter->status != PG_VALID) {
				ppa_copy.g.pg++;
				continue;
			}
			if (!imig_source_current(conv_ftl, &ppa_copy)) {
				ppa_copy.g.pg++;
				continue;
			}
			cnt++;
			ppa_copy.g.pg++;
			continue;
		}
		if (pg_iter->status == PG_FREE) {
			ppa_copy.g.pg++;
			continue;
		}
		if (pg_iter->status == PG_VALID)
			cnt++;

		ppa_copy.g.pg++;
	}

	ppa_copy = *ppa;

	{
		int sl = get_line(conv_ftl, &ppa_copy)->mode == LINE_SLC ? 1 : 0;

		gcfp_n[sl]++;
		if (cnt <= 0) {
			return 0;
		}
	}

	if (cpp->enable_gc_delay) {
		struct nand_cmd gcr = {
			.type = GC_IO,
			.cmd = NAND_READ,
			.stime = 0,
			.xfer_size = spp->pgsz * cnt,
			.interleave_pci_dma = false,
			.ppa = &ppa_copy,
			.fold = get_line(conv_ftl, &ppa_copy)->mode == LINE_SLC,
			.migrate = in_migration,
		};
		imig_note_copy_op(conv_ftl, NAND_READ, gcr.xfer_size);
		ssd_advance_nand(conv_ftl->ssd, &gcr);
	}

	for (i = 0; i < spp->pgs_per_flashpg; i++) {
		pg_iter = get_pg(conv_ftl->ssd, &ppa_copy);

		if (pg_iter->status == PG_VALID &&
		    (!in_migration_job || imig_source_current(conv_ftl, &ppa_copy))) {
			gcacct_move++;
			gc_write_page_iot(conv_ftl, &ppa_copy, iot);
		}

		ppa_copy.g.pg++;
	}

	return cnt;
}

static void mark_line_free(struct conv_ftl *conv_ftl, struct ppa *ppa)
{
	struct line_mgmt *lm = &conv_ftl->lm;
	struct line *line = get_line(conv_ftl, ppa);
	line->ipc = 0;
	line->vpc = 0;

	line->rd_cnt = 0;
	line->last_rd_ns = 0;

	if (line->mig_pos) {
		pqueue_remove(conv_ftl->migrating_line_pq, line);
		line->mig_pos = 0;
		mig_removed++;
	}
	line->mig_order = 0;

	line->inflight = 0;
	line->free_seq = ++line_free_seq;
	list_add_tail(&line->entry, pool_free_list(conv_ftl, line_pool_of(line)));
	if (line->mode == LINE_SLC) {
		lm->free_slc_line_cnt++;
		slcline_reclaimed++;
		if (lm->free_slc_line_cnt > slcline_free_hwm)
			slcline_free_hwm = lm->free_slc_line_cnt;
	} else {
		lm->free_line_cnt++;
	}
}

static void maybe_do_read_reclaim_for_namespace(struct conv_ftl *conv_ftls, uint32_t nr_parts)
{
	(void)conv_ftls;
	(void)nr_parts;
}

static int do_gc_pool(struct conv_ftl *conv_ftl, bool force, int pool, bool to_tlc)
{
	struct line *victim_line = NULL;
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	struct ppa ppa;
	int flashpg;
	int nflashpg;
	uint32_t gc_iot;

	victim_line = select_victim_line_pool(conv_ftl, force, pool);
	if (!victim_line) {
		return -1;
	}

	gc_iot = GC_IO;
	if (victim_line->mode == LINE_SLC)
		gc_iot |= IOT_SLC_BIT;

	if (to_tlc && (gc_iot & IOT_SLC_BIT)) {
		gc_iot &= ~IOT_SLC_BIT;
	}

	ppa.g.blk = victim_line->id;
	NVMEV_DEBUG_VERBOSE("GC-ing line:%d,ipc=%d(%d),victim=%d,full=%d,free=%d\n", ppa.g.blk,
		    victim_line->ipc, victim_line->vpc, conv_ftl->lm.victim_line_cnt,
		    conv_ftl->lm.full_line_cnt, conv_ftl->lm.free_line_cnt);

	conv_ftl->wfc.credits_to_refill = victim_line->ipc;

	nflashpg = (victim_line->mode == LINE_SLC) ? spp->flashpgs_slc_per_blk
						  : spp->flashpgs_per_blk;

	for (flashpg = 0; flashpg < nflashpg; flashpg++) {
		int ch, lun, pl;

		ppa.g.pg = flashpg * spp->pgs_per_flashpg;
		for (ch = 0; ch < spp->nchs; ch++) {
			for (lun = 0; lun < spp->luns_per_ch; lun++) {
				for (pl = 0; pl < spp->pls_per_lun; pl++) {
					struct nand_lun *lunp;

					ppa.g.ch = ch;
					ppa.g.lun = lun;
					ppa.g.pl = pl;
					lunp = get_lun(conv_ftl->ssd, &ppa);

					if (block_has_nonfree_pages(conv_ftl, &ppa))
						clean_one_flashpg_iot(conv_ftl, &ppa, gc_iot);

					if (flashpg == (nflashpg - 1) &&
					    block_has_nonfree_pages(conv_ftl, &ppa)) {
						struct convparams *cpp = &conv_ftl->cp;

						mark_block_free(conv_ftl, &ppa);

						if (cpp->enable_gc_delay) {
							struct nand_cmd gce = {
								.type = GC_IO,
								.cmd = NAND_ERASE,
								.stime = 0,
								.interleave_pci_dma = false,
								.ppa = &ppa,

								.fold = get_line(conv_ftl, &ppa)->mode == LINE_SLC,
							};

							gcacct_erase++;
							ssd_advance_nand(conv_ftl->ssd, &gce);
						}

						lunp->gc_endtime = lunp->next_lun_avail_time;
					}
				}
			}
		}
	}

	mark_line_free(conv_ftl, &ppa);

	return 0;
}

static int do_gc(struct conv_ftl *conv_ftl, bool force)
{
	return do_gc_pool(conv_ftl, force, TLC_POOL, false);
}

u64 rawp_pick_n;
u64 rawp_vpc0_hit;
u64 rawp_scan_sum;
u64 mig_erase2_n;
u64 wgc2_mig_fail_n;

enum rplace_action {
	RPLACE_LEGACY = 0,
	RPLACE_SLC,
	RPLACE_TLC_COLD,
	RPLACE_TLC_LARGE,
	RPLACE_TLC_PRESSURE,
	RPLACE_TLC_MIG_COST,
	RPLACE_TLC_NO_SLC,
	RPLACE_SLC_COLD,
};

enum { TMG_OFF = 0, TMG_MIGRATE = 1, TMG_SKIP = 2 };

#ifndef GC_PACE_STRIPES
#define GC_PACE_STRIPES 1
#endif

#ifndef GC_PACE_HEADROOM

#define GC_PACE_HEADROOM 4
#endif

static void foreground_gc(struct conv_ftl *conv_ftl, bool tick)
{
	bool need_gc;

	if (tick)

		need_gc = (int)(conv_ftl->lm.free_line_cnt + conv_ftl->lm.free_slc_line_cnt) <
			  gc_target_lines;
	else {
		need_gc = should_gc_high(conv_ftl);
	}

	if (need_gc) {
		u64 t0 = ktime_get_ns(), dt;
		NVMEV_DEBUG_VERBOSE("should_gc_high passed");

		do_gc(conv_ftl, true);
		dt = ktime_get_ns() - t0;
		if (dt > tetris_fgc_ns_max) {
			tetris_fgc_ns_max = dt;
		}
	}
}

static bool is_same_flash_page(struct conv_ftl *conv_ftl, struct ppa ppa1, struct ppa ppa2)
{
	struct ssdparams *spp = &conv_ftl->ssd->sp;
	uint32_t ppa1_page = ppa1.g.pg / spp->pgs_per_flashpg;
	uint32_t ppa2_page = ppa2.g.pg / spp->pgs_per_flashpg;

	return (ppa1.h.blk_in_ssd == ppa2.h.blk_in_ssd) && (ppa1_page == ppa2_page);
}

#define LR_ADD(done) do { nsecs_latest = max((done), nsecs_latest); \
	if ((done) < nsecs_first) nsecs_first = (done); nsub++; } while (0)

struct scan_stream tetris_streams[TETRIS_MAX_STREAMS];
struct scan_sid tetris_sids[TETRIS_MAX_SIDS];

static bool conv_read(struct nvmev_ns *ns, struct nvmev_request *req, struct nvmev_result *ret)
{
	struct conv_ftl *conv_ftls = (struct conv_ftl *)ns->ftls;
	struct conv_ftl *conv_ftl = &conv_ftls[0];

	struct ssdparams *spp = &conv_ftl->ssd->sp;

	struct nvme_command *cmd = req->cmd;
	uint64_t lba = cmd->rw.slba;
	uint64_t nr_lba = (cmd->rw.length + 1);
	uint64_t start_lpn = lba / spp->secs_per_pg;
	uint64_t end_lpn = (lba + nr_lba - 1) / spp->secs_per_pg;
	uint64_t ra_start = start_lpn;
	uint64_t lpn;
	uint64_t chunk_lpn = 0;
	uint8_t chunk_mask;
	uint64_t nsecs_start = req->nsecs_start;
	uint64_t nsecs_completed, nsecs_latest = nsecs_start;
	uint64_t nsecs_first = ~0ULL, lr_base = nsecs_start;
	uint32_t nsub = 0;
	uint32_t xfer_size, i;
	uint32_t nr_parts = ns->nr_parts;
	int stream_slot = -1;
	u8 raw_tn;

	struct ppa prev_ppa;
	struct nand_cmd srd = {
		.type = USER_IO,
		.cmd = NAND_READ,
		.stime = nsecs_start,
		.interleave_pci_dma = true,
	};

	NVMEV_ASSERT(conv_ftls);
	NVMEV_DEBUG_VERBOSE("%s: start_lpn=%lld, len=%lld, end_lpn=%lld", __func__, start_lpn, nr_lba, end_lpn);
	if ((end_lpn / nr_parts) >= spp->tt_pgs) {
		NVMEV_ERROR("%s: lpn passed FTL range (start_lpn=%lld > tt_pgs=%ld)\n", __func__,
			    start_lpn, spp->tt_pgs);
		return false;
	}

	{
		bool size_large = (LBA_TO_BYTE(nr_lba) > TETRIS_SMALL_READ_BYTES);

		srd.is_small_read = !size_large;
		srd.seq_hint = 0;
		srd.size_small = !size_large;

		srd.host_bytes = (uint32_t)LBA_TO_BYTE(nr_lba);
		tetris_hostrd_bytes += LBA_TO_BYTE(nr_lba);
	}

	srd.stream_id = (stream_slot >= 0) ? (uint16_t)(stream_slot + 1) : 0;

	srd.host_seq = (uint32_t)atomic_inc_return(&tetris_host_seq);
	raw_tn = lpn_to_tenant(ra_start);
	srd.tn = (uint8_t)raw_tn;

	ssd_optrace_record_host(srd.host_seq, nsecs_start, srd.tn, srd.is_small_read ? 0 : 1,
				(uint32_t)LBA_TO_BYTE(nr_lba));

	srd.sq_id = (uint16_t)req->sq_id;

	if (LBA_TO_BYTE(nr_lba) <= (KB(4) * nr_parts)) {
		srd.stime += spp->fw_4kb_rd_lat;
	} else {
		srd.stime += spp->fw_rd_lat;
	}
	lr_base = srd.stime;

	for (i = 0; (i < nr_parts) && (start_lpn <= end_lpn); i++, start_lpn++) {
		conv_ftl = &conv_ftls[start_lpn % nr_parts];
		xfer_size = 0;
		prev_ppa = get_maptbl_ent(conv_ftl, start_lpn / nr_parts);
		chunk_lpn = start_lpn;
		chunk_mask = 0;

		for (lpn = start_lpn; lpn <= end_lpn; lpn += nr_parts) {
			uint64_t local_lpn;
			struct ppa cur_ppa;

			local_lpn = lpn / nr_parts;
			cur_ppa = get_maptbl_ent(conv_ftl, local_lpn);

			if (!mapped_ppa(&cur_ppa) || (!ppa_is_virtual(&cur_ppa) && !valid_ppa(conv_ftl, &cur_ppa))) {
				NVMEV_DEBUG_VERBOSE("lpn 0x%llx not mapped to valid ppa\n", local_lpn);
				NVMEV_DEBUG_VERBOSE("Invalid ppa,ch:%d,lun:%d,blk:%d,pl:%d,pg:%d\n",
					    cur_ppa.g.ch, cur_ppa.g.lun, cur_ppa.g.blk,
					    cur_ppa.g.pl, cur_ppa.g.pg);
				continue;
			}

			if (mapped_ppa(&prev_ppa) &&
			    is_same_flash_page(conv_ftl, cur_ppa, prev_ppa)) {
				xfer_size += spp->pgsz;
				chunk_mask |= 1u << (cur_ppa.g.pg % spp->pgs_per_flashpg);
				continue;
			}

			if (xfer_size > 0) {
				nsecs_completed = conv_issue_read_chunk(conv_ftl, &srd, &prev_ppa,
									xfer_size, chunk_lpn, nr_parts,
									chunk_mask);
				LR_ADD(nsecs_completed);
			}

			xfer_size = spp->pgsz;
			prev_ppa = cur_ppa;
			chunk_lpn = lpn;
			chunk_mask = 1u << (cur_ppa.g.pg % spp->pgs_per_flashpg);
		}

		if (xfer_size > 0) {
			nsecs_completed = conv_issue_read_chunk(conv_ftl, &srd, &prev_ppa,
								xfer_size, chunk_lpn, nr_parts, chunk_mask);
			LR_ADD(nsecs_completed);
		}
	}

	maybe_do_read_reclaim_for_namespace(conv_ftls, nr_parts);

	ret->nsecs_target = nsecs_latest;
	ret->status = NVME_SC_SUCCESS;
	return true;
}

#undef LR_ADD
static bool conv_write(struct nvmev_ns *ns, struct nvmev_request *req, struct nvmev_result *ret)
{
	struct conv_ftl *conv_ftls = (struct conv_ftl *)ns->ftls;
	struct conv_ftl *conv_ftl = &conv_ftls[0];

	struct ssdparams *spp = &conv_ftl->ssd->sp;

	struct nvme_command *cmd = req->cmd;
	uint64_t lba = cmd->rw.slba;
	uint64_t nr_lba = (cmd->rw.length + 1);
	uint64_t start_lpn = lba / spp->secs_per_pg;
	uint64_t end_lpn = (lba + nr_lba - 1) / spp->secs_per_pg;

	uint64_t lpn;
	uint32_t nr_parts = ns->nr_parts;
	uint32_t grp_slc_bit;
	bool lb_split;

	uint64_t nsecs_latest;
	uint64_t nsecs_xfer_completed;

	uint16_t _wstream = rw_stream_id(&cmd->rw);
	uint64_t xfer_base;

	struct nand_cmd swr = {
		.type = USER_IO,
		.cmd = NAND_WRITE,
		.interleave_pci_dma = false,
		.xfer_size = spp->pgsz * spp->pgs_per_oneshotpg,
	};

	NVMEV_DEBUG_VERBOSE("%s: start_lpn=%lld, len=%lld, end_lpn=%lld", __func__, start_lpn, nr_lba, end_lpn);
	if ((end_lpn / nr_parts) >= spp->tt_pgs) {
		NVMEV_ERROR("%s: lpn passed FTL range (start_lpn=%lld > tt_pgs=%ld)\n",
				__func__, start_lpn, spp->tt_pgs);
		return false;
	}

	if (le16_to_cpu(cmd->rw.control) & NVME_RW_DTYPE_STREAMS)
		;

	if (_wstream == 0)
		_wstream = (uint16_t)lpn_to_tenant(start_lpn);

	gcacct_hostwr += end_lpn - start_lpn + 1;

	{
			int r;

			if (host_free_lines(conv_ftl, SLC_POOL) <= 1) {
				do_gc_pool(conv_ftl,
					   (int)conv_ftl->lm.free_slc_line_cnt <= gc_force_lines,
					   SLC_POOL, false);
				wgc2_slcgc_n++;
				wgc2_gcsweep_n++;
			}

			if (host_free_lines(conv_ftl, TLC_POOL) <= 2) {
				for (r = 0; r < gc_retry_max &&
					    host_free_lines(conv_ftl, TLC_POOL) < 1; r++)
					do_gc_pool(conv_ftl, true, TLC_POOL, false);
				if (host_free_lines(conv_ftl, TLC_POOL) < 1)
					wgc2_gc_retry_hit++;
				wgc2_tlcgc_n++;
			}

		nsecs_latest =
			ssd_advance_write_buffer(conv_ftl->ssd, req->nsecs_start, LBA_TO_BYTE(nr_lba));
		nsecs_xfer_completed = nsecs_latest;
		xfer_base = nsecs_xfer_completed;

		swr.stime = nsecs_latest;
		swr.host_seq = (uint32_t)atomic_inc_return(&tetris_host_seq);
		swr.sq_id = (uint16_t)req->sq_id;
		swr.owner = nvmev_cur_cmd;

		grp_slc_bit = 0;
		lb_split = false;
		for (lpn = start_lpn; lpn <= end_lpn; lpn++) {
			uint64_t local_lpn;
			uint64_t nsecs_completed = 0;
			struct ppa ppa;
			struct ppa vold;
			uint32_t io_tn;
			bool inpl;

			conv_ftl = &conv_ftls[lpn % nr_parts];
			local_lpn = lpn / nr_parts;

			inpl = lb_on() && lb_in_dirty(conv_ftl, local_lpn, &vold);
			if (lb_on() && (lpn % CE_SLOTS) == 0 && conv_grp_full(lpn, start_lpn, end_lpn)) {
				struct ppa t;
				uint64_t k;

				lb_split = false;
				for (k = 0; k < CE_SLOTS; k++) {
					if (lb_in_dirty(&conv_ftls[(lpn + k) % nr_parts], (lpn + k) / nr_parts, &t))
						lb_split = true;
				}
			}
			ppa = get_maptbl_ent(
				conv_ftl, local_lpn);
			if (!inpl && mapped_ppa(&ppa)) {
				if (ppa_is_virtual(&ppa)) {
					lb_vinval(&ppa);
				} else {
					mark_page_invalid(conv_ftl, &ppa);
					set_rmap_ent(conv_ftl, INVALID_LPN, &ppa);
				}

				if (rand_split_on()) {
					if (dcache_rand_drop(ce_pgkey(&ppa), ppa.g.pg % CE_SLOTS, lpn)) {
						nvmev_diq_full_wake(req->nsecs_start);
					}
				}
				NVMEV_DEBUG("%s: %lld is invalid, ", __func__, ppa2pgidx(conv_ftl, &ppa));
			}

			{
				uint8_t tn = (_wstream >= 1 && _wstream <= TENANT_N)
						     ? (uint8_t)_wstream : 0;

				io_tn = USER_IO | ((uint32_t)tn << IOT_TN_SHIFT);
			}

			if (rand_split_on() && !inpl &&
			    (!conv_grp_full(lpn, start_lpn, end_lpn) || (lb_on() && lb_split))) {
				io_tn |= IOT_RAND_BIT;
			}

			if (lb_on()) {
			} else if (rand_split_on() && !(io_tn & IOT_RAND_BIT) && (lpn % CE_SLOTS) != 0) {
				io_tn |= grp_slc_bit;
			} else {
				struct write_pointer *swp = host_slc_base_wp(conv_ftl, io_tn);
				int hfree = host_free_lines(conv_ftl, SLC_POOL);
				bool room = wp_has_room(conv_ftl, swp);

				bool want_slc = swp->curline && hfree >= 1 &&
						(hfree > 1 || room);

				if (want_slc && pslc_raw_reserve && slc_free_pgs(conv_ftl) <= pslc_reserve_pgs())
					want_slc = false;

				if (swp->curline && !want_slc)
					wgc2_tlc_redirect++;
				else if (want_slc && room && hfree <= 1)
					wgc2_slc_keep++;
				if (swp->curline && hfree < 1)
					wgc2_slc_floor++;
				if (want_slc)
					io_tn |= IOT_SLC_BIT;
			}

			if (rand_split_on() && !(io_tn & IOT_RAND_BIT) && (lpn % CE_SLOTS) == 0) {
				if (!lb_on()) {
					if ((io_tn & IOT_SLC_BIT) && !conv_slc_grp_fits(conv_ftl, io_tn)) {
						io_tn &= ~IOT_SLC_BIT;
					}
					grp_slc_bit = io_tn & IOT_SLC_BIT;
				}
			}
			if (inpl) {
				ppa = vold;
				lpn_tenant_set(conv_ftl, local_lpn,
					       (_wstream >= 1 && _wstream <= TENANT_N) ? (u8)_wstream : 0);
			} else if (lb_on()) {
				ppa = lb_vget(io_tn);
				set_maptbl_ent(conv_ftl, local_lpn, &ppa);
				lb_vset(&ppa, local_lpn);
				lpn_tenant_set(conv_ftl, local_lpn,
					       (_wstream >= 1 && _wstream <= TENANT_N) ? (u8)_wstream : 0);
			} else {
				pool_host_pgs[(io_tn & IOT_SLC_BIT) ? SLC_POOL : TLC_POOL]++;
				ppa = get_new_page(conv_ftl, io_tn);
				if (io_tn & IOT_SLC_BIT)
					slcline_host_pgs++;

				set_maptbl_ent(conv_ftl, local_lpn, &ppa);
				NVMEV_DEBUG("%s: got new ppa %lld, ", __func__, ppa2pgidx(conv_ftl, &ppa));

				set_rmap_ent(conv_ftl, local_lpn, &ppa);

				mark_page_valid(conv_ftl, &ppa,
					(_wstream >= 1 && _wstream <= TENANT_N) ? (int)_wstream : 0);

				advance_write_pointer(conv_ftl, io_tn);
			}

			if (inpl || (io_tn & IOT_RAND_BIT) || last_pg_in_wordline(conv_ftl, &ppa)) {
				if (dcache_get()) {
					uint64_t done;

					done = conv_cache_write_wl(conv_ftl, &ppa, lpn, nr_parts, xfer_base,
								   req->nsecs_start, swr.host_seq,

								   (uint8_t)((io_tn >> IOT_TN_SHIFT) & 0xFF),
								   inpl || (io_tn & IOT_RAND_BIT) != 0, inpl);

					nsecs_xfer_completed = max(nsecs_xfer_completed, done);
					nsecs_latest = max(nsecs_latest, done);
				} else {
					swr.ppa = &ppa;

					nsecs_completed = ssd_advance_nand(conv_ftl->ssd, &swr);
					nsecs_latest = max(nsecs_completed, nsecs_latest);
				}
			}

			consume_write_credit(conv_ftl);
			check_and_refill_write_credit(conv_ftl);
		}
	}

	ret->nsecs_target = nsecs_latest;
	ret->status = NVME_SC_SUCCESS;

	return true;
}

static void conv_flush(struct nvmev_ns *ns, struct nvmev_request *req, struct nvmev_result *ret)
{
	uint64_t start, latest;
	uint32_t i;
	struct conv_ftl *conv_ftls = (struct conv_ftl *)ns->ftls;

	start = local_clock();
	latest = start;
	(void)conv_ftls; (void)i;

	dcache_flush_sync(start, nvmev_cur_cmd);

	ret->status = NVME_SC_SUCCESS;
	ret->nsecs_target = latest;
	return;
}

void conv_outbox_drain(struct nvmev_ns *ns)
{
	struct conv_ftl *conv_ftls = (struct conv_ftl *)ns->ftls;
	uint32_t i;

	for (i = 0; i < ns->nr_parts; i++)
		lunq3_outbox_drain(conv_ftls[i].ssd, nvmev_apply_completion);
}

static struct conv_ftl *g_qserve_ftl;

u8 conv_qserve_tenant_of_ppa(struct ppa *ppa)
{
	uint64_t lpn;

	if (!g_qserve_ftl || !mapped_ppa(ppa))
		return 0;
	lpn = get_rmap_ent(g_qserve_ftl, ppa);
	if (lpn == INVALID_LPN)
		return 0;
	return (u8)lpn_to_tenant(lpn);
}

void conv_gc_tick(struct nvmev_ns *ns)
{
	struct conv_ftl *conv_ftls = (struct conv_ftl *)ns->ftls;
	const uint32_t nr_parts = SSD_PARTITIONS;
	uint32_t i;
	int free_min;

	free_min = INT_MAX;
	for (i = 0; i < nr_parts; i++) {
		struct conv_ftl *c = &conv_ftls[i];
		int free_total;

		foreground_gc(c, true);

		free_total = (int)(c->lm.free_line_cnt + c->lm.free_slc_line_cnt);
		if (free_total < free_min)
			free_min = free_total;
	}

	WRITE_ONCE(tetris_free_lines, free_min);

	if (free_min <= gc_reserve_lines + 1) {
		WRITE_ONCE(backpressure_free_line_stall, 1);
	} else if (free_min >= gc_reserve_lines + 8) {
		WRITE_ONCE(backpressure_free_line_stall, 0);
	}
	if (backpressure_free_line_stall) {
		bp_fl_cur++;
	} else if (bp_fl_cur) {
		bp_fl_cur = 0;
	}
	if (free_min < bp_fl_free_lo)
		bp_fl_free_lo = free_min;
}

void conv_qserve_pass(struct nvmev_ns *ns, unsigned long long now)
{
	struct conv_ftl *conv_ftls = (struct conv_ftl *)ns->ftls;
	uint32_t i;

	for (i = 0; i < ns->nr_parts; i++) {
		g_qserve_ftl = &conv_ftls[i];
		ssd_qserve_pass(conv_ftls[i].ssd, now);
	}
	g_qserve_ftl = NULL;
}

bool conv_proc_nvme_io_cmd(struct nvmev_ns *ns, struct nvmev_request *req, struct nvmev_result *ret)
{
	struct nvme_command *cmd = req->cmd;

	NVMEV_ASSERT(ns->csi == NVME_CSI_NVM);

	switch (cmd->common.opcode) {
	case nvme_cmd_write:
		if (!conv_write(ns, req, ret))
			return false;
		break;
	case nvme_cmd_read:
		if (!conv_read(ns, req, ret))
			return false;
		break;
	case nvme_cmd_flush:
		conv_flush(ns, req, ret);
		break;
	default:
		NVMEV_ERROR("%s: command not implemented: %s (0x%x)\n", __func__,
				nvme_opcode_string(cmd->common.opcode), cmd->common.opcode);
		break;
	}

	return true;
}
