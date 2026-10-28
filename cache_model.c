// SPDX-License-Identifier: GPL-2.0-only

#include <linux/vmalloc.h>
#include "nvmev.h"
#include "cache_model.h"

#define STAGE_INFLIGHT (~0ULL)

static inline void occ_ewma_note(struct cache_model *c, uint32_t v)
{
	uint32_t r = (rawp_occ_r_ppk >= 0 && rawp_occ_r_ppk < 1000) ?
			     (uint32_t)rawp_occ_r_ppk : 900;

	c->occ_ewma_x100 = (c->occ_ewma_x100 * r + (uint64_t)v * 100 * (1000 - r)) / 1000;
}

static struct cache_model *g_dcache;
static struct cache_ent *lookup_pg(struct cache_model *c, uint64_t pgkey);

static inline void ce_gen_next(struct cache_ent *e)
{
	e->gen++;
	if (DFW_GEN(e->gen) == 0)
		e->gen++;
}

static struct dflush_wait dfw_pool[DFW_POOL];
static uint16_t dfw_free_head;
static uint32_t dfw_live;

static void dfw_init(void)
{
	uint32_t i;

	for (i = 0; i < DFW_POOL; i++)
		dfw_pool[i].next = (i + 1 < DFW_POOL) ? (uint16_t)(i + 1) : DFW_NIL;
	dfw_free_head = 0;
	dfw_live = 0;
}

static uint16_t dfw_alloc(struct cache_model *c)
{
	uint16_t i = dfw_free_head;

	if (i == DFW_NIL) {
		return DFW_NIL;
	}
	dfw_free_head = dfw_pool[i].next;
	if (++dfw_live > c->st.dfw_hwm)
		c->st.dfw_hwm = dfw_live;
	return i;
}

static void dfw_free(uint16_t i)
{
	dfw_pool[i].next = dfw_free_head;
	dfw_free_head = i;
	dfw_live--;
}

#define FBH_POOL 4096
struct flush_budget_hold {
	uint16_t next;
	uint16_t die;
	uint32_t gen;
	uint64_t release_at;
};
static struct flush_budget_hold fbh_pool[FBH_POOL];
static uint16_t fbh_free_head;
static uint16_t fbh_timed_head;
static uint32_t fbh_live;

static void fbh_init(void)
{
	uint32_t i;

	for (i = 0; i < FBH_POOL; i++)
		fbh_pool[i].next = (i + 1 < FBH_POOL) ? (uint16_t)(i + 1) : DFW_NIL;
	fbh_free_head = 0;
	fbh_timed_head = DFW_NIL;
	fbh_live = 0;
}

static uint16_t fbh_alloc(struct cache_model *c)
{
	uint16_t i;

	i = fbh_free_head;
	if (i == DFW_NIL) {
		c->st.fbh_full++;
		return DFW_NIL;
	}
	fbh_free_head = fbh_pool[i].next;
	fbh_live++;
	if (fbh_live > c->st.fbh_hwm)
		c->st.fbh_hwm = fbh_live;
	return i;
}

static void fbh_free(uint16_t i)
{
	fbh_pool[i].next = fbh_free_head;
	fbh_free_head = i;
	fbh_live--;
}

static void fbh_on_done(struct cache_model *c, struct cache_ent *e, uint32_t gen, uint64_t done)
{
	uint16_t *pp;
	uint16_t h;

	pp = &e->hold_head;
	while (*pp != DFW_NIL) {
		h = *pp;
		if (fbh_pool[h].gen == gen) {
			*pp = fbh_pool[h].next;
			fbh_pool[h].release_at = done ? done : 1;
			fbh_pool[h].next = fbh_timed_head;
			fbh_timed_head = h;
			c->st.fbh_cb++;
			return;
		}
		pp = &fbh_pool[h].next;
	}
}

void dcache_flush_done(uint32_t frame, uint64_t done,
		       void (*release)(uint32_t ticket, uint64_t done))
{
	struct cache_model *c = g_dcache;
	uint32_t idx, gen;
	struct cache_ent *e;
	uint16_t *pp;
	uint64_t chain = 0;

	if (!c || !frame)
		return;
	idx = frame & 0xFFFFu;
	gen = frame >> 16;
	if (idx >= c->nr_ents)
		return;
	e = &c->ents[idx];

	spin_lock(&c->lock);

	if (DFW_GEN(e->gen) == gen) {
		if (e->state == CE_DIRTY_ISSUED && e->await) {
			if (done > e->clean_at) {
				uint64_t r = done - e->clean_at;

				c->st.fh_raise_n++;
				c->st.fh_raise_sum += r;
				if (r > c->st.fh_raise_max)
					c->st.fh_raise_max = r;
			}
			e->clean_at = done;
			e->await = 0;
		} else if (e->await) {
			e->await = 0;
		}
	} else {
		fbh_on_done(c, e, gen, done);
	}

	pp = &e->wait_head;
	while (*pp != DFW_NIL) {
		struct dflush_wait *w = &dfw_pool[*pp];
		uint16_t self = *pp;

		chain++;
		if (DFW_GEN(w->gen) == gen) {
			*pp = w->next;
			dfw_free(self);
			c->st.dfw_rel++;
			spin_unlock(&c->lock);
			release(w->ticket, done);
			spin_lock(&c->lock);
			continue;
		}
		pp = &w->next;
	}
	if (chain > c->st.dfw_chain_max)
		c->st.dfw_chain_max = chain;
	spin_unlock(&c->lock);
}

int dcache_fill_done(uint32_t fill, uint64_t done, void (*release)(uint32_t ticket, uint64_t done))
{
	struct cache_model *c;
	struct cache_ent *e;
	struct dflush_wait *w;
	uint16_t *pp;
	uint16_t self;
	uint32_t ticket;
	uint32_t idx;
	uint32_t gen;
	int rc;

	c = g_dcache;
	if (!c || !fill)
		return -1;
	idx = fill & 0xFFFFu;
	gen = fill >> 16;
	if (idx >= c->nr_ents)
		return -1;
	e = &c->ents[idx];

	spin_lock(&c->lock);
	rc = -1;
	if (DFW_GEN(e->gen) == gen && e->state == CE_LOADING && !e->wload) {
		rc = 0;
		e->vmask |= e->fmask;
		e->fmask = 0;
		e->state = CE_CLEAN;
		e->clean_at = done;
		c->clean_cnt++;
		c->st.lba_fill_done++;
	} else {
		c->st.lba_fill_stale++;
	}
	pp = &e->wait_head;
	while (*pp != DFW_NIL) {
		self = *pp;
		w = &dfw_pool[self];
		if (DFW_GEN(w->gen) == gen) {
			*pp = w->next;
			ticket = w->ticket;
			dfw_free(self);
			c->st.dfw_rel++;
			spin_unlock(&c->lock);
			release(ticket, done);
			spin_lock(&c->lock);
			continue;
		}
		pp = &w->next;
	}
	spin_unlock(&c->lock);
	return rc;
}

struct cache_model *dcache_get(void)
{
	return g_dcache;
}

static inline struct ppa *ce_die_ppa(struct cache_ent *e, struct ppa *tmp)
{
	if (!ppa_is_virtual(&e->ppa))
		return &e->ppa;
	conv_bind_peek(e->ssd, &e->ppa, tmp);
	return tmp;
}

int dcache_frame_die(uint32_t frame)
{
	struct cache_model *c;
	struct ppa dp;
	uint32_t idx;

	c = g_dcache;
	if (!c || frame == CE_WAIT_FULL)
		return -1;
	idx = frame & 0xFFFFu;
	if (idx >= c->nr_ents)
		return -1;
	return ppa_die_id(ce_die_ppa(&c->ents[idx], &dp));
}

int dcache_frame_describe(uint32_t frame, char *buf, int len)
{
	struct cache_model *c;
	struct cache_ent *e;
	uint32_t idx;
	uint32_t gen;

	c = g_dcache;
	if (!c)
		return scnprintf(buf, len, "no cache");
	if (frame == CE_WAIT_FULL)
		return scnprintf(buf, len, "WAIT_FULL(wait queue)");
	idx = frame & 0xFFFFu;
	gen = frame >> 16;
	if (idx >= c->nr_ents)
		return scnprintf(buf, len, "ent=%u out of range (nr_ents=%llu)", idx, c->nr_ents);
	e = &c->ents[idx];
	return scnprintf(buf, len,
			 "ent=%u gen frame/now=%u/%u st=%s wload=%u await=%u vm=%x fm=%x dfw=%s"
			 " lba=%lld,%lld,%lld,%lld ready_at=%llu clean_at=%llu",
			 idx, gen, DFW_GEN(e->gen), ce_st_name(e->state), e->wload, e->await, e->vmask,
			 e->fmask, e->wait_head != DFW_NIL ? "yes" : "no", (long long)e->lba[0],
			 (long long)e->lba[1], (long long)e->lba[2], (long long)e->lba[3], e->ready_at,
			 e->clean_at);
}

void dcache_dump_census(const char *why)
{
	struct cache_model *c;
	struct cache_ent *e;
	uint64_t i;
	uint32_t n[6];
	uint32_t n_await;
	uint32_t k;

	c = g_dcache;
	if (!c)
		return;
	for (k = 0; k < 6; k++)
		n[k] = 0;
	n_await = 0;
	for (i = 0; i < c->nr_ents; i++) {
		e = &c->ents[i];
		if (e->state == CE_LOADING)
			n[e->wload ? 5 : 4]++;
		else if (e->state <= CE_DIRTY_ISSUED)
			n[e->state]++;
		if (e->state == CE_DIRTY_ISSUED && e->await)
			n_await++;
	}
}

bool nvmev_diq_mig_hint;

u64 tstage_wr_n, tstage_wr_sum, tstage_wr_max, tstage_wr_nodirty;
u64 tstage_wr_backwards, tstage_wr_back_sum, tstage_wr_back_max;
u64 tstage_wr_hist[8];
u64 tstage_wr_n_tn[TENANT_N + 1], tstage_wr_sum_tn[TENANT_N + 1];

static inline int tstage_bucket(u64 ns)
{
	u64 us;

	us = ns / 1000;
	if (us < 10)     return 0;
	if (us < 100)    return 1;
	if (us < 500)    return 2;
	if (us < 1000)   return 3;
	if (us < 5000)   return 4;
	if (us < 20000)  return 5;
	if (us < 100000) return 6;
	return 7;
}

void dcache_init(void)
{
	struct cache_model *c;
	uint64_t nr, i;

	if (cache_mb <= 0 || g_dcache)
		return;
	nr = ((uint64_t)cache_mb << 20) / CACHE_ENTRY_BYTES;
	c = vzalloc(sizeof(*c));
	if (!c)
		return;
	c->ents = vzalloc(sizeof(struct cache_ent) * nr);
	if (!c->ents) {
		vfree(c);
		return;
	}
	c->nr_ents = nr;
	INIT_LIST_HEAD(&c->lru_list);
	INIT_LIST_HEAD(&c->dirty_list);
	INIT_LIST_HEAD(&c->free_list);
	INIT_LIST_HEAD(&c->rand_open);
	INIT_LIST_HEAD(&c->issued_list);
	for (i = 0; i < TETRIS_NR_DIES; i++)
		INIT_LIST_HEAD(&c->dirty_die[i]);
	hash_init(c->map);
	hash_init(c->pmap);
	spin_lock_init(&c->lock);
	dfw_init();
	fbh_init();
	for (i = 0; i < nr; i++) {
		INIT_LIST_HEAD(&c->ents[i].dnode);
		INIT_LIST_HEAD(&c->ents[i].gnode);
		list_add_tail(&c->ents[i].lru, &c->free_list);
		c->ents[i].wait_head = DFW_NIL;
		c->ents[i].hold_head = DFW_NIL;
		c->ents[i].gen = 1;
	}

	if (nr > 0xFFFF) {
		NVMEV_ERROR("dcache: nr_ents=%llu > 65535 -- frame-handle completion disabled "
			    "(lower cache_mb to <= 1024)\n", nr);
		c->frame_handle_on = 0;
	} else {
		c->frame_handle_on = 1;
	}

	if (!c->stage_cap) {
		uint32_t n = rawp_sh_slots > 0 ? (uint32_t)rawp_sh_slots : SHSTAGE_N_DEF;

		if (n & (n - 1))
			n = SHSTAGE_N_DEF;
		c->sh_release = vzalloc(sizeof(uint64_t) * n);
		if (c->sh_release) {
			c->sh_n = n;
			c->sh_mask = n - 1;
			NVMEV_INFO("dcache: shadow staging ring %u slots sweep=%d (policy=%d, staging %s)\n",
				   n, rawp_sh_sweep > 0 ? rawp_sh_sweep : SHSTAGE_SWEEP_DEF,
				   1, c->stage_cap ? "ON" : "off");
		}
	}

	g_dcache = c;
	{
		const char *pn = "CFLRU";
		NVMEV_INFO("dcache: %d MiB, %llu x 16KB entries, policy=%d(%s) dirty_ratio=%d%% batch=%d"
			   " stage=%u/%u\n",
			   cache_mb, (unsigned long long)nr, 1, pn, cache_dirty_ratio,
			   cache_flush_batch, c->stage_cur, c->stage_cap);

		NVMEV_INFO("dcache: lba_check=1 (entry = copy of a physical page; LBA slots are set only at outbox completion)\n");
		NVMEV_INFO("dcache: rand_entry=1 (writes that do not cover a whole LBA group use a dedicated random host write pointer)\n");
	}
}

void dcache_exit(void)
{
	struct cache_model *c = g_dcache;

	if (!c)
		return;
	g_dcache = NULL;
	vfree(c->stage_owner);
	vfree(c->stage_release);
	vfree(c->sh_release);
	vfree(c->ents);
	vfree(c);
}

static inline int cm_clean_bkt(uint64_t v)
{
	int b = 0;

	while (v && b < CM_CLEAN_BKTS - 1) {
		v >>= 1;
		b++;
	}
	return b;
}

void dcache_reset(void)
{
	struct cache_model *c = g_dcache;

	if (!c)
		return;
	spin_lock(&c->lock);
	memset(&c->st, 0, sizeof(c->st));

	pslc_window_reset();
	spin_unlock(&c->lock);
}

static struct cache_ent *lookup_pg(struct cache_model *c, uint64_t pgkey)
{
	struct cache_ent *e;

	hash_for_each_possible(c->pmap, e, pnode, pgkey) {
		if (e->pgkey == pgkey)
			return e;
	}
	return NULL;
}

static bool ce_same_lbas(const struct cache_ent *e, const struct ce_page *pg)
{
	bool found;
	int n_e;
	int n_p;
	int s;
	int t;

	n_e = 0;
	n_p = 0;
	for (s = 0; s < CE_SLOTS; s++) {
		if (e->lba[s] != CE_LBA_NONE)
			n_e++;
		if (!(pg->live & (1u << s)))
			continue;
		n_p++;
		found = false;
		for (t = 0; t < CE_SLOTS; t++) {
			if (e->lba[t] == pg->lba[s]) {
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}
	return n_e == n_p;
}

static struct cache_ent *lookup_lbas(struct cache_model *c, const struct ce_page *pg, bool *tagdup)
{
	struct cache_ent *e;

	*tagdup = false;
	hash_for_each_possible(c->map, e, hnode, pg->tag) {
		if (e->tag != pg->tag)
			continue;
		if (ce_same_lbas(e, pg))
			return e;
		*tagdup = true;
	}
	return NULL;
}

static void ce_pnode_set(struct cache_model *c, struct cache_ent *e, uint64_t pgkey)
{
	struct cache_ent *o;

	if (!hlist_unhashed(&e->pnode))
		hash_del(&e->pnode);
	o = lookup_pg(c, pgkey);
	if (o) {
		hash_del(&o->pnode);
		c->st.lba_pg_stale++;
		if (o->rpmask) {
			c->st.lba_rand_unhash_set++;
		}
	}
	e->pgkey = pgkey;
	hash_add(c->pmap, &e->pnode, pgkey);
}

static uint8_t ce_held_mask(const struct cache_ent *e)
{
	uint8_t m;
	int s;

	m = 0;
	for (s = 0; s < CE_SLOTS; s++) {
		if (e->lba[s] != CE_LBA_NONE)
			m |= 1u << s;
	}
	return m;
}

static int ce_held_cnt(const struct cache_ent *e)
{
	int n;
	int s;

	n = 0;
	for (s = 0; s < CE_SLOTS; s++) {
		if (e->lba[s] != CE_LBA_NONE)
			n++;
	}
	return n;
}

static void ce_rebind_rekey(struct cache_model *c, struct cache_ent *e, int held0)
{
	uint8_t m;
	int held1;
	int s;

	m = 0;
	held1 = 0;
	for (s = 0; s < CE_SLOTS; s++) {
		if (e->lba[s] == CE_LBA_NONE)
			continue;
		m |= 1u << s;
		held1++;
	}
	if (held0 > held1)
		c->st.lba_rebind_drop += held0 - held1;
	e->vmask = m;
	ce_pnode_set(c, e, ce_pgkey(&e->ppa));
}

const char *ce_st_name(uint8_t st)
{
	switch (st) {
	case CE_FREE:           return "FREE";
	case CE_CLEAN:          return "CLEAN";
	case CE_DIRTY_UNISSUED: return "DIRTY_U";
	case CE_DIRTY_ISSUED:   return "DIRTY_I";
	case CE_LOADING:        return "LOADING";
	default:                return "-";
	}
}

static void dirty_add(struct cache_model *c, struct cache_ent *e)
{
	int d = ppa_die_id(&e->ppa);

	list_add_tail(&e->gnode, &c->dirty_list);
	list_add_tail(&e->dnode, &c->dirty_die[d]);
	c->dirty_unissued++;
}

static void dirty_del(struct cache_model *c, struct cache_ent *e)
{
	list_del_init(&e->gnode);
	list_del_init(&e->dnode);
	c->dirty_unissued--;
}

enum { CE_PL_NONE = 0, CE_PL_DIRTY, CE_PL_RAND };

static inline int ce_place_want(const struct cache_ent *e)
{
	if (e->rpmask || e->lbhold)
		return CE_PL_NONE;
	return e->rpart ? CE_PL_RAND : CE_PL_DIRTY;
}

static inline int ce_place_of(const struct cache_ent *e)
{
	return e->state == CE_DIRTY_UNISSUED ? ce_place_want(e) : CE_PL_NONE;
}

static void ce_place_put(struct cache_model *c, struct cache_ent *e, int pl)
{
	if (pl == CE_PL_DIRTY) {
		dirty_add(c, e);
	} else if (pl == CE_PL_RAND) {
		list_add_tail(&e->gnode, &c->rand_open);
		c->rand_open_n++;
	}
}

static void ce_place_take(struct cache_model *c, struct cache_ent *e, int pl)
{
	if (pl == CE_PL_DIRTY) {
		dirty_del(c, e);
	} else if (pl == CE_PL_RAND) {
		list_del_init(&e->gnode);
		c->rand_open_n--;
	}
}

static void ce_place_move(struct cache_model *c, struct cache_ent *e, int p0)
{
	int p1;

	p1 = ce_place_of(e);
	if (p0 == p1)
		return;
	ce_place_take(c, e, p0);
	ce_place_put(c, e, p1);
	if (p1 == CE_PL_DIRTY && p0 == CE_PL_NONE)
		c->st.lba_rand_ready++;
	else if (p1 == CE_PL_RAND && p0 == CE_PL_NONE)
		c->st.lba_rand_ready++;
}

int dcache_rand_drop(uint64_t pgkey, uint32_t slot, uint64_t lpn)
{
	struct cache_model *c;
	struct cache_ent *e;
	uint8_t bit;
	int rc;

	c = g_dcache;
	if (!c || slot >= CE_SLOTS)
		return 0;
	rc = 0;
	bit = (uint8_t)(1u << slot);
	spin_lock(&c->lock);
	e = lookup_pg(c, pgkey);
	if (e && e->rnd && e->lba[slot] == lpn) {
		uint64_t r0;
		int pl0;

		pl0 = ce_place_of(e);
		r0 = c->st.lba_rand_ready;
		if (e->rpmask & bit)
			c->st.lba_rand_drop_pend++;
		e->lba[slot] = CE_LBA_NONE;
		e->vmask &= ~bit;
		e->fmask &= ~bit;
		e->rpmask &= ~bit;
		ce_place_move(c, e, pl0);
		rc = c->st.lba_rand_ready != r0;
		c->st.lba_rand_drop++;
	}
	spin_unlock(&c->lock);
	return rc;
}

int dcache_rand_unpend(uint64_t pgkey, uint8_t chunk)
{
	struct cache_model *c;
	struct cache_ent *e;
	uint64_t r0;
	uint8_t m;
	int pl0;
	int rc;
	int s;

	c = g_dcache;
	if (!c)
		return 0;
	rc = 0;
	spin_lock(&c->lock);
	e = lookup_pg(c, pgkey);
	if (e && e->rnd && (e->rpmask & chunk)) {
		pl0 = ce_place_of(e);
		r0 = c->st.lba_rand_ready;
		m = e->rpmask & chunk;
		for (s = 0; s < CE_SLOTS; s++) {
			if (m & (1u << s))
				e->lba[s] = CE_LBA_NONE;
		}
		e->vmask &= ~m;
		e->fmask &= ~m;
		e->rpmask &= ~m;
		ce_place_move(c, e, pl0);
		c->st.lba_rand_unpend++;
		rc = (c->st.lba_rand_ready != r0) ? 2 : 1;
	}
	spin_unlock(&c->lock);
	return rc;
}

static void dirty_rebucket(struct cache_model *c, struct cache_ent *e, struct ppa *new_ppa)
{
	int nd = ppa_die_id(new_ppa), od = ppa_die_id(&e->ppa);

	if (nd == od)
		return;
	list_move_tail(&e->dnode, &c->dirty_die[nd]);
}

static void issued_add(struct cache_model *c, struct cache_ent *e)
{
	int d = ppa_die_id(&e->ppa);

	list_add_tail(&e->dnode, &c->issued_list);
	c->issued_cnt++;
	if (++c->issued_die[d] == (uint32_t)flush_stage_k)
		c->dies_at_cap++;
}

static void issued_del(struct cache_model *c, struct cache_ent *e)
{
	int d = ppa_die_id(&e->ppa);

	list_del_init(&e->dnode);
	c->issued_cnt--;
	if (c->issued_die[d]-- == (uint32_t)flush_stage_k)
		c->dies_at_cap--;
}

static void issued_die_release(struct cache_model *c, uint32_t d)
{
	if (c->issued_die[d]-- == (uint32_t)flush_stage_k)
		c->dies_at_cap--;
}

static void fbh_sweep(struct cache_model *c, uint64_t now)
{
	uint16_t *pp;
	uint16_t h;

	pp = &fbh_timed_head;
	while (*pp != DFW_NIL) {
		h = *pp;
		if (fbh_pool[h].release_at <= now) {
			*pp = fbh_pool[h].next;
			issued_die_release(c, fbh_pool[h].die);
			c->st.fbh_rel++;
			fbh_free(h);
		} else {
			pp = &fbh_pool[h].next;
		}
	}
}

static void issued_leave(struct cache_model *c, struct cache_ent *e, uint64_t now)
{
	uint32_t d;
	uint16_t h;

	if (!c->frame_handle_on) {
		issued_del(c, e);
		return;
	}

	if (e == c->ents && DFW_GEN(e->gen) == 0) {
		issued_del(c, e);
		return;
	}
	if (!e->await && e->clean_at <= now) {
		c->st.fbh_now++;
		issued_del(c, e);
		return;
	}
	h = fbh_alloc(c);
	if (h == DFW_NIL) {
		issued_del(c, e);
		return;
	}
	d = (uint32_t)ppa_die_id(&e->ppa);
	list_del_init(&e->dnode);
	c->issued_cnt--;
	fbh_pool[h].die = (uint16_t)d;
	fbh_pool[h].gen = DFW_GEN(e->gen);
	c->st.fbh_hold++;
	if (e->await) {
		fbh_pool[h].release_at = 0;
		fbh_pool[h].next = e->hold_head;
		e->hold_head = h;
	} else {
		fbh_pool[h].release_at = e->clean_at;
		fbh_pool[h].next = fbh_timed_head;
		fbh_timed_head = h;
	}
}

static uint64_t issue_flush(struct cache_model *c, struct cache_ent *e, uint64_t now, int rawk,
			    uint32_t owner)
{
	uint64_t pk0;
	int pl0;
	bool pslc = false;
	uint32_t frame;
	uint64_t pstart;

	pslc_note_flush(rawk != RAWK_NONE);

	pk0 = e->pgkey;
	pl0 = ce_place_of(e);
	if (ppa_is_virtual(&e->ppa)) {
		struct ppa nppa;
		uint8_t bm;
		bool raw_go;
		int held0;
		int s;

		if (e->rpart) {
			(void)conv_rand_pad_wl(e->ssd, &e->ppa);
			e->rpart = 0;
			c->st.lba_rand_pad++;
		}
		raw_go = rawk != RAWK_NONE && pslc_raw_rebind;
		held0 = ce_held_cnt(e);
		conv_bind_wl(e->ssd, &e->ppa, raw_go ? rawk : RAWK_NONE, &nppa, &bm);
		if (pl0 == CE_PL_DIRTY)
			dirty_rebucket(c, e, &nppa);
		e->ppa = nppa;

		for (s = 0; s < CE_SLOTS; s++) {
			if (!(bm & (1u << s)))
				e->lba[s] = CE_LBA_NONE;
		}
		ce_rebind_rekey(c, e, held0);
	} else if (rawk != RAWK_NONE && pslc_raw_rebind) {
		struct ppa nppa;
		int held0;

		held0 = ce_held_cnt(e);
		if (conv_rebind_raw_to_slc_k(e->ssd, e->tag, &e->ppa, &nppa, rawk,
					     e->lba) > 0) {
			if (pl0 == CE_PL_DIRTY)
				dirty_rebucket(c, e, &nppa);
			e->ppa = nppa;
			ce_rebind_rekey(c, e, held0);
		}
	}

	if (e->rpart) {
		if (e->pgkey == pk0)
			(void)conv_rand_pad_wl(e->ssd, &e->ppa);
		else
			c->st.lba_rand_rebind++;
		e->rpart = 0;
		c->st.lba_rand_pad++;
	}

	ce_gen_next(e);
	e->pblk = 0;
	e->await = 1;
	frame = c->frame_handle_on ? ((DFW_GEN(e->gen) << 16) | (uint32_t)(e - c->ents)) : 0;

	pstart = now;
	if (e->ready_at > now) {
		pstart = e->ready_at;
		c->st.lba_ready_flush++;
	}
	{
	struct nand_cmd swr = {
		.type = USER_IO,
		.cmd = NAND_WRITE,
		.pslc = pslc,
		.stime = pstart,
		.interleave_pci_dma = false,

		.xfer_size = e->ssd->sp.pgsz * e->ssd->sp.pgs_per_oneshotpg,
		.host_seq = e->host_seq,
		.owner = owner,
		.cache_frame = frame,
		.raw_ppa_enc = 0,
		.ppa = &e->ppa,
	};
	uint64_t f;

	f = ssd_advance_nand(e->ssd, &swr);

	pslc_fold_tick(e->ssd, now);

	e->clean_at = f;
	{
		uint64_t real = die_free_at_ns(e->ssd, &e->ppa, now);

		if (real > e->clean_at)
			e->clean_at = real;
	}

	if (e->dirty_ns) {
		u64 st;

		if (now > e->dirty_ns) {
			st = now - e->dirty_ns;
		} else {
			u64 back = e->dirty_ns - now;

			st = 0;
			tstage_wr_backwards++;
			tstage_wr_back_sum += back;
			if (back > tstage_wr_back_max)
				tstage_wr_back_max = back;
		}
		tstage_wr_n++;
		tstage_wr_sum += st;
		if (st > tstage_wr_max)
			tstage_wr_max = st;
		tstage_wr_hist[tstage_bucket(st)]++;
		if (e->owner <= TENANT_N) {
			tstage_wr_n_tn[e->owner]++;
			tstage_wr_sum_tn[e->owner] += st;
		}
	} else {
		tstage_wr_nodirty++;
	}
	e->state = CE_DIRTY_ISSUED;
	ce_place_take(c, e, pl0);
	issued_add(c, e);
	return f;
	}
}

static void promote_if_ready(struct cache_model *c, struct cache_ent *e, uint64_t now)
{
	if (e->state != CE_DIRTY_ISSUED)
		return;

	if (e->await) {
		if (!e->pblk) {
			e->pblk = 1;
		}
		return;
	}
	if (e->clean_at > now)
		return;
	issued_del(c, e);
	ce_gen_next(e);
	e->pblk = 0;
	e->await = 0;
	e->state = CE_CLEAN;
	e->clean_at = 0;
	e->pf = 0;
	e->was_flushed = 1;
	c->clean_cnt++;
	c->st.promote_clean++;
}

#define PROMOTE_SWEEP 16
static void promote_issued(struct cache_model *c, uint64_t now)
{
	struct cache_ent *e, *tmp;
	int n = 0;

	fbh_sweep(c, now);
	list_for_each_entry_safe(e, tmp, &c->issued_list, dnode) {
		if (e->clean_at > now || ++n > PROMOTE_SWEEP)
			break;
		promote_if_ready(c, e, now);
	}
}

static inline bool ce_zero_stall(struct cache_ent *e, uint64_t now)
{
	return e->state == CE_CLEAN ||
	       (e->state == CE_DIRTY_ISSUED && e->clean_at <= now);
}

#define GHOST_BITS 15
#define GHOST_N (1u << GHOST_BITS)
static struct { uint64_t tag; uint8_t by; uint8_t by_tn; } cache_ghost[GHOST_N];

static inline void ghost_note(struct cache_model *c, uint64_t tag, uint8_t by, uint8_t by_tn)
{
	unsigned h = (unsigned)hash_min(tag, GHOST_BITS);

	cache_ghost[h].tag = tag;
	cache_ghost[h].by = by;
	cache_ghost[h].by_tn = by_tn;
	c->st.ghost_ins++;
}

struct ghost_snap {
	bool hit;
	uint8_t by;
	uint8_t by_tn;
};

static inline void ghost_peek(uint64_t tag, struct ghost_snap *g)
{
	unsigned h;

	h = (unsigned)hash_min(tag, GHOST_BITS);
	g->hit = (cache_ghost[h].tag == tag);
	g->by = cache_ghost[h].by;
	g->by_tn = cache_ghost[h].by_tn;
}

static inline void ghost_count(struct cache_model *c, const struct ghost_snap *g, bool is_small, uint8_t tn)
{
	if (!g->hit)
		return;
	if (g->by < GB_N)
		c->st.ghost_hit[is_small ? 0 : 1][g->by]++;

	if (tn <= TENANT_N && g->by_tn <= TENANT_N)
		c->st.ghost_tn[tn][g->by_tn]++;
}

static inline void ghost_probe(struct cache_model *c, uint64_t tag, bool is_small, uint8_t tn)
{
	struct ghost_snap g;

	ghost_peek(tag, &g);
	ghost_count(c, &g, is_small, tn);
}

uint32_t dcache_flush_sync(uint64_t now, uint32_t owner)
{
	struct cache_model *c = g_dcache;
	struct cache_ent *e, *tmp;
	uint32_t waited = 0;

	if (!c || !owner)
		return 0;
	spin_lock(&c->lock);

	list_for_each_entry_safe(e, tmp, &c->dirty_list, gnode) {
		if (e->state != CE_DIRTY_UNISSUED)
			continue;
		issue_flush(c, e, now, RAWK_NONE, owner);
		waited++;
	}

	list_for_each_entry_safe(e, tmp, &c->rand_open, gnode) {
		if (e->state != CE_DIRTY_UNISSUED)
			continue;
		issue_flush(c, e, now, RAWK_NONE, owner);
		waited++;
	}

	list_for_each_entry_safe(e, tmp, &c->issued_list, dnode) {
		uint16_t w;

		if (e->state != CE_DIRTY_ISSUED || !e->await || !c->frame_handle_on)
			continue;
		w = dfw_alloc(c);
		if (w == DFW_NIL) {
			continue;
		}
		dfw_pool[w].gen = e->gen;
		dfw_pool[w].ticket = owner;
		dfw_pool[w].next = e->wait_head;
		e->wait_head = w;
		c->st.dfw_reg++;
		nvmev_note_issued(owner);
		waited++;
	}
	c->st.flush_cmd++;
	c->st.flush_cmd_ops += waited;
	if (waited > c->st.flush_cmd_max)
		c->st.flush_cmd_max = waited;
	spin_unlock(&c->lock);
	return waited;
}

static uint64_t evict(struct cache_model *c, struct cache_ent *e, uint64_t now, uint8_t evictor,
		      uint8_t evictor_tn, uint32_t *dep)
{
	uint64_t stall = 0;
	uint32_t idx = (uint32_t)(e - c->ents);

	ghost_note(c, e->tag, evictor, evictor_tn);

	if (e->owner <= TENANT_N)
		c->st.tn_evict[e->owner]++;
	if (evictor_tn <= TENANT_N)
		c->st.tn_evict_by[evictor_tn]++;

	switch (e->state) {
	case CE_CLEAN:
		c->st.evict_clean++;
		c->clean_cnt--;
		if (e->pf && e->clean_at > now)
			c->st.pf_evict_inflight++;
		e->pf = 0;
		break;
	case CE_LOADING: {
		uint32_t fr;

		c->st.lba_evict_loading++;
		fr = (DFW_GEN(e->gen) << 16) | idx;
		if (dep && nvmev_cur_cmd && c->frame_handle_on) {
			*dep = fr;
			nvmev_diq_mig_hint = false;
		} else if (nvmev_cur_cmd) {
			uint16_t w;

			w = dfw_alloc(c);
			if (w != DFW_NIL) {
				dfw_pool[w].gen = e->gen;
				dfw_pool[w].ticket = nvmev_cur_cmd;
				dfw_pool[w].next = e->wait_head;
				e->wait_head = w;
				c->st.dfw_reg++;
				nvmev_note_issued(nvmev_cur_cmd);
			}
		}
		e->fmask = 0;
		break;
	}
	case CE_DIRTY_ISSUED:
		c->st.evict_issued++;
		if (dep && e->await && nvmev_cur_cmd && c->frame_handle_on) {
			*dep = (DFW_GEN(e->gen) << 16) | idx;
			c->st.diq_dep_issued++;
		} else if (e->await && nvmev_cur_cmd) {
			uint16_t w = dfw_alloc(c);

			if (w != DFW_NIL) {
				dfw_pool[w].gen = e->gen;
				dfw_pool[w].ticket = nvmev_cur_cmd;
				dfw_pool[w].next = e->wait_head;
				e->wait_head = w;
				c->st.dfw_reg++;
				nvmev_note_issued(nvmev_cur_cmd);
			}
		} else if (e->clean_at > now) {
			stall = e->clean_at - now;
			c->st.exact_stall_n++;
			c->st.exact_stall_sum += stall;

			{
				uint64_t own = ssd_program_ns(e->ssd, &e->ppa);

				if (stall > own) {
					c->st.stall_own_sum += own;
					c->st.stall_q_sum += stall - own;
					c->st.stall_q_n++;
				} else {
					c->st.stall_own_sum += stall;
				}
			}
			if (evictor < GB_N) {
				c->st.exact_stall_by[evictor]++;

				if (stall > c->st.exact_stall_max_by[evictor])
					c->st.exact_stall_max_by[evictor] = stall;
			}
			if (evictor_tn <= TENANT_N) {
				c->st.exact_stall_tn[evictor_tn]++;
				c->st.exact_stall_ns_tn[evictor_tn] += stall;
			}
		}

		issued_leave(c, e, now);
		break;
	case CE_DIRTY_UNISSUED: {
		int rawk = (evictor == GB_LARGE) ? RAWK_L :
			   (evictor == GB_SMALL) ? RAWK_S : RAWK_NONE;
		uint64_t f = issue_flush(c, e, now, rawk, nvmev_cur_cmd);
		bool mig_hit;

		mig_hit = false;
		c->st.evict_flushes++;
		if (evictor < GB_N) {
			c->st.evict_flush_by[evictor]++;

			if (f > now) {
				uint64_t _w = f - now;

				c->st.raw_wait_ns_by[evictor] += _w;
				if (e->ssd && !nvmev_ppa_is_slc(e->ssd, &e->ppa))
					c->st.raw_wait_ns_tlc[evictor] += _w;

				else if (e->ssd && nvmev_ppa_line_migrating(e->ssd, &e->ppa)) {
					c->st.raw_wait_ns_mig[evictor] += _w;
					mig_hit = true;
				}
			}
		}
		if (dep && nvmev_cur_cmd && c->frame_handle_on) {
			*dep = (DFW_GEN(e->gen) << 16) | idx;
			c->st.diq_dep_unissued++;

			nvmev_diq_mig_hint = mig_hit;
		} else if (f > now) {
			stall = f - now;
			c->st.est_stall_n++;
			c->st.est_stall_sum += stall;
			if (evictor < GB_N) {
				c->st.est_stall_by[evictor]++;

				if (stall > c->st.est_stall_max_by[evictor])
					c->st.est_stall_max_by[evictor] = stall;
			}
			if (stall > c->st.est_stall_max)
				c->st.est_stall_max = stall;
			if (evictor_tn <= TENANT_N) {
				c->st.est_stall_tn[evictor_tn]++;
				c->st.est_stall_ns_tn[evictor_tn] += stall;
			}
		}
		c->st.evict_unissued++;
		issued_del(c, e);
		break;
	}
	}
	if (stall) {
		c->st.stall_cnt++;
		c->st.stall_ns_sum += stall;
		if (stall > c->st.stall_ns_max)
			c->st.stall_ns_max = stall;
	}
	hash_del(&e->hnode);

	if (!hlist_unhashed(&e->pnode))
		hash_del(&e->pnode);
	e->vmask = 0;
	e->fmask = 0;
	e->wload = 0;
	e->ready_at = 0;
	e->rnd = 0;
	e->rpart = 0;
	e->rpmask = 0;
	e->lbhold = 0;
	list_del_init(&e->lru);

	ce_gen_next(e);
	e->pblk = 0;
	e->await = 0;
	e->state = CE_FREE;

	e->was_flushed = 0;
	return stall;
}

#define CLEAN_PREF_DEPTH 32

static struct cache_ent *lru_tail_victim(struct cache_model *c)
{
	struct cache_ent *e;
	int skipped;

	skipped = 0;
	list_for_each_entry_reverse(e, &c->lru_list, lru) {
		if (e->state != CE_LOADING && !e->rpmask && !e->lbhold) {
			return e;
		}
		c->st.lba_skip_loading++;
		skipped++;
	}

	return NULL;
}

static struct cache_ent *take_slot(struct cache_model *c, uint64_t now, uint64_t *stall_ns,
				   bool prefer_clean, uint8_t evictor, uint8_t evictor_tn,
				   uint32_t *dep, uint32_t *full_wait)
{
	struct cache_ent *e;
	uint64_t budget = c->nr_ents - c->stage_cur;

	if (c->used > budget) {
		int depth = 0;

		list_for_each_entry_reverse(e, &c->lru_list, lru) {
			promote_if_ready(c, e, now);
			if (ce_zero_stall(e, now)) {
				evict(c, e, now, evictor, evictor_tn, NULL);
				list_add(&e->lru, &c->free_list);
				c->used--;
				break;
			}
			if (++depth >= CLEAN_PREF_DEPTH)
				break;
		}
	}
	if (c->used < budget && !list_empty(&c->free_list)) {
		e = list_first_entry(&c->free_list, struct cache_ent, lru);
		list_del_init(&e->lru);
		c->used++;
		return e;
	}
	if (prefer_clean) {
		int depth = 0;

		list_for_each_entry_reverse(e, &c->lru_list, lru) {
			promote_if_ready(c, e, now);
			if (ce_zero_stall(e, now)) {
				c->st.zero_stall_hit++;
				*stall_ns += evict(c, e, now, evictor, evictor_tn, NULL);
				return e;
			}
			if (++depth >= CLEAN_PREF_DEPTH)
				break;
		}
		c->st.zero_stall_miss++;
	}
	e = lru_tail_victim(c);
	if (!e && full_wait) {
		*full_wait = CE_WAIT_FULL;
		return NULL;
	}
	if (!e)
		e = list_last_entry(&c->lru_list, struct cache_ent, lru);
	*stall_ns += evict(c, e, now, evictor, evictor_tn, dep);
	return e;
}

int cache_read_pg(struct cache_model *c, const struct ce_page *pg, uint64_t now, bool admit,
		  uint64_t *stall_ns, bool is_small, uint8_t tn, uint32_t *dep, uint32_t *fill,
		  uint32_t *wait)
{
	struct cache_ent *e;
	struct ghost_snap gs;
	uint64_t cl;
	uint64_t w;
	uint32_t idx;
	uint32_t fw;
	uint8_t held;
	uint8_t bad;
	int r;
	int s;

	*fill = 0;
	*wait = 0;
	spin_lock(&c->lock);
	c->st.lba_rd_lookup++;
	e = lookup_pg(c, pg->pgkey);
	held = 0;
	bad = 0;
	if (e) {
		for (s = 0; s < CE_SLOTS; s++) {
			if (!(pg->live & (1u << s)))
				continue;
			if (e->lba[s] == pg->lba[s])
				held |= 1u << s;
			else if (e->lba[s] != CE_LBA_NONE)
				bad |= 1u << s;
		}
		if (bad) {
			if (e->rpmask) {
				c->st.lba_rand_unhash_rd++;
			}
			hash_del(&e->pnode);
			c->st.lba_pg_stale++;
			e = NULL;
		}
	}

	if (e && e->state == CE_LOADING && !e->wload) {
		c->st.lba_rd_miss_dup++;
		hash_del(&e->pnode);
		hash_del(&e->hnode);
		e = NULL;
	}
	if (e) {
		idx = (uint32_t)(e - c->ents);
		list_move(&e->lru, &c->lru_list);
		promote_if_ready(c, e, now);
		if (e->state == CE_LOADING) {
			*wait = (DFW_GEN(e->gen) << 16) | idx;
			c->st.lba_rd_miss_wait++;
			c->st.misses++;
			if (tn <= TENANT_N)
				c->st.tn_miss[tn]++;
			spin_unlock(&c->lock);
			return CACHE_WAIT;
		}
		if (e->state == CE_DIRTY_ISSUED) {
			c->st.lba_rd_miss_wait++;
			c->st.lba_rd_miss_issued++;
			c->st.misses++;
			if (tn <= TENANT_N)
				c->st.tn_miss[tn]++;
			if (e->await) {
				*wait = (DFW_GEN(e->gen) << 16) | idx;
				spin_unlock(&c->lock);
				return CACHE_WAIT;
			}

			w = e->clean_at > now ? e->clean_at - now : 0;
			*stall_ns += w;
			c->st.lba_rd_issued_exact_n++;
			c->st.lba_rd_issued_exact_ns += w;
			spin_unlock(&c->lock);
			return CACHE_HIT;
		}

		if (!(pg->chunk & ~held) && !(held & ~e->vmask)) {
			r = (e->state == CE_CLEAN) ? CACHE_HIT : CACHE_HIT_DIRTY;
			if (r == CACHE_HIT && e->clean_at > now) {
				w = e->clean_at - now;
				*stall_ns += w;
				c->st.lba_rd_hit_wait_n++;
				c->st.lba_rd_hit_wait_ns += w;
			}
			if (r == CACHE_HIT_DIRTY && e->ready_at > now) {
				w = e->ready_at - now;
				*stall_ns += w;
				c->st.lba_ready_rd_n++;
				c->st.lba_ready_rd_ns += w;
			}
			if (r == CACHE_HIT) {
				c->st.hits++;
				c->st.lba_rd_hit++;
			} else {
				c->st.hits_dirty++;
				c->st.lba_rd_hit_dirty++;
				c->st.hits_dirty_by[is_small ? GB_SMALL : GB_LARGE]++;
				if (tn <= TENANT_N)
					c->st.tn_hits_dirty[tn]++;
			}
			if (tn <= TENANT_N)
				c->st.tn_hit[tn]++;
			spin_unlock(&c->lock);
			return r;
		}
		c->st.misses++;
		if (tn <= TENANT_N)
			c->st.tn_miss[tn]++;
		if (e->state != CE_CLEAN || !admit) {
			c->st.lba_rd_miss_nofill++;
			spin_unlock(&c->lock);
			return CACHE_MISS;
		}

		for (s = 0; s < CE_SLOTS; s++) {
			if (pg->live & (1u << s))
				e->lba[s] = pg->lba[s];
		}
		e->state = CE_LOADING;
		e->fmask = CE_ALL_MASK;
		e->owner = tn;
		c->clean_cnt--;
		c->st.lba_rd_miss_part++;
		c->st.lba_fill_issue++;
		*fill = (DFW_GEN(e->gen) << 16) | idx;
		spin_unlock(&c->lock);
		return CACHE_MISS;
	}
	if (!admit) {
		c->st.misses++;
		if (tn <= TENANT_N)
			c->st.tn_miss[tn]++;
		ghost_probe(c, pg->tag, is_small, tn);
		c->st.bypasses++;
		c->st.lba_rd_bypass++;
		spin_unlock(&c->lock);
		return CACHE_MISS;
	}

	ghost_peek(pg->tag, &gs);
	cl = c->clean_cnt;
	fw = 0;
	e = take_slot(c, now, stall_ns, true,
		      is_small ? GB_SMALL : GB_LARGE, tn, dep, &fw);
	c->st.misses++;
	if (tn <= TENANT_N)
		c->st.tn_miss[tn]++;
	if (!e) {
		*wait = fw;
		c->st.lba_rd_miss_wait++;
		c->st.lba_rd_wait_full++;
		spin_unlock(&c->lock);
		return CACHE_WAIT;
	}
	ghost_count(c, &gs, is_small, tn);
	c->st.rdmiss_clean_hist[cm_clean_bkt(cl)]++;
	if (!c->st.rdmiss_clean_n || cl < c->st.rdmiss_clean_min)
		c->st.rdmiss_clean_min = cl;
	c->st.rdmiss_clean_n++;
	e->tag = pg->tag;
	for (s = 0; s < CE_SLOTS; s++)
		e->lba[s] = pg->lba[s];
	e->vmask = 0;

	e->fmask = CE_ALL_MASK;
	e->state = CE_LOADING;
	e->clean_at = 0;
	e->pf = 0;
	e->owner = tn;
	hash_add(c->map, &e->hnode, pg->tag);
	ce_pnode_set(c, e, pg->pgkey);
	list_add(&e->lru, &c->lru_list);
	c->st.lba_rd_miss_new++;
	c->st.lba_fill_issue++;
	*fill = (DFW_GEN(e->gen) << 16) | (uint32_t)(e - c->ents);
	spin_unlock(&c->lock);
	return CACHE_MISS;
}

int cache_read_vpg(struct cache_model *c, const struct ce_page *pg, uint64_t now, uint64_t *stall_ns,
		   bool is_small, uint8_t tn, uint32_t *wait)
{
	struct cache_ent *e;
	uint32_t fill;

	*wait = 0;
	spin_lock(&c->lock);
	e = lookup_pg(c, pg->pgkey);
	spin_unlock(&c->lock);
	if (!e)
		return CACHE_MISS;
	return cache_read_pg(c, pg, now, false, stall_ns, is_small, tn, NULL, &fill, wait);
}

int dcache_lb_hold(uint64_t pgkey, int delta)
{
	struct cache_model *c;
	struct cache_ent *e;
	int p0;
	int rc;

	c = g_dcache;
	if (!c)
		return 0;
	rc = 0;
	spin_lock(&c->lock);
	e = lookup_pg(c, pgkey);
	if (e) {
		p0 = ce_place_of(e);
		if (delta < 0 && e->lbhold < (uint16_t)(-delta))
			e->lbhold = 0;
		else
			e->lbhold = (uint16_t)(e->lbhold + delta);
		ce_place_move(c, e, p0);
		rc = (delta < 0 && !e->lbhold) ? 1 : 0;
	}
	spin_unlock(&c->lock);
	return rc;
}

static inline bool dcache_need_flush(struct cache_model *c)
{
	if (flush_mode == FLUSH_ASYNC_BUDGET) {
		return (c->dirty_unissued || c->rand_open_n) && c->dies_at_cap < TETRIS_NR_DIES;
	}
	return c->dirty_unissued * 100 > c->nr_ents * cache_dirty_ratio;
}

#define DIRTY_SCAN_DEPTH 64
#define DIRTY_HEAD_AGE 64

static struct cache_ent *pick_dirty_victim(struct cache_model *c, uint64_t now, int avoid_die)
{
	struct cache_ent *e;
	int depth = 0;
	bool forced;

	if (!c->dirty_unissued)
		return NULL;
	forced = c->head_skips >= DIRTY_HEAD_AGE;

	list_for_each_entry(e, &c->dirty_list, gnode) {
		struct ppa dp;
		int dd;

		dd = ppa_die_id(ce_die_ppa(e, &dp));

		if (c->issued_die[dd] < (uint32_t)flush_stage_k) {
			if (forced) {
				c->head_skips = 0;
				c->st.flush_forced++;
			}
			return e;
		}
		if (++depth >= DIRTY_SCAN_DEPTH)
			break;
	}
	c->st.flush_die_full++;
	return NULL;
}

static struct cache_ent *pick_rand_open(struct cache_model *c, int avoid_die)
{
	struct cache_ent *e;
	struct cache_ent *best;
	struct ppa dp;
	struct ppa *pp;
	int d;

	best = NULL;
	list_for_each_entry(e, &c->rand_open, gnode) {
		pp = ce_die_ppa(e, &dp);
		d = ppa_die_id(pp);
		if (d == avoid_die || c->issued_die[d] >= (uint32_t)flush_stage_k)
			continue;
		if (die_occupancy(e->ssd, pp) >= (uint32_t)bp_hi)
			continue;
		if (!best || e->rfree < best->rfree)
			best = e;
	}
	return best;
}

static void bg_flush_batch(struct cache_model *c, uint64_t now, int budget, int avoid_die,
			   uint64_t *cnt, int rawk, bool rand_ok)
{
	struct cache_ent *d;
	bool more;
	uint32_t rdw;

	promote_issued(c, now);

	c->st.bgf_calls++;
	more = true;
	while (more) {
		if (budget <= 0) {
			c->st.bgf_exit_budget++;
			more = false;
		} else if (!dcache_need_flush(c)) {
			c->st.bgf_exit_gate++;
			more = false;
		} else {
			d = pick_dirty_victim(c, now, avoid_die);

			if (!d && !c->dirty_unissued && rand_ok)
				d = pick_rand_open(c, avoid_die);
			else if (!d && !c->dirty_unissued && c->rand_open_n)
				c->st.lba_rand_bgf_wskip++;
			if (d && d->rnd)
				c->st.lba_rand_bgf[d->rpart ? d->rfree : 0]++;
			if (!d) {
				c->st.bgf_exit_novic++;
				more = false;
			} else {
				{
					struct ppa dp;

					rdw = die_host_rd_n(d->ssd, ce_die_ppa(d, &dp));
				}
				if (rdw)
					c->st.bgf_on_rd++;
				c->st.bgf_rd_sum += rdw;
				issue_flush(c, d, now, rawk, 0);
				(*cnt)++;
				c->st.bgf_issued++;
				budget--;
			}
		}
	}
}

static int ce_rand_add(struct cache_model *c, struct cache_ent *e, const struct ce_page *pg, uint64_t now,
		       uint64_t *stall_ns, uint32_t *wait, int budget)
{
	uint32_t idx;
	int pl0;
	int s;

	idx = (uint32_t)(e - c->ents);
	list_move(&e->lru, &c->lru_list);
	if (e->state == CE_LOADING) {
		if (e->wload) {
			for (s = 0; s < CE_SLOTS; s++) {
				if (!(pg->chunk & pg->live & (1u << s)))
					continue;
				e->lba[s] = pg->lba[s];
				e->fmask |= 1u << s;
				e->rpmask |= 1u << s;
			}
			e->rpart = pg->wl_open;
			e->rfree = pg->wl_free;
		} else {
			c->st.lba_rand_late++;
		}
		*wait = (DFW_GEN(e->gen) << 16) | idx;
		c->st.lba_rand_wait++;
		c->st.lba_wr_wait_loading++;
		return CW_WAIT;
	}
	if (e->state != CE_DIRTY_UNISSUED) {
		c->st.lba_rand_late++;
		return CW_DONE;
	}
	pl0 = ce_place_of(e);
	for (s = 0; s < CE_SLOTS; s++) {
		if (pg->live & (1u << s))
			e->lba[s] = pg->lba[s];
	}
	e->vmask |= pg->live;
	e->rpmask &= ~pg->chunk;
	e->rpart = pg->wl_open;
	e->rfree = pg->wl_free;
	if (e->ready_at < now + *stall_ns)
		e->ready_at = now + *stall_ns;
	c->st.lba_rand_add++;

	ce_place_move(c, e, pl0);
	bg_flush_batch(c, now, budget, -1, &c->st.wm_flushes, RAWK_NONE, false);
	return CW_DONE;
}

int cache_write_pg(struct cache_model *c, const struct ce_page *pg, uint64_t now, uint64_t load_ns,
		   struct ssd *ssd, struct ppa *ppa, uint32_t host_seq, uint64_t *stall_ns, uint8_t tn,
		   bool force, uint32_t *wait, uint32_t *own)
{
	struct cache_ent *e;
	bool tagdup;
	uint32_t idx;
	uint32_t vdep;
	uint32_t fw;
	uint64_t s0;
	int budget;
	int s;

	*wait = 0;
	*own = 0;
	budget = cache_flush_batch;
	spin_lock(&c->lock);

	if (pg->rnd) {
		tagdup = false;
		e = lookup_pg(c, pg->pgkey);
		if (e) {
			int rr;

			rr = ce_rand_add(c, e, pg, now, stall_ns, wait, budget);
			spin_unlock(&c->lock);
			return rr;
		}
	} else {
		e = lookup_lbas(c, pg, &tagdup);
	}
	if (e) {
		idx = (uint32_t)(e - c->ents);
		list_move(&e->lru, &c->lru_list);
		promote_if_ready(c, e, now);
		if (!force && (e->state == CE_LOADING ||
			       (e->state == CE_DIRTY_ISSUED && e->await))) {
			*wait = (DFW_GEN(e->gen) << 16) | idx;
			if (e->state == CE_LOADING)
				c->st.lba_wr_wait_loading++;
			else
				c->st.lba_wr_wait_issued++;
			spin_unlock(&c->lock);
			return CW_WAIT;
		}
		if (force)
			c->st.lba_wr_force++;
		if (e->state == CE_DIRTY_UNISSUED) {
			c->st.coalesced++;
			c->st.lba_wr_hit++;
			if (e->ready_at && e->ready_at + load_ns > now) {
				*stall_ns += e->ready_at + load_ns - now;
				c->st.lba_ready_wr++;
			}
			dirty_rebucket(c, e, ppa);
		} else if (e->state == CE_DIRTY_ISSUED) {
			if (!e->await && e->clean_at + load_ns > now)
				*stall_ns += e->clean_at + load_ns - now;
			c->st.lba_wr_wait_exact++;
			c->st.redirty_issued++;
			issued_leave(c, e, now);
			ce_gen_next(e);
			e->pblk = 0;
			e->await = 0;
			e->ppa = *ppa;
			dirty_add(c, e);
		} else if (e->state == CE_LOADING) {
			ce_gen_next(e);
			e->fmask = 0;
			e->wload = 0;
			e->ppa = *ppa;
			dirty_add(c, e);
		} else {
			if (!(ce_held_mask(e) & ~e->vmask)) {
				c->st.lba_wr_clean_hit++;
			} else {
				c->st.lba_wr_clean_part++;
			}
			if (e->clean_at + load_ns > now) {
				*stall_ns += e->clean_at + load_ns - now;
				c->st.lba_wr_wait_fill_exact++;
			}
			c->clean_cnt--;
			e->pf = 0;
			if (e->was_flushed)
				c->st.redirty_clean++;
			e->ppa = *ppa;
			dirty_add(c, e);
		}
	} else {
		c->st.lba_wr_miss++;
		if (tagdup)
			c->st.lba_wr_miss_tagdup++;

		vdep = 0;
		fw = 0;
		s0 = *stall_ns;
		e = take_slot(c, now, stall_ns, true, GB_WRITE, tn,
			      force ? NULL : &vdep, force ? NULL : &fw);
		nvmev_diq_mig_hint = false;
		if (!e) {
			*wait = fw;
			c->st.lba_wr_wait_full++;
			spin_unlock(&c->lock);
			return CW_WAIT;
		}
		e->tag = pg->tag;
		hash_add(c->map, &e->hnode, pg->tag);
		list_add(&e->lru, &c->lru_list);
		e->ppa = *ppa;

		e->rnd = pg->rnd;
		e->rpart = pg->rnd ? pg->wl_open : 0;
		e->rfree = pg->rnd ? pg->wl_free : 0;

		e->rpmask = pg->rnd ? pg->rpend : 0;

		e->lbhold = ppa_is_virtual(ppa) ? conv_lb_held(pg->pgkey) : 0;
		if (pg->rnd)
			c->st.lba_rand_new++;
		if (vdep) {
			for (s = 0; s < CE_SLOTS; s++)
				e->lba[s] = (pg->live & (1u << s)) ? pg->lba[s] : CE_LBA_NONE;
			e->vmask = 0;
			e->fmask = pg->live;
			e->wload = 1;
			ce_pnode_set(c, e, pg->pgkey);
			e->state = CE_LOADING;
			e->clean_at = 0;
			e->ready_at = 0;
			e->dirty_ns = now;
			e->owner = tn;
			if (tn <= TENANT_N)
				c->st.tn_wr[tn]++;
			e->was_flushed = 0;
			e->ssd = ssd;
			e->host_seq = host_seq;
			*wait = vdep;
			*own = (DFW_GEN(e->gen) << 16) | (uint32_t)(e - c->ents);
			c->st.lba_wr_wait_victim++;
			spin_unlock(&c->lock);
			return CW_WAIT;
		}
		if (*stall_ns > s0) {
			*stall_ns += load_ns;
			c->st.lba_wr_exact_victim++;
		}

		ce_place_put(c, e, ce_place_want(e));
	}

	for (s = 0; s < CE_SLOTS; s++)
		e->lba[s] = (pg->live & (1u << s)) ? pg->lba[s] : CE_LBA_NONE;
	e->vmask = pg->live;
	e->fmask = 0;
	e->wload = 0;
	ce_pnode_set(c, e, pg->pgkey);
	e->state = CE_DIRTY_UNISSUED;
	e->clean_at = 0;

	e->ready_at = now + *stall_ns;
	e->dirty_ns = now;
	e->owner = tn;
	if (tn <= TENANT_N)
		c->st.tn_wr[tn]++;
	e->was_flushed = 0;
	e->ssd = ssd;
	e->ppa = *ppa;
	e->host_seq = host_seq;
	list_move(&e->lru, &c->lru_list);
	bg_flush_batch(c, now, budget, -1, &c->st.wm_flushes, RAWK_NONE, false);
	spin_unlock(&c->lock);
	return CW_DONE;
}

int dcache_wload_done(uint32_t own, uint64_t t, bool forced, uint32_t *rel,
		      void (*release)(uint32_t ticket, uint64_t done))
{
	struct cache_model *c;
	struct cache_ent *e;
	struct dflush_wait *w;
	uint16_t *pp;
	uint16_t self;
	uint32_t ticket;
	uint32_t idx;
	uint32_t gen;
	int budget;
	int rc;

	*rel = 0;
	c = g_dcache;
	if (!c || !own)
		return -1;
	idx = own & 0xFFFFu;
	gen = own >> 16;
	if (idx >= c->nr_ents)
		return -1;
	e = &c->ents[idx];
	budget = cache_flush_batch;

	spin_lock(&c->lock);
	if (DFW_GEN(e->gen) != gen || e->state != CE_LOADING || !e->wload) {
		c->st.lba_wload_stale++;
		rc = -1;
	} else {
		e->wload = 0;
		e->vmask = e->fmask;
		e->fmask = 0;
		e->state = CE_DIRTY_UNISSUED;
		e->clean_at = 0;
		e->ready_at = t;
		e->dirty_ns = t;

		ce_place_put(c, e, ce_place_want(e));
		if (forced)
			c->st.lba_wload_force++;
		else
			c->st.lba_wload_done++;
		*rel = own;
		bg_flush_batch(c, t, budget, -1, &c->st.wm_flushes, RAWK_NONE, false);
		rc = 0;
	}

	pp = &e->wait_head;
	while (*pp != DFW_NIL) {
		self = *pp;
		w = &dfw_pool[self];
		if (DFW_GEN(w->gen) == gen) {
			*pp = w->next;
			ticket = w->ticket;
			dfw_free(self);
			c->st.dfw_rel++;
			spin_unlock(&c->lock);
			release(ticket, t);
			spin_lock(&c->lock);
			continue;
		}
		pp = &w->next;
	}
	spin_unlock(&c->lock);
	return rc;
}

int cache_stage_acquire(struct cache_model *c, uint64_t now, int avoid_die, uint16_t stream_id,
			uint64_t *stall_ns)
{
	uint64_t best_rel = STAGE_INFLIGHT;
	uint32_t occ_s[TETRIS_MAX_STREAMS + 1];
	uint32_t occ = 0, active = 0;
	int best = -1;
	uint32_t i;

	if (!c->stage_cap || !c->stage_cur)
		return -1;
	if (stream_id > TETRIS_MAX_STREAMS)
		stream_id = 0;
	spin_lock(&c->lock);
	c->st.stage_acq++;
	c->st.stage_str_acq[stream_id]++;
	memset(occ_s, 0, sizeof(occ_s));

	for (i = 0; i < c->stage_cur; i++) {
		if (c->stage_release[i] > now) {
			occ++;
			occ_s[c->stage_owner[i]]++;
		}
		if (c->stage_release[i] < best_rel) {
			best_rel = c->stage_release[i];
			best = i;
		}
	}
	if (occ > c->st.stage_occ_hwm)
		c->st.stage_occ_hwm = occ;

	c->st.stage_occ_sum += occ;
	c->st.stage_occ_n++;
	occ_ewma_note(c, occ);
	for (i = 0; i <= TETRIS_MAX_STREAMS; i++) {
		if (occ_s[i] > c->st.stage_str_hwm[i])
			c->st.stage_str_hwm[i] = occ_s[i];
		if (i && occ_s[i])
			active++;
	}
	if (active > c->st.stage_active_hwm)
		c->st.stage_active_hwm = active;

	if (best < 0) {
		c->st.stage_race++;
		best = 0;
		best_rel = now;
	}
	if (best_rel > now) {
		uint64_t s = best_rel - now;

		*stall_ns += s;
		c->st.stage_stall_cnt++;
		c->st.stage_stall_ns_sum += s;
		if (s > c->st.stage_stall_ns_max)
			c->st.stage_stall_ns_max = s;
	}
	c->stage_release[best] = STAGE_INFLIGHT;
	c->stage_owner[best] = stream_id;

	spin_unlock(&c->lock);
	return best;
}

void cache_stage_commit(struct cache_model *c, int slot, uint64_t release_at)
{
	if (slot < 0)
		return;

	if (slot >= SHSTAGE_BASE) {
		if (!c->sh_release)
			return;
		spin_lock(&c->lock);
		c->sh_release[slot - SHSTAGE_BASE] = release_at;
		c->sh_commit++;
		spin_unlock(&c->lock);
		return;
	}
	if (!c->stage_cap)
		return;
	spin_lock(&c->lock);
	c->stage_release[slot] = release_at;
	spin_unlock(&c->lock);
}

int cache_shadow_acquire(struct cache_model *c, uint64_t now)
{
	uint32_t sweep = rawp_sh_sweep > 0 ? (uint32_t)rawp_sh_sweep : SHSTAGE_SWEEP_DEF;
	uint64_t r;
	int sh;
	uint32_t tries, sw;

	if (!c || !c->sh_release || !c->sh_n)
		return -1;

	if (c->stage_cap)
		return -1;
	spin_lock(&c->lock);

	for (sw = 0; sw < sweep; sw++) {
		uint32_t i = c->sh_sweep++ & c->sh_mask;

		r = c->sh_release[i];
		if (r && r != STAGE_INFLIGHT && r <= now) {
			c->sh_release[i] = 0;
			if (c->sh_live)
				c->sh_live--;
		}
	}

	sh = (int)c->sh_cursor;
	for (tries = 0; tries < c->sh_n; tries++) {
		r = c->sh_release[sh];
		if (r == 0 || (r != STAGE_INFLIGHT && r <= now))
			break;
		sh = (int)((sh + 1) & c->sh_mask);
	}
	if (tries == c->sh_n) {
		c->sh_drop++;
		spin_unlock(&c->lock);
		return -1;
	}
	if (c->sh_release[sh] != 0 && c->sh_live)
		c->sh_live--;

	c->sh_release[sh] = STAGE_INFLIGHT;
	c->sh_live++;
	if (c->sh_live > c->sh_occ_hwm)
		c->sh_occ_hwm = c->sh_live;

	c->sh_occ_sum += c->sh_live;
	c->sh_occ_n++;
	occ_ewma_note(c, c->sh_live);
	c->sh_cursor = (uint32_t)((sh + 1) & c->sh_mask);
	c->sh_acq++;
	spin_unlock(&c->lock);
	return SHSTAGE_BASE + sh;
}
