// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/highmem.h>
#include <linux/sched/clock.h>

#include "nvmev.h"
#include "dma.h"

#include "ssd.h"

#include "cache_model.h"

#undef PERF_DEBUG

#define sq_entry(entry_id) sq->sq[SQ_ENTRY_TO_PAGE_NUM(entry_id)][SQ_ENTRY_TO_PAGE_OFFSET(entry_id)]
#define cq_entry(entry_id) cq->cq[CQ_ENTRY_TO_PAGE_NUM(entry_id)][CQ_ENTRY_TO_PAGE_OFFSET(entry_id)]

extern bool io_using_dma;

static inline unsigned int __get_io_worker(int sqid)
{
#ifdef CONFIG_NVMEV_IO_WORKER_BY_SQ
	return (sqid - 1) % nvmev_vdev->config.nr_io_workers;
#else
	return nvmev_vdev->io_worker_turn;
#endif
}

static inline unsigned long long __get_wallclock(void)
{
	return cpu_clock(nvmev_vdev->config.cpu_nr_dispatcher);
}

static inline size_t __cmd_io_offset(const struct nvme_rw_command *cmd)
{
	return (cmd->slba) << LBA_BITS;
}

static inline size_t __cmd_io_size(const struct nvme_rw_command *cmd)
{
	return (cmd->length + 1) << LBA_BITS;
}

static unsigned int __do_perform_io(int sqid, int sq_entry, const struct nvme_command *sqe_copy)
{
	const struct nvme_rw_command *cmd = &sqe_copy->rw;
	size_t offset;
	size_t length, remaining;
	int prp_offs = 0;
	int prp2_offs = 0;
	u64 paddr;
	u64 *paddr_list = NULL;
	size_t nsid = cmd->nsid - 1;
	bool is_paddr_memremap = false;

	offset = __cmd_io_offset(cmd);
	length = __cmd_io_size(cmd);
	remaining = length;

	while (remaining) {
		size_t io_size;
		void *vaddr;
		size_t mem_offs = 0;
		bool is_vaddr_memremap = false;

		prp_offs++;
		if (prp_offs == 1) {
			paddr = cmd->prp1;
		} else if (prp_offs == 2) {
			paddr = cmd->prp2;
			if (remaining > PAGE_SIZE) {
				if (pfn_valid(paddr >> PAGE_SHIFT)) {
					paddr_list = kmap_atomic_pfn(PRP_PFN(paddr)) +
						(paddr & PAGE_OFFSET_MASK);
				} else {
					paddr_list = memremap(paddr, PAGE_SIZE, MEMREMAP_WT);
					paddr_list += (paddr & PAGE_OFFSET_MASK);
					is_paddr_memremap = true;
				}
				paddr = paddr_list[prp2_offs++];
			}
		} else {
			paddr = paddr_list[prp2_offs++];
		}

		if (pfn_valid(paddr >> PAGE_SHIFT)) {
			vaddr = kmap_atomic_pfn(PRP_PFN(paddr));
		} else {
			vaddr = memremap(paddr, PAGE_SIZE, MEMREMAP_WT);
			is_vaddr_memremap = true;
		}

		io_size = min_t(size_t, remaining, PAGE_SIZE);

		if (paddr & PAGE_OFFSET_MASK) {
			mem_offs = paddr & PAGE_OFFSET_MASK;
			if (io_size + mem_offs > PAGE_SIZE)
				io_size = PAGE_SIZE - mem_offs;
		}

		if (cmd->opcode == nvme_cmd_write ||
		    cmd->opcode == nvme_cmd_zone_append) {
			memcpy(nvmev_vdev->ns[nsid].mapped + offset, vaddr + mem_offs, io_size);
		} else if (cmd->opcode == nvme_cmd_read) {
			memcpy(vaddr + mem_offs, nvmev_vdev->ns[nsid].mapped + offset, io_size);
		}

		if (vaddr != NULL && !is_vaddr_memremap) {
			kunmap_atomic(vaddr);
			vaddr = NULL;
		} else if (vaddr != NULL && is_vaddr_memremap) {
			memunmap(vaddr);
			vaddr = NULL;
			is_vaddr_memremap = false;
		}

		remaining -= io_size;
		offset += io_size;
	}

	if (paddr_list) {
		if (!is_paddr_memremap)
			kunmap_atomic(paddr_list);
		else if (is_paddr_memremap)
			memunmap(paddr_list);
	}
	paddr_list = NULL;

	return length;
}

static u64 paddr_list[513] = {
	0,
};
static unsigned int __do_perform_io_using_dma(int sqid, int sq_entry,
					     const struct nvme_command *sqe_copy)
{
	const struct nvme_rw_command *cmd = &sqe_copy->rw;
	size_t offset;
	size_t length, remaining;
	int prp_offs = 0;
	int prp2_offs = 0;
	int num_prps = 0;
	u64 paddr;
	u64 *tmp_paddr_list = NULL;
	size_t io_size;
	size_t mem_offs = 0;
	bool is_memremap = false;

	offset = __cmd_io_offset(cmd);
	length = __cmd_io_size(cmd);
	remaining = length;

	memset(paddr_list, 0, sizeof(paddr_list));

	while (remaining) {
		io_size = 0;

		prp_offs++;
		if (prp_offs == 1) {
			paddr_list[prp_offs] = cmd->prp1;
		} else if (prp_offs == 2) {
			paddr_list[prp_offs] = cmd->prp2;
			if (remaining > PAGE_SIZE) {
				if (pfn_valid(paddr_list[prp_offs] >> PAGE_SHIFT)) {
 					tmp_paddr_list = kmap_atomic_pfn(PRP_PFN(paddr_list[prp_offs])) +
							(paddr_list[prp_offs] & PAGE_OFFSET_MASK);
 				} else {
 					tmp_paddr_list = memremap(paddr_list[prp_offs], PAGE_SIZE, MEMREMAP_WT);
 					tmp_paddr_list += (paddr_list[prp_offs] & PAGE_OFFSET_MASK);
 					is_memremap = true;
 				}
				paddr_list[prp_offs] = tmp_paddr_list[prp2_offs++];
			}
		} else {
			paddr_list[prp_offs] = tmp_paddr_list[prp2_offs++];
		}

		io_size = min_t(size_t, remaining, PAGE_SIZE);

		if (paddr_list[prp_offs] & PAGE_OFFSET_MASK) {
			mem_offs = paddr_list[prp_offs] & PAGE_OFFSET_MASK;
			if (io_size + mem_offs > PAGE_SIZE)
				io_size = PAGE_SIZE - mem_offs;
		}

		remaining -= io_size;
	}
	num_prps = prp_offs;

	if (tmp_paddr_list != NULL && !is_memremap) {
 		kunmap_atomic(tmp_paddr_list);
 	} else if (tmp_paddr_list != NULL && is_memremap) {
 		memunmap(tmp_paddr_list);
 		is_memremap = false;
 	}

	remaining = length;
	prp_offs = 1;

	while (remaining) {
		size_t page_size;
		mem_offs = 0;
		io_size = 0;
		page_size = 0;

		paddr = paddr_list[prp_offs];
		page_size = min_t(size_t, remaining, PAGE_SIZE);

		if (paddr & PAGE_OFFSET_MASK) {
			mem_offs = paddr & PAGE_OFFSET_MASK;
			if (page_size + mem_offs > PAGE_SIZE) {
				page_size = PAGE_SIZE - mem_offs;
			}
		}

		for (prp_offs++; prp_offs <= num_prps; prp_offs++) {
			if (paddr_list[prp_offs] == paddr_list[prp_offs - 1] + PAGE_SIZE)
				page_size += PAGE_SIZE;
			else
				break;
		}

		io_size = min_t(size_t, remaining, page_size);

		if (cmd->opcode == nvme_cmd_write ||
		    cmd->opcode == nvme_cmd_zone_append) {
			ioat_dma_submit(paddr, nvmev_vdev->config.storage_start + offset, io_size);
		} else if (cmd->opcode == nvme_cmd_read) {
			ioat_dma_submit(nvmev_vdev->config.storage_start + offset, paddr, io_size);
		}

		remaining -= io_size;
		offset += io_size;
	}

	return length;
}

static void __insert_req_sorted(unsigned int entry, struct nvmev_io_worker *worker,
				unsigned long nsecs_target)
{
	if (worker->io_seq == -1) {
		worker->io_seq = entry;
		worker->io_seq_end = entry;
	} else {
		unsigned int curr = worker->io_seq_end;

		while (curr != -1) {
			if (worker->work_queue[curr].nsecs_target <= worker->latest_nsecs)
				break;

			if (worker->work_queue[curr].nsecs_target <= nsecs_target)
				break;

			curr = worker->work_queue[curr].prev;
		}

		if (curr == -1) {
			worker->work_queue[worker->io_seq].prev = entry;
			worker->work_queue[entry].next = worker->io_seq;
			worker->io_seq = entry;
		} else if (worker->work_queue[curr].next == -1) {
			worker->work_queue[entry].prev = curr;
			worker->io_seq_end = entry;
			worker->work_queue[curr].next = entry;
		} else {
			worker->work_queue[entry].prev = curr;
			worker->work_queue[entry].next = worker->work_queue[curr].next;

			worker->work_queue[worker->work_queue[entry].next].prev = entry;
			worker->work_queue[curr].next = entry;
		}
	}
}

static struct nvmev_io_worker *__allocate_work_queue_entry(int sqid, unsigned int *entry)
{
	unsigned int io_worker_turn = __get_io_worker(sqid);
	struct nvmev_io_worker *worker = &nvmev_vdev->io_workers[io_worker_turn];
	unsigned int e = worker->free_seq;
	struct nvmev_io_work *w = worker->work_queue + e;

	if (w->next >= NR_MAX_PARALLEL_IO) {
		WARN_ON_ONCE("IO queue is almost full");
		return NULL;
	}

	if (++io_worker_turn == nvmev_vdev->config.nr_io_workers)
		io_worker_turn = 0;
	nvmev_vdev->io_worker_turn = io_worker_turn;

	worker->free_seq = w->next;
	BUG_ON(worker->free_seq >= NR_MAX_PARALLEL_IO);
	if (++worker->in_use > worker->in_use_hwm)
		worker->in_use_hwm = worker->in_use;
	*entry = e;

	return worker;
}

static void __free_work_queue_entry(struct nvmev_io_worker *worker, unsigned int entry)
{
	struct nvmev_io_work *w = &worker->work_queue[entry];

	w->next = worker->free_seq;
	w->prev = -1;
	worker->work_queue[worker->free_seq].prev = entry;
	worker->free_seq = entry;
	if (worker->in_use)
		worker->in_use--;
}

#define NVMEV_NOT_DONE (~0ULL)

#define P4_RING_MASK (P4_RING_SZ - 1)
#define P4_PROBE_MAX 4096

struct p4_cmd {
	uint32_t ticket;
	uint32_t issued, committed;

	unsigned long long earliest_host_completion_ns;

	unsigned long long nand_completion_ns;

	unsigned char opc;
	struct nvmev_io_worker *worker;
	unsigned int entry;
	bool issue_done;
};

static struct p4_cmd p4_ring[P4_RING_SZ];
static uint32_t p4_next_ticket = 1;

uint32_t nvmev_cur_cmd;

u32 nvmev_p4_early_seen;
u32 nvmev_p4_lostc_seen;
u32 nvmev_p4_stuck_seen, nvmev_p4_stuck_issued, nvmev_p4_stuck_committed,
    nvmev_p4_stuck_worker, nvmev_p4_stuck_age;
u64 nvmev_p4_nn_rd, nvmev_p4_nn_wr, nvmev_p4_nn_fl, nvmev_p4_nn_other;

u64 nvmev_p4_relink, nvmev_p4_badsq, nvmev_p4_stale_owner, nvmev_p4_badcopy, nvmev_p4_lost_cmpl,
    nvmev_p4_sq_stall;
unsigned int nvmev_p4_bc_sqid, nvmev_p4_bc_entry, nvmev_p4_bc_sqe, nvmev_p4_bc_wid,
	     nvmev_p4_bc_idx, nvmev_p4_bc_iss, nvmev_p4_bc_cmt;

static struct p4_cmd *p4_slot(uint32_t ticket)
{
	struct p4_cmd *c = &p4_ring[ticket & P4_RING_MASK];

	return (ticket && c->ticket == ticket) ? c : NULL;
}

static void p4_account_retire(const struct p4_cmd *c)
{
	if (!c->nand_completion_ns) {
		switch (c->opc) {
		case nvme_cmd_read:  nvmev_p4_nn_rd++; break;
		case nvme_cmd_write: nvmev_p4_nn_wr++; break;
		case nvme_cmd_flush: nvmev_p4_nn_fl++; break;
		default:             nvmev_p4_nn_other++; break;
		}
		return;
	}
}

static void p4_retire(struct p4_cmd *c, int site)
{
	if (!c->ticket)
		return;

	if (c->committed < c->issued) {
		if (!nvmev_p4_early_seen) {
			nvmev_p4_early_seen = 1;
		}
	}
	c->ticket = 0;
}

static struct p4_cmd *p4_open(void)
{
	uint32_t t = p4_next_ticket++;
	struct p4_cmd *c;

	if (!p4_next_ticket)
		p4_next_ticket = 1;
	c = &p4_ring[t & P4_RING_MASK];

	{
		uint32_t probe = 0;

		while (c->ticket && c->committed < c->issued && probe < P4_PROBE_MAX) {
			t = p4_next_ticket++;
			if (!p4_next_ticket)
				p4_next_ticket = 1;
			c = &p4_ring[t & P4_RING_MASK];
			probe++;
		}
	}
	if (c->ticket && c->issue_done && c->committed < c->issued) {
		nvmev_p4_stale_owner++;
		if (!nvmev_p4_stuck_seen) {
			nvmev_p4_stuck_seen = 1;
			nvmev_p4_stuck_issued = c->issued;
			nvmev_p4_stuck_committed = c->committed;
			nvmev_p4_stuck_worker = c->worker ? 1 : 0;
			nvmev_p4_stuck_age = t - c->ticket;
		}
	}
	c->ticket = t;
	c->issued = 0;
	c->committed = 0;
	c->earliest_host_completion_ns = 0;
	c->nand_completion_ns = 0;
	c->worker = NULL;
	c->entry = 0;
	c->issue_done = false;
	return c;
}

void nvmev_note_issued(uint32_t owner)
{
	struct p4_cmd *c = p4_slot(owner);

	if (c) {
		c->issued++;
	}
}

static void nvmev_release_dflush_wait(uint32_t ticket, uint64_t done)
{
	struct p4_cmd *c = p4_slot(ticket);

	if (!c) {
		nvmev_p4_lost_cmpl++;
		return;
	}

	if (done > c->earliest_host_completion_ns) {
		c->earliest_host_completion_ns = done;
	} else if (done) {
	}
	if (done > c->nand_completion_ns)
		c->nand_completion_ns = done;
	c->committed++;
	if (c->issue_done && c->committed >= c->issued && c->worker) {
		struct nvmev_io_work *w = &c->worker->work_queue[c->entry];

		mb();
		p4_account_retire(c);
		WRITE_ONCE(w->nsecs_target, c->earliest_host_completion_ns);
		p4_retire(c, 0);
	}
}

void nvmev_dep_stats_reset(void)
{
	nvmev_p4_nn_rd = nvmev_p4_nn_wr = nvmev_p4_nn_fl = nvmev_p4_nn_other = 0;
	diq_park = diq_resume_n = diq_full = diq_stale = diq_hwm = 0;
	diq_delay_n = diq_delay_sum = diq_delay_max = diq_nodelay = 0;

	memset(diq_delay_n_tn, 0, sizeof(diq_delay_n_tn));
	memset(diq_delay_hist_tn, 0, sizeof(diq_delay_hist_tn));
	memset(diq_delay_hsum_tn, 0, sizeof(diq_delay_hsum_tn));

	nvmev_diq_mig_reset();
}

void nvmev_p4_commit_token(uint32_t ticket)
{
	nvmev_release_dflush_wait(ticket, 0);
}

void nvmev_p4_commit_at(uint32_t ticket, uint64_t t)
{
	struct p4_cmd *c;
	struct nvmev_io_work *w;

	c = p4_slot(ticket);
	if (!c) {
		nvmev_p4_lost_cmpl++;
		return;
	}
	if (t > c->earliest_host_completion_ns)
		c->earliest_host_completion_ns = t;
	if (t > c->nand_completion_ns)
		c->nand_completion_ns = t;
	c->committed++;
	if (c->issue_done && c->committed >= c->issued && c->worker) {
		w = &c->worker->work_queue[c->entry];
		mb();
		p4_account_retire(c);
		WRITE_ONCE(w->nsecs_target, c->earliest_host_completion_ns);
		p4_retire(c, 0);
	}
}

static void nvmev_fill_complete(uint32_t fill, uint64_t chnl_end)
{
	int rc;

	if (!fill)
		return;
	rc = dcache_fill_done(fill, chnl_end, nvmev_release_dflush_wait);
	nvmev_diq_resume(fill, chnl_end);

	if (rc == 0)
		nvmev_diq_full_wake(chnl_end);
}

int nvmev_wload_finish(uint32_t own, uint64_t t, bool forced)
{
	uint32_t rel;
	int rc;

	rel = 0;
	rc = dcache_wload_done(own, t, forced, &rel, nvmev_release_dflush_wait);
	if (rel)
		nvmev_diq_resume(rel, t);

	if (rc == 0)
		nvmev_diq_full_wake(t);
	return rc;
}

void nvmev_apply_completion(struct ssd *ssd, uint32_t owner, uint32_t xfer, uint64_t chnl_end,
			    uint32_t frame, uint32_t fill)
{
	struct p4_cmd *c = p4_slot(owner);
	uint64_t done;

	if (frame) {
		dcache_flush_done(frame, chnl_end, nvmev_release_dflush_wait);

		nvmev_diq_resume(frame, chnl_end);
	}

	if (!c) {
		if (owner) {
			nvmev_p4_lost_cmpl++;

			if (!nvmev_p4_lostc_seen) {
				nvmev_p4_lostc_seen = 1;
			}
		}
		nvmev_fill_complete(fill, chnl_end);
		return;
	}

	done = xfer ? ssd_advance_pcie(ssd, chnl_end, xfer) : chnl_end;
	if (done > c->earliest_host_completion_ns)
		c->earliest_host_completion_ns = done;
	if (done > c->nand_completion_ns)
		c->nand_completion_ns = done;
	c->committed++;

	if (c->issue_done && c->committed >= c->issued && c->worker) {
		struct nvmev_io_work *w = &c->worker->work_queue[c->entry];

		mb();
		p4_account_retire(c);
		WRITE_ONCE(w->nsecs_target, c->earliest_host_completion_ns);

		p4_retire(c, 0);
	}
	nvmev_fill_complete(fill, chnl_end);
}

static void __reclaim_completed_reqs(void)
{
	unsigned int turn;

	for (turn = 0; turn < nvmev_vdev->config.nr_io_workers; turn++) {
		struct nvmev_io_worker *worker;
		struct nvmev_io_work *w;

		unsigned int first_entry = -1;
		unsigned int last_entry = -1;
		unsigned int curr;
		int nr_reclaimed = 0;

		worker = &nvmev_vdev->io_workers[turn];

		first_entry = worker->io_seq;
		curr = first_entry;

		while (curr != -1) {
			w = &worker->work_queue[curr];
			if (w->is_completed == true && w->is_copied == true &&
			    w->nsecs_target <= worker->latest_nsecs) {
				last_entry = curr;
				curr = w->next;
				nr_reclaimed++;
			} else {
				break;
			}
		}

		if (last_entry != -1) {
			w = &worker->work_queue[last_entry];
			worker->io_seq = w->next;
			if (w->next != -1) {
				worker->work_queue[w->next].prev = -1;
			}
			w->next = -1;

			w = &worker->work_queue[first_entry];
			w->prev = worker->free_seq_end;

			w = &worker->work_queue[worker->free_seq_end];
			w->next = first_entry;

			worker->free_seq_end = last_entry;
			worker->in_use = (worker->in_use >= (unsigned int)nr_reclaimed) ?
					 worker->in_use - nr_reclaimed : 0;
			NVMEV_DEBUG_VERBOSE("%s: %u -- %u, %d\n", __func__,
					first_entry, last_entry, nr_reclaimed);
		}
	}
}

static struct cmdbox g_cmdbox;

static bool cmdbox_push(struct nvmev_ns *ns, struct nvme_command *cmd, int sqid, int cqid,
			int sq_entry, uint16_t command_id, struct p4_cmd *p4c,
			struct nvmev_io_worker *worker, unsigned int entry,
			unsigned long long nsecs_start, size_t io_size)
{
	uint32_t t = g_cmdbox.tail;
	uint32_t h = READ_ONCE(g_cmdbox.head);
	struct cmdbox_ent *e;

	if (t - h >= CMDBOX_SZ) {
		return false;
	}
	e = &g_cmdbox.slot[t & CMDBOX_MASK];
	e->ns = ns;
	e->sqe = *cmd;
	e->sq_id = sqid;
	e->cq_id = cqid;
	e->sq_entry = sq_entry;
	e->command_id = command_id;
	e->io_size = io_size;
	e->p4c = p4c;
	e->worker = worker;
	e->entry = entry;
	e->nsecs_start = nsecs_start;

	smp_wmb();
	WRITE_ONCE(g_cmdbox.tail, t + 1);

	return true;
}

static bool cmdbox_pop(struct cmdbox_ent *out)
{
	uint32_t h = g_cmdbox.head;
	uint32_t t = READ_ONCE(g_cmdbox.tail);

	if (h == t)
		return false;
	smp_rmb();
	*out = g_cmdbox.slot[h & CMDBOX_MASK];
	WRITE_ONCE(g_cmdbox.head, h + 1);

	return true;
}

static size_t __nvmev_proc_io(int sqid, int sq_entry, size_t *io_size)
{
	struct nvmev_submission_queue *sq = nvmev_vdev->sqes[sqid];
	unsigned long long nsecs_start = __get_wallclock();
	struct nvme_command *cmd = &sq_entry(sq_entry);
	uint32_t nsid = cmd->common.nsid - 1;
	struct nvmev_ns *ns = &nvmev_vdev->ns[nsid];

	struct nvmev_request req = {
		.cmd = cmd,
		.sq_id = sqid,
		.nsecs_start = nsecs_start,
	};
	struct nvmev_result ret = {
		.nsecs_target = nsecs_start,
		.status = NVME_SC_SUCCESS,
	};
	struct p4_cmd *p4c = NULL;
	struct nvmev_io_worker *p4_worker = NULL;
	unsigned int p4_entry = 0;
	struct cmdbox_ent ce;

#ifdef PERF_DEBUG
	unsigned long long prev_clock = local_clock();
	unsigned long long prev_clock2 = 0;
	unsigned long long prev_clock3 = 0;
	unsigned long long prev_clock4 = 0;
	static unsigned long long clock1 = 0;
	static unsigned long long clock2 = 0;
	static unsigned long long clock3 = 0;
	static unsigned long long counter = 0;
#endif

	p4_worker = __allocate_work_queue_entry(sqid, &p4_entry);
	if (!p4_worker) {
		nvmev_p4_sq_stall++;
		return false;
	}

	p4c = p4_open();
	p4c->opc = cmd->common.opcode;
	nvmev_cur_cmd = p4c->ticket;

	*io_size = __cmd_io_size(&sq_entry(sq_entry).rw);

	if (!cmdbox_push(ns, cmd, sqid, sq->cqid, sq_entry,
			 sq_entry(sq_entry).common.command_id,
			 p4c, p4_worker, p4_entry, nsecs_start, *io_size)) {
		nvmev_cur_cmd = 0;
		p4_retire(p4c, 1);
		__free_work_queue_entry(p4_worker, p4_entry);
		return false;
	}
	if (!cmdbox_pop(&ce)) {
		NVMEV_ERROR("cmdbox: pop failed right after push -- ring invariant broken\n");
		nvmev_cur_cmd = 0;
		p4_retire(p4c, 1);
		__free_work_queue_entry(p4_worker, p4_entry);
		return false;
	}
	req.cmd = &ce.sqe;
	req.sq_id = ce.sq_id;
	req.nsecs_start = ce.nsecs_start;
	ret.nsecs_target = ce.nsecs_start;

	if (!ns->proc_io_cmd(ce.ns, &req, &ret)) {
		ret.status = NVME_SC_LBA_RANGE;
		ret.nsecs_target = ce.nsecs_start;
	}

	{
		struct nvmev_io_worker *worker;
		struct nvmev_io_work *w;
		unsigned int entry;

		nvmev_cur_cmd = 0;
		p4c->issue_done = true;

		if (ret.nsecs_target > p4c->earliest_host_completion_ns)
			p4c->earliest_host_completion_ns = ret.nsecs_target;

		worker = p4_worker;
		entry = p4_entry;
		w = worker->work_queue + entry;
		w->sqid = sqid;
		w->cqid = sq->cqid;
		w->sq_entry = sq_entry;
		w->command_id = sq_entry(sq_entry).common.command_id;
		w->nsecs_start = nsecs_start;
		w->nsecs_enqueue = local_clock();
		w->status = ret.status;
		w->is_completed = false;
		w->is_copied = false;
		w->prev = -1;
		w->next = -1;
		w->wid = worker->id;
		w->idx = entry;
		w->p4_published = true;

		w->sqe = ce.sqe;

		if (p4c->committed >= p4c->issued) {
			p4_account_retire(p4c);
			w->nsecs_target = p4c->earliest_host_completion_ns;
			p4_retire(p4c, 2);
		} else {
			w->nsecs_target = NVMEV_NOT_DONE;
		}
		p4c->worker = worker;
		p4c->entry = entry;
		mb();
		__insert_req_sorted(entry, worker, w->nsecs_target);
	}

	__reclaim_completed_reqs();

#ifdef PERF_DEBUG
	prev_clock4 = local_clock();

	clock1 += (prev_clock2 - prev_clock);
	clock2 += (prev_clock3 - prev_clock2);
	clock3 += (prev_clock4 - prev_clock3);
	counter++;

	if (counter > 1000) {
		NVMEV_DEBUG("LAT: %llu, ENQ: %llu, CLN: %llu\n", clock1 / counter, clock2 / counter,
			    clock3 / counter);
		clock1 = 0;
		clock2 = 0;
		clock3 = 0;
		counter = 0;
	}
#endif
	return true;
}

int nvmev_proc_io_sq(int sqid, int new_db, int old_db)
{
	struct nvmev_submission_queue *sq = nvmev_vdev->sqes[sqid];
	int num_proc = new_db - old_db;
	int seq;
	int sq_entry = old_db;
	int latest_db;

	if (unlikely(!sq))
		return old_db;
	if (unlikely(num_proc < 0))
		num_proc += sq->queue_size;

	for (seq = 0; seq < num_proc; seq++) {
		size_t io_size;

		if (!nvmev_diq_room()) {
			break;
		}
		if (!__nvmev_proc_io(sqid, sq_entry, &io_size))
			break;

		if (++sq_entry == sq->queue_size) {
			sq_entry = 0;
		}
		sq->stat.nr_dispatched++;
		sq->stat.nr_in_flight++;
		sq->stat.total_io += io_size;
	}
	sq->stat.nr_dispatch++;
	sq->stat.max_nr_in_flight = max_t(int, sq->stat.max_nr_in_flight, sq->stat.nr_in_flight);

	latest_db = (old_db + seq) % sq->queue_size;
	return latest_db;
}

void nvmev_proc_io_cq(int cqid, int new_db, int old_db)
{
	struct nvmev_completion_queue *cq = nvmev_vdev->cqes[cqid];
	int i;
	for (i = old_db; i != new_db; i++) {
		int sqid = cq_entry(i).sq_id;
		if (i >= cq->queue_size) {
			i = -1;
			continue;
		}

		if (!nvmev_vdev->sqes[sqid]) continue;

		nvmev_vdev->sqes[sqid]->stat.nr_in_flight--;
	}

	cq->cq_tail = new_db - 1;
	if (new_db == -1)
		cq->cq_tail = cq->queue_size - 1;
}

static void __fill_cq_result(struct nvmev_io_work *w)
{
	int sqid = w->sqid;
	int cqid = w->cqid;
	int sq_entry = w->sq_entry;
	unsigned int command_id = w->command_id;
	unsigned int status = w->status;
	unsigned int result0 = w->result0;
	unsigned int result1 = w->result1;

	struct nvmev_completion_queue *cq = nvmev_vdev->cqes[cqid];
	struct nvme_completion *cqe;
	int cq_head;

	spin_lock(&cq->entry_lock);
	cq_head = cq->cq_head;
	cqe = &cq_entry(cq_head);

	cqe->command_id = command_id;
	cqe->sq_id = sqid;
	cqe->sq_head = sq_entry;
	cqe->status = cq->phase | (status << 1);
	cqe->result0 = result0;
	cqe->result1 = result1;

	if (++cq_head == cq->queue_size) {
		cq_head = 0;
		cq->phase = !cq->phase;
	}

	cq->cq_head = cq_head;
	cq->interrupt_ready = true;
	spin_unlock(&cq->entry_lock);
}

static int nvmev_io_worker(void *data)
{
	struct nvmev_io_worker *worker = (struct nvmev_io_worker *)data;
	static unsigned long last_io_time = 0;

#ifdef PERF_DEBUG
	static unsigned long long intr_clock[NR_MAX_IO_QUEUE + 1];
	static unsigned long long intr_counter[NR_MAX_IO_QUEUE + 1];

	unsigned long long prev_clock;
#endif

	NVMEV_INFO("%s started on cpu %d (node %d)\n", worker->thread_name, smp_processor_id(),
		   cpu_to_node(smp_processor_id()));

	while (!kthread_should_stop()) {
		unsigned long long curr_nsecs_wall = __get_wallclock();
		unsigned long long curr_nsecs_local = local_clock();
		long long delta = curr_nsecs_wall - curr_nsecs_local;

		volatile unsigned int curr = worker->io_seq;
		int qidx;

		while (curr != -1) {
			struct nvmev_io_work *w = &worker->work_queue[curr];
			unsigned long long curr_nsecs = local_clock() + delta;
			worker->latest_nsecs = curr_nsecs;

			if (w->is_completed == true) {
				curr = w->next;
				continue;
			}

			if (w->is_copied == false) {
				if (!w->p4_published) {
					if (!nvmev_p4_badcopy) {
						nvmev_p4_bc_sqid = w->sqid;
						nvmev_p4_bc_entry = curr;
						nvmev_p4_bc_sqe = w->sq_entry;
						nvmev_p4_bc_wid = w->wid;
						nvmev_p4_bc_idx = w->idx;
						nvmev_p4_bc_iss = w->nand_issued;
						nvmev_p4_bc_cmt = w->nand_committed;
					}
					nvmev_p4_badcopy++;
				}
#ifdef PERF_DEBUG
				w->nsecs_copy_start = local_clock() + delta;
#endif
				if (io_using_dma) {
					__do_perform_io_using_dma(w->sqid, w->sq_entry, &w->sqe);
				} else {
					__do_perform_io(w->sqid, w->sq_entry, &w->sqe);
				}

#ifdef PERF_DEBUG
				w->nsecs_copy_done = local_clock() + delta;
#endif
				w->is_copied = true;
				last_io_time = jiffies;

				NVMEV_DEBUG_VERBOSE("%s: copied %u, %d %d %d\n", worker->thread_name, curr,
					    w->sqid, w->cqid, w->sq_entry);
			}

			if (w->nsecs_target <= curr_nsecs) {
				__fill_cq_result(w);

				NVMEV_DEBUG_VERBOSE("%s: completed %u, %d %d %d\n", worker->thread_name, curr,
					    w->sqid, w->cqid, w->sq_entry);

#ifdef PERF_DEBUG
				w->nsecs_cq_filled = local_clock() + delta;
				trace_printk("%llu %llu %llu %llu %llu %llu\n", w->nsecs_start,
					     w->nsecs_enqueue - w->nsecs_start,
					     w->nsecs_copy_start - w->nsecs_start,
					     w->nsecs_copy_done - w->nsecs_start,
					     w->nsecs_cq_filled - w->nsecs_start,
					     w->nsecs_target - w->nsecs_start);
#endif
				mb();
				w->is_completed = true;
			}

			curr = w->next;
		}

		for (qidx = 1; qidx <= nvmev_vdev->nr_cq; qidx++) {
			struct nvmev_completion_queue *cq = nvmev_vdev->cqes[qidx];

#ifdef CONFIG_NVMEV_IO_WORKER_BY_SQ
			if ((worker->id) != __get_io_worker(qidx))
				continue;
#endif
			if (cq == NULL || !cq->irq_enabled)
				continue;

			if (mutex_trylock(&cq->irq_lock)) {
				if (cq->interrupt_ready == true) {
#ifdef PERF_DEBUG
					prev_clock = local_clock();
#endif
					cq->interrupt_ready = false;
					nvmev_signal_irq(cq->irq_vector);

#ifdef PERF_DEBUG
					intr_clock[qidx] += (local_clock() - prev_clock);
					intr_counter[qidx]++;

					if (intr_counter[qidx] > 1000) {
						NVMEV_DEBUG("Intr %d: %llu\n", qidx,
							    intr_clock[qidx] / intr_counter[qidx]);
						intr_clock[qidx] = 0;
						intr_counter[qidx] = 0;
					}
#endif
				}
				mutex_unlock(&cq->irq_lock);
			}
		}
		if (CONFIG_NVMEVIRT_IDLE_TIMEOUT != 0 &&
		    time_after(jiffies, last_io_time + (CONFIG_NVMEVIRT_IDLE_TIMEOUT * HZ)))
			schedule_timeout_interruptible(1);
		else
			cond_resched();
	}

	return 0;
}

void NVMEV_IO_WORKER_INIT(struct nvmev_dev *nvmev_vdev)
{
	unsigned int i, worker_id;

	nvmev_vdev->io_workers =
		kcalloc(nvmev_vdev->config.nr_io_workers, sizeof(struct nvmev_io_worker), GFP_KERNEL);
	nvmev_vdev->io_worker_turn = 0;

	for (worker_id = 0; worker_id < nvmev_vdev->config.nr_io_workers; worker_id++) {
		struct nvmev_io_worker *worker = &nvmev_vdev->io_workers[worker_id];

		worker->work_queue =
			vzalloc(sizeof(struct nvmev_io_work) * NR_MAX_PARALLEL_IO);
		if (!worker->work_queue) {
			NVMEV_ERROR("work_queue alloc failed (%zu B x %d)\n",
				    sizeof(struct nvmev_io_work) * NR_MAX_PARALLEL_IO,
				    nvmev_vdev->config.nr_io_workers);
			return;
		}
		for (i = 0; i < NR_MAX_PARALLEL_IO; i++) {
			worker->work_queue[i].next = i + 1;
			worker->work_queue[i].prev = i - 1;
		}
		worker->work_queue[NR_MAX_PARALLEL_IO - 1].next = -1;
		worker->id = worker_id;
		worker->free_seq = 0;
		worker->free_seq_end = NR_MAX_PARALLEL_IO - 1;
		worker->io_seq = -1;
		worker->io_seq_end = -1;

		snprintf(worker->thread_name, sizeof(worker->thread_name), "nvmev_io_worker_%d", worker_id);

		worker->task_struct = kthread_create(nvmev_io_worker, worker, "%s", worker->thread_name);

		kthread_bind(worker->task_struct, nvmev_vdev->config.cpu_nr_io_workers[worker_id]);
		wake_up_process(worker->task_struct);
	}
}

void NVMEV_IO_WORKER_FINAL(struct nvmev_dev *nvmev_vdev)
{
	unsigned int i;

	for (i = 0; i < nvmev_vdev->config.nr_io_workers; i++) {
		struct nvmev_io_worker *worker = &nvmev_vdev->io_workers[i];

		if (!IS_ERR_OR_NULL(worker->task_struct)) {
			kthread_stop(worker->task_struct);
		}

		vfree(worker->work_queue);
	}

	kfree(nvmev_vdev->io_workers);
}
