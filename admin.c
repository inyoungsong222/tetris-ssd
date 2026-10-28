// SPDX-License-Identifier: GPL-2.0-only

#include "nvmev.h"
#include "conv_ftl.h"

#define sq_entry(entry_id) \
	queue->nvme_sq[SQ_ENTRY_TO_PAGE_NUM(entry_id)][SQ_ENTRY_TO_PAGE_OFFSET(entry_id)]
#define cq_entry(entry_id) \
	queue->nvme_cq[CQ_ENTRY_TO_PAGE_NUM(entry_id)][CQ_ENTRY_TO_PAGE_OFFSET(entry_id)]

#define prp_address_offset(prp, offset) \
	(page_address(pfn_to_page(prp >> PAGE_SHIFT) + offset) + (prp & ~PAGE_MASK))
#define prp_address(prp) prp_address_offset(prp, 0)

static void __make_cq_entry_results(int eid, u16 ret, u32 result0, u32 result1)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_common_command *cmd = &sq_entry(eid).common;
	int cq_head = queue->cq_head;

	cq_entry(cq_head) = (struct nvme_completion) {
		.command_id = cmd->command_id,
		.sq_id = 0,
		.sq_head = eid,
		.result0 = result0,
		.result1 = result1,
		.status = queue->phase | (ret << 1),
	};

	if (++cq_head == queue->cq_depth) {
		cq_head = 0;
		queue->phase = !queue->phase;
	}
	queue->cq_head = cq_head;
}

static void __make_cq_entry(int eid, u16 ret)
{
	__make_cq_entry_results(eid, ret, 0, 0);
}

static void __nvmev_admin_create_cq(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvmev_completion_queue *cq;
	struct nvme_create_cq *cmd = &sq_entry(eid).create_cq;
	unsigned int num_pages, i;
	int dbs_idx;

	cq = kzalloc(sizeof(struct nvmev_completion_queue), GFP_KERNEL);

	cq->qid = cmd->cqid;

	cq->irq_enabled = cmd->cq_flags & NVME_CQ_IRQ_ENABLED ? true : false;
	if (cq->irq_enabled) {
		cq->irq_vector = cmd->irq_vector;
	}
	cq->interrupt_ready = false;

	cq->queue_size = cmd->qsize + 1;
	cq->phase = 1;

	cq->cq_head = 0;
	cq->cq_tail = -1;

	spin_lock_init(&cq->entry_lock);
	mutex_init(&cq->irq_lock);

	cq->phys_contig = cmd->cq_flags & NVME_QUEUE_PHYS_CONTIG ? true : false;
	WARN_ON(!cq->phys_contig);

	num_pages = DIV_ROUND_UP(cq->queue_size * sizeof(struct nvme_completion), PAGE_SIZE);
	cq->cq = kzalloc(sizeof(struct nvme_completion *) * num_pages, GFP_KERNEL);

	if (pfn_valid(cmd->prp1 >> PAGE_SHIFT)) {
		cq->mapped = NULL;
		for (i = 0; i < num_pages; i++)
			cq->cq[i] = prp_address_offset(cmd->prp1, i);
	} else {
		cq->mapped = memremap(cmd->prp1, num_pages * PAGE_SIZE, MEMREMAP_WT);
		for (i = 0; i < num_pages; i++)
			cq->cq[i] = (void *)((uint64_t)cq->mapped + i * PAGE_SIZE);
	}

	nvmev_vdev->cqes[cq->qid] = cq;

	dbs_idx = cq->qid * 2 + 1;
	nvmev_vdev->dbs[dbs_idx] = nvmev_vdev->old_dbs[dbs_idx] = 0;

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_delete_cq(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvmev_completion_queue *cq;
	unsigned int qid;

	qid = sq_entry(eid).delete_queue.qid;

	cq = nvmev_vdev->cqes[qid];
	nvmev_vdev->cqes[qid] = NULL;

	if (cq) {
		kfree(cq->cq);
		if (cq->mapped)
			memunmap(cq->mapped);
		kfree(cq);
	}

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_create_sq(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_create_sq *cmd = &sq_entry(eid).create_sq;
	struct nvmev_submission_queue *sq;
	unsigned int num_pages, i;
	int dbs_idx;

	sq = kzalloc(sizeof(struct nvmev_submission_queue), GFP_KERNEL);

	sq->qid = cmd->sqid;
	sq->cqid = cmd->cqid;

	sq->priority = cmd->sq_flags & 0xFFFE;
	sq->queue_size = cmd->qsize + 1;

	sq->phys_contig = (cmd->sq_flags & NVME_QUEUE_PHYS_CONTIG) ? true : false;
	WARN_ON(!sq->phys_contig);

	num_pages = DIV_ROUND_UP(sq->queue_size * sizeof(struct nvme_command), PAGE_SIZE);
	sq->sq = kzalloc(sizeof(struct nvme_command *) * num_pages, GFP_KERNEL);

	if (pfn_valid(cmd->prp1 >> PAGE_SHIFT)) {
		sq->mapped = NULL;
		for (i = 0; i < num_pages; i++)
			sq->sq[i] = prp_address_offset(cmd->prp1, i);
	} else {
		sq->mapped = memremap(cmd->prp1, num_pages * PAGE_SIZE, MEMREMAP_WT);
		for (i = 0; i < num_pages; i++)
			sq->sq[i] = (void *)((uint64_t)sq->mapped + i * PAGE_SIZE);
	}

	nvmev_vdev->sqes[sq->qid] = sq;

	dbs_idx = sq->qid * 2;
	nvmev_vdev->dbs[dbs_idx] = 0;
	nvmev_vdev->old_dbs[dbs_idx] = 0;

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_delete_sq(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_delete_queue *cmd = &sq_entry(eid).delete_queue;
	struct nvmev_submission_queue *sq;
	unsigned int qid;

	qid = cmd->qid;

	sq = nvmev_vdev->sqes[qid];
	nvmev_vdev->sqes[qid] = NULL;

	if (sq) {
		kfree(sq->sq);
		if (sq->mapped)
			memunmap(sq->mapped);
		kfree(sq);
	}

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_get_log_page(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_get_log_page_command *cmd = &sq_entry(eid).get_log_page;
	void *page;
	uint32_t len = ((((uint32_t)cmd->numdu << 16) | cmd->numdl) + 1) << 2;

	page = prp_address(cmd->prp1);

	switch (cmd->lid) {
	case NVME_LOG_SMART: {
		static const struct nvme_smart_log smart_log = {
			.critical_warning = 0,
			.spare_thresh = 20,
			.host_reads[0] = cpu_to_le64(0),
			.host_writes[0] = cpu_to_le64(0),
			.num_err_log_entries[0] = cpu_to_le64(0),
			.temperature[0] = 0 & 0xff,
			.temperature[1] = (0 >> 8) & 0xff,
		};

		__memcpy(page, &smart_log, len);
		break;
	}
	case NVME_LOG_CMD_EFFECTS: {
		static const struct nvme_effects_log effects_log = {
			.acs = {
				[nvme_admin_get_log_page] = cpu_to_le32(NVME_CMD_EFFECTS_CSUPP),
				[nvme_admin_identify] = cpu_to_le32(NVME_CMD_EFFECTS_CSUPP),

				[nvme_admin_set_features] = cpu_to_le32(NVME_CMD_EFFECTS_CSUPP),
				[nvme_admin_get_features] = cpu_to_le32(NVME_CMD_EFFECTS_CSUPP),
				[nvme_admin_async_event] = cpu_to_le32(NVME_CMD_EFFECTS_CSUPP),

				[nvme_admin_directive_send] = cpu_to_le32(NVME_CMD_EFFECTS_CSUPP),
				[nvme_admin_directive_recv] = cpu_to_le32(NVME_CMD_EFFECTS_CSUPP),
			},
			.iocs = {
			},
			.resv = { 0, },
		};

		__memcpy(page, &effects_log, len);
		break;
	}
	default:

		NVMEV_ERROR("Unimplemented log page identifier: 0x%hhx,"
			    "the system will be unstable!\n", cmd->lid);
		__memset(page, 0, len);
		break;
	}

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_identify_namespace(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_id_ns *ns;
	struct nvme_identify *cmd = &sq_entry(eid).identify;
	size_t nsid = cmd->nsid - 1;

	ns = prp_address(cmd->prp1);
	memset(ns, 0x0, PAGE_SIZE);

	ns->lbaf[0].ms = 0;
	ns->lbaf[0].ds = 9;
	ns->lbaf[0].rp = NVME_LBAF_RP_GOOD;

	ns->lbaf[1].ms = 8;
	ns->lbaf[1].ds = 9;
	ns->lbaf[1].rp = NVME_LBAF_RP_GOOD;

	ns->lbaf[2].ms = 16;
	ns->lbaf[2].ds = 9;
	ns->lbaf[2].rp = NVME_LBAF_RP_GOOD;

	ns->lbaf[3].ms = 0;
	ns->lbaf[3].ds = 12;
	ns->lbaf[3].rp = NVME_LBAF_RP_BEST;

	ns->lbaf[4].ms = 8;
	ns->lbaf[4].ds = 12;
	ns->lbaf[4].rp = NVME_LBAF_RP_BEST;

	ns->lbaf[5].ms = 64;
	ns->lbaf[5].ds = 12;
	ns->lbaf[5].rp = NVME_LBAF_RP_BEST;

	ns->lbaf[6].ms = 128;
	ns->lbaf[6].ds = 12;
	ns->lbaf[6].rp = NVME_LBAF_RP_BEST;

	if (LBA_BITS == 9) {
		ns->flbas = 0;
	} else if (LBA_BITS == 12) {
		ns->flbas = 3;
	} else {
		BUG();
	}

	ns->nlbaf = 6;
	ns->dps = 0;

	ns->nsze = (nvmev_vdev->ns[nsid].size >> ns->lbaf[ns->flbas].ds);
	ns->ncap = ns->nsze;
	ns->nuse = ns->nsze;

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_identify_namespaces(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_identify *cmd = &sq_entry(eid).identify;
	unsigned int *ns;
	int i;

	ns = prp_address(cmd->prp1);
	memset(ns, 0x00, PAGE_SIZE * 2);

	for (i = 1; i <= nvmev_vdev->nr_ns; i++) {
		if (i > cmd->nsid) {
			*ns = i;
			ns++;
		}
	}

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_identify_namespace_desc(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_identify *cmd = &sq_entry(eid).identify;
	struct nvme_id_ns_desc *ns_desc;
	int nsid = cmd->nsid - 1;

	ns_desc = prp_address(cmd->prp1);
	memset(ns_desc, 0x00, sizeof(*ns_desc));

	ns_desc->nidt = NVME_NIDT_CSI;
	ns_desc->nidl = 1;

	ns_desc->nid[0] = nvmev_vdev->ns[nsid].csi;

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_identify_cs_ctrl(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_identify *cmd = &sq_entry(eid).identify;
	__u8 *res = prp_address(cmd->prp1);

	res[0] = 0;

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_identify_ctrl(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_identify *cmd = &sq_entry(eid).identify;
	struct nvme_id_ctrl *ctrl;

	ctrl = prp_address(cmd->prp1);
	memset(ctrl, 0x00, sizeof(*ctrl));

	ctrl->nn = nvmev_vdev->nr_ns;
	ctrl->oncs = 0;

	ctrl->oacs = cpu_to_le16(NVME_CTRL_OACS_DIRECTIVES);
	ctrl->acl = 3;

	ctrl->vwc = 0;
	snprintf(ctrl->sn, sizeof(ctrl->sn), "CSL_Virt_SN_%02d", 1);
	snprintf(ctrl->mn, sizeof(ctrl->mn), "CSL_Virt_MN_%02d", 1);
	snprintf(ctrl->fr, sizeof(ctrl->fr), "CSL_%03d", 2);
	ctrl->mdts = nvmev_vdev->mdts;
	ctrl->sqes = 0x66;
	ctrl->cqes = 0x44;

	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_directive_send(int eid)
{
	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static void __nvmev_admin_directive_recv(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_directive_cmd *cmd = &sq_entry(eid).directive;

	if (cmd->dtype == NVME_DIR_STREAMS && cmd->doper == NVME_DIR_RCV_ST_OP_PARAM) {
		struct streams_directive_params *s = prp_address(cmd->prp1);

		memset(s, 0, sizeof(*s));
		s->msl  = cpu_to_le16(8);
		s->nssa = cpu_to_le16(8);
		s->nsso = cpu_to_le16(0);
		s->sws  = cpu_to_le32((ONESHOT_PAGE_SIZE) / LBA_SIZE);
		s->sgs  = cpu_to_le16(1);
		s->nsa  = cpu_to_le16(8);
		s->nso  = cpu_to_le16(0);
		__make_cq_entry(eid, NVME_SC_SUCCESS);
		return;
	}

	__make_cq_entry_results(eid, NVME_SC_SUCCESS, 8, 0);
}

static void __nvmev_admin_identify(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	int cns = sq_entry(eid).identify.cns;

	switch (cns) {
	case 0x00:
		__nvmev_admin_identify_namespace(eid);
		break;
	case 0x01:
		__nvmev_admin_identify_ctrl(eid);
		break;
	case 0x02:
		__nvmev_admin_identify_namespaces(eid);
		break;
	case 0x03:
		__nvmev_admin_identify_namespace_desc(eid);
		break;
	case 0x05:
		__make_cq_entry(eid, NVME_SC_SUCCESS);
		break;
	case 0x06:
		__nvmev_admin_identify_cs_ctrl(eid);
		break;
	default:
		__make_cq_entry(eid, NVME_SC_INVALID_OPCODE);
		NVMEV_ERROR("I don't know %d\n", cns);
	}
}

static void __nvmev_admin_set_features(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_features *cmd = &sq_entry(eid).features;
	__le32 result0 = 0;
	__le32 result1 = 0;

	switch (cmd->fid) {
	case NVME_FEAT_ARBITRATION:
	case NVME_FEAT_POWER_MGMT:
	case NVME_FEAT_LBA_RANGE:
	case NVME_FEAT_TEMP_THRESH:
	case NVME_FEAT_ERR_RECOVERY:
	case NVME_FEAT_VOLATILE_WC:
		break;
	case NVME_FEAT_NUM_QUEUES: {
		int num_queue;

		num_queue = (sq_entry(eid).features.dword11 & 0xFFFF) + 1;
		nvmev_vdev->nr_sq = min(num_queue, NR_MAX_IO_QUEUE);

		num_queue = ((sq_entry(eid).features.dword11 >> 16) & 0xFFFF) + 1;
		nvmev_vdev->nr_cq = min(num_queue, NR_MAX_IO_QUEUE);

		result0 = ((nvmev_vdev->nr_cq - 1) << 16 | (nvmev_vdev->nr_sq - 1));
		break;
	}
	case NVME_FEAT_IRQ_COALESCE:
	case NVME_FEAT_IRQ_CONFIG:
	case NVME_FEAT_WRITE_ATOMIC:
	case NVME_FEAT_ASYNC_EVENT:
	case NVME_FEAT_AUTO_PST:
	case NVME_FEAT_SW_PROGRESS:
	case NVME_FEAT_HOST_ID:
	case NVME_FEAT_RESV_MASK:
	case NVME_FEAT_RESV_PERSIST:
	default:
		break;
	}

	__make_cq_entry_results(eid, NVME_SC_SUCCESS, result0, result1);
}

static void __nvmev_admin_get_features(int eid)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_features *cmd = &sq_entry(eid).features;
	__le32 result0 = 0;
	__le32 result1 = 0;

	switch (cmd->fid) {
	case NVME_FEAT_ARBITRATION:
	case NVME_FEAT_POWER_MGMT:
	case NVME_FEAT_LBA_RANGE:
	case NVME_FEAT_TEMP_THRESH:
	case NVME_FEAT_ERR_RECOVERY:
	case NVME_FEAT_VOLATILE_WC:
		break;
	case NVME_FEAT_NUM_QUEUES:
		result0 = ((nvmev_vdev->nr_cq - 1) << 16 | (nvmev_vdev->nr_sq - 1));
		break;
	case NVME_FEAT_IRQ_COALESCE:
	case NVME_FEAT_IRQ_CONFIG:
	case NVME_FEAT_WRITE_ATOMIC:
	case NVME_FEAT_ASYNC_EVENT:
	case NVME_FEAT_AUTO_PST:
	case NVME_FEAT_SW_PROGRESS:
	case NVME_FEAT_HOST_ID:
	case NVME_FEAT_RESV_MASK:
	case NVME_FEAT_RESV_PERSIST:
	default:
		break;
	}

	__make_cq_entry_results(eid, NVME_SC_SUCCESS, result0, result1);
}

static void __nvmev_admin_async_event(int eid)
{
	__make_cq_entry(eid, NVME_SC_SUCCESS);
}

static u64 nvmev_abort_n;

static void __nvmev_admin_abort(int eid)
{
	nvmev_abort_n++;

	if (nvmev_abort_n <= 3)
		nvmev_diq_dump("abort");
	__make_cq_entry_results(eid, NVME_SC_SUCCESS, 1, 0);
}

static void __nvmev_proc_admin_req(int entry_id)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	struct nvme_command *sqe = &sq_entry(entry_id);

	NVMEV_DEBUG("%s: %d 0x%x 0x%x\n", __func__, entry_id,
			sqe->common.opcode, sqe->common.command_id);

	switch (sqe->common.opcode) {
	case nvme_admin_delete_sq:
		__nvmev_admin_delete_sq(entry_id);
		break;
	case nvme_admin_create_sq:
		__nvmev_admin_create_sq(entry_id);
		break;
	case nvme_admin_get_log_page:
		__nvmev_admin_get_log_page(entry_id);
		break;
	case nvme_admin_delete_cq:
		__nvmev_admin_delete_cq(entry_id);
		break;
	case nvme_admin_create_cq:
		__nvmev_admin_create_cq(entry_id);
		break;
	case nvme_admin_identify:
		__nvmev_admin_identify(entry_id);
		break;
	case nvme_admin_abort_cmd:
		__nvmev_admin_abort(entry_id);
		break;
	case nvme_admin_set_features:
		__nvmev_admin_set_features(entry_id);
		break;
	case nvme_admin_get_features:
		__nvmev_admin_get_features(entry_id);
		break;
	case nvme_admin_async_event:
		__nvmev_admin_async_event(entry_id);
		break;
	case nvme_admin_directive_send:
		__nvmev_admin_directive_send(entry_id);
		break;
	case nvme_admin_directive_recv:
		__nvmev_admin_directive_recv(entry_id);
		break;
	case nvme_admin_activate_fw:
	case nvme_admin_download_fw:
	case nvme_admin_format_nvm:
	case nvme_admin_security_send:
	case nvme_admin_security_recv:
	default:
		__make_cq_entry(entry_id, NVME_SC_INVALID_OPCODE);
		NVMEV_ERROR("Unhandled admin requests: %d", sqe->common.opcode);
		break;
	}
}

void nvmev_proc_admin_sq(int new_db, int old_db)
{
	struct nvmev_admin_queue *queue = nvmev_vdev->admin_q;
	int num_proc = new_db - old_db;
	int curr = old_db;
	int seq;

	if (num_proc < 0)
		num_proc += queue->sq_depth;

	for (seq = 0; seq < num_proc; seq++) {
		__nvmev_proc_admin_req(curr++);

		if (curr == queue->sq_depth) {
			curr = 0;
		}
	}

	nvmev_signal_irq(0);
}
