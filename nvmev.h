// SPDX-License-Identifier: GPL-2.0-only

#ifndef _LIB_NVMEV_H
#define _LIB_NVMEV_H

#include <linux/pci.h>
#include <linux/msi.h>
#include <asm/apic.h>

#include "nvme.h"

#define CONFIG_NVMEV_IO_WORKER_BY_SQ
#undef CONFIG_NVMEV_FAST_X86_IRQ_HANDLING

#undef CONFIG_NVMEV_VERBOSE
#undef CONFIG_NVMEV_DEBUG
#undef CONFIG_NVMEV_DEBUG_VERBOSE

#define CONFIG_NVMEVIRT_IDLE_TIMEOUT 60

#define NVMEV_DRV_NAME "NVMeVirt"
#define NVMEV_VERSION 0x0110
#define NVMEV_DEVICE_ID	NVMEV_VERSION
#define NVMEV_VENDOR_ID 0x0c51
#define NVMEV_SUBSYSTEM_ID	0x370d
#define NVMEV_SUBSYSTEM_VENDOR_ID NVMEV_VENDOR_ID

#define NVMEV_INFO(string, args...) printk(KERN_INFO "%s: " string, NVMEV_DRV_NAME, ##args)
#define NVMEV_ERROR(string, args...) printk(KERN_ERR "%s: " string, NVMEV_DRV_NAME, ##args)
#define NVMEV_ASSERT(x) BUG_ON((!(x)))

#ifdef CONFIG_NVMEV_DEBUG
#define  NVMEV_DEBUG(string, args...) printk(KERN_INFO "%s: " string, NVMEV_DRV_NAME, ##args)
#ifdef CONFIG_NVMEV_DEBUG_VERBOSE
#define  NVMEV_DEBUG_VERBOSE(string, args...) printk(KERN_INFO "%s: " string, NVMEV_DRV_NAME, ##args)
#else
#define  NVMEV_DEBUG_VERBOSE(string, args...)
#endif
#else
#define NVMEV_DEBUG(string, args...)
#define NVMEV_DEBUG_VERBOSE(string, args...)
#endif

#define NR_MAX_IO_QUEUE 72

#define NR_MAX_PARALLEL_IO 262144

#define NVMEV_INTX_IRQ 15

#define PAGE_OFFSET_MASK (PAGE_SIZE - 1)
#define PRP_PFN(x) ((unsigned long)((x) >> PAGE_SHIFT))

#define KB(k) ((k) << 10)
#define MB(m) ((m) << 20)
#define GB(g) ((g) << 30)

#define BYTE_TO_KB(b) ((b) >> 10)
#define BYTE_TO_MB(b) ((b) >> 20)
#define BYTE_TO_GB(b) ((b) >> 30)

#define MS_PER_SEC(s) ((s)*1000)
#define US_PER_SEC(s) (MS_PER_SEC(s) * 1000)
#define NS_PER_SEC(s) (US_PER_SEC(s) * 1000)

#define LBA_TO_BYTE(lba) ((lba) << LBA_BITS)
#define BYTE_TO_LBA(byte) ((byte) >> LBA_BITS)

#define BITMASK32_ALL (0xFFFFFFFF)
#define BITMASK64_ALL (0xFFFFFFFFFFFFFFFF)
#define ASSERT(X)

#include "ssd_config.h"

struct nvmev_sq_stat {
	unsigned int nr_dispatched;
	unsigned int nr_dispatch;
	unsigned int nr_in_flight;
	unsigned int max_nr_in_flight;
	unsigned long long total_io;
};

struct nvmev_submission_queue {
	int qid;
	int cqid;
	int priority;
	bool phys_contig;

	int queue_size;

	struct nvmev_sq_stat stat;

	struct nvme_command __iomem **sq;
	void *mapped;
};

struct nvmev_completion_queue {
	int qid;
	int irq_vector;
	bool irq_enabled;
	bool interrupt_ready;
	bool phys_contig;

	spinlock_t entry_lock;
	struct mutex irq_lock;

	int queue_size;

	int phase;
	int cq_head;
	int cq_tail;

	struct nvme_completion __iomem **cq;
	void *mapped;
};

struct nvmev_admin_queue {
	int phase;

	int sq_depth;
	int cq_depth;

	int cq_head;

	struct nvme_command __iomem **nvme_sq;
	struct nvme_completion __iomem **nvme_cq;
};

#define NR_SQE_PER_PAGE (PAGE_SIZE / sizeof(struct nvme_command))
#define NR_CQE_PER_PAGE (PAGE_SIZE / sizeof(struct nvme_completion))

#define SQ_ENTRY_TO_PAGE_NUM(entry_id) (entry_id / NR_SQE_PER_PAGE)
#define CQ_ENTRY_TO_PAGE_NUM(entry_id) (entry_id / NR_CQE_PER_PAGE)

#define SQ_ENTRY_TO_PAGE_OFFSET(entry_id) (entry_id % NR_SQE_PER_PAGE)
#define CQ_ENTRY_TO_PAGE_OFFSET(entry_id) (entry_id % NR_CQE_PER_PAGE)

struct nvmev_config {
	unsigned long memmap_start;
	unsigned long memmap_size;

	unsigned long storage_start;
	unsigned long storage_size;

	unsigned int cpu_nr_dispatcher;
	int cpu_nr_qserve;
	unsigned int nr_io_workers;
	unsigned int cpu_nr_io_workers[32];

	unsigned int nr_io_units;
	unsigned int io_unit_shift;

	unsigned int read_delay;
	unsigned int read_time;
	unsigned int read_trailing;
	unsigned int write_delay;
	unsigned int write_time;
	unsigned int write_trailing;
};

struct nvmev_io_work {
	int sqid;
	int cqid;

	int sq_entry;
	unsigned int command_id;

	struct nvme_command sqe;

	unsigned long long nsecs_start;
	unsigned long long nsecs_target;

	unsigned long long nsecs_enqueue;
	unsigned long long nsecs_copy_start;
	unsigned long long nsecs_copy_done;
	unsigned long long nsecs_cq_filled;

	bool is_copied;
	bool is_completed;

	unsigned int status;
	unsigned int result0;
	unsigned int result1;

	unsigned int next, prev;

	unsigned int wid;
	unsigned int idx;

	unsigned int nand_issued;
	unsigned int nand_committed;
	bool issue_done;
	bool p4_published;
};

struct nvmev_io_worker {
	struct nvmev_io_work *work_queue;

	unsigned int free_seq;
	unsigned int free_seq_end;
	unsigned int io_seq;
	unsigned int io_seq_end;

	unsigned long long latest_nsecs;

	unsigned int id;

	unsigned int in_use, in_use_hwm;
	struct task_struct *task_struct;
	char thread_name[32];
};

struct nvmev_dev {
	struct pci_bus *virt_bus;
	void *virtDev;
	struct pci_header *pcihdr;
	struct pci_pm_cap *pmcap;
	struct pci_msix_cap *msixcap;
	struct pcie_cap *pciecap;
	struct pci_ext_cap *extcap;

	struct pci_dev *pdev;

	struct nvmev_config config;
	struct task_struct *nvmev_dispatcher;
	struct task_struct *nvmev_qserve;

	void *storage_mapped;

	struct nvmev_io_worker *io_workers;
	unsigned int io_worker_turn;

	void __iomem *msix_table;

	bool intx_disabled;

	struct __nvme_bar *old_bar;
	struct nvme_ctrl_regs __iomem *bar;

	u32 *old_dbs;
	u32 __iomem *dbs;

	struct nvmev_ns *ns;
	unsigned int nr_ns;
	unsigned int nr_sq;
	unsigned int nr_cq;

	struct nvmev_admin_queue *admin_q;
	struct nvmev_submission_queue *sqes[NR_MAX_IO_QUEUE + 1];
	struct nvmev_completion_queue *cqes[NR_MAX_IO_QUEUE + 1];

	unsigned int mdts;

	struct proc_dir_entry *proc_root;
	struct proc_dir_entry *proc_read_times;
	struct proc_dir_entry *proc_write_times;
	struct proc_dir_entry *proc_io_units;
	struct proc_dir_entry *proc_stat;
	struct proc_dir_entry *proc_debug;

	unsigned long long *io_unit_stat;
};

struct nvmev_request {
	struct nvme_command *cmd;
	uint32_t sq_id;
	uint64_t nsecs_start;
};

struct nvmev_result {
	uint32_t status;
	uint64_t nsecs_target;
};

struct nvmev_ns {
	uint32_t id;
	uint32_t csi;
	uint64_t size;
	void *mapped;

	uint32_t nr_parts;
	void *ftls;

	bool (*proc_io_cmd)(struct nvmev_ns *ns, struct nvmev_request *req,
			    struct nvmev_result *ret);

	bool (*identify_io_cmd)(struct nvmev_ns *ns, struct nvme_command cmd);

	unsigned int (*perform_io_cmd)(struct nvmev_ns *ns, struct nvme_command *cmd,
				       uint32_t *status);

	void (*qserve_pass)(struct nvmev_ns *ns, unsigned long long now);

	void (*outbox_drain)(struct nvmev_ns *ns);

	void (*gc_tick)(struct nvmev_ns *ns);
};

extern struct nvmev_dev *nvmev_vdev;
struct nvmev_dev *VDEV_INIT(void);
void VDEV_FINALIZE(struct nvmev_dev *nvmev_vdev);

bool nvmev_proc_bars(void);
bool NVMEV_PCI_INIT(struct nvmev_dev *dev);
void nvmev_signal_irq(int msi_index);

void nvmev_proc_admin_sq(int new_db, int old_db);

struct buffer;
struct ssd;
extern u64 nvmev_p4_relink, nvmev_p4_badsq, nvmev_p4_stale_owner, nvmev_p4_badcopy,
	   nvmev_p4_lost_cmpl, nvmev_p4_sq_stall, nvmev_bp_fetch_skipped;

#define CMDBOX_SZ   4096
#define CMDBOX_MASK (CMDBOX_SZ - 1)

struct cmdbox_ent {
	struct nvmev_ns *ns;
	struct nvme_command sqe;
	int sq_id;
	int cq_id;
	int sq_entry;
	uint16_t command_id;
	struct p4_cmd *p4c;
	struct nvmev_io_worker *worker;
	unsigned int entry;
	unsigned long long nsecs_start;
	size_t io_size;
};

struct cmdbox {
	struct cmdbox_ent slot[CMDBOX_SZ];
	uint32_t head;
	uint32_t tail;
};

#define RETBOX_SZ   4096

struct retbox_ent {
	struct nvme_command sqe;
	struct p4_cmd *p4c;
	struct nvmev_io_worker *worker;
	unsigned int entry;
	int sqid;
	int cqid;
	int sq_entry;
	uint16_t command_id;
	uint16_t status;
	unsigned long long nsecs_start;
	unsigned long long nsecs_target;
	size_t io_size;
	bool ok;
};

struct retbox {
	struct retbox_ent slot[RETBOX_SZ];
	uint32_t head;
	uint32_t tail;
};

#define P4_RING_SZ (NR_MAX_PARALLEL_IO * 4)
extern u32 nvmev_p4_early_seen;
extern u32 nvmev_p4_lostc_seen;
extern u32 nvmev_p4_stuck_seen, nvmev_p4_stuck_issued, nvmev_p4_stuck_committed,
	   nvmev_p4_stuck_worker, nvmev_p4_stuck_age;

extern u64 nvmev_p4_nn_rd, nvmev_p4_nn_wr, nvmev_p4_nn_fl, nvmev_p4_nn_other;
void nvmev_dep_stats_reset(void);

extern unsigned int nvmev_p4_bc_sqid, nvmev_p4_bc_entry, nvmev_p4_bc_sqe, nvmev_p4_bc_wid,
		    nvmev_p4_bc_idx, nvmev_p4_bc_iss, nvmev_p4_bc_cmt;
extern uint32_t nvmev_cur_cmd;
void nvmev_note_issued(uint32_t owner);

void nvmev_p4_commit_token(uint32_t ticket);

void nvmev_p4_commit_at(uint32_t ticket, uint64_t t);

int nvmev_wload_finish(uint32_t own, uint64_t t, bool forced);
void nvmev_diq_full_wake(uint64_t done);
void nvmev_diq_dump(const char *why);

bool nvmev_diq_room(void);
int nvmev_diq_bp_eval(void);

void nvmev_diq_resume(uint32_t frame, uint64_t done);

void nvmev_apply_completion(struct ssd *ssd, uint32_t owner, uint32_t xfer, uint64_t chnl_end,
			    uint32_t frame, uint32_t fill);

void NVMEV_IO_WORKER_INIT(struct nvmev_dev *nvmev_vdev);
void NVMEV_IO_WORKER_FINAL(struct nvmev_dev *nvmev_vdev);
int nvmev_proc_io_sq(int qid, int new_db, int old_db);
void nvmev_proc_io_cq(int qid, int new_db, int old_db);

#endif
