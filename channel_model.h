// SPDX-License-Identifier: GPL-2.0-only

#ifndef _CHANNEL_MODEL_H
#define _CHANNEL_MODEL_H

#define NR_BW_SLOTS (1024 * 384)
#define UNIT_TIME_INTERVAL (1000ULL)
#define UNIT_XFER_SIZE (32ULL)
#define UNIT_XFER_BUDGET (1)

#define SIZE_OF_BWBUDGET_T 2

#if (SIZE_OF_BWBUDGET_T == 1)
typedef uint8_t bwbudget_t;
#define MEMSET(dest, value, length) memset(dest, value, length)

#elif (SIZE_OF_BWBUDGET_T == 2)
typedef uint16_t bwbudget_t;
#define MEMSET(dest, value, length) memset16(dest, value, length)

#elif (SIZE_OF_BWBUDGET_T == 4)
typedef uint32_t bwbudget_t;
#define MEMSET(dest, value, length) memset32(dest, value, length)
#else
#error "Invalid budget size"
#endif

struct channel_model {
	uint64_t cur_time;
	uint32_t head;
	uint32_t valid_len;
	uint32_t slot_budget;
	uint32_t cmd_budget;
	uint32_t xfer_lat;

	uint64_t overflow_tail_time;
	uint64_t overflow_count;
	uint64_t overflow_bytes;
	uint32_t overflow_max_offs;

	uint64_t win_hi;
	uint64_t span_max;
	uint64_t oow_past_n;
	uint64_t oow_past_bytes;
	uint64_t oow_slot_past_n;

	uint64_t req_n;
	uint64_t late_n;
	uint64_t late_ns_sum;
	uint64_t late_ns_max;

	bwbudget_t avail_budget[NR_BW_SLOTS];

	uint32_t slot_epoch[NR_BW_SLOTS];
};

static inline uint32_t chmodel_slot_of(uint64_t t)
{
	return (uint32_t)((t / UNIT_TIME_INTERVAL) % NR_BW_SLOTS);
}

static inline uint32_t chmodel_epoch_of(uint64_t t)
{
	return (uint32_t)((t / UNIT_TIME_INTERVAL) / NR_BW_SLOTS);
}

#define BANDWIDTH_TO_TX_TIME(MB_S) \
	((((UNIT_XFER_SIZE) * NS_PER_SEC(1)) + (MB(MB_S)) / 2) / (MB(MB_S)))
#define BANDWIDTH_TO_SLOT_BUDGET(MB_S) \
	(MB(MB_S) * UNIT_TIME_INTERVAL / NS_PER_SEC(1) / UNIT_XFER_SIZE * UNIT_XFER_BUDGET)

uint64_t chmodel_request(struct channel_model *ch, uint64_t request_time, uint64_t length,
			 int lun_idx, uint64_t *cont_ns_out);
void chmodel_init(struct channel_model *ch, uint64_t bandwidth );
#endif
