// SPDX-License-Identifier: GPL-2.0-only

#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/highmem.h>
#include <linux/sched/clock.h>
#include <linux/math64.h>

#include "nvmev.h"
#include "channel_model.h"

static inline unsigned long long __get_wallclock(void)
{
	return cpu_clock(nvmev_vdev->config.cpu_nr_dispatcher);
}

static inline bool chmodel_should_log_overflow(uint64_t count)
{
	return count <= 8 || !(count & (count - 1));
}

static uint64_t chmodel_transfer_latency_ns(struct channel_model *ch, uint64_t length)
{
	uint64_t units_to_xfer;
	uint64_t latency;

	if (!length)
		return 0;

	units_to_xfer = div64_u64(length + UNIT_XFER_SIZE - 1, UNIT_XFER_SIZE);

	if (units_to_xfer && ch->xfer_lat > div64_u64(U64_MAX, units_to_xfer))
		return U64_MAX;

	latency = units_to_xfer * ch->xfer_lat;

	if (ch->cmd_budget) {
		if (ch->xfer_lat && ch->cmd_budget > div64_u64(U64_MAX, ch->xfer_lat))
			return U64_MAX;
		if (latency > U64_MAX - (uint64_t)ch->cmd_budget * ch->xfer_lat)
			return U64_MAX;
		latency += (uint64_t)ch->cmd_budget * ch->xfer_lat;
	}

	return latency;
}

static uint64_t chmodel_overflow_fallback(struct channel_model *ch, uint64_t request_time,
						 uint64_t cur_time, uint64_t length,
						 uint32_t request_time_offs)
{
	uint64_t start_time = request_time;
	uint64_t xfer_latency = chmodel_transfer_latency_ns(ch, length);
	uint64_t completed_time;

	if (ch->overflow_tail_time > start_time)
		start_time = ch->overflow_tail_time;

	if (xfer_latency == U64_MAX || start_time > U64_MAX - xfer_latency)
		completed_time = U64_MAX;
	else
		completed_time = start_time + xfer_latency;

	ch->overflow_tail_time = completed_time;
	ch->overflow_count++;
	ch->overflow_bytes += length;
	ch->overflow_max_offs = max(ch->overflow_max_offs, request_time_offs);

	if (chmodel_should_log_overflow(ch->overflow_count))
		NVMEV_ERROR("[%s] budget horizon overflow: request_time=0x%llx cur_time=0x%llx "
			    "offs=0x%x entries=0x%x count=%llu bytes=%llu "
			    "fallback_complete=0x%llx\n",
			    __func__, request_time, cur_time, request_time_offs,
			    NR_BW_SLOTS, ch->overflow_count, ch->overflow_bytes,
			    completed_time);

	return completed_time;
}

void chmodel_init(struct channel_model *ch, uint64_t bandwidth )
{
	ch->head = 0;
	ch->valid_len = 0;
	ch->cur_time = 0;
	ch->slot_budget = BANDWIDTH_TO_SLOT_BUDGET(bandwidth);
	ch->cmd_budget = 0;
	ch->xfer_lat = BANDWIDTH_TO_TX_TIME(bandwidth);
	ch->overflow_tail_time = 0;
	ch->overflow_count = 0;
	ch->overflow_bytes = 0;
	ch->overflow_max_offs = 0;

	ch->win_hi = 0;
	ch->span_max = 0;
	ch->oow_past_n = 0;
	ch->oow_past_bytes = 0;
	ch->oow_slot_past_n = 0;
	ch->req_n = 0;
	ch->late_n = 0;
	ch->late_ns_sum = 0;
	ch->late_ns_max = 0;

	MEMSET(&(ch->avail_budget[0]), ch->slot_budget, NR_BW_SLOTS);

	memset(ch->slot_epoch, 0, sizeof(ch->slot_epoch));

	NVMEV_INFO("[%s] bandwidth %llu slot_budget %u tx_time %u\n", __func__, bandwidth,
		   ch->slot_budget, ch->xfer_lat);
}

static uint32_t chmodel_take(struct channel_model *ch, uint64_t t, uint32_t n, bool *overflowed)
{
	uint32_t start = chmodel_slot_of(t);
	uint32_t pos = start;
	uint32_t advanced = 0;

	*overflowed = false;
	while (n) {
		uint32_t want = chmodel_epoch_of(t + (uint64_t)advanced * UNIT_TIME_INTERVAL);
		uint32_t take;

		int32_t d = (int32_t)(want - ch->slot_epoch[pos]);

		if (d > 0) {
			ch->slot_epoch[pos] = want;
			ch->avail_budget[pos] = ch->slot_budget;
		} else if (d < 0) {
			ch->oow_slot_past_n++;
			advanced++;
			if (advanced >= NR_BW_SLOTS) {
				*overflowed = true;
				break;
			}
			pos = (pos + 1) % NR_BW_SLOTS;
			continue;
		}
		take = (n <= ch->avail_budget[pos]) ? n : ch->avail_budget[pos];
		ch->avail_budget[pos] -= take;
		n -= take;
		if (!n)
			break;

		advanced++;
		if (advanced >= NR_BW_SLOTS) {
			*overflowed = true;
			break;
		}
		pos = (pos + 1) % NR_BW_SLOTS;
	}
	return advanced;
}

uint64_t chmodel_request(struct channel_model *ch, uint64_t request_time, uint64_t length,
			 int lun_idx, uint64_t *cont_ns_out)
{
	uint32_t units_to_xfer = DIV_ROUND_UP(length, UNIT_XFER_SIZE);
	uint32_t remaining_budget = units_to_xfer * UNIT_XFER_BUDGET + ch->cmd_budget;
	uint32_t default_delay = ch->slot_budget ? remaining_budget / ch->slot_budget : 0;
	uint64_t total_latency;
	uint32_t delay;
	bool overflowed;

	if (cont_ns_out)
		*cont_ns_out = 0;
	ch->req_n++;

	if (request_time > ch->win_hi) {
		ch->win_hi = request_time;
	} else {
		uint64_t behind = ch->win_hi - request_time;

		if (behind > ch->span_max)
			ch->span_max = behind;
		if (behind >= (uint64_t)NR_BW_SLOTS * UNIT_TIME_INTERVAL) {
			ch->oow_past_n++;
			ch->oow_past_bytes += length;
			return request_time + chmodel_transfer_latency_ns(ch, length);
		}
	}

	delay = chmodel_take(ch, request_time, remaining_budget, &overflowed);

	if (overflowed)
		return chmodel_overflow_fallback(ch, request_time, request_time, length,
						 NR_BW_SLOTS);

	delay = (delay > default_delay) ? (delay - default_delay) : 0;

	{
		u64 ns = (u64)delay * UNIT_TIME_INTERVAL;

		if (cont_ns_out)
			*cont_ns_out = ns;
	}

	total_latency = (ch->xfer_lat * (units_to_xfer + ch->cmd_budget)) +
			((uint64_t)delay * UNIT_TIME_INTERVAL);

	return request_time + total_latency;
}
