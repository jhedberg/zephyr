/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Advertising PDU buffers of legacy advertising, shared between the thread
 * context that updates the AD and scan response data, and the LLL that
 * transmits them.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "hal/cpu.h"
#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/mfifo.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_adv_types.h"
#include "lll_adv.h"
#include "lll_adv_pdu.h"

#include "hal/debug.h"

#define PDU_FREE_TIMEOUT K_SECONDS(5)

/* AD data and scan response data need 2 PDU buffers each in the double
 * buffer, plus one buffer being filled while the LLL still holds the other
 * two (CONFIG_BT_CTLR_ADV_DATA_BUF_MAX is 1 without extended advertising).
 */
#define PDU_MEM_COUNT ((BT_CTLR_ADV_SET * 3) + CONFIG_BT_CTLR_ADV_DATA_BUF_MAX)

/* PDU buffers returned by the LLL: one per advertising set and update */
#define PDU_MEM_FIFO_COUNT (BT_CTLR_ADV_SET + 1)

static struct {
	void *free;
	uint8_t pool[PDU_ADV_MEM_SIZE * PDU_MEM_COUNT];
} mem_pdu;

/* PDU buffers the LLL has switched away from, returned to the thread context */
static MFIFO_DEFINE(pdu_free, sizeof(void *), PDU_MEM_FIFO_COUNT);

/* Wakes up a thread waiting for a PDU buffer to be returned */
static struct k_sem sem_pdu_free;

void lll_adv_pdu_init_reset(void)
{
	mem_init(mem_pdu.pool, PDU_ADV_MEM_SIZE, PDU_MEM_COUNT, &mem_pdu.free);

	MFIFO_INIT(pdu_free);

	k_sem_init(&sem_pdu_free, 0, PDU_MEM_FIFO_COUNT);
}

int lll_adv_data_init(struct lll_adv_pdu *pdu)
{
	struct pdu_adv *p;

	p = mem_acquire(&mem_pdu.free);
	if (!p) {
		return -ENOMEM;
	}

	p->len = 0U;
	pdu->pdu[0] = (void *)p;

	return 0;
}

int lll_adv_data_reset(struct lll_adv_pdu *pdu)
{
	/* Used on HCI reset, pdu[0] is assigned by a later lll_adv_data_init */
	pdu->first = 0U;
	pdu->last = 0U;
	pdu->pdu[1] = NULL;

	return 0;
}

int lll_adv_data_release(struct lll_adv_pdu *pdu)
{
	uint8_t last;
	void *p;

	last = pdu->last;
	p = pdu->pdu[last];
	if (p) {
		pdu->pdu[last] = NULL;
		mem_release(p, &mem_pdu.free);
	}

	last++;
	if (last == DOUBLE_BUFFER_SIZE) {
		last = 0U;
	}
	p = pdu->pdu[last];
	if (p) {
		pdu->pdu[last] = NULL;
		mem_release(p, &mem_pdu.free);
	}

	return 0;
}

static struct pdu_adv *pdu_buf_alloc(void)
{
	struct pdu_adv *p;
	int err;

	p = MFIFO_DEQUEUE_PEEK(pdu_free);
	if (p) {
		k_sem_reset(&sem_pdu_free);

		MFIFO_DEQUEUE(pdu_free);

		return p;
	}

	p = mem_acquire(&mem_pdu.free);
	if (p) {
		return p;
	}

	/* Wait for the LLL to switch to a newer PDU and return the old one */
	err = k_sem_take(&sem_pdu_free, PDU_FREE_TIMEOUT);
	LL_ASSERT_DBG(!err);

	k_sem_reset(&sem_pdu_free);

	p = MFIFO_DEQUEUE(pdu_free);
	LL_ASSERT_ERR(p);

	return p;
}

struct pdu_adv *lll_adv_pdu_alloc(struct lll_adv_pdu *pdu, uint8_t *idx)
{
	uint8_t first, last;
	void *p;

	first = pdu->first;
	last = pdu->last;
	if (first == last) {
		/* Return the index of the next free PDU in the double buffer */
		last++;
		if (last == DOUBLE_BUFFER_SIZE) {
			last = 0U;
		}
	} else {
		uint8_t first_latest;

		/* The LLL has not switched to the last enqueued PDU yet. Revert
		 * last so that it keeps using the first one while the caller
		 * updates the latest one.
		 *
		 * If the LLL switches before last is reverted, first has
		 * changed: restore last and return the index of the next free
		 * PDU. If it switches after, first is unchanged and the saved
		 * last is the free PDU.
		 */
		pdu->last = first;
		cpu_dmb();
		first_latest = pdu->first;
		if (first_latest != first) {
			pdu->last = last;
			last++;
			if (last == DOUBLE_BUFFER_SIZE) {
				last = 0U;
			}
		}
	}

	*idx = last;

	p = (void *)pdu->pdu[last];
	if (p) {
		return p;
	}

	p = pdu_buf_alloc();
	pdu->pdu[last] = (void *)p;

	return p;
}

struct pdu_adv *lll_adv_pdu_latest_get(struct lll_adv_pdu *pdu, uint8_t *is_modified)
{
	uint8_t first;

	first = pdu->first;
	if (first != pdu->last) {
		uint8_t free_idx;

		/* Keep the old PDU in its slot if it cannot be returned now, it
		 * is returned on a later switch.
		 */
		if (MFIFO_ENQUEUE_IDX_GET(pdu_free, &free_idx)) {
			MFIFO_BY_IDX_ENQUEUE(pdu_free, free_idx, pdu->pdu[first]);
			pdu->pdu[first] = NULL;

			k_sem_give(&sem_pdu_free);
		}

		first++;
		if (first == DOUBLE_BUFFER_SIZE) {
			first = 0U;
		}
		pdu->first = first;
		*is_modified = 1U;
	}

	return (void *)pdu->pdu[first];
}
