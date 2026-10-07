/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Double buffered advertising PDUs (AD data and scan response data) of
 * legacy advertising: the thread context allocates and fills the next PDU and
 * enqueues it, and the LLL switches to the latest enqueued PDU at the start
 * of each advertising channel PDU, returning the one it replaces.
 */

#define PDU_ADV_MEM_SIZE MROUND(PDU_AC_LL_HEADER_SIZE + PDU_AC_PAYLOAD_SIZE_MAX)

/* Initialize the PDU buffer pool, at power up and on HCI reset */
void lll_adv_pdu_init_reset(void);

int lll_adv_data_init(struct lll_adv_pdu *pdu);
int lll_adv_data_reset(struct lll_adv_pdu *pdu);
int lll_adv_data_release(struct lll_adv_pdu *pdu);

struct pdu_adv *lll_adv_pdu_alloc(struct lll_adv_pdu *pdu, uint8_t *idx);

/* Used by the LLL: switch to the latest enqueued PDU, if any. is_modified is
 * set if it did.
 */
struct pdu_adv *lll_adv_pdu_latest_get(struct lll_adv_pdu *pdu, uint8_t *is_modified);

static inline void lll_adv_pdu_enqueue(struct lll_adv_pdu *pdu, uint8_t idx)
{
	pdu->last = idx;
}

static inline struct pdu_adv *lll_adv_data_alloc(struct lll_adv *lll, uint8_t *idx)
{
	return lll_adv_pdu_alloc(&lll->adv_data, idx);
}

static inline void lll_adv_data_enqueue(struct lll_adv *lll, uint8_t idx)
{
	lll_adv_pdu_enqueue(&lll->adv_data, idx);
}

static inline struct pdu_adv *lll_adv_data_peek(struct lll_adv *lll)
{
	return (void *)lll->adv_data.pdu[lll->adv_data.last];
}

static inline struct pdu_adv *lll_adv_data_curr_get(struct lll_adv *lll)
{
	return (void *)lll->adv_data.pdu[lll->adv_data.first];
}

static inline struct pdu_adv *lll_adv_data_latest_get(struct lll_adv *lll, uint8_t *is_modified)
{
	return lll_adv_pdu_latest_get(&lll->adv_data, is_modified);
}

static inline struct pdu_adv *lll_adv_scan_rsp_alloc(struct lll_adv *lll, uint8_t *idx)
{
	return lll_adv_pdu_alloc(&lll->scan_rsp, idx);
}

static inline void lll_adv_scan_rsp_enqueue(struct lll_adv *lll, uint8_t idx)
{
	lll_adv_pdu_enqueue(&lll->scan_rsp, idx);
}

static inline struct pdu_adv *lll_adv_scan_rsp_peek(const struct lll_adv *lll)
{
	return (void *)lll->scan_rsp.pdu[lll->scan_rsp.last];
}

static inline struct pdu_adv *lll_adv_scan_rsp_curr_get(struct lll_adv *lll)
{
	return (void *)lll->scan_rsp.pdu[lll->scan_rsp.first];
}

static inline struct pdu_adv *lll_adv_scan_rsp_latest_get(struct lll_adv *lll,
							  uint8_t *is_modified)
{
	return lll_adv_pdu_latest_get(&lll->scan_rsp, is_modified);
}

static inline struct pdu_adv *lll_adv_pdu_latest_peek(const struct lll_adv_pdu *const pdu)
{
	uint8_t first;

	first = pdu->first;
	if (first != pdu->last) {
		first += 1U;
		if (first == DOUBLE_BUFFER_SIZE) {
			first = 0U;
		}
	}

	return (void *)pdu->pdu[first];
}

static inline struct pdu_adv *lll_adv_data_latest_peek(const struct lll_adv *const lll)
{
	return lll_adv_pdu_latest_peek(&lll->adv_data);
}
