/*
 * Copyright (c) 2018-2020 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>

#include "hal/ccm.h"
#include "hal/ticker.h"

#include "util/util.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_df_types.h"
#include "lll_conn.h"
#include "lll_peripheral.h"

#include "lll_internal.h"
#include "lll_tim_internal.h"
#include "lll_conn_internal.h"

#include "hal/debug.h"

static int prepare_cb(struct lll_prepare_param *p);

int lll_periph_init(void)
{
	return 0;
}

int lll_periph_reset(void)
{
	return 0;
}

void lll_periph_prepare(void *param)
{
	int err;

	err = lll_hfclock_on();
	LL_ASSERT_ERR(err >= 0);

	err = lll_prepare(lll_conn_peripheral_is_abort_cb, lll_conn_abort_cb, prepare_cb, 0U,
			  param);
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);
}

static int prepare_cb(struct lll_prepare_param *p)
{
	struct lll_conn *lll = p->param;
	uint32_t window_us;
	uint32_t ticks_ref;
	uint32_t overhead;
	uint32_t start_us;
	uint8_t phy_rx;
	uint8_t chan;
	int err;

	DEBUG_RADIO_START_S(1);

	/* Check if stopped (on disconnection between prepare and preempt) */
	if (unlikely(lll->handle == 0xFFFF)) {
		lll_event_abort(lll);

		return 0;
	}

	chan = lll_conn_event_setup(lll, p);

	/* Accumulate window widening */
	lll->periph.window_widening_prepare_us += lll->periph.window_widening_periodic_us *
						  lll->lazy_prepare;
	if (lll->periph.window_widening_prepare_us > lll->periph.window_widening_max_us) {
		lll->periph.window_widening_prepare_us = lll->periph.window_widening_max_us;
	}

	/* Current window widening */
	lll->periph.window_widening_event_us += lll->periph.window_widening_prepare_us;
	if (lll->periph.window_widening_event_us > lll->periph.window_widening_max_us) {
		lll->periph.window_widening_event_us = lll->periph.window_widening_max_us;
	}

	/* Pre-increment window widening */
	lll->periph.window_widening_prepare_us = lll->periph.window_widening_periodic_us;

	/* Current window size */
	lll->periph.window_size_event_us += lll->periph.window_size_prepare_us;
	lll->periph.window_size_prepare_us = 0U;

#if defined(CONFIG_BT_CTLR_PHY)
	/* Back up rx PHY for use in drift compensation */
	lll->periph.phy_rx_event = lll->phy_rx;
	phy_rx = lll->phy_rx;
#else /* !CONFIG_BT_CTLR_PHY */
	phy_rx = PHY_1M;
#endif /* !CONFIG_BT_CTLR_PHY */

	/* Ensure that empty flag reflects the state of the Tx queue, as a
	 * peripheral if this is the first connection event and as no prior PDU
	 * is transmitted, an incorrect acknowledgment by peer should not
	 * dequeue a PDU that has not been transmitted on air.
	 */
	if (!lll->empty) {
		memq_link_t *link;

		/* Check for any Tx PDU at the head of the queue */
		link = memq_peek(lll->memq_tx.head, lll->memq_tx.tail, NULL);
		if (!link) {
			/* Update empty flag to reflect that no valid non-empty
			 * PDU was transmitted prior to this connection event.
			 */
			lll->empty = 1U;
		}
	}

	overhead = lll_preempt_calc(p);
	if (overhead) {
		if (p->defer == 1U) {
			/* We accept the overlap as previous event elected to continue */
			err = 0;
		} else {
			LL_ASSERT_OVERHEAD(overhead);

			err = -ECANCELED;
		}

		lll_event_abort(lll);

		return err;
	}

	/* The ULL starts the event the jitter, ticker resolution margin and
	 * window widening before the earliest expected anchor point. Listen
	 * until the latest one, plus the transmit window if any.
	 */
	start_us = lll_event_start_get(p, &ticks_ref);
	window_us = ((EVENT_JITTER_US + EVENT_TICKER_RES_MARGIN_US +
		      lll->periph.window_widening_event_us) << 1) +
		    lll->periph.window_size_event_us + addr_us_get(phy_rx);

	lll_conn_peripheral_start(lll, chan, start_us, window_us);

	err = lll_prepare_done(lll);
	LL_ASSERT_ERR(!err);

	DEBUG_RADIO_START_S(1);

	return 0;
}
