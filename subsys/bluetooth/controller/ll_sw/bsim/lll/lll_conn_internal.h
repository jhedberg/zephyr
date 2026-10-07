/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Connection events, common to the central and peripheral roles */

/* Set up the connection event of a prepare: reset the event state, and
 * update the event counter and latency. Returns the data channel of the
 * event.
 */
uint8_t lll_conn_event_setup(struct lll_conn *lll, const struct lll_prepare_param *p);

/* Start the connection event as central, transmitting at start_us */
void lll_conn_central_start(struct lll_conn *lll, uint8_t chan, uint32_t start_us);

/* Start the connection event as peripheral, receiving from start_us. The
 * access address of the central's packet must have been received within
 * window_us.
 */
void lll_conn_peripheral_start(struct lll_conn *lll, uint8_t chan, uint32_t start_us,
			       uint32_t window_us);
