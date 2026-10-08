/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Used by the common prepare pipeline */
int lll_prepare_done(void *param);
int lll_done(void *param);
bool lll_is_done(void *param, bool *is_resume);

/* Returns the start of the event, its first packet on air, and in ticks_ref the
 * whole tick at or before it, which the times reported to the ULL are relative
 * to (node_rx_ftr.ticks_anchor).
 */
uint32_t lll_event_start_get(const struct lll_prepare_param *p, uint32_t *ticks_ref);

/* Returns 0 if the radio event of a prepare can still start on time, else
 * how many ticks it is late.
 */
uint32_t lll_preempt_calc(const struct lll_prepare_param *p);

/* Turn a prepare param of a radio event that was preempted into the one of a
 * resume starting as soon as possible.
 */
void lll_resume_param_set(struct lll_prepare_param *p);

/* End the current radio event from the radio ISR, without any done extra,
 * e.g. when it is stopped in its prepare. The event must not be ended
 * synchronously there, as its done can start the next prepare.
 */
void lll_event_abort(void *param);

/* The callbacks of the radio events that are never resumed, and that need no
 * clean up of their own when aborted.
 */
int lll_is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
void lll_abort_cb(struct lll_prepare_param *prepare_param, void *param);

/* End the current radio event now, from a radio ISR callback */
void lll_isr_cleanup(void *param);

/* Offset from the first payload of the event of the payload sent in subevent
 * se of a BIS. The first IRC x BN subevents send the BN payloads of the event
 * IRC times, and each next group of BN subevents the payloads of the event
 * PTO events further ahead (pre-transmissions).
 */
static inline uint16_t lll_big_payload_offset(uint8_t bn, uint8_t irc, uint8_t pto, uint8_t se)
{
	uint8_t group = se / bn;
	uint8_t n = se % bn;

	if (group < irc) {
		return n;
	}

	return ((group - irc + 1U) * pto * bn) + n;
}

/* RSSI of a received packet, as the positive magnitude in dBm that the ULL
 * and HCI expect.
 */
static inline uint8_t lll_rssi_get(int8_t rssi_dbm)
{
	return CLAMP(-(int16_t)rssi_dbm, 0, 126);
}
