/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Radio operations of the BabbleSim LLL.
 *
 * The LLL asks the packet level 2.4GHz radio model of the board for one Tx or
 * Rx at a time, at an absolute time of the model's 1 MHz counter, which is
 * also the ticker counter. When the operation has ended, its callback is run
 * in the radio ISR with the model's event, and a role chains its next
 * operation from there, e.g. a Tx tIFS after the end of a received PDU.
 *
 * There is no radio ramp-up or Tx/Rx chain delay: a time given here is the
 * time of the first preamble bit on air, and the times in the event are the
 * on air times of the PDU.
 */

#include "bs_2g4_radio_if.h"

/* Radio operation end callback. evt->status is BSR_STATUS_ABORTED for an
 * operation stopped with lll_radio_stop(), or one that could not be started.
 */
typedef void (*lll_radio_cb_t)(const struct bsr_evt *evt, void *param);

void lll_radio_init(void);
void lll_radio_isr(void);

/* Transmit pdu (header and payload, without CRC) with its first bit at time
 * 'at'. The PDU is read when the transmission starts, as by a radio with DMA:
 * the buffer must be kept until then, and changes to it until then are sent,
 * e.g. an AuxPtr offset filled in after the Tx has been requested.
 */
void lll_radio_tx(const struct bsr_pkt_cfg *cfg, uint32_t at, const void *pdu,
		  lll_radio_cb_t cb, void *param);

/* Receive a PDU into buf, listening from time 'start'. With a window_us that
 * is not 0, the Rx ends without a PDU unless an access address has been
 * received within window_us from start. buf needs room for the PDU header and
 * cfg->max_len payload octets.
 */
void lll_radio_rx(const struct bsr_pkt_cfg *cfg, uint32_t start, uint32_t window_us,
		  void *buf, lll_radio_cb_t cb, void *param);

/* Stop the pending operation, if any, and call cb from the radio ISR as soon
 * as possible, as the end of an aborted operation.
 */
void lll_radio_stop(lll_radio_cb_t cb, void *param);

/* Stop the pending operation, if any, without any callback */
void lll_radio_abort(void);

static inline uint32_t lll_radio_now(void)
{
	return bsr_cntr_get();
}

/* Model PHY for a Bluetooth PHY (PHY_1M, PHY_2M, or 0 for legacy) */
static inline uint8_t lll_radio_phy(uint8_t phy)
{
	return (phy == BIT(1)) ? BSR_PHY_2M : BSR_PHY_1M;
}

/* Rx window for a PDU expected tifs_us after the end of a Tx at tx_end_us:
 * +/- the active clock jitter around the expected start, plus the range
 * delay, and the access address must have been received within it.
 */
static inline uint32_t lll_radio_tifs_rx_start(uint32_t tx_end_us, uint16_t tifs_us)
{
	return tx_end_us + tifs_us - EVENT_CLOCK_JITTER_US;
}

static inline uint32_t lll_radio_tifs_rx_window(uint8_t phy)
{
	return (EVENT_CLOCK_JITTER_US << 1) + RANGE_DELAY_US + addr_us_get(phy);
}
