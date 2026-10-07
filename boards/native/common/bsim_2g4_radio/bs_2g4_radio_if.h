/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Packet level 2.4GHz radio for BabbleSim based boards.
 *
 * This is the interface between the embedded software (e.g. a Bluetooth LE
 * controller lower link layer) and a radio model running in the native
 * simulator runner context. It is independent of any vendor radio register
 * interface: the embedded side requests a Tx or Rx of a packet at an absolute
 * time, and the model raises an interrupt when the operation has ended.
 *
 * All times are values of a free running 32-bit microsecond counter, which
 * is also provided by the model (bsr_cntr_*) so that the embedded side can
 * use it as its only time base.
 *
 * The structures here are shared between the runner and the embedded image,
 * so they only use fixed size types.
 */

#ifndef BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_IF_H
#define BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_IF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Supported PHYs */
#define BSR_PHY_1M 1U
#define BSR_PHY_2M 2U

/* Packet configuration, common to Tx and Rx */
struct bsr_pkt_cfg {
	uint32_t aa;       /* Access address */
	uint32_t crc_init; /* CRC24 initial value */
	uint8_t phy;       /* BSR_PHY_* */
	uint8_t chan;      /* Bluetooth LE channel index, 0..39 */
	int8_t tx_power;   /* dBm, Tx only */
	uint8_t max_len;   /* Maximum payload length accepted on Rx */
};

/* bsr_evt.status values */
#define BSR_STATUS_OK      0U /* Packet sent, or received with valid CRC */
#define BSR_STATUS_CRC_ERR 1U /* Packet received, but with errors */
#define BSR_STATUS_NO_SYNC 2U /* Rx ended without finding a packet */
#define BSR_STATUS_ABORTED 3U /* Operation aborted with bsr_abort() */

struct bsr_evt {
	uint32_t id;        /* Id returned by bsr_tx()/bsr_rx() */
	uint32_t ts_start;  /* Time of the first preamble bit */
	uint32_t ts_aa_end; /* Time at which the access address ended */
	uint32_t ts_end;    /* Time at which the operation ended */
	uint8_t status;     /* BSR_STATUS_* */
	uint8_t is_rx;
	uint8_t len;        /* Received payload length (PDU header excluded) */
	int8_t rssi;        /* dBm, Rx only */
};

/*
 * Initialize the model, giving the interrupt lines used to signal the end of
 * a radio operation and the counter compare.
 */
void bsr_init(unsigned int radio_irq, unsigned int cntr_irq);

/* Current value of the 1 MHz counter */
uint32_t bsr_cntr_get(void);

/*
 * Raise the counter interrupt when the counter reaches the given value.
 * As with a hardware compare register, a value equal to the current counter
 * value only matches after the counter wraps.
 */
void bsr_cntr_cmp_set(uint32_t value);
void bsr_cntr_cmp_disable(void);

/*
 * Return true if the counter compare matched since the last call, clearing
 * the match. The counter interrupt line may be shared with other uses.
 */
bool bsr_cntr_cmp_evt_get_clear(void);

/*
 * Transmit a PDU (2 byte header followed by the payload, without CRC) with
 * its first preamble bit at time 'at'. The PDU is copied, and the model
 * appends the CRC.
 *
 * Returns an operation id, or 0 if the request could not be accepted (an
 * operation is already pending, or 'at' is not in the future).
 */
uint32_t bsr_tx(const struct bsr_pkt_cfg *cfg, uint32_t at, const uint8_t *pdu);

/*
 * Receive a PDU into buf, listening from time 'start'. If window_us is not 0,
 * the Rx ends with BSR_STATUS_NO_SYNC unless an access address has ended
 * within window_us from start. If window_us is 0, the Rx continues until
 * aborted.
 *
 * buf must have room for a 2 byte header and cfg->max_len payload bytes. It
 * is only written when the packet has ended.
 *
 * Returns an operation id, or 0 if the request could not be accepted.
 */
uint32_t bsr_rx(const struct bsr_pkt_cfg *cfg, uint32_t start, uint32_t window_us,
		uint8_t *buf);

/*
 * Abort the pending operation, if any. No event will be generated for it.
 */
void bsr_abort(void);

/*
 * Get the next radio event. Returns 1 if an event was returned, 0 otherwise.
 */
int bsr_evt_get(struct bsr_evt *evt);

#ifdef __cplusplus
}
#endif

#endif /* BOARDS_NATIVE_COMMON_BSIM_2G4_RADIO_BS_2G4_RADIO_IF_H */
