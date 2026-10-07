/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/arch/posix/posix_soc_if.h>

#include "util/mem.h"

#include "hal/cpu.h"
#include "hal/ccm.h"
#include "hal/swi.h"
#include "hal/radio.h"
#include "hal/ticker.h"

#include "ll_sw/pdu_df.h"
#include "lll/pdu_vendor.h"
#include "ll_sw/pdu.h"

#include "bs_2g4_radio_if.h"

#include "hal/debug.h"

enum trx {
	TRX_NONE,
	TRX_RX,
	TRX_TX,
};

static radio_isr_cb_t isr_cb;
static void *isr_cb_param;

static struct {
	struct bsr_pkt_cfg cfg;
	void *pkt_tx;
	void *pkt_rx;

	/* Timer start, all captures are relative to it */
	uint32_t tmr_start;
	uint32_t tmr_start_ticks;
	uint32_t tifs;

	/* Header complete timeout, absolute */
	uint32_t hcto;
	bool hcto_set;

	/* What to do tIFS after the current Tx/Rx */
	enum trx sw_next;

	/* Operation armed, and given to the radio model when the current
	 * execution context is done configuring it.
	 */
	enum trx armed;
	uint32_t armed_ready;
	bool armed_rssi;

	/* Operation given to the radio model */
	enum trx busy;
	uint32_t busy_id;

	/* radio_disable() was called, a DISABLED event is to be signalled */
	bool disabled_irq;
	/* radio_disable() was called from the ISR callback */
	bool disabled_in_isr;

	/* Status */
	bool ev_ready;
	bool ev_address;
	bool ev_end;
	bool ev_disabled;
	bool crc_valid;
	bool rssi_enable;
	bool rssi_ready;
	uint8_t rssi;
	uint32_t ready;
	uint32_t aa;
	uint32_t end;

	/* Device address filter */
	uint8_t filter_enable;
	uint8_t filter_addr_type;
	uint8_t filter_addr[8][BDADDR_SIZE];
	bool filter_match;
	uint8_t filter_match_idx;
} r;

static void op_arm(enum trx trx, uint32_t ready)
{
	r.armed = trx;
	r.armed_ready = ready;

	/* As the nRF timer capture of the radio ready event, only the timer
	 * started operations are captured, not the tIFS switched ones.
	 */
	r.ready = ready;

	/* Commit the operation from the radio ISR, once the caller is done
	 * setting up packet pointers, timeouts, etc.
	 */
	posix_sw_set_pending_IRQ(HAL_RADIO_IRQn);
}

static void op_commit(void)
{
	enum trx trx = r.armed;
	uint32_t id;

	if (trx == TRX_NONE) {
		return;
	}

	r.armed = TRX_NONE;

	if (trx == TRX_TX) {
		id = bsr_tx(&r.cfg, r.armed_ready, r.pkt_tx);
	} else {
		uint32_t window_us = 0U;

		if (r.hcto_set) {
			int32_t diff = (int32_t)(r.hcto - r.armed_ready);

			/* As with a timer compare, a timeout in the past has no
			 * effect.
			 */
			if (diff > 0) {
				window_us = diff;
			}
		}

		id = bsr_rx(&r.cfg, r.armed_ready, window_us, r.pkt_rx);
#if defined(RADIO_BSIM_TRACE)
		printk("RC rx t0 %u ready %u hcto %u(%d) win %u now %u\n", r.tmr_start,
		       r.armed_ready, r.hcto, r.hcto_set, window_us, bsr_cntr_get());
#endif
	}

	/* The LLL always starts the radio in the future */
	LL_ASSERT_ERR(id != 0U);

	r.busy = trx;
	r.busy_id = id;
}

static void filter_check(const uint8_t *pkt)
{
	const struct pdu_adv *pdu = (const void *)pkt;

	r.filter_match = false;

	for (uint8_t i = 0U; i < ARRAY_SIZE(r.filter_addr); i++) {
		if (!(r.filter_enable & BIT(i))) {
			continue;
		}

		if ((((r.filter_addr_type >> i) & 1U) == pdu->tx_addr) &&
		    !memcmp(&pdu->payload[0], r.filter_addr[i], BDADDR_SIZE)) {
			r.filter_match = true;
			r.filter_match_idx = i;
			break;
		}
	}
}

static void isr_radio_evt(const struct bsr_evt *evt)
{
	enum trx trx = r.busy;
	enum trx next;

	r.busy = TRX_NONE;

	r.ev_ready = true;
	r.ev_disabled = true;

	if (evt->status == BSR_STATUS_NO_SYNC) {
		/* Header complete timeout, the radio was disabled without an
		 * address match, there is no tIFS switch.
		 */
		r.sw_next = TRX_NONE;
		next = TRX_NONE;
	} else {
		r.ev_address = true;
		r.ev_end = true;
		r.aa = evt->ts_aa_end;
		r.end = evt->ts_end;

		if (trx == TRX_RX) {
			r.crc_valid = (evt->status == BSR_STATUS_OK);

			if (r.armed_rssi) {
				r.rssi = (uint8_t)(-evt->rssi);
				r.rssi_ready = true;
			}

			filter_check(r.pkt_rx);
		}

		next = r.sw_next;
		r.sw_next = TRX_NONE;
	}

	r.disabled_in_isr = false;

#if defined(RADIO_BSIM_TRACE)
	{
		const uint8_t *p = (trx == TRX_RX) ? r.pkt_rx : r.pkt_tx;

		printk("RT %s st %d ch %u aa %08x aa_end %u end %u hdr %02x %02x %02x %02x %02x\n",
		       (trx == TRX_RX) ? "rx" : "tx", evt->status, r.cfg.chan, r.cfg.aa,
		       evt->ts_aa_end, evt->ts_end, p[0], p[1], p[2], p[3], p[4]);
	}
#endif

	if (isr_cb) {
		isr_cb(isr_cb_param);
	}

	/* Perform the tIFS switch, unless the radio was disabled or started
	 * otherwise from the ISR callback.
	 */
	if ((next != TRX_NONE) && !r.disabled_in_isr && (r.armed == TRX_NONE) &&
	    (r.busy == TRX_NONE)) {
		uint32_t ready = r.end + r.tifs;

		if (next == TRX_RX) {
			ready -= HAL_RADIO_RX_TIFS_MARGIN_US;
			r.armed_rssi = r.rssi_enable;
		}

		r.armed = next;
		r.armed_ready = ready;
	}
}

void isr_radio(void)
{
	struct bsr_evt evt;

	while (bsr_evt_get(&evt)) {
		if ((r.busy == TRX_NONE) || (evt.id != r.busy_id)) {
			/* Event of an operation that has since been disabled */
			continue;
		}

		isr_radio_evt(&evt);
	}

	if (r.disabled_irq) {
		r.disabled_irq = false;

		if (radio_has_disabled() && isr_cb) {
			isr_cb(isr_cb_param);
		}
	}

	op_commit();
}

void radio_isr_set(radio_isr_cb_t cb, void *param)
{
	isr_cb_param = param;
	isr_cb = cb;
}

void radio_setup(void)
{
	bsr_init(HAL_RADIO_IRQn, HAL_RTC_IRQn);
}

void radio_reset(void)
{
	/* As a radio power cycle, no DISABLED event is generated */
	r.sw_next = TRX_NONE;
	r.armed = TRX_NONE;
	if (r.busy != TRX_NONE) {
		bsr_abort();
		r.busy = TRX_NONE;
	}
	r.disabled_irq = false;
	radio_status_reset();

	r.tifs = EVENT_IFS_US;
	r.cfg.tx_power = RADIO_TXP_DEFAULT;
	r.cfg.phy = BSR_PHY_1M;
	r.cfg.max_len = HAL_RADIO_PDU_LEN_MAX;
}

void radio_stop(void)
{
}

void radio_phy_set(uint8_t phy, uint8_t flags)
{
	ARG_UNUSED(flags);

	r.cfg.phy = (phy == PHY_2M) ? BSR_PHY_2M : BSR_PHY_1M;
}

void radio_tx_power_set(int8_t power)
{
	r.cfg.tx_power = power;
}

void radio_tx_power_max_set(void)
{
	r.cfg.tx_power = RADIO_TXP_MAX;
}

int8_t radio_tx_power_min_get(void)
{
	return RADIO_TXP_MIN;
}

int8_t radio_tx_power_max_get(void)
{
	return RADIO_TXP_MAX;
}

int8_t radio_tx_power_floor(int8_t power)
{
	return CLAMP(power, RADIO_TXP_MIN, RADIO_TXP_MAX);
}

void radio_freq_chan_set(uint32_t chan)
{
	/* chan is the frequency offset from 2400MHz, convert back to the
	 * Bluetooth LE channel index.
	 */
	switch (chan) {
	case 2:
		r.cfg.chan = 37U;
		break;
	case 26:
		r.cfg.chan = 38U;
		break;
	case 80:
		r.cfg.chan = 39U;
		break;
	default:
		if (chan < 26) {
			r.cfg.chan = (chan - 4U) / 2U;
		} else {
			r.cfg.chan = ((chan - 28U) / 2U) + 11U;
		}
		break;
	}
}

void radio_whiten_iv_set(uint32_t iv)
{
	/* Whitening is not modelled */
	ARG_UNUSED(iv);
}

void radio_aa_set(const uint8_t *aa)
{
	r.cfg.aa = sys_get_le32(aa);
}

void radio_pkt_configure(uint8_t bits_len, uint8_t max_len, uint8_t flags)
{
	ARG_UNUSED(bits_len);
	ARG_UNUSED(flags);

	r.cfg.max_len = max_len;
}

void radio_pkt_rx_set(void *rx_packet)
{
	r.pkt_rx = rx_packet;
}

void radio_pkt_tx_set(void *tx_packet)
{
	r.pkt_tx = tx_packet;
}

uint32_t radio_tx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return HAL_RADIO_TX_READY_DELAY_US;
}

uint32_t radio_tx_chain_delay_get(uint8_t phy, uint8_t flags)
{
	return 0U;
}

uint32_t radio_rx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return HAL_RADIO_RX_READY_DELAY_US;
}

uint32_t radio_rx_chain_delay_get(uint8_t phy, uint8_t flags)
{
	return 0U;
}

void radio_rx_enable(void)
{
	r.armed_rssi = r.rssi_enable;
	op_arm(TRX_RX, bsr_cntr_get() + HAL_RADIO_RX_READY_DELAY_US);
}

void radio_tx_enable(void)
{
	op_arm(TRX_TX, bsr_cntr_get() + HAL_RADIO_TX_READY_DELAY_US);
}

void radio_disable(void)
{
	r.sw_next = TRX_NONE;
	r.armed = TRX_NONE;
	r.disabled_in_isr = true;

	if (r.busy != TRX_NONE) {
		bsr_abort();
		r.busy = TRX_NONE;
	}

	/* As with the nRF RADIO, a DISABLED event is generated even when the
	 * radio is already disabled.
	 */
	r.ev_disabled = true;
	r.disabled_irq = true;
	posix_sw_set_pending_IRQ(HAL_RADIO_IRQn);
}

void radio_status_reset(void)
{
	r.ev_ready = false;
	r.ev_address = false;
	r.ev_end = false;
	r.ev_disabled = false;
}

uint32_t radio_is_ready(void)
{
	return r.ev_ready;
}

uint32_t radio_is_address(void)
{
	return r.ev_address;
}

uint32_t radio_is_done(void)
{
	return r.ev_end;
}

uint32_t radio_is_tx_done(void)
{
	return 1U;
}

uint32_t radio_has_disabled(void)
{
	return r.ev_disabled;
}

uint32_t radio_is_idle(void)
{
	return (r.armed == TRX_NONE) && (r.busy == TRX_NONE);
}

void radio_crc_configure(uint32_t polynomial, uint32_t iv)
{
	ARG_UNUSED(polynomial);

	r.cfg.crc_init = iv;
}

uint32_t radio_crc_is_valid(void)
{
	return r.crc_valid;
}

static uint8_t MALIGN(4) _pkt_empty[PDU_EM_LL_SIZE_MAX];
static uint8_t MALIGN(4) _pkt_scratch[MAX((HAL_RADIO_PDU_LEN_MAX + 3), PDU_AC_LL_SIZE_MAX)];

void *radio_pkt_empty_get(void)
{
	return _pkt_empty;
}

void *radio_pkt_scratch_get(void)
{
	return _pkt_scratch;
}

#if defined(CONFIG_BT_CTLR_LE_ENC)
static uint8_t MALIGN(4) _pkt_decrypt[MAX((HAL_RADIO_PDU_LEN_MAX + 3), PDU_AC_LL_SIZE_MAX)];

void *radio_pkt_decrypt_get(void)
{
	return _pkt_decrypt;
}
#endif /* CONFIG_BT_CTLR_LE_ENC */

void radio_switch_complete_and_rx(uint8_t phy_rx)
{
	r.sw_next = TRX_RX;
}

void radio_switch_complete_and_tx(uint8_t phy_rx, uint8_t flags_rx, uint8_t phy_tx,
				  uint8_t flags_tx)
{
	r.sw_next = TRX_TX;
}

void radio_switch_complete_with_delay_compensation_and_tx(
	uint8_t phy_rx, uint8_t flags_rx, uint8_t phy_tx, uint8_t flags_tx,
	enum radio_end_evt_delay_state end_evt_delay_en)
{
	r.sw_next = TRX_TX;
}

void radio_switch_complete_and_b2b_tx(uint8_t phy_curr, uint8_t flags_curr,
				      uint8_t phy_next, uint8_t flags_next)
{
	r.sw_next = TRX_TX;
}

void radio_switch_complete_and_b2b_rx(uint8_t phy_curr, uint8_t flags_curr,
				      uint8_t phy_next, uint8_t flags_next)
{
	r.sw_next = TRX_RX;
}

void radio_switch_complete_and_b2b_tx_disable(void)
{
	r.sw_next = TRX_NONE;
}

void radio_switch_complete_and_b2b_rx_disable(void)
{
	r.sw_next = TRX_NONE;
}

void radio_switch_complete_and_disable(void)
{
	r.sw_next = TRX_NONE;
}

void radio_switch_complete_end_capture_and_disable(void)
{
	r.sw_next = TRX_NONE;
}

uint8_t radio_phy_flags_rx_get(void)
{
	return 0U;
}

void radio_rssi_measure(void)
{
	r.rssi_enable = true;
	r.armed_rssi = true;
}

uint32_t radio_rssi_get(void)
{
	return r.rssi;
}

void radio_rssi_status_reset(void)
{
	r.rssi_enable = false;
	r.rssi_ready = false;
}

uint32_t radio_rssi_is_ready(void)
{
	return r.rssi_ready;
}

void radio_filter_configure(uint8_t bitmask_enable, uint8_t bitmask_addr_type,
			    uint8_t *bdaddr)
{
	r.filter_enable = bitmask_enable;
	r.filter_addr_type = bitmask_addr_type;
	memcpy(r.filter_addr, bdaddr, sizeof(r.filter_addr));
}

void radio_filter_disable(void)
{
	r.filter_enable = 0U;
}

void radio_filter_status_reset(void)
{
	r.filter_match = false;
}

uint32_t radio_filter_has_match(void)
{
	return r.filter_match;
}

uint32_t radio_filter_match_get(void)
{
	return r.filter_match_idx;
}

void radio_bc_configure(uint32_t n)
{
	ARG_UNUSED(n);
}

void radio_bc_status_reset(void)
{
}

uint32_t radio_bc_has_match(void)
{
	return 0U;
}

void radio_tmr_status_reset(void)
{
	r.hcto_set = false;
}

void radio_tmr_tx_status_reset(void)
{
}

void radio_tmr_rx_status_reset(void)
{
}

void radio_tmr_tx_enable(void)
{
}

void radio_tmr_rx_enable(void)
{
}

void radio_tmr_tx_disable(void)
{
}

void radio_tmr_rx_disable(void)
{
}

void radio_tmr_tifs_set(uint32_t tifs)
{
	r.tifs = tifs;
}

static uint32_t tmr_start_trx(uint8_t trx, uint32_t enable)
{
	if (trx) {
		op_arm(TRX_TX, enable + HAL_RADIO_TX_READY_DELAY_US);
	} else {
		r.armed_rssi = r.rssi_enable;
		op_arm(TRX_RX, enable + HAL_RADIO_RX_READY_DELAY_US);
	}

	return enable - r.tmr_start;
}

uint32_t radio_tmr_start(uint8_t trx, uint32_t ticks_start, uint32_t remainder)
{
	uint32_t remainder_us;

	/* Convert jitter to positive offset remainder in microseconds */
	hal_ticker_remove_jitter(&ticks_start, &remainder);
	remainder_us = remainder;

	r.tmr_start_ticks = ticks_start;
	r.tmr_start = HAL_TICKER_TICKS_TO_US(ticks_start);

	(void)tmr_start_trx(trx, r.tmr_start + remainder_us);

	return remainder_us;
}

uint32_t radio_tmr_start_tick(uint8_t trx, uint32_t ticks_start)
{
	r.tmr_start_ticks = ticks_start;
	r.tmr_start = HAL_TICKER_TICKS_TO_US(ticks_start);

	/* Setup compare event with min. 1 us offset */
	return tmr_start_trx(trx, r.tmr_start + 1U);
}

uint32_t radio_tmr_start_us(uint8_t trx, uint32_t start_us)
{
	uint32_t now_us = bsr_cntr_get() - r.tmr_start;

	if ((int32_t)(start_us - now_us) < 0) {
		start_us = now_us;
	}

	/* Setup compare event with min. 1 us offset */
	return tmr_start_trx(trx, r.tmr_start + start_us + 1U);
}

uint32_t radio_tmr_start_now(uint8_t trx)
{
	return radio_tmr_start_us(trx, bsr_cntr_get() - r.tmr_start);
}

uint32_t radio_tmr_start_get(void)
{
	return r.tmr_start_ticks;
}

uint32_t radio_tmr_start_latency_get(void)
{
	return 0U;
}

void radio_tmr_stop(void)
{
}

void radio_tmr_hcto_configure(uint32_t hcto_us)
{
	r.hcto = r.tmr_start + hcto_us;
	r.hcto_set = true;
}

void radio_tmr_hcto_configure_abs(uint32_t hcto_from_start_us)
{
	radio_tmr_hcto_configure(hcto_from_start_us);
}

void radio_tmr_aa_capture(void)
{
}

uint32_t radio_tmr_aa_get(void)
{
	return r.aa - r.tmr_start;
}

static uint32_t radio_tmr_aa;

void radio_tmr_aa_save(uint32_t aa)
{
	radio_tmr_aa = aa;
}

uint32_t radio_tmr_aa_restore(void)
{
	return radio_tmr_aa;
}

uint32_t radio_tmr_ready_get(void)
{
	return r.ready - r.tmr_start;
}

static uint32_t radio_tmr_ready;

void radio_tmr_ready_save(uint32_t ready)
{
	radio_tmr_ready = ready;
}

uint32_t radio_tmr_ready_restore(void)
{
	return radio_tmr_ready;
}

void radio_tmr_end_capture(void)
{
}

uint32_t radio_tmr_end_get(void)
{
	return r.end - r.tmr_start;
}

uint32_t radio_tmr_tifs_base_get(void)
{
	return radio_tmr_end_get();
}

static uint32_t tmr_sample;

void radio_tmr_sample(void)
{
	tmr_sample = bsr_cntr_get() - r.tmr_start;
}

uint32_t radio_tmr_sample_get(void)
{
	return tmr_sample;
}

int radio_gpio_pa_lna_init(void)
{
	return 0;
}

void radio_gpio_pa_lna_deinit(void)
{
}

void radio_gpio_pa_setup(void)
{
}

void radio_gpio_lna_setup(void)
{
}

void radio_gpio_pdn_setup(void)
{
}

void radio_gpio_lna_on(void)
{
}

void radio_gpio_lna_off(void)
{
}

void radio_gpio_pa_lna_enable(uint32_t trx_us)
{
	ARG_UNUSED(trx_us);
}

void radio_gpio_pa_lna_disable(void)
{
}

void *radio_ccm_rx_pkt_set(struct ccm *ccm, uint8_t phy, void *pkt)
{
	/* TODO: Encryption */
	ARG_UNUSED(ccm);
	ARG_UNUSED(phy);

	return pkt;
}

void *radio_ccm_tx_pkt_set(struct ccm *ccm, void *pkt)
{
	/* TODO: Encryption */
	ARG_UNUSED(ccm);

	return pkt;
}

uint32_t radio_ccm_is_done(void)
{
	return 1U;
}

uint32_t radio_ccm_mic_is_valid(void)
{
	return 1U;
}

void radio_ccm_disable(void)
{
}

void radio_ar_configure(uint32_t nirk, void *irk, uint8_t flags)
{
	ARG_UNUSED(nirk);
	ARG_UNUSED(irk);
	ARG_UNUSED(flags);
}

uint32_t radio_ar_match_get(void)
{
	return 0U;
}

void radio_ar_status_reset(void)
{
}

uint32_t radio_ar_has_match(void)
{
	return 0U;
}

uint8_t radio_ar_resolve(const uint8_t *addr)
{
	ARG_UNUSED(addr);

	return 0U;
}
