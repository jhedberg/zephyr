/*
 * Copyright (c) 2018-2020 Nordic Semiconductor ASA
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/toolchain.h>

#include <zephyr/device.h>

#include <zephyr/drivers/entropy.h>
#include <zephyr/irq.h>

#include "hal/swi.h"
#include "hal/ccm.h"
#include "hal/cntr.h"
#include "hal/radio.h"
#include "hal/ticker.h"

#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "ticker/ticker.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_internal.h"
#include "lll_prof_internal.h"

#include "hal/debug.h"


/* Entropy device */
#if defined(CONFIG_ENTROPY_HAS_DRIVER)
static const struct device *const dev_entropy = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
#endif /* CONFIG_ENTROPY_HAS_DRIVER */

static int init_reset(void);
static void isr_race(void *param);

static void radio_bsim_isr(const void *arg)
{
	DEBUG_RADIO_ISR(1);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_enter_radio();
	}

	isr_radio();

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_exit_radio();
	}

	DEBUG_RADIO_ISR(0);
}

static void cntr_bsim_isr(const void *arg)
{
	DEBUG_TICKER_ISR(1);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_enter_ull_high();
	}

	/* The counter interrupt line is shared with the ULL mayflies */
	if (cntr_cmp_evt_get_clear()) {
		ticker_trigger(0);
	}

	mayfly_run(TICKER_USER_ID_ULL_HIGH);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_exit_ull_high();
	}

#if !defined(CONFIG_BT_CTLR_LOW_LAT) && \
	(CONFIG_BT_CTLR_ULL_HIGH_PRIO == CONFIG_BT_CTLR_ULL_LOW_PRIO)
	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_enter_ull_low();
	}

	mayfly_run(TICKER_USER_ID_ULL_LOW);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_exit_ull_low();
	}
#endif

	DEBUG_TICKER_ISR(0);
}

static void swi_lll_bsim_isr(const void *arg)
{
	DEBUG_RADIO_ISR(1);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_enter_lll();
	}

	mayfly_run(TICKER_USER_ID_LLL);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_exit_lll();
	}

	DEBUG_RADIO_ISR(0);
}

#if defined(CONFIG_BT_CTLR_LOW_LAT) || \
	(CONFIG_BT_CTLR_ULL_HIGH_PRIO != CONFIG_BT_CTLR_ULL_LOW_PRIO)
static void swi_ull_low_bsim_isr(const void *arg)
{
	DEBUG_TICKER_JOB(1);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_enter_ull_low();
	}

	mayfly_run(TICKER_USER_ID_ULL_LOW);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_exit_ull_low();
	}

	DEBUG_TICKER_JOB(0);
}
#endif

int lll_init(void)
{
	int err;

#if defined(CONFIG_ENTROPY_HAS_DRIVER)
	/* Get reference to entropy device */
	if (!device_is_ready(dev_entropy)) {
		return -ENODEV;
	}
#endif /* CONFIG_ENTROPY_HAS_DRIVER */

	/* Initialise LLL internals */
	lll_prepare_pipeline_init();

	/* Initialize Clocks */
	err = lll_clock_init();
	if (err < 0) {
		return err;
	}

	err = init_reset();
	if (err) {
		return err;
	}

	/* Initialize SW IRQ structure */
	hal_swi_init();

	/* Connect ISRs */
	IRQ_CONNECT(HAL_RADIO_IRQn, CONFIG_BT_CTLR_LLL_PRIO, radio_bsim_isr, NULL, 0);
	IRQ_CONNECT(HAL_RTC_IRQn, CONFIG_BT_CTLR_ULL_HIGH_PRIO, cntr_bsim_isr, NULL, 0);
	IRQ_CONNECT(HAL_SWI_RADIO_IRQ, CONFIG_BT_CTLR_LLL_PRIO, swi_lll_bsim_isr, NULL, 0);
#if defined(CONFIG_BT_CTLR_LOW_LAT) || \
	(CONFIG_BT_CTLR_ULL_HIGH_PRIO != CONFIG_BT_CTLR_ULL_LOW_PRIO)
	IRQ_CONNECT(HAL_SWI_JOB_IRQ, CONFIG_BT_CTLR_ULL_LOW_PRIO, swi_ull_low_bsim_isr, NULL, 0);
#endif

	/* Enable IRQs */
	irq_enable(HAL_RADIO_IRQn);
	irq_enable(HAL_RTC_IRQn);
	irq_enable(HAL_SWI_RADIO_IRQ);
	if (IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT) ||
		(CONFIG_BT_CTLR_ULL_HIGH_PRIO != CONFIG_BT_CTLR_ULL_LOW_PRIO)) {
		irq_enable(HAL_SWI_JOB_IRQ);
	}

	radio_setup();

	return 0;
}

int lll_deinit(void)
{
	int err;

	/* Release clocks */
	err = lll_clock_deinit();
	if (err < 0) {
		return err;
	}

	/* Disable IRQs */
	irq_disable(HAL_RADIO_IRQn);
	irq_disable(HAL_RTC_IRQn);
	irq_disable(HAL_SWI_RADIO_IRQ);
	if (IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT) ||
		(CONFIG_BT_CTLR_ULL_HIGH_PRIO != CONFIG_BT_CTLR_ULL_LOW_PRIO)) {
		irq_disable(HAL_SWI_JOB_IRQ);
	}

	return 0;
}

int lll_csrand_get(void *buf, size_t len)
{
#if defined(CONFIG_ENTROPY_HAS_DRIVER)
	return entropy_get_entropy(dev_entropy, buf, len);
#else /* !CONFIG_ENTROPY_HAS_DRIVER */
	return -ENODEV;
#endif /* !CONFIG_ENTROPY_HAS_DRIVER */
}

int lll_csrand_isr_get(void *buf, size_t len)
{
#if defined(CONFIG_ENTROPY_HAS_DRIVER)
	return entropy_get_entropy_isr(dev_entropy, buf, len, 0);
#else /* !CONFIG_ENTROPY_HAS_DRIVER */
	return -ENODEV;
#endif /* !CONFIG_ENTROPY_HAS_DRIVER */
}

int lll_rand_get(void *buf, size_t len)
{
	return lll_csrand_get(buf, len);
}

int lll_rand_isr_get(void *buf, size_t len)
{
	return lll_csrand_isr_get(buf, len);
}

int lll_reset(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int lll_is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
	return -ECANCELED;
}

void lll_abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	int err;

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* Perform event abort here.
		 * After event has been cleanly aborted, clean up resources
		 * and dispatch event done.
		 */
		radio_isr_set(lll_isr_done, param);
		radio_disable();
		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	lll_done(param);
}

uint32_t lll_event_offset_get(struct ull_hdr *ull)
{
	return HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_XTAL_US);
}

uint32_t lll_preempt_calc(struct ull_hdr *ull, uint8_t ticker_id,
		       uint32_t ticks_at_event)
{
	uint32_t ticks_now;
	uint32_t diff;

	ticks_now = ticker_ticks_now_get();
	diff = ticker_ticks_diff_get(ticks_now, ticks_at_event);
	if (diff & BIT(HAL_TICKER_CNTR_MSBIT)) {
		return 0;
	}

	diff += HAL_TICKER_CNTR_CMP_OFFSET_MIN +
		HAL_TICKER_US_TO_TICKS_CEIL(HAL_RADIO_ISR_LATENCY_MAX_US);
	if (diff > HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US)) {
		/* TODO: for Low Latency Feature with Advanced XTAL feature.
		 * 1. Release retained HF clock.
		 * 2. Advance the radio event to accommodate normal prepare
		 *    duration.
		 * 3. Increase the preempt to start ticks for future events.
		 */
		return diff;
	}

	return 0U;
}

void lll_chan_set(uint32_t chan)
{
	switch (chan) {
	case 37:
		radio_freq_chan_set(2);
		break;

	case 38:
		radio_freq_chan_set(26);
		break;

	case 39:
		radio_freq_chan_set(80);
		break;

	default:
		if (chan < 11) {
			radio_freq_chan_set(4 + (chan * 2U));
		} else if (chan < 40) {
			radio_freq_chan_set(28 + ((chan - 11) * 2U));
		} else {
			LL_ASSERT_DBG(0);
		}
		break;
	}

	radio_whiten_iv_set(chan);
}


uint32_t lll_radio_is_idle(void)
{
	return radio_is_idle();
}

uint32_t lll_radio_tx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return radio_tx_ready_delay_get(phy, flags);
}

uint32_t lll_radio_rx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return radio_rx_ready_delay_get(phy, flags);
}

int8_t lll_radio_tx_pwr_min_get(void)
{
	return radio_tx_power_min_get();
}

int8_t lll_radio_tx_pwr_max_get(void)
{
	return radio_tx_power_max_get();
}

int8_t lll_radio_tx_pwr_floor(int8_t tx_pwr_lvl)
{
	return radio_tx_power_floor(tx_pwr_lvl);
}

void lll_isr_tx_status_reset(void)
{
	radio_status_reset();
	radio_tmr_status_reset();

	if (IS_ENABLED(HAL_RADIO_GPIO_HAVE_PA_PIN) ||
	    IS_ENABLED(HAL_RADIO_GPIO_HAVE_LNA_PIN)) {
		radio_gpio_pa_lna_disable();
	}
}

void lll_isr_rx_status_reset(void)
{
	radio_status_reset();
	radio_tmr_status_reset();
	radio_rssi_status_reset();

	if (IS_ENABLED(HAL_RADIO_GPIO_HAVE_PA_PIN) ||
	    IS_ENABLED(HAL_RADIO_GPIO_HAVE_LNA_PIN)) {
		radio_gpio_pa_lna_disable();
	}
}

void lll_isr_tx_sub_status_reset(void)
{
	radio_status_reset();
	radio_tmr_tx_status_reset();

	if (IS_ENABLED(HAL_RADIO_GPIO_HAVE_PA_PIN) ||
	    IS_ENABLED(HAL_RADIO_GPIO_HAVE_LNA_PIN)) {
		radio_gpio_pa_lna_disable();
	}
}

void lll_isr_rx_sub_status_reset(void)
{
	radio_status_reset();
	radio_tmr_rx_status_reset();

	if (IS_ENABLED(HAL_RADIO_GPIO_HAVE_PA_PIN) ||
	    IS_ENABLED(HAL_RADIO_GPIO_HAVE_LNA_PIN)) {
		radio_gpio_pa_lna_disable();
	}
}

void lll_isr_status_reset(void)
{
	radio_status_reset();
	radio_tmr_status_reset();
	radio_filter_status_reset();
	if (IS_ENABLED(CONFIG_BT_CTLR_PRIVACY)) {
		radio_ar_status_reset();
	}
	radio_rssi_status_reset();

	if (IS_ENABLED(HAL_RADIO_GPIO_HAVE_PA_PIN) ||
	    IS_ENABLED(HAL_RADIO_GPIO_HAVE_LNA_PIN)) {
		radio_gpio_pa_lna_disable();
	}
}

inline void lll_isr_abort(void *param)
{
	lll_isr_status_reset();
	lll_isr_cleanup(param);
}

void lll_isr_done(void *param)
{
	lll_isr_abort(param);
}

void lll_isr_cleanup(void *param)
{
	int err;

	radio_isr_set(isr_race, param);
	if (!radio_is_idle()) {
		radio_disable();
	}

	radio_tmr_stop();
	radio_stop();

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	lll_done(NULL);
}

void lll_isr_early_abort(void *param)
{
	int err;

	radio_status_reset();

	radio_isr_set(isr_race, param);
	if (!radio_is_idle()) {
		radio_disable();
	}

	err = lll_hfclock_off();
	LL_ASSERT_ERR(err >= 0);

	lll_done(NULL);
}

static int init_reset(void)
{
	return 0;
}

static void isr_race(void *param)
{
	/* NOTE: lll_disable could have a race with ... */
	radio_status_reset();
}
