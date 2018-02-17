/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <clock_control.h>
#include <misc/printk.h>
#include <string.h>
#include <board.h>
#include <gpio.h>
#include <device.h>

#include <display/mb_display.h>

#define LED_PIN     EXT_P0_GPIO_PIN

static struct device *led_gpio;

#define BLOCKING ((void *)1)

static struct mb_image smiley = MB_IMAGE({ 0, 1, 0, 1, 0 },
					 { 0, 1, 0, 1, 0 },
					 { 0, 0, 0, 0, 0 },
					 { 1, 0, 0, 0, 1 },
					 { 0, 1, 1, 1, 0 });

/* WS2812 uses GRB 8:8:8 format */
struct rgb {
	u8_t g;
	u8_t r;
	u8_t b;
};

static void send_buf(struct rgb rgb[], size_t len)
{
	/* Address of OUTSET. OUTCLR is OUTSET + 4 */
	volatile u32_t *base = (u32_t *)(NRF_GPIO_BASE + 0x508);
	size_t buf_len = len * sizeof(struct rgb);
	u32_t pin = BIT(LED_PIN);
	u8_t *buf = (u8_t *)rgb;
	struct device *clock;
	unsigned int key;
	/* Initilization of i is strictly not needed, but it avoids an
	 * uninitialized warning with the inline assembly.
	 */
	u32_t i = 0;

	clock = device_get_binding(CONFIG_CLOCK_CONTROL_NRF5_M16SRC_DRV_NAME);
	if (!clock) {
		printk("Unable to get HF clock\n");
		return;
	}

	clock_control_on(clock, BLOCKING);
	key = irq_lock();

	while (buf_len--) {
		u32_t b = *buf++;

		/* Generate signal out of the bits, MSB. 1-bit should be
		 * roughly 0.85us high, 0.4us low, whereas a 0-bit should be
		 * roughly 0.4us high, 0.85us low.
		 */
		__asm volatile ("movs %[i], #8\n" /* i = 8 */
				".start_bit:\n"

				/* OUTSET = BIT(LED_PIN) */
				"strb %[p], [%[r], #0]\n"

				/* if (b & 0x80) goto .long */
				"tst %[b], %[m]\n"
				"bne .long\n"

				/* 0-bit */
				"nop\nnop\n"
				/* OUTCLR = BIT(LED_PIN) */
				"strb %[p], [%[r], #4]\n"
				"nop\nnop\nnop\n"
				"b .next_bit\n"

				/* 1-bit */
				".long:\n"
				"nop\nnop\nnop\nnop\nnop\nnop\nnop\n"
				/* OUTCLR = BIT(LED_PIN) */
				"strb %[p], [%[r], #4]\n"

				".next_bit:\n"
				/* b <<= 1 */
				"lsl %[b], #1\n"
				/* i-- */
				"sub %[i], #1\n"
				/* if (i > 0) goto .start_bit */
				"bne .start_bit\n"
				:
				[i] "+r" (i)
				:
				[b] "l" (b),
				[m] "l" (0x80),
				[r] "l" (base),
				[p] "r" (pin)
				:);
	}

	irq_unlock(key);
	clock_control_off(clock, NULL);
}

#define DELAY  K_SECONDS(1)

#define BLACK  { 0x00, 0x00, 0x00 }
#define WHITE  { 0xff, 0xff, 0xff }
#define GREY   { 0x33, 0x33, 0x33 }
#define GREEN  { 0xff, 0x00, 0x00 }
#define RED    { 0x00, 0xff, 0x00 }
#define BLUE   { 0x00, 0x00, 0xff }

#define ALL(name, color) struct rgb name[24] = { [ 0 ... 23 ] = color }
#define ONE(name, color) struct rgb name[1] = { color }

void main(void)
{
	struct mb_display *disp = mb_display_get();
	ALL(dim, WHITE);
	ONE(one_grey, GREY);
	ONE(one_black, BLACK);
	ALL(all_grey, GREY);
	ALL(all_black, BLACK);
	ALL(all_white, WHITE);
	ALL(all_green, GREEN);
	ALL(all_red, RED);
	ALL(all_blue, BLUE);
	int led;

	/* Show a smiley-face */
	mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, K_SECONDS(1),
			 &smiley, 1);
	k_sleep(K_SECONDS(2));

	led_gpio = device_get_binding(SW0_GPIO_NAME);
	gpio_pin_configure(led_gpio, LED_PIN, GPIO_DIR_OUT);

	send_buf(one_grey, ARRAY_SIZE(one_grey));
	k_sleep(DELAY);
	send_buf(one_black, ARRAY_SIZE(one_black));
	k_sleep(DELAY);

	send_buf(all_grey, ARRAY_SIZE(all_grey));
	k_sleep(DELAY);
	send_buf(all_black, ARRAY_SIZE(all_black));
	k_sleep(DELAY);

	send_buf(all_white, ARRAY_SIZE(all_white));
	k_sleep(DELAY);
	send_buf(all_black, ARRAY_SIZE(all_black));
	k_sleep(DELAY);

	send_buf(all_green, ARRAY_SIZE(all_green));
	k_sleep(DELAY);
	send_buf(all_black, ARRAY_SIZE(all_black));
	k_sleep(DELAY);

	send_buf(all_red, ARRAY_SIZE(all_red));
	k_sleep(DELAY);
	send_buf(all_black, ARRAY_SIZE(all_black));
	k_sleep(DELAY);

	send_buf(all_blue, ARRAY_SIZE(all_blue));
	k_sleep(DELAY);
	send_buf(all_black, ARRAY_SIZE(all_black));
	k_sleep(DELAY);

	/* Gradually dim all LEDs from max to min */
	while (dim[0].g) {
		int i;

		send_buf(dim, ARRAY_SIZE(dim));

		for (i = 0; i < ARRAY_SIZE(dim); i++) {
			struct rgb *rgb = &dim[i];

			rgb->r--;
			rgb->g--;
			rgb->b--;
		}

		k_sleep(K_MSEC(20));
	}

	/* Sequentially enable & disable each LED */
	for (led = 0; led < 24; led++) {
		struct rgb rgb[24];

		memset(rgb, 0, sizeof(rgb));
		memset(&rgb[led], 0xff, sizeof(rgb[led]));

		send_buf(rgb, ARRAY_SIZE(rgb));

		k_sleep(K_MSEC(200));
	}

	send_buf(all_black, ARRAY_SIZE(all_black));
}
