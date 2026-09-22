/*
 * ADNS-5050 input driver for ZMK - bit-banged 3-wire serial bus, polled.
 *
 * Transport ported from QMK's drivers/sensors/adns5050.c:
 *   Copyright 2021 Colin Lam (Ploopy Corporation)
 *   Copyright 2020 Christopher Courtney (Drashna Jael're)
 *   Copyright 2019 Sunjun Kim
 *   Copyright 2019 Hiroyuki Okada
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Driver structure (polling, scroll layers, input_report_rel) adapted from
 * inorichi's zmk-pmw3610-driver, updated for ZMK v0.3.0 / Zephyr 3.5.
 */

#define DT_DRV_COMPAT pixart_adns5050

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>

#include "adns5050.h"

/* zmk/keymap.h is not exported to external modules on ZMK v0.3.0
 * (app/CMakeLists.txt keeps the include dir PRIVATE to the app target),
 * so declare the single API used here. Signature pinned to v0.3.0:
 * returns the highest active layer INDEX (uint8_t). */
typedef uint8_t zmk_keymap_layer_index_t;
zmk_keymap_layer_index_t zmk_keymap_highest_layer_active(void);

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adns5050, CONFIG_INPUT_LOG_LEVEL);

#define ADNS5050_POLL_MS 8

struct adns5050_config {
	struct gpio_dt_spec sclk;
	struct gpio_dt_spec sdio;
	struct gpio_dt_spec cs;
	uint16_t cpi;
	bool invert_x;
	bool invert_y;
	const int32_t *scroll_layers;
	size_t scroll_layers_len;
};

struct adns5050_data {
	const struct device *dev;
	struct k_timer poll_timer;
	struct k_work poll_work;
	struct k_work init_work;
	bool ready;
};

/* CS pulse while SCLK is low: resynchronizes the chip's serial state machine
 * between transactions, per QMK's adns5050_sync(). GPIO values are logical:
 * 1 = selected, 0 = deselected (cs-gpios is declared active low). */
static void adns5050_sync(const struct device *dev)
{
	const struct adns5050_config *config = dev->config;

	gpio_pin_set_dt(&config->cs, 1);
	k_busy_wait(1);
	gpio_pin_set_dt(&config->cs, 0);
}

static void adns5050_serial_write(const struct device *dev, uint8_t data)
{
	const struct adns5050_config *config = dev->config;
	int8_t b;

	/* SDIO switches to output at the START of the write phase, matching
	 * QMK's serial_write(). Between transactions it stays input. */
	gpio_pin_configure_dt(&config->sdio, GPIO_OUTPUT_INACTIVE);

	for (b = 7; b >= 0; b--) {
		gpio_pin_set_dt(&config->sclk, 0);
		gpio_pin_set_dt(&config->sdio, (data & BIT(b)) ? 1 : 0);
		k_busy_wait(2);
		gpio_pin_set_dt(&config->sclk, 1);
	}

	/* tSWR: minimum delay between writing a register address and reading
	 * data back from it. QMK waits 4us here unconditionally. */
	k_busy_wait(4);
}

static uint8_t adns5050_serial_read(const struct device *dev)
{
	const struct adns5050_config *config = dev->config;
	uint8_t byte = 0;
	uint8_t i;

	/* SDIO switches to input for the read phase. Pull-up keeps the line
	 * defined while the sensor drives it, and while it is high-Z between
	 * transactions (nRF input pins float without a pull). */
	gpio_pin_configure_dt(&config->sdio, GPIO_INPUT | GPIO_PULL_UP);

	for (i = 0; i < 8; i++) {
		gpio_pin_set_dt(&config->sclk, 0);
		k_busy_wait(1);
		byte = (byte << 1) | (gpio_pin_get_dt(&config->sdio) & 1);
		gpio_pin_set_dt(&config->sclk, 1);
		k_busy_wait(1);
	}

	/* No tail reconfigure: SDIO stays input until the next write phase
	 * begins (QMK structure). Driving it low here while CS is asserted
	 * - e.g. between the two burst bytes - fights the sensor's output
	 * driver and corrupts the second byte. */

	return byte;
}

static uint8_t adns5050_read_reg(const struct device *dev, uint8_t reg_addr)
{
	const struct adns5050_config *config = dev->config;
	uint8_t byte;

	adns5050_sync(dev);
	gpio_pin_set_dt(&config->cs, 1);
	adns5050_serial_write(dev, reg_addr);
	byte = adns5050_serial_read(dev);
	k_busy_wait(1); /* tSRR */
	gpio_pin_set_dt(&config->cs, 0);

	return byte;
}

static void adns5050_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t data)
{
	const struct adns5050_config *config = dev->config;

	adns5050_sync(dev);
	gpio_pin_set_dt(&config->cs, 1);
	adns5050_serial_write(dev, ADNS5050_SPI_ADDRESS_WRITE | reg_addr);
	adns5050_serial_write(dev, data);
	gpio_pin_set_dt(&config->cs, 0);
}

/* Motion burst: write burst address, read dX then dY, raise CS to end the
 * burst early (the remaining burst registers are not needed). */
static void adns5050_read_burst(const struct device *dev, int8_t *dx, int8_t *dy)
{
	const struct adns5050_config *config = dev->config;
	uint8_t x, y;

	adns5050_sync(dev);
	gpio_pin_set_dt(&config->cs, 1);
	adns5050_serial_write(dev, ADNS5050_REG_MOTION_BURST);
	x = adns5050_serial_read(dev);
	y = adns5050_serial_read(dev);
	gpio_pin_set_dt(&config->cs, 0);

	*dx = (int8_t)x;
	*dy = (int8_t)y;
}

/* QMK: value 0x1..0xD in MOUSE_CONTROL2 selects 125..1625 CPI in 125 steps. */
static void adns5050_set_cpi(const struct device *dev, uint16_t cpi)
{
	uint8_t cpival = (uint8_t)CLAMP(cpi / 125, 1, 13);

	adns5050_write_reg(dev, ADNS5050_REG_MOUSE_CONTROL2, BIT(4) | cpival);
}

/* Non-blocking init: reset, wait for wake-up, prime the serial interface,
 * apply CPI, then start the poll timer. Runs on the system workqueue. */
static void adns5050_init_work_fn(struct k_work *work)
{
	struct adns5050_data *data = CONTAINER_OF(work, struct adns5050_data, init_work);
	const struct device *dev = data->dev;
	const struct adns5050_config *config = dev->config;
	uint8_t pid, rid, pid2;
	int8_t dx, dy;

	gpio_pin_configure_dt(&config->sclk, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&config->sdio, GPIO_OUTPUT_INACTIVE);
	/* logical inactive = deselected = physically high (active-low pin) */
	gpio_pin_configure_dt(&config->cs, GPIO_OUTPUT_INACTIVE);

	adns5050_write_reg(dev, ADNS5050_REG_CHIP_RESET, ADNS5050_CHIP_RESET_MAGIC);
	k_msleep(55); /* datasheet maximum reset-to-ready time */

	adns5050_read_burst(dev, &dx, &dy); /* prime writes, discarded */

	pid = adns5050_read_reg(dev, ADNS5050_REG_PRODUCT_ID);
	rid = adns5050_read_reg(dev, ADNS5050_REG_REVISION_ID);
	pid2 = adns5050_read_reg(dev, ADNS5050_REG_PRODUCT_ID2);
	if (pid != ADNS5050_PRODUCT_ID || rid != ADNS5050_REVISION_ID ||
	    pid2 != ADNS5050_PRODUCT_ID2) {
		/* Signature mismatch: sensor absent, miswired, or the bus reads
		 * garbage. Do NOT start the poll timer and do NOT mark ready -
		 * a dead trackball beats a cursor that drifts on corrupt reads. */
		LOG_ERR("signature check FAILED: PID 0x%02x (want 0x%02x) "
			"REV 0x%02x (want 0x%02x) PID2 0x%02x (want 0x%02x) "
			"- trackball polling disabled, check wiring/SDIO",
			pid, ADNS5050_PRODUCT_ID, rid, ADNS5050_REVISION_ID, pid2,
			ADNS5050_PRODUCT_ID2);
		return;
	}

	adns5050_set_cpi(dev, config->cpi);

	data->ready = true;
	k_timer_start(&data->poll_timer, K_MSEC(ADNS5050_POLL_MS), K_MSEC(ADNS5050_POLL_MS));

	LOG_INF("ADNS-5050 ready, CPI %u", config->cpi);
}

static void adns5050_poll_timer_fn(struct k_timer *timer)
{
	struct adns5050_data *data = CONTAINER_OF(timer, struct adns5050_data, poll_timer);

	k_work_submit(&data->poll_work);
}

static void adns5050_poll_work_fn(struct k_work *work)
{
	struct adns5050_data *data = CONTAINER_OF(work, struct adns5050_data, poll_work);
	const struct device *dev = data->dev;
	const struct adns5050_config *config = dev->config;
	int8_t dx, dy;
	int32_t x, y;

	if (!data->ready) {
		return;
	}

	/* Gate on the MOTION register bit 7: skip the burst entirely when no
	 * motion occurred. At rest the deltas are 0 on a healthy read path,
	 * but ANY corrupted read becomes visible drift if reported, so do not
	 * trust delta reads that the sensor itself did not flag as motion.
	 * Reading MOTION does not clear Delta_X/Delta_Y (only writing to the
	 * Motion register does), so gate-then-burst loses nothing. */
	if (!(adns5050_read_reg(dev, ADNS5050_REG_MOTION) & BIT(7))) {
		return;
	}

	adns5050_read_burst(dev, &dx, &dy);
	if (dx == 0 && dy == 0) {
		return;
	}

	x = dx;
	y = dy;
	if (config->invert_x) {
		x = -x;
	}
	if (config->invert_y) {
		y = -y;
	}

	if (config->scroll_layers_len > 0) {
		uint8_t layer = zmk_keymap_highest_layer_active();
		bool scroll = false;
		size_t i;

		for (i = 0; i < config->scroll_layers_len; i++) {
			if (config->scroll_layers[i] == (int32_t)layer) {
				scroll = true;
				break;
			}
		}

		if (scroll) {
			if (y != 0) {
				input_report_rel(dev, INPUT_REL_WHEEL, y, true, K_FOREVER);
			}
			if (x != 0) {
				input_report_rel(dev, INPUT_REL_HWHEEL, x, true, K_FOREVER);
			}
			return;
		}
	}

	if (x != 0) {
		input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
	}
	if (y != 0) {
		input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
	}
}

static int adns5050_init(const struct device *dev)
{
	struct adns5050_data *data = dev->data;
	const struct adns5050_config *config = dev->config;

	if (config == NULL || !device_is_ready(config->sclk.port) ||
	    !device_is_ready(config->sdio.port) ||
	    !device_is_ready(config->cs.port)) {
		LOG_ERR("GPIO controller not ready");
		return -ENODEV;
	}

	data->dev = dev;
	k_work_init(&data->poll_work, adns5050_poll_work_fn);
	k_timer_init(&data->poll_timer, adns5050_poll_timer_fn, NULL);
	k_work_init(&data->init_work, adns5050_init_work_fn);
	k_work_submit(&data->init_work);

	return 0;
}

#define ADNS5050_DEFINE(n)                                                                         \
	static struct adns5050_data data##n;                                                       \
	static const int32_t scroll_layers##n[] = DT_PROP(DT_DRV_INST(n), scroll_layers);          \
	static const struct adns5050_config config##n = {                                          \
		.sclk = GPIO_DT_SPEC_INST_GET(n, sclk_gpios),                                      \
		.sdio = GPIO_DT_SPEC_INST_GET(n, sdio_gpios),                                      \
		.cs = GPIO_DT_SPEC_INST_GET(n, cs_gpios),                                          \
		.cpi = DT_INST_PROP(n, cpi),                                                       \
		.invert_x = DT_INST_PROP(n, invert_x),                                             \
		.invert_y = DT_INST_PROP(n, invert_y),                                             \
		.scroll_layers = scroll_layers##n,                                                 \
		.scroll_layers_len = DT_PROP_LEN(DT_DRV_INST(n), scroll_layers),                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, adns5050_init, NULL, &data##n, &config##n, POST_KERNEL,           \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ADNS5050_DEFINE)
