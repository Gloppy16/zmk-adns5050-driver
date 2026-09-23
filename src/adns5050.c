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
 *
 * Serial timing (ADNS-5050 datasheet AV02-1045EN p12 AC table):
 *   tSWW       30 us   between the two data bytes of a write command
 *   tSWR       20 us   write command to read command (address to address)
 *   tSRAD      4-10 us read address to read data (10 us used here)
 *   tSCLK-NCS  20 us   write: last SCLK rising edge to NCS rising edge
 *              120 ns  read case (satisfied by the read tail)
 * QMK (and earlier versions of this port) raised NCS ~4 us after the last
 * write bit, below the 20 us write minimum, so register writes could abort.
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
/* Register at a fixed level, NOT a CONFIG_* symbol: this driver's only log
 * lines are the init verdict ("ready" / "signature check FAILED"), which is
 * the bench oracle. A Kconfig default (or a stray =0 in a conf file) must
 * never be able to compile the oracle out. LOG_LEVEL_INF is defined as 3U in
 * zephyr/logging/log_core.h (Zephyr 3.5) and is accepted directly by
 * LOG_MODULE_REGISTER (the level argument is used verbatim, no Kconfig
 * indirection). */
LOG_MODULE_REGISTER(adns5050, LOG_LEVEL_INF);

#define ADNS5050_POLL_MS 8

/* Serial timing, from the datasheet AC table (see file header). */
#define ADNS5050_TSWW_US 30  /* between the two write bytes */
#define ADNS5050_TSWR_US 4   /* >=20 us total before a read address; the read
                              * path adds tSRAD on top, so 4 us here suffices */
#define ADNS5050_TSRAD_US 10 /* upper datasheet bound; QMK uses the 4 us min */
#define ADNS5050_TSCLK_NCS_WRITE_US                                                              \
	CONFIG_ADNS5050_TSCLK_NCS_WRITE_US /* 20 us: write CS-setup */
#define ADNS5050_TSRW_US 1    /* read tail: tSRR, also covers tSCLK-NCS read (120ns) */

/* Bench visibility: the only adns5050 log lines are emitted during init,
 * which runs ~100 ms after power-on - long before the CDC-ACM console is
 * enumerated. Zephyr's CDC driver drops output while the device is
 * unconfigured, so a terminal opened after plug-in (the normal bench flow)
 * never sees the boot-time verdict. Retry init a few times (covers a slow
 * sensor power-up) and then re-log the verdict periodically for ~1 minute
 * so the oracle is visible whenever the console is opened. */
#define ADNS5050_ANNOUNCE_INTERVAL_MS 5000
#define ADNS5050_ANNOUNCE_COUNT 12

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
	/* Delayable: init retries are rescheduled from inside the handler. */
	struct k_work_delayable init_work;
	struct k_timer announce_timer;
	struct k_timer health_timer;
	/* Health reads run in WORK context (submitted by health_timer), not
	 * ISR context: they bit-bang the bus, and must not preempt a poll
	 * burst already running on the system workqueue. All bus users
	 * (poll_work, health_work, init_work) share the one workqueue and
	 * are therefore serialized. */
	struct k_work_delayable health_work;
	uint8_t init_attempts;
	uint8_t announce_left;
	uint8_t health_strikes;
	bool ready;
	/* Set once the first init cycle has fully finished (verdict reached):
	 * gates the periodic health check so it never overlaps an init
	 * retry cycle (debounce - see adns5050_health_timer_fn). */
	bool cycle_done;
	uint8_t pid;
	uint8_t rid;
	uint8_t rid2_inv;
	uint8_t pid2;
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

	/* tSWR floor: minimum delay after the last SCLK rising edge of a
	 * byte before the next command byte. QMK waits 4us here. The write
	 * path (write_reg) waits the full tSWW 30us between its two bytes;
	 * the read path adds tSRAD on top, so 4us is enough in both. */
	k_busy_wait(ADNS5050_TSWR_US);
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
	/* tSRAD: 4-10us from the last address bit to the first data bit.
	 * Wait the 10us upper bound (QMK sits on the 4us minimum). */
	k_busy_wait(ADNS5050_TSRAD_US);
	byte = adns5050_serial_read(dev);
	k_busy_wait(ADNS5050_TSRW_US); /* tSRR; covers tSCLK-NCS read (120ns) */
	gpio_pin_set_dt(&config->cs, 0);

	return byte;
}

static void adns5050_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t data)
{
	const struct adns5050_config *config = dev->config;

	adns5050_sync(dev);
	gpio_pin_set_dt(&config->cs, 1);
	adns5050_serial_write(dev, ADNS5050_SPI_ADDRESS_WRITE | reg_addr);
	/* tSWW: 30us from the last SCLK rising edge of the address byte to
	 * the last SCLK rising edge of the data byte. The QMK-derived 4us
	 * serial_write tail alone leaves this gap at ~20us, below spec. */
	k_busy_wait(ADNS5050_TSWW_US - ADNS5050_TSWR_US);
	adns5050_serial_write(dev, data);
	/* tSCLK-NCS (write): 20us from the last SCLK rising edge to NCS
	 * rising edge, "for valid SDIO data transfer". QMK raises NCS after
	 * only the ~4us serial_write tail; without this delay every
	 * register write (CPI, power-down, Chip_Reset) may silently abort. */
	k_busy_wait(ADNS5050_TSCLK_NCS_WRITE_US);
	gpio_pin_set_dt(&config->cs, 0);
}

/* Motion burst: write burst address, read dX then dY, raise CS to end the
 * burst early (the remaining burst registers are not needed). CS raise here
 * follows a READ, so the read-case tSCLK-NCS (120ns) applies - satisfied by
 * the 10us tSRAD wait and the read loop's 1us half-clocks. */
static void adns5050_read_burst(const struct device *dev, int8_t *dx, int8_t *dy)
{
	const struct adns5050_config *config = dev->config;
	uint8_t x, y;

	adns5050_sync(dev);
	gpio_pin_set_dt(&config->cs, 1);
	adns5050_serial_write(dev, ADNS5050_REG_MOTION_BURST);
	/* tSRAD before the first burst data byte (datasheet p15: "the
	 * microcontroller must wait tSRAD and then begin reading data"). */
	k_busy_wait(ADNS5050_TSRAD_US);
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
	uint8_t want = BIT(4) | cpival;
	uint8_t got;

	adns5050_write_reg(dev, ADNS5050_REG_MOUSE_CONTROL2, want);
	/* Read-back verify. The p19 register table lists 0x19 as R/W while
	 * the p27 register detail page says "Access: Write"; if the detail
	 * page is right the read returns 0xFF-class data, which is exactly
	 * what an aborted write would also produce - so a mismatch is
	 * logged loudly but is NOT fatal. Confirms writes actually land
	 * once the tSCLK-NCS/tSWW timing fix is in. */
	got = adns5050_read_reg(dev, ADNS5050_REG_MOUSE_CONTROL2);
	if (got != want) {
		LOG_WRN("Mouse_Control2 read-back mismatch: wrote 0x%02x read 0x%02x "
			"(write may have aborted or register is write-only)",
			want, got);
	} else {
		LOG_INF("Mouse_Control2 read-back OK: 0x%02x", got);
	}
}

/* Re-log the init verdict periodically (see ADNS5050_ANNOUNCE_* above).
 * Runs in timer (ISR) context - deferred-mode logging is ISR-safe. */
static void adns5050_announce_timer_fn(struct k_timer *timer)
{
	struct adns5050_data *data = CONTAINER_OF(timer, struct adns5050_data, announce_timer);
	const struct device *dev = data->dev;
	const struct adns5050_config *config = dev->config;

	if (data->ready) {
		LOG_INF("ADNS-5050 ready, CPI %u", config->cpi);
	} else {
		LOG_ERR("signature check FAILED: PID 0x%02x (want 0x%02x) "
			"REV 0x%02x (want 0x%02x) PID2 0x%02x (want 0x%02x) "
			"InvRev 0x%02x (want 0x%02x) "
			"- trackball polling disabled, check wiring/SDIO",
			data->pid, ADNS5050_PRODUCT_ID, data->rid, ADNS5050_REVISION_ID,
			data->pid2, ADNS5050_PRODUCT_ID2, data->rid2_inv,
			ADNS5050_INV_REV_ID_EXPECTED);
	}

	if (--data->announce_left == 0U) {
		return; /* last announcement: do not re-arm */
	}
	k_timer_start(&data->announce_timer, K_MSEC(ADNS5050_ANNOUNCE_INTERVAL_MS), K_NO_WAIT);
}

static void adns5050_start_announcements(struct adns5050_data *data)
{
	data->announce_left = ADNS5050_ANNOUNCE_COUNT;
	k_timer_start(&data->announce_timer, K_MSEC(ADNS5050_ANNOUNCE_INTERVAL_MS), K_NO_WAIT);
}

/* Non-blocking init: full resync (CS pulse), Chip_Reset 0x5a, wait for
 * wake-up, prime the serial interface, signature reads, then - on success -
 * apply CPI and start the poll timer. Runs on the system workqueue.
 * Retried CONFIG_ADNS5050_INIT_RETRIES times before the signature check is
 * declared fatal; the health check re-runs it if the sensor comes back. */
static void adns5050_init_work_fn(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct adns5050_data *data = CONTAINER_OF(dwork, struct adns5050_data, init_work);
	const struct device *dev = data->dev;
	const struct adns5050_config *config = dev->config;
	int8_t dx, dy;

	gpio_pin_configure_dt(&config->sclk, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&config->sdio, GPIO_OUTPUT_INACTIVE);
	/* logical inactive = deselected = physically high (active-low pin) */
	gpio_pin_configure_dt(&config->cs, GPIO_OUTPUT_INACTIVE);

	adns5050_write_reg(dev, ADNS5050_REG_CHIP_RESET, ADNS5050_CHIP_RESET_MAGIC);
	k_msleep(55); /* datasheet maximum reset-to-ready time */

	adns5050_read_burst(dev, &dx, &dy); /* prime writes, discarded */

	data->pid = adns5050_read_reg(dev, ADNS5050_REG_PRODUCT_ID);
	data->rid = adns5050_read_reg(dev, ADNS5050_REG_REVISION_ID);
	data->pid2 = adns5050_read_reg(dev, ADNS5050_REG_PRODUCT_ID2);
	data->rid2_inv = adns5050_read_reg(dev, ADNS5050_REG_INV_REV_ID);
	if (data->pid != ADNS5050_PRODUCT_ID || data->rid != ADNS5050_REVISION_ID ||
	    data->pid2 != ADNS5050_PRODUCT_ID2 ||
	    data->rid2_inv != ADNS5050_INV_REV_ID_EXPECTED) {
		data->init_attempts++;
		if (data->init_attempts < CONFIG_ADNS5050_INIT_RETRIES) {
			/* A miss can be a transient power-up race: the sensor
			 * powers up with the board when the cable is plugged, so
			 * retry before declaring the bus dead. */
			LOG_WRN("signature mismatch on attempt %u/%u: "
				"PID 0x%02x REV 0x%02x PID2 0x%02x InvRev 0x%02x "
				"- retrying in %d ms",
				data->init_attempts, CONFIG_ADNS5050_INIT_RETRIES,
				data->pid, data->rid, data->pid2, data->rid2_inv,
				CONFIG_ADNS5050_INIT_RETRY_MS);
			k_work_schedule(&data->init_work, K_MSEC(CONFIG_ADNS5050_INIT_RETRY_MS));
			return;
		}
		/* Signature mismatch: sensor absent, miswired, or the bus reads
		 * garbage. Do NOT start the poll timer and do NOT mark ready -
		 * a dead trackball beats a cursor that drifts on corrupt reads. */
		LOG_ERR("signature check FAILED: PID 0x%02x (want 0x%02x) "
			"REV 0x%02x (want 0x%02x) PID2 0x%02x (want 0x%02x) "
			"InvRev 0x%02x (want 0x%02x) "
			"- trackball polling disabled, check wiring/SDIO",
			data->pid, ADNS5050_PRODUCT_ID, data->rid, ADNS5050_REVISION_ID,
			data->pid2, ADNS5050_PRODUCT_ID2, data->rid2_inv,
			ADNS5050_INV_REV_ID_EXPECTED);
		adns5050_start_announcements(data);
		data->cycle_done = true; /* re-arm the health check below */
		return;
	}

	if (data->init_attempts != 0U) {
		LOG_INF("ADNS-5050 ready on attempt %u", data->init_attempts + 1U);
	}
	data->init_attempts = 0;
	data->health_strikes = 0;

	adns5050_set_cpi(dev, config->cpi);

	data->ready = true;
	data->cycle_done = true;
	k_timer_start(&data->poll_timer, K_MSEC(ADNS5050_POLL_MS), K_MSEC(ADNS5050_POLL_MS));

	LOG_INF("ADNS-5050 ready, CPI %u", config->cpi);
	adns5050_start_announcements(data);
}

/* Runtime self-heal. Periodic while ready: read Product_ID and one burst.
 * - PID mismatch: the serial port or the sensor itself went away -> heal.
 * - A saturated delta pair (raw -1 and/or 0 => 0xFF-class bytes, exactly
 *   what a floating/open SDIO line reads) is one strike; strikes must be
 *   CONSECUTIVE - any real (non-saturated) motion resets the count.
 * After CONFIG_ADNS5050_HEALTH_STRIKES consecutive strikes: stop polling,
 * mark not-ready, re-run the init retry loop.
 * Debounce: this work item only acts when cycle_done is set (a full init
 * cycle has finished and its verdict is known) and immediately clears it,
 * so a failed re-init cycle must complete before the next health attempt
 * - no thrash while a retry cycle is still running. Runs on the system
 * workqueue (see struct comment for why this must not be ISR context). */
static void adns5050_health_work_fn(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct adns5050_data *data = CONTAINER_OF(dwork, struct adns5050_data, health_work);
	const struct device *dev = data->dev;
	int8_t dx, dy;
	uint8_t pid;

	if (!data->cycle_done) {
		return; /* init/retry cycle in flight; check back next period */
	}

	if (!data->ready) {
		/* Down (or still down) after a completed cycle: re-run the
		 * FULL retry cycle, one cycle per health period. */
		data->cycle_done = false;
		data->init_attempts = 0;
		LOG_INF("health check: sensor down, re-running init cycle");
		k_work_schedule(&data->init_work, K_NO_WAIT);
		return;
	}

	pid = adns5050_read_reg(dev, ADNS5050_REG_PRODUCT_ID);
	if (pid != ADNS5050_PRODUCT_ID) {
		LOG_ERR("health check: Product_ID read 0x%02x (want 0x%02x) "
			"- serial port lost, re-initializing",
			pid, ADNS5050_PRODUCT_ID);
		data->cycle_done = false;
		goto self_heal;
	}

	adns5050_read_burst(dev, &dx, &dy);
	if ((dx == -1 || dx == 0) && (dy == -1 || dy == 0)) {
		data->health_strikes++;
		LOG_WRN("health check: saturated delta pair dx=%d dy=%d "
			"(strike %u/%u, 0xFF-class = open SDIO signature)",
			dx, dy, data->health_strikes,
			CONFIG_ADNS5050_HEALTH_STRIKES);
		if (data->health_strikes >= CONFIG_ADNS5050_HEALTH_STRIKES) {
			LOG_ERR("health check: %u consecutive saturated deltas "
				"- bus went bad, re-initializing",
				data->health_strikes);
			data->cycle_done = false;
			goto self_heal;
		}
	} else {
		if (data->health_strikes != 0U) {
			LOG_INF("health check: real motion dx=%d dy=%d, "
				"strike counter reset",
				dx, dy);
		}
		data->health_strikes = 0;
	}
	return;

self_heal:
	/* Stop the poll timer first: no burst transactions may run while
	 * the init loop resets the sensor (burst vs reset race). */
	data->ready = false;
	data->health_strikes = 0;
	data->init_attempts = 0;
	k_timer_stop(&data->poll_timer);
	k_work_schedule(&data->init_work, K_NO_WAIT);
}

/* Timer (ISR) context: only submits the health work item - never touches
 * the bus from here (see struct comment). */
static void adns5050_health_timer_fn(struct k_timer *timer)
{
	struct adns5050_data *data = CONTAINER_OF(timer, struct adns5050_data, health_timer);

	k_work_reschedule(&data->health_work, K_NO_WAIT);
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

	/* One burst transaction per poll; no separate pre-read of the Motion
	 * register (0x02) before the burst. The ADNS-5050 datasheet is
	 * ambiguous about read side effects on 0x02: only writing is
	 * documented to clear MOT/Delta_X/Delta_Y, but the MOT bit is defined
	 * as "motion ... since the last time it was read", and bench testing
	 * showed gate-then-burst reporting no motion at all - consistent with
	 * the Motion read consuming/clearing the pending deltas before the
	 * burst could fetch them. QMK's reference driver for this chip never
	 * reads 0x02: it bursts every poll and discards 0/0 deltas, which is
	 * what this driver does too. (Note: unlike the PMW3360, the ADNS-5050
	 * burst carries no Motion byte - it starts at Delta_X - so the report
	 * gate is the burst's own deltas, not a MOT bit.) */
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
	k_timer_init(&data->announce_timer, adns5050_announce_timer_fn, NULL);
	k_timer_init(&data->health_timer, adns5050_health_timer_fn, NULL);
	k_work_init_delayable(&data->health_work, adns5050_health_work_fn);
	k_work_init_delayable(&data->init_work, adns5050_init_work_fn);
	k_work_schedule(&data->init_work, K_NO_WAIT);
	if (CONFIG_ADNS5050_HEALTH_CHECK_MS != 0) {
		k_timer_start(&data->health_timer, K_MSEC(CONFIG_ADNS5050_HEALTH_CHECK_MS),
			      K_MSEC(CONFIG_ADNS5050_HEALTH_CHECK_MS));
	}

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
