/*
 * ADNS-5050 register map and protocol constants.
 *
 * Ported from QMK's drivers/sensors/adns5050.{c,h}
 * Copyright 2021 Colin Lam (Ploopy Corporation)
 * Copyright 2020 Christopher Courtney (Drashna Jael're)
 * Copyright 2019 Sunjun Kim
 * Copyright 2019 Hiroyuki Okada
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ADNS-5050 datasheet: register addresses, serial timings (tSWW, tSRAD,
 * tSCLK-NCS, tSRR, tSCLN/tSCLP) and the 3-wire "clocked serial" protocol.
 * Full datasheet: AV02-1045EN (April 25, 2012), 29 pages - the commonly
 * mirrored 8-page copy is the ABBREVIATED version and lacks the AC table.
 */
#ifndef ADNS5050_H
#define ADNS5050_H

#include <stdint.h>

/* Registers */
#define ADNS5050_REG_PRODUCT_ID     0x00
#define ADNS5050_REG_REVISION_ID    0x01
#define ADNS5050_REG_MOTION         0x02
#define ADNS5050_REG_DELTA_X        0x03
#define ADNS5050_REG_DELTA_Y        0x04
#define ADNS5050_REG_SQUAL          0x05
#define ADNS5050_REG_SHUTTER_UPPER  0x06
#define ADNS5050_REG_SHUTTER_LOWER  0x07
#define ADNS5050_REG_MAXIMUM_PIXEL  0x08
#define ADNS5050_REG_PIXEL_SUM      0x09
#define ADNS5050_REG_MINIMUM_PIXEL  0x0a
#define ADNS5050_REG_PIXEL_GRAB     0x0b
#define ADNS5050_REG_MOUSE_CONTROL  0x0d
#define ADNS5050_REG_MOUSE_CONTROL2 0x19
#define ADNS5050_REG_LED_DC_MODE    0x22
#define ADNS5050_REG_CHIP_RESET     0x3a
#define ADNS5050_REG_PRODUCT_ID2    0x3e
#define ADNS5050_REG_INV_REV_ID     0x3f
#define ADNS5050_REG_MOTION_BURST   0x63

/* Serial protocol */
#define ADNS5050_SPI_ADDRESS_WRITE  0x80u

/* Expected signature */
#define ADNS5050_PRODUCT_ID          0x12
#define ADNS5050_REVISION_ID         0x01
#define ADNS5050_PRODUCT_ID2         0x26
#define ADNS5050_INV_REV_ID_EXPECTED 0xfe /* p19/p28: Inv_Rev_ID = ~Revision_ID */

/* Chip reset magic */
#define ADNS5050_CHIP_RESET_MAGIC   0x5a

#endif /* ADNS5050_H */
