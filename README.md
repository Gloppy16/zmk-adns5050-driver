# zmk-adns5050-driver

ZMK input driver for the Pixart **ADNS-5050** optical mouse sensor over a
**bit-banged 3-wire serial bus** (SCLK / SDIO / CS), with polled motion
reporting and configurable scroll layers.

This is a continuation of [msfmfjt/zmk-adns5050-driver](https://github.com/msfmfjt/zmk-adns5050-driver),
rewritten because the original driver targeted hardware SPI plus a MOTION
interrupt pin. The ADNS-5050's half-duplex single-data-line protocol cannot be
produced by hardware SPI controllers such as the nRF52's SPIM (MOSI cannot be
tri-stated mid-transfer), so the bus is bit-banged on three plain GPIOs —
the same approach as QMK's `drivers/sensors/adns5050.c`, from which the
transport is ported (GPL-2.0-or-later, © Ploopy Corporation, Drashna Jael're,
Sunjun Kim, Hiroyuki Okada).

Developed for and tested on the [cocot46plus](https://github.com/aki27kbd/cocot46plus)
trackball (nice_nano_v2).

## Properties

| Property       | Type         | Description                                                    |
| -------------- | ------------ | -------------------------------------------------------------- |
| `sclk-gpios`   | phandle-array| Serial clock (idles low)                                       |
| `sdio-gpios`   | phandle-array| Bidirectional data line                                        |
| `cs-gpios`     | phandle-array| Chip select (active low)                                       |
| `vcc-gpios`    | phandle-array| Optional supply-rail enable; driven active and re-asserted at every init attempt. On switched-rail boards (nice!nano) set it to the SAME pin as the board `ext_power` node (`P0.13` on nice_nano_v2) |
| `cpi`          | int          | 125..1375 in 125-CPI steps (default 500)                       |
| `invert-x`     | flag         | Invert X direction                                             |
| `invert-y`     | flag         | Invert Y direction                                             |
| `scroll-layers`| array        | Layer indexes on which motion is reported as wheel scrolling   |

## Usage

```dts
/ {
    trackball: adns5050@0 {
        compatible = "pixart,adns5050";
        sclk-gpios = <&pro_micro 16 GPIO_ACTIVE_HIGH>;   /* B2 */
        sdio-gpios = <&pro_micro 8  GPIO_ACTIVE_HIGH>;   /* B4 */
        cs-gpios   = <&pro_micro 9  GPIO_ACTIVE_LOW>;    /* B5 */
        /* nice!nano only: hold the switched Pro Micro VCC rail ON (same
           pin as the board EXT_POWER node's control-gpios) */
        vcc-gpios  = <&gpio0 13 GPIO_ACTIVE_HIGH>;
        cpi = <500>;
        invert-x;
        invert-y;
        scroll-layers = <1 2>;
    };

    trackball_listener {
        compatible = "zmk,input-listener";
        device = <&trackball>;
    };
};
```

```conf
CONFIG_GPIO=y
CONFIG_INPUT=y
CONFIG_ADNS5050=y
CONFIG_ZMK_POINTING=y
```

Add the module to `west.yml`:

```yaml
- name: zmk-adns5050-driver
  url: https://github.com/Gloppy16/zmk-adns5050-driver
  revision: main
```

## Design notes

- Motion is polled every 8 ms (`k_timer` → `k_work` on the system workqueue);
  no MOTION pin is used.
- Scroll mode reports `INPUT_REL_WHEEL` / `INPUT_REL_HWHEEL` when the highest
  active keymap layer matches one of `scroll-layers`.
- The sensor is reset and primed asynchronously at boot (system workqueue),
  then the poll timer starts; a signature mismatch disables polling and the
  trackball stays silent (check wiring).
- Each poll issues ONE Motion_Burst transaction and reports the deltas if
  non-zero. The Motion register (0x02) is never read: its read side effects
  are ambiguous in the datasheet and a pre-burst 0x02 gate rendered the
  trackball dead on the bench (ee9651f regression, fixed here).

## License

GPL-2.0-or-later (inherited from the QMK transport this driver ports).
