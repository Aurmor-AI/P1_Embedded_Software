#pragma once
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DW3000_DEV_ID_EXPECTED_HI  0xDECA
#define DW3000_DEV_ID_EXPECTED     0xDECA0302u

/* Initialize SPI bus + hard reset + Qorvo driver probe + dwt_initialise. */
esp_err_t dwm3000_init(int mosi, int miso, int sclk, int cs, int rst);
esp_err_t dwm3000_deinit(void);

/* Phase-1 sanity reads. dwm3000_read_devid uses our own minimal SPI path
 * to confirm wiring before handing control to the Qorvo driver. */
esp_err_t dwm3000_read_devid(uint32_t *out_devid);
esp_err_t dwm3000_wait_ready(int timeout_ms);

/* Hard-reset (RST pin pulse). Wraps reset_DWIC for backward compatibility
 * with code that already calls these names. */
void dwm3000_hard_reset(void);
void dwm3000_reset_pin_only(int rst);

#ifdef __cplusplus
}
#endif