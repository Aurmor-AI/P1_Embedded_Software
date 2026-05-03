#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DW3000_DEV_ID_EXPECTED_HI  0xDECA
#define DW3000_DEV_ID_EXPECTED     0xDECA0302u

esp_err_t dwm3000_init(int mosi, int miso, int sclk, int cs, int rst);
esp_err_t dwm3000_deinit(void);
esp_err_t dwm3000_read_devid(uint32_t *out_devid);
esp_err_t dwm3000_wait_ready(int timeout_ms);
void      dwm3000_hard_reset(void);
void      dwm3000_reset_pin_only(int rst);

/* ---- Phase 2: arbitrary register access + soft reset ------------------ */

esp_err_t dwm3000_reg_read(uint8_t base_id, uint8_t sub_addr,
                           void *dst, size_t len);
esp_err_t dwm3000_reg_write(uint8_t base_id, uint8_t sub_addr,
                            const void *src, size_t len);

esp_err_t dwm3000_read32(uint8_t base_id, uint8_t sub_addr, uint32_t *out);
esp_err_t dwm3000_write32(uint8_t base_id, uint8_t sub_addr, uint32_t val);
esp_err_t dwm3000_read8(uint8_t base_id, uint8_t sub_addr, uint8_t *out);
esp_err_t dwm3000_write8(uint8_t base_id, uint8_t sub_addr, uint8_t val);

esp_err_t dwm3000_soft_reset(void);
esp_err_t dwm3000_wait_idle_rc(int timeout_ms);

/* ---- Phase 3a: OTP access + factory trim load ------------------------- */

esp_err_t dwm3000_otp_read(uint16_t otp_addr, uint32_t *out);
esp_err_t dwm3000_otp_read_eui64(uint64_t *out_eui);
esp_err_t dwm3000_load_factory_trims(void);
void      dwm3000_otp_dump_calibration(void);

#ifdef __cplusplus
}
#endif