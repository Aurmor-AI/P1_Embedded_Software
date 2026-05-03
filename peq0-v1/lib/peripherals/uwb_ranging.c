#include "uwb_ranging.h"
#include "dwm3000.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "uwb";
static uwb_role_t s_role;

esp_err_t uwb_init(uwb_role_t role)
{
    s_role = role;
    ESP_LOGI(TAG, "Initializing UWB (role: %s)",
             role == UWB_ROLE_INITIATOR ? "initiator" : "responder");

    /* Phase 1: short-form DEV_ID. */
    uint32_t dev_id = 0;
    esp_err_t err = dwm3000_read_devid(&dev_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "DEV_ID (short-form) = 0x%08lX", (unsigned long)dev_id);
    if ((dev_id >> 16) != 0xDECA) {
        ESP_LOGE(TAG, "Bad DEV_ID");
        return ESP_FAIL;
    }

    /* Phase 2a: generic-path DEV_ID for header-layout sanity. */
    uint32_t dev_id_generic = 0;
    err = dwm3000_read32(0x00, 0x00, &dev_id_generic);
    if (err != ESP_OK || dev_id_generic != dev_id) {
        ESP_LOGE(TAG, "Generic DEV_ID mismatch (0x%08lX vs 0x%08lX)",
                 (unsigned long)dev_id_generic, (unsigned long)dev_id);
        return ESP_FAIL;
    }

    /* Phase 2b: hard-reset path → IDLE_RC. */
    err = dwm3000_wait_idle_rc(5);
    if (err != ESP_OK) return err;

    /* Phase 2c: soft-reset path → IDLE_RC. */
    ESP_LOGI(TAG, "Issuing soft reset...");
    err = dwm3000_soft_reset();
    if (err != ESP_OK) return err;
    err = dwm3000_wait_idle_rc(5);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Phase 2 complete: IDLE_RC after both hard and soft reset");

    /* Phase 3a: OTP read + factory trim load. */
    uint64_t eui = 0;
    err = dwm3000_otp_read_eui64(&eui);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "EUI64 = 0x%016llX", (unsigned long long)eui);
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "EUI64 unprogrammed in OTP (acceptable for bring-up)");
    } else {
        ESP_LOGE(TAG, "OTP read failed: %s", esp_err_to_name(err));
        return err;
    }

    // dwm3000_otp_diagnose();
    dwm3000_otp_dump_calibration();

    dwm3000_otp_dump_calibration();

    err = dwm3000_load_factory_trims();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Trim load failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Phase 3a complete: factory trims applied");

    return ESP_OK;
}

esp_err_t uwb_perform_ranging(uwb_range_result_t *result)
{
    if (result) {
        result->valid = false;
        result->distance_m = 0.0f;
        result->timestamp_us = 0;
    }
    return ESP_ERR_NOT_SUPPORTED;
}