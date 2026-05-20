#include "dwm3000.h"
#include "port.h"
#include "deca_probe_interface.h"
#include "deca_device_api.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "dwm3000";

/* Provided by port.c */
extern spi_device_handle_t g_dw_spi;

/* The pin-config args to dwm3000_init are kept for backward compatibility
 * with main.c, but the actual pins live in port.h. We log a warning if
 * they don't match. */
static void check_pins(int mosi, int miso, int sclk, int cs, int rst)
{
    extern int DW3000_MOSI_PIN_CHK __attribute__((weak));
    if (mosi != DW3000_MOSI_PIN || miso != DW3000_MISO_PIN ||
        sclk != DW3000_SCLK_PIN || cs   != DW3000_CS_PIN   ||
        rst  != DW3000_RST_PIN) {
        ESP_LOGW(TAG, "pin mismatch with port.h — using port.h values");
    }
}

void dwm3000_hard_reset(void)
{
    reset_DWIC();
}

void dwm3000_reset_pin_only(int rst)
{
    (void)rst;
    reset_DWIC();
}


esp_err_t dwm3000_init(int mosi, int miso, int sclk, int cs, int rst)
{
    check_pins(mosi, miso, sclk, cs, rst);

    /* Idempotent setup */
    dwm3000_deinit();

    esp_err_t err = port_init_dw3000();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "port_init_dw3000 failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(200));  // generous boot delay
    gpio_set_pull_mode((gpio_num_t)DW3000_RST_PIN, GPIO_PULLUP_ONLY);  // belt + suspenders
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t check_devid = 0;
    if (dwm3000_read_devid(&check_devid) == ESP_OK) {
        ESP_LOGI(TAG, "DEV_ID after extended wait: 0x%08lX", (unsigned long)check_devid);
    } else {
        ESP_LOGE(TAG, "Phase-1 DEV_ID read failed");
    }
    /* Hard reset before any driver activity. */
    reset_DWIC();
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Hand control to Qorvo's driver: probe → wait IDLE_RC → initialise. */
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_probe failed");
        return ESP_FAIL;
    }

    /* Wait for IDLE_RC. dwt_checkidlerc returns non-zero when ready. */
    int waited_ms = 0;
    while (!dwt_checkidlerc()) {
        vTaskDelay(pdMS_TO_TICKS(2));
        waited_ms += 2;
        if (waited_ms > 200) {
            ESP_LOGE(TAG, "IDLE_RC timeout");
            return ESP_ERR_TIMEOUT;
        }
    }
    ESP_LOGI(TAG, "IDLE_RC reached after %d ms", waited_ms);

    /* Driver init: reads OTP, kicks LDO/BIAS, applies XTAL_TRIM, etc. */
    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_initialise failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "dwt_initialise OK");

    /* Driver's setfastrate is supposed to fire automatically but doesn't on
    * this version. Call it manually before any time-critical operation. */
    port_set_dw_ic_spi_fastrate();
    ESP_LOGI(TAG, "switched to fast SPI rate");
    
    return ESP_OK;
}

esp_err_t dwm3000_deinit(void)
{
    port_deinit_dw3000();
    return ESP_OK;
}

esp_err_t dwm3000_read_devid(uint32_t *out_devid)
{
    if (out_devid == NULL) return ESP_ERR_INVALID_ARG;
    if (g_dw_spi == NULL)  return ESP_ERR_INVALID_STATE;

    uint8_t tx[5] = {0}, rx[5] = {0};
    spi_transaction_t t = { .length = 5 * 8, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t err = spi_device_polling_transmit(g_dw_spi, &t);
    if (err != ESP_OK) return err;

    *out_devid = ((uint32_t)rx[1])
               | ((uint32_t)rx[2] << 8)
               | ((uint32_t)rx[3] << 16)
               | ((uint32_t)rx[4] << 24);
    return ESP_OK;
}

esp_err_t dwm3000_wait_ready(int timeout_ms)
{
    int elapsed = 0;
    uint32_t devid = 0;
    while (elapsed < timeout_ms) {
        if (dwm3000_read_devid(&devid) == ESP_OK &&
            (devid >> 16) == DW3000_DEV_ID_EXPECTED_HI) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        elapsed += 5;
    }
    ESP_LOGE(TAG, "Not ready after %d ms (last DEV_ID=0x%08lX)",
             timeout_ms, (unsigned long)devid);
    return ESP_ERR_TIMEOUT;
}