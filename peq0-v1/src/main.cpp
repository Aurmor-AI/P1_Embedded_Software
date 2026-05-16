#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "dwm3000.h"
#include "lsm6dsv.h"
#include "uwb_ranging.h"
#include "freertos/semphr.h"
#include <math.h>
#include "port.h"
extern "C" {
    #include "deca_device_api.h"
}

static const char *TAG = "main";

#define PIN_MOSI 18
#define PIN_MISO 20
#define PIN_SCLK 19
#define PIN_CS   16
#define PIN_RST  17
#define PIN_SDA  22
#define PIN_SCL  23

// Set this per-board: one as INITIATOR, the other as RESPONDER.
// You can flash the same firmware to both and just change this define.
#define MY_UWB_ROLE  UWB_ROLE_RESPONDER// UWB_ROLE_RESPONDER or UWB_ROLE_INITIATOR

// Task rates
#define IMU_PRINT_HZ      10
#define UWB_RANGE_HZ      10
#define IMU_PERIOD_MS     (1000 / IMU_PRINT_HZ)
#define UWB_PERIOD_MS     (1000 / UWB_RANGE_HZ)

#define SAMPLE_PERIOD_MS  20  // 50 Hz print rate

// Shared state — protected by atomic-ish access (single writer, single reader is OK
// for floats on 32-bit MCUs without explicit synchronization)
static uwb_range_result_t s_last_range = {0.0f, 0, false};
static SemaphoreHandle_t s_range_mutex = NULL;

// ===========================================================================
// IMU task — reads sample, prints combined IMU + ranging data
// ===========================================================================
static void imu_task(void *arg)
{
    ESP_LOGI(TAG, "IMU task started");
    // ... [printf headers stay the same] ...

    int64_t next_print = esp_timer_get_time();
    int64_t next_sample = esp_timer_get_time();

    // Peak tracker for high-g
    float peak_h_mag = 0.0f;
    lsm6_sample_t peak_sample = {};

    while (true) {
        int64_t now = esp_timer_get_time();

        // Sample at high rate
        if (now >= next_sample) {
            lsm6_sample_t s;
            if (lsm6_read_sample(&s) == ESP_OK) {
                // Track peak high-g magnitude
                float h_mag = sqrtf(s.hx_g * s.hx_g +
                                    s.hy_g * s.hy_g +
                                    s.hz_g * s.hz_g);
                if (h_mag > peak_h_mag) {
                    peak_h_mag = h_mag;
                    peak_sample = s;
                }
            }
            next_sample += 5000;  // 200 Hz sampling
        }

        // Print at slower rate
        if (now >= next_print) {
            int64_t now_ms = now / 1000;

            uwb_range_result_t r;
            if (xSemaphoreTake(s_range_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                r = s_last_range;
                xSemaphoreGive(s_range_mutex);
            } else {
                r.valid = false;
            }
            char range_str[16];
            if (r.valid) snprintf(range_str, sizeof(range_str), "%7.3f", r.distance_m);
            else snprintf(range_str, sizeof(range_str), "   ---");

            // Print using the peak high-g sample from this interval
            printf("%-10lld | %+8.3f %+8.3f %+8.3f | "
                   "%+8.2f %+8.2f %+8.2f | "
                   "%+9.2f %+9.2f %+9.2f | %5.1f | peak_h=%5.2fg | %s\n",
                   now_ms,
                   peak_sample.ax_g, peak_sample.ay_g, peak_sample.az_g,
                   peak_sample.hx_g, peak_sample.hy_g, peak_sample.hz_g,
                   peak_sample.gx_dps, peak_sample.gy_dps, peak_sample.gz_dps,
                   peak_sample.temp_c,
                   peak_h_mag,
                   range_str);

            // Reset peak for next interval
            peak_h_mag = 0.0f;

            next_print += 100000;  // 100 ms = 10 Hz print rate
        }

        // Sleep until next event (sample or print, whichever comes first)
        int64_t sleep_us = next_sample < next_print ? next_sample - now
                                                    : next_print - now;
        if (sleep_us > 0) {
            vTaskDelay(pdMS_TO_TICKS(sleep_us / 1000 + 1));
        }
    }
}

// ===========================================================================
// UWB task — performs ranging, updates shared state
// ===========================================================================
static void uwb_task(void *arg)
{
    ESP_LOGI(TAG, "UWB task started (role: %s)",
             MY_UWB_ROLE == UWB_ROLE_INITIATOR ? "initiator" : "responder");

    int64_t next = esp_timer_get_time();
    int fail_count = 0;

    while (true) {
        uwb_range_result_t r;
        esp_err_t err = uwb_perform_ranging(&r);

        if (xSemaphoreTake(s_range_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (err == ESP_OK && r.valid) {
            s_last_range = r;
            fail_count = 0;
        } else {
            if (++fail_count == 1) {
                ESP_LOGW(TAG, "Ranging not yet implemented (stub returns ERR)");
            }
            if (fail_count > 5) {
                s_last_range.valid = false;
            }
        }
        xSemaphoreGive(s_range_mutex);
        }

        next += UWB_PERIOD_MS * 1000;
        int64_t now = esp_timer_get_time();
        int64_t sleep_us = next - now;
        if (sleep_us > 0) {
            vTaskDelay(pdMS_TO_TICKS(sleep_us / 1000));
        } else {
            next = now;
        }
    }
}

// ===========================================================================
// Boot-time peripheral reset
// ===========================================================================
static void boot_reset_peripherals(void)
{
    ESP_LOGI(TAG, "Resetting peripherals...");

    // Force IMU into I2C mode (in case it's stuck in I3C)
    lsm6_force_i2c_mode(PIN_SDA, PIN_SCL);

    // Hard-reset DWM3000
    gpio_set_direction((gpio_num_t)PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_direction((gpio_num_t)PIN_RST, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Software-reset IMU
    lsm6_init(PIN_SDA, PIN_SCL);
    lsm6_software_reset();
    lsm6_deinit();
}

// ===========================================================================
// app_main — initialize everything, hand off to tasks
// ===========================================================================
extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(4000));
    ESP_LOGI(TAG, "=== Boot ===");

    boot_reset_peripherals();


    if (uwb_init(MY_UWB_ROLE, PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, PIN_RST) != ESP_OK) {
        ESP_LOGE(TAG, "UWB init failed");
        return;
    }

    // --- IMU ---
    if (lsm6_init(PIN_SDA, PIN_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed");
        return;
    }

    uint8_t imu_addr, whoami;
    if (lsm6_read_who_am_i(&imu_addr, &whoami) != ESP_OK) {
        ESP_LOGE(TAG, "IMU not found");
        return;
    }
    ESP_LOGI(TAG, "IMU at 0x%02X, WHO_AM_I=0x%02X", imu_addr, whoami);

    if (lsm6_configure_default() != ESP_OK) {
        ESP_LOGE(TAG, "IMU configuration failed");
        return;
    }

    // --- Create mutex (MUST be before xTaskCreate) ---
    s_range_mutex = xSemaphoreCreateMutex();
    if (s_range_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    ESP_LOGI(TAG, "Mutex created");

    // --- Launch tasks ---
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    xTaskCreate(uwb_task, "uwb_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Tasks running. app_main exiting.");
}