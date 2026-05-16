/* uwb_ranging.c — Path D test
 *
 * Hypothesis: TX-then-RX fails on this hardware, but RX-then-TX may work.
 *
 * "Initiator" role: enables RX, waits for a Poll, then sends an Ack.
 *                   After sending Ack, logs status.
 *                   (This is RX-then-TX-then-status-check — the reverse pattern.)
 *
 * "Responder" role: sends a Poll, then enables RX to listen for the Ack.
 *                   (This is the TX-then-RX pattern that we've shown fails.)
 *
 * If the "initiator" successfully receives Poll and sends Ack reliably,
 * RX-then-TX works. We can build a protocol around that.
 *
 * If the "responder" successfully gets the Ack back, TX-then-RX works too
 * (just hadn't worked in our specific previous code paths).
 */

#include "uwb_ranging.h"
#include "dwm3000.h"
#include "deca_device_api.h"
#include "dw3000_deca_regs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include <string.h>

static const char *TAG = "uwb";
static uwb_role_t s_role;

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define FCS_LEN 2

static uint8_t tx_poll_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x21
};
static uint8_t tx_ack_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x10
};

#define RX_BUF_LEN 32
static uint8_t rx_buffer[RX_BUF_LEN];
static uint8_t frame_seq_nb = 0;

float g_uwb_distance_offset_m = 0.0f;

#ifndef SYS_STATUS_ALL_RX_TO
#define SYS_STATUS_ALL_RX_TO   (SYS_STATUS_RXFTO_BIT_MASK | SYS_STATUS_RXPTO_BIT_MASK)
#endif
#ifndef SYS_STATUS_ALL_RX_ERR
#define SYS_STATUS_ALL_RX_ERR  (SYS_STATUS_RXPHE_BIT_MASK | SYS_STATUS_RXFCE_BIT_MASK | \
                                SYS_STATUS_RXFSL_BIT_MASK | SYS_STATUS_RXSTO_BIT_MASK | \
                                SYS_STATUS_ARFE_BIT_MASK)
#endif

static const dwt_config_t s_uwb_config = {
    .chan = 5, .txPreambLength = DWT_PLEN_128, .rxPAC = DWT_PAC8,
    .txCode = 9, .rxCode = 9, .sfdType = 1,
    .dataRate = DWT_BR_6M8, .phrMode = DWT_PHRMODE_STD,
    .phrRate = DWT_PHRRATE_STD, .sfdTO = (129 + 8 - 8),
    .stsMode = DWT_STS_MODE_OFF, .stsLength = DWT_STS_LEN_64,
    .pdoaMode = DWT_PDOA_M0,
};
static const dwt_txconfig_t s_txconfig = {
    .PGdly = 0x34, .power = 0xfdfdfdfd, .PGcount = 0,
};

esp_err_t uwb_init(uwb_role_t role, int mosi, int miso, int sclk, int cs, int rst)
{
    s_role = role;
    ESP_LOGI(TAG, "Initializing UWB (role: %s, Path D test)",
             role == UWB_ROLE_INITIATOR ? "RX-first (init)" : "TX-first (resp)");

    esp_err_t err = dwm3000_init(mosi, miso, sclk, cs, rst);
    if (err != ESP_OK) return err;

    uint32_t dev_id = 0;
    err = dwm3000_read_devid(&dev_id);
    if (err != ESP_OK) return err;
    if ((dev_id >> 16) != 0xDECA) return ESP_FAIL;

    if (dwt_configure((dwt_config_t *)&s_uwb_config) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_configure failed");
        return ESP_FAIL;
    }
    dwt_configuretxrf((dwt_txconfig_t *)&s_txconfig);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    ESP_LOGI(TAG, "UWB init complete");
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* "Initiator" role: do RX first, then TX. The reverse of normal initiator. */
/* ------------------------------------------------------------------------- */

static esp_err_t initiator_one_cycle(void)
{
    /* 1. Enable RX and wait for a Poll. */
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
                         SYS_STATUS_ALL_RX_ERR);
    dwt_setrxtimeout(0);
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_rxenable failed");
        return ESP_FAIL;
    }

    int waited_ms = 0;
    int caught = 0;
    while (waited_ms < 200) {
        uint32_t status = dwt_readsysstatuslo();
        if (status & SYS_STATUS_RXFCG_BIT_MASK) {
            uint8_t rng = 0;
            uint16_t len = dwt_getframelength(&rng);
            if (len > RX_BUF_LEN) len = RX_BUF_LEN;
            dwt_readrxdata(rx_buffer, len, 0);
            dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

            static int rx_count = 0;
            rx_count++;
            if (rx_count % 10 == 1) {
                ESP_LOGI(TAG, "Got Poll #%d (seq=%u)", rx_count, rx_buffer[2]);
            }
            caught = 1;
            break;
        }
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            ESP_LOGW(TAG, "RX error/timeout: status=0x%08lX", (unsigned long)status);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_forcetrxoff();
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        waited_ms += 2;
    }
    if (!caught) {
        static int silent_log = 0;
        if (silent_log++ < 10) {
            ESP_LOGW(TAG, "RX-first init silent. status=0x%08lX",
                    (unsigned long)dwt_readsysstatuslo());
        }
        // dwt_forcetrxoff();
        return ESP_ERR_TIMEOUT;
    }

    /* 2. Now send an Ack back. RX-then-TX. */
    vTaskDelay(pdMS_TO_TICKS(2));  /* give the other end time to enable RX */

    tx_ack_msg[2] = frame_seq_nb;
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_ack_msg), tx_ack_msg, 0);
    dwt_writetxfctrl(sizeof(tx_ack_msg) + FCS_LEN, 0, 1);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "Ack TX failed");
        return ESP_FAIL;
    }

    int waited_us = 0;
    while (waited_us < 5000) {
        if (dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
            frame_seq_nb++;
            static int tx_count = 0;
            tx_count++;
            if (tx_count % 10 == 1) {
                ESP_LOGI(TAG, "Ack TX OK #%d", tx_count);
            }
            return ESP_OK;
        }
        esp_rom_delay_us(50);
        waited_us += 50;
    }
    ESP_LOGW(TAG, "TXFRS timeout, status=0x%08lX", (unsigned long)dwt_readsysstatuslo());
    return ESP_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------------- */
/* "Responder" role: do TX first, then RX. The pattern that's been failing.  */
/* Log status thoroughly so we can see if RX after this TX behaves            */
/* differently from RX after the initiator's TX.                              */
/* ------------------------------------------------------------------------- */

static esp_err_t responder_one_cycle(void)
{
    /* 1. Send a Poll. */
    tx_poll_msg[2] = frame_seq_nb;
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "Poll TX failed");
        return ESP_FAIL;
    }
    int waited_us = 0;
    while (waited_us < 5000) {
        if (dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
            frame_seq_nb++;
            break;
        }
        esp_rom_delay_us(50);
        waited_us += 50;
    }
    if (waited_us >= 5000) {
        ESP_LOGW(TAG, "Poll TXFRS timeout");
        return ESP_ERR_TIMEOUT;
    }

    static int tx_count = 0;
    tx_count++;
    if (tx_count % 10 == 1) {
        ESP_LOGI(TAG, "Poll TX OK #%d (seq=%u)", tx_count, frame_seq_nb - 1);
    }

    /* 2. Enable RX. Wait for Ack. Log status carefully. */
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
                         SYS_STATUS_ALL_RX_ERR);
    dwt_setrxtimeout(0);
    int8_t rx_ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);

    int waited_ms = 0;
    int caught = 0;
    while (waited_ms < 30) {
        uint32_t status = dwt_readsysstatuslo();
        if (status & SYS_STATUS_RXFCG_BIT_MASK) {
            static int ack_count = 0;
            ack_count++;
            if (ack_count % 10 == 1) {
                ESP_LOGI(TAG, "Ack received! #%d", ack_count);
            }
            dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);
            caught = 1;
            break;
        }
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            static int rx_err_count = 0;
            if (rx_err_count++ < 5) {
                ESP_LOGW(TAG, "Ack RX err: status=0x%08lX", (unsigned long)status);
            }
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        waited_ms += 2;
    }

    if (!caught) {
        static int log_count = 0;
        if (log_count++ < 5) {
            ESP_LOGW(TAG, "No Ack: rxenable=%d status=0x%08lX",
                     rx_ret, (unsigned long)dwt_readsysstatuslo());
        }
    }
    dwt_forcetrxoff();
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */

esp_err_t uwb_perform_ranging(uwb_range_result_t *result)
{
    if (result) {
        result->valid = false;
        result->distance_m = 0.0f;
        result->timestamp_us = 0;
    }

    if (s_role == UWB_ROLE_INITIATOR) {
        esp_err_t err = initiator_one_cycle();
        if (err == ESP_ERR_TIMEOUT) err = ESP_OK;
        return err;
    }
    esp_err_t err = responder_one_cycle();
    if (err == ESP_ERR_TIMEOUT) err = ESP_OK;
    return err;
}

esp_err_t uwb_send_poll_blocking(void) { return ESP_OK; }
esp_err_t uwb_receive_frame_blocking(uint8_t *out_buf, uint16_t buf_size,
                                     uint16_t *out_len, int timeout_ms)
{
    (void)out_buf; (void)buf_size; (void)out_len; (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
}