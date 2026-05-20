/* uwb_ranging.c — Double-Sided Two-Way Ranging (DS-TWR)
 *
 * Frame sequence:
 *
 *   Initiator                            Responder
 *      |                                     |
 *      |--- Poll -------------------------->|   reads poll_rx_ts
 *      |<-- Response -----------------------|   (poll_rx_ts, resp_tx_ts)
 *      |   reads resp_rx_ts                  |   scheduled via delayed TX
 *      |--- Final ------------------------->|   reads final_rx_ts
 *      |   includes [poll_tx, resp_rx,       |   computes DS-TWR ToF
 *      |             final_tx]               |
 *      |<-- Report -------------------------|   (distance_mm)
 *      |   stores distance                   |
 *
 * Why DS-TWR over SS-TWR:
 *   - SS-TWR computes ToF = (Ra - Db) / 2 where Ra uses initiator clock
 *     and Db uses responder clock. Clock drift between the two adds a
 *     systematic error proportional to the reply delay (~18 mm/ms at
 *     20 ppm clock skew). With our 3 ms reply delays, that's tens of cm.
 *
 *   - DS-TWR uses (Ra*Rb - Da*Db) / (Ra+Rb+Da+Db). Drift terms cancel
 *     to first order, leaving residual error in the cm range or better.
 *
 * Formula derivation (Qorvo "asymmetric DS-TWR"):
 *   Ra = resp_rx_ts - poll_tx_ts         (initiator round-trip 1)
 *   Rb = final_rx_ts - resp_tx_ts        (responder round-trip)
 *   Da = final_tx_ts - resp_rx_ts        (initiator reply delay)
 *   Db = resp_tx_ts - poll_rx_ts         (responder reply delay)
 *   ToF = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
 *
 *   Distance = ToF * speed_of_light * tick_period.
 *
 * Two scheduled-TX gotchas (both apply on both ends):
 *   - dwt_setdelayedtrxtime takes timestamp >> 8 (top 32 bits of a
 *     40-bit value).
 *   - The hardware ALSO zeros bit 0 of that value (= bit 8 of timestamp).
 *     So the embedded TX timestamp must mask with & 0xFFFFFFFEUL before
 *     shifting back. Skipping this gives a 256-tick alternating error
 *     (~0.6 m bimodal distance, halved by ToF math).
 */

#include "uwb_ranging.h"
#include "dwm3000.h"
#include "deca_device_api.h"
#include "dw3000_deca_regs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include <string.h>

static const char *TAG = "uwb";

/* --------------------------------------------------------------------- */
/* Tunables                                                              */
/* --------------------------------------------------------------------- */

/* Antenna delay (in DW3000 ticks ~15.65 ps each). 16385 is Qorvo's stock
 * default for channel 5 — uncalibrated. Run a known-distance calibration
 * once DS-TWR is stable. Off-by-1 here = off by ~0.5 cm in distance. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* UWB µs (uus) = 65536 DTU = ~1.0256 µs wall-clock.
 *
 * NOTE: the suffix matters! Without it, expressions like
 *   POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME
 * where the delay is >= 32768 overflow signed int (32-bit) and produce
 * undefined behavior — typically wrapping to a negative number which,
 * when added to poll_rx_ts as uint64, makes the scheduled time go
 * backwards. Use the UL suffix to force unsigned-long evaluation. */
#define UUS_TO_DWT_TIME 65536UL

/* Initiator: Response RX window after Poll TX. */
#define POLL_TX_TO_RESP_RX_DLY_UUS  4000
#define RESP_RX_TIMEOUT_UUS         8000

/* Responder: scheduled Response TX delay after Poll RX. ~500 us of host
 * overhead measured + 1 ms detection jitter + margin = 6 ms. */
#define POLL_RX_TO_RESP_TX_DLY_UUS  6000

/* Initiator: scheduled Final TX delay after Response RX. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 6000

/* Responder: Final RX window after Response TX. Open immediately, keep
 * open for ~12 ms to absorb initiator-side scheduling jitter. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS  0
#define FINAL_RX_TIMEOUT_UUS         12000

/* Initiator: Report RX window after Final TX. */
#define FINAL_TX_TO_REPORT_RX_DLY_UUS 0
#define REPORT_RX_TIMEOUT_UUS         8000

/* Speed of light in air ≈ 299,702,547 m/s. */
#define SPEED_OF_LIGHT_M_PER_S      299702547.0

/* DW3000 timestamp tick period: 1 / (499.2e6 * 128) ≈ 15.65 ps. */
#define DWT_TIME_UNITS              (1.0 / 499200000.0 / 128.0)

float g_uwb_distance_offset_m = 0.0f;

/* --------------------------------------------------------------------- */
/* Frame layout                                                          */
/* --------------------------------------------------------------------- */

/* IEEE 802.15.4 short-address header (10 bytes):
 *   [0..1] frame control (0x88 0x41)
 *   [2]    sequence number
 *   [3..4] PAN ID (0xDECA)
 *   [5..6] dest short address
 *   [7..8] src  short address
 *   [9]    function code
 * Then function-specific payload, then 2-byte FCS appended by hardware. */

#define FRAME_FC_0      0x41
#define FRAME_FC_1      0x88
#define PAN_ID_LO       0xCA
#define PAN_ID_HI       0xDE

#define ADDR_INIT_LO    'V'
#define ADDR_INIT_HI    'E'

/* All responders share the prefix 'W'; the suffix ('A'..'I') is set at
 * uwb_init time via responder_addr_suffix. The initiator picks a specific
 * responder per cycle via the peer_addr_suffix argument to
 * uwb_perform_ranging. */
#define ADDR_RESP_PREFIX 'W'
#define DEFAULT_RESP_SUFFIX 'A'

#define FN_POLL         0x21
#define FN_RESPONSE     0x10
#define FN_FINAL        0x29
#define FN_REPORT       0x2A

#define FCS_LEN         2

#define POLL_FRAME_LEN  10
/* Response: header + poll_rx_ts(5) + resp_tx_ts(5) */
#define RESP_FRAME_LEN  20
#define RESP_POLL_RX_TS_IDX 10
#define RESP_RESP_TX_TS_IDX 15
/* Final: header + poll_tx_ts(5) + resp_rx_ts(5) + final_tx_ts(5) */
#define FINAL_FRAME_LEN 25
#define FINAL_POLL_TX_TS_IDX  10
#define FINAL_RESP_RX_TS_IDX  15
#define FINAL_FINAL_TX_TS_IDX 20
/* Report: header + distance_mm(4) + imu_sample (sizeof(lsm6_sample_t)).
 *
 * Floats serialize via memcpy — both boards are little-endian RISC-V
 * with the same IEEE-754 representation, so byte-equality round-trips.
 * If you ever support a mixed-architecture peer, switch to explicit
 * float-to-int16 packing or a wire-level fixed-point encoding. */
#define REPORT_FRAME_LEN   (14 + (int)sizeof(lsm6_sample_t))
#define REPORT_DIST_IDX    10
#define REPORT_IMU_IDX     14

/* --------------------------------------------------------------------- */
/* State                                                                  */
/* --------------------------------------------------------------------- */

static uwb_role_t s_role;
static uint8_t    s_seq = 0;
static uint8_t    s_my_addr_lo;
static uint8_t    s_my_addr_hi;
static uint8_t    s_peer_addr_lo;
static uint8_t    s_peer_addr_hi;

/* Local IMU snapshot, published by uwb_publish_local_imu and read by the
 * responder when assembling the Report frame. Protected by spinlock so
 * the imu timer callback (publisher) and uwb task (consumer) can run on
 * different cores without tearing. */
static portMUX_TYPE  s_imu_lock = portMUX_INITIALIZER_UNLOCKED;
static lsm6_sample_t s_local_imu;
static bool          s_local_imu_valid = false;

/* Report frame size is 14 + sizeof(lsm6_sample_t) = 54. Plus FCS = 56.
 * Bump RX buffer to comfortably hold it. */
#define RX_BUF_LEN 64
static uint8_t rx_buf[RX_BUF_LEN];

/* --------------------------------------------------------------------- */
/* Config                                                                 */
/* --------------------------------------------------------------------- */

static const dwt_config_t s_uwb_config = {
    .chan          = 5,
    .txPreambLength = DWT_PLEN_64,
    .rxPAC         = DWT_PAC8,
    .txCode        = 9,
    .rxCode        = 9,
    .sfdType       = 1,
    .dataRate      = DWT_BR_6M8,
    .phrMode       = DWT_PHRMODE_STD,
    .phrRate       = DWT_PHRRATE_STD,
    .sfdTO         = (64 + 1 + 8 - 8),
    .stsMode       = DWT_STS_MODE_OFF,
    .stsLength     = DWT_STS_LEN_64,
    .pdoaMode      = DWT_PDOA_M0,
};
static const dwt_txconfig_t s_txconfig = {
    .PGdly = 0x34, .power = 0xfdfdfdfd, .PGcount = 0,
};

#ifndef SYS_STATUS_ALL_RX_TO
#define SYS_STATUS_ALL_RX_TO   (SYS_STATUS_RXFTO_BIT_MASK | SYS_STATUS_RXPTO_BIT_MASK)
#endif
#ifndef SYS_STATUS_ALL_RX_ERR
#define SYS_STATUS_ALL_RX_ERR  (SYS_STATUS_RXPHE_BIT_MASK | SYS_STATUS_RXFCE_BIT_MASK | \
                                SYS_STATUS_RXFSL_BIT_MASK | SYS_STATUS_RXSTO_BIT_MASK | \
                                SYS_STATUS_ARFE_BIT_MASK)
#endif
#define SYS_STATUS_RX_ANY (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)

/* --------------------------------------------------------------------- */
/* Frame helpers                                                          */
/* --------------------------------------------------------------------- */

static void fill_header(uint8_t *frame, uint8_t fn,
                        uint8_t dst_lo, uint8_t dst_hi,
                        uint8_t src_lo, uint8_t src_hi)
{
    frame[0] = FRAME_FC_0;
    frame[1] = FRAME_FC_1;
    frame[2] = s_seq;
    frame[3] = PAN_ID_LO;
    frame[4] = PAN_ID_HI;
    frame[5] = dst_lo;
    frame[6] = dst_hi;
    frame[7] = src_lo;
    frame[8] = src_hi;
    frame[9] = fn;
}

static bool header_ok(const uint8_t *f, uint8_t expected_fn)
{
    return f[0] == FRAME_FC_0 &&
           f[1] == FRAME_FC_1 &&
           f[3] == PAN_ID_LO  &&
           f[4] == PAN_ID_HI  &&
           f[5] == s_my_addr_lo &&
           f[6] == s_my_addr_hi &&
           f[7] == s_peer_addr_lo &&
           f[8] == s_peer_addr_hi &&
           f[9] == expected_fn;
}

static void ts_to_frame(uint8_t *p, uint64_t ts)
{
    p[0] = (uint8_t)(ts);
    p[1] = (uint8_t)(ts >> 8);
    p[2] = (uint8_t)(ts >> 16);
    p[3] = (uint8_t)(ts >> 24);
    p[4] = (uint8_t)(ts >> 32);
}

static uint64_t ts_from_frame(const uint8_t *p)
{
    return  (uint64_t)p[0]
         | ((uint64_t)p[1] << 8)
         | ((uint64_t)p[2] << 16)
         | ((uint64_t)p[3] << 24)
         | ((uint64_t)p[4] << 32);
}

static uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    dwt_readtxtimestamp(ts_tab);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) { ts <<= 8; ts |= ts_tab[i]; }
    return ts;
}
static uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    dwt_readrxtimestamp(ts_tab, DWT_COMPAT_NONE);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) { ts <<= 8; ts |= ts_tab[i]; }
    return ts;
}

/* --------------------------------------------------------------------- */
/* Wait loops                                                            */
/* --------------------------------------------------------------------- */

static bool wait_txfrs(int timeout_us)
{
    int waited = 0;
    while (waited < timeout_us) {
        if (dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
            return true;
        }
        esp_rom_delay_us(20);
        waited += 20;
    }
    return false;
}

/* Wait for RX completion / error / timeout. Polls every 50 µs; every
 * 1 ms without an event, yields via vTaskDelay(1) so IDLE can run and
 * feed the WDT. Assumes CONFIG_FREERTOS_HZ=1000 so vTaskDelay(1) = 1 ms. */
static uint32_t wait_rx_event(int timeout_us)
{
    int waited = 0;
    uint32_t status = 0;
    const int poll_us  = 50;
    const int yield_us = 1000;
    int since_yield = 0;

    while (waited < timeout_us) {
        status = dwt_readsysstatuslo();
        if (status & SYS_STATUS_RX_ANY) return status;
        esp_rom_delay_us(poll_us);
        waited      += poll_us;
        since_yield += poll_us;
        if (since_yield >= yield_us) {
            vTaskDelay(1);
            since_yield = 0;
            waited += (portTICK_PERIOD_MS * 1000);
        }
    }
    return status;
}

/* --------------------------------------------------------------------- */
/* DS-TWR distance computation (used on the responder)                   */
/* --------------------------------------------------------------------- */

/* Compute distance from the four DS-TWR durations.
 * All inputs are 64-bit, but the *differences* fit in 32 bits and that's
 * how Qorvo's reference computes them. Returns distance in meters. */
static double dstwr_distance_m(uint64_t poll_tx_ts, uint64_t resp_rx_ts,
                               uint64_t final_tx_ts,
                               uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                               uint64_t final_rx_ts)
{
    int64_t Ra = (int64_t)((uint32_t)(resp_rx_ts  - poll_tx_ts));
    int64_t Rb = (int64_t)((uint32_t)(final_rx_ts - resp_tx_ts));
    int64_t Da = (int64_t)((uint32_t)(final_tx_ts - resp_rx_ts));
    int64_t Db = (int64_t)((uint32_t)(resp_tx_ts  - poll_rx_ts));

    /* Numerator and denominator can be quite large; use int64 (safe up to
     * ~9.2e18) before converting to double. */
    int64_t num = Ra * Rb - Da * Db;
    int64_t den = Ra + Rb + Da + Db;
    if (den == 0) return -1.0;  /* shouldn't happen, defensive */

    double tof_ticks = (double)num / (double)den;
    double tof_s     = tof_ticks * DWT_TIME_UNITS;
    return tof_s * SPEED_OF_LIGHT_M_PER_S;
}

/* --------------------------------------------------------------------- */
/* Initiator                                                              */
/* --------------------------------------------------------------------- */

static esp_err_t initiator_cycle(uwb_range_result_t *result)
{
    dwt_forcetrxoff();
    /* Clear ALL TX and RX status bits — leftover from previous cycle
     * (especially after a failed delayed-TX) can break the next cycle. */
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK
                       | SYS_STATUS_TXFRB_BIT_MASK
                       | SYS_STATUS_TXPRS_BIT_MASK
                       | SYS_STATUS_TXPHS_BIT_MASK
                       | SYS_STATUS_RX_ANY);

    /* === 1. Send Poll, expect Response. === */
    uint8_t poll[POLL_FRAME_LEN];
    fill_header(poll, FN_POLL,
                s_peer_addr_lo, s_peer_addr_hi,
                s_my_addr_lo, s_my_addr_hi);

    dwt_writetxdata(POLL_FRAME_LEN, poll, 0);
    dwt_writetxfctrl(POLL_FRAME_LEN + FCS_LEN, 0, 1);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "Poll TX start failed");
        return ESP_FAIL;
    }

    /* === 2. Wait for Response RX. ===
     * Software timeout exceeds hw RESP_RX_TIMEOUT_UUS + POLL_TX_TO_RESP_RX_DLY_UUS
     * plus margin. */
    uint32_t status = wait_rx_event(15000);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        static int miss_log = 0;
        if (miss_log < 5 || miss_log % 50 == 0) {
            ESP_LOGW(TAG, "Resp RX miss #%d (status=0x%08lX)", miss_log, (unsigned long)status);
        }
        miss_log++;
        dwt_forcetrxoff();
        return ESP_OK;
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* === 3. Validate Response and extract timestamps. === */
    uint8_t  ranging_bit = 0;
    uint16_t flen = dwt_getframelength(&ranging_bit);
    if (flen != RESP_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) {
        ESP_LOGW(TAG, "Resp wrong length %u", flen);
        return ESP_OK;
    }
    dwt_readrxdata(rx_buf, RESP_FRAME_LEN, 0);
    if (!header_ok(rx_buf, FN_RESPONSE)) {
        ESP_LOGW(TAG, "Resp header mismatch (fn=0x%02X)", rx_buf[9]);
        return ESP_OK;
    }

    uint64_t poll_tx_ts = get_tx_timestamp_u64();
    uint64_t resp_rx_ts = get_rx_timestamp_u64();
    uint64_t poll_rx_ts = ts_from_frame(&rx_buf[RESP_POLL_RX_TS_IDX]);
    uint64_t resp_tx_ts = ts_from_frame(&rx_buf[RESP_RESP_TX_TS_IDX]);

    /* Periodic diagnostic — much less frequent now that protocol is working. */
    static int resp_dbg = 0;
    if (resp_dbg++ % 100 == 0) {
        ESP_LOGI(TAG, "Resp ts: poll_rx=%llu resp_tx=%llu",
                 (unsigned long long)poll_rx_ts, (unsigned long long)resp_tx_ts);
    }

    /* === 4. Schedule Final TX and embed all three local timestamps. ===
     *
     * Same ordering as the responder: set RX-after-TX config first (it
     * applies post-TX), then build and write the frame, then the time-
     * critical setdelayedtrxtime → starttx pair. */

    dwt_setrxaftertxdelay(FINAL_TX_TO_REPORT_RX_DLY_UUS);
    dwt_setrxtimeout(REPORT_RX_TIMEOUT_UUS);

    uint32_t final_tx_time = (uint32_t)((resp_rx_ts +
                                         (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME))
                                        >> 8);
    uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    uint8_t final_frame[FINAL_FRAME_LEN];
    fill_header(final_frame, FN_FINAL,
                s_peer_addr_lo, s_peer_addr_hi,
                s_my_addr_lo, s_my_addr_hi);
    ts_to_frame(&final_frame[FINAL_POLL_TX_TS_IDX],  poll_tx_ts);
    ts_to_frame(&final_frame[FINAL_RESP_RX_TS_IDX],  resp_rx_ts);
    ts_to_frame(&final_frame[FINAL_FINAL_TX_TS_IDX], final_tx_ts);

    dwt_writetxdata(FINAL_FRAME_LEN, final_frame, 0);
    dwt_writetxfctrl(FINAL_FRAME_LEN + FCS_LEN, 0, 1);

    dwt_setdelayedtrxtime(final_tx_time);

    int8_t txret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (txret != DWT_SUCCESS) {
        /* Same workaround as on responder: HPDWARN trips spuriously, but
         * TX often fires anyway. Poll for TXFRS to determine actual fate. */
        bool tx_fired = false;
        int waited = 0;
        while (waited < 8000) {
            if (dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK) {
                tx_fired = true;
                break;
            }
            esp_rom_delay_us(50);
            waited += 50;
        }
        static int late_log = 0;
        if (late_log < 5 || late_log % 100 == 0) {
            ESP_LOGW(TAG, "Final TX driver-error #%d (fired=%d)",
                     late_log, tx_fired ? 1 : 0);
        }
        late_log++;
        if (!tx_fired) {
            dwt_forcetrxoff();
            return ESP_OK;
        }
        dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    }

    /* === 5. Wait for Report RX. === */
    status = wait_rx_event(15000);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        static int rep_miss = 0;
        if (rep_miss++ % 20 == 0) {
            ESP_LOGW(TAG, "Report RX miss (status=0x%08lX)", (unsigned long)status);
        }
        dwt_forcetrxoff();
        return ESP_OK;
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* === 6. Validate Report and extract distance. === */
    flen = dwt_getframelength(&ranging_bit);
    if (flen != REPORT_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) {
        static int rep_len_log = 0;
        if (rep_len_log++ < 5) {
            ESP_LOGW(TAG, "Report wrong length %u (expected %d)",
                     flen, REPORT_FRAME_LEN + FCS_LEN);
        }
        return ESP_OK;
    }
    dwt_readrxdata(rx_buf, REPORT_FRAME_LEN, 0);

    if (!header_ok(rx_buf, FN_REPORT)) {
        static int rep_hdr_log = 0;
        if (rep_hdr_log++ < 5) {
            ESP_LOGW(TAG, "Report header mismatch: fn=0x%02X expected 0x%02X",
                     rx_buf[9], FN_REPORT);
        }
        return ESP_OK;
    }

    int32_t distance_mm = (int32_t)((uint32_t)rx_buf[REPORT_DIST_IDX + 0]
                                  | ((uint32_t)rx_buf[REPORT_DIST_IDX + 1] << 8)
                                  | ((uint32_t)rx_buf[REPORT_DIST_IDX + 2] << 16)
                                  | ((uint32_t)rx_buf[REPORT_DIST_IDX + 3] << 24));
    double distance_m = distance_mm / 1000.0;

    if (distance_m < -1.0 || distance_m > 500.0) {
        static int sane_log = 0;
        if (sane_log++ % 20 == 0) {
            ESP_LOGW(TAG, "Insane reported distance %.3f m", distance_m);
        }
        return ESP_OK;
    }

    if (result) {
        result->distance_m   = (float)(distance_m - g_uwb_distance_offset_m);
        result->timestamp_us = esp_timer_get_time();
        result->valid        = true;

        /* Pull the responder's IMU sample out of the Report. The bytes
         * are all 0xFF if the responder hadn't published a sample yet —
         * detect that and mark invalid. */
        bool all_ff = true;
        for (size_t i = 0; i < sizeof(lsm6_sample_t); i++) {
            if (rx_buf[REPORT_IMU_IDX + i] != 0xFF) { all_ff = false; break; }
        }
        if (!all_ff) {
            memcpy(&result->peer_imu, &rx_buf[REPORT_IMU_IDX],
                   sizeof(result->peer_imu));
            result->peer_imu_valid = true;
        } else {
            result->peer_imu_valid = false;
        }
    }

    /* Periodic confirmation. */
    static int success_count = 0;
    success_count++;
    if (success_count % 20 == 1) {
        ESP_LOGI(TAG, "Range OK #%d d=%.3f m", success_count, result->distance_m);
    }

    s_seq++;
    return ESP_OK;
}

/* --------------------------------------------------------------------- */
/* Responder                                                              */
/* --------------------------------------------------------------------- */

static esp_err_t responder_cycle(uwb_range_result_t *result)
{
    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK
                       | SYS_STATUS_TXFRB_BIT_MASK
                       | SYS_STATUS_TXPRS_BIT_MASK
                       | SYS_STATUS_TXPHS_BIT_MASK
                       | SYS_STATUS_RX_ANY);

    /* === 1. Listen for Poll. === */
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "rxenable failed");
        return ESP_FAIL;
    }

    uint32_t status = wait_rx_event(150000);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        static int poll_miss = 0;
        if (poll_miss < 5 || poll_miss % 50 == 0) {
            ESP_LOGW(TAG, "Poll RX miss #%d (status=0x%08lX)", poll_miss, (unsigned long)status);
        }
        poll_miss++;
        dwt_forcetrxoff();
        return ESP_OK;
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* === 2. Validate Poll. === */
    uint8_t  ranging_bit = 0;
    uint16_t flen = dwt_getframelength(&ranging_bit);
    if (flen != POLL_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) return ESP_OK;
    dwt_readrxdata(rx_buf, POLL_FRAME_LEN, 0);
    if (!header_ok(rx_buf, FN_POLL)) return ESP_OK;
    uint8_t peer_seq = rx_buf[2];

    static int poll_rx_count = 0;
    poll_rx_count++;
    if (poll_rx_count % 20 == 1) {
        ESP_LOGI(TAG, "Poll RX #%d (seq=%u)", poll_rx_count, peer_seq);
    }

    /* === 3. Schedule Response TX (with embedded poll_rx_ts, resp_tx_ts). ===
     *
     * Order of operations matters here: setdelayedtrxtime → starttx must
     * happen with minimal SPI in between, because the chip evaluates the
     * scheduled time against its current system time at starttx. Set up
     * RX-after-TX config FIRST (it only takes effect after TX completes,
     * so timing doesn't matter), then build the frame and write it, then
     * do the time-critical setdelayedtrxtime → starttx sequence. */

    /* Set up RX-after-TX first — these don't affect TX scheduling timing. */
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    int64_t t_before_sched_us = esp_timer_get_time();
    uint64_t poll_rx_ts = get_rx_timestamp_u64();
    uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
                                        (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME))
                                       >> 8);
    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    /* Build the Response frame BEFORE we set the delayed TX time, so the
     * time-critical window between setdelayedtrxtime and starttx is tight. */
    uint8_t resp[RESP_FRAME_LEN];
    s_seq = peer_seq;
    fill_header(resp, FN_RESPONSE,
                ADDR_INIT_LO, ADDR_INIT_HI,
                s_my_addr_lo, s_my_addr_hi);
    ts_to_frame(&resp[RESP_POLL_RX_TS_IDX], poll_rx_ts);
    ts_to_frame(&resp[RESP_RESP_TX_TS_IDX], resp_tx_ts);

    dwt_writetxdata(RESP_FRAME_LEN, resp, 0);
    dwt_writetxfctrl(RESP_FRAME_LEN + FCS_LEN, 0, 1);

    /* Now the tight window: setdelayedtrxtime → starttx. */
    dwt_setdelayedtrxtime(resp_tx_time);

    int8_t txret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    int64_t t_after_sched_us = esp_timer_get_time();
    int64_t sched_elapsed_us = t_after_sched_us - t_before_sched_us;

    static int sched_diag = 0;
    if (sched_diag++ < 5) {
        ESP_LOGI(TAG, "Resp TX sched host time: %lld us (delay budget %u uus)",
                 (long long)sched_elapsed_us,
                 (unsigned)POLL_RX_TO_RESP_TX_DLY_UUS);
    }

    if (txret != DWT_SUCCESS) {
        /* On this chip/driver, HPDWARN trips spuriously even with ample
         * scheduling margin. The driver attempts to cancel via CMD_TXRXOFF
         * but the cancel races with the TX preamble start and frequently
         * loses — TX fires anyway. Poll for TXFRS to detect actual fire.
         * If TXFRS arrives within a few ms, the TX really did happen with
         * the embedded timestamps we set; the initiator will receive it
         * correctly and the cycle continues. */
        bool tx_fired = false;
        int waited = 0;
        while (waited < 8000) {  /* 8 ms covers worst-case delay + air time */
            if (dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK) {
                tx_fired = true;
                break;
            }
            esp_rom_delay_us(50);
            waited += 50;
        }
        static int late_log = 0;
        if (late_log < 5 || late_log % 100 == 0) {
            ESP_LOGW(TAG, "Resp TX driver-error #%d (host %lld us, fired=%d)",
                     late_log, (long long)sched_elapsed_us, tx_fired ? 1 : 0);
        }
        late_log++;
        if (!tx_fired) {
            /* Genuine cancel — TX did not happen. Abort cycle. */
            dwt_forcetrxoff();
            return ESP_OK;
        }
        /* TX did fire. Clear the TXFRS bit and continue. */
        dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    }

    /* === 4. Wait for Final RX. ===
     * Software timeout exceeds hardware FINAL_RX_TIMEOUT_UUS plus detection
     * jitter. */
    status = wait_rx_event(20000);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        static int final_miss = 0;
        if (final_miss < 5 || final_miss % 20 == 0) {
            ESP_LOGW(TAG, "Final RX miss #%d (status=0x%08lX)",
                     final_miss, (unsigned long)status);
        }
        final_miss++;
        dwt_forcetrxoff();
        return ESP_OK;
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* === 5. Validate Final and read embedded timestamps. === */
    flen = dwt_getframelength(&ranging_bit);
    if (flen != FINAL_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) return ESP_OK;
    dwt_readrxdata(rx_buf, FINAL_FRAME_LEN, 0);
    if (!header_ok(rx_buf, FN_FINAL)) return ESP_OK;

    uint64_t poll_tx_ts  = ts_from_frame(&rx_buf[FINAL_POLL_TX_TS_IDX]);
    uint64_t resp_rx_ts  = ts_from_frame(&rx_buf[FINAL_RESP_RX_TS_IDX]);
    uint64_t final_tx_ts = ts_from_frame(&rx_buf[FINAL_FINAL_TX_TS_IDX]);
    uint64_t final_rx_ts = get_rx_timestamp_u64();

    /* === 6. Compute DS-TWR distance. === */
    double distance_m = dstwr_distance_m(poll_tx_ts, resp_rx_ts, final_tx_ts,
                                         poll_rx_ts, resp_tx_ts, final_rx_ts);

    /* Periodic timestamp diagnostic, like SS-TWR. */
    static int diag_count = 0;
    if (diag_count++ % 30 == 0) {
        uint32_t Ra = (uint32_t)(resp_rx_ts  - poll_tx_ts);
        uint32_t Rb = (uint32_t)(final_rx_ts - resp_tx_ts);
        uint32_t Da = (uint32_t)(final_tx_ts - resp_rx_ts);
        uint32_t Db = (uint32_t)(resp_tx_ts  - poll_rx_ts);
        ESP_LOGI(TAG, "TS: pT=%llu pR=%llu rT=%llu rR=%llu fT=%llu fR=%llu",
                 (unsigned long long)poll_tx_ts,
                 (unsigned long long)poll_rx_ts,
                 (unsigned long long)resp_tx_ts,
                 (unsigned long long)resp_rx_ts,
                 (unsigned long long)final_tx_ts,
                 (unsigned long long)final_rx_ts);
        ESP_LOGI(TAG, "    Ra=%lu Rb=%lu Da=%lu Db=%lu d=%.3f m",
                 (unsigned long)Ra, (unsigned long)Rb,
                 (unsigned long)Da, (unsigned long)Db, distance_m);
    }

    if (distance_m < -1.0 || distance_m > 500.0) {
        static int sane_log = 0;
        if (sane_log++ % 20 == 0) {
            ESP_LOGW(TAG, "Insane DS-TWR distance %.3f m", distance_m);
        }
        dwt_forcetrxoff();
        return ESP_OK;
    }

    /* Apply offset and store locally. */
    double distance_m_calibrated = distance_m - g_uwb_distance_offset_m;
    if (result) {
        result->distance_m   = (float)distance_m_calibrated;
        result->timestamp_us = esp_timer_get_time();
        result->valid        = true;
    }

    /* === 7. Send Report back so the initiator knows the distance too. === */
    int32_t distance_mm = (int32_t)(distance_m_calibrated * 1000.0);
    uint8_t report[REPORT_FRAME_LEN];
    fill_header(report, FN_REPORT,
                ADDR_INIT_LO, ADDR_INIT_HI,
                s_my_addr_lo, s_my_addr_hi);
    report[REPORT_DIST_IDX + 0] = (uint8_t)(distance_mm);
    report[REPORT_DIST_IDX + 1] = (uint8_t)(distance_mm >> 8);
    report[REPORT_DIST_IDX + 2] = (uint8_t)(distance_mm >> 16);
    report[REPORT_DIST_IDX + 3] = (uint8_t)(distance_mm >> 24);

    /* Snapshot the latest local IMU sample (if any) into the Report. */
    lsm6_sample_t imu_snap;
    bool imu_snap_valid;
    portENTER_CRITICAL(&s_imu_lock);
    imu_snap       = s_local_imu;
    imu_snap_valid = s_local_imu_valid;
    portEXIT_CRITICAL(&s_imu_lock);
    if (imu_snap_valid) {
        memcpy(&report[REPORT_IMU_IDX], &imu_snap, sizeof(imu_snap));
    } else {
        /* Zero the IMU bytes so the initiator can detect "no sample yet"
         * via a known sentinel. We use NaN in the first float as the
         * "invalid" signal (zero would be ambiguous with a real reading
         * of motionless sensor at rest). */
        memset(&report[REPORT_IMU_IDX], 0xFF, sizeof(imu_snap));
    }

    dwt_writetxdata(REPORT_FRAME_LEN, report, 0);
    dwt_writetxfctrl(REPORT_FRAME_LEN + FCS_LEN, 0, 1);

    /* Report is fire-and-forget; no RX after. */
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGW(TAG, "Report TX start failed");
        return ESP_OK;
    }
    if (!wait_txfrs(3000)) {
        ESP_LOGW(TAG, "Report TXFRS timeout");
    }

    return ESP_OK;
}

/* --------------------------------------------------------------------- */
/* Public API                                                             */
/* --------------------------------------------------------------------- */

esp_err_t uwb_init(uwb_role_t role,
                   char responder_addr_suffix,
                   int mosi, int miso, int sclk, int cs, int rst)
{
    s_role = role;

    /* Validate / normalise the responder suffix. Accept 'A'..'I' (9 max).
     * Anything else (including 0) becomes the default 'A'. */
    char suffix = responder_addr_suffix;
    if (suffix < 'A' || suffix > 'I') suffix = DEFAULT_RESP_SUFFIX;

    if (role == UWB_ROLE_INITIATOR) {
        s_my_addr_lo   = ADDR_INIT_LO;     s_my_addr_hi   = ADDR_INIT_HI;
        /* Peer addr will be set per-cycle by uwb_perform_ranging. Initialise
         * to the default (single-peer compatibility) so logs read sensibly
         * before the first ranging call. */
        s_peer_addr_lo = ADDR_RESP_PREFIX; s_peer_addr_hi = DEFAULT_RESP_SUFFIX;
    } else {
        s_my_addr_lo   = ADDR_RESP_PREFIX; s_my_addr_hi   = suffix;
        s_peer_addr_lo = ADDR_INIT_LO;     s_peer_addr_hi = ADDR_INIT_HI;
    }
    ESP_LOGI(TAG, "Init UWB role=%s addr=%c%c peer=%c%c (DS-TWR)",
             role == UWB_ROLE_INITIATOR ? "initiator" : "responder",
             s_my_addr_lo, s_my_addr_hi,
             s_peer_addr_lo, s_peer_addr_hi);

    esp_err_t err = dwm3000_init(mosi, miso, sclk, cs, rst);
    if (err != ESP_OK) return err;

    uint32_t dev_id = 0;
    err = dwm3000_read_devid(&dev_id);
    if (err != ESP_OK) return err;
    if ((dev_id >> 16) != 0xDECA) {
        ESP_LOGE(TAG, "Wrong DEV_ID: 0x%08lX", (unsigned long)dev_id);
        return ESP_FAIL;
    }

    if (dwt_configure((dwt_config_t *)&s_uwb_config) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_configure failed");
        return ESP_FAIL;
    }
    dwt_configuretxrf((dwt_txconfig_t *)&s_txconfig);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    ESP_LOGI(TAG, "UWB ready (chan 5, PLEN 64, 6.8 Mbps, DS-TWR)");
    return ESP_OK;
}

esp_err_t uwb_perform_ranging(char peer_addr_suffix, uwb_range_result_t *result)
{
    if (result) {
        result->valid          = false;
        result->distance_m     = 0.0f;
        result->timestamp_us   = 0;
        result->peer_imu_valid = false;
        /* Don't bother zeroing result->peer_imu — meaningless when invalid. */
    }

    if (s_role == UWB_ROLE_INITIATOR) {
        /* Validate the peer suffix; fall back to default if junk. The
         * responder ignores peer_addr_suffix entirely (it only ever
         * talks to the single initiator 'VE'). */
        char suffix = peer_addr_suffix;
        if (suffix < 'A' || suffix > 'I') suffix = DEFAULT_RESP_SUFFIX;

        /* Latch the per-cycle peer addr into the module-static fields that
         * fill_header and header_ok read. This is safe because
         * uwb_perform_ranging is called serially from a single task. */
        s_peer_addr_lo = ADDR_RESP_PREFIX;
        s_peer_addr_hi = (uint8_t)suffix;

        return initiator_cycle(result);
    }
    return responder_cycle(result);
}

void uwb_publish_local_imu(const lsm6_sample_t *sample)
{
    /* Only the responder embeds this into Report frames; initiator-side
     * calls are accepted but unused. Storing on both sides anyway is
     * harmless and makes the API role-agnostic. */
    if (!sample) return;
    portENTER_CRITICAL(&s_imu_lock);
    s_local_imu       = *sample;
    s_local_imu_valid = true;
    portEXIT_CRITICAL(&s_imu_lock);
}