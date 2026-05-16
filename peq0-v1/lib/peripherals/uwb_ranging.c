/* uwb_ranging.c — SS-TWR with three frames (Poll / Response / Final)
 *
 * Frame sequence:
 *
 *   Initiator                          Responder
 *      |                                   |
 *      |--- Poll ------------------------->|  (magic, seq)
 *      |                                   |  reads poll_rx_ts
 *      |<-- Response ----------------------|  (magic, seq, poll_rx_ts, resp_tx_ts)
 *      |   reads resp_rx_ts                |
 *      |   computes ToF -> distance        |
 *      |--- Final ------------------------>|  (magic, seq, distance_mm)
 *      |                                   |  stores distance
 *
 * Why three frames instead of two: the responder cannot compute distance on
 * its own (it doesn't see the initiator's poll_tx or resp_rx), so the
 * initiator shares its result back. The Final is a short fire-and-forget
 * frame; the responder uses RX-after-TX to catch it.
 *
 * TX/RX state machine — the critical bit that broke the previous version:
 *
 *   Initiator: TX Poll (DWT_RESPONSE_EXPECTED) → HW arms RX → wait RXFCG →
 *              read Response → TX Final (no RX expected) → done.
 *
 *   Responder: RX (forever) → got Poll → TX Response (DWT_RESPONSE_EXPECTED)
 *              → HW arms RX → wait RXFCG → read Final → done.
 *
 * Using DWT_RESPONSE_EXPECTED + dwt_setrxaftertxdelay means the chip arms
 * RX in hardware as part of TX completion. No software gap. Polling-based
 * RX-enable after TX is too slow to catch the partner's response on PLEN 64.
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
 * to tune this. Off-by-1 here = off by ~0.5 cm in distance. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Time units. dwt_readsystime / timestamps are in 15.65 ps units (DTU).
 * UWB µs (uus) = 65536 DTU. */
#define UUS_TO_DWT_TIME 65536

/* Initiator: how long after Poll-TX to enable RX, and how long to wait
 * for Response. Both values are in UWB µs (uus = 65536 DTU = ~1.0256 µs
 * wall-clock). These have to be aligned with the responder's scheduled
 * response TX time below — if the initiator's RX window opens too late
 * or closes too early, you'll see "Resp RX miss" with RXPTO in status.
 *
 * Sized to comfortably cover ESP-IDF SPI overhead on both ends (~9 SPI
 * transactions between RX-of-Poll and TX-of-Response on the responder). */
#define POLL_TX_TO_RESP_RX_DLY_UUS  1100
#define RESP_RX_TIMEOUT_UUS         1200

/* Responder: scheduled delay from poll-RX-timestamp to response-TX. Must
 * exceed the wall-clock time taken for ~9 SPI transactions (frame length,
 * RX data read, RX timestamp read, TX data write, FCTRL, two delay regs,
 * TX-start) plus task scheduling jitter. 1500 uus = ~1.54 ms — generous
 * but safe. If this is too small you'll see "Resp TX delayed-start failed". */
#define POLL_RX_TO_RESP_TX_DLY_UUS  1500

/* Responder: after Response TX, how long to wait for the Final.
 *
 * The initiator's Final TX fires fairly quickly after Response RX
 * (~1.5 ms of SPI overhead to build and start the Final frame). So
 * the responder's RX window must open early — if it opens too late,
 * the Final's preamble has already passed and we get RXPTO.
 *
 * Open the window immediately (delay=0), keep it open for 4 ms to
 * cover initiator-side SPI jitter. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS  0
#define FINAL_RX_TIMEOUT_UUS         4000

/* Speed of light in air ≈ 299,702,547 m/s. DW3000 datasheet uses this
 * value for distance computation. */
#define SPEED_OF_LIGHT_M_PER_S      299702547.0

/* DW3000 timestamp tick period in seconds. 1 / (499.2e6 * 128) ≈ 15.65 ps. */
#define DWT_TIME_UNITS              (1.0 / 499200000.0 / 128.0)

float g_uwb_distance_offset_m = 0.0f;

/* --------------------------------------------------------------------- */
/* Frame layout — IEEE 802.15.4 short addressing (PAN-ID + short addr).  */
/* --------------------------------------------------------------------- */

/* Common header (10 bytes):
 *   [0..1] frame control (0x88 0x41)
 *   [2]    sequence number
 *   [3..4] PAN ID (0xDECA)
 *   [5..6] dest short address ('W' 'A' or 'V' 'E')
 *   [7..8] src  short address
 *   [9]    function code
 *
 * Then function-specific payload, then 2-byte FCS appended by hardware.
 *
 * Magic bytes we check on RX:
 *   - frame control = 0x88 0x41
 *   - PAN ID = 0xDE 0xCA
 *   - function code at byte 9
 *
 * Initiator address = 'V''E'  (V = Vector, E = Initiator-ish, mnemonic only)
 * Responder address = 'W''A'  (anything distinct works)
 */

#define FRAME_FC_0          0x41
#define FRAME_FC_1          0x88
#define PAN_ID_LO           0xCA
#define PAN_ID_HI           0xDE

#define ADDR_INIT_LO        'V'
#define ADDR_INIT_HI        'E'
#define ADDR_RESP_LO        'W'
#define ADDR_RESP_HI        'A'

#define FN_POLL             0x21
#define FN_RESPONSE         0x10
#define FN_FINAL            0x29

#define FCS_LEN             2

/* Poll: header only (10 bytes) */
#define POLL_FRAME_LEN      10

/* Response: header + poll_rx_ts(5) + resp_tx_ts(5) = 20 bytes
 * Timestamps are 40-bit DW3000 values, sent low-byte first. */
#define RESP_FRAME_LEN      20
#define RESP_POLL_RX_TS_IDX 10
#define RESP_RESP_TX_TS_IDX 15

/* Final: header + distance_mm(4) = 14 bytes
 * distance_mm is signed int32, little-endian. We use mm so we don't need
 * to send floats over the air. Range fits easily in int32. */
#define FINAL_FRAME_LEN     14
#define FINAL_DIST_IDX      10

/* --------------------------------------------------------------------- */
/* State                                                                  */
/* --------------------------------------------------------------------- */

static uwb_role_t s_role;
static uint8_t    s_seq = 0;
static uint8_t    s_my_addr_lo;
static uint8_t    s_my_addr_hi;
static uint8_t    s_peer_addr_lo;
static uint8_t    s_peer_addr_hi;

#define RX_BUF_LEN 32
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
    .sfdType       = 1,                /* IEEE 802.15.4z 8-bit SFD */
    .dataRate      = DWT_BR_6M8,
    .phrMode       = DWT_PHRMODE_STD,
    .phrRate       = DWT_PHRRATE_STD,
    .sfdTO         = (64 + 1 + 8 - 8), /* sfdTO for PLEN 64 */
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

/* Validate header + function code + sender. Returns true if frame matches
 * what we expected from the peer. Length check is up to the caller. */
static bool header_ok(const uint8_t *f, uint8_t expected_fn)
{
    return f[0] == FRAME_FC_0 &&
           f[1] == FRAME_FC_1 &&
           f[3] == PAN_ID_LO  &&
           f[4] == PAN_ID_HI  &&
           f[5] == s_my_addr_lo &&     /* addressed to me */
           f[6] == s_my_addr_hi &&
           f[7] == s_peer_addr_lo &&   /* from the peer */
           f[8] == s_peer_addr_hi &&
           f[9] == expected_fn;
}

/* Write a 40-bit DW3000 timestamp into a frame, little-endian, 5 bytes. */
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

/* Read DW3000's RX/TX timestamps as 64-bit values (only low 40 bits used).
 * The segment arg to dwt_readrxtimestamp is unused for DW3xxx — set to
 * DWT_COMPAT_NONE per the API header. If your driver version also takes
 * a segment arg on dwt_readtxtimestamp, add it there too. */
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
/* Poll loops for status bits.                                            */
/* --------------------------------------------------------------------- */

/* Wait for TXFRS or timeout (microseconds). Returns true on TXFRS. */
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

/* Wait for any RX completion or RX error/timeout, with software timeout.
 * Returns the status word; caller checks RXFCG vs error bits. */
static uint32_t wait_rx_event(int timeout_us)
{
    int waited = 0;
    uint32_t status = 0;
    while (waited < timeout_us) {
        status = dwt_readsysstatuslo();
        if (status & SYS_STATUS_RX_ANY) return status;
        esp_rom_delay_us(50);
        waited += 50;
    }
    /* Software timeout reached without hardware status — return what we have. */
    return status;
}

/* --------------------------------------------------------------------- */
/* Initiator                                                              */
/* --------------------------------------------------------------------- */

static esp_err_t initiator_cycle(uwb_range_result_t *result)
{
    /* Always start clean. */
    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_RX_ANY);

    /* --- 1. Send Poll, expect Response. --- */
    uint8_t poll[POLL_FRAME_LEN];
    fill_header(poll, FN_POLL,
                ADDR_RESP_LO, ADDR_RESP_HI,
                s_my_addr_lo, s_my_addr_hi);

    dwt_writetxdata(POLL_FRAME_LEN, poll, 0);
    dwt_writetxfctrl(POLL_FRAME_LEN + FCS_LEN, 0, 1);

    /* Arm RX-after-TX so we catch the Response without a software gap. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "Poll TX start failed");
        return ESP_FAIL;
    }

    /* --- 2. Wait for Response RX. --- */
    uint32_t status = wait_rx_event(5000);  /* 5 ms covers UWB timeout */
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            static int err_log = 0;
            if (err_log++ % 50 == 0) {
                ESP_LOGW(TAG, "Resp RX miss (status=0x%08lX)", (unsigned long)status);
            }
        } else {
            static int to_log = 0;
            if (to_log++ % 50 == 0) {
                ESP_LOGW(TAG, "Resp RX sw-timeout (status=0x%08lX)", (unsigned long)status);
            }
            dwt_forcetrxoff();
        }
        return ESP_OK;  /* not a driver failure — just no answer this cycle */
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* --- 3. Validate Response frame. --- */
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

    /* --- 4. Compute ToF and distance. --- */
    uint64_t poll_tx_ts = get_tx_timestamp_u64();
    uint64_t resp_rx_ts = get_rx_timestamp_u64();
    uint64_t poll_rx_ts = ts_from_frame(&rx_buf[RESP_POLL_RX_TS_IDX]);
    uint64_t resp_tx_ts = ts_from_frame(&rx_buf[RESP_RESP_TX_TS_IDX]);

    /* All arithmetic in 32-bit subtraction (deltas fit in <2^32), with the
     * top bits naturally truncated. This is the standard SS-TWR pattern. */
    uint32_t rtd_init = (uint32_t)(resp_rx_ts - poll_tx_ts);
    uint32_t rtd_resp = (uint32_t)(resp_tx_ts - poll_rx_ts);

    double tof = ((double)rtd_init - (double)rtd_resp) / 2.0 * DWT_TIME_UNITS;
    double distance_m = tof * SPEED_OF_LIGHT_M_PER_S - g_uwb_distance_offset_m;

    /* Periodic raw-timestamp diagnostic. Print every 30 cycles so we
     * can spot patterns (bimodal distances, clock drift, etc). */
    static int diag_count = 0;
    if (diag_count++ % 30 == 0) {
        ESP_LOGI(TAG, "TS: poll_tx=%llu poll_rx=%llu resp_tx=%llu resp_rx=%llu",
                 (unsigned long long)poll_tx_ts, (unsigned long long)poll_rx_ts,
                 (unsigned long long)resp_tx_ts, (unsigned long long)resp_rx_ts);
        ESP_LOGI(TAG, "    rtd_init=%lu rtd_resp=%lu diff=%ld d=%.3f m",
                 (unsigned long)rtd_init, (unsigned long)rtd_resp,
                 (long)((int32_t)rtd_init - (int32_t)rtd_resp), distance_m);
    }

    /* Sanity-check before reporting. ToF math can produce garbage if the
     * peer is out of range or a frame got corrupted. */
    if (distance_m < -1.0 || distance_m > 500.0) {
        static int sane_log = 0;
        if (sane_log++ % 20 == 0) {
            ESP_LOGW(TAG, "Insane distance %.2f m (rtd_init=%lu rtd_resp=%lu)",
                     distance_m, (unsigned long)rtd_init, (unsigned long)rtd_resp);
        }
        return ESP_OK;
    }

    if (result) {
        result->distance_m   = (float)distance_m;
        result->timestamp_us = esp_timer_get_time();
        result->valid        = true;
    }

    /* Periodic confirmation. */
    static int success_count = 0;
    success_count++;
    if (success_count % 20 == 1) {
        ESP_LOGI(TAG, "Range OK #%d d=%.3f m", success_count, distance_m);
    }

    /* --- 5. Send Final with the distance, so responder knows it too. --- */
    uint8_t final[FINAL_FRAME_LEN];
    fill_header(final, FN_FINAL,
                ADDR_RESP_LO, ADDR_RESP_HI,
                s_my_addr_lo, s_my_addr_hi);
    int32_t distance_mm = (int32_t)(distance_m * 1000.0);
    final[FINAL_DIST_IDX + 0] = (uint8_t)(distance_mm);
    final[FINAL_DIST_IDX + 1] = (uint8_t)(distance_mm >> 8);
    final[FINAL_DIST_IDX + 2] = (uint8_t)(distance_mm >> 16);
    final[FINAL_DIST_IDX + 3] = (uint8_t)(distance_mm >> 24);

    dwt_writetxdata(FINAL_FRAME_LEN, final, 0);
    dwt_writetxfctrl(FINAL_FRAME_LEN + FCS_LEN, 0, 1);

    /* No response expected for Final — fire and forget. */
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGW(TAG, "Final TX start failed");
        return ESP_OK;
    }
    if (!wait_txfrs(3000)) {
        ESP_LOGW(TAG, "Final TXFRS timeout");
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
    dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_RX_ANY);

    /* --- 1. Listen for Poll. Time-bounded so we can return to the task
     *        loop and remain responsive to shutdown / role-change. --- */
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);  /* no HW timeout; we'll use a SW one */
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "rxenable failed");
        return ESP_FAIL;
    }

    /* Wait up to ~150 ms for a Poll. At 10 Hz partner cadence we should
     * see one every 100 ms. */
    uint32_t status = wait_rx_event(150000);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        dwt_forcetrxoff();
        return ESP_OK;  /* no poll this window, try again */
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* --- 2. Validate Poll. --- */
    uint8_t  ranging_bit = 0;
    uint16_t flen = dwt_getframelength(&ranging_bit);
    if (flen != POLL_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) {
        return ESP_OK;
    }
    dwt_readrxdata(rx_buf, POLL_FRAME_LEN, 0);
    if (!header_ok(rx_buf, FN_POLL)) {
        return ESP_OK;
    }
    uint8_t peer_seq = rx_buf[2];

    /* Periodic confirmation that Poll RX is working. */
    static int poll_rx_count = 0;
    poll_rx_count++;
    if (poll_rx_count % 20 == 1) {
        ESP_LOGI(TAG, "Poll RX #%d (seq=%u)", poll_rx_count, peer_seq);
    }

    /* --- 3. Schedule Response TX at poll_rx_ts + delay. ---
     *
     * (a) dwt_setdelayedtrxtime takes the *top 32 bits* of a 40-bit
     *     timestamp. The hardware then schedules TX at that value
     *     left-shifted by 8 — but it *also* zeroes the bottom bit
     *     of the value we passed in (so the bottom 9 bits of the
     *     full 40-bit timestamp are guaranteed zero, not just the
     *     bottom 8). This is documented in the Qorvo SS-TWR examples
     *     but not in the API header.
     *
     * (b) Because of (a), the *embedded* resp_tx_ts that we put in
     *     the frame must mask bit 0 of resp_tx_time *before* the
     *     left-shift. If we don't, on cycles where bit 0 was 1 the
     *     embedded value is 256 ticks higher than what actually went
     *     on the air. After halving in the initiator's ToF math,
     *     that's a 128-tick (~0.6 m) bimodal error in computed
     *     distance — every other cycle wrong by half a meter.
     *
     * (c) The embedded value includes TX_ANT_DLY because the air-time
     *     timestamp the initiator measures is offset by that delay.
     */
    uint64_t poll_rx_ts = get_rx_timestamp_u64();
    uint32_t resp_tx_time = (uint32_t)((poll_rx_ts +
                                        (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME))
                                       >> 8);
    dwt_setdelayedtrxtime(resp_tx_time);

    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    /* Build Response frame. */
    uint8_t resp[RESP_FRAME_LEN];
    s_seq = peer_seq;  /* mirror the initiator's seq for the Response */
    fill_header(resp, FN_RESPONSE,
                ADDR_INIT_LO, ADDR_INIT_HI,
                s_my_addr_lo, s_my_addr_hi);
    ts_to_frame(&resp[RESP_POLL_RX_TS_IDX], poll_rx_ts);
    ts_to_frame(&resp[RESP_RESP_TX_TS_IDX], resp_tx_ts);

    dwt_writetxdata(RESP_FRAME_LEN, resp, 0);
    dwt_writetxfctrl(RESP_FRAME_LEN + FCS_LEN, 0, 1);

    /* Arm RX-after-TX so we catch the Final without a software gap. */
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    int8_t txret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (txret != DWT_SUCCESS) {
        /* Most common cause: scheduled TX time has already passed.
         * That means the SPI write took longer than expected — bump
         * POLL_RX_TO_RESP_TX_DLY_UUS. */
        static int late_log = 0;
        if (late_log++ % 20 == 0) {
            ESP_LOGW(TAG, "Resp TX delayed-start failed (late?); raise POLL_RX_TO_RESP_TX_DLY_UUS");
        }
        dwt_forcetrxoff();
        return ESP_OK;
    }

    /* --- 4. Wait for Final RX. --- */
    status = wait_rx_event(8000);
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }
        static int final_miss = 0;
        if (final_miss++ % 20 == 0) {
            ESP_LOGW(TAG, "Final RX miss (status=0x%08lX, count=%d)",
                     (unsigned long)status, final_miss);
        }
        dwt_forcetrxoff();
        return ESP_OK;  /* Response sent but distance lost this cycle */
    }
    dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

    /* --- 5. Validate Final and extract distance. --- */
    flen = dwt_getframelength(&ranging_bit);
    if (flen != FINAL_FRAME_LEN + FCS_LEN || flen > RX_BUF_LEN) {
        return ESP_OK;
    }
    dwt_readrxdata(rx_buf, FINAL_FRAME_LEN, 0);
    if (!header_ok(rx_buf, FN_FINAL)) {
        return ESP_OK;
    }
    int32_t distance_mm = (int32_t)((uint32_t)rx_buf[FINAL_DIST_IDX + 0]
                                  | ((uint32_t)rx_buf[FINAL_DIST_IDX + 1] << 8)
                                  | ((uint32_t)rx_buf[FINAL_DIST_IDX + 2] << 16)
                                  | ((uint32_t)rx_buf[FINAL_DIST_IDX + 3] << 24));

    if (result) {
        result->distance_m   = distance_mm / 1000.0f;
        result->timestamp_us = esp_timer_get_time();
        result->valid        = true;
    }
    return ESP_OK;
}

/* --------------------------------------------------------------------- */
/* Public API                                                             */
/* --------------------------------------------------------------------- */

esp_err_t uwb_init(uwb_role_t role, int mosi, int miso, int sclk, int cs, int rst)
{
    s_role = role;
    if (role == UWB_ROLE_INITIATOR) {
        s_my_addr_lo   = ADDR_INIT_LO; s_my_addr_hi   = ADDR_INIT_HI;
        s_peer_addr_lo = ADDR_RESP_LO; s_peer_addr_hi = ADDR_RESP_HI;
    } else {
        s_my_addr_lo   = ADDR_RESP_LO; s_my_addr_hi   = ADDR_RESP_HI;
        s_peer_addr_lo = ADDR_INIT_LO; s_peer_addr_hi = ADDR_INIT_HI;
    }
    ESP_LOGI(TAG, "Init UWB role=%s addr=%c%c peer=%c%c",
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

    ESP_LOGI(TAG, "UWB ready (chan 5, PLEN 64, 6.8 Mbps)");
    return ESP_OK;
}

esp_err_t uwb_perform_ranging(uwb_range_result_t *result)
{
    if (result) {
        result->valid        = false;
        result->distance_m   = 0.0f;
        result->timestamp_us = 0;
    }
    if (s_role == UWB_ROLE_INITIATOR) return initiator_cycle(result);
    return responder_cycle(result);
}