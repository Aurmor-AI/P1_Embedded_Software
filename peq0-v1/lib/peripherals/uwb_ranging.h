#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "lsm6dsv.h"   /* for lsm6_sample_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UWB_ROLE_INITIATOR,
    UWB_ROLE_RESPONDER,
} uwb_role_t;

typedef struct {
    /* Distance in meters after applying g_uwb_distance_offset_m. Valid
     * only when .valid is true. Negative values can occur very close in
     * if the calibration offset is too large; that's expected. */
    float   distance_m;
    int64_t timestamp_us;
    bool    valid;

    /* Peer (responder) IMU snapshot, carried inside the Report frame.
     * Only meaningful on the initiator side, after a successful range
     * cycle. peer_imu_valid is false on the responder. */
    lsm6_sample_t peer_imu;
    bool          peer_imu_valid;
} uwb_range_result_t;

/* Initialise UWB. For UWB_ROLE_RESPONDER, the responder's own address
 * suffix is 'A'..'I' for boards WA..WI. Pass 0 to use the default 'A'.
 * Initiator ignores responder_addr_suffix. */
esp_err_t uwb_init(uwb_role_t role,
                   char responder_addr_suffix,
                   int mosi, int miso, int sclk, int cs, int rst);

/* Performs one DS-TWR cycle.
 *
 * On the initiator, peer_addr_suffix selects which responder to talk to
 * ('A'..'I' for WA..WI). The responder ignores these parameters.
 *
 * Four-frame exchange:
 *   Initiator:  Poll TX -> Response RX -> Final TX (with timestamps) -> Report RX
 *   Responder:  Poll RX (only if addressed to us) -> Response TX -> Final RX
 *                -> compute DS-TWR ToF -> Report TX (with distance + IMU)
 *
 * Returns ESP_OK whether or not the cycle completed end-to-end — check
 * result->valid for that. Returns an error only on driver/SPI failure. */
esp_err_t uwb_perform_ranging(char peer_addr_suffix, uwb_range_result_t *result);

/* Publish the local IMU sample so the responder can embed it in the next
 * Report frame. Should be called from whatever task produces IMU samples
 * (e.g. the imu timer callback in main.cpp). Safe to call concurrently
 * with uwb_perform_ranging — uses an internal spinlock.
 *
 * Has no effect on the initiator. */
void uwb_publish_local_imu(const lsm6_sample_t *sample);

/* Calibration offset subtracted from raw distance (meters). Leave at 0
 * until you've run a known-distance calibration. */
extern float g_uwb_distance_offset_m;

#ifdef __cplusplus
}
#endif