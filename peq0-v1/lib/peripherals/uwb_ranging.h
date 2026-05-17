#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

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
} uwb_range_result_t;

esp_err_t uwb_init(uwb_role_t role, int mosi, int miso, int sclk, int cs, int rst);

/* Performs one DS-TWR cycle.
 *
 * Four-frame exchange:
 *   Initiator:  Poll TX -> Response RX -> Final TX (with timestamps) -> Report RX
 *   Responder:  Poll RX -> Response TX -> Final RX (read timestamps,
 *                 compute DS-TWR ToF) -> Report TX (with distance)
 *
 * On success, *result is populated on BOTH ends with the responder's
 * computed distance (initiator gets it via the Report frame).
 *
 * Returns ESP_OK whether or not the cycle completed end-to-end — check
 * result->valid for that. Returns an error only on driver/SPI failure. */
esp_err_t uwb_perform_ranging(uwb_range_result_t *result);

/* Calibration offset subtracted from raw distance (meters). Leave at 0
 * until you've run a known-distance calibration. */
extern float g_uwb_distance_offset_m;

#ifdef __cplusplus
}
#endif