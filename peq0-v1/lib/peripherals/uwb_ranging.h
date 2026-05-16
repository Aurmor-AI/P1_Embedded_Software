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
    /* Distance after applying calibration offset, in meters. Valid only
     * when .valid is true. Negative values can occur for very small
     * distances when the calibration offset is wrong; that's expected. */
    float   distance_m;
    int64_t timestamp_us;
    bool    valid;
} uwb_range_result_t;

esp_err_t uwb_init(uwb_role_t role, int mosi, int miso, int sclk, int cs, int rst);

/* Performs one full DS-TWR exchange (initiator) or services one cycle
 * (responder). On the initiator, on success, *result is populated with
 * the distance reported by the responder via the Report frame. On the
 * responder, *result tracks the distance the responder itself computed.
 * Either way, .valid indicates whether the cycle completed end-to-end. */
esp_err_t uwb_perform_ranging(uwb_range_result_t *result);

/* Calibration offset applied to the raw distance, in meters. Set to
 * non-zero (typically a small negative number to subtract overshoot)
 * after running the calibration procedure. Phase 4c-2 leaves it at 0. */
extern float g_uwb_distance_offset_m;

/* Phase 4a/4b primitives kept for backwards compatibility. */
esp_err_t uwb_send_poll_blocking(void);
esp_err_t uwb_receive_frame_blocking(uint8_t *out_buf, uint16_t buf_size,
                                     uint16_t *out_len, int timeout_ms);

#ifdef __cplusplus
}
#endif