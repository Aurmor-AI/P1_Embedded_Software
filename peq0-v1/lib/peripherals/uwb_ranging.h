#pragma once
#include <stdint.h>
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UWB_ROLE_INITIATOR,   // Sends poll, computes distance
    UWB_ROLE_RESPONDER,   // Receives poll, sends back response
} uwb_role_t;

typedef struct {
    float distance_m;     // Computed distance in meters
    int64_t timestamp_us; // When this measurement was taken
    bool valid;           // True if a successful exchange happened
} uwb_range_result_t;

esp_err_t uwb_init(uwb_role_t role);
esp_err_t uwb_perform_ranging(uwb_range_result_t *result);

#ifdef __cplusplus
}
#endif