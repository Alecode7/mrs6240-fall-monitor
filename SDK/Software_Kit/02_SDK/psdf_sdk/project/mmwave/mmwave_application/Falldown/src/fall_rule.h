#ifndef FALL_RULE_H
#define FALL_RULE_H

#include <stdbool.h>
#include <stdint.h>

#include "fall_pipeline.h"
#include "prj_config.h"

typedef enum {
    FALL_STATE_NORMAL = 0,
    FALL_STATE_SUSPECTED,
    FALL_STATE_CONFIRMED,
} FallState_e;

typedef enum {
    FALL_EVENT_NONE = 0,
    FALL_EVENT_SUSPECTED,
    FALL_EVENT_CONFIRMED,
    FALL_EVENT_RECOVERED,
} FallEventState_e;

typedef enum {
    FALL_REASON_NONE = 0,
    FALL_REASON_TERMINAL_PERSISTED,
    FALL_REASON_LOW_STILL,
    FALL_REASON_LOW_ACTIVE,
    FALL_REASON_SUSPECTED_EXPIRED,
    FALL_REASON_STOOD_UP,
} FallEventReason_e;

typedef struct {
    FallEventState_e state;
    FallEventReason_e reason;
    uint32_t frame_index;
    uint32_t timestamp_ms;
    int16_t drop_height_cm;
    int16_t center_z_cm;
} FallEvent_t;

typedef struct {
    uint8_t count;
    FallEvent_t events[FALL_EVENT_BATCH_MAX];
} FallEventBatch_t;

typedef struct {
    uint32_t drop_window_ms;
    uint32_t confirm_window_ms;
    int16_t min_drop_cm;
    int16_t motion_velocity_threshold_cm_s;
    uint16_t motion_point_threshold;
    int16_t candidate_low_height_cm;
    int16_t terminal_height_cm;
    uint32_t suspect_low_duration_ms;
    uint16_t suspect_min_fresh_samples;
    uint8_t suspect_low_ratio_percent;
    uint32_t confirm_low_duration_ms;
    uint16_t confirm_min_fresh_samples;
    uint8_t confirm_low_ratio_percent;
    uint16_t confirm_min_horizontal_displacement_cm;
    uint16_t fresh_min_point_count;
    int16_t still_velocity_threshold_cm_s;
    uint16_t still_motion_points_max;
    int16_t rearm_height_cm;
    uint32_t rearm_duration_ms;
    uint16_t rearm_max_horizontal_displacement_cm;
    uint16_t rearm_origin_max_horizontal_displacement_cm;
    uint32_t candidate_hold_tolerance_ms;
} FallRuleConfig_t;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t frame_index;
    int16_t center_x_cm;
    int16_t center_y_cm;
    int16_t center_z_cm;
    uint16_t motion_count;
    int16_t max_abs_velocity_cm_s;
} FallRuleRecentSample_t;

typedef struct {
    uint32_t timestamp_ms;
    int16_t center_z_cm;
} FallRuleHeightSample_t;

typedef struct {
    FallRuleConfig_t config;
    FallRuleRecentSample_t recent[FALL_HISTORY_MAX_SAMPLES];
    FallRuleHeightSample_t height_history[FALL_HISTORY_MAX_SAMPLES];
    FallRuleHeightSample_t fresh_history[FALL_HISTORY_MAX_SAMPLES];
    uint16_t recent_head;
    uint16_t recent_count;
    uint16_t height_head;
    uint16_t height_count;
    uint16_t fresh_head;
    uint16_t fresh_count;
    uint8_t recent_incomplete;
    uint8_t height_incomplete;
    uint8_t fresh_incomplete;
    uint32_t last_fresh_timestamp_ms;
    uint8_t candidate_active;
    uint32_t candidate_frame_index;
    uint32_t candidate_timestamp_ms;
    int16_t candidate_drop_cm;
    int16_t candidate_anchor_x_cm;
    int16_t candidate_anchor_y_cm;
    uint32_t candidate_max_horizontal_displacement_sq_cm;
    uint8_t suspected_active;
    uint32_t suspected_frame_index;
    uint32_t suspected_timestamp_ms;
    int16_t suspected_drop_cm;
    uint8_t confirmed;
    uint32_t confirmed_timestamp_ms;
    int16_t confirmed_center_x_cm;
    int16_t confirmed_center_y_cm;
    int16_t confirmed_low_z_cm;
    uint8_t rearm_active;
    uint32_t rearm_started_ms;
    uint8_t candidate_rearm_active;
    uint32_t candidate_rearm_started_ms;
    FallState_e state;
    uint8_t current_fresh;
    uint8_t current_terminal_low;
    uint16_t diagnostic_fresh_count;
    uint8_t diagnostic_low_ratio_percent;
} FallRuleRuntime_t;

void fall_rule_init(FallRuleRuntime_t *runtime, const FallRuleConfig_t *config);
FallEventBatch_t fall_rule_step(FallRuleRuntime_t *runtime, const FallTarget_t *target);
FallState_e fall_rule_state_get(const FallRuleRuntime_t *runtime);

#endif
