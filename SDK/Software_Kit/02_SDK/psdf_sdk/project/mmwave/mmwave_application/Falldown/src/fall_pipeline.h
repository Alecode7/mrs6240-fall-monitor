#ifndef FALL_PIPELINE_H
#define FALL_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

#include "prj_config.h"

typedef enum {
    FALL_TRACK_LOST = 0,
    FALL_TRACK_TRACKING,
    FALL_TRACK_STATIC_LOCK,
    FALL_TRACK_HOLD,
    FALL_TRACK_REACQUIRE,
} FallTrackState_e;

typedef struct {
    float x_cm;
    float y_cm;
    float z_cm;
    int16_t velocity_cm_s;
    int16_t snr_db;
    uint8_t is_motion;
} FallInputPoint_t;

typedef struct {
    int16_t x_cm;
    int16_t y_cm;
    int16_t z_cm;
    int16_t velocity_cm_s;
    int16_t snr_db;
    uint8_t is_motion;
} FallPoint_t;

typedef struct {
    uint16_t start;
    uint16_t count;
    uint16_t point_count;
    uint16_t motion_count;
    int16_t center_x_cm;
    int16_t center_y_cm;
    int16_t center_z_cm;
    int16_t x_min_cm;
    int16_t x_max_cm;
    int16_t y_min_cm;
    int16_t y_max_cm;
    int16_t z_min_cm;
    int16_t z_max_cm;
    int16_t max_abs_velocity_cm_s;
} FallCluster_t;

typedef struct {
    uint8_t has_target;
    FallTrackState_e state;
    int16_t center_x_cm;
    int16_t center_y_cm;
    int16_t center_z_cm;
    int16_t z_min_cm;
    int16_t z_max_cm;
    int16_t height_cm;
    int16_t width_x_cm;
    int16_t depth_y_cm;
    uint16_t point_count;
    uint16_t motion_count;
    int16_t max_abs_velocity_cm_s;
    uint32_t frame_index;
    uint32_t timestamp_ms;
} FallTarget_t;

typedef struct {
    int16_t mount_height_cm;
    int16_t mount_pitch_deg;
    int16_t roi_x_min_cm;
    int16_t roi_x_max_cm;
    int16_t roi_y_min_cm;
    int16_t roi_y_max_cm;
    int16_t roi_z_min_cm;
    int16_t roi_z_max_cm;
    int16_t min_snr_db;
    int16_t cluster_radius_cm;
    uint16_t min_cluster_points;
    int16_t max_cluster_width_cm;
    int16_t max_cluster_depth_cm;
    int16_t track_gate_cm;
    uint32_t hold_ms;
    uint8_t switch_confirm_frames;
    uint16_t static_body_min_points;
} FallPipelineConfig_t;

typedef struct {
    uint8_t active;
    int16_t center_x_cm;
    int16_t center_y_cm;
    uint8_t frames;
    FallCluster_t cluster;
} FallPendingSwitch_t;

typedef struct {
    FallPipelineConfig_t config;
    float pitch_cos;
    float pitch_sin;
    FallPoint_t points[FALL_MAX_POINTS];
    uint8_t assigned[FALL_MAX_POINTS];
    uint16_t queue[FALL_MAX_POINTS];
    uint16_t cluster_point_indices[FALL_MAX_POINTS];
    FallCluster_t clusters[FALL_MAX_CLUSTERS];
    int16_t sort_scratch[FALL_MAX_POINTS];
    uint16_t point_count;
    uint16_t cluster_count;
    uint32_t frame_index;
    uint32_t timestamp_ms;
    uint32_t point_overflow_count;
    uint32_t cluster_overflow_count;
    FallTarget_t last_target;
    uint32_t last_target_timestamp_ms;
    uint8_t has_last_target;
    FallPendingSwitch_t pending_switch;
} FallPipelineRuntime_t;

void fall_pipeline_init(FallPipelineRuntime_t *runtime, const FallPipelineConfig_t *config);
void fall_pipeline_begin_frame(
    FallPipelineRuntime_t *runtime,
    uint32_t frame_index,
    uint32_t timestamp_ms);
bool fall_pipeline_add_point(FallPipelineRuntime_t *runtime, const FallInputPoint_t *point);
FallTarget_t fall_pipeline_finish_frame(FallPipelineRuntime_t *runtime);
uint16_t fall_pipeline_cluster_count_get(const FallPipelineRuntime_t *runtime);
uint32_t fall_pipeline_overflow_count_get(const FallPipelineRuntime_t *runtime);

#endif
