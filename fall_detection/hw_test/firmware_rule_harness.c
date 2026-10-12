#include <stdint.h>
#include <string.h>

#include "fall_pipeline.h"
#include "fall_rule.h"

#ifdef _WIN32
#define HW_EXPORT __declspec(dllexport)
#else
#define HW_EXPORT
#endif

typedef struct {
    float x_m;
    float y_m;
    float z_m;
    float velocity_m_s;
    float snr_db;
    int32_t is_motion;
} HwInputPoint_t;

typedef struct {
    int32_t has_target;
    int32_t track_state;
    int32_t center_x_cm;
    int32_t center_y_cm;
    int32_t center_z_cm;
    int32_t height_cm;
    int32_t point_count;
    int32_t motion_count;
    int32_t max_abs_velocity_cm_s;
    uint32_t frame_index;
    uint32_t timestamp_ms;
    uint32_t accepted_point_count;
    uint32_t overflow_count;
    uint32_t cluster_count;
    uint32_t event_count;
    int32_t event_state[FALL_EVENT_BATCH_MAX];
    int32_t event_reason[FALL_EVENT_BATCH_MAX];
    int32_t event_drop_cm[FALL_EVENT_BATCH_MAX];
    int32_t event_center_z_cm[FALL_EVENT_BATCH_MAX];
    uint32_t event_frame_index[FALL_EVENT_BATCH_MAX];
    uint32_t event_timestamp_ms[FALL_EVENT_BATCH_MAX];
} HwFrameResult_t;

typedef struct {
    int32_t frame_period_ms;
    int32_t mount_height_cm;
    int32_t mount_pitch_deg;
    int32_t roi_x_min_cm;
    int32_t roi_x_max_cm;
    int32_t roi_y_min_cm;
    int32_t roi_y_max_cm;
    int32_t roi_z_min_cm;
    int32_t roi_z_max_cm;
    int32_t min_snr_db;
    int32_t cluster_radius_cm;
    int32_t min_cluster_points;
    int32_t max_cluster_width_cm;
    int32_t max_cluster_depth_cm;
    int32_t track_gate_cm;
    int32_t hold_ms;
    int32_t switch_confirm_frames;
    int32_t drop_window_ms;
    int32_t confirm_window_ms;
    int32_t min_drop_cm;
    int32_t motion_velocity_threshold_cm_s;
    int32_t motion_point_threshold;
    int32_t candidate_low_height_cm;
    int32_t terminal_height_cm;
    int32_t suspect_low_duration_ms;
    int32_t suspect_min_fresh_samples;
    int32_t suspect_low_ratio_percent;
    int32_t confirm_low_duration_ms;
    int32_t confirm_min_fresh_samples;
    int32_t confirm_low_ratio_percent;
    int32_t confirm_min_horizontal_displacement_cm;
    int32_t fresh_min_point_count;
    int32_t still_velocity_cm_s;
    int32_t still_motion_points_max;
    int32_t rearm_height_cm;
    int32_t rearm_duration_ms;
} HwConfigSnapshot_t;

static FallPipelineRuntime_t s_pipeline;
static FallRuleRuntime_t s_rule;

static const FallPipelineConfig_t s_pipeline_config = {
    .mount_height_cm = FALL_MOUNT_HEIGHT_CM,
    .mount_pitch_deg = FALL_MOUNT_PITCH_DEG,
    .roi_x_min_cm = FALL_ROI_X_MIN_CM,
    .roi_x_max_cm = FALL_ROI_X_MAX_CM,
    .roi_y_min_cm = FALL_ROI_Y_MIN_CM,
    .roi_y_max_cm = FALL_ROI_Y_MAX_CM,
    .roi_z_min_cm = FALL_ROI_Z_MIN_CM,
    .roi_z_max_cm = FALL_ROI_Z_MAX_CM,
    .min_snr_db = FALL_MIN_SNR_DB,
    .cluster_radius_cm = FALL_CLUSTER_RADIUS_CM,
    .min_cluster_points = FALL_MIN_CLUSTER_POINTS,
    .max_cluster_width_cm = FALL_MAX_CLUSTER_WIDTH_CM,
    .max_cluster_depth_cm = FALL_MAX_CLUSTER_DEPTH_CM,
    .track_gate_cm = FALL_TRACK_GATE_CM,
    .hold_ms = FALL_HOLD_MS,
    .switch_confirm_frames = FALL_SWITCH_CONFIRM_FRAMES,
    .static_body_min_points = FALL_STATIC_BODY_MIN_POINTS,
};

static const FallRuleConfig_t s_rule_config = {
    .drop_window_ms = FALL_DROP_WINDOW_MS,
    .confirm_window_ms = FALL_CONFIRM_WINDOW_MS,
    .min_drop_cm = FALL_MIN_DROP_CM,
    .motion_velocity_threshold_cm_s = FALL_MOTION_VELOCITY_TRIGGER_CM_S,
    .motion_point_threshold = FALL_MOTION_POINT_THRESHOLD,
    .candidate_low_height_cm = FALL_CANDIDATE_LOW_HEIGHT_CM,
    .terminal_height_cm = FALL_TERMINAL_HEIGHT_CM,
    .suspect_low_duration_ms = FALL_SUSPECT_LOW_DURATION_MS,
    .suspect_min_fresh_samples = FALL_SUSPECT_MIN_FRESH_SAMPLES,
    .suspect_low_ratio_percent = FALL_SUSPECT_LOW_RATIO_PERCENT,
    .confirm_low_duration_ms = FALL_CONFIRM_LOW_DURATION_MS,
    .confirm_min_fresh_samples = FALL_CONFIRM_MIN_FRESH_SAMPLES,
    .confirm_low_ratio_percent = FALL_CONFIRM_LOW_RATIO_PERCENT,
    .confirm_min_horizontal_displacement_cm = FALL_CONFIRM_MIN_HORIZONTAL_DISPLACEMENT_CM,
    .fresh_min_point_count = FALL_FRESH_MIN_POINT_COUNT,
    .still_velocity_threshold_cm_s = FALL_STILL_VELOCITY_CM_S,
    .still_motion_points_max = FALL_STILL_MOTION_POINTS_MAX,
    .rearm_height_cm = FALL_REARM_HEIGHT_CM,
    .rearm_duration_ms = FALL_REARM_DURATION_MS,
    .rearm_max_horizontal_displacement_cm = FALL_REARM_TRACK_GATE_CM,
    .candidate_hold_tolerance_ms = FALL_CANDIDATE_HOLD_TOLERANCE_MS,
};

static int16_t hw_round_i16(float value)
{
    if (value >= 32767.0f) {
        return 32767;
    }
    if (value <= -32768.0f) {
        return (int16_t)-32768;
    }
    return (value >= 0.0f) ? (int16_t)(value + 0.5f) : (int16_t)(value - 0.5f);
}

HW_EXPORT void hw_fall_reset(void)
{
    fall_pipeline_init(&s_pipeline, &s_pipeline_config);
    fall_rule_init(&s_rule, &s_rule_config);
}

HW_EXPORT void hw_fall_get_config(HwConfigSnapshot_t *output)
{
    if (output == NULL) {
        return;
    }
    memset(output, 0, sizeof(*output));
    output->frame_period_ms = RADAR_FRAMEWORK_SAMPLE_FRAME_PERIOD_MS;
    output->mount_height_cm = FALL_MOUNT_HEIGHT_CM;
    output->mount_pitch_deg = FALL_MOUNT_PITCH_DEG;
    output->roi_x_min_cm = FALL_ROI_X_MIN_CM;
    output->roi_x_max_cm = FALL_ROI_X_MAX_CM;
    output->roi_y_min_cm = FALL_ROI_Y_MIN_CM;
    output->roi_y_max_cm = FALL_ROI_Y_MAX_CM;
    output->roi_z_min_cm = FALL_ROI_Z_MIN_CM;
    output->roi_z_max_cm = FALL_ROI_Z_MAX_CM;
    output->min_snr_db = FALL_MIN_SNR_DB;
    output->cluster_radius_cm = FALL_CLUSTER_RADIUS_CM;
    output->min_cluster_points = FALL_MIN_CLUSTER_POINTS;
    output->max_cluster_width_cm = FALL_MAX_CLUSTER_WIDTH_CM;
    output->max_cluster_depth_cm = FALL_MAX_CLUSTER_DEPTH_CM;
    output->track_gate_cm = FALL_TRACK_GATE_CM;
    output->hold_ms = FALL_HOLD_MS;
    output->switch_confirm_frames = FALL_SWITCH_CONFIRM_FRAMES;
    output->drop_window_ms = FALL_DROP_WINDOW_MS;
    output->confirm_window_ms = FALL_CONFIRM_WINDOW_MS;
    output->min_drop_cm = FALL_MIN_DROP_CM;
    output->motion_velocity_threshold_cm_s = FALL_MOTION_VELOCITY_TRIGGER_CM_S;
    output->motion_point_threshold = FALL_MOTION_POINT_THRESHOLD;
    output->candidate_low_height_cm = FALL_CANDIDATE_LOW_HEIGHT_CM;
    output->terminal_height_cm = FALL_TERMINAL_HEIGHT_CM;
    output->suspect_low_duration_ms = FALL_SUSPECT_LOW_DURATION_MS;
    output->suspect_min_fresh_samples = FALL_SUSPECT_MIN_FRESH_SAMPLES;
    output->suspect_low_ratio_percent = FALL_SUSPECT_LOW_RATIO_PERCENT;
    output->confirm_low_duration_ms = FALL_CONFIRM_LOW_DURATION_MS;
    output->confirm_min_fresh_samples = FALL_CONFIRM_MIN_FRESH_SAMPLES;
    output->confirm_low_ratio_percent = FALL_CONFIRM_LOW_RATIO_PERCENT;
    output->confirm_min_horizontal_displacement_cm = FALL_CONFIRM_MIN_HORIZONTAL_DISPLACEMENT_CM;
    output->fresh_min_point_count = FALL_FRESH_MIN_POINT_COUNT;
    output->still_velocity_cm_s = FALL_STILL_VELOCITY_CM_S;
    output->still_motion_points_max = FALL_STILL_MOTION_POINTS_MAX;
    output->rearm_height_cm = FALL_REARM_HEIGHT_CM;
    output->rearm_duration_ms = FALL_REARM_DURATION_MS;
}

HW_EXPORT int32_t hw_fall_process_frame(
    uint32_t frame_index,
    const HwInputPoint_t *inputs,
    uint32_t input_count,
    HwFrameResult_t *output)
{
    FallTarget_t target;
    FallEventBatch_t events;
    uint32_t index;

    if (output == NULL) {
        return -1;
    }
    memset(output, 0, sizeof(*output));
    fall_pipeline_begin_frame(
        &s_pipeline,
        frame_index,
        frame_index * RADAR_FRAMEWORK_SAMPLE_FRAME_PERIOD_MS);
    for (index = 0U; index < input_count; index++) {
        FallInputPoint_t point;
        point.x_cm = inputs[index].x_m * 100.0f;
        point.y_cm = inputs[index].y_m * 100.0f;
        point.z_cm = inputs[index].z_m * 100.0f;
        point.velocity_cm_s = hw_round_i16(inputs[index].velocity_m_s * 100.0f);
        point.snr_db = hw_round_i16(inputs[index].snr_db);
        point.is_motion = (inputs[index].is_motion != 0) ? 1U : 0U;
        (void)fall_pipeline_add_point(&s_pipeline, &point);
    }

    target = fall_pipeline_finish_frame(&s_pipeline);
    events = fall_rule_step(&s_rule, &target);

    output->has_target = target.has_target;
    output->track_state = target.state;
    output->center_x_cm = target.center_x_cm;
    output->center_y_cm = target.center_y_cm;
    output->center_z_cm = target.center_z_cm;
    output->height_cm = target.height_cm;
    output->point_count = target.point_count;
    output->motion_count = target.motion_count;
    output->max_abs_velocity_cm_s = target.max_abs_velocity_cm_s;
    output->frame_index = target.frame_index;
    output->timestamp_ms = target.timestamp_ms;
    output->accepted_point_count = s_pipeline.point_count;
    output->overflow_count = fall_pipeline_overflow_count_get(&s_pipeline);
    output->cluster_count = fall_pipeline_cluster_count_get(&s_pipeline);
    output->event_count = events.count;
    for (index = 0U; index < events.count; index++) {
        output->event_state[index] = events.events[index].state;
        output->event_reason[index] = events.events[index].reason;
        output->event_drop_cm[index] = events.events[index].drop_height_cm;
        output->event_center_z_cm[index] = events.events[index].center_z_cm;
        output->event_frame_index[index] = events.events[index].frame_index;
        output->event_timestamp_ms[index] = events.events[index].timestamp_ms;
    }
    return 0;
}
