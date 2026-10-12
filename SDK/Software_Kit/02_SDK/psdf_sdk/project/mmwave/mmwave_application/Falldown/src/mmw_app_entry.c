/**
 ******************************************************************************
 * @file    applications.c
 * @brief   Developer callback functions called from radar_framework.
 * @verbatim    null
 ******************************************************************************
 * @attention
 *
 * Copyright (C) 2025 POSSUMIC TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of POSSUMIC TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "fall_pipeline.h"
#include "fall_rule.h"
#include "log.h"
#include "math.h"
#include "mmw_app_micro_pointcloud.h"
#include "mmw_app_pointcloud.h"
#include "mmw_app_pointcloud_config.h"
#include "mmw_ctrl.h"
#include "mmw_type.h"
#include "prj_config.h"
#include "kvf.h"
#include "shell.h"
#include <stdlib.h>

static FallPipelineRuntime_t s_fall_pipeline;
static FallRuleRuntime_t s_fall_rule;
static uint8_t s_frame_period_warning_printed;

typedef struct {
    uint32_t magic;
    int16_t mount_height_cm;
    uint16_t reserved;
} FallMountConfigStorage_t;

static FallPipelineConfig_t s_pipeline_config = {
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

static uint8_t fall_mount_height_is_valid(int32_t height_cm)
{
    return (height_cm >= FALL_MOUNT_HEIGHT_MIN_CM) &&
           (height_cm <= FALL_MOUNT_HEIGHT_MAX_CM);
}

static void fall_mount_height_load(void)
{
    FallMountConfigStorage_t storage;
    int ret;

    ret = KVF_Load(FALL_MOUNT_CONFIG_KVF_ID, &storage, sizeof(storage));
    if ((ret == (int)sizeof(storage)) &&
        (storage.magic == FALL_MOUNT_CONFIG_MAGIC) &&
        fall_mount_height_is_valid(storage.mount_height_cm)) {
        s_pipeline_config.mount_height_cm = storage.mount_height_cm;
        LOG_PRINT("FALL_HEIGHT loaded=%dcm\n", storage.mount_height_cm);
    } else {
        s_pipeline_config.mount_height_cm = FALL_MOUNT_HEIGHT_CM;
        LOG_PRINT("FALL_HEIGHT default=%dcm\n", FALL_MOUNT_HEIGHT_CM);
    }
}

static int fall_mount_height_save(int16_t height_cm)
{
    FallMountConfigStorage_t storage;
    FallMountConfigStorage_t verify;
    int ret;

    storage.magic = FALL_MOUNT_CONFIG_MAGIC;
    storage.mount_height_cm = height_cm;
    storage.reserved = 0U;
    ret = KVF_Save(FALL_MOUNT_CONFIG_KVF_ID, &storage, sizeof(storage));
    if (ret != (int)sizeof(storage)) {
        return -1;
    }
    ret = KVF_Load(FALL_MOUNT_CONFIG_KVF_ID, &verify, sizeof(verify));
    if ((ret != (int)sizeof(verify)) ||
        (verify.magic != storage.magic) ||
        (verify.mount_height_cm != storage.mount_height_cm)) {
        return -2;
    }
    return 0;
}

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
    .rearm_origin_max_horizontal_displacement_cm = FALL_REARM_ORIGIN_GATE_CM,
    .candidate_hold_tolerance_ms = FALL_CANDIDATE_HOLD_TOLERANCE_MS,
};

static void fall_config_print(Shell *shell)
{
    shellPrint(
        shell,
        "FALL_CFG alg=3 mount_h=%d pitch=%d roi_x=%d..%d roi_y=%d..%d roi_z=%d..%d terminal=%d suspect_ms=%u suspect_ratio=%u confirm_ms=%u confirm_xy=%u acquire_pts=%u rearm_xy=%u candidate_hold_ms=%u\n",
        s_pipeline_config.mount_height_cm,
        FALL_MOUNT_PITCH_DEG,
        FALL_ROI_X_MIN_CM,
        FALL_ROI_X_MAX_CM,
        FALL_ROI_Y_MIN_CM,
        FALL_ROI_Y_MAX_CM,
        FALL_ROI_Z_MIN_CM,
        FALL_ROI_Z_MAX_CM,
        FALL_TERMINAL_HEIGHT_CM,
        (uint32_t)FALL_SUSPECT_LOW_DURATION_MS,
        (uint32_t)FALL_SUSPECT_LOW_RATIO_PERCENT,
        (uint32_t)FALL_CONFIRM_LOW_DURATION_MS,
        (uint32_t)FALL_CONFIRM_MIN_HORIZONTAL_DISPLACEMENT_CM,
        (uint32_t)FALL_ACQUIRE_MIN_POINTS,
        (uint32_t)FALL_REARM_TRACK_GATE_CM,
        (uint32_t)FALL_CANDIDATE_HOLD_TOLERANCE_MS);
}

static int fall_config_cmd(Shell *shell, int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        shellPrint(shell, "usage: fall_config\n");
        return -1;
    }
    fall_config_print(shell);
    return 0;
}

SHELL_CMD(fall_config, fall_config_cmd, "Read active fall configuration");

static int fall_height_cmd(Shell *shell, int argc, char *argv[])
{
    char *end;
    long value;
    uint32_t frame_sent = 0U;
    int state;
    int16_t height_cm;

    if (argc == 1) {
        shellPrint(shell, "FALL_HEIGHT %dcm\n", s_pipeline_config.mount_height_cm);
        return 0;
    }
    if (argc != 2) {
        shellPrint(shell, "usage: fall_height <cm>\n");
        return -1;
    }

    value = strtol(argv[1], &end, 10);
    if ((*argv[1] == '\0') || (*end != '\0') ||
        !fall_mount_height_is_valid((int32_t)value)) {
        shellPrint(shell, "FALL_HEIGHT invalid, range=%d..%dcm\n",
                   FALL_MOUNT_HEIGHT_MIN_CM,
                   FALL_MOUNT_HEIGHT_MAX_CM);
        return -2;
    }

    state = mmw_state(&frame_sent);
    if (state == MMW_STATE_RUNNING) {
        shellPrint(shell, "FALL_HEIGHT stop radar before changing\n");
        return -3;
    }

    height_cm = (int16_t)value;
    if (fall_mount_height_save(height_cm) != 0) {
        shellPrint(shell, "FALL_HEIGHT save failed\n");
        return -4;
    }

    s_pipeline_config.mount_height_cm = height_cm;
    fall_pipeline_init(&s_fall_pipeline, &s_pipeline_config);
    fall_rule_init(&s_fall_rule, &s_rule_config);
    shellPrint(shell, "FALL_HEIGHT set=%dcm saved=1\n", height_cm);
    return 0;
}

SHELL_CMD(fall_height, fall_height_cmd, "<cm> Set installation height");

static int16_t fall_round_i16(float value)
{
    if (value >= 32767.0f) {
        return 32767;
    }
    if (value <= -32768.0f) {
        return (int16_t)-32768;
    }
    return (value >= 0.0f) ? (int16_t)(value + 0.5f) : (int16_t)(value - 0.5f);
}

static void fall_motion_point_to_input(
    const MmwClusterDetectData_t *source,
    float range_bin_size_cm,
    float doppler_bin_size_cm,
    int16_t doppler_index_half,
    FallInputPoint_t *point)
{
    float sin_x = source->azi_phase;
    float sin_z = source->ele_phase;
    float sin_y_squared = 1.0f - sin_x * sin_x - sin_z * sin_z;
    float user_coordinate[3] = {0.0f, 0.0f, 0.0f};

    if (sin_y_squared < 0.0f) {
        sin_y_squared = 0.0f;
    }
    mmw_point_cloud_trans_radar_coord_to_user_coord(
        source->range_idx * sin_x * range_bin_size_cm,
        source->range_idx * sqrtf(sin_y_squared) * range_bin_size_cm,
        source->range_idx * sin_z * range_bin_size_cm,
        user_coordinate);

    point->x_cm = user_coordinate[0];
    point->y_cm = user_coordinate[1];
    point->z_cm = user_coordinate[2];
    point->velocity_cm_s = fall_round_i16(
        (source->doppler_idx - doppler_index_half) * doppler_bin_size_cm);
    point->snr_db = fall_round_i16(source->sig_snr);
    point->is_motion = 1U;
}

static void fall_presence_point_to_input(
    const MmwMicroDetectData_t *source,
    float range_bin_size_cm,
    float micro_velocity_unit_cm,
    int16_t doppler_index_half,
    FallInputPoint_t *point)
{
    float sin_x = source->azi_phase;
    float sin_z = source->ele_phase;
    float sin_y_squared = 1.0f - sin_x * sin_x - sin_z * sin_z;
    float user_coordinate[3] = {0.0f, 0.0f, 0.0f};

    if (sin_y_squared < 0.0f) {
        sin_y_squared = 0.0f;
    }
    mmw_point_cloud_trans_radar_coord_to_user_coord(
        source->range_idx * sin_x * range_bin_size_cm,
        source->range_idx * sqrtf(sin_y_squared) * range_bin_size_cm,
        source->range_idx * sin_z * range_bin_size_cm,
        user_coordinate);

    point->x_cm = user_coordinate[0];
    point->y_cm = user_coordinate[1];
    point->z_cm = user_coordinate[2];
    point->velocity_cm_s = fall_round_i16(
        (source->doppler_idx - doppler_index_half) * micro_velocity_unit_cm);
    point->snr_db = source->sig_snr;
    point->is_motion = 0U;
}

static uint16_t fall_submit_points(
    const PointCloudBuffer_t *motion_buffer,
    const PresencePointCloudBuffer_t *presence_buffer,
    uint32_t frame_period_ms)
{
    uint32_t range_mm = 0U;
    uint32_t range_resolution_mm = 0U;
    uint32_t velocity_mm = 0U;
    uint32_t velocity_resolution_mm = 0U;
    uint16_t range_fft_num = 1U;
    uint16_t doppler_fft_num = 1U;
    uint32_t micro_doppler_num;
    uint8_t micro_frame_rate;
    uint32_t micro_frame_period_ms;
    float range_bin_size_cm;
    float doppler_bin_size_cm;
    float micro_velocity_unit_cm;
    int16_t doppler_index_half;
    int16_t micro_doppler_index_half;
    uint16_t raw_point_count = 0U;
    uint16_t index;
    FallInputPoint_t point;

    mmw_range_get(&range_mm, &range_resolution_mm);
    mmw_velocity_get(&velocity_mm, &velocity_resolution_mm);
    mmw_fft_num_get(&range_fft_num, &doppler_fft_num);
    if (range_fft_num == 0U) {
        range_fft_num = 1U;
    }
    if (doppler_fft_num == 0U) {
        doppler_fft_num = 1U;
    }
    range_bin_size_cm = (float)range_mm / range_fft_num * 0.1f;
    doppler_bin_size_cm = (float)velocity_mm / doppler_fft_num * 0.2f;
    doppler_index_half = (int16_t)(doppler_fft_num >> 1U);

    micro_doppler_num = mmw_micro_doppler_num_get();
    if (micro_doppler_num == 0U) {
        micro_doppler_num = 1U;
    }
    micro_frame_rate = mmw_micro_frame_rate_get();
    if (micro_frame_rate == 0U) {
        micro_frame_rate = 1U;
    }
    micro_frame_period_ms = frame_period_ms * micro_frame_rate;
    if (micro_frame_period_ms == 0U) {
        micro_frame_period_ms = RADAR_FRAMEWORK_SAMPLE_FRAME_PERIOD_MS;
    }
    micro_velocity_unit_cm =
        500.0f / ((float)micro_frame_period_ms * (float)micro_doppler_num);
    micro_doppler_index_half = (int16_t)(micro_doppler_num >> 1U);

    if ((motion_buffer != NULL) &&
        (motion_buffer->ptr_motion_point_cloud_data != NULL)) {
        for (index = 0U; index < motion_buffer->point_cloud_num; index++) {
            fall_motion_point_to_input(
                &motion_buffer->ptr_motion_point_cloud_data[index],
                range_bin_size_cm,
                doppler_bin_size_cm,
                doppler_index_half,
                &point);
            fall_pipeline_add_point(&s_fall_pipeline, &point);
            raw_point_count++;
        }
    }

    if ((presence_buffer != NULL) &&
        (presence_buffer->ptr_presence_point_cloud_data != NULL)) {
        for (index = 0U; index < presence_buffer->presence_point_cloud_num; index++) {
            fall_presence_point_to_input(
                &presence_buffer->ptr_presence_point_cloud_data[index],
                range_bin_size_cm,
                micro_velocity_unit_cm,
                micro_doppler_index_half,
                &point);
            fall_pipeline_add_point(&s_fall_pipeline, &point);
            raw_point_count++;
        }
    }
    return raw_point_count;
}

static const char *fall_event_reason_text(FallEventReason_e reason)
{
    switch (reason) {
    case FALL_REASON_TERMINAL_PERSISTED:
        return "terminal_persisted";
    case FALL_REASON_LOW_STILL:
        return "low_still";
    case FALL_REASON_LOW_ACTIVE:
        return "low_active";
    case FALL_REASON_SUSPECTED_EXPIRED:
        return "suspected_expired";
    case FALL_REASON_STOOD_UP:
        return "stood_up";
    default:
        return "none";
    }
}

static void fall_print_events(const FallEventBatch_t *batch)
{
    uint8_t index;

    for (index = 0U; index < batch->count; index++) {
        const FallEvent_t *event = &batch->events[index];
        if (event->state == FALL_EVENT_SUSPECTED) {
            LOG_PRINT(
                "FALL_EVENT state=suspected frame=%u t=%u drop_cm=%d z=%d reason=%s\n",
                event->frame_index,
                event->timestamp_ms,
                event->drop_height_cm,
                event->center_z_cm,
                fall_event_reason_text(event->reason));
        } else if (event->state == FALL_EVENT_CONFIRMED) {
            LOG_PRINT(
                "FALL_EVENT state=confirmed frame=%u t=%u start_frame=%u drop_cm=%d z=%d reason=%s\n",
                event->frame_index,
                event->timestamp_ms,
                s_fall_rule.suspected_frame_index,
                event->drop_height_cm,
                event->center_z_cm,
                fall_event_reason_text(event->reason));
        } else if (event->state == FALL_EVENT_RECOVERED) {
            LOG_PRINT(
                "FALL_EVENT state=recovered frame=%u t=%u drop_cm=%d z=%d reason=%s\n",
                event->frame_index,
                event->timestamp_ms,
                event->drop_height_cm,
                event->center_z_cm,
                fall_event_reason_text(event->reason));
        }
    }
}

static void fall_print_status(
    uint32_t frame_index,
    const FallTarget_t *target,
    uint16_t raw_point_count)
{
    uint16_t cluster_count;
    uint32_t overflow_count;

    if ((frame_index % FALL_STATUS_PERIOD_FRAMES) != 0U) {
        return;
    }
    cluster_count = fall_pipeline_cluster_count_get(&s_fall_pipeline);
    overflow_count = fall_pipeline_overflow_count_get(&s_fall_pipeline);
    if (target->has_target == 0U) {
        LOG_PRINT(
            "FALL frame=%u state=%d track=%d target=0 raw=%u clusters=%u fresh=0 low=0 overflow=%u\n",
            frame_index,
            fall_rule_state_get(&s_fall_rule),
            target->state,
            raw_point_count,
            cluster_count,
            overflow_count);
        return;
    }

    LOG_PRINT(
        "FALL frame=%u state=%d track=%d target=1 x=%d y=%d z=%d h=%d pts=%u mot=%u v=%d raw=%u clusters=%u fresh=%u low=%u overflow=%u\n",
        frame_index,
        fall_rule_state_get(&s_fall_rule),
        target->state,
        target->center_x_cm,
        target->center_y_cm,
        target->center_z_cm,
        target->height_cm,
        target->point_count,
        target->motion_count,
        target->max_abs_velocity_cm_s,
        raw_point_count,
        cluster_count,
        s_fall_rule.current_fresh,
        s_fall_rule.current_terminal_low,
        overflow_count);
}

__sram_text int mmw_frame_cb(uint8_t argc, void *arg[])
{
    uint32_t frame_index;
    uint32_t frame_period_ms = RADAR_FRAMEWORK_SAMPLE_FRAME_PERIOD_MS;
    uint32_t frame_num = 0U;
    uint32_t timestamp_ms;
    PointCloudBuffer_t *motion_buffer;
    PresencePointCloudBuffer_t *presence_buffer;
    uint16_t raw_point_count;
    FallTarget_t target;
    FallEventBatch_t events;

    if ((argc < 3U) || (arg == NULL) || (arg[0] == NULL)) {
        return 0;
    }

    frame_index = *((uint32_t *)arg[0]);
    motion_buffer = (PointCloudBuffer_t *)arg[1];
    presence_buffer = (PresencePointCloudBuffer_t *)arg[2];
    if ((mmw_frame_get(&frame_period_ms, &frame_num) != 0) ||
        (frame_period_ms == 0U)) {
        frame_period_ms = RADAR_FRAMEWORK_SAMPLE_FRAME_PERIOD_MS;
        if (s_frame_period_warning_printed == 0U) {
            LOG_PRINT("FALL_WARN frame_period_read_failed fallback_ms=%u\n", frame_period_ms);
            s_frame_period_warning_printed = 1U;
        }
    }
    timestamp_ms = frame_index * frame_period_ms;

    fall_pipeline_begin_frame(&s_fall_pipeline, frame_index, timestamp_ms);
    raw_point_count = fall_submit_points(
        motion_buffer,
        presence_buffer,
        frame_period_ms);
    target = fall_pipeline_finish_frame(&s_fall_pipeline);
    events = fall_rule_step(&s_fall_rule, &target);
    fall_print_events(&events);
    fall_print_status(frame_index, &target, raw_point_count);
    return 0;
}

int app_init_cb(void)
{
    fall_mount_height_load();
    fall_pipeline_init(&s_fall_pipeline, &s_pipeline_config);
    fall_rule_init(&s_fall_rule, &s_rule_config);
    s_frame_period_warning_printed = 0U;
    LOG_PRINT(
        "FALL_CFG alg=3 mount_h=%d pitch=%d roi_x=%d..%d roi_y=%d..%d roi_z=%d..%d terminal=%d suspect_ms=%u suspect_ratio=%u confirm_ms=%u confirm_xy=%u acquire_pts=%u rearm_xy=%u candidate_hold_ms=%u\n",
        s_pipeline_config.mount_height_cm,
        FALL_MOUNT_PITCH_DEG,
        FALL_ROI_X_MIN_CM,
        FALL_ROI_X_MAX_CM,
        FALL_ROI_Y_MIN_CM,
        FALL_ROI_Y_MAX_CM,
        FALL_ROI_Z_MIN_CM,
        FALL_ROI_Z_MAX_CM,
        FALL_TERMINAL_HEIGHT_CM,
        (uint32_t)FALL_SUSPECT_LOW_DURATION_MS,
        (uint32_t)FALL_SUSPECT_LOW_RATIO_PERCENT,
        (uint32_t)FALL_CONFIRM_LOW_DURATION_MS,
        (uint32_t)FALL_CONFIRM_MIN_HORIZONTAL_DISPLACEMENT_CM,
        (uint32_t)FALL_ACQUIRE_MIN_POINTS,
        (uint32_t)FALL_REARM_TRACK_GATE_CM,
        (uint32_t)FALL_CANDIDATE_HOLD_TOLERANCE_MS);
    return 0;
}

int hw_init_cb(void)
{
    return 0;
}

int app_deinit_cb(void)
{
    fall_pipeline_init(&s_fall_pipeline, &s_pipeline_config);
    fall_rule_init(&s_fall_rule, &s_rule_config);
    s_frame_period_warning_printed = 0U;
    return 0;
}

int hw_deinit_cb(void)
{
    return 0;
}
