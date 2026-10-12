#include "fall_rule.h"

#include <string.h>

static uint16_t fall_ring_index(uint16_t head, uint16_t offset)
{
    return (uint16_t)((head + offset) % FALL_HISTORY_MAX_SAMPLES);
}

static int16_t fall_max_i16(int16_t left, int16_t right)
{
    return (left > right) ? left : right;
}

static void fall_recent_clear(FallRuleRuntime_t *runtime)
{
    runtime->recent_head = 0U;
    runtime->recent_count = 0U;
    runtime->recent_incomplete = 0U;
}

static void fall_height_clear(FallRuleRuntime_t *runtime)
{
    runtime->height_head = 0U;
    runtime->height_count = 0U;
    runtime->height_incomplete = 0U;
}

static void fall_fresh_clear(FallRuleRuntime_t *runtime)
{
    runtime->fresh_head = 0U;
    runtime->fresh_count = 0U;
    runtime->fresh_incomplete = 0U;
}

static void fall_recent_trim(FallRuleRuntime_t *runtime, uint32_t timestamp_ms)
{
    while (runtime->recent_count > 0U) {
        FallRuleRecentSample_t *oldest = &runtime->recent[runtime->recent_head];
        if (timestamp_ms - oldest->timestamp_ms <= runtime->config.drop_window_ms) {
            break;
        }
        runtime->recent_head = fall_ring_index(runtime->recent_head, 1U);
        runtime->recent_count--;
    }
}

static void fall_height_trim(
    FallRuleHeightSample_t *history,
    uint16_t *head,
    uint16_t *count,
    uint32_t timestamp_ms,
    uint32_t window_ms)
{
    while (*count > 1U) {
        uint16_t second_index = fall_ring_index(*head, 1U);
        if (timestamp_ms - history[second_index].timestamp_ms <= window_ms) {
            break;
        }
        *head = second_index;
        (*count)--;
    }
}

static void fall_recent_push(FallRuleRuntime_t *runtime, const FallTarget_t *target)
{
    uint16_t index;

    fall_recent_trim(runtime, target->timestamp_ms);
    if (runtime->recent_count >= FALL_HISTORY_MAX_SAMPLES) {
        runtime->recent_head = fall_ring_index(runtime->recent_head, 1U);
        runtime->recent_count--;
        runtime->recent_incomplete = 1U;
    }
    index = fall_ring_index(runtime->recent_head, runtime->recent_count);
    runtime->recent[index].timestamp_ms = target->timestamp_ms;
    runtime->recent[index].frame_index = target->frame_index;
    runtime->recent[index].center_x_cm = target->center_x_cm;
    runtime->recent[index].center_y_cm = target->center_y_cm;
    runtime->recent[index].center_z_cm = target->center_z_cm;
    runtime->recent[index].motion_count = target->motion_count;
    runtime->recent[index].max_abs_velocity_cm_s = target->max_abs_velocity_cm_s;
    runtime->recent_count++;

    if (runtime->recent_incomplete && runtime->recent_count > 0U &&
        target->timestamp_ms - runtime->recent[runtime->recent_head].timestamp_ms >=
            runtime->config.drop_window_ms) {
        runtime->recent_incomplete = 0U;
    }
}

static void fall_height_push(
    FallRuleHeightSample_t *history,
    uint16_t *head,
    uint16_t *count,
    uint8_t *incomplete,
    uint32_t timestamp_ms,
    int16_t center_z_cm,
    uint32_t window_ms)
{
    uint16_t index;

    fall_height_trim(history, head, count, timestamp_ms, window_ms);
    if (*count >= FALL_HISTORY_MAX_SAMPLES) {
        *head = fall_ring_index(*head, 1U);
        (*count)--;
        *incomplete = 1U;
    }
    index = fall_ring_index(*head, *count);
    history[index].timestamp_ms = timestamp_ms;
    history[index].center_z_cm = center_z_cm;
    (*count)++;

    if (*incomplete && *count > 0U &&
        timestamp_ms - history[*head].timestamp_ms >= window_ms) {
        *incomplete = 0U;
    }
}

static bool fall_height_history_covers(
    const FallRuleHeightSample_t *history,
    uint16_t head,
    uint16_t count,
    uint8_t incomplete,
    uint32_t timestamp_ms,
    uint32_t duration_ms)
{
    if (count == 0U || incomplete != 0U) {
        return false;
    }
    if (duration_ms == 0U) {
        return true;
    }
    return timestamp_ms - history[head].timestamp_ms >= duration_ms;
}

static bool fall_height_state_covers(
    const FallRuleHeightSample_t *history,
    uint16_t head,
    uint16_t count,
    uint32_t timestamp_ms,
    uint32_t duration_ms,
    int16_t threshold_cm)
{
    uint16_t offset;

    if (count == 0U) {
        return false;
    }
    if (duration_ms == 0U) {
        uint16_t last = fall_ring_index(head, (uint16_t)(count - 1U));
        return history[last].center_z_cm <= threshold_cm;
    }
    for (offset = 0; offset < count; offset++) {
        const FallRuleHeightSample_t *sample = &history[fall_ring_index(head, offset)];
        if (timestamp_ms - sample->timestamp_ms >= duration_ms &&
            sample->center_z_cm <= threshold_cm) {
            return true;
        }
    }
    return false;
}

static uint16_t fall_height_sample_count(
    const FallRuleHeightSample_t *history,
    uint16_t head,
    uint16_t count,
    uint32_t timestamp_ms,
    uint32_t duration_ms)
{
    uint16_t relevant = 0U;
    uint16_t offset;
    for (offset = 0; offset < count; offset++) {
        const FallRuleHeightSample_t *sample = &history[fall_ring_index(head, offset)];
        if (timestamp_ms - sample->timestamp_ms <= duration_ms) {
            relevant++;
        }
    }
    return relevant;
}

static uint16_t fall_height_low_count(
    const FallRuleHeightSample_t *history,
    uint16_t head,
    uint16_t count,
    uint32_t timestamp_ms,
    uint32_t duration_ms,
    int16_t threshold_cm)
{
    uint16_t low = 0U;
    uint16_t offset;
    for (offset = 0; offset < count; offset++) {
        const FallRuleHeightSample_t *sample = &history[fall_ring_index(head, offset)];
        if (timestamp_ms - sample->timestamp_ms <= duration_ms &&
            sample->center_z_cm <= threshold_cm) {
            low++;
        }
    }
    return low;
}

static bool fall_height_ratio_meets(
    const FallRuleHeightSample_t *history,
    uint16_t head,
    uint16_t count,
    uint32_t timestamp_ms,
    uint32_t duration_ms,
    int16_t threshold_cm,
    uint8_t ratio_percent)
{
    uint16_t relevant = fall_height_sample_count(
        history,
        head,
        count,
        timestamp_ms,
        duration_ms);
    uint16_t low;

    if (relevant == 0U) {
        return false;
    }
    low = fall_height_low_count(
        history,
        head,
        count,
        timestamp_ms,
        duration_ms,
        threshold_cm);
    return (uint32_t)low * 100U >= (uint32_t)relevant * ratio_percent;
}

static int16_t fall_recent_drop_cm(const FallRuleRuntime_t *runtime, const FallTarget_t *target)
{
    int16_t highest = target->center_z_cm;
    uint16_t offset;
    for (offset = 0; offset < runtime->recent_count; offset++) {
        const FallRuleRecentSample_t *sample =
            &runtime->recent[fall_ring_index(runtime->recent_head, offset)];
        if (sample->center_z_cm > highest) {
            highest = sample->center_z_cm;
        }
    }
    return highest - target->center_z_cm;
}

static bool fall_recent_has_motion(const FallRuleRuntime_t *runtime)
{
    uint16_t offset;
    for (offset = 0; offset < runtime->recent_count; offset++) {
        const FallRuleRecentSample_t *sample =
            &runtime->recent[fall_ring_index(runtime->recent_head, offset)];
        if (sample->max_abs_velocity_cm_s >= runtime->config.motion_velocity_threshold_cm_s ||
            sample->motion_count >= runtime->config.motion_point_threshold) {
            return true;
        }
    }
    return false;
}

static uint32_t fall_horizontal_displacement_sq_cm(
    int16_t first_x_cm,
    int16_t first_y_cm,
    int16_t second_x_cm,
    int16_t second_y_cm)
{
    int32_t delta_x_cm = (int32_t)first_x_cm - second_x_cm;
    int32_t delta_y_cm = (int32_t)first_y_cm - second_y_cm;
    uint64_t squared =
        (uint64_t)((int64_t)delta_x_cm * delta_x_cm) +
        (uint64_t)((int64_t)delta_y_cm * delta_y_cm);
    return (squared > UINT32_MAX) ? UINT32_MAX : (uint32_t)squared;
}

static void fall_candidate_begin(
    FallRuleRuntime_t *runtime,
    const FallTarget_t *target,
    int16_t drop_cm)
{
    const FallRuleRecentSample_t *anchor = NULL;
    const FallRuleRecentSample_t *highest = NULL;
    uint16_t remaining = runtime->recent_count;

    while (remaining > 0U) {
        const FallRuleRecentSample_t *sample = &runtime->recent[
            fall_ring_index(runtime->recent_head, (uint16_t)(remaining - 1U))];
        if ((highest == NULL) || (sample->center_z_cm > highest->center_z_cm)) {
            highest = sample;
        }
        if (sample->center_z_cm >= runtime->config.rearm_height_cm) {
            anchor = sample;
            break;
        }
        remaining--;
    }
    if (anchor == NULL) {
        anchor = highest;
    }

    runtime->candidate_active = 1U;
    runtime->candidate_frame_index = target->frame_index;
    runtime->candidate_timestamp_ms = target->timestamp_ms;
    runtime->candidate_drop_cm = drop_cm;
    runtime->candidate_anchor_x_cm = (anchor != NULL) ? anchor->center_x_cm : target->center_x_cm;
    runtime->candidate_anchor_y_cm = (anchor != NULL) ? anchor->center_y_cm : target->center_y_cm;
    runtime->candidate_max_horizontal_displacement_sq_cm = fall_horizontal_displacement_sq_cm(
        runtime->candidate_anchor_x_cm,
        runtime->candidate_anchor_y_cm,
        target->center_x_cm,
        target->center_y_cm);
    runtime->candidate_rearm_active = 0U;
}

static void fall_candidate_update_horizontal_displacement(
    FallRuleRuntime_t *runtime,
    const FallTarget_t *target)
{
    uint32_t displacement_sq_cm = fall_horizontal_displacement_sq_cm(
        runtime->candidate_anchor_x_cm,
        runtime->candidate_anchor_y_cm,
        target->center_x_cm,
        target->center_y_cm);
    if (displacement_sq_cm > runtime->candidate_max_horizontal_displacement_sq_cm) {
        runtime->candidate_max_horizontal_displacement_sq_cm = displacement_sq_cm;
    }
}

static bool fall_candidate_horizontal_displacement_ready(const FallRuleRuntime_t *runtime)
{
    uint32_t threshold_cm = runtime->config.confirm_min_horizontal_displacement_cm;
    return threshold_cm == 0U ||
           runtime->candidate_max_horizontal_displacement_sq_cm >= threshold_cm * threshold_cm;
}

static void fall_event_append(
    FallEventBatch_t *batch,
    FallEventState_e state,
    FallEventReason_e reason,
    const FallTarget_t *target,
    int16_t drop_height_cm);

static void fall_reset_to_normal(FallRuleRuntime_t *runtime);

/* Cancel a candidate that has clearly returned to a high position. A hold or
 * one noisy high point is insufficient; the same target must remain fresh,
 * nearby, and high for the configured rearm duration. */
static bool fall_candidate_rearm_if_ready(
    FallRuleRuntime_t *runtime,
    const FallTarget_t *target,
    FallEventBatch_t *events)
{
    uint32_t rearm_gate_cm;
    uint32_t rearm_gate_squared;
    uint32_t displacement_sq_cm;
    bool was_suspected;
    int16_t previous_drop;

    if (runtime->candidate_active == 0U) {
        runtime->candidate_rearm_active = 0U;
        return false;
    }

    rearm_gate_cm = runtime->config.rearm_max_horizontal_displacement_cm;
    rearm_gate_squared = rearm_gate_cm * rearm_gate_cm;
    displacement_sq_cm = fall_horizontal_displacement_sq_cm(
        target->center_x_cm,
        target->center_y_cm,
        runtime->candidate_anchor_x_cm,
        runtime->candidate_anchor_y_cm);
    if (target->point_count < runtime->config.fresh_min_point_count ||
        target->center_z_cm < runtime->config.rearm_height_cm ||
        displacement_sq_cm > rearm_gate_squared) {
        runtime->candidate_rearm_active = 0U;
        return false;
    }

    if (runtime->candidate_rearm_active == 0U) {
        runtime->candidate_rearm_active = 1U;
        runtime->candidate_rearm_started_ms = target->timestamp_ms;
        return false;
    }
    if (target->timestamp_ms - runtime->candidate_rearm_started_ms <
        runtime->config.rearm_duration_ms) {
        return false;
    }

    was_suspected = runtime->suspected_active != 0U;
    previous_drop = was_suspected ? runtime->suspected_drop_cm : runtime->candidate_drop_cm;
    fall_reset_to_normal(runtime);
    if (was_suspected) {
        fall_event_append(
            events,
            FALL_EVENT_RECOVERED,
            FALL_REASON_STOOD_UP,
            target,
            previous_drop);
    }
    return true;
}

static void fall_event_append(
    FallEventBatch_t *batch,
    FallEventState_e state,
    FallEventReason_e reason,
    const FallTarget_t *target,
    int16_t drop_height_cm)
{
    FallEvent_t *event;
    if (batch->count >= FALL_EVENT_BATCH_MAX) {
        return;
    }
    event = &batch->events[batch->count++];
    event->state = state;
    event->reason = reason;
    event->frame_index = target->frame_index;
    event->timestamp_ms = target->timestamp_ms;
    event->drop_height_cm = drop_height_cm;
    event->center_z_cm = target->center_z_cm;
}

static void fall_reset_to_normal(FallRuleRuntime_t *runtime)
{
    FallRuleConfig_t config = runtime->config;
    memset(runtime, 0, sizeof(*runtime));
    runtime->config = config;
    runtime->state = FALL_STATE_NORMAL;
}

static bool fall_rearm_confirmed(
    FallRuleRuntime_t *runtime,
    const FallTarget_t *target,
    FallEventBatch_t *events)
{
    int16_t previous_drop;
    uint32_t rearm_gate_cm;
    uint32_t rearm_gate_squared;
    uint32_t origin_gate_cm;
    uint32_t origin_gate_squared;
    bool near_confirmed_center;
    bool near_fall_origin;

    if (runtime->state != FALL_STATE_CONFIRMED) {
        return false;
    }

    /* A high point only clears an alarm when it is a fresh continuation of the
     * confirmed target. This blocks a distant high background cluster from
     * being interpreted as the fallen person standing up. */
    rearm_gate_cm = runtime->config.rearm_max_horizontal_displacement_cm;
    rearm_gate_squared = rearm_gate_cm * rearm_gate_cm;
    origin_gate_cm = runtime->config.rearm_origin_max_horizontal_displacement_cm;
    origin_gate_squared = origin_gate_cm * origin_gate_cm;
    near_confirmed_center =
        fall_horizontal_displacement_sq_cm(
            target->center_x_cm,
            target->center_y_cm,
            runtime->confirmed_center_x_cm,
            runtime->confirmed_center_y_cm) <= rearm_gate_squared;
    near_fall_origin =
        fall_horizontal_displacement_sq_cm(
            target->center_x_cm,
            target->center_y_cm,
            runtime->candidate_anchor_x_cm,
            runtime->candidate_anchor_y_cm) <= origin_gate_squared;

    /* A backward/forward fall can move the low point-cloud center away from
     * the place where the person naturally stands again. Keep the strict low
     * center gate, but also accept the pre-fall origin after the same fresh,
     * high and sustained recovery evidence. */
    if (target->point_count < runtime->config.fresh_min_point_count ||
        (!near_confirmed_center && !near_fall_origin) ||
        target->center_z_cm < runtime->config.rearm_height_cm) {
        runtime->rearm_active = 0U;
        return false;
    }
    if (runtime->rearm_active == 0U) {
        runtime->rearm_active = 1U;
        runtime->rearm_started_ms = target->timestamp_ms;
        return false;
    }
    if (target->timestamp_ms - runtime->rearm_started_ms < runtime->config.rearm_duration_ms) {
        return false;
    }

    previous_drop = runtime->suspected_active ? runtime->suspected_drop_cm : 0;
    fall_reset_to_normal(runtime);
    fall_event_append(
        events,
        FALL_EVENT_RECOVERED,
        FALL_REASON_STOOD_UP,
        target,
        previous_drop);
    return true;
}
void fall_rule_init(FallRuleRuntime_t *runtime, const FallRuleConfig_t *config)
{
    if ((runtime == NULL) || (config == NULL)) {
        return;
    }
    memset(runtime, 0, sizeof(*runtime));
    runtime->config = *config;
    runtime->state = FALL_STATE_NORMAL;
}

FallEventBatch_t fall_rule_step(FallRuleRuntime_t *runtime, const FallTarget_t *target)
{
    FallEventBatch_t events;
    uint32_t height_window_ms;
    int16_t drop_cm;
    bool candidate_low_ready;
    bool terminal_suspect_ready;
    bool confirm_low_ready;
    bool suspected_created = false;

    memset(&events, 0, sizeof(events));
    if ((runtime == NULL) || (target == NULL)) {
        return events;
    }

    runtime->current_fresh = 0U;
    runtime->current_terminal_low = 0U;
    runtime->diagnostic_fresh_count = 0U;
    runtime->diagnostic_low_ratio_percent = 0U;
    if (target->has_target == 0U) {
        fall_height_clear(runtime);
        fall_fresh_clear(runtime);
        runtime->last_fresh_timestamp_ms = 0U;
        if (runtime->state == FALL_STATE_CONFIRMED) {
            runtime->rearm_active = 0U;
        }
        return events;
    }

    if (runtime->state == FALL_STATE_CONFIRMED) {
        fall_rearm_confirmed(runtime, target, &events);
        if (runtime->state == FALL_STATE_CONFIRMED) {
            return events;
        }
    }

    if (fall_candidate_rearm_if_ready(runtime, target, &events)) {
        return events;
    }

    fall_recent_push(runtime, target);
    height_window_ms = (runtime->config.suspect_low_duration_ms >
                        runtime->config.confirm_low_duration_ms) ?
                       runtime->config.suspect_low_duration_ms :
                       runtime->config.confirm_low_duration_ms;
    fall_height_push(
        runtime->height_history,
        &runtime->height_head,
        &runtime->height_count,
        &runtime->height_incomplete,
        target->timestamp_ms,
        target->center_z_cm,
        height_window_ms);

    runtime->current_fresh = target->point_count >= runtime->config.fresh_min_point_count;
    runtime->current_terminal_low = target->center_z_cm <= runtime->config.terminal_height_cm;
    if (runtime->current_fresh) {
        runtime->last_fresh_timestamp_ms = target->timestamp_ms;
        fall_height_push(
            runtime->fresh_history,
            &runtime->fresh_head,
            &runtime->fresh_count,
            &runtime->fresh_incomplete,
            target->timestamp_ms,
            target->center_z_cm,
            height_window_ms);
    } else if (runtime->candidate_active &&
               target->state == FALL_TRACK_HOLD &&
               runtime->last_fresh_timestamp_ms != 0U) {
        uint32_t fresh_gap_ms = target->timestamp_ms - runtime->last_fresh_timestamp_ms;
        if (fresh_gap_ms <= runtime->config.candidate_hold_tolerance_ms) {
            /* Preserve short tracker holds as usable continuity evidence. The
             * actual current_fresh flag remains false for diagnostics. */
            fall_height_push(
                runtime->fresh_history,
                &runtime->fresh_head,
                &runtime->fresh_count,
                &runtime->fresh_incomplete,
                target->timestamp_ms,
                target->center_z_cm,
                height_window_ms);
        } else {
            /* Do not allow stale samples from before a long gap to satisfy a
             * later confirmation window. */
            fall_fresh_clear(runtime);
            runtime->last_fresh_timestamp_ms = 0U;
        }
    }

    drop_cm = fall_recent_drop_cm(runtime, target);
    candidate_low_ready =
        target->center_z_cm <= runtime->config.candidate_low_height_cm &&
        runtime->recent_incomplete == 0U &&
        fall_height_history_covers(
            runtime->height_history,
            runtime->height_head,
            runtime->height_count,
            runtime->height_incomplete,
            target->timestamp_ms,
            runtime->config.suspect_low_duration_ms) &&
        fall_height_ratio_meets(
            runtime->height_history,
            runtime->height_head,
            runtime->height_count,
            target->timestamp_ms,
            runtime->config.suspect_low_duration_ms,
            runtime->config.candidate_low_height_cm,
            runtime->config.suspect_low_ratio_percent);

    if (runtime->candidate_active == 0U &&
        drop_cm >= runtime->config.min_drop_cm &&
        fall_recent_has_motion(runtime) &&
        candidate_low_ready) {
        fall_candidate_begin(runtime, target, drop_cm);
    }

    if (runtime->candidate_active == 0U) {
        return events;
    }

    fall_candidate_update_horizontal_displacement(runtime, target);

    if (target->timestamp_ms - runtime->candidate_timestamp_ms > runtime->config.confirm_window_ms) {
        if (runtime->suspected_active != 0U) {
            fall_event_append(
                &events,
                FALL_EVENT_RECOVERED,
                FALL_REASON_SUSPECTED_EXPIRED,
                target,
                fall_max_i16(runtime->candidate_drop_cm, drop_cm));
        }
        runtime->candidate_active = 0U;
        runtime->suspected_active = 0U;
        runtime->state = FALL_STATE_NORMAL;
        return events;
    }

    runtime->diagnostic_fresh_count = fall_height_sample_count(
        runtime->fresh_history,
        runtime->fresh_head,
        runtime->fresh_count,
        target->timestamp_ms,
        runtime->config.confirm_low_duration_ms);
    if (runtime->diagnostic_fresh_count > 0U) {
        uint16_t diagnostic_low = fall_height_low_count(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            target->timestamp_ms,
            runtime->config.confirm_low_duration_ms,
            runtime->config.terminal_height_cm);
        runtime->diagnostic_low_ratio_percent =
            (uint8_t)((uint32_t)diagnostic_low * 100U / runtime->diagnostic_fresh_count);
    }

    terminal_suspect_ready =
        runtime->current_fresh != 0U &&
        runtime->current_terminal_low != 0U &&
        fall_height_history_covers(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            runtime->fresh_incomplete,
            target->timestamp_ms,
            runtime->config.suspect_low_duration_ms) &&
        fall_height_sample_count(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            target->timestamp_ms,
            runtime->config.suspect_low_duration_ms) >=
            runtime->config.suspect_min_fresh_samples &&
        fall_height_state_covers(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            target->timestamp_ms,
            runtime->config.suspect_low_duration_ms,
            runtime->config.terminal_height_cm) &&
        fall_height_ratio_meets(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            target->timestamp_ms,
            runtime->config.suspect_low_duration_ms,
            runtime->config.terminal_height_cm,
            runtime->config.suspect_low_ratio_percent);

    if (runtime->suspected_active == 0U && terminal_suspect_ready) {
        runtime->suspected_active = 1U;
        runtime->suspected_frame_index = target->frame_index;
        runtime->suspected_timestamp_ms = target->timestamp_ms;
        runtime->suspected_drop_cm = fall_max_i16(runtime->candidate_drop_cm, drop_cm);
        runtime->state = FALL_STATE_SUSPECTED;
        fall_event_append(
            &events,
            FALL_EVENT_SUSPECTED,
            FALL_REASON_TERMINAL_PERSISTED,
            target,
            runtime->suspected_drop_cm);
        suspected_created = true;
    }

    if (runtime->suspected_active == 0U || suspected_created) {
        return events;
    }

    confirm_low_ready =
        runtime->current_fresh != 0U &&
        runtime->current_terminal_low != 0U &&
        target->timestamp_ms - runtime->suspected_timestamp_ms >=
            runtime->config.confirm_low_duration_ms &&
        fall_height_history_covers(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            runtime->fresh_incomplete,
            target->timestamp_ms,
            runtime->config.confirm_low_duration_ms) &&
        fall_height_sample_count(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            target->timestamp_ms,
            runtime->config.confirm_low_duration_ms) >=
            runtime->config.confirm_min_fresh_samples &&
        fall_height_state_covers(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            target->timestamp_ms,
            runtime->config.confirm_low_duration_ms,
            runtime->config.terminal_height_cm) &&
        fall_height_ratio_meets(
            runtime->fresh_history,
            runtime->fresh_head,
            runtime->fresh_count,
            target->timestamp_ms,
            runtime->config.confirm_low_duration_ms,
            runtime->config.terminal_height_cm,
            runtime->config.confirm_low_ratio_percent) &&
        fall_candidate_horizontal_displacement_ready(runtime);

    if (confirm_low_ready) {
        bool still = target->max_abs_velocity_cm_s <=
                         runtime->config.still_velocity_threshold_cm_s &&
                     target->motion_count <= runtime->config.still_motion_points_max;
        int16_t final_drop = fall_max_i16(
            runtime->candidate_drop_cm,
            fall_max_i16(runtime->suspected_drop_cm, drop_cm));
        runtime->confirmed = 1U;
        runtime->confirmed_timestamp_ms = target->timestamp_ms;
        runtime->confirmed_center_x_cm = target->center_x_cm;
        runtime->confirmed_center_y_cm = target->center_y_cm;
        runtime->confirmed_low_z_cm = target->center_z_cm;
        runtime->rearm_active = 0U;
        runtime->state = FALL_STATE_CONFIRMED;
        fall_event_append(
            &events,
            FALL_EVENT_CONFIRMED,
            still ? FALL_REASON_LOW_STILL : FALL_REASON_LOW_ACTIVE,
            target,
            final_drop);
    }
    return events;
}

FallState_e fall_rule_state_get(const FallRuleRuntime_t *runtime)
{
    return (runtime == NULL) ? FALL_STATE_NORMAL : runtime->state;
}
