#include "fall_pipeline.h"

#include <math.h>
#include <string.h>

static int16_t fall_abs_i16(int16_t value)
{
    if (value == (int16_t)0x8000) {
        return 32767;
    }
    return (value < 0) ? (int16_t)-value : value;
}

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

static int16_t fall_divide_round_nearest(int32_t value, int32_t divisor)
{
    if (value >= 0) {
        return (int16_t)((value + divisor / 2) / divisor);
    }
    return (int16_t)((value - divisor / 2) / divisor);
}

static int16_t fall_median_i16(int16_t *values, uint16_t count)
{
    uint16_t index;

    for (index = 1; index < count; index++) {
        int16_t key = values[index];
        int16_t cursor = (int16_t)index - 1;
        while ((cursor >= 0) && (values[cursor] > key)) {
            values[cursor + 1] = values[cursor];
            cursor--;
        }
        values[cursor + 1] = key;
    }

    if ((count & 1U) != 0U) {
        return values[count / 2U];
    }
    return fall_divide_round_nearest(
        (int32_t)values[count / 2U - 1U] + values[count / 2U],
        2);
}

static uint32_t fall_xyz_distance_squared(const FallPoint_t *left, const FallPoint_t *right)
{
    int32_t dx = (int32_t)left->x_cm - right->x_cm;
    int32_t dy = (int32_t)left->y_cm - right->y_cm;
    int32_t dz = (int32_t)left->z_cm - right->z_cm;
    return (uint32_t)(dx * dx + dy * dy + dz * dz);
}

static uint32_t fall_xy_distance_squared(
    int16_t left_x,
    int16_t left_y,
    int16_t right_x,
    int16_t right_y)
{
    int32_t dx = (int32_t)left_x - right_x;
    int32_t dy = (int32_t)left_y - right_y;
    return (uint32_t)(dx * dx + dy * dy);
}

static bool fall_point_is_in_roi(const FallPipelineRuntime_t *runtime, const FallPoint_t *point)
{
    const FallPipelineConfig_t *config = &runtime->config;

    return point->x_cm >= config->roi_x_min_cm &&
           point->x_cm <= config->roi_x_max_cm &&
           point->y_cm >= config->roi_y_min_cm &&
           point->y_cm <= config->roi_y_max_cm &&
           point->z_cm >= config->roi_z_min_cm &&
           point->z_cm <= config->roi_z_max_cm &&
           point->snr_db >= config->min_snr_db;
}

static bool fall_cluster_can_include(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *cluster,
    const FallPoint_t *point)
{
    int16_t x_min = (point->x_cm < cluster->x_min_cm) ? point->x_cm : cluster->x_min_cm;
    int16_t x_max = (point->x_cm > cluster->x_max_cm) ? point->x_cm : cluster->x_max_cm;
    int16_t y_min = (point->y_cm < cluster->y_min_cm) ? point->y_cm : cluster->y_min_cm;
    int16_t y_max = (point->y_cm > cluster->y_max_cm) ? point->y_cm : cluster->y_max_cm;

    return (x_max - x_min) <= runtime->config.max_cluster_width_cm &&
           (y_max - y_min) <= runtime->config.max_cluster_depth_cm;
}

static void fall_cluster_expand_bounds(FallCluster_t *cluster, const FallPoint_t *point)
{
    if (point->x_cm < cluster->x_min_cm) {
        cluster->x_min_cm = point->x_cm;
    }
    if (point->x_cm > cluster->x_max_cm) {
        cluster->x_max_cm = point->x_cm;
    }
    if (point->y_cm < cluster->y_min_cm) {
        cluster->y_min_cm = point->y_cm;
    }
    if (point->y_cm > cluster->y_max_cm) {
        cluster->y_max_cm = point->y_cm;
    }
    if (point->z_cm < cluster->z_min_cm) {
        cluster->z_min_cm = point->z_cm;
    }
    if (point->z_cm > cluster->z_max_cm) {
        cluster->z_max_cm = point->z_cm;
    }
}

static void fall_cluster_build_statistics(
    FallPipelineRuntime_t *runtime,
    FallCluster_t *cluster,
    const uint16_t *indices,
    uint16_t count)
{
    uint16_t index;
    uint16_t motion_count = 0;
    int16_t max_abs_velocity = 0;

    for (index = 0; index < count; index++) {
        uint16_t point_index = indices[index];
        const FallPoint_t *point = &runtime->points[point_index];
        int16_t abs_velocity = fall_abs_i16(point->velocity_cm_s);
        runtime->sort_scratch[index] = point->x_cm;
        if (point->is_motion != 0U) {
            motion_count++;
        }
        if (abs_velocity > max_abs_velocity) {
            max_abs_velocity = abs_velocity;
        }
    }
    cluster->center_x_cm = fall_median_i16(runtime->sort_scratch, count);

    for (index = 0; index < count; index++) {
        uint16_t point_index = indices[index];
        runtime->sort_scratch[index] = runtime->points[point_index].y_cm;
    }
    cluster->center_y_cm = fall_median_i16(runtime->sort_scratch, count);

    for (index = 0; index < count; index++) {
        uint16_t point_index = indices[index];
        runtime->sort_scratch[index] = runtime->points[point_index].z_cm;
    }
    cluster->center_z_cm = fall_median_i16(runtime->sort_scratch, count);
    cluster->point_count = count;
    cluster->motion_count = motion_count;
    cluster->max_abs_velocity_cm_s = max_abs_velocity;
}

static void fall_cluster_points(FallPipelineRuntime_t *runtime)
{
    uint16_t seed;
    uint16_t cluster_index_write = 0;
    uint32_t radius_cm = (uint32_t)(uint16_t)runtime->config.cluster_radius_cm;
    uint32_t radius_squared = radius_cm * radius_cm;

    memset(runtime->assigned, 0, runtime->point_count * sizeof(runtime->assigned[0]));
    runtime->cluster_count = 0;

    for (seed = 0; seed < runtime->point_count; seed++) {
        FallCluster_t cluster;
        uint16_t queue_tail = 0;
        uint16_t current;

        if (runtime->assigned[seed] != 0U) {
            continue;
        }
        if (runtime->cluster_count >= FALL_MAX_CLUSTERS) {
            runtime->cluster_overflow_count++;
            break;
        }

        memset(&cluster, 0, sizeof(cluster));
        cluster.start = cluster_index_write;
        cluster.x_min_cm = runtime->points[seed].x_cm;
        cluster.x_max_cm = runtime->points[seed].x_cm;
        cluster.y_min_cm = runtime->points[seed].y_cm;
        cluster.y_max_cm = runtime->points[seed].y_cm;
        cluster.z_min_cm = runtime->points[seed].z_cm;
        cluster.z_max_cm = runtime->points[seed].z_cm;
        runtime->assigned[seed] = 1U;
        runtime->queue[queue_tail++] = seed;

        while (queue_tail > 0U) {
            uint16_t candidate;
            current = runtime->queue[--queue_tail];
            runtime->cluster_point_indices[cluster_index_write++] = current;
            cluster.count++;

            for (candidate = 0; candidate < runtime->point_count; candidate++) {
                if (runtime->assigned[candidate] != 0U) {
                    continue;
                }
                if (fall_xyz_distance_squared(
                        &runtime->points[current],
                        &runtime->points[candidate]) > radius_squared) {
                    continue;
                }
                if (!fall_cluster_can_include(runtime, &cluster, &runtime->points[candidate])) {
                    continue;
                }
                runtime->assigned[candidate] = 1U;
                runtime->queue[queue_tail++] = candidate;
                fall_cluster_expand_bounds(&cluster, &runtime->points[candidate]);
            }
        }

        fall_cluster_build_statistics(
            runtime,
            &cluster,
            &runtime->cluster_point_indices[cluster.start],
            cluster.count);
        runtime->clusters[runtime->cluster_count++] = cluster;
    }
}

static uint16_t fall_static_body_min_points(const FallPipelineRuntime_t *runtime)
{
    uint16_t derived = runtime->config.min_cluster_points * 6U;
    return (derived > runtime->config.static_body_min_points) ?
           derived : runtime->config.static_body_min_points;
}

/* During the descent, a person can move more than the normal frame-to-frame
 * gate. Expand the gate only after the locked target is already below the
 * upper-body height range, and only accept a substantial/moving continuation.
 * This keeps standing/crouching acquisition strict while preserving a lateral
 * fall track when the point cloud shifts forward or sideways. */
static uint16_t fall_effective_track_gate_cm(
    const FallPipelineRuntime_t *runtime,
    const FallTarget_t *target)
{
    uint16_t gate_cm = (uint16_t)runtime->config.track_gate_cm;

    if ((target != NULL) &&
        target->center_z_cm <= FALL_LATERAL_TRACK_HEIGHT_CM &&
        gate_cm < FALL_LATERAL_TRACK_GATE_CM) {
        gate_cm = FALL_LATERAL_TRACK_GATE_CM;
    }
    return gate_cm;
}

static bool fall_cluster_is_lateral_continuation(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *cluster)
{
    return cluster->motion_count > 0U ||
           cluster->center_z_cm <= FALL_CANDIDATE_LOW_HEIGHT_CM ||
           cluster->point_count >= fall_static_body_min_points(runtime);
}

static bool fall_cluster_is_candidate(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *cluster)
{
    return cluster->point_count >= runtime->config.min_cluster_points;
}

/* Acquisition is stricter than an already locked target so sparse static clutter
 * cannot become the first tracked object. Locked low-body clusters still use the
 * original minimum, which preserves a fallen person after a valid track exists. */
static bool fall_cluster_is_acquisition_candidate(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *cluster)
{
    return fall_cluster_is_candidate(runtime, cluster) &&
           cluster->point_count >= FALL_ACQUIRE_MIN_POINTS &&
           (cluster->motion_count > 0U ||
            cluster->point_count >= fall_static_body_min_points(runtime));
}

static uint16_t fall_cluster_distance_cm(
    const FallCluster_t *cluster,
    const FallTarget_t *target)
{
    return (uint16_t)sqrtf((float)fall_xy_distance_squared(
        cluster->center_x_cm,
        cluster->center_y_cm,
        target->center_x_cm,
        target->center_y_cm));
}

static FallCluster_t *fall_select_global_cluster(FallPipelineRuntime_t *runtime)
{
    FallCluster_t *best = NULL;
    bool motion_candidates = false;
    uint16_t index;

    for (index = 0; index < runtime->cluster_count; index++) {
        FallCluster_t *cluster = &runtime->clusters[index];
        if (fall_cluster_is_acquisition_candidate(runtime, cluster) && cluster->motion_count > 0U) {
            motion_candidates = true;
            break;
        }
    }

    for (index = 0; index < runtime->cluster_count; index++) {
        FallCluster_t *cluster = &runtime->clusters[index];
        int16_t height;
        int16_t best_height;

        if (!fall_cluster_is_acquisition_candidate(runtime, cluster)) {
            continue;
        }
        if (motion_candidates && cluster->motion_count == 0U) {
            continue;
        }
        if (best == NULL) {
            best = cluster;
            continue;
        }
        height = cluster->z_max_cm - cluster->z_min_cm;
        best_height = best->z_max_cm - best->z_min_cm;
        if (cluster->motion_count > best->motion_count ||
            (cluster->motion_count == best->motion_count && cluster->point_count > best->point_count) ||
            (cluster->motion_count == best->motion_count && cluster->point_count == best->point_count &&
             height > best_height)) {
            best = cluster;
        }
    }
    return best;
}

static int32_t fall_near_target_score(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *cluster,
    const FallTarget_t *target)
{
    uint16_t distance_cm = fall_cluster_distance_cm(cluster, target);
    uint16_t point_cap = (cluster->point_count > 80U) ? 80U : cluster->point_count;
    uint16_t height_cm = (uint16_t)(cluster->z_max_cm - cluster->z_min_cm);
    uint16_t height_cap = (height_cm > 150U) ? 150U : height_cm;
    uint16_t gate_cm = (runtime->config.track_gate_cm > 0) ?
                       (uint16_t)runtime->config.track_gate_cm : 1U;
    uint16_t doubled_gate_cm = (uint16_t)(gate_cm * 2U);
    uint16_t distance_cap = (distance_cm > doubled_gate_cm) ? doubled_gate_cm : distance_cm;
    int32_t score = 0;

    score += (int32_t)cluster->motion_count * 500;
    score += (int32_t)point_cap * 25;
    if (cluster->point_count > 0U) {
        score += (int32_t)cluster->motion_count * 2000 / cluster->point_count;
    }
    score += (int32_t)height_cap * 4;
    score -= (int32_t)distance_cap * 1500 / gate_cm;
    if (cluster->point_count < 6U) {
        score -= 2500;
    }
    if (cluster->motion_count == 0U &&
        cluster->point_count < fall_static_body_min_points(runtime)) {
        score -= 2000;
    }
    return score;
}

static FallCluster_t *fall_select_locked_cluster(
    FallPipelineRuntime_t *runtime,
    const FallTarget_t *target)
{
    FallCluster_t *best = NULL;
    uint16_t best_distance = 0;
    uint16_t gate_cm = fall_effective_track_gate_cm(runtime, target);
    uint16_t index;

    for (index = 0; index < runtime->cluster_count; index++) {
        FallCluster_t *cluster = &runtime->clusters[index];
        uint16_t distance;
        if (!fall_cluster_is_candidate(runtime, cluster)) {
            continue;
        }
        distance = fall_cluster_distance_cm(cluster, target);
        if (distance > gate_cm ||
            (distance > (uint16_t)runtime->config.track_gate_cm &&
             !fall_cluster_is_lateral_continuation(runtime, cluster))) {
            continue;
        }
        if (best == NULL || distance < best_distance ||
            (distance == best_distance && cluster->motion_count > best->motion_count) ||
            (distance == best_distance && cluster->motion_count == best->motion_count &&
             cluster->point_count > best->point_count)) {
            best = cluster;
            best_distance = distance;
        }
    }
    return best;
}

static FallCluster_t *fall_select_scored_near_cluster(
    FallPipelineRuntime_t *runtime,
    const FallTarget_t *target)
{
    FallCluster_t *best = NULL;
    int32_t best_score = 0;
    uint16_t best_distance = 0;
    uint16_t gate_cm = fall_effective_track_gate_cm(runtime, target);
    uint16_t index;

    for (index = 0; index < runtime->cluster_count; index++) {
        FallCluster_t *cluster = &runtime->clusters[index];
        uint16_t distance;
        int32_t score;
        if (!fall_cluster_is_candidate(runtime, cluster)) {
            continue;
        }
        distance = fall_cluster_distance_cm(cluster, target);
        if (distance > gate_cm ||
            (distance > (uint16_t)runtime->config.track_gate_cm &&
             !fall_cluster_is_lateral_continuation(runtime, cluster))) {
            continue;
        }
        score = fall_near_target_score(runtime, cluster, target);
        if (best == NULL || score > best_score ||
            (score == best_score && cluster->motion_count > best->motion_count) ||
            (score == best_score && cluster->motion_count == best->motion_count &&
             cluster->point_count > best->point_count) ||
            (score == best_score && cluster->motion_count == best->motion_count &&
             cluster->point_count == best->point_count && distance < best_distance)) {
            best = cluster;
            best_score = score;
            best_distance = distance;
        }
    }
    return best;
}

static bool fall_should_keep_locked_cluster(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *locked,
    const FallCluster_t *scored,
    const FallTarget_t *target)
{
    uint16_t locked_distance = fall_cluster_distance_cm(locked, target);
    uint16_t scored_distance = fall_cluster_distance_cm(scored, target);
    uint16_t close_limit = (uint16_t)runtime->config.track_gate_cm / 5U;
    uint16_t strong_motion = runtime->config.min_cluster_points * 2U;

    if (close_limit < 12U) {
        close_limit = 12U;
    }
    if (strong_motion < 6U) {
        strong_motion = 6U;
    }
    return locked_distance <= close_limit &&
           scored_distance > locked_distance + 18U &&
           locked->point_count >= fall_static_body_min_points(runtime) &&
           locked->point_count >= scored->point_count * 4U &&
           scored->motion_count < strong_motion;
}

static bool fall_build_locked_core(
    FallPipelineRuntime_t *runtime,
    const FallCluster_t *cluster,
    const FallTarget_t *target,
    FallCluster_t *core)
{
    int16_t half_width = runtime->config.max_cluster_width_cm / 2;
    int16_t half_depth = runtime->config.max_cluster_depth_cm / 2;
    uint16_t count = 0;
    uint16_t index;

    for (index = 0; index < cluster->count; index++) {
        uint16_t point_index = runtime->cluster_point_indices[cluster->start + index];
        const FallPoint_t *point = &runtime->points[point_index];
        if (fall_abs_i16(point->x_cm - target->center_x_cm) <= half_width &&
            fall_abs_i16(point->y_cm - target->center_y_cm) <= half_depth) {
            runtime->queue[count++] = point_index;
        }
    }
    if (count == 0U) {
        return false;
    }

    memset(core, 0, sizeof(*core));
    core->count = count;
    core->x_min_cm = runtime->points[runtime->queue[0]].x_cm;
    core->x_max_cm = core->x_min_cm;
    core->y_min_cm = runtime->points[runtime->queue[0]].y_cm;
    core->y_max_cm = core->y_min_cm;
    core->z_min_cm = runtime->points[runtime->queue[0]].z_cm;
    core->z_max_cm = core->z_min_cm;
    for (index = 1; index < count; index++) {
        fall_cluster_expand_bounds(core, &runtime->points[runtime->queue[index]]);
    }
    fall_cluster_build_statistics(runtime, core, runtime->queue, count);
    return true;
}

static bool fall_should_use_full_cluster(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *cluster,
    const FallCluster_t *core)
{
    uint16_t strong_motion = runtime->config.min_cluster_points * 2U;
    if (strong_motion < 6U) {
        strong_motion = 6U;
    }
    return cluster->motion_count >= strong_motion ||
           (cluster->point_count >= fall_static_body_min_points(runtime) &&
            core->point_count * 100U >= cluster->point_count * 65U);
}

static bool fall_should_freeze_core(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *core)
{
    return core->motion_count == 0U &&
           core->point_count < fall_static_body_min_points(runtime);
}

static bool fall_target_looks_like_floor_clutter(const FallTarget_t *target)
{
    return target->motion_count == 0U &&
           target->center_z_cm <= 20 &&
           target->height_cm <= 15;
}

static bool fall_cluster_looks_like_elevated_body(const FallCluster_t *cluster)
{
    return cluster->motion_count > 0U &&
           (cluster->center_z_cm >= 50 ||
            cluster->z_max_cm >= 80 ||
            cluster->z_max_cm - cluster->z_min_cm >= 60);
}

static bool fall_can_hold(const FallPipelineRuntime_t *runtime)
{
    return runtime->has_last_target != 0U &&
           runtime->last_target.has_target != 0U &&
           runtime->timestamp_ms - runtime->last_target_timestamp_ms <= runtime->config.hold_ms;
}

static bool fall_confirm_far_candidate(
    FallPipelineRuntime_t *runtime,
    const FallCluster_t *candidate,
    FallCluster_t *confirmed)
{
    uint32_t gate_cm = (uint32_t)fall_effective_track_gate_cm(
        runtime,
        runtime->has_last_target ? &runtime->last_target : NULL);
    uint32_t gate_squared = gate_cm * gate_cm;
    uint8_t required_frames = (runtime->config.switch_confirm_frames > 0U) ?
                              runtime->config.switch_confirm_frames : 1U;

    if (candidate == NULL) {
        memset(&runtime->pending_switch, 0, sizeof(runtime->pending_switch));
        return false;
    }
    if (runtime->pending_switch.active == 0U ||
        fall_xy_distance_squared(
            candidate->center_x_cm,
            candidate->center_y_cm,
            runtime->pending_switch.center_x_cm,
            runtime->pending_switch.center_y_cm) > gate_squared) {
        runtime->pending_switch.active = 1U;
        runtime->pending_switch.center_x_cm = candidate->center_x_cm;
        runtime->pending_switch.center_y_cm = candidate->center_y_cm;
        runtime->pending_switch.frames = 1U;
        runtime->pending_switch.cluster = *candidate;
        return false;
    }

    runtime->pending_switch.center_x_cm = candidate->center_x_cm;
    runtime->pending_switch.center_y_cm = candidate->center_y_cm;
    runtime->pending_switch.frames++;
    runtime->pending_switch.cluster = *candidate;
    if (runtime->pending_switch.frames < required_frames) {
        return false;
    }
    *confirmed = runtime->pending_switch.cluster;
    memset(&runtime->pending_switch, 0, sizeof(runtime->pending_switch));
    return true;
}

static FallTarget_t fall_target_from_cluster(
    const FallPipelineRuntime_t *runtime,
    const FallCluster_t *cluster,
    bool freeze_xy,
    FallTrackState_e state)
{
    FallTarget_t target;
    memset(&target, 0, sizeof(target));
    target.has_target = 1U;
    target.state = state;
    target.center_x_cm = (freeze_xy && runtime->has_last_target) ?
                         runtime->last_target.center_x_cm : cluster->center_x_cm;
    target.center_y_cm = (freeze_xy && runtime->has_last_target) ?
                         runtime->last_target.center_y_cm : cluster->center_y_cm;
    target.center_z_cm = cluster->center_z_cm;
    target.z_min_cm = cluster->z_min_cm;
    target.z_max_cm = cluster->z_max_cm;
    target.height_cm = cluster->z_max_cm - cluster->z_min_cm;
    target.width_x_cm = (freeze_xy && runtime->has_last_target) ?
                        runtime->last_target.width_x_cm : cluster->x_max_cm - cluster->x_min_cm;
    target.depth_y_cm = (freeze_xy && runtime->has_last_target) ?
                        runtime->last_target.depth_y_cm : cluster->y_max_cm - cluster->y_min_cm;
    target.point_count = cluster->point_count;
    target.motion_count = cluster->motion_count;
    target.max_abs_velocity_cm_s = cluster->max_abs_velocity_cm_s;
    target.frame_index = runtime->frame_index;
    target.timestamp_ms = runtime->timestamp_ms;
    return target;
}

static FallTarget_t fall_held_target(const FallPipelineRuntime_t *runtime)
{
    FallTarget_t target = runtime->last_target;
    target.state = FALL_TRACK_HOLD;
    target.point_count = 0U;
    target.motion_count = 0U;
    target.max_abs_velocity_cm_s = 0;
    target.frame_index = runtime->frame_index;
    target.timestamp_ms = runtime->timestamp_ms;
    return target;
}

static FallTarget_t fall_track_frame(FallPipelineRuntime_t *runtime)
{
    FallTarget_t target;
    FallCluster_t *global;
    FallCluster_t *locked;
    FallCluster_t *scored;
    FallCluster_t selected_copy;
    FallCluster_t core;
    bool freeze_xy = false;

    memset(&target, 0, sizeof(target));
    target.frame_index = runtime->frame_index;
    target.timestamp_ms = runtime->timestamp_ms;
    target.state = FALL_TRACK_LOST;

    if (runtime->has_last_target == 0U || runtime->last_target.has_target == 0U) {
        global = fall_select_global_cluster(runtime);
        memset(&runtime->pending_switch, 0, sizeof(runtime->pending_switch));
        if (global == NULL) {
            return target;
        }
        target = fall_target_from_cluster(runtime, global, false, FALL_TRACK_TRACKING);
        runtime->last_target = target;
        runtime->last_target_timestamp_ms = runtime->timestamp_ms;
        runtime->has_last_target = 1U;
        return target;
    }

    if (fall_target_looks_like_floor_clutter(&runtime->last_target)) {
        global = fall_select_global_cluster(runtime);
        if (global != NULL &&
            fall_cluster_distance_cm(global, &runtime->last_target) >
                (uint16_t)runtime->config.track_gate_cm &&
            fall_cluster_looks_like_elevated_body(global)) {
            memset(&runtime->pending_switch, 0, sizeof(runtime->pending_switch));
            target = fall_target_from_cluster(runtime, global, false, FALL_TRACK_TRACKING);
            runtime->last_target = target;
            runtime->last_target_timestamp_ms = runtime->timestamp_ms;
            return target;
        }
    }

    locked = fall_select_locked_cluster(runtime, &runtime->last_target);
    scored = fall_select_scored_near_cluster(runtime, &runtime->last_target);
    if (locked != NULL && scored != NULL) {
        memset(&runtime->pending_switch, 0, sizeof(runtime->pending_switch));
        if (scored != locked &&
            !fall_should_keep_locked_cluster(runtime, locked, scored, &runtime->last_target)) {
            selected_copy = *scored;
        } else if (!fall_build_locked_core(runtime, locked, &runtime->last_target, &core)) {
            selected_copy.point_count = 0U;
        } else if (fall_should_use_full_cluster(runtime, locked, &core)) {
            selected_copy = *locked;
        } else {
            selected_copy = core;
            freeze_xy = fall_should_freeze_core(runtime, &core);
        }

        if (selected_copy.point_count > 0U) {
            target = fall_target_from_cluster(
                runtime,
                &selected_copy,
                freeze_xy,
                freeze_xy ? FALL_TRACK_STATIC_LOCK : FALL_TRACK_TRACKING);
            runtime->last_target = target;
            runtime->last_target_timestamp_ms = runtime->timestamp_ms;
            return target;
        }
    }

    if (fall_can_hold(runtime)) {
        memset(&runtime->pending_switch, 0, sizeof(runtime->pending_switch));
        return fall_held_target(runtime);
    }

    global = fall_select_global_cluster(runtime);
    if (fall_confirm_far_candidate(runtime, global, &selected_copy)) {
        target = fall_target_from_cluster(runtime, &selected_copy, false, FALL_TRACK_TRACKING);
        runtime->last_target = target;
        runtime->last_target_timestamp_ms = runtime->timestamp_ms;
        return target;
    }
    target.state = runtime->pending_switch.active ? FALL_TRACK_REACQUIRE : FALL_TRACK_LOST;
    return target;
}

void fall_pipeline_init(FallPipelineRuntime_t *runtime, const FallPipelineConfig_t *config)
{
    float pitch_radians;

    if ((runtime == NULL) || (config == NULL)) {
        return;
    }
    memset(runtime, 0, sizeof(*runtime));
    runtime->config = *config;
    pitch_radians = config->mount_pitch_deg * (3.14159265358979323846f / 180.0f);
    runtime->pitch_cos = cosf(pitch_radians);
    runtime->pitch_sin = sinf(pitch_radians);
}

void fall_pipeline_begin_frame(
    FallPipelineRuntime_t *runtime,
    uint32_t frame_index,
    uint32_t timestamp_ms)
{
    if (runtime == NULL) {
        return;
    }
    runtime->frame_index = frame_index;
    runtime->timestamp_ms = timestamp_ms;
    runtime->point_count = 0;
    runtime->cluster_count = 0;
}

bool fall_pipeline_add_point(FallPipelineRuntime_t *runtime, const FallInputPoint_t *input)
{
    FallPoint_t point;
    float world_y;
    float world_z;

    if ((runtime == NULL) || (input == NULL)) {
        return false;
    }

    world_y = input->y_cm * runtime->pitch_cos + input->z_cm * runtime->pitch_sin;
    world_z = input->z_cm * runtime->pitch_cos - input->y_cm * runtime->pitch_sin +
              runtime->config.mount_height_cm;
    point.x_cm = fall_round_i16(input->x_cm);
    point.y_cm = fall_round_i16(world_y);
    point.z_cm = fall_round_i16(world_z);
    point.velocity_cm_s = input->velocity_cm_s;
    point.snr_db = input->snr_db;
    point.is_motion = input->is_motion;

    if (!fall_point_is_in_roi(runtime, &point)) {
        return false;
    }
    if (runtime->point_count >= FALL_MAX_POINTS) {
        runtime->point_overflow_count++;
        return false;
    }
    runtime->points[runtime->point_count++] = point;
    return true;
}

FallTarget_t fall_pipeline_finish_frame(FallPipelineRuntime_t *runtime)
{
    FallTarget_t target;

    memset(&target, 0, sizeof(target));
    if (runtime == NULL) {
        target.state = FALL_TRACK_LOST;
        return target;
    }

    fall_cluster_points(runtime);
    return fall_track_frame(runtime);
}

uint16_t fall_pipeline_cluster_count_get(const FallPipelineRuntime_t *runtime)
{
    return (runtime == NULL) ? 0U : runtime->cluster_count;
}

uint32_t fall_pipeline_overflow_count_get(const FallPipelineRuntime_t *runtime)
{
    return (runtime == NULL) ? 0U :
           runtime->point_overflow_count + runtime->cluster_overflow_count;
}
