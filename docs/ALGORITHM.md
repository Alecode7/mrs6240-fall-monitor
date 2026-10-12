# Board-Side Fall Decision Strategy

## Scope

The fall decision runs on the MRS6240-P2512 board. The real-time PC application only parses `FALL` status frames and `FALL_EVENT` events, presents the alarm, records evidence, and visualizes raw point clouds. It does not replace, vote on, or override the board-side decision.

Core implementation files:

- `Falldown/src/fall_pipeline.c`: coordinate conversion, ROI filtering, clustering, single-target association, temporary hold, and target reacquisition.
- `Falldown/src/fall_rule.c`: state machine, height history, candidate/suspect/confirmed transitions, and recovery.
- `Falldown/src/prj_config.h`: current deployment and rule thresholds.

The current public configuration targets a ceiling-down installation: default mount height `310 cm`, pitch `90 deg`, and ROI `x=-250..250 cm`, `y=-250..250 cm`, `z=0..220 cm`. `fall_height <cm>` changes the persisted mount height only. Pitch, ROI, and decision thresholds remain build-time settings and must be validated again after a deployment change.

## Processing Pipeline

```text
motion + presence point clouds
  -> coordinate conversion and ROI/SNR filtering
  -> radius clustering and body-size filtering
  -> one selected target with short loss hold / controlled reacquisition
  -> height and motion history
  -> NORMAL -> CANDIDATE -> SUSPECTED -> CONFIRMED
  -> sustained stand-up evidence -> NORMAL
```

Both point-cloud streams are used. Motion points expose movement and speed near the fall, while presence points preserve a low, still body after motion subsides. The decision uses the selected target rather than any individual raw point.

## State Transitions

### 1. Candidate

A candidate is created only when all of the following are true:

- In the preceding `3500 ms`, the selected target has fallen at least `80 cm` from its recent high point.
- Recent samples show fall-related motion: absolute speed reaches `45 cm/s` or motion-point count reaches `20`.
- The target is in a low posture: height is no greater than `55 cm`, and at least `92%` of the preceding `700 ms` height history is also at or below that level.

This prevents a single low point or a normal slow posture change from immediately becoming an alarm candidate.

### 2. Suspected fall

An active candidate enters `SUSPECTED` only when the target remains fresh and terminally low:

- Current target height is no greater than `40 cm`.
- At least three fresh target samples cover `700 ms`.
- At least `92%` of that fresh-sample window is no greater than `40 cm`.

Brief tracker hold periods are tolerated for up to `800 ms`; a longer interruption clears continuity evidence rather than stitching unrelated targets together.

### 3. Confirmed fall

`CONFIRMED` is emitted only after the suspected target continues to satisfy all confirmation evidence:

- At least `1200 ms` has elapsed since the suspected event.
- At least six fresh samples cover the confirmation window.
- The target remains at or below `40 cm` for at least `70%` of the window.
- The target has moved at least `30 cm` horizontally from the pre-fall anchor at some point in the candidate period.

The event reason records whether the final posture is still (`LOW_STILL`) or remains active (`LOW_ACTIVE`). The alarm stays in `CONFIRMED`; it does not disappear after one frame.

### 4. Recovery

The system returns to `NORMAL` only after fresh, sustained standing evidence:

- Target height reaches at least `85 cm` for `1000 ms`.
- The standing target stays within `80 cm` of the confirmed low-position center, or within `100 cm` of the pre-fall standing anchor.

The two recovery anchors handle backward/forward falls: a person may stand up near the original leg position instead of the low center of the fallen posture. They also prevent a distant high background cluster from clearing an active alarm.

## Tracking Guardrails

The pipeline rejects weak/background clusters before they reach the state machine. It uses a `45 cm` clustering radius, minimum cluster size of three points, normal association gate of `50 cm`, lateral reacquisition gate of `85 cm` for low targets, and a `3000 ms` tracking hold. A far candidate must remain stable for five frames before replacing the tracked target.

## Important Limits

This is a single-target, rule-based embedded algorithm, not a machine-learning classifier and not a certified medical or safety device. Thresholds above were tuned against the current installation and test actions. Changing room geometry, radar height, pitch, ROI, or target population requires new regression data and test verification; thresholds must not be copied to a new deployment without validation.
