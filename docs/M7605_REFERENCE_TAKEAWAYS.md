# M7605 Reference Takeaways

## Status

The local M7605 module and its Qt client were used only as a black-box reference. The module and client were reported as successfully run by the operator. No M7605 binary frame parser, command code, Qt source, or deployment thresholds are copied into this project.

MRS6240 keeps its own transport and protocol contracts:

- Control UART: text `FALL` status frames, `FALL_EVENT` events, and text configuration commands.
- Point cloud: the SDK/HIF SPI data path.
- Decision authority: the MRS6240 board; the PC never re-votes or overrides a fall decision.

## What was absorbed

### 1. Make link and parser quality visible

The reference client surfaces invalid frames with a reason and a short raw sample instead of silently dropping them. The same principle is now applied to the MRS6240 text control link:

- `SerialLogReader` keeps a thread-safe diagnostic snapshot.
- It counts received lines, `FALL` status frames, `FALL_EVENT` events, unrelated console lines, and lines that required Unicode replacement characters.
- The engineering serial tab shows `RX`, `FALL`, `EVENT`, other-line, decode-loss, and status-frame ratio.
- A compact diagnostic line is written to the session log every 10 seconds while the control port is connected.

This does not change command encoding or event semantics. It makes a noisy console, a stalled firmware loop, or a baud/encoding problem visible during field diagnosis.

### 2. Keep an explicit capability boundary

The reference project uses a support matrix to distinguish enabled commands from protocol placeholders that the firmware does not currently implement. The equivalent MRS6240 boundary is:

| Capability | Current MRS6240 contract |
| --- | --- |
| Board heartbeat/status | Continuous `FALL frame=... state=...` text frames |
| Realtime alarm | Board-emitted `FALL_EVENT state=suspected/confirmed/recovered` |
| Persistent deployment value | `fall_height <cm>` and `FALL_HEIGHT` replies |
| Build-time deployment echo | `fall_config` / `FALL_CFG` |
| PC-side visualization | SPI/HIF point clouds and board-target association for diagnosis only |
| PC-side decision | Not supported; the board is the only realtime decision authority |

A feature should remain hidden or shown as an explicit diagnostic state until the board firmware defines a stable reply contract and tests are added on both sides.

### 3. Preserve queryable deployment state

The reference client queries parameters after connecting instead of relying only on startup messages. MRS6240 already follows this pattern with `fall_config`, and the monitor uses the reply for deployment readiness. This remains a required pattern after any firmware configuration change.

### 4. Separate operator and engineering surfaces

The reference client keeps unsupported or risky maintenance functions out of the normal operator flow. MRS6240 preserves the same separation: the simplified monitor exposes monitoring, alarm, health, and deployment state; raw point-cloud, serial console, threshold, and debug controls remain in the engineering UI.

## Deliberately not adopted

- The M7605 `53 59` business framing and `55 AA` cloud framing.
- M7605 point scaling, angle conversion, and sign-magnitude trajectory fields.
- M7605 installation-height, ROI, presence, track, and OTA parameter ranges.
- Any PC-side fall classifier or trajectory-based alarm override.

These values and formats were validated only for the M7605 module. Mixing them into MRS6240 would create a false appearance of protocol compatibility.

## Future backlog

- Add a board identity/version query only after defining a stable text command and reply.
- Add explicit fault-code reporting if firmware can distinguish SPI callback, clustering, flash, and UART faults.
- Consider a compact target-history view only if field testing shows that the existing point timeline and target association are insufficient.

## Regression requirement

Changes to serial parsing or diagnostics must continue to exercise:

- `FALL` status parsing.
- `FALL_EVENT` parsing and duplicate suppression.
- TX CRLF encoding.
- RX decode fallback.
- Serial diagnostics classification and throttling.
- Batched UI polling.
