# PhotonVision vision-to-pose audit progress

Audit root commit: `0e834f0851584237b0dd0913381b52fe6110dd08` (main, 2026-05-16)

Status legend: `[ ]` not started · `[~]` in progress (with timestamp) · `[x]` done (with one-line summary)

## Stage 1 — Capture timestamp anchoring
Start-of-exposure vs mid-exposure across the V4L2 driver, `Frame::capture_timestamp_ns`,
`PhotonPipelineMetadata.captureTimestampMicros`. Cross-reference PR #2489 + driver #33 in memory.

- [x] 1.1 trace one capture_timestamp value from V4L2 driver → Frame → PhotonPipelineMetadata — 1 DIVERGENCE (getter Javadoc lies about timebase); mid-exposure absence is pre-found (PR #2489).
- [x] 1.2 verify mid-exposure correction is applied (or documented as not applied) on each port — clean: absence is pre-found (PR #2489), design doc admits it explicitly (e2e-latency.md:38), Java/Python sim both apply `capture = publish - latency` consistently.
- [x] 1.3 VisionRunner disconnect path publishes `captureTimestampMicros = 0 + offset` — confirmed; 1× SMELL filed. Empty `targets` list shields well-behaved consumers; UI latency widget shows coproc-uptime.
- [x] 1.4 LibcameraGpuFrameProvider two-clock race — clean: ~1–10µs positive bias bounded by JNI overhead (NIT); anchor question folds under pre-found PR #2489.

## Stage 2 — Per-tag PnP
`bestCameraToTarget` / `alternateCameraToTarget` definitions, the ambiguity score, OpenCV
IPPE_SQUARE inputs, reprojection-error sign conventions.

- [x] 2.1 IPPE_SQUARE object-point ordering vs OpenCV contract — OpenCVHelp path consistent (same reorder applied to 3D+2D); ArucoPoseEstimatorPipe path's two swap-pair reorders cancel — correctness hinges on OpenCV ArUco's actual return order, which conflicts between docs and PV's comments. **Needs runtime verification — no finding filed.**
- [x] 2.1a runtime-verified `cv2.aruco.detectMarkers` returns `[TL, TR, BR, BL]` (per OpenCV docs, not per PV comment); two contradictory swap-pairs cancel → path is correct-by-coincidence; **1× SMELL filed** for misleading comments.
- [x] 2.2 ambiguity = err1/err2 — direction OK; no denominator-zero/NaN guard on errors[1]; NaN poisons AVERAGE_BEST_TARGETS — 1× SMELL filed.
- [x] 2.2a C++/Python parity confirmed — both have identical NaN-retry-on-errors[0] loop as Java; existing 2.2 SMELL applies equally to all three ports, no port-specific divergence.
- [x] 2.3 reprojection error sign/units — pixels (declared + matches OpenCV's RMS); units OK across IPPE/SQPNP; **CONSTRAINED_SOLVEPNP returns 0**: 1× SMELL filed. Python port lacks CONSTRAINED_SOLVEPNP entirely.
- [x] 2.3a Python lacks CONSTRAINED_SOLVEPNP — rolled into consolidated DIVERGENCE (Python strategy coverage gap).
- [x] 2.4 bestCameraToTarget orientation — clean. Name matches semantics (target's pose in camera frame). All paths (AprilTag/Aruco/SolvePNP/OpenCVHelp) converge to NWU + tag-Z-out via deliberate but split-across-files EDN→NWU conversions. Consumer-side math at PhotonPoseEstimator inverts correctly.

## Stage 3 — Multi-tag SQPnP on coprocessor
Object/image point pairing order, the OpenCV ↔ wpilib frame conversion (`detail::ToPose3d` /
`ToPoint3d`), ambiguity reporting.

- [x] 3.1 object/image point pairing preserves index correspondence — clean. knownTags + corners populated in lockstep; TargetModel.vertices order [BL,BR,TR,TL] matches AprilTag detector + ArUco wrapper output order. solvePNP_SQPNP applies no internal reordering.
- [x] 3.2 OpenCV ↔ wpilib frame conversion — clean. Conversion matrices byte-identical across Java/C++/Python. Math is correct (NWU↔EDN similarity transform); handedness preserved. Round-trip NWU→EDN→NWU symmetric at SQPnP boundaries. Adjacent SMELL noted: two parallel implementations (PV matrix vs WPILib CoordinateSystem) — numerically identical today.
- [x] 3.3 multi-tag ambiguity reported (or stubbed) — clean. Always 0 by design (OpenCV SQPnP is single-solution); matches the documented contract "no alt → 0". All three ports consistent. Consumer-side MULTI_TAG_PNP_* strategies don't gate by ambiguity (use raw best transform).

## Stage 4 — NT packet serialization
`PhotonPipelineMetadata` field units (µs? ms? FPGA? coproc-clock?), docs vs code.

- [x] 4.1 captureTimestampMicros — clean. Declared unit (µs) matches encoded value (µs); byte order is little-endian int64 on all three ports; schema-version md5 hash protects against silent renames. Timebase issue already filed at 1.1 (DIVERGENCE).
- [x] 4.2 publishTimestampMicros — clean on unit/encoding (µs, int64 LE, same path as 4.1). Surfaced **DIVERGENCE**: Java `getTimestampSeconds` uses absolute capture timestamp, C++/Python use receive-minus-latency — filed; plus correction to 1.1 (Python has the same misleading "coprocessor timebase" comment).
- [x] 4.3 sequenceID / timeSinceLastPong fields — clean. sequenceID is unit-less int64 frame counter; timeSinceLastPong is int64 µs per field comment. Sentinel inconsistency (default `Long.MAX_VALUE`, server-mode `0`) is NIT and harmless in practice.
- [x] 4.4 cross-port serializer agreement — clean. Identical MESSAGE_VERSION md5 + MESSAGE_FORMAT + field order + little-endian int64 byte order across Java/C++/Python; single source of truth (messages.yaml + generate_messages.py). Stage 4 closed.

## Stage 5 — NT4 time-sync to FPGA
Server-time offset path, coproc-clock → RIO-FPGA conversion, behavior under offset jumps
mid-match.

- [x] 5.1 server-time offset acquisition + caching — 1× SMELL filed. `getOffset()` returns `0` indistinguishably for server-mode and fresh/reconnecting client-mode. Transient ~1–2s window after startup or NT reconnect publishes wrong timestamps; can poison wpilib pose estimator if coproc/server boot offset is within ~1.5s.
- [x] 5.2 coproc-clock → RIO-FPGA conversion math — **1× BLOCKER filed**. C++ `GetAllUnreadResults` double-subtracts latency: `SetReceiveTimestamp(value.time - latency)` + `GetTimestamp()` subtracts again → C++ timestamps off by one full latency (~16ms@60fps). Java & Python correct. Also filed correction to 4.2 (its "minor numerical difference" framing understated this bug).
- [x] 5.3 behavior under offset jumps mid-match — no new finding. Root cause is the 5.1 SMELL (fresh per-frame offset read, no smoothing); large offset jumps can produce non-monotonic captureTimestampMicros. 5.2 BLOCKER's C++ −latency offset is independent and additive. Stage 5 complete.

## Stage 6 — `PhotonPipelineResult.getTimestampSeconds()`
What does it actually return? `receive - latency`, or `capture - serverOffset`? Document the
math, then verify the implementation matches.

- [x] 6.1 document the intended math from docs/comments — synthesis: all three ports' docs promise "T_capture in robot FPGA-time seconds". Java/C++ docs are abstract ("Time Sync Server's time base"), Python is explicit ("receive - latency"). Implementation divergence captured by 4.2 DIVERGENCE + 5.2 BLOCKER + 5.1 SMELL.
- [x] 6.2 verify implementation matches (Java/C++/Python) — synthesis: Java + Python match intent within error bounds (µs / network-jitter respectively); C++ modern API DOES NOT (5.2 BLOCKER, off by ~latency). C++ deprecated `GetLatestResult` is correct. No new finding; consolidates 4.2 + 5.1 + 5.2 + 1.1. Stage 6 complete.

## Stage 7 — Pose-strategy math
`PNP_DISTANCE_TRIG_SOLVE` sign conventions (pitch/yaw), AVERAGE_BEST_TARGETS weighted-rotation
math (rotating by a scaled Rotation3d is **not** the same as quaternion slerp),
`CONSTRAINED_SOLVEPNP` JNI contract.

- [x] 7.1 PNP_DISTANCE_TRIG_SOLVE pitch/yaw sign conventions — clean. PV's "yaw positive right + pitch positive up" maps correctly to `Rotation3d(0, -pitchRad, -yawRad)`; verified by hand-derivation on synthetic case. All three ports' math is byte-equivalent. Filed adjacent 1× DIVERGENCE: Python EstimatedRobotPose dataclass lacks the strategy field that Java/C++ carry.
- [x] 7.2 AVERAGE_BEST_TARGETS weighted-rotation math — **1× SMELL filed.** Iterative `rotation.rotateBy(R_i.times(w_i))` is a non-commutative quaternion product, order-dependent. Hand-derived counter-example shows two equal-weight 90° rotations about orthogonal axes produce different results based on iteration order. Java + C++ identical; Python doesn't implement this strategy at all.
- [x] 7.2a Python lacks AVERAGE_BEST_TARGETS — rolled into consolidated DIVERGENCE (Python strategy coverage gap).
- [x] 7.3 CONSTRAINED_SOLVEPNP JNI contract — clean. Same param order Java↔C++; Eigen handles the Map<RowMajor> → Matrix<ColMajor> conversion correctly (element-by-element, mathematical equivalence preserved); error path (null on failure) propagates. 2.3 SMELL about discarded residual cost still applies (separate concern). Stage 7 effectively complete.
- [x] 7.4 `targetsUsed` shape — addressed by PR #2493 (do not re-catalog)
- [x] 7.5 C++ no-winner null-return for closest-to-reference-pose — addressed by PR #2493
- [x] 7.6 C++ LOWEST_AMBIGUITY `-1` guard — addressed by PR #2494

## Stage 8 — stddev gap
`EstimatedRobotPose` has no uncertainty; reprojection error from PnP could provide one.
Document the gap.

- [x] 8.1 confirm absence of stddev on EstimatedRobotPose across all three ports — verified absent on all three ports (Java, C++, Python). Structural deliberate gap; not a finding per audit prompt's "document the gap" framing. Note also captures the 7.1 DIVERGENCE (Python's missing strategy field).
- [x] 8.2 document candidate sources (reprojection error, multi-tag agreement) — done. Inventory of signals already on wire (poseAmbiguity, distance, multitag bestReprojErr, tag count). PV's deliberate choice to leave stddev to consumer is reasonable; structural gap is "no aggregate quality field on EstimatedRobotPose". Stage 8 complete.

## Stage 9 — RIO integration
Example projects' `addVisionMeasurement` call sites, do they assume the timestamp is already
FPGA-relative?

- [x] 9.1 enumerate example projects calling addVisionMeasurement — done. Java (3 examples), C++ (1), Python (1). All pass est.timestampSeconds directly assuming FPGA-relative. **5.2 BLOCKER's blast radius now includes the canonical C++ example** — every C++ user following the example inherits the ~16ms latency offset.
- [x] 9.2 verify each treats getTimestampSeconds() as FPGA-relative — verified. Java passes T_capture (correct), Python passes T_capture+network_delay (correct within ms), C++ passes T_capture−latency (wrong; 5.2 BLOCKER, not example bug). WPILib's contract is "FPGA-relative seconds matching Timer.getFPGATimestamp()". Stage 9 complete.

---

## Iteration log

- `2026-05-16T<bootstrap>` — bootstrap: created audit scaffolding, no check performed.
- `2026-05-16T04:16-04:00` — check 1.1: end-to-end capture-timestamp trace; filed 1× DIVERGENCE (getCaptureTimestampMicros Javadoc); added sub-checks 1.3, 1.4.
- `2026-05-16T04:51-04:00` — check 1.2: mid-exposure correction status; clean (absence is pre-found PR #2489; design doc documents the limitation).
- `2026-05-16T05:25-04:00` — check 1.3: VisionRunner disconnect-path zero timestamp; 1× SMELL filed (published value looks valid; targets=[] shields consumers).
- `2026-05-16T06:00-04:00` — check 1.4: libcamera two-clock race; clean (bias ≤ JNI overhead). Stage 1 complete.
- `2026-05-16T06:34-04:00` — check 2.1: IPPE_SQUARE corner ordering. OpenCVHelp path is self-consistent. ArucoPoseEstimatorPipe path correctness depends on OpenCV ArUco's true return order — PV comments contradict OpenCV docs. No finding (needs runtime). Added 2.1a.
- `2026-05-16T07:10-04:00` — check 2.2: ambiguity ratio. Direction correct (best/alt, lower wins, matches LOWEST_AMBIGUITY). Missing NaN guard on errors[1]: 1× SMELL filed; AVERAGE_BEST_TARGETS poisons on NaN. Added 2.2a.
- `2026-05-16T07:44-04:00` — check 2.3: reprojection error sign/units. Pixels declared and matches OpenCV's RMS for IPPE/SQPNP. CONSTRAINED_SOLVEPNP returns default `bestReprojErr=0`: 1× SMELL filed. Added 2.3a (Python parity gap).
- `2026-05-16T08:18-04:00` — check 2.4: bestCameraToTarget orientation. Clean. All paths converge to NWU camera frame + tag-Z-out-of-face. EDN→NWU conversions are split across pipeline files but consistent.
- `2026-05-16T08:54-04:00` — check 3.1: multi-tag SQPnP point pairing. Clean. Index correspondence preserved end-to-end; TargetModel vertex order [BL,BR,TR,TL] matches both AprilTag and ArUco detector output order.
- `2026-05-16T09:28-04:00` — check 3.2: NWU↔EDN frame conversion. Clean. Matrices byte-identical across all three ports; conjugation formulas all correct. Adjacent SMELL: two parallel paths (PV matrices vs WPILib CoordinateSystem) — no observable divergence today.
- `2026-05-16T10:02-04:00` — check 3.3: multi-tag SQPnP ambiguity. Clean. Always 0 because OpenCV SQPnP is single-solution by design; matches "no alt → 0" contract. Stage 3 complete.
- `2026-05-16T10:35-04:00` — check 4.1: captureTimestampMicros unit/encoding. Clean. µs declared and encoded; little-endian int64 across all three ports; md5 schema-hash guards against silent renames. Timebase issue covered by existing 1.1 DIVERGENCE.
- `2026-05-16T11:08-04:00` — check 4.2: publishTimestampMicros. Clean on unit/encoding (same as 4.1). **Found big Java↔C++/Python DIVERGENCE in `getTimestampSeconds`** (Java: absolute capture; C++/Python: receive-minus-latency) — filed. Also filed correction to 1.1 (Python has misleading "coprocessor timebase" comment too).
- `2026-05-16T11:43-04:00` — check 4.3: sequenceID / timeSinceLastPong units. Clean (unit-less int64 counter / int64 µs). Adjacent NIT: sentinel inconsistency on timeSinceLastPong (Long.MAX_VALUE in default, 0 in server-mode) but harmless.
- `2026-05-16T12:16-04:00` — check 4.4: cross-port serializer agreement. Clean. md5 hash + format string + field order + LE byte order all match across Java/C++/Python. Stage 4 complete.
- `2026-05-16T12:49-04:00` — check 5.1: server-time offset acquisition + caching. 1× SMELL filed (transient-zero offset publishes wrong timestamps during startup / NT reconnect window).
- `2026-05-16T13:22-04:00` — check 5.2: coproc → FPGA conversion math. **1× BLOCKER filed.** C++ `GetAllUnreadResults` double-subtracts latency → vision timestamps off by ~16ms@60fps; pose estimate systematically lags one frame on C++. Also filed correction to 4.2.
- `2026-05-16T13:57-04:00` — check 5.3: offset jumps mid-match. No new finding (covered by 5.1 SMELL + 5.2 BLOCKER); note that large jumps can produce non-monotonic captureTimestampMicros. Stage 5 complete.
- `2026-05-16T14:30-04:00` — check 6.1: documented intent of getTimestampSeconds. All three ports promise "T_capture in robot FPGA-time"; Python's docstring is explicit, Java/C++ abstract. No new finding; consolidates 4.2 + 5.2 + 5.1.
- `2026-05-16T15:02-04:00` — check 6.2: verify implementation vs intent. Java + Python match; C++ modern API doesn't (5.2 BLOCKER). Stage 6 complete.
- `2026-05-16T15:34-04:00` — check 7.1: PNP_DISTANCE_TRIG_SOLVE sign conventions. Clean on signs; all three ports byte-equivalent. Filed 1× DIVERGENCE (Python EstimatedRobotPose lacks `strategy` field).
- `2026-05-16T16:09-04:00` — check 7.2: AVERAGE_BEST_TARGETS weighted-rotation math. 1× SMELL filed: iterative `rotateBy(R.times(w))` is a non-commutative quaternion product, order-dependent. Python doesn't implement this strategy. Added 7.2a.
- `2026-05-16T16:43-04:00` — check 7.3: CONSTRAINED_SOLVEPNP JNI contract. Clean. Stage 7 effectively complete (7.4-7.6 pre-found).
- `2026-05-16T17:17-04:00` — check 8.1: stddev absence on EstimatedRobotPose. Verified absent across all three ports. Documentation note; no finding per audit prompt framing.
- `2026-05-16T17:50-04:00` — check 8.2: stddev candidate sources inventory. Per-strategy recipe table; PV's choice to leave to consumer is reasonable but structural gap noted. Stage 8 complete.
- `2026-05-16T18:22-04:00` — check 9.1: example projects enumerated (Java/C++/Python). All assume FPGA-relative timestamp. C++ poseest example unknowingly inherits 5.2 BLOCKER — escalates user-visible impact.
- `2026-05-16T18:55-04:00` — check 9.2: FPGA-relative timestamp assumption verified per port. Java + Python correct; C++ wrong (5.2 BLOCKER, not example bug). Stage 9 complete. Main checks done; only sub-checks (2.1a runtime, 2.2a/2.3a/7.2a Python parity) remain.
- `2026-05-16T19:30-04:00` — batch-close 2.1a/2.2a/2.3a/7.2a: ran ArUco corner-order runtime verifier (Python cv2 4.12.0 available, scratch test under `audit/scratch/`). Resolved 2.1a (corners are [TL,TR,BR,BL]; comments wrong; path correct-by-cancellation — **1× SMELL filed**). Confirmed 2.2a C++/Python NaN-retry parity (no new finding). Confirmed 2.3a + 7.2a (Python lacks both CONSTRAINED_SOLVEPNP and AVERAGE_BEST_TARGETS, plus 4 more strategies) — **1× DIVERGENCE filed** for Python coverage gap. All sub-checks now closed. AUDIT COMPLETE.
