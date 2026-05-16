# PhotonVision vision-to-pose audit — SUMMARY

- Audit root commit: `0e834f0851584237b0dd0913381b52fe6110dd08` (main, 2026-05-16)
- 27 working notes + 1 batch-closure note under `audit/notes/`
- 1 runtime scratch verifier under `audit/scratch/` (gitignored)
- 9 stages, 27 checks + 4 sub-checks (all closed)

## Findings by severity

| Severity   | Count | Notes                                         |
| ---------- | ----- | --------------------------------------------- |
| BLOCKER    | **1** | Silent wrong outputs in production for C++ users |
| DIVERGENCE | 4     | Cross-port or contract-vs-code disagreements  |
| SMELL      | 6     | Suspicious; not proven wrong, but fragile     |
| CORRECTION | 2     | Amend earlier findings; not standalone bugs   |
| NIT        | 0 (filed as adjacent observations in notes; none promoted) |

Pre-found (already PR'd, not catalogued):
- PR #2493 — pose-estimator `targetsUsed` leak + null-return on closest-to-reference-pose
- PR #2494 — C++ `LOWEST_AMBIGUITY` `-1` guard
- PR #2489 — mid-exposure timestamp anchor (driver #33 + PV)

## Top finding

**BLOCKER — C++ `GetAllUnreadResults` double-subtracts processing latency**
[photon-lib/src/main/native/cpp/photon/PhotonCamera.cpp:242](photon-lib/src/main/native/cpp/photon/PhotonCamera.cpp#L242)

`SetReceiveTimestamp(value.time - result.GetLatency())` writes `T_capture` to the
`ntReceiveTimestamp` field. Then `PhotonPipelineResult::GetTimestamp()` returns
`ntReceiveTimestamp - GetLatency()`, subtracting latency a second time. Net result:
every C++ vision timestamp is off by one full processing latency (~16 ms at 60 fps).
Java and Python ports are correct.

User-visible impact: a C++ team following the canonical poseest example fuses vision
measurements stamped ~16 ms earlier than the actual capture. WPILib's
`SwerveDrivePoseEstimator` interpolates to that earlier moment, producing a systematic
~one-frame lag in the vision-fused pose under any motion. The pose error grows with robot
speed: at 3 m/s linear that's ~5 cm; at 2π rad/s rotational that's ~6° heading error.

Fix is one line: in `PhotonCamera.cpp:242-243`, either pass `value.time` un-modified to
`SetReceiveTimestamp` and let `GetTimestamp` subtract latency once, OR pass
`value.time - latency` and remove the subtraction in `GetTimestamp`. Java's idiom is to
store the absolute capture timestamp directly — that's the cleanest target shape.

## Other notable findings

1. **DIVERGENCE — Java vs C++/Python `getTimestampSeconds` semantics**
   ([PhotonPipelineResult.java:177](photon-targeting/src/main/java/org/photonvision/targeting/PhotonPipelineResult.java#L177)).
   Java returns `captureTimestampMicros / 1e6` (absolute capture). C++ and Python use
   `receive - latency`. Java + Python *happen* to converge to the same physical instant
   (T_capture) on the robot; C++ does not (BLOCKER). The three ports should agree on
   semantics, not just on output value through coincidence.

2. **DIVERGENCE — Python `PhotonPoseEstimator` implements only 3 of 9 strategies**
   ([photonPoseEstimator.py:39-271](photon-lib/py/photonlibpy/photonPoseEstimator.py#L39-L271)).
   Missing: AVERAGE_BEST_TARGETS, CONSTRAINED_SOLVEPNP, MULTI_TAG_PNP_ON_RIO, all three
   CLOSEST_TO_* strategies. No `PoseStrategy` enum, no `getEstimatedPose()` dispatcher.
   Users porting code between languages discover the gap at integration time.

3. **DIVERGENCE — `getCaptureTimestampMicros()` Javadoc + Python comment lie about timebase**
   ([PhotonPipelineMetadata.java:62](photon-targeting/src/main/java/org/photonvision/targeting/PhotonPipelineMetadata.java#L62)).
   Both claim "coprocessor timebase" but the published value is in **server timebase**
   (coproc-clock + TimeSyncClient offset). A user trusting the docs would build the wrong
   FPGA conversion. Python carries the same misleading comment.

4. **DIVERGENCE — Python `EstimatedRobotPose` lacks `strategy` field**
   ([estimatedRobotPose.py:25-36](photon-lib/py/photonlibpy/estimatedRobotPose.py#L25-L36)).
   Java + C++ carry which strategy produced the pose; Python does not. Underlying cause
   is the strategy-coverage DIVERGENCE above (no enum to put in the field).

5. **SMELL — `AVERAGE_BEST_TARGETS` rotation accumulation is non-commutative**
   ([PhotonPoseEstimator.java:739](photon-lib/src/main/java/org/photonvision/PhotonPoseEstimator.java#L739)).
   Iterative `rotation.rotateBy(R_i.times(w_i))` is a quaternion product — order-dependent
   and not a true weighted rotation average. Hand-derived counter-example with two equal-
   weight 90° rotations about orthogonal axes produces different results based on iteration
   order. Should use quaternion log-space weighted average (or simply commit to multi-tag
   PnP as the canonical multi-tag fusion).

## Stages with most surprises

- **Stage 5** (NT4 time-sync to FPGA) — yielded the audit's single BLOCKER. The trace
  through `PhotonCamera.cpp::GetAllUnreadResults` → `PhotonPipelineResult::GetTimestamp`
  → `addVisionMeasurement` revealed the double-subtraction by following one concrete
  value end-to-end.
- **Stage 7** (pose-strategy math) — confirmed the audit prompt's pre-stated suspicion
  about `rotateBy(R.times(scalar))` not being a quaternion average, plus surfaced the
  Python strategy-coverage gap that grew from "lacks strategy field" (a 7.1 DIVERGENCE)
  to "lacks 6 strategies" (the 7.2a closure DIVERGENCE).
- **Stage 4** (NT serialization) — looked routine on the surface but check 4.2 surfaced
  the Java↔C++/Python `getTimestampSeconds` semantic divergence which then drove the
  Stage 5 BLOCKER discovery.

## Stages with least coverage / fragile audit conclusions

- **Stage 2 / 2.1** — runtime resolution of the ArUco corner order resolved the OpenCV
  contract question, but the audit only tested face-on geometry. ArUco's "TL" is
  determined by the marker's *internal* code-orientation, not its image position; a
  marker physically rotated 180° in the world would have OpenCV report its
  internal-TL at the image-BR. The current runtime test verifies face-on only. Full
  confidence on the IPPE pose path under arbitrary marker orientations needs either a
  rotated-marker test or a code reading of `cv::aruco::Dictionary::identify`. The
  audit's conclusion holds for the path's index-correspondence (which doesn't depend
  on rotation), but anyone touching the swap-pair code should validate against rotated
  markers before refactoring.
- **Stage 3 / 3.2** — the OpenCV↔WPILib frame conversion verified by byte-identity of
  matrices across three ports + a similarity-transform math check. Numerical drift
  from the *two parallel* paths (PV's hard-coded matrices vs WPILib's
  `CoordinateSystem`) is not observable today but is a latent fragility.
- **Stage 8** — entirely documentation-style. No findings filed because the audit
  prompt framed the stddev gap as "document the gap" rather than "find a bug". A
  future audit pass could promote the stddev gap to a DIVERGENCE if the wpilib
  contract for `addVisionMeasurement` evolves toward "stddev required."

## Recommended next actions

In priority order:

1. **Fix the 5.2 BLOCKER first.** C++ users of the canonical `poseest` example are
   shipping with the bug. One-line fix in
   [photon-lib/src/main/native/cpp/photon/PhotonCamera.cpp:242](photon-lib/src/main/native/cpp/photon/PhotonCamera.cpp#L242).
   Pair with the 4.2 DIVERGENCE fix (align all three ports on "absolute capture
   timestamp" as the wire semantics).

2. **Fix the 1.1 DIVERGENCE.** Update Javadoc + Python docstring on
   `captureTimestampMicros` to say "server timebase, RIO-relative after `TimeSyncClient`
   offset is applied" — same wording as the 5.2 fix lands. Trivial doc change but blocks
   future users from re-deriving the BLOCKER's root confusion.

3. **Add a NaN guard for `errors[1]`** in all three ports' `solvePNP_SQUARE` (2.2
   SMELL). One-line each: substitute `1.0` (fully ambiguous) when `errors[1] == 0 ||
   isnan(errors[1])` before the division. Prevents NaN poisoning of
   `AVERAGE_BEST_TARGETS`.

4. **Update misleading ArUco comments** (2.1a SMELL) — both `PhotonArucoDetector` and
   `ArucoPoseEstimatorPipe`. Optional: simplify the path by removing the two cancelling
   swap-pairs entirely. The comments lie about OpenCV's behavior and could mislead a
   future maintainer into "fixing" them and breaking the PnP.

5. **Re-think `AVERAGE_BEST_TARGETS` math** (7.2 SMELL) — switch to quaternion
   log-space weighted average, OR deprecate the strategy in favor of
   `MULTI_TAG_PNP_ON_COPROCESSOR` (which is mathematically equivalent for the
   multi-tag case and lacks the non-commutativity issue).

6. **Surface CONSTRAINED reprojection cost** (2.3 SMELL) — Sleipnir's solver returns a
   final cost; plumb it back through JNI into `PnpResult.bestReprojErr` so consumers
   can build stddev heuristics.

7. **Fix Python strategy coverage** (new DIVERGENCE, 7.2a closure) — port the missing
   six strategies, add the `PoseStrategy` enum + dispatcher. Highest user-impact
   defect after the BLOCKER but largest code change.

8. **Server-time offset smoothing** (5.1 SMELL) — `TimeSyncManager.getOffset()` should
   distinguish "no offset known yet" from "server mode (offset = 0)". A `is_ready` flag
   or `Optional<Duration>` return type would close the fragile boot/reconnect window.

## Audit close-out

All checks `[x]` in `audit/PROGRESS.md`. No new sub-items identified during the closure
iteration. The audit found one BLOCKER + four DIVERGENCEs + six SMELLs spanning all nine
stages, with the BLOCKER being a one-line fix and the DIVERGENCEs being a mix of one-line
documentation fixes and substantive API alignment work.

AUDIT COMPLETE
