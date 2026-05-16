# PhotonVision vision-to-pose audit — findings

Severity order: **BLOCKER** > **DIVERGENCE** > **SMELL** > **NIT**.

Audit root commit: `0e834f0851584237b0dd0913381b52fe6110dd08` (main, 2026-05-16).

> Already-PR'd issues (PR #2493, #2494) are deliberately excluded from this file per
> `memory/audit_prefound.md`. See `audit/notes/prefound-<topic>.md` for one-liners.

---

<!-- New findings appended below this line in iteration order -->

## [BLOCKER] C++ `GetAllUnreadResults` double-subtracts latency → vision timestamps off by one full processing latency — photon-lib/src/main/native/cpp/photon/PhotonCamera.cpp:242
**Stage:** 5 (surfaced during check 5.2; cross-references the Stage 6 DIVERGENCE filed at 4.2 — supersedes its framing).
**Reproducer:** Call `PhotonCamera::GetAllUnreadResults()` (the recommended modern API,
per the Javadoc that flags `GetLatestResult` as deprecated) on a robot in C++. Inspect
`result.GetTimestamp()` for any returned result. Compare against Java's
`PhotonPipelineResult.getTimestampSeconds()` for the same wire bytes. The C++ value is
systematically `≈ latency` (≈ 16 ms at 60 fps with typical processing) earlier than the
Java value. WPILib's `SwerveDrivePoseEstimator.addVisionMeasurement` doesn't reject the
measurement as stale (error << 1.5 s history buffer), so it gets blended into the pose
estimate against odometry ~one frame in the past.
**Expected:** Either store `ntReceiveTimestamp` as the receive time and let
`GetTimestamp` subtract latency once, OR store `ntReceiveTimestamp` as the capture-time
estimate and have `GetTimestamp` return it directly. The current code does **both**:
`SetReceiveTimestamp(value.time - latency)` pre-computes the capture-time, then
`GetTimestamp()` subtracts latency *again*. The deprecated `GetLatestResult` at
[PhotonCamera.cpp:183-210](photon-lib/src/main/native/cpp/photon/PhotonCamera.cpp#L183-L210)
correctly passes `now` (no pre-subtraction) so its `GetTimestamp` returns `now -
latency`, which is the intended formula. Java and Python both produce ≈ `T_capture`;
C++ produces `T_capture − latency`.
**Observed:** At
[PhotonCamera.cpp:240-243](photon-lib/src/main/native/cpp/photon/PhotonCamera.cpp#L240-L243):

    // TODO: NT4 timestamps are still not to be trusted. But it's the best we
    // can do until we can make time sync more reliable.
    result.SetReceiveTimestamp(wpi::units::microsecond_t(value.time) -
                               result.GetLatency());

then at
[PhotonPipelineResult.h:102-104](photon-targeting/src/main/native/include/photon/targeting/PhotonPipelineResult.h#L102-L104):

    wpi::units::second_t GetTimestamp() const {
        return ntReceiveTimestamp - GetLatency();
    }

Composition: `(value.time - latency) - latency = value.time - 2*latency`.
**Why it's wrong:** The intent (per `SetReceiveTimestamp`'s doc comment "FPGA timestamp
this result was Received by robot code") is that `ntReceiveTimestamp` holds the receive
time, and `GetTimestamp` subtracts latency once to get capture-time. But
`GetAllUnreadResults` pre-subtracts latency before calling `SetReceiveTimestamp`, so
`GetTimestamp`'s subtraction is now an extra (wrong) operation. The bug only affects the
recommended modern API; the deprecated `GetLatestResult` is correct because it passes
`now` without pre-subtraction. Cross-port: Java
([PhotonCamera.java:253-268](photon-lib/src/main/java/org/photonvision/PhotonCamera.java#L253-L268))
doesn't write `ntReceiveTimestamp` at all (it uses `captureTimestampMicros / 1e6` directly,
which is server-timebase and equivalent on the robot). Python
([photonCamera.py:139-152](photon-lib/py/photonlibpy/photonCamera.py#L139-L152))
writes `value.time` without pre-subtraction, so its single `getTimestampSeconds`
subtraction yields the correct single-latency formula.
**Confidence:** high — the math is unambiguous; Java and Python both produce the
intended T_capture while C++ produces T_capture − latency. The error is systematic and
of magnitude ~one full processing latency (≈ 10–50 ms in practice).
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [CORRECTION to 4.2 DIVERGENCE] The Java↔C++ getTimestampSeconds divergence is C++ bug, not "minor numerical difference"
The 4.2 finding described the Java↔C++/Python divergence as roughly equivalent under
perfect time-sync ("differs by ~network_delay (1–5 ms)"). After check 5.2, that framing
is **wrong**. Java and Python both compute ≈ T_capture (within their respective error
sources). C++ computes T_capture − latency, off by ~one full processing latency
(typically 10–50 ms, dwarfing network delay). The Java↔C++ disagreement is not a
design tradeoff; it's a C++ implementation bug surfaced by 5.2. The 4.2 finding stands as
a DIVERGENCE label for the unequal implementations, but its severity should be revised
upward in light of the 5.2 BLOCKER. C++ alone is at fault; Python's "receive minus
latency" approach is equivalent to Java's "absolute capture timestamp" approach up to
network delay.

## [DIVERGENCE] `getCaptureTimestampMicros()` Javadoc claims coprocessor timebase but value is server timebase — photon-targeting/src/main/java/org/photonvision/targeting/PhotonPipelineMetadata.java:62
**Stage:** 1 (surfaced while tracing capture timestamp), but really a Stage 4/6 doc bug.
**Reproducer:** A user (or LLM) reads the getter Javadoc, treats the returned `µs` value as
on the coprocessor's local clock, and tries to subtract `WPIUtilJNI.now()` from it (or feed
it as an FPGA timestamp). The number is actually on the **time-sync server** timebase, so
subtraction silently yields a wrong latency / wrong FPGA-frame time.
**Expected:** Field-level comment at [PhotonPipelineMetadata.java:25-26](photon-targeting/src/main/java/org/photonvision/targeting/PhotonPipelineMetadata.java#L25-L26)
states: "The timebase is wpi::nt::Now on the time sync server". NTDataPublisher at
[NTDataPublisher.java:186](photon-core/src/main/java/org/photonvision/common/dataflow/networktables/NTDataPublisher.java#L186)
explicitly adds the server offset: `captureMicros + offset`. The field comment + the code
agree.
**Observed:** Getter Javadoc at [PhotonPipelineMetadata.java:61-65](photon-targeting/src/main/java/org/photonvision/targeting/PhotonPipelineMetadata.java#L61-L65)
says: "The time that this image was captured, **in the coprocessor's time base.**"
[PhotonPipelineMetadata.java:69-75](photon-targeting/src/main/java/org/photonvision/targeting/PhotonPipelineMetadata.java#L69-L75)
likewise: "The time that this result was published to NT, in the coprocessor's time base."
**Why it's wrong:** The two contracts disagree about the timebase. The code path
demonstrably matches the *field* comment (server-side timebase). The *getter* Javadoc is
the public-facing API documentation and is what consumers will read — it is the source of
truth for anyone subclassing or using photon-lib. It's stale wording from before the NT4
time-sync server was introduced. Cross-port check: the C++ struct comment at
[PhotonPipelineMetadataStruct.h](photon-targeting/src/generated/main/native/include/photon/struct/PhotonPipelineMetadataStruct.h)
is generated from `photon-serde/messages.yaml` and inherits whatever wording is there; the
Python port has no Javadoc-equivalent. Java is the only port carrying the contradictory
prose.
**Confidence:** high — the field comment, the code, the offset semantics, and the
PhotonPipelineResult.getTimestampSeconds path all confirm server-timebase. The
"coprocessor's time base" prose is the outlier.
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [CORRECTION to 1.1] Python port also carries the "coprocessor timebase" comment — photon-lib/py/photonlibpy/targeting/photonPipelineResult.py:14-15
The 1.1 finding stated "the Python port has no Javadoc-equivalent." That was wrong.
Python at
[photonPipelineResult.py:14-15](photon-lib/py/photonlibpy/targeting/photonPipelineResult.py#L14-L15)
has the same misleading prose: `"Image capture and NT publish timestamp, in microseconds
and in the coprocessor timebase. As reported by WPIUtilJNI::now."` The wire value is
server-timebase (NTDataPublisher adds the time-sync offset), so the comment is
inconsistent with the actual value — same as the Java getter Javadoc. The Python file
mitigates this with an honest "Python users beware!" disclaimer at
[photonPipelineResult.py:33-34](photon-lib/py/photonlibpy/targeting/photonPipelineResult.py#L33-L34),
but the per-field comment itself remains misleading. The 1.1 finding's scope therefore
covers Python in addition to Java.

## [SMELL] `AVERAGE_BEST_TARGETS` rotation accumulation is an order-dependent quaternion product, not a true weighted rotation average — photon-lib/src/main/java/org/photonvision/PhotonPoseEstimator.java:739
**Stage:** 7
**Reproducer:** Set `PoseStrategy.AVERAGE_BEST_TARGETS`. Present two visible AprilTags whose individual PnP rotation estimates differ by more than a few degrees (e.g., one tag with very bad fit at the FOV edge plus one good tag at center). Reorder `cameraResult.targets` (e.g., via PhotonVision UI's target-sort mode change) — the returned `EstimatedRobotPose.rotation` shifts even though the per-target estimates are unchanged. Magnitude of shift grows with the disagreement between per-target rotations.
**Expected:** A "weighted average of rotations" should be order-independent — either Markley's quaternion-eigenvector average or the tangent-space (Lie-algebra) sum `exp(Σ wᵢ · log(qᵢ))` would satisfy this. Both also reduce to the correct answer in the "all-equal" and "small-rotation" cases.
**Observed:** [PhotonPoseEstimator.java:733-739](photon-lib/src/main/java/org/photonvision/PhotonPoseEstimator.java#L733-L739) and C++ at [PhotonPoseEstimator.cpp:444-449](photon-lib/src/main/native/cpp/photon/PhotonPoseEstimator.cpp#L444-L449) accumulate `rotation = rotation.rotateBy(estimatedPose.getRotation().times(weight))` in a loop. This composes to `q_N^{w_N} · q_{N-1}^{w_{N-1}} · ... · q_1^{w_1}` — a non-commutative quaternion product that depends on iteration order. Python doesn't implement this strategy at all.
**Why it's wrong:** `R.times(w)` scales the rotation angle by w (axis preserved), so `q^w` is well-defined. But `q_a · q_b ≠ q_b · q_a` for non-parallel rotations — the iterative product is therefore not a proper rotation mean. Hand-derivation in the working note shows that two equal-weight 90° rotations about orthogonal axes produce different results depending on which target's rotation enters the loop first; a true average should be the midpoint of the geodesic between the two rotations (order-independent). In the typical FRC scenario where all targets agree (multi-tag detection of a stationary robot), the math accidentally gives the correct answer because `q^{w_1} · q^{w_2} · ... = q^{Σwᵢ} = q^1 = q` (weights sum to 1). It also gives approximately the right answer for small-disagreement cases (BCH first-order). The bug manifests when one tag has a much-worse PnP fit than the others — exactly when the weighting is supposed to discount it but instead its rotation still contributes a fixed angular slice that depends on where it sits in the iteration.
**Confidence:** high — math is unambiguous; the iterative product is not a weighted average in the rotation-space sense.
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [DIVERGENCE] Python `EstimatedRobotPose` dataclass lacks the `strategy` field that Java + C++ have — photon-lib/py/photonlibpy/estimatedRobotPose.py:25-36
**Stage:** 7 (surfaced during check 7.1).
**Reproducer:** In Python: `estimate = photonPoseEstimator.estimatePnpDistanceTrigSolvePose(result)`. Try `estimate.strategy` — `AttributeError: 'EstimatedRobotPose' object has no attribute 'strategy'`. In Java: `estimate.strategy` is `PoseStrategy.PNP_DISTANCE_TRIG_SOLVE`. In C++: `estimate.strategy` is `PNP_DISTANCE_TRIG_SOLVE`. Python users have no way to programmatically determine which strategy produced a given EstimatedRobotPose — relevant if they wrap the strategy dispatcher and want to switch behavior on strategy in downstream code.
**Expected:** All three ports' EstimatedRobotPose should expose the same set of fields. Java's `EstimatedRobotPose(Pose3d, double, List, PoseStrategy)` and C++'s `EstimatedRobotPose{Pose3d, second_t, vector, PoseStrategy}` both carry the strategy enum.
**Observed:** Python's dataclass at [estimatedRobotPose.py:25-36](photon-lib/py/photonlibpy/estimatedRobotPose.py#L25-L36) has only `estimatedPose`, `timestampSeconds`, `targetsUsed`. All three call sites in [photonPoseEstimator.py](photon-lib/py/photonlibpy/photonPoseEstimator.py) at lines 189-191, 213-215, 257-259 pass only 3 args.
**Why it's wrong:** Java + C++ deliberately carry the strategy field in the returned pose (so the consumer's `addVisionMeasurement` glue or telemetry code can branch on strategy). Python's dataclass silently drops this field, so Python users get a strictly less informative API. The `update()` dispatcher pattern in the docs (line 44-45 of photonPoseEstimator.py says "using the strategy set below") becomes opaque to Python callers.
**Confidence:** high — the dataclass field list is unambiguous; the missing constructor argument is missing in all three Python call sites.
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [SMELL] `TimeSyncManager.getOffset()` returns `0` indistinguishably for "server mode" and "client mode, not yet converged" — photon-core/src/main/java/org/photonvision/common/dataflow/networktables/TimeSyncManager.java:66
**Stage:** 5
**Reproducer:** Boot a PhotonVision coprocessor; subscribe to the published
`captureTimestampMicros` field over NT during the first ~1–2 seconds after startup, or
during the first ~1–2 s after any NT reconnect (triggered by
`reportNtConnected` at [TimeSyncManager.java:146-154](photon-core/src/main/java/org/photonvision/common/dataflow/networktables/TimeSyncManager.java#L146-L154)).
Observe that `captureTimestampMicros` carries the coprocessor's local-clock µs rather
than the server-timebase µs that the field contract promises — because the freshly-(re)created
`TimeSyncClient` hasn't received a pong yet and `getOffset()` returns 0. If the
coproc-vs-server boot offset is large enough, downstream
`SwerveDrivePoseEstimator.addVisionMeasurement` silently drops the frame as stale; if the
offset is within wpilib's history buffer (~1.5 s), the bad timestamp is accepted and
contaminates the pose estimate.
**Expected:** `getOffset()` should either (a) block / return a sentinel until the first
successful ping-pong, (b) return a wide flag like `Long.MIN_VALUE` to signal "not
converged", or (c) NTDataPublisher should skip publishing when `getOffset()` is known to
be uninitialised. The current API implies "offset is always usable when this returns",
which isn't true during the transient window.
**Observed:** [TimeSyncManager.java:66-75](photon-core/src/main/java/org/photonvision/common/dataflow/networktables/TimeSyncManager.java#L66-L75):
`if (m_client != null) return m_client.getOffset();` — unconditional return. The same `0`
value means "server mode, no offset needed" (line 70), "fresh-client, no pongs received"
(implicit), and "both-client-and-server null" (the impossible state at line 73 with the
`// ?????` comment). [NTDataPublisher.java:186](photon-core/src/main/java/org/photonvision/common/dataflow/networktables/NTDataPublisher.java#L186)
then publishes `captureMicros + offset` unconditionally.
**Why it's wrong:** During NT reconnect or coprocessor startup, the published
`captureTimestampMicros` silently drops out of the server's timebase for ~1–2 seconds.
Within wpilib's vision-history buffer, the bad values can be applied to the pose
estimate. A robot observing pose-jumps shortly after NT reconnects would have a hard
time diagnosing this from the symptoms — the field still looks like valid microseconds.
**Confidence:** medium — the gap exists unambiguously in code; whether the JNI client's
`getOffset()` returns `0` (as I assume) or a sentinel before convergence is
implementation-defined and not auditable from this tree. Either way, PhotonVision doesn't
inspect the JNI's return value for "valid" sentinels, so any sentinel that fits in a
`long` would be silently combined with `captureMicros` and published.
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [DIVERGENCE] Java `getTimestampSeconds` uses absolute capture timestamp; C++/Python use receive-minus-latency — photon-targeting/src/main/java/org/photonvision/targeting/PhotonPipelineResult.java:177
**Stage:** 6 (surfaced during check 4.2; full analysis there).
**Reproducer:** Same `PhotonPipelineResult` byte stream is received by a Java robot and a
C++/Python robot using identical `addVisionMeasurement` glue. The two call sites get
different `getTimestampSeconds()` answers: Java returns
`metadata.captureTimestampMicros / 1e6`; C++/Python return `ntReceiveTimestamp - (publish
- capture)`. Under typical network delay (1–5 ms), the values differ by that much; under
imperfect time-sync (offset drift), the values diverge further.
**Expected:** Three ports' Javadoc/doxygen/docstring all say the **same** thing:
"Returns the estimated time the frame was taken, in the Time Sync Server's time base
(wpi::nt::Now)." The C++ doc at
[PhotonPipelineResult.h:98-99](photon-targeting/src/main/native/include/photon/targeting/PhotonPipelineResult.h#L98-L99)
even explicitly says "This is much more accurate than using GetLatency()" — implying
they're aware of the alternative and prefer the receive-minus-latency form.
**Observed:**
- Java [PhotonPipelineResult.java:177-179](photon-targeting/src/main/java/org/photonvision/targeting/PhotonPipelineResult.java#L177-L179):
  `return metadata.captureTimestampMicros / 1e6;`
- C++ [PhotonPipelineResult.h:102-104](photon-targeting/src/main/native/include/photon/targeting/PhotonPipelineResult.h#L102-L104):
  `return ntReceiveTimestamp - GetLatency();`
- Python [photonPipelineResult.py:43-53](photon-lib/py/photonlibpy/targeting/photonPipelineResult.py#L43-L53):
  `return (self.ntReceiveTimestampMicros - latency) / 1e6` where `latency = publish - capture`.
**Why it's wrong:** Identical documented contract, divergent implementations. The Java
implementation trusts the server-timebase capture timestamp (good when time-sync is
perfect, bad when it drifts). The C++/Python implementation trusts NT4's
`ntReceiveTimestamp` and the relative latency (robust to time-sync error, but inflated
by network delay). Either approach can be defended in isolation; having Java behave
**differently from C++/Python** is the issue. A team porting a robot codebase between
languages gets subtly different vision-measurement timestamps. Note: the comment "This
is much more accurate than using GetLatency()" in C++/Java docs implies the team
considered the question; the Java implementation may simply not have been updated to
match C++ when the receive-minus-latency form was adopted.
**Confidence:** high — code differs unambiguously; numerical difference is observable on
a robot.
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [SMELL] `CONSTRAINED_SOLVEPNP` returns `bestReprojErr = 0` — no quality metric surfaced — photon-targeting/src/main/java/org/photonvision/estimation/VisionEstimation.java:287
**Stage:** 2 (reprojection-error coverage), with downstream impact at Stage 7.
**Reproducer:** Use `PoseStrategy.CONSTRAINED_SOLVEPNP` with valid AprilTags. On a real
robot, call `getPnpResult().get().bestReprojErr` — value is always `0.0` regardless of how
well the optimizer fit, because the JNI `do_optimization` returns only `[x, y, theta]` and
the Java/C++ caller default-constructs a `PnpResult` then only assigns `best`.
**Expected:** Per
[PhotonPoseEstimator.java:111-113](photon-lib/src/main/java/org/photonvision/PhotonPoseEstimator.java#L111-L113):
"the cost function is a sum-squared of pixel reprojection error + (optionally) heading
error * heading scale factor." That residual is exactly the quality signal consumers
want — it should populate `bestReprojErr` (or a dedicated cost field), in pixels to match
the other PnP paths.
**Observed:** [VisionEstimation.java:286-288](photon-targeting/src/main/java/org/photonvision/estimation/VisionEstimation.java#L286-L288):
`var pnpresult = new PnpResult(); pnpresult.best = new Transform3d(...); return Optional.of(pnpresult);`
— `bestReprojErr`, `altReprojErr`, `ambiguity` all left at default `0`. Identical pattern
in C++ at [VisionEstimation.cpp:203-210](photon-targeting/src/main/native/cpp/photon/estimation/VisionEstimation.cpp#L203-L210).
**Why it's wrong:** The optimizer demonstrably computes a residual cost (it's the
optimization objective). Discarding it leaves consumers with no in-band way to detect
poor convergence: a high-residual converged solution is indistinguishable from a
perfect-fit solution. Both LOWEST_AMBIGUITY and MULTI_TAG_PNP_* strategies surface a
quality signal; CONSTRAINED_SOLVEPNP silently does not. Users who tune
`addVisionMeasurement` thresholds against one strategy get inconsistent behaviour when
switching to this one. Cross-port: Python does not implement CONSTRAINED_SOLVEPNP at all,
so this is a Java/C++ shared gap.
**Confidence:** high — the gap is unambiguous in code; the only judgement call is
whether "discard a residual the solver clearly computes" rises to a finding (it does,
because the strategy's documented purpose is precision-improvement, where a quality
metric is load-bearing).
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [SMELL] `ambiguity = errors[0] / errors[1]` has no denominator-zero / NaN guard; NaN poisons AVERAGE_BEST_TARGETS — photon-targeting/src/main/java/org/photonvision/estimation/OpenCVHelp.java:582
**Stage:** 2 (per-tag PnP) with downstream impact at Stage 7.
**Reproducer:** Construct (or wait for OpenCV to produce) a `solvePnPGeneric(SOLVEPNP_IPPE_SQUARE)`
call where `errors[0]` is finite but `errors[1]` is NaN. The retry-with-noise logic at
[OpenCVHelp.java:568-575](photon-targeting/src/main/java/org/photonvision/estimation/OpenCVHelp.java#L568-L575)
checks only `errors[0]`, so the NaN passes through. `PnpResult.ambiguity` is set to NaN.
A robot using `PoseStrategy.AVERAGE_BEST_TARGETS` then receives an `EstimatedRobotPose` with
NaN translation and rotation, because `1.0 / NaN = NaN` poisons `totalAmbiguity` at
[PhotonPoseEstimator.java:715](photon-lib/src/main/java/org/photonvision/PhotonPoseEstimator.java#L715)
and every subsequent weight at line 736.
**Expected:** Either skip the publish (treat NaN as invalid), substitute a sentinel
(e.g. `1.0` to express "fully ambiguous"), or extend the existing NaN-retry to also cover
`errors[1]`. Cross-port: C++
([OpenCVHelp.h:266](photon-targeting/src/main/native/include/photon/estimation/OpenCVHelp.h#L266))
and Python
([openCVHelp.py:258](photon-lib/py/photonlibpy/estimation/openCVHelp.py#L258))
also lack the guard — so all three ports leak NaN identically. Consumer side at
PhotonPoseEstimator's `LOWEST_AMBIGUITY` strategy handles NaN/Inf gracefully (`NaN < x` is
false, tag is silently dropped); only `AVERAGE_BEST_TARGETS` is vulnerable.
**Observed:** `return Optional.of(new PnpResult(best, alt, errors[0] / errors[1], errors[0], errors[1]));`
— unconditional division.
**Why it's wrong:** `errors[1]` is unguarded. `errors[1] = 0` → ambiguity = ±Inf (handled
downstream OK). `errors[1] = NaN` while `errors[0]` finite → ambiguity = NaN, which
poisons `AVERAGE_BEST_TARGETS` into returning a NaN robot pose. The retry block was
authored precisely because the authors observed OpenCV returning NaN — but it only checks
slot 0.
**Confidence:** medium — the gap exists in code unambiguously; the trigger condition
(NaN errors[1] with finite errors[0] out of `SOLVEPNP_IPPE_SQUARE`) is rare but not
impossible.
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [SMELL] Disconnect-path results publish `captureTimestampMicros = offset` + `latencyMillis = coproc-uptime` — photon-core/src/main/java/org/photonvision/vision/processes/VisionRunner.java:137,195
**Stage:** 1
**Reproducer:** Boot a coprocessor running PhotonVision against an empty USB port (no camera).
Subscribe to the NT `result` topic on the robot, or watch the PhotonVision UI's Latency
widget. Every 100 ms, observe `captureTimestampMicros == NetworkTablesManager.getOffset()`
(server-relative-to-coproc, in µs) — a non-sentinel-looking positive integer — and
`latencyMillisEntry == coproc-uptime-in-ms` (also non-sentinel).
**Expected:** A "frame not available" publish should either (a) not publish at all,
(b) carry a sentinel (`Long.MIN_VALUE`, `-1`, etc.) that consumers can detect, or
(c) carry a current-`wpi::Now`-anchored timestamp with `targets=[]` so latency reads ~0.
The contract on the wire is "µs in the server timebase" — neither `0` (the missing default)
nor `0+offset` is a valid in-band signal for "no frame".
**Observed:** Both disconnect-path call sites build `new CVPipelineResult(0l, 0, 0, null, new Frame())`
**bypassing** `CVPipeline.run()`. That ctor never calls `setImageCaptureTimestampNanos`, so
`CVPipelineResult.imageCaptureTimestampNanos` keeps its default `0L`. `NTDataPublisher.accept()`
then publishes `captureTimestampMicros = 0 + offset = offset` and
`latencyMillisEntry = wpiNanoTime()/1e6 = coproc-uptime-in-ms`.
**Why it's wrong:** The published values look like valid measurements. Empty `targets`
prevents misuse by a well-behaved consumer (`hasTargets()` gates the pose-estimator call,
and wpilib's pose estimator additionally drops stale vision measurements outside its
~1.5 s history). But:
(1) the PhotonVision UI's Latency widget reads coproc-uptime instead of a no-data
    indicator, which is misleading UX users *do* see;
(2) a downstream consumer that doesn't gate on `hasTargets()` (e.g. someone newly
    integrating, or a logger that records every result) silently logs garbage;
(3) the implicit contract "`captureTimestampMicros` is monotonic and meaningful for
    every published frame" is violated;
(4) the fix is one line — set `imageCaptureTimestampNanos` to `wpiNanoTime()` on the
    disconnect path, or skip the publish, or use a clear sentinel.
**Confidence:** high — every step of the trace is verified in the code. Real-world
impact severity is contained by the empty targets list, so this is a SMELL not a
BLOCKER, but the underlying design is fragile.
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.

## [SMELL] Misleading ArUco corner-order comments — two contradictory swap-pairs cancel; runtime verification disagrees with both — photon-core/src/main/java/org/photonvision/vision/aruco/PhotonArucoDetector.java:93 + photon-core/src/main/java/org/photonvision/vision/pipe/impl/ArucoPoseEstimatorPipe.java:73
**Stage:** 2
**Reproducer:** Read the two comment chains describing corner-order reordering in the
legacy ArUco PnP path. Run `audit/scratch/aruco_corner_order_check.py` on the same OpenCV
the project depends on (verified 4.12.0).
**Expected:** Comments describing the corner order produced by
`cv::aruco::detectMarkers` should match OpenCV's documented behavior — `[TL, TR, BR, BL]`
in image-pixel positions when the marker is face-on. Each subsequent reorder's comment
should describe its actual permutation.
**Observed:** `PhotonArucoDetector` says `// ArUco detection returns corners (BR, BL, TL,
TR).` Runtime check: actual order is `[TL, TR, BR, BL]`. After the swap-pair, the comment
claims the result is `(BL, BR, TR, TL)`; actual result is `[TR, TL, BL, BR]`.
`ArucoPoseEstimatorPipe` then says `// We receive 2d corners as (BL, BR, TR, TL) but we
want (BR, BL, TL, TR)` and applies another swap-pair. The actual second-pass result is
`[TL, TR, BR, BL]` — the original raw order, and (by coincidence) the correct match for
the declared `objectPoints` `[TL, TR, BR, BL]`. Both reorders are correct-by-cancellation
and both comments are wrong.
**Why it's wrong:** The end-to-end PnP math is correct *today* because two wrong
reorderings cancel. A future maintainer who reads either comment, notices "this swap-pair
is redundant," and removes it will break the PnP — the recovered pose will be rotated 180°
around the tag's normal. Either both reorders should be removed (the path simplifies to
just passing the raw OpenCV order through) and the comments rewritten, or one should
remain and the other removed — but the current state is fragile and self-misleading.
**Confidence:** high — runtime-verified against OpenCV 4.12.0 (the version `cv2` package
exposes).
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`; runtime check
`audit/scratch/aruco_corner_order_check.py`.

## [DIVERGENCE] Python PhotonPoseEstimator implements only 3 of 9 strategies; missing AVERAGE_BEST_TARGETS, CONSTRAINED_SOLVEPNP, MULTI_TAG_PNP_ON_RIO, CLOSEST_TO_* — photon-lib/py/photonlibpy/photonPoseEstimator.py:39-271
**Stage:** 7
**Reproducer:** Compare the method list in `photonlibpy.PhotonPoseEstimator` (Python)
against `org.photonvision.PhotonPoseEstimator` (Java) and `photon::PhotonPoseEstimator`
(C++). Python exposes only `estimatePnpDistanceTrigSolvePose`,
`estimateCoprocMultiTagPose`, and `estimateLowestAmbiguityPose`. Java/C++ expose all nine
strategies plus a `setPrimaryStrategy(PoseStrategy)` + `getEstimatedPose()` dispatcher.
**Expected:** Either (a) the Python port reaches parity with Java/C++ on strategy
coverage, or (b) the docs / API surface explicitly call out the Python-only subset so
users porting a robot codebase between languages aren't surprised. The PV docs landing
page treats Python as a first-class port.
**Observed:** Python silently lacks AVERAGE_BEST_TARGETS, CONSTRAINED_SOLVEPNP,
MULTI_TAG_PNP_ON_RIO, CLOSEST_TO_CAMERA_HEIGHT, CLOSEST_TO_REFERENCE_POSE, and
CLOSEST_TO_LAST_POSE. Also missing the `PoseStrategy` enum entirely and the dispatcher
`getEstimatedPose()` — Python users must call a specific method directly with knowledge
of which strategies exist.
**Why it's wrong:** A team migrating from Java to Python (or vice versa) discovers the
gap only at integration time. Six strategies absent + no enum = the divergence isn't
visible from auto-complete or type stubs; users have to read the source. The 7.1
DIVERGENCE already noted that Python's `EstimatedRobotPose` lacks the `strategy` field —
this is the underlying cause (Python has no enum to put in that field). This is also
context for the 2.3 SMELL (CONSTRAINED returns reprojErr=0) and 7.2 SMELL
(AVERAGE_BEST_TARGETS rotation non-commutativity) being trigger-able only on Java/C++.
**Confidence:** high — directly read from the 271-line Python file at
`photon-lib/py/photonlibpy/photonPoseEstimator.py`.
**Verified against:** `0e834f0851584237b0dd0913381b52fe6110dd08`.
