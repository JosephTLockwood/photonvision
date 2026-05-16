# Sub-check closure — 2.1a, 2.2a, 2.3a, 7.2a

- Iteration: 2026-05-16T19:30-04:00
- Audit root commit: `0e834f0851584237b0dd0913381b52fe6110dd08`
- Files read at that SHA:
  - `photon-targeting/src/main/native/include/photon/estimation/OpenCVHelp.h` (lines 220-275)
  - `photon-lib/py/photonlibpy/estimation/openCVHelp.py` (lines 200-267)
  - `photon-lib/py/photonlibpy/photonPoseEstimator.py` (full file, 271 lines)
  - `audit/scratch/aruco_corner_order_check.py` (runtime verifier)

This iteration batch-closes the four remaining sub-checks created by deeper inspections
during stages 2 and 7.

## 2.1a — runtime ArUco corner order verifier

**Question carried over from 2.1:** OpenCV's docs say `cv::aruco::detectMarkers` returns
corners in `[TL, TR, BR, BL]` order. PV's comment chain in
`PhotonArucoDetector` + `ArucoPoseEstimatorPipe` asserts `[BR, BL, TL, TR]`. These two
claims contradict, and the legacy ArUco PnP path's correctness hinges on which is true.

**Runtime resolution:** ran `audit/scratch/aruco_corner_order_check.py`:

    Expected geometric positions:
      TL = (100, 100)
      TR = (500, 100)
      BR = (500, 500)
      BL = (100, 500)

    cv2.aruco.detectMarkers returned corners in this order:
      corners[0][0] = (100.0, 100.0) -> matches geometric TL
      corners[0][1] = (499.0, 100.0) -> matches geometric TR
      corners[0][2] = (499.0, 499.0) -> matches geometric BR
      corners[0][3] = (100.0, 499.0) -> matches geometric BL

**OpenCV's docs are correct; PV's comments are wrong.** The actual order is
`[TL, TR, BR, BL]`.

**End-to-end correctness verdict (re-tracing with verified raw order):**

| stage                                                | actual order        | PV comment claim       |
| ---------------------------------------------------- | ------------------- | ---------------------- |
| Raw `cv2.aruco.detectMarkers`                        | `[TL, TR, BR, BL]`  | `[BR, BL, TL, TR]` ✗   |
| After `PhotonArucoDetector` swap-pair (0↔1, 2↔3)     | `[TR, TL, BL, BR]`  | `[BL, BR, TR, TL]` ✗   |
| After `ArucoPoseEstimatorPipe` swap-pair (0↔1, 2↔3)  | `[TL, TR, BR, BL]`  | `[BR, BL, TL, TR]` ✗   |
| `objectPoints` declared in ArucoPoseEstimatorPipe   | `[TL, TR, BR, BL]`  | (matches)              |

**Index correspondence is preserved.** Two wrong swap-pairs cancel; final pairing
`imagePoints[i] ↔ objectPoints[i]` is correct. The math works **despite** the comments.

This is a SMELL, not a BLOCKER: the path is correct today, but a future maintainer who
tries to "fix" the misleading comments by removing one of the swap-pair calls breaks the
PnP. Filing as `SMELL` in FINDINGS.md.

## 2.2a — C++/Python parity on NaN-retry logic

**Question carried over from 2.2:** Java's `OpenCVHelp.solvePNP_SQUARE` has a retry-with-
perturbed-input loop guarded by `if (!Double.isNaN(errors[0])) break`. Do C++ and Python
have the same loop?

**C++** [OpenCVHelp.h:234-256](photon-targeting/src/main/native/include/photon/estimation/OpenCVHelp.h#L234-L256):

    for (int tries = 0; tries < 2; tries++) {
      cv::solvePnPGeneric(...);
      errors = reprojectionError.at<cv::Vec2f>(...);
      best = Transform3d{...};
      if (tvecs.size() > 1) alt = Transform3d{...};
      if (!std::isnan(errors[0])) {
        break;
      } else {
        cv::Point2f pt = imagePoints[0];
        pt.x -= 0.001f;
        pt.y -= 0.001f;
        imagePoints[0] = pt;
      }
    }
    if (std::isnan(errors[0])) { return std::nullopt; }

**Python** [openCVHelp.py:215-250](photon-lib/py/photonlibpy/estimation/openCVHelp.py#L215-L250):

    for tries in range(2):
        retval, rvecs, tvecs, reprojectionError = cv.solvePnPGeneric(...)
        best = Transform3d(...)
        if len(tvecs) > 1: alt = Transform3d(...)
        if reprojectionError is not None and not math.isnan(reprojectionError[0, 0]):
            break
        else:
            pt = imagePoints[0]; pt[0,0] -= 0.001; pt[0,1] -= 0.001
            imagePoints[0] = pt
    if reprojectionError is None or math.isnan(reprojectionError[0, 0]):
        return None

**All three ports have full parity** on the NaN-retry logic. The 2.2 SMELL (no
`errors[1]` guard) applies **equally to all three ports**, no port-specific divergence.
No new finding; the existing 2.2 SMELL covers it.

## 2.3a — Python lacks CONSTRAINED_SOLVEPNP

**Question carried over from 2.3:** Java + C++ implement `CONSTRAINED_SOLVEPNP` (Sleipnir
JNI). Does Python?

**Grep result on `photon-lib/py/`:**

    $ grep -ri "CONSTRAINED_SOLVEPNP" photon-lib/py
    (no matches)

`photon-lib/py/photonlibpy/photonPoseEstimator.py` only implements three methods:
- `estimatePnpDistanceTrigSolvePose` (PNP_DISTANCE_TRIG_SOLVE)
- `estimateCoprocMultiTagPose` (MULTI_TAG_PNP_ON_COPROCESSOR)
- `estimateLowestAmbiguityPose` (LOWEST_AMBIGUITY)

No `CONSTRAINED_SOLVEPNP`, no Sleipnir JNI binding. The 2.3 SMELL (CONSTRAINED returns
`bestReprojErr=0`) is moot for Python users — they can't trigger it.

## 7.2a — Python lacks AVERAGE_BEST_TARGETS

**Question carried over from 7.2:** Java + C++ implement `AVERAGE_BEST_TARGETS`. Does
Python?

Same grep + same file inspection as 2.3a. Python's `photonPoseEstimator.py` does not
implement `AVERAGE_BEST_TARGETS`. The 7.2 SMELL (iterative `rotateBy(R.times(w))` is
non-commutative) is moot for Python users — they can't trigger it.

## Consolidated DIVERGENCE — Python PhotonPoseEstimator strategy coverage gap

The 7.1 DIVERGENCE already flagged that Python's `EstimatedRobotPose` dataclass lacks the
`strategy` field. The deeper issue surfaces here: Python's `PhotonPoseEstimator`
implements only **3 of the 8** strategies offered by Java/C++.

| Strategy                          | Java | C++ | Python |
| --------------------------------- | ---- | --- | ------ |
| LOWEST_AMBIGUITY                  | ✓    | ✓   | ✓      |
| MULTI_TAG_PNP_ON_COPROCESSOR      | ✓    | ✓   | ✓      |
| PNP_DISTANCE_TRIG_SOLVE           | ✓    | ✓   | ✓      |
| AVERAGE_BEST_TARGETS              | ✓    | ✓   | ✗      |
| CONSTRAINED_SOLVEPNP              | ✓    | ✓   | ✗      |
| MULTI_TAG_PNP_ON_RIO              | ✓    | ✓   | ✗      |
| CLOSEST_TO_CAMERA_HEIGHT          | ✓    | ✓   | ✗      |
| CLOSEST_TO_REFERENCE_POSE         | ✓    | ✓   | ✗      |
| CLOSEST_TO_LAST_POSE              | ✓    | ✓   | ✗      |

Python is also missing the `PoseStrategy` enum entirely and lacks the
`getEstimatedPose()` dispatcher method that switches by `primaryStrategy`. Python users
call the specific strategy method directly. This is consistent with Python being newer
than Java/C++ but is a **structural API divergence**.

Filing **one consolidated DIVERGENCE** rather than three (one per missing strategy),
since the gap is structural.

## Outcome

- 2.1a `[x]` — runtime resolved; ArUco corner order is `[TL, TR, BR, BL]`; two
  contradictory PV swap-pairs cancel; path is correct; **filing 1× SMELL** for the
  misleading comments.
- 2.2a `[x]` — C++/Python parity with Java confirmed; existing 2.2 SMELL covers all
  three ports.
- 2.3a `[x]` — Python lacks CONSTRAINED_SOLVEPNP; rolled into consolidated DIVERGENCE
  below.
- 7.2a `[x]` — Python lacks AVERAGE_BEST_TARGETS; rolled into consolidated DIVERGENCE
  below.
- **Filing 1× DIVERGENCE** for Python PhotonPoseEstimator strategy coverage (6 of 9
  strategies missing).

All open sub-checks are now closed. Audit reaches SUMMARY.md stage.
