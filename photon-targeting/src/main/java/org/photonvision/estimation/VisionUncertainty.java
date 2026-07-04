/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.photonvision.estimation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N6;
import org.wpilib.math.numbers.N8;
import org.wpilib.vision.apriltag.AprilTag;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.camera.OpenCvLoader;

/**
 * Estimates the uncertainty of a PnP pose estimate by propagating tag corner measurement noise
 * through the reprojection Jacobian (first-order / Cramer-Rao style propagation).
 *
 * <p>This runs entirely from data already available to robot code -- the detected corners, camera
 * calibration, and tag layout -- so it works with any coprocessor version and any strategy that
 * produces a field-to-robot pose from tag corners (multi-tag on coprocessor or RIO, single tag).
 *
 * <p>Typical use is replacing hand-tuned constants passed to {@code
 * SwerveDrivePoseEstimator.addVisionMeasurement}:
 *
 * <pre>
 * var est = photonPoseEstimator.update(result);
 * if (est.isPresent()) {
 *     var stdDevs = VisionUncertainty.estimateStdDevs(
 *             cameraMatrix, distCoeffs,
 *             est.get().targetsUsed, fieldLayout, TargetModel.kAprilTag36h11,
 *             est.get().estimatedPose, robotToCamera, 1.0);
 *     stdDevs.ifPresent(sd -&gt; poseEstimator.addVisionMeasurement(
 *             est.get().estimatedPose.toPose2d(), est.get().timestampSeconds, sd));
 * }
 * </pre>
 *
 * <p>Caveats: this is a local linearization about the given solution. For a single tag it describes
 * only the reported solution branch -- it knows nothing about the alternate IPPE solution, so pose
 * ambiguity must still be handled separately (e.g. reject high-ambiguity measurements).
 */
public final class VisionUncertainty {
    /** Numeric differentiation step for the pose Jacobian, in radians / meters. */
    private static final double kNumericDiffStep = 1e-6;

    private VisionUncertainty() {}

    /**
     * Computes the 6x6 covariance of a PnP solution in OpenCV's (rvec, tvec) parameterization (EDN
     * camera basis), as {@code sigma^2 * (J^T J)^-1} where J is the Jacobian of the projected image
     * points with respect to the pose at the solution.
     *
     * <p>The corner noise sigma is estimated from the reprojection residuals ({@code
     * sqrt(SSE/(2N-6))}) and floored at {@code minCornerNoisePx}. The floor matters most for a single
     * tag, where only 2 residual degrees of freedom exist and the estimate is very noisy.
     *
     * @param cameraMatrix The camera intrinsics matrix in standard OpenCV form
     * @param distCoeffs The camera distortion matrix in standard OpenCV form
     * @param objectTrls The 3d model points (field frame, NWU) corresponding to imagePoints
     * @param imagePoints The detected image points (distorted pixels), same order as objectTrls
     * @param fieldToCamera The solved camera pose in the field frame (NWU)
     * @param minCornerNoisePx Lower bound on the corner noise used, in pixels. 1.0 is a reasonable
     *     starting point for AprilTag corners.
     * @return Covariance of [rvec; tvec], or empty if the geometry is degenerate
     */
    public static Optional<Matrix<N6, N6>> pnpCovariance(
            Matrix<N3, N3> cameraMatrix,
            Matrix<N8, N1> distCoeffs,
            List<Translation3d> objectTrls,
            Point[] imagePoints,
            Pose3d fieldToCamera,
            double minCornerNoisePx) {
        int numPoints = imagePoints.length;
        // 6 pose parameters need more than 6 observations for a residual-based noise
        // estimate; a single tag (8 observations) is the minimum useful input
        if (objectTrls.size() != numPoints || 2 * numPoints <= 6) return Optional.empty();
        OpenCvLoader.forceStaticLoad();

        MatOfPoint3f objectPoints = null;
        MatOfPoint3f rvec = null;
        MatOfPoint3f tvec = null;
        Mat cameraMatrixMat = null;
        MatOfDouble distCoeffsMat = null;
        MatOfPoint2f projectedMat = null;
        Mat jacobianMat = null;
        try {
            var camRt = RotTrlTransform3d.makeRelativeTo(fieldToCamera);
            objectPoints = translationToPoints(objectTrls);
            rvec = OpenCVHelp.rotationToRvec(camRt.getRotation());
            tvec = OpenCVHelp.translationToTvec(camRt.getTranslation());
            cameraMatrixMat = OpenCVHelp.matrixToMat(cameraMatrix.getStorage());
            distCoeffsMat = new MatOfDouble(OpenCVHelp.matrixToMat(distCoeffs.getStorage()));
            projectedMat = new MatOfPoint2f();
            jacobianMat = new Mat();

            Calib3d.projectPoints(
                    objectPoints, rvec, tvec, cameraMatrixMat, distCoeffsMat, projectedMat, jacobianMat, 0);

            // Corner noise from reprojection residuals, floored
            var projected = projectedMat.toArray();
            double sse = 0;
            for (int i = 0; i < numPoints; i++) {
                double dx = projected[i].x - imagePoints[i].x;
                double dy = projected[i].y - imagePoints[i].y;
                sse += dx * dx + dy * dy;
            }
            double sigma = Math.max(Math.sqrt(sse / (2 * numPoints - 6)), minCornerNoisePx);

            // First 6 jacobian columns are d(image points)/d[rvec; tvec]
            var fullJacobian = OpenCVHelp.matToMatrix(jacobianMat).getStorage();
            var jac = fullJacobian.extractMatrix(0, 2 * numPoints, 0, 6);

            var jtj = jac.transpose().mult(jac);
            var covariance = jtj.invert().scale(sigma * sigma);
            if (covariance.hasUncountable()) return Optional.empty();

            return Optional.of(new Matrix<>(covariance));
        } catch (RuntimeException e) {
            // singular J^T J -- degenerate geometry
            return Optional.empty();
        } finally {
            if (objectPoints != null) objectPoints.release();
            if (rvec != null) rvec.release();
            if (tvec != null) tvec.release();
            if (cameraMatrixMat != null) cameraMatrixMat.release();
            if (distCoeffsMat != null) distCoeffsMat.release();
            if (projectedMat != null) projectedMat.release();
            if (jacobianMat != null) jacobianMat.release();
        }
    }

    /**
     * Estimates the standard deviations of a field-to-robot pose estimate as (x meters, y meters,
     * theta radians), suitable for direct use with {@code
     * SwerveDrivePoseEstimator.addVisionMeasurement}.
     *
     * <p>The result reflects the actual measurement geometry: it grows with distance and shrinks with
     * tag count and spread, instead of relying on hand-tuned constants.
     *
     * @param cameraMatrix The camera intrinsics matrix in standard OpenCV form (from {@code
     *     PhotonCamera.getCameraMatrix()})
     * @param distCoeffs The camera distortion matrix in standard OpenCV form (from {@code
     *     PhotonCamera.getDistCoeffs()})
     * @param visTags The targets used to compute the pose estimate. Non-tag targets and tags missing
     *     from the layout are automatically excluded.
     * @param tagLayout The known tag layout on the field
     * @param tagModel The model describing the tag's geometry
     * @param estimatedRobotPose The estimated field-to-robot pose these targets produced
     * @param robotToCamera The transform from the robot pose to the camera optical frame
     * @param minCornerNoisePx Lower bound on the corner noise, in pixels. 1.0 is a reasonable
     *     starting point for AprilTag corners.
     * @return Standard deviations of (x, y, theta), or empty if the geometry is degenerate
     */
    public static Optional<Matrix<N3, N1>> estimateStdDevs(
            Matrix<N3, N3> cameraMatrix,
            Matrix<N8, N1> distCoeffs,
            List<PhotonTrackedTarget> visTags,
            AprilTagFieldLayout tagLayout,
            TargetModel tagModel,
            Pose3d estimatedRobotPose,
            Transform3d robotToCamera,
            double minCornerNoisePx) {
        if (tagLayout == null || visTags == null || visTags.isEmpty()) return Optional.empty();

        // Same tag/corner collection as VisionEstimation.estimateCamPosePNP, so the
        // object/image point correspondence matches the original solve
        var corners = new ArrayList<TargetCorner>();
        var knownTags = new ArrayList<AprilTag>();
        for (var tgt : visTags) {
            int id = tgt.getFiducialId();
            tagLayout
                    .getTagPose(id)
                    .ifPresent(
                            pose -> {
                                knownTags.add(new AprilTag(id, pose));
                                corners.addAll(tgt.getDetectedCorners());
                            });
        }
        if (knownTags.isEmpty() || corners.isEmpty() || corners.size() % 4 != 0) {
            return Optional.empty();
        }
        var objectTrls = new ArrayList<Translation3d>();
        for (var tag : knownTags) objectTrls.addAll(tagModel.getFieldVertices(tag.pose));
        Point[] points = OpenCVHelp.cornersToPoints(corners);

        var fieldToCamera = estimatedRobotPose.transformBy(robotToCamera);

        var maybeThetaCov =
                pnpCovariance(
                        cameraMatrix, distCoeffs, objectTrls, points, fieldToCamera, minCornerNoisePx);
        if (maybeThetaCov.isEmpty()) return Optional.empty();
        var thetaCov = maybeThetaCov.get().getStorage();

        // Map the (rvec, tvec) covariance into (x, y, yaw) of the robot pose with a
        // numeric Jacobian of the theta -> robot pose function
        var camRt = RotTrlTransform3d.makeRelativeTo(fieldToCamera);
        MatOfPoint3f rvecMat = OpenCVHelp.rotationToRvec(camRt.getRotation());
        MatOfPoint3f tvecMat = OpenCVHelp.translationToTvec(camRt.getTranslation());
        double[] theta = new double[6];
        System.arraycopy(rvecMat.get(0, 0), 0, theta, 0, 3);
        System.arraycopy(tvecMat.get(0, 0), 0, theta, 3, 3);
        rvecMat.release();
        tvecMat.release();

        double[] base = thetaToRobotXYYaw(theta, robotToCamera);
        var poseJacobian = new SimpleMatrix(3, 6);
        for (int i = 0; i < 6; i++) {
            double[] perturbed = theta.clone();
            perturbed[i] += kNumericDiffStep;
            double[] p = thetaToRobotXYYaw(perturbed, robotToCamera);
            poseJacobian.set(0, i, (p[0] - base[0]) / kNumericDiffStep);
            poseJacobian.set(1, i, (p[1] - base[1]) / kNumericDiffStep);
            poseJacobian.set(2, i, Math.IEEEremainder(p[2] - base[2], 2.0 * Math.PI) / kNumericDiffStep);
        }

        var poseCov = poseJacobian.mult(thetaCov).mult(poseJacobian.transpose());
        double sx = Math.sqrt(poseCov.get(0, 0));
        double sy = Math.sqrt(poseCov.get(1, 1));
        double syaw = Math.sqrt(poseCov.get(2, 2));
        if (!Double.isFinite(sx) || !Double.isFinite(sy) || !Double.isFinite(syaw)) {
            return Optional.empty();
        }
        return Optional.of(VecBuilder.fill(sx, sy, syaw));
    }

    /** Maps an OpenCV [rvec; tvec] world-to-camera solution to robot pose (x, y, yaw). */
    private static double[] thetaToRobotXYYaw(double[] theta, Transform3d robotToCamera) {
        var rvec = new MatOfPoint3f(new Point3(theta[0], theta[1], theta[2]));
        var tvec = new MatOfPoint3f(new Point3(theta[3], theta[4], theta[5]));
        var worldToCam =
                new Transform3d(OpenCVHelp.tvecToTranslation(tvec), OpenCVHelp.rvecToRotation(rvec));
        rvec.release();
        tvec.release();
        // the inverse of the world->camera basis change, applied to the origin, is the
        // camera pose in the world (field) frame
        var cameraPose = new Pose3d().plus(worldToCam.inverse());
        var robotPose = cameraPose.plus(robotToCamera.inverse());
        return new double[] {robotPose.getX(), robotPose.getY(), robotPose.getRotation().getZ()};
    }

    /** Converts NWU field-frame translations to an OpenCV EDN point Mat. */
    private static MatOfPoint3f translationToPoints(List<Translation3d> translations) {
        return OpenCVHelp.translationToTvec(translations.toArray(new Translation3d[0]));
    }
}
