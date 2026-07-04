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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.photonvision.jni.CombinedRuntimeLoader;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;
import org.wpilib.math.util.Nat;
import org.wpilib.vision.apriltag.AprilTag;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.camera.OpenCvLoader;

public class VisionUncertaintyTest {
    // 720p-ish pinhole camera, no distortion
    private static final Matrix<N3, N3> cameraMatrix =
            MatBuilder.fill(Nat.N3(), Nat.N3(), 900, 0, 640, 0, 900, 360, 0, 0, 1);
    private static final Matrix<N8, N1> distCoeffs = new Matrix<>(Nat.N8(), Nat.N1());

    private static final TargetModel tagModel = TargetModel.kAprilTag36h11;

    // Three tags on a wall at x=4, facing -X (toward the robot)
    private static final List<AprilTag> tags =
            List.of(
                    new AprilTag(1, new Pose3d(4, -0.5, 1.0, new Rotation3d(0, 0, Math.PI))),
                    new AprilTag(2, new Pose3d(4, 0.8, 1.3, new Rotation3d(0, 0, Math.PI))),
                    new AprilTag(3, new Pose3d(4, 2.0, 0.9, new Rotation3d(0, 0, Math.PI))));
    private static final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(tags, 16, 8);

    private static final Pose3d robotPose = new Pose3d(1.0, 0.5, 0, new Rotation3d(0, 0, 0.25));
    private static final Transform3d robotToCamera =
            new Transform3d(new Translation3d(0.2, 0.0, 0.5), new Rotation3d(0, -0.05, 0.1));

    private static final double kCornerNoisePx = 1.0;

    @BeforeAll
    public static void setUp() throws IOException {
        // Load OpenCV with photon's loader (same idiom as photon-lib's OpenCVTest).
        // OpenCvLoader's static initializer System.exit()s the JVM when it can't
        // extract its own copy, so disarm it before anything touches that class.
        OpenCvLoader.Helper.setExtractOnStaticLoad(false);
        CombinedRuntimeLoader.loadLibraries(VisionUncertaintyTest.class, Core.NATIVE_LIBRARY_NAME);
    }

    /** Projects the corners of a tag into the image from the true robot pose. */
    private static Point[] projectTagCorners(Pose3d tagPose) {
        var fieldToCamera = robotPose.transformBy(robotToCamera);
        var camRt = RotTrlTransform3d.makeRelativeTo(fieldToCamera);
        return OpenCVHelp.projectPoints(
                cameraMatrix, distCoeffs, camRt, tagModel.getFieldVertices(tagPose));
    }

    private static PhotonTrackedTarget makeTarget(int id, Point[] corners) {
        var cornerList = OpenCVHelp.pointsToCorners(corners);
        return new PhotonTrackedTarget(
                0, 0, 0, 0, id, -1, -1f, new Transform3d(), new Transform3d(), 0, cornerList, cornerList);
    }

    private static List<PhotonTrackedTarget> makeTargets(int numTags, Random rand, double noisePx) {
        var targets = new ArrayList<PhotonTrackedTarget>();
        for (int t = 0; t < numTags; t++) {
            var tag = tags.get(t);
            var corners = projectTagCorners(tag.pose);
            for (var corner : corners) {
                corner.x += rand.nextGaussian() * noisePx;
                corner.y += rand.nextGaussian() * noisePx;
            }
            targets.add(makeTarget(tag.ID, corners));
        }
        return targets;
    }

    /** Solves PnP from targets and returns the robot pose (x, y, yaw). */
    private static double[] solveRobotPose(List<PhotonTrackedTarget> targets) {
        var pnp =
                VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, targets, tagLayout, tagModel);
        assertTrue(pnp.isPresent());
        var cameraPose = new Pose3d().plus(pnp.get().best);
        var robot = cameraPose.plus(robotToCamera.inverse());
        return new double[] {robot.getX(), robot.getY(), robot.getRotation().getZ()};
    }

    @Test
    public void testSanityAgainstTruePose() {
        // Noiseless corners must reproduce the true robot pose (validates the frame
        // chain used by both the test and the numeric pose jacobian)
        var pose = solveRobotPose(makeTargets(3, new Random(42), 0.0));
        assertEquals(robotPose.getX(), pose[0], 1e-4);
        assertEquals(robotPose.getY(), pose[1], 1e-4);
        assertEquals(robotPose.getRotation().getZ(), pose[2], 1e-4);
    }

    @Test
    public void testPredictedStdDevsMatchMonteCarlo() {
        // Predicted stddevs at the true pose, with noiseless corners: the residual-based
        // noise estimate is ~0, so the floor supplies exactly the true corner noise
        var cleanTargets = makeTargets(3, new Random(42), 0.0);
        var predicted =
                VisionUncertainty.estimateStdDevs(
                        cameraMatrix,
                        distCoeffs,
                        cleanTargets,
                        tagLayout,
                        tagModel,
                        robotPose,
                        robotToCamera,
                        kCornerNoisePx);
        assertTrue(predicted.isPresent());

        // Empirical scatter of the same solve under the same corner noise
        var rand = new Random(1234);
        int trials = 400;
        double[] sum = new double[3];
        double[] sumSq = new double[3];
        for (int i = 0; i < trials; i++) {
            var pose = solveRobotPose(makeTargets(3, rand, kCornerNoisePx));
            for (int k = 0; k < 3; k++) {
                sum[k] += pose[k];
                sumSq[k] += pose[k] * pose[k];
            }
        }
        double[] empirical = new double[3];
        for (int k = 0; k < 3; k++) {
            empirical[k] = Math.sqrt((sumSq[k] - sum[k] * sum[k] / trials) / (trials - 1));
        }

        System.out.println(
                "predicted (x,y,yaw): "
                        + predicted.get().get(0, 0)
                        + ", "
                        + predicted.get().get(1, 0)
                        + ", "
                        + predicted.get().get(2, 0));
        System.out.println(
                "empirical (x,y,yaw): " + empirical[0] + ", " + empirical[1] + ", " + empirical[2]);

        for (int k = 0; k < 3; k++) {
            double ratio = predicted.get().get(k, 0) / empirical[k];
            assertTrue(
                    ratio > 0.7 && ratio < 1.45,
                    "axis " + k + ": predicted/empirical ratio " + ratio + " outside [0.7, 1.45]");
        }
    }

    @Test
    public void testFewerTagsMeansMoreUncertainty() {
        var threeTag =
                VisionUncertainty.estimateStdDevs(
                        cameraMatrix,
                        distCoeffs,
                        makeTargets(3, new Random(7), 0.0),
                        tagLayout,
                        tagModel,
                        robotPose,
                        robotToCamera,
                        kCornerNoisePx);
        var oneTag =
                VisionUncertainty.estimateStdDevs(
                        cameraMatrix,
                        distCoeffs,
                        makeTargets(1, new Random(7), 0.0),
                        tagLayout,
                        tagModel,
                        robotPose,
                        robotToCamera,
                        kCornerNoisePx);
        assertTrue(threeTag.isPresent());
        assertTrue(oneTag.isPresent());

        for (int k = 0; k < 3; k++) {
            assertTrue(
                    oneTag.get().get(k, 0) > threeTag.get().get(k, 0),
                    "axis "
                            + k
                            + ": one-tag stddev "
                            + oneTag.get().get(k, 0)
                            + " should exceed three-tag "
                            + threeTag.get().get(k, 0));
        }
    }

    @Test
    public void testNoiseEstimatedFromResiduals() {
        // With a tiny floor, sigma comes from the residuals of the noisy corners
        // against the true pose; it should land in the ballpark of the injected noise
        var noisyTargets = makeTargets(3, new Random(99), kCornerNoisePx);
        var floored =
                VisionUncertainty.estimateStdDevs(
                        cameraMatrix,
                        distCoeffs,
                        noisyTargets,
                        tagLayout,
                        tagModel,
                        robotPose,
                        robotToCamera,
                        kCornerNoisePx);
        var estimated =
                VisionUncertainty.estimateStdDevs(
                        cameraMatrix,
                        distCoeffs,
                        noisyTargets,
                        tagLayout,
                        tagModel,
                        robotPose,
                        robotToCamera,
                        1e-6);
        assertTrue(floored.isPresent());
        assertTrue(estimated.isPresent());

        // residual-estimated sigma should be within a factor ~2.5 of the true noise
        for (int k = 0; k < 3; k++) {
            double ratio = estimated.get().get(k, 0) / floored.get().get(k, 0);
            assertTrue(
                    ratio > 0.4 && ratio < 2.5,
                    "axis " + k + ": residual-estimated/true-noise ratio " + ratio);
        }
    }

    @Test
    public void testDegenerateInputsReturnEmpty() {
        // no targets
        assertTrue(
                VisionUncertainty.estimateStdDevs(
                                cameraMatrix,
                                distCoeffs,
                                List.of(),
                                tagLayout,
                                tagModel,
                                robotPose,
                                robotToCamera,
                                kCornerNoisePx)
                        .isEmpty());
        // tag not in the layout
        var unknownTag = makeTarget(99, projectTagCorners(tags.get(0).pose));
        assertTrue(
                VisionUncertainty.estimateStdDevs(
                                cameraMatrix,
                                distCoeffs,
                                List.of(unknownTag),
                                tagLayout,
                                tagModel,
                                robotPose,
                                robotToCamera,
                                kCornerNoisePx)
                        .isEmpty());
    }
}
