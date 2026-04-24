package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers.PoseEstimate;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class Vision {
    public static class TrackedTag {
        public final Limelight limelight;
        public final int tagId;
        public final double distanceMeters;
        public final double[] targetData;

        public TrackedTag(Limelight limelight, int tagId, double distanceMeters, double[] targetData) {
            this.limelight = limelight;
            this.tagId = tagId;
            this.distanceMeters = distanceMeters;
            this.targetData = targetData.clone();
        }
    }

    public static class CameraPoseEstimate {
        public final Limelight limelight;
        public final PoseEstimate poseEstimate;

        public CameraPoseEstimate(Limelight limelight, PoseEstimate poseEstimate) {
            this.limelight = limelight;
            this.poseEstimate = poseEstimate;
        }
    }

    private final Limelight[] limelights;

    public Vision(Limelight... limelights) {
        List<Limelight> configuredLimelights = new ArrayList<>();
        for (Limelight limelight : limelights) {
            if (limelight != null) {
                configuredLimelights.add(limelight);
            }
        }

        this.limelights = configuredLimelights.toArray(new Limelight[0]);
    }

    public Limelight[] getLimelights() {
        return limelights.clone();
    }

    

    public void pushFieldHeadingToLimelights(Rotation2d fieldHeading) {
        for (Limelight limelight : limelights) {
            limelight.pushRobotOrientation(fieldHeading);
        }
    }

    public void setIMUMode(int imuMode) {
        for (Limelight limelight : limelights) {
            limelight.setIMUMode(imuMode);
        }
    }

    public CameraPoseEstimate[] getMegaTag2PoseEstimates() {
        List<CameraPoseEstimate> poseEstimates = new ArrayList<>();
        for (Limelight limelight : limelights) {
            PoseEstimate poseEstimate = limelight.getMegaTag2PoseEstimate();
            if (poseEstimate != null) {
                poseEstimates.add(new CameraPoseEstimate(limelight, poseEstimate));
            }
        }

        return poseEstimates.toArray(new CameraPoseEstimate[0]);
    }

    public TrackedTag getBestTarget(double[] validTagIds) {
        return getBestTarget(validTagIds, null);
    }

    public TrackedTag getBestTarget(double[] validTagIds, String limelightName) {
        TrackedTag bestTarget = null;

        for (Limelight limelight : limelights) {
            if (limelightName != null && !limelightName.equals(limelight.getName())) {
                continue;
            }

            double closestTagId = limelight.getClosestTag(validTagIds);
            if (closestTagId < 0.0) {
                continue;
            }

            double[] targetData = limelight.getTarget((int) closestTagId);
            if (targetData == null) {
                continue;
            }

            double distanceMeters = limelight.getDistanceToTag(closestTagId);
            if (bestTarget == null || isBetterTarget(distanceMeters, bestTarget.distanceMeters)) {
                bestTarget = new TrackedTag(limelight, (int) closestTagId, distanceMeters, targetData);
            }
        }

        return bestTarget;
    }

    private boolean isBetterTarget(double candidateDistanceMeters, double bestDistanceMeters) {
        if (candidateDistanceMeters >= 0.0 && bestDistanceMeters < 0.0) {
            return true;
        }

        if (candidateDistanceMeters < 0.0) {
            return false;
        }

        return candidateDistanceMeters < bestDistanceMeters;
    }
}
