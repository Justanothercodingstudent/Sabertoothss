package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class Vision {
  private static final CameraPoseEstimate[] EMPTY_POSE_ESTIMATES = new CameraPoseEstimate[0];

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
  private final List<CameraPoseEstimate> reusablePoseEstimates = new ArrayList<>();

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

  public void forEachLimelight(Consumer<Limelight> action) {
    for (Limelight limelight : limelights) {
      action.accept(limelight);
    }
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
    if (limelights.length == 0) {
      return EMPTY_POSE_ESTIMATES;
    }

    reusablePoseEstimates.clear();
    for (Limelight limelight : limelights) {
      PoseEstimate poseEstimate = limelight.getMegaTag2PoseEstimate();
      if (poseEstimate != null) {
        reusablePoseEstimates.add(new CameraPoseEstimate(limelight, poseEstimate));
      }
    }

    return reusablePoseEstimates.toArray(new CameraPoseEstimate[reusablePoseEstimates.size()]);
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

      Limelight.TargetObservation observation = limelight.getBestTargetObservation(validTagIds);
      if (observation == null) {
        continue;
      }

      double distanceMeters = observation.distanceMeters;
      if (bestTarget == null || isBetterTarget(distanceMeters, bestTarget.distanceMeters)) {
        bestTarget =
            new TrackedTag(
                limelight, observation.tagId, distanceMeters, observation.toTargetData());
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
