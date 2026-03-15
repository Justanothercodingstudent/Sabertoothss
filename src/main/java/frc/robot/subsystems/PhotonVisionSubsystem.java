package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVisionSubsystem extends SubsystemBase {
    public static record VisionMeasurement(
        Pose2d pose,
        double timestampSeconds,
        int tagCount,
        double averageTagDistanceMeters,
        double averageTagArea,
        double bestTargetAmbiguity,
        String cameraName
    ) {}

    private record TagObservation(
        int tagId,
        double distanceMeters,
        PhotonTrackedTarget target,
        String cameraName
    ) {}

    private static final class CameraState {
        private final PhotonCamera camera;
        private final PhotonPoseEstimator poseEstimator;
        private final String name;
        private PhotonPipelineResult latestResult = new PhotonPipelineResult();
        private VisionMeasurement latestMeasurement;

        private CameraState(String name, PhotonPoseEstimator poseEstimator) {
            this.name = name;
            this.camera = new PhotonCamera(name);
            this.poseEstimator = poseEstimator;
        }
    }

    private static final Comparator<VisionMeasurement> NEWEST_FIRST =
        Comparator.comparingDouble(VisionMeasurement::timestampSeconds)
            .reversed()
            .thenComparing(Comparator.comparingInt(VisionMeasurement::tagCount).reversed())
            .thenComparing(Comparator.comparingDouble(VisionMeasurement::averageTagArea).reversed())
            .thenComparing(Comparator.comparingDouble(VisionMeasurement::averageTagDistanceMeters));

    private static final Comparator<VisionMeasurement> OLDEST_FIRST =
        Comparator.comparingDouble(VisionMeasurement::timestampSeconds)
            .thenComparing(Comparator.comparingInt(VisionMeasurement::tagCount).reversed())
            .thenComparing(Comparator.comparingDouble(VisionMeasurement::averageTagArea).reversed())
            .thenComparing(Comparator.comparingDouble(VisionMeasurement::averageTagDistanceMeters));

    private final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private final CameraState[] cameraStates = new CameraState[] {
        new CameraState(
            Constants.PhotonVisionConstants.leftCameraName,
            new PhotonPoseEstimator(fieldLayout, Constants.PhotonVisionConstants.robotToLeftCamera)
        ),
        new CameraState(
            Constants.PhotonVisionConstants.rightCameraName,
            new PhotonPoseEstimator(fieldLayout, Constants.PhotonVisionConstants.robotToRightCamera)
        )
    };

    private VisionMeasurement bestRobotPoseEstimate;

    public Optional<VisionMeasurement> getBestEstimatedPose() {
        return Optional.ofNullable(bestRobotPoseEstimate);
    }

    public List<VisionMeasurement> getVisionMeasurementsSince(double timestampSeconds) {
        return Arrays.stream(cameraStates)
            .map(cameraState -> cameraState.latestMeasurement)
            .filter(Objects::nonNull)
            .filter(measurement -> measurement.timestampSeconds() > timestampSeconds)
            .sorted(OLDEST_FIRST)
            .toList();
    }

    public double getClosestTag(double[] validTagIds) {
        double closestTag = -1.0;
        double closestDistanceMeters = Double.MAX_VALUE;

        for (CameraState cameraState : cameraStates) {
            for (PhotonTrackedTarget target : cameraState.latestResult.getTargets()) {
                int fiducialId = target.getFiducialId();
                if (fiducialId < 0) {
                    continue;
                }

                for (double validTagId : validTagIds) {
                    if (fiducialId == (int) validTagId) {
                        double distanceMeters = getTargetDistanceMeters(target);
                        if (distanceMeters < closestDistanceMeters) {
                            closestDistanceMeters = distanceMeters;
                            closestTag = fiducialId;
                        }
                    }
                }
            }
        }

        return closestTag;
    }

    public double getDistanceToTag(int tagId) {
        return getTagObservation(tagId)
            .map(TagObservation::distanceMeters)
            .orElse(-1.0);
    }

    public Translation2d getAllianceAutoAimTarget() {
        return Constants.FieldConstants.getAllianceAutoAimTarget();
    }

    public double getDistanceToAutoAimTarget(Pose2d robotPose) {
        return getAllianceAutoAimTarget().getDistance(robotPose.getTranslation());
    }

    private void updateCameraState(CameraState cameraState) {
        List<PhotonPipelineResult> unreadResults = cameraState.camera.getAllUnreadResults();
        if (unreadResults.isEmpty()) {
            return;
        }

        PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
        cameraState.latestResult = latestResult;
        cameraState.latestMeasurement = buildVisionMeasurement(cameraState, latestResult).orElse(null);
    }

    private Optional<VisionMeasurement> buildVisionMeasurement(
        CameraState cameraState,
        PhotonPipelineResult result
    ) {
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> estimatedRobotPose =
            result.getTargets().size() > 1
                ? cameraState.poseEstimator.estimateCoprocMultiTagPose(result)
                : cameraState.poseEstimator.estimateLowestAmbiguityPose(result);

        if (estimatedRobotPose.isEmpty()) {
            return Optional.empty();
        }

        List<PhotonTrackedTarget> targetsUsed = estimatedRobotPose.get().targetsUsed;
        double averageDistanceMeters = targetsUsed.stream()
            .mapToDouble(this::getTargetDistanceMeters)
            .average()
            .orElse(Double.MAX_VALUE);
        double averageArea = targetsUsed.stream()
            .mapToDouble(PhotonTrackedTarget::getArea)
            .average()
            .orElse(0.0);
        double bestTargetAmbiguity = targetsUsed.stream()
            .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
            .filter(ambiguity -> ambiguity >= 0.0)
            .min()
            .orElse(-1.0);

        return Optional.of(
            new VisionMeasurement(
                estimatedRobotPose.get().estimatedPose.toPose2d(),
                estimatedRobotPose.get().timestampSeconds,
                targetsUsed.size(),
                averageDistanceMeters,
                averageArea,
                bestTargetAmbiguity,
                cameraState.name
            )
        );
    }

    private Optional<TagObservation> getTagObservation(int tagId) {
        TagObservation closestObservation = null;

        for (CameraState cameraState : cameraStates) {
            for (PhotonTrackedTarget target : cameraState.latestResult.getTargets()) {
                if (target.getFiducialId() != tagId) {
                    continue;
                }

                double distanceMeters = getTargetDistanceMeters(target);
                if (closestObservation == null || distanceMeters < closestObservation.distanceMeters()) {
                    closestObservation = new TagObservation(tagId, distanceMeters, target, cameraState.name);
                }
            }
        }

        return Optional.ofNullable(closestObservation);
    }

    private double getTargetDistanceMeters(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getNorm();
    }

    @Override
    public void periodic() {
        for (CameraState cameraState : cameraStates) {
            updateCameraState(cameraState);
            SmartDashboard.putBoolean(
                "PhotonVision " + cameraState.name + " Connected",
                cameraState.camera.isConnected()
            );
        }

        bestRobotPoseEstimate = Arrays.stream(cameraStates)
            .map(cameraState -> cameraState.latestMeasurement)
            .filter(Objects::nonNull)
            .sorted(NEWEST_FIRST)
            .findFirst()
            .orElse(null);

        SmartDashboard.putNumber("Nearest April Tag Red", getClosestTag(Constants.TeamDependentFactors.reefIDsRed));
        SmartDashboard.putNumber("Nearest April Tag Blue", getClosestTag(Constants.TeamDependentFactors.reefIDsBlue));
        SmartDashboard.putNumber("Nearest Hub Tage", getClosestTag(Constants.TeamDependentFactors.validAprilTagIds));

        if (bestRobotPoseEstimate != null) {
            SmartDashboard.putNumber("PhotonVision Pose X", bestRobotPoseEstimate.pose().getX());
            SmartDashboard.putNumber("PhotonVision Pose Y", bestRobotPoseEstimate.pose().getY());
            SmartDashboard.putNumber(
                "PhotonVision Pose Heading",
                bestRobotPoseEstimate.pose().getRotation().getDegrees()
            );
            SmartDashboard.putNumber("PhotonVision Tag Count", bestRobotPoseEstimate.tagCount());
            SmartDashboard.putString("PhotonVision Camera", bestRobotPoseEstimate.cameraName());
        }
    }
}
