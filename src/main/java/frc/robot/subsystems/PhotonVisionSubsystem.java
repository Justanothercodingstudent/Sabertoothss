package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Deque;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    public static record RobotRelativeTargetObservation(
        int tagId,
        Translation2d robotRelativeTargetTranslation,
        double robotRelativeYawDegrees,
        double targetArea,
        double distanceMeters,
        String cameraName
    ) {}

    private static final class CameraState {
        private final PhotonCamera camera;
        private final PhotonPoseEstimator estimator;
        private final String name;
        private final Transform3d robotToCamera;
        private PhotonPipelineResult latestResult = new PhotonPipelineResult();
        private String latestStatus = "No frames received";

        private CameraState(String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
            this.name = name;
            this.camera = new PhotonCamera(name);
            this.robotToCamera = robotToCamera;
            this.estimator = new PhotonPoseEstimator(layout, robotToCamera);
        }
    }

    private static final int MAX_BUFFERED_MEASUREMENTS = 100;

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
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private final Field2d photonField = new Field2d();
    private final CameraState[] cameraStates = new CameraState[] {
        new CameraState(
            Constants.PhotonVisionConstants.leftCameraName,
            Constants.PhotonVisionConstants.robotToLeftCamera,
            fieldLayout
        ),
        new CameraState(
            Constants.PhotonVisionConstants.rightCameraName,
            Constants.PhotonVisionConstants.robotToRightCamera,
            fieldLayout
        )
    };

    private final Deque<VisionMeasurement> measurementBuffer = new ArrayDeque<>();
    private VisionMeasurement bestRobotPoseEstimate;

    public PhotonVisionSubsystem() {
        SmartDashboard.putData("PhotonVision Field", photonField);
    }

    public Optional<VisionMeasurement> getBestEstimatedPose() {
        return Optional.ofNullable(bestRobotPoseEstimate);
    }

    public String getStatusSummary() {
        return Arrays.stream(cameraStates)
            .map(cameraState -> cameraState.name + ": " + cameraState.latestStatus)
            .reduce((left, right) -> left + " | " + right)
            .orElse("No cameras configured");
    }

    public List<VisionMeasurement> getVisionMeasurementsSince(double timestampSeconds) {
        return measurementBuffer.stream()
            .filter(measurement -> measurement.timestampSeconds() > timestampSeconds)
            .sorted(OLDEST_FIRST)
            .toList();
    }

    public Optional<RobotRelativeTargetObservation> getBestRobotRelativeAllianceTarget() {
        boolean preferRedAlliance = Constants.TeamDependentFactors.isRedAlliance();
        Comparator<RobotRelativeTargetObservation> comparator =
            Comparator.comparingDouble(RobotRelativeTargetObservation::targetArea)
                .reversed()
                .thenComparingDouble(observation -> Math.abs(observation.robotRelativeYawDegrees()))
                .thenComparingDouble(RobotRelativeTargetObservation::distanceMeters);

        return Arrays.stream(cameraStates)
            .flatMap(cameraState -> cameraState.latestResult.getTargets().stream()
                .filter(target -> isPreferredAllianceTag(target.getFiducialId(), preferRedAlliance))
                .map(target -> buildRobotRelativeTargetObservation(
                    cameraState,
                    target,
                    getAutoAimTargetForTag(target.getFiducialId())
                ))
                .flatMap(Optional::stream))
            .sorted(comparator)
            .findFirst();
    }

    public double getClosestTag(double[] validTagIds) {
        double closestTag = -1.0;
        double closestDistanceMeters = Double.MAX_VALUE;

        for (CameraState cameraState : cameraStates) {
            for (PhotonTrackedTarget target : cameraState.latestResult.getTargets()) {
                int tagId = target.getFiducialId();
                if (tagId < 0 || !containsTag(validTagIds, tagId)) {
                    continue;
                }

                double distanceMeters = getTargetDistanceMeters(target);
                if (distanceMeters < closestDistanceMeters) {
                    closestDistanceMeters = distanceMeters;
                    closestTag = tagId;
                }
            }
        }

        return closestTag;
    }

    public double getDistanceToTag(int tagId) {
        return Arrays.stream(cameraStates)
            .flatMap(cameraState -> cameraState.latestResult.getTargets().stream())
            .filter(target -> target.getFiducialId() == tagId)
            .mapToDouble(this::getTargetDistanceMeters)
            .min()
            .orElse(-1.0);
    }

    public Translation2d getAllianceAutoAimTarget() {
        return Constants.FieldConstants.getAllianceAutoAimTarget();
    }

    public double getDistanceToAutoAimTarget(Pose2d robotPose) {
        return getAllianceAutoAimTarget().getDistance(robotPose.getTranslation());
    }

    private boolean containsTag(double[] tagIds, int tagId) {
        for (double configuredTagId : tagIds) {
            if (tagId == (int) configuredTagId) {
                return true;
            }
        }

        return false;
    }

    private boolean isPreferredAllianceTag(int tagId, boolean preferRedAlliance) {
        return preferRedAlliance
            ? containsTag(Constants.TeamDependentFactors.reefIDsRed, tagId)
            : containsTag(Constants.TeamDependentFactors.reefIDsBlue, tagId);
    }

    private Translation2d getAutoAimTargetForTag(int tagId) {
        return containsTag(Constants.TeamDependentFactors.reefIDsRed, tagId)
            ? Constants.FieldConstants.redAutoAimTarget
            : Constants.FieldConstants.blueAutoAimTarget;
    }

    private Optional<RobotRelativeTargetObservation> buildRobotRelativeTargetObservation(
        CameraState cameraState,
        PhotonTrackedTarget target,
        Translation2d autoAimTarget
    ) {
        Translation2d robotRelativeTagTranslation = getRobotRelativeTagPose(cameraState, target).getTranslation();
        Translation2d robotRelativeTargetTranslation =
            getTagToAutoAimTargetInTagFrame(target.getFiducialId(), autoAimTarget)
                .map(tagToTargetInTagFrame -> robotRelativeTagTranslation.plus(
                    tagToTargetInTagFrame.rotateBy(
                        getRobotRelativeTagPose(cameraState, target).getRotation()
                    )
                ))
                .orElse(robotRelativeTagTranslation);

        if (!Double.isFinite(robotRelativeTargetTranslation.getX())
            || !Double.isFinite(robotRelativeTargetTranslation.getY())) {
            return Optional.empty();
        }

        return Optional.of(
            new RobotRelativeTargetObservation(
                target.getFiducialId(),
                robotRelativeTargetTranslation,
                robotRelativeTargetTranslation.getAngle().getDegrees(),
                target.getArea(),
                robotRelativeTargetTranslation.getNorm(),
                cameraState.name
            )
        );
    }

    private Pose2d getRobotRelativeTagPose(CameraState cameraState, PhotonTrackedTarget target) {
        Pose3d robotPose = new Pose3d();
        Pose3d cameraPose = robotPose.transformBy(cameraState.robotToCamera);
        return cameraPose.transformBy(target.getBestCameraToTarget()).toPose2d();
    }

    private Optional<Translation2d> getTagToAutoAimTargetInTagFrame(int tagId, Translation2d autoAimTarget) {
        return fieldLayout.getTagPose(tagId)
            .map(Pose3d::toPose2d)
            .map(tagFieldPose -> autoAimTarget
                .minus(tagFieldPose.getTranslation())
                .rotateBy(tagFieldPose.getRotation().unaryMinus()));
    }

    private double getTargetDistanceMeters(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getNorm();
    }

    private void addMeasurement(VisionMeasurement measurement) {
        measurementBuffer.addLast(measurement);
        while (measurementBuffer.size() > MAX_BUFFERED_MEASUREMENTS) {
            measurementBuffer.removeFirst();
        }
    }

    private Optional<VisionMeasurement> toVisionMeasurement(
        CameraState cameraState,
        PhotonPipelineResult result,
        EstimatedRobotPose estimatedRobotPose
    ) {
        List<PhotonTrackedTarget> targetsUsed = new ArrayList<>(estimatedRobotPose.targetsUsed);
        if (targetsUsed.isEmpty() && result.hasTargets()) {
            targetsUsed = result.getTargets();
        }

        if (targetsUsed.isEmpty()) {
            return Optional.empty();
        }

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

        Pose2d estimatedPose = estimatedRobotPose.estimatedPose.toPose2d();
        if (!Double.isFinite(estimatedPose.getX())
            || !Double.isFinite(estimatedPose.getY())
            || !Double.isFinite(estimatedPose.getRotation().getRadians())) {
            return Optional.empty();
        }

        return Optional.of(
            new VisionMeasurement(
                estimatedPose,
                estimatedRobotPose.timestampSeconds,
                targetsUsed.size(),
                averageDistanceMeters,
                averageArea,
                bestTargetAmbiguity,
                cameraState.name
            )
        );
    }

    private void processCamera(CameraState cameraState) {
        List<PhotonPipelineResult> unreadResults = cameraState.camera.getAllUnreadResults();
        if (unreadResults.isEmpty()) {
            if (!cameraState.camera.isConnected()) {
                cameraState.latestStatus = "Disconnected";
            }
            return;
        }

        for (PhotonPipelineResult result : unreadResults) {
            cameraState.latestResult = result;

            if (!result.hasTargets()) {
                cameraState.latestStatus = "No targets";
                continue;
            }

            Optional<EstimatedRobotPose> estimate = result.getTargets().size() > 1
                ? cameraState.estimator.estimateCoprocMultiTagPose(result)
                : cameraState.estimator.estimateLowestAmbiguityPose(result);

            if (estimate.isEmpty() && result.getTargets().size() > 1) {
                estimate = cameraState.estimator.estimateLowestAmbiguityPose(result);
            }
            if (estimate.isEmpty()) {
                cameraState.latestStatus = "Targets seen but pose solve failed";
                continue;
            }

            Optional<VisionMeasurement> measurement = toVisionMeasurement(cameraState, result, estimate.get());
            if (measurement.isEmpty()) {
                cameraState.latestStatus = "Pose solve rejected as invalid";
                continue;
            }

            addMeasurement(measurement.get());
            cameraState.latestStatus = "Pose ready";
        }

        SmartDashboard.putBoolean(
            "PhotonVision " + cameraState.name + " Connected",
            cameraState.camera.isConnected()
        );
        SmartDashboard.putBoolean(
            "PhotonVision " + cameraState.name + " Has Targets",
            cameraState.latestResult.hasTargets()
        );
        SmartDashboard.putNumber(
            "PhotonVision " + cameraState.name + " Target Count",
            cameraState.latestResult.getTargets().size()
        );
    }

    @Override
    public void periodic() {
        for (CameraState cameraState : cameraStates) {
            processCamera(cameraState);
            SmartDashboard.putString(
                "PhotonVision " + cameraState.name + " Status",
                cameraState.latestStatus
            );
        }

        bestRobotPoseEstimate = measurementBuffer.stream()
            .filter(Objects::nonNull)
            .sorted(NEWEST_FIRST)
            .findFirst()
            .orElse(null);

        if (bestRobotPoseEstimate != null) {
            photonField.setRobotPose(bestRobotPoseEstimate.pose());
            SmartDashboard.putNumber("PhotonVision Best Pose X", bestRobotPoseEstimate.pose().getX());
            SmartDashboard.putNumber("PhotonVision Best Pose Y", bestRobotPoseEstimate.pose().getY());
            SmartDashboard.putNumber(
                "PhotonVision Best Pose Heading",
                bestRobotPoseEstimate.pose().getRotation().getDegrees()
            );
            SmartDashboard.putNumber("PhotonVision Best Tag Count", bestRobotPoseEstimate.tagCount());
        }
    }
}
