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
import edu.wpi.first.math.geometry.Pose3d;
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
        private final Transform3d robotToCamera;
        private PhotonPipelineResult latestResult = new PhotonPipelineResult();
        private VisionMeasurement latestMeasurement;
        private String latestStatus = "No frames received yet";

        private CameraState(
            String name,
            Transform3d robotToCamera,
            PhotonPoseEstimator poseEstimator
        ) {
            this.name = name;
            this.camera = new PhotonCamera(name);
            this.robotToCamera = robotToCamera;
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
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private final Field2d photonVisionField = new Field2d();
    private final CameraState[] cameraStates = new CameraState[] {
        new CameraState(
            Constants.PhotonVisionConstants.leftCameraName,
            Constants.PhotonVisionConstants.robotToLeftCamera,
            new PhotonPoseEstimator(fieldLayout, Constants.PhotonVisionConstants.robotToLeftCamera)
        ),
        new CameraState(
            Constants.PhotonVisionConstants.rightCameraName,
            Constants.PhotonVisionConstants.robotToRightCamera,
            new PhotonPoseEstimator(fieldLayout, Constants.PhotonVisionConstants.robotToRightCamera)
        )
    };

    private VisionMeasurement bestRobotPoseEstimate;

    public PhotonVisionSubsystem() {
        SmartDashboard.putData("PhotonVision Field", photonVisionField);
    }

    public Optional<VisionMeasurement> getBestEstimatedPose() {
        return Optional.ofNullable(bestRobotPoseEstimate);
    }

    public String getStatusSummary() {
        StringBuilder statusSummary = new StringBuilder();

        for (int i = 0; i < cameraStates.length; i++) {
            if (i > 0) {
                statusSummary.append(" | ");
            }

            CameraState cameraState = cameraStates[i];
            statusSummary
                .append(cameraState.name)
                .append(": ")
                .append(cameraState.latestStatus);
        }

        return statusSummary.toString();
    }

    public List<VisionMeasurement> getVisionMeasurementsSince(double timestampSeconds) {
        return Arrays.stream(cameraStates)
            .map(cameraState -> cameraState.latestMeasurement)
            .filter(Objects::nonNull)
            .filter(measurement -> measurement.timestampSeconds() > timestampSeconds)
            .sorted(OLDEST_FIRST)
            .toList();
    }

    public Optional<RobotRelativeTargetObservation> getBestRobotRelativeAllianceTarget() {
        boolean preferRedAlliance = Constants.TeamDependentFactors.isRedAlliance();
        Comparator<RobotRelativeTargetObservation> targetComparator =
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
            .sorted(targetComparator)
            .findFirst();
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
        return getBestRobotRelativeAllianceTarget()
            .map(RobotRelativeTargetObservation::distanceMeters)
            .orElseGet(() -> getAllianceAutoAimTarget().getDistance(robotPose.getTranslation()));
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
        Translation2d robotRelativeTagTranslation = getRobotRelativeTagTranslation(cameraState, target);
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

    private Translation2d getRobotRelativeTagTranslation(
        CameraState cameraState,
        PhotonTrackedTarget target
    ) {
        return getRobotRelativeTagPose(cameraState, target).getTranslation();
    }

    private Optional<Translation2d> getTagToAutoAimTargetInTagFrame(
        int tagId,
        Translation2d autoAimTarget
    ) {
        return fieldLayout.getTagPose(tagId)
            .map(Pose3d::toPose2d)
            .map(tagFieldPose -> autoAimTarget
                .minus(tagFieldPose.getTranslation())
                .rotateBy(tagFieldPose.getRotation().unaryMinus()));
    }

    private void updateCameraState(CameraState cameraState) {
        List<PhotonPipelineResult> unreadResults = cameraState.camera.getAllUnreadResults();
        if (unreadResults.isEmpty()) {
            if (!cameraState.camera.isConnected()) {
                cameraState.latestStatus = "Camera disconnected";
            }
            return;
        }

        PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size() - 1);
        cameraState.latestResult = latestResult;
        cameraState.latestMeasurement = buildVisionMeasurement(cameraState, latestResult).orElse(null);
    }

    private boolean hasFieldLayoutTag(PhotonPipelineResult result) {
        return result.getTargets().stream()
            .mapToInt(PhotonTrackedTarget::getFiducialId)
            .filter(fiducialId -> fiducialId >= 0)
            .anyMatch(fiducialId -> fieldLayout.getTagPose(fiducialId).isPresent());
    }

    private Optional<VisionMeasurement> buildVisionMeasurement(
        CameraState cameraState,
        PhotonPipelineResult result
    ) {
        if (!result.hasTargets()) {
            cameraState.latestStatus = "No AprilTags detected";
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        String estimationMode = "LOWEST_AMBIGUITY";

        if (result.getTargets().size() > 1) {
            estimatedRobotPose = cameraState.poseEstimator.estimateCoprocMultiTagPose(result);
            estimationMode = "MULTI_TAG_PNP_ON_COPROCESSOR";

            if (estimatedRobotPose.isEmpty()) {
                estimatedRobotPose = cameraState.poseEstimator.estimateLowestAmbiguityPose(result);
                estimationMode = "LOWEST_AMBIGUITY fallback";
            }
        } else {
            estimatedRobotPose = cameraState.poseEstimator.estimateLowestAmbiguityPose(result);
        }

        if (estimatedRobotPose.isEmpty()) {
            cameraState.latestStatus = hasFieldLayoutTag(result)
                ? "Field tags seen, but no 3D pose estimate. Check calibration and 3D mode"
                : "Detected tag IDs not in loaded field layout";
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
        cameraState.latestStatus = "Pose ready via " + estimationMode;

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
                "PhotonVision " + cameraState.name + " Has Targets",
                cameraState.latestResult.hasTargets()
            );
            SmartDashboard.putNumber(
                "PhotonVision " + cameraState.name + " Target Count",
                cameraState.latestResult.getTargets().size()
            );
        }

        bestRobotPoseEstimate = Arrays.stream(cameraStates)
            .map(cameraState -> cameraState.latestMeasurement)
            .filter(Objects::nonNull)
            .sorted(NEWEST_FIRST)
            .findFirst()
            .orElse(null);

        if (bestRobotPoseEstimate != null) {
            photonVisionField.setRobotPose(bestRobotPoseEstimate.pose());
        }
    }
}
