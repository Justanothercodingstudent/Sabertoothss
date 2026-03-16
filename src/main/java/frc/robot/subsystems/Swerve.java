package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    public final SwerveModule[] mSwerveMods;
    public final Pigeon2 gyro;

    private final PhotonVisionSubsystem photonVision;

    private Rotation2d lastKnownTagHeading = new Rotation2d();
    private Rotation2d originalHeading = new Rotation2d();
    private Rotation2d driverHeadingOffset = new Rotation2d();

    private boolean autonMovingEnabled = true;
    private boolean hasFieldPoseReference = false;
    private boolean hasAcceptedVisionMeasurement = false;
    private double lastVisionTimestampSeconds = -1.0;
    private Pose2d lastAcceptedVisionPose = new Pose2d();
    private String lastVisionRejectReason = "No vision measurements processed";

    public Swerve(PhotonVisionSubsystem photonVision) {
        this.photonVision = photonVision;

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(Constants.Swerve.SwerveStartHeading);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions()
        );
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(ChassisSpeeds speeds) {
        if (!autonMovingEnabled) {
            speeds = new ChassisSpeeds();
        }

        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(moduleStates[mod.moduleNumber], true);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getDriverHeading()
                )
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxSpeed);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(moduleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public TalonFX[] getTalons() {
        TalonFX[] talons = new TalonFX[8];
        for (int i = 0; i < 4; i++) {
            talons[i * 2] = mSwerveMods[i].getTalons()[0];
            talons[i * 2 + 1] = mSwerveMods[i].getTalons()[1];
        }
        return talons;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void enableAutonMoving() {
        autonMovingEnabled = true;
    }

    public void disableAutonMoving() {
        autonMovingEnabled = false;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometryAuto(Pose2d pose) {
        setPose(pose);
    }

    public void setPose(Pose2d pose) {
        hasFieldPoseReference = true;
        resetPoseTrackers(pose);
    }

    public void resetPose(Pose2d pose) {
        setPose(pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Rotation2d getDriverHeading() {
        return getGyroYaw().minus(driverHeadingOffset);
    }

    public boolean hasFieldPoseReference() {
        return hasFieldPoseReference;
    }

    public void setHeading(Rotation2d heading) {
        resetPoseTrackers(new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        driverHeadingOffset = getGyroYaw();
    }

    public Command flipHeading() {
        return new InstantCommand(() -> resetPoseTrackers(
            new Pose2d(getPose().getTranslation(), getHeading().rotateBy(Rotation2d.fromDegrees(180.0)))
        ));
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void setOriginalHeading(Rotation2d heading) {
        originalHeading = heading;
    }

    private void resetPoseTrackers(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        lastVisionTimestampSeconds = photonVision == null
            ? -1.0
            : photonVision.getBestEstimatedPose()
                .map(PhotonVisionSubsystem.VisionMeasurement::timestampSeconds)
                .orElse(-1.0);
    }

    private boolean isPoseWithinField(Pose2d pose) {
        double margin = Constants.PhotonVisionConstants.visionFieldBoundaryMarginMeters;
        return pose.getX() >= -margin
            && pose.getX() <= Constants.FieldConstants.fieldLengthMeters + margin
            && pose.getY() >= -margin
            && pose.getY() <= Constants.FieldConstants.fieldWidthMeters + margin;
    }

    private boolean shouldSeedPoseFromVision(PhotonVisionSubsystem.VisionMeasurement measurement) {
        if (measurement == null || measurement.tagCount() < Constants.PhotonVisionConstants.minVisionSeedTagCount) {
            return false;
        }

        if (hasFieldPoseReference || hasAcceptedVisionMeasurement) {
            return false;
        }

        return poseEstimator.getEstimatedPosition().getTranslation().getNorm()
                <= Constants.PhotonVisionConstants.poseSeedOriginToleranceMeters
            && swerveOdometry.getPoseMeters().getTranslation().getNorm()
                <= Constants.PhotonVisionConstants.poseSeedOriginToleranceMeters;
    }

    private String getVisionMeasurementRejectReason(PhotonVisionSubsystem.VisionMeasurement measurement) {
        if (measurement == null || measurement.tagCount() <= 0 || measurement.timestampSeconds() <= 0.0) {
            return "Missing tags or timestamp";
        }

        Pose2d measuredPose = measurement.pose();
        if (!Double.isFinite(measuredPose.getX())
            || !Double.isFinite(measuredPose.getY())
            || !Double.isFinite(measuredPose.getRotation().getRadians())) {
            return "Pose contained NaN/Inf";
        }

        if (!isPoseWithinField(measuredPose)) {
            return "Pose outside field";
        }

        if (measurement.averageTagArea() < Constants.PhotonVisionConstants.minVisionTargetArea) {
            return "Target area too small";
        }

        if (measurement.tagCount() == 1) {
            if (measurement.averageTagDistanceMeters() > Constants.PhotonVisionConstants.maxSingleTagDistanceMeters) {
                return "Single-tag distance too large";
            }

            if (measurement.bestTargetAmbiguity() >= 0.0
                && measurement.bestTargetAmbiguity() > Constants.PhotonVisionConstants.maxSingleTagAmbiguity) {
                return "Single-tag ambiguity too high";
            }
        } else if (measurement.averageTagDistanceMeters() > Constants.PhotonVisionConstants.maxMultiTagDistanceMeters) {
            return "Multi-tag distance too large";
        }

        if (shouldSeedPoseFromVision(measurement)) {
            return null;
        }

        if (!hasFieldPoseReference) {
            return "Waiting for valid field seed";
        }

        double poseDeltaMeters = measuredPose.getTranslation()
            .getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        double maxPoseDelta = measurement.tagCount() > 1
            ? Constants.PhotonVisionConstants.maxMultiTagPoseDeltaMeters
            : Constants.PhotonVisionConstants.maxSingleTagPoseDeltaMeters;

        SmartDashboard.putNumber("Vision Pose Delta", poseDeltaMeters);
        SmartDashboard.putNumber("Vision Max Pose Delta", maxPoseDelta);

        if (poseDeltaMeters > maxPoseDelta) {
            return "Pose delta too large";
        }

        return null;
    }

    private double getVisionTranslationStdDev(PhotonVisionSubsystem.VisionMeasurement measurement) {
        double stdDev = Constants.PhotonVisionConstants.visionStdDevBase
            + measurement.averageTagDistanceMeters()
                * Constants.PhotonVisionConstants.visionStdDevPerMeter
                / Math.max(1, measurement.tagCount());

        if (measurement.tagCount() == 1) {
            stdDev *= Constants.PhotonVisionConstants.singleTagStdDevMultiplier;
        }

        if (measurement.averageTagArea() < 0.15) {
            stdDev *= Constants.PhotonVisionConstants.lowAreaStdDevMultiplier;
        }

        return Math.max(0.05, Math.min(stdDev, 2.0));
    }

    private void addVisionMeasurementsIfAvailable() {
        if (photonVision == null) {
            SmartDashboard.putBoolean("Vision Measurement Accepted", false);
            SmartDashboard.putString("Vision Reject Reason", "PhotonVision subsystem missing");
            return;
        }

        List<PhotonVisionSubsystem.VisionMeasurement> measurements =
            photonVision.getVisionMeasurementsSince(lastVisionTimestampSeconds);

        boolean acceptedThisCycle = false;
        boolean seededThisCycle = false;
        String rejectReason = measurements.isEmpty()
            ? photonVision.getStatusSummary()
            : "No valid measurements this cycle";

        for (PhotonVisionSubsystem.VisionMeasurement measurement : measurements) {
            String measurementRejectReason = getVisionMeasurementRejectReason(measurement);
            if (measurementRejectReason != null) {
                rejectReason = measurementRejectReason;
                continue;
            }

            if (shouldSeedPoseFromVision(measurement)) {
                hasFieldPoseReference = true;
                resetPoseTrackers(measurement.pose());
                seededThisCycle = true;
            } else {
                double stdDev = getVisionTranslationStdDev(measurement);
                poseEstimator.addVisionMeasurement(
                    measurement.pose(),
                    measurement.timestampSeconds(),
                    VecBuilder.fill(
                        stdDev,
                        stdDev,
                        Constants.PhotonVisionConstants.visionRotationStdDev
                    )
                );
                SmartDashboard.putNumber("Vision Std Dev XY", stdDev);
            }

            lastVisionTimestampSeconds = measurement.timestampSeconds();
            lastAcceptedVisionPose = measurement.pose();
            hasAcceptedVisionMeasurement = true;
            hasFieldPoseReference = true;
            acceptedThisCycle = true;
            rejectReason = "Accepted";

            SmartDashboard.putNumber("Vision Tag Count", measurement.tagCount());
            SmartDashboard.putNumber("Vision Avg Tag Dist", measurement.averageTagDistanceMeters());
            SmartDashboard.putNumber("Vision Avg Tag Area", measurement.averageTagArea());
            SmartDashboard.putString("Vision Camera", measurement.cameraName());
        }

        lastVisionRejectReason = rejectReason;
        SmartDashboard.putBoolean("Vision Measurement Accepted", hasAcceptedVisionMeasurement);
        SmartDashboard.putBoolean("Vision Measurement Accepted This Cycle", acceptedThisCycle);
        SmartDashboard.putBoolean("Vision Pose Seeded This Cycle", seededThisCycle);
        SmartDashboard.putString("Vision Reject Reason", lastVisionRejectReason);
    }

    public void updateParallelMotion(
        boolean parallelModeActive,
        boolean returnToOriginal,
        boolean allowRotation,
        PhotonVisionSubsystem photonVision
    ) {
        Pose2d detectedPose = photonVision == null
            ? null
            : photonVision.getBestEstimatedPose()
                .map(PhotonVisionSubsystem.VisionMeasurement::pose)
                .orElse(null);

        if (detectedPose != null) {
            lastKnownTagHeading = detectedPose.getRotation();
        }

        if (parallelModeActive && !allowRotation) {
            setHeading(lastKnownTagHeading);
        }

        if (returnToOriginal) {
            setHeading(originalHeading);
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        poseEstimator.update(getGyroYaw(), getModulePositions());
        addVisionMeasurementsIfAvailable();

        SmartDashboard.putNumber("Odometry X", getOdometryPose().getX());
        SmartDashboard.putNumber("Odometry Y", getOdometryPose().getY());
        SmartDashboard.putNumber("Estimated Pose X", getPose().getX());
        SmartDashboard.putNumber("Estimated Pose Y", getPose().getY());
        SmartDashboard.putNumber("Estimated Pose Heading", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Driver Heading", getDriverHeading().getDegrees());
        SmartDashboard.putBoolean("Field Pose Ready", hasFieldPoseReference);
        SmartDashboard.putNumber("Last Vision Pose X", lastAcceptedVisionPose.getX());
        SmartDashboard.putNumber("Last Vision Pose Y", lastAcceptedVisionPose.getY());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Pigeon Yaw", gyro.getYaw().getValueAsDouble());
        }
    }
}
