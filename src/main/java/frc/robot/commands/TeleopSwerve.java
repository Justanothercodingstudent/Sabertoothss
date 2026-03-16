package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;


public class TeleopSwerve extends Command {    
    private static final double TAG_AIM_KP = 0.015;
    private static final double TAG_AIM_KI = 0.0;
    private static final double TAG_AIM_KD = 0.0;
    private static final double TAG_AIM_TOLERANCE_DEGREES = 2.5;
    private static final double TAG_AIM_MAX_ANGULAR_SPEED = 1.0;
    private static final double TAG_AIM_MAX_ANGULAR_ACCELERATION = 3.0;
    private static final double MIN_AUTO_AIM_DISTANCE_METERS = 0.05;
    private static final double AUTO_AIM_CROSSOVER_LOCK_DISTANCE_METERS = 0.75;
    private static final double AUTO_AIM_HEADING_JUMP_GUARD_DISTANCE_METERS = 1.5;
    private static final double AUTO_AIM_MAX_HEADING_JUMP_DEGREES = 100.0;
    private static final String AUTO_AIM_SOURCE_MANUAL = "Manual";
    private static final String AUTO_AIM_SOURCE_DIRECT_TAG = "DIRECT_TAG";
    private static final String AUTO_AIM_SOURCE_FIELD_TARGET = "FIELD_TARGET";
    private static final String AUTO_AIM_SOURCE_LATCHED_HEADING = "LATCHED_HEADING";
    private static final String AUTO_AIM_SOURCE_WAITING_FOR_TAG = "WAITING_FOR_TAG";
    private final Swerve s_Swerve;    
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;
    private final PhotonVisionSubsystem photonVision;
    private final BooleanSupplier aimAtTagSup;
    private final PIDController tagAimController = new PIDController(TAG_AIM_KP, TAG_AIM_KI, TAG_AIM_KD);
    private final SlewRateLimiter tagAimOutputLimiter =
        new SlewRateLimiter(TAG_AIM_MAX_ANGULAR_ACCELERATION);
    private Translation2d latchedAutoAimTarget = new Translation2d();
    private boolean hasLatchedAutoAimTarget;
    private Rotation2d latchedAutoAimHeading = new Rotation2d();
    private boolean hasLatchedAutoAimHeading;

    private static record AutoAimState(
        Rotation2d currentHeading,
        Rotation2d desiredHeading,
        Rotation2d rawDesiredHeading,
        double headingErrorDegrees,
        double rotationCommand,
        boolean usedLatchedHeading,
        String source
    ) {}

    public TeleopSwerve(
            Swerve s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            PhotonVisionSubsystem photonVision,
            BooleanSupplier aimAtTagSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.photonVision = photonVision;
        this.aimAtTagSup = aimAtTagSup;
        tagAimController.enableContinuousInput(-180.0, 180.0);
        tagAimController.setTolerance(TAG_AIM_TOLERANCE_DEGREES);
    }

    @Override
    public void initialize() {
        tagAimController.reset();
        tagAimOutputLimiter.reset(0.0);
        hasLatchedAutoAimTarget = false;
        hasLatchedAutoAimHeading = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("pose", 
            s_Swerve.getPose().getX() + ", " + s_Swerve.getPose().getY());

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double rotationCommand = rotationVal * Constants.Swerve.maxAngularVelocity;

        boolean aimAtTag = aimAtTagSup.getAsBoolean();
        boolean fieldPoseReady = s_Swerve.hasFieldPoseReference();
        Pose2d odometryPose = s_Swerve.getOdometryPose();
        Pose2d fieldPose = s_Swerve.getPose();
        Pose2d aimPose = fieldPoseReady ? fieldPose : odometryPose;
        Translation2d configuredAutoAimTarget = photonVision.getAllianceAutoAimTarget();
        Translation2d autoAimTarget = configuredAutoAimTarget;
        Optional<PhotonVisionSubsystem.RobotRelativeTargetObservation> directAutoAimTarget =
            photonVision.getBestRobotRelativeAllianceTarget();

        if (aimAtTag && !hasLatchedAutoAimTarget) {
            latchedAutoAimTarget = configuredAutoAimTarget;
            hasLatchedAutoAimTarget = true;
        }

        if (aimAtTag && hasLatchedAutoAimTarget) {
            autoAimTarget = latchedAutoAimTarget;
        }

        Translation2d robotTranslation = aimPose.getTranslation();
        Translation2d targetOffset = autoAimTarget.minus(robotTranslation);
        double distanceToTarget = photonVision.getDistanceToAutoAimTarget(aimPose);
        Rotation2d gyroHeading = s_Swerve.getGyroYaw();
        AutoAimState autoAimState = new AutoAimState(
            gyroHeading,
            gyroHeading,
            gyroHeading,
            0.0,
            rotationCommand,
            false,
            AUTO_AIM_SOURCE_MANUAL
        );

        SmartDashboard.putBoolean("Auto Aim Enabled", aimAtTag);
        SmartDashboard.putBoolean("Auto Aim Pose Ready", fieldPoseReady);
        SmartDashboard.putBoolean("Auto Aim Direct Tag Visible", directAutoAimTarget.isPresent());
        SmartDashboard.putBoolean("Auto Aim Red Alliance", Constants.TeamDependentFactors.isRedAlliance());
        SmartDashboard.putNumber("Configured Auto Aim Target X", configuredAutoAimTarget.getX());
        SmartDashboard.putNumber("Configured Auto Aim Target Y", configuredAutoAimTarget.getY());
        SmartDashboard.putNumber("Auto Aim Target X", autoAimTarget.getX());
        SmartDashboard.putNumber("Auto Aim Target Y", autoAimTarget.getY());
        SmartDashboard.putNumber("Auto Aim Pose X", aimPose.getX());
        SmartDashboard.putNumber("Auto Aim Pose Y", aimPose.getY());
        SmartDashboard.putNumber("Hub Center Distance", distanceToTarget);
        SmartDashboard.putNumber(
            "Auto Aim Visible Tag ID",
            directAutoAimTarget
                .map(PhotonVisionSubsystem.RobotRelativeTargetObservation::tagId)
                .orElse(-1)
        );

        if (aimAtTag && distanceToTarget > MIN_AUTO_AIM_DISTANCE_METERS) {
            autoAimState = calculateAutoAimState(
                fieldPoseReady,
                gyroHeading,
                targetOffset,
                distanceToTarget,
                directAutoAimTarget
            );
            rotationCommand = autoAimState.rotationCommand();
            SmartDashboard.putNumber(
                "Auto Aim Current Heading Degrees",
                autoAimState.currentHeading().getDegrees()
            );
            SmartDashboard.putNumber(
                "Auto Aim Desired Heading Degrees",
                autoAimState.desiredHeading().getDegrees()
            );
            SmartDashboard.putNumber(
                "Auto Aim Raw Desired Heading Degrees",
                autoAimState.rawDesiredHeading().getDegrees()
            );
            SmartDashboard.putNumber("Auto Aim Error Degrees", autoAimState.headingErrorDegrees());
            SmartDashboard.putNumber("Auto Aim Rotation Command", rotationCommand);
            SmartDashboard.putBoolean("Auto Aim Heading Latched", autoAimState.usedLatchedHeading());
            SmartDashboard.putString("Auto Aim Source", autoAimState.source());
        } else {
            tagAimController.reset();
            tagAimOutputLimiter.reset(0.0);
            hasLatchedAutoAimTarget = false;
            hasLatchedAutoAimHeading = false;
            SmartDashboard.putNumber("Auto Aim Current Heading Degrees", gyroHeading.getDegrees());
            SmartDashboard.putNumber("Auto Aim Desired Heading Degrees", gyroHeading.getDegrees());
            SmartDashboard.putNumber("Auto Aim Raw Desired Heading Degrees", gyroHeading.getDegrees());
            SmartDashboard.putNumber("Auto Aim Error Degrees", 0.0);
            SmartDashboard.putNumber("Auto Aim Rotation Command", rotationCommand);
            SmartDashboard.putBoolean("Auto Aim Heading Latched", false);
            SmartDashboard.putString("Auto Aim Source", AUTO_AIM_SOURCE_MANUAL);
        }

        double speedLimit = Constants.Swerve.maxSpeed;

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(speedLimit), 
            rotationCommand, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }

    private AutoAimState calculateAutoAimState(
        boolean fieldPoseReady,
        Rotation2d gyroHeading,
        Translation2d targetOffset,
        double distanceToTarget,
        Optional<PhotonVisionSubsystem.RobotRelativeTargetObservation> directAutoAimTarget
    ) {
        if (directAutoAimTarget.isPresent()) {
            return calculateDirectTagAutoAimState(gyroHeading, directAutoAimTarget.get());
        }

        if (fieldPoseReady) {
            return calculateFieldTargetAutoAimState(targetOffset, distanceToTarget);
        }

        if (hasLatchedAutoAimHeading) {
            return calculateLatchedHeadingAutoAimState(gyroHeading);
        }

        tagAimController.reset();
        tagAimOutputLimiter.reset(0.0);
        return new AutoAimState(
            gyroHeading,
            gyroHeading,
            gyroHeading,
            0.0,
            0.0,
            false,
            AUTO_AIM_SOURCE_WAITING_FOR_TAG
        );
    }

    private AutoAimState calculateDirectTagAutoAimState(
        Rotation2d gyroHeading,
        PhotonVisionSubsystem.RobotRelativeTargetObservation observation
    ) {
        double robotRelativeYawDegrees =
            observation.robotRelativeYawDegrees()
                + Constants.PhotonVisionConstants.cameraHeadingOffsetDegrees;
        Rotation2d rawDesiredHeading = gyroHeading.rotateBy(Rotation2d.fromDegrees(robotRelativeYawDegrees));
        double rotationCommand = getAutoAimRotationCommand(0.0, robotRelativeYawDegrees);

        latchedAutoAimHeading = rawDesiredHeading;
        hasLatchedAutoAimHeading = true;

        return buildAutoAimState(
            gyroHeading,
            rawDesiredHeading,
            rawDesiredHeading,
            robotRelativeYawDegrees,
            rotationCommand,
            false,
            AUTO_AIM_SOURCE_DIRECT_TAG
        );
    }

    private AutoAimState calculateFieldTargetAutoAimState(
        Translation2d targetOffset,
        double distanceToTarget
    ) {
        Rotation2d currentHeading = s_Swerve.getHeading();
        Rotation2d rawDesiredHeading = targetOffset.getAngle().rotateBy(
            Rotation2d.fromDegrees(Constants.PhotonVisionConstants.cameraHeadingOffsetDegrees)
        );
        Rotation2d desiredHeading = rawDesiredHeading;
        boolean usedLatchedHeading = false;

        if (!hasLatchedAutoAimHeading) {
            latchedAutoAimHeading = rawDesiredHeading;
            hasLatchedAutoAimHeading = true;
        } else if (distanceToTarget <= AUTO_AIM_CROSSOVER_LOCK_DISTANCE_METERS) {
            desiredHeading = latchedAutoAimHeading;
            usedLatchedHeading = true;
        } else {
            double desiredHeadingJumpDegrees = rawDesiredHeading.minus(latchedAutoAimHeading).getDegrees();

            if (distanceToTarget <= AUTO_AIM_HEADING_JUMP_GUARD_DISTANCE_METERS
                && Math.abs(desiredHeadingJumpDegrees) > AUTO_AIM_MAX_HEADING_JUMP_DEGREES) {
                desiredHeading = latchedAutoAimHeading;
                usedLatchedHeading = true;
            } else {
                latchedAutoAimHeading = rawDesiredHeading;
            }
        }

        double headingErrorDegrees = desiredHeading.minus(currentHeading).getDegrees();
        double rotationCommand = getAutoAimRotationCommand(
            currentHeading.getDegrees(),
            desiredHeading.getDegrees()
        );

        return buildAutoAimState(
            currentHeading,
            desiredHeading,
            rawDesiredHeading,
            headingErrorDegrees,
            rotationCommand,
            usedLatchedHeading,
            AUTO_AIM_SOURCE_FIELD_TARGET
        );
    }

    private AutoAimState calculateLatchedHeadingAutoAimState(Rotation2d gyroHeading) {
        double headingErrorDegrees = latchedAutoAimHeading.minus(gyroHeading).getDegrees();
        double rotationCommand = getAutoAimRotationCommand(
            gyroHeading.getDegrees(),
            latchedAutoAimHeading.getDegrees()
        );

        return buildAutoAimState(
            gyroHeading,
            latchedAutoAimHeading,
            latchedAutoAimHeading,
            headingErrorDegrees,
            rotationCommand,
            true,
            AUTO_AIM_SOURCE_LATCHED_HEADING
        );
    }

    private AutoAimState buildAutoAimState(
        Rotation2d currentHeading,
        Rotation2d desiredHeading,
        Rotation2d rawDesiredHeading,
        double headingErrorDegrees,
        double rotationCommand,
        boolean usedLatchedHeading,
        String source
    ) {
        if (Math.abs(headingErrorDegrees) <= TAG_AIM_TOLERANCE_DEGREES || tagAimController.atSetpoint()) {
            tagAimOutputLimiter.reset(0.0);
            rotationCommand = 0.0;
        }

        return new AutoAimState(
            currentHeading,
            desiredHeading,
            rawDesiredHeading,
            headingErrorDegrees,
            rotationCommand,
            usedLatchedHeading,
            source
        );
    }

    private double getAutoAimRotationCommand(double currentHeadingDegrees, double desiredHeadingDegrees) {
        return tagAimOutputLimiter.calculate(MathUtil.clamp(
            tagAimController.calculate(currentHeadingDegrees, desiredHeadingDegrees),
            -TAG_AIM_MAX_ANGULAR_SPEED,
            TAG_AIM_MAX_ANGULAR_SPEED
        ));
    }
}
