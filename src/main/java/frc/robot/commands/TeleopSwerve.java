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

        if (aimAtTag) {
            if (!hasLatchedAutoAimTarget) {
                latchedAutoAimTarget = configuredAutoAimTarget;
                hasLatchedAutoAimTarget = true;
            }

            autoAimTarget = latchedAutoAimTarget;
        } else {
            hasLatchedAutoAimTarget = false;
        }

        Translation2d robotTranslation = aimPose.getTranslation();
        Translation2d targetOffset = autoAimTarget.minus(robotTranslation);
        double distanceToTarget = photonVision.getDistanceToAutoAimTarget(aimPose);
        Rotation2d gyroHeading = s_Swerve.getGyroYaw();
        Rotation2d currentHeading = gyroHeading;
        Rotation2d desiredHeading = gyroHeading;
        Rotation2d rawDesiredHeading = gyroHeading;
        double headingErrorDegrees = 0.0;
        boolean usedLatchedHeading = false;
        String autoAimSource = "Manual";

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
            if (directAutoAimTarget.isPresent()) {
                PhotonVisionSubsystem.RobotRelativeTargetObservation observation =
                    directAutoAimTarget.get();
                double robotRelativeYawDegrees =
                    observation.robotRelativeYawDegrees()
                        + Constants.PhotonVisionConstants.cameraHeadingOffsetDegrees;

                currentHeading = gyroHeading;
                rawDesiredHeading = gyroHeading.rotateBy(
                    Rotation2d.fromDegrees(robotRelativeYawDegrees)
                );
                desiredHeading = rawDesiredHeading;
                headingErrorDegrees = robotRelativeYawDegrees;
                rotationCommand = tagAimOutputLimiter.calculate(MathUtil.clamp(
                    tagAimController.calculate(0.0, headingErrorDegrees),
                    -TAG_AIM_MAX_ANGULAR_SPEED,
                    TAG_AIM_MAX_ANGULAR_SPEED
                ));
                latchedAutoAimHeading = desiredHeading;
                hasLatchedAutoAimHeading = true;
                autoAimSource = "DIRECT_TAG";
            } else if (fieldPoseReady) {
                currentHeading = s_Swerve.getHeading();
                rawDesiredHeading = targetOffset.getAngle().rotateBy(
                    Rotation2d.fromDegrees(Constants.PhotonVisionConstants.cameraHeadingOffsetDegrees)
                );
                desiredHeading = rawDesiredHeading;

                if (!hasLatchedAutoAimHeading) {
                    latchedAutoAimHeading = rawDesiredHeading;
                    hasLatchedAutoAimHeading = true;
                } else if (distanceToTarget <= AUTO_AIM_CROSSOVER_LOCK_DISTANCE_METERS) {
                    desiredHeading = latchedAutoAimHeading;
                    usedLatchedHeading = true;
                } else {
                    double desiredHeadingJumpDegrees =
                        rawDesiredHeading.minus(latchedAutoAimHeading).getDegrees();

                    if (distanceToTarget <= AUTO_AIM_HEADING_JUMP_GUARD_DISTANCE_METERS
                        && Math.abs(desiredHeadingJumpDegrees) > AUTO_AIM_MAX_HEADING_JUMP_DEGREES) {
                        desiredHeading = latchedAutoAimHeading;
                        usedLatchedHeading = true;
                    } else {
                        latchedAutoAimHeading = rawDesiredHeading;
                    }
                }

                headingErrorDegrees = desiredHeading.minus(currentHeading).getDegrees();
                rotationCommand = tagAimOutputLimiter.calculate(MathUtil.clamp(
                    tagAimController.calculate(
                        currentHeading.getDegrees(),
                        desiredHeading.getDegrees()
                    ),
                    -TAG_AIM_MAX_ANGULAR_SPEED,
                    TAG_AIM_MAX_ANGULAR_SPEED
                ));
                autoAimSource = "FIELD_TARGET";
            } else if (hasLatchedAutoAimHeading) {
                currentHeading = gyroHeading;
                desiredHeading = latchedAutoAimHeading;
                rawDesiredHeading = latchedAutoAimHeading;
                headingErrorDegrees = desiredHeading.minus(currentHeading).getDegrees();
                rotationCommand = tagAimOutputLimiter.calculate(MathUtil.clamp(
                    tagAimController.calculate(
                        currentHeading.getDegrees(),
                        desiredHeading.getDegrees()
                    ),
                    -TAG_AIM_MAX_ANGULAR_SPEED,
                    TAG_AIM_MAX_ANGULAR_SPEED
                ));
                usedLatchedHeading = true;
                autoAimSource = "LATCHED_HEADING";
            } else {
                tagAimController.reset();
                tagAimOutputLimiter.reset(0.0);
                autoAimSource = "WAITING_FOR_TAG";
            }

            if (autoAimSource.equals("DIRECT_TAG")
                || autoAimSource.equals("FIELD_TARGET")
                || autoAimSource.equals("LATCHED_HEADING")) {
                if (Math.abs(headingErrorDegrees) <= TAG_AIM_TOLERANCE_DEGREES
                    || tagAimController.atSetpoint()) {
                    rotationCommand = 0.0;
                    tagAimOutputLimiter.reset(0.0);
                }
            } else {
                rotationCommand = 0.0;
            }
            SmartDashboard.putNumber("Auto Aim Current Heading Degrees", currentHeading.getDegrees());
            SmartDashboard.putNumber("Auto Aim Desired Heading Degrees", desiredHeading.getDegrees());
            SmartDashboard.putNumber("Auto Aim Raw Desired Heading Degrees", rawDesiredHeading.getDegrees());
            SmartDashboard.putNumber("Auto Aim Error Degrees", headingErrorDegrees);
            SmartDashboard.putNumber("Auto Aim Rotation Command", rotationCommand);
            SmartDashboard.putBoolean("Auto Aim Heading Latched", usedLatchedHeading);
            SmartDashboard.putString("Auto Aim Source", autoAimSource);
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
            SmartDashboard.putString("Auto Aim Source", "Manual");
        }

        double speedLimit = Constants.Swerve.maxSpeed;

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(speedLimit), 
            rotationCommand, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
