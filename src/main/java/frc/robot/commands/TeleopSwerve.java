package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private static final double TAG_AIM_KP = 0.025;
    private static final double TAG_AIM_KI = 0.0;
    private static final double TAG_AIM_KD = 0.0;
    private static final double TAG_AIM_TOLERANCE_DEGREES = 1.5;
    private static final double TAG_AIM_MAX_ANGULAR_SPEED = 2.0;
    private static final double TAG_AIM_MAX_ANGULAR_ACCELERATION = 6.0;
    private static final double MIN_AUTO_AIM_DISTANCE_METERS = 0.05;

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
        Translation2d autoAimTarget = photonVision.getAllianceAutoAimTarget();
        Translation2d robotTranslation = s_Swerve.getPose().getTranslation();
        Translation2d targetOffset = autoAimTarget.minus(robotTranslation);
        double distanceToTarget = photonVision.getDistanceToAutoAimTarget(s_Swerve.getPose());

        SmartDashboard.putBoolean("Auto Aim Enabled", aimAtTag);
        SmartDashboard.putNumber("Auto Aim Target X", autoAimTarget.getX());
        SmartDashboard.putNumber("Auto Aim Target Y", autoAimTarget.getY());
        SmartDashboard.putNumber("Auto Aim Distance", distanceToTarget);

        if (aimAtTag && distanceToTarget > MIN_AUTO_AIM_DISTANCE_METERS) {
            Rotation2d desiredHeading = targetOffset.getAngle()
                .rotateBy(
                    Rotation2d.fromDegrees(Constants.PhotonVisionConstants.cameraHeadingOffsetDegrees)
                );
            double currentHeadingDegrees = s_Swerve.getHeading().getDegrees();
            double desiredHeadingDegrees = desiredHeading.getDegrees();
            double headingErrorDegrees = desiredHeading.minus(s_Swerve.getHeading()).getDegrees();

            rotationCommand = tagAimOutputLimiter.calculate(MathUtil.clamp(
                tagAimController.calculate(currentHeadingDegrees, desiredHeadingDegrees),
                -TAG_AIM_MAX_ANGULAR_SPEED,
                TAG_AIM_MAX_ANGULAR_SPEED
            ));

            if (tagAimController.atSetpoint()) {
                rotationCommand = 0.0;
                tagAimOutputLimiter.reset(0.0);
            }

            SmartDashboard.putNumber("Auto Aim Desired Heading Degrees", desiredHeadingDegrees);
            SmartDashboard.putNumber("Auto Aim Error Degrees", headingErrorDegrees);
            SmartDashboard.putNumber("Auto Aim Rotation Command", rotationCommand);
        } else {
            tagAimController.reset();
            tagAimOutputLimiter.reset(0.0);
            SmartDashboard.putNumber("Auto Aim Desired Heading Degrees", s_Swerve.getHeading().getDegrees());
            SmartDashboard.putNumber("Auto Aim Error Degrees", 0.0);
            SmartDashboard.putNumber("Auto Aim Rotation Command", rotationCommand);
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
