package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private static final double TAG_AIM_KP = 0.15;   //Armando set to 0.025
    private static final double TAG_AIM_KI = 0.0;
    private static final double TAG_AIM_KD = 0.0;
    private static final double TAG_AIM_TOLERANCE_DEGREES = 1.5;
    private static final double TAG_AIM_MAX_ANGULAR_SPEED = 1;
    private static final double TAG_AIM_MAX_ANGULAR_ACCELERATION = 6.0;
    private static final double TAG_AIM_MANUAL_BLEND = 0.35;

    private final Swerve s_Swerve;    
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;
    private final Limelight limelight;
    private final BooleanSupplier aimAtTagSup;
    //private final BooleanSupplier anglerHubAimActiveSup;
    private final PIDController tagAimController = new PIDController(TAG_AIM_KP, TAG_AIM_KI, TAG_AIM_KD);
    private final SlewRateLimiter tagAimOutputLimiter =
        new SlewRateLimiter(TAG_AIM_MAX_ANGULAR_ACCELERATION);

    public TeleopSwerve(
            Swerve s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            Limelight aprilTagDetection,
            BooleanSupplier aimAtTagSup//,
            //BooleanSupplier anglerHubAimActiveSup
            ) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.limelight = aprilTagDetection;
        this.aimAtTagSup = aimAtTagSup;
        //this.anglerHubAimActiveSup = anglerHubAimActiveSup;
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
        double manualRotationCommand = rotationVal * Constants.Swerve.maxAngularVelocity;
        double rotationCommand = manualRotationCommand;

        //boolean hubAimActive = anglerHubAimActiveSup.getAsBoolean();
        boolean aimAtTag = aimAtTagSup.getAsBoolean() ;//&& hubAimActive;
        double targetTagId = limelight == null
            ? -1.0
            : limelight.getClosestTag(Constants.TeamDependentFactors.getHubTagIds());
        double[] tagData = targetTagId < 0.0
            ? null
            : limelight.getTarget((int) targetTagId);
        boolean tagVisible = tagData != null;

        SmartDashboard.putBoolean("Tag Aim Requested", aimAtTagSup.getAsBoolean());
        //SmartDashboard.putBoolean("Hub Aim Active", hubAimActive);
        SmartDashboard.putBoolean("Tag Aim Enabled", aimAtTag);
        SmartDashboard.putNumber("Tag Aim Target ID", targetTagId);
        SmartDashboard.putBoolean("Tag Aim Visible", tagVisible);
        SmartDashboard.putNumber("Tag Aim Manual Rotation", manualRotationCommand);

        if (aimAtTag && tagVisible) {
            double yawErrorDegrees = tagData[1];
            double autoRotationCommand = tagAimOutputLimiter.calculate(MathUtil.clamp(
                tagAimController.calculate(yawErrorDegrees, 0.0),
                -TAG_AIM_MAX_ANGULAR_SPEED,
                TAG_AIM_MAX_ANGULAR_SPEED
            ));

            if (tagAimController.atSetpoint()) {
                autoRotationCommand = 0.0;
                tagAimOutputLimiter.reset(0.0);
            }

            rotationCommand = MathUtil.clamp(
                autoRotationCommand + (manualRotationCommand * TAG_AIM_MANUAL_BLEND),
                -Constants.Swerve.maxAngularVelocity,
                Constants.Swerve.maxAngularVelocity
            );

            SmartDashboard.putNumber("Tag Aim Error Degrees", yawErrorDegrees);
            SmartDashboard.putNumber("Tag Aim Rotation Command", rotationCommand);
            SmartDashboard.putNumber("Tag Aim Auto Rotation", autoRotationCommand);
        } else {
            tagAimController.reset();
            tagAimOutputLimiter.reset(0.0);
            SmartDashboard.putNumber("Tag Aim Error Degrees", 0.0);
            SmartDashboard.putNumber("Tag Aim Rotation Command", rotationCommand);
            SmartDashboard.putNumber("Tag Aim Auto Rotation", 0.0);
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
