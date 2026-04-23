package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private static final double TAG_AIM_KP = 0.1; // Armando set to 0.025
  private static final double TAG_AIM_KI = 0.0;
  private static final double TAG_AIM_KD = 0.0;
  private static final double TAG_AIM_TOLERANCE_DEGREES = 1.5;
  private static final double TAG_AIM_MAX_ANGULAR_SPEED = 3;
  private static final double TAG_AIM_MAX_ANGULAR_ACCELERATION = 6.0;
  private static final double TAG_AIM_MANUAL_BLEND = 0.35;
  private static final double DASHBOARD_UPDATE_INTERVAL_SECONDS = 0.10;
  private static final String DASHBOARD_ROOT = "TeleopSwerve/";

  private final Swerve s_Swerve;
  private final DoubleSupplier translationSup;
  private final DoubleSupplier strafeSup;
  private final DoubleSupplier rotationSup;
  private final BooleanSupplier robotCentricSup;
  private final Vision vision;
  private final BooleanSupplier aimAtTagSup;
  // private final BooleanSupplier anglerHubAimActiveSup;
  private final PIDController tagAimController =
      new PIDController(TAG_AIM_KP, TAG_AIM_KI, TAG_AIM_KD);
  private final SlewRateLimiter tagAimOutputLimiter =
      new SlewRateLimiter(TAG_AIM_MAX_ANGULAR_ACCELERATION);
  private double lastDashboardUpdateSeconds = -1.0;
  private String lastAimSource = "Manual";

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      Vision vision,
      BooleanSupplier aimAtTagSup // ,
      // BooleanSupplier anglerHubAimActiveSup
      ) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.vision = vision;
    this.aimAtTagSup = aimAtTagSup;
    // this.anglerHubAimActiveSup = anglerHubAimActiveSup;
    tagAimController.setTolerance(TAG_AIM_TOLERANCE_DEGREES);
  }

  @Override
  public void initialize() {
    tagAimController.reset();
    tagAimOutputLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    double nowSeconds = Timer.getFPGATimestamp();
    boolean updateDashboard =
        lastDashboardUpdateSeconds < 0.0
            || nowSeconds - lastDashboardUpdateSeconds >= DASHBOARD_UPDATE_INTERVAL_SECONDS;
    if (updateDashboard) {
      lastDashboardUpdateSeconds = nowSeconds;
      Pose2d pose = s_Swerve.getPose();
      SmartDashboard.putNumber(DASHBOARD_ROOT + "Pose/X", pose.getX());
      SmartDashboard.putNumber(DASHBOARD_ROOT + "Pose/Y", pose.getY());
    }

    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
    double manualRotationCommand = rotationVal * Constants.Swerve.maxAngularVelocity;
    double rotationCommand = manualRotationCommand;
    boolean fieldRelative = !robotCentricSup.getAsBoolean();

    // boolean hubAimActive = anglerHubAimActiveSup.getAsBoolean();
    boolean aimAtTag = aimAtTagSup.getAsBoolean(); // && hubAimActive;
    Vision.TrackedTag trackedTag =
        aimAtTag && vision != null
            ? vision.getBestTarget(
                Constants.TeamDependentFactors.getHubTagIds(),
                Constants.LimelightConstants.frontCamera.name)
            : null;
    double targetTagId = trackedTag == null ? -1.0 : trackedTag.tagId;
    boolean tagVisible = trackedTag != null;
    boolean lineOfSightAimAvailable =
        tagVisible && trackedTag.targetData.length > 1 && Double.isFinite(trackedTag.targetData[1]);

    if (updateDashboard) {
      SmartDashboard.putBoolean(DASHBOARD_ROOT + "TagAim/Requested", aimAtTag);
      // SmartDashboard.putBoolean("Hub Aim Active", hubAimActive);
      SmartDashboard.putBoolean(
          DASHBOARD_ROOT + "TagAim/Enabled", aimAtTag && lineOfSightAimAvailable);
      SmartDashboard.putNumber(DASHBOARD_ROOT + "TagAim/TargetID", targetTagId);
      SmartDashboard.putBoolean(DASHBOARD_ROOT + "TagAim/Visible", tagVisible);
      SmartDashboard.putNumber(DASHBOARD_ROOT + "TagAim/ManualRotation", manualRotationCommand);
      SmartDashboard.putString(
          DASHBOARD_ROOT + "TagAim/Camera",
          trackedTag == null ? "None" : trackedTag.limelight.getName());
    }

    if (aimAtTag && lineOfSightAimAvailable) {
      double yawErrorDegrees = trackedTag.targetData[1];
      lastAimSource = "Front Limelight";

      double autoRotationCommand =
          tagAimOutputLimiter.calculate(
              MathUtil.clamp(
                  tagAimController.calculate(yawErrorDegrees, 0.0),
                  -TAG_AIM_MAX_ANGULAR_SPEED,
                  TAG_AIM_MAX_ANGULAR_SPEED));

      if (tagAimController.atSetpoint()) {
        autoRotationCommand = 0.0;
        tagAimOutputLimiter.reset(0.0);
      }

      rotationCommand =
          MathUtil.clamp(
              autoRotationCommand + (manualRotationCommand * TAG_AIM_MANUAL_BLEND),
              -Constants.Swerve.maxAngularVelocity,
              Constants.Swerve.maxAngularVelocity);

      if (updateDashboard) {
        SmartDashboard.putNumber(DASHBOARD_ROOT + "TagAim/ErrorDegrees", yawErrorDegrees);
        SmartDashboard.putNumber(DASHBOARD_ROOT + "TagAim/RotationCommand", rotationCommand);
        SmartDashboard.putNumber(DASHBOARD_ROOT + "TagAim/AutoRotation", autoRotationCommand);
        SmartDashboard.putBoolean(DASHBOARD_ROOT + "TagAim/FieldPoseFallback", false);
      }
    } else {
      lastAimSource = aimAtTag ? "No Tag" : "Manual";
      tagAimController.reset();
      tagAimOutputLimiter.reset(0.0);
      if (updateDashboard) {
        SmartDashboard.putNumber(DASHBOARD_ROOT + "TagAim/ErrorDegrees", 0.0);
        SmartDashboard.putNumber(DASHBOARD_ROOT + "TagAim/RotationCommand", rotationCommand);
        SmartDashboard.putNumber(DASHBOARD_ROOT + "TagAim/AutoRotation", 0.0);
        SmartDashboard.putBoolean(DASHBOARD_ROOT + "TagAim/FieldPoseFallback", false);
      }
    }

    if (updateDashboard) {
      SmartDashboard.putString(DASHBOARD_ROOT + "TagAim/Source", lastAimSource);
    }

    double speedLimit = Constants.Swerve.maxSpeed;
    Translation2d requestedTranslation = new Translation2d(translationVal, strafeVal);
    if (fieldRelative) {
      requestedTranslation = requestedTranslation.rotateBy(s_Swerve.getDriverForwardHeading());
    }

    if (updateDashboard) {
      SmartDashboard.putNumber(
          DASHBOARD_ROOT + "Driver/ForwardAppliedHeading",
          s_Swerve.getDriverForwardHeading().getDegrees());
    }

    s_Swerve.drive(requestedTranslation.times(speedLimit), rotationCommand, fieldRelative, true);
  }
}
