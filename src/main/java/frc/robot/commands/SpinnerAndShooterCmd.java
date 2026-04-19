package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShootOnMoveCalculator;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.intake;

public class SpinnerAndShooterCmd extends Command {
  private static final double DASHBOARD_UPDATE_INTERVAL_SECONDS = 0.10;

  private final SpinnerAndShooter shoot;
  private final intake Intake;
  private final Angler angler;
  private final XboxController Operator;
  private final XboxController Driver;
  private final Vision vision;
  private final Swerve swerve;

  private double SpinSpeed;
  private double Rollerspeed;
  private double lastDashboardUpdateSeconds = -1.0;

  public SpinnerAndShooterCmd(
      SpinnerAndShooter shoot,
      XboxController Operator,
      Angler angler,
      intake Intake,
      Vision vision,
      XboxController Driver,
      Swerve swerve) {
    this.shoot = shoot;
    addRequirements(this.shoot);

    this.angler = angler;
    this.Intake = Intake;
    this.vision = vision;
    this.Operator = Operator;
    this.Driver = Driver;
    this.swerve = swerve;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (DriverStation.isTeleop()) {
      double nowSeconds = Timer.getFPGATimestamp();
      boolean updateDashboard =
          lastDashboardUpdateSeconds < 0.0
              || nowSeconds - lastDashboardUpdateSeconds >= DASHBOARD_UPDATE_INTERVAL_SECONDS;
      if (updateDashboard) {
        lastDashboardUpdateSeconds = nowSeconds;
      }

      boolean rtPressed =
          Operator.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
      boolean rbPressed = Operator.getRightBumperButton();
      boolean xpressed = Operator.getXButton();

      boolean DriverRT =
          Driver.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
      boolean driverAimPressed = Driver.getAButton();

      if (updateDashboard) {
        SmartDashboard.putBoolean("Right Trigger Button Pressed", rtPressed); // Debugging
      }

      // if (rbPressed){
      //     Constants.Spin.ShootSpeed -= 1;
      //     Constants.Spin.ShootReq -= 1;
      // } else if (lbPressed){
      //     Constants.Spin.ShootSpeed += 1;
      //     Constants.Spin.ShootReq += 1;
      // }

      // double ShootSpeed = Constants.Spin.ShootSpeed;
      // ShootReq = Constants.Spin.ShootReq;
      // SmartDashboard.putNumber("Shoot Speed Target", ShootSpeed);

      if (rtPressed && !xpressed) {
        Vision.TrackedTag trackedTag =
            vision != null
                ? vision.getBestTarget(
                    Constants.TeamDependentFactors.getHubTagIds(),
                    Constants.LimelightConstants.frontCamera.name)
                : null;
        ShootOnMoveCalculator.ShotSolution shotSolution = getLineOfSightShotSolution(trackedTag);
        double trackedDistanceMeters = trackedTag == null ? -1.0 : trackedTag.distanceMeters;
        double anglerTarget =
            shotSolution == null ? angler.getAngleTarget() : shotSolution.getHoodAngleRotations();
        double tableShootSpeed =
            shotSolution == null
                ? Constants.Spin.getShooterRPSForAngle(anglerTarget)
                : shotSolution.getShooterTargetRPS();
        double shootRequirement =
            shotSolution == null
                ? Constants.Spin.getShooterFeedRPSForAngle(anglerTarget)
                : shotSolution.getShooterFeedRPS();
        boolean robotAimed =
            !driverAimPressed
                || shotSolution == null
                || shotSolution.isRobotAimed(new Rotation2d());

        if (updateDashboard) {
          SmartDashboard.putNumber("Shooter Hub Tag Distance", trackedDistanceMeters);
          SmartDashboard.putNumber("Shooter Aim Distance", angler.getLastAimDistanceMeters());
          SmartDashboard.putString(
              "Shooter Aim Distance Source", angler.getLastAimDistanceSource());
          SmartDashboard.putNumber("Shooter Angler Target", anglerTarget);
          SmartDashboard.putNumber("Shoot Speed Target", tableShootSpeed);
          SmartDashboard.putNumber("Shoot Feed Requirement", shootRequirement);
          SmartDashboard.putBoolean("Shoot On Move Ready To Feed", robotAimed);
          SmartDashboard.putBoolean(
              "Shoot On Move Active", shotSolution != null && shotSolution.isCompensationActive());
          SmartDashboard.putNumber(
              "Shoot On Move Shooter RPS Correction",
              shotSolution == null ? 0.0 : shotSolution.getShooterRpsCorrection());
          SmartDashboard.putNumber(
              "Shoot On Move Aim Error",
              shotSolution == null ? 0.0 : shotSolution.getYawErrorDegrees(new Rotation2d()));
          SmartDashboard.putNumber(
              "Shoot On Move Release Delay",
              shotSolution == null ? 0.0 : shotSolution.getReleaseDelaySeconds());
          SmartDashboard.putNumber(
              "Shoot On Move Launch Angle",
              shotSolution == null ? 0.0 : shotSolution.getEstimatedLaunchAngleDegrees());
          SmartDashboard.putNumber(
              "Shoot On Move Exit Velocity",
              shotSolution == null
                  ? 0.0
                  : shotSolution.getEstimatedNoteExitVelocityMetersPerSecond());
          SmartDashboard.putString(
              "Shooter Hub Tag Camera",
              trackedTag == null ? "None" : trackedTag.limelight.getName());
        }

        SpinSpeed = Constants.Spin.SpinSpeed;
        Rollerspeed = Constants.Spin.Rollerspeed;

        if (Constants.Spin.ClosedLoopShooter) {
          shoot.setShooterRPS(tableShootSpeed, tableShootSpeed);
        } else {
          shoot.OpenShootSpeed(Constants.Spin.ShootSpeed);
        }

        // shoot.setShooterRPS(ShootSpeed, ShootSpeed);

        if (shoot.isAtOrAboveRPS(shootRequirement) && robotAimed) {
          shoot.roller(Rollerspeed);
          shoot.SpinSpeed(SpinSpeed);
        } else {
          shoot.SpinSpeed(0);
          shoot.roller(0);
        }
      } else if (DriverRT && xpressed) {
        shoot.setShooterRPS(Constants.Spin.PassingShootSpeed, Constants.Spin.PassingShootSpeed);
        if (shoot.isAtOrAboveRPS(Constants.Spin.PassingShootReq)) {
          shoot.roller(Constants.Spin.Rollerspeed);
          shoot.SpinSpeed(Constants.Spin.SpinSpeed);
        } else {
          shoot.SpinSpeed(0);
          shoot.roller(0);
        }
      } else if (rbPressed) {
        Rollerspeed = Constants.Spin.Rollerspeed;
        shoot.roller(-Rollerspeed);
      } else {
        shoot.SpinSpeed(0);
        shoot.OpenShootSpeed(0);
        shoot.roller(0);
      }
    }
  }

  private ShootOnMoveCalculator.ShotSolution getLineOfSightShotSolution(
      Vision.TrackedTag trackedTag) {
    if (swerve == null
        || trackedTag == null
        || trackedTag.distanceMeters <= 0.0
        || trackedTag.targetData.length <= 1
        || !Double.isFinite(trackedTag.targetData[1])) {
      return null;
    }

    return ShootOnMoveCalculator.calculateLineOfSight(
        trackedTag.targetData[1], trackedTag.distanceMeters, swerve.getChassisSpeeds());
  }
}
