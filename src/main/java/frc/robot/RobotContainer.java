// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LimelightConstants.IMUmode;
import frc.robot.autos.AutoController;
import frc.robot.autos.Autos;
import frc.robot.commands.AnglerCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SpinnerAndShooterCmd;
// import frc.robot.commands.climberCmd;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SpinnerAndShooter;
// import frc.robot.subsystems.climber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final double AUTO_HEADING_DASHBOARD_UPDATE_INTERVAL_SECONDS = 0.25;

  // Previous manual team-color override kept for reference only.
  // private SendableChooser<Constants.TeamDependentFactors.TeamColorSelection> teamColorChooser;

  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kX.value);

  /* Subsystems */
  private final Swerve s_Swerve;
  private final Limelight frontLimelight;
  private final Limelight rearLimelight;
  private final Vision vision;
  private final intake Intake;
  private final SpinnerAndShooter Spin;
  // private final climber climb;
  private final Angler angler;

  /* Commands */
  private IntakeCmd intakeCmd;
  private SpinnerAndShooterCmd ShootCmd;
  // private climberCmd climbCmd;
  private AnglerCmd anglerCmd;

  private AutoController autoController;

  private SendableChooser<Command> chooser;
  private SendableChooser<IMUmode> ImuMode;
  private Pose2d lastSeededAutoPose;
  private boolean autoHeadingSeedLocked;
  private double lastAutoHeadingDashboardUpdateSeconds = -1.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    frontLimelight = new Limelight(Constants.LimelightConstants.frontCamera);
    rearLimelight = new Limelight(Constants.LimelightConstants.rearCamera);
    vision = new Vision(frontLimelight, rearLimelight);
    s_Swerve = new Swerve(vision);

    angler = new Angler(vision);
    anglerCmd = new AnglerCmd(angler, operator, driver, s_Swerve);
    angler.setDefaultCommand(anglerCmd);

    Intake = new intake();
    intakeCmd = new IntakeCmd(Intake, operator, driver);
    Intake.setDefaultCommand(intakeCmd);

    Spin = new SpinnerAndShooter();
    ShootCmd = new SpinnerAndShooterCmd(Spin, operator, angler, Intake, vision, driver, s_Swerve);
    Spin.setDefaultCommand(ShootCmd);

    // climb = new climber();
    // climbCmd = new climberCmd(climb, driver);
    // climb.setDefaultCommand(climbCmd);

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> false,
            vision,
            driver::getAButton));

    autoController = new AutoController(Intake, Spin, angler);
    Autos.registerNamedCommands(s_Swerve, autoController);
    configureAutoSelector();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
  }

  private void configureAutoSelector() {
    chooser = Autos.buildChooser();
    SmartDashboard.putData("Auto Mode", chooser);
    SmartDashboard.putString(
        "Driver Station Alliance", DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));

    // Previous manual team-color override kept for reference only.
    ImuMode = Constants.LimelightConstants.getImuMode();
    SmartDashboard.putData("IMU Mode", ImuMode);
  }

  public void updateSelectedAutoHeadingDashboard() {
    updateSelectedAutoHeadingDashboard(false);
  }

  private Pose2d updateSelectedAutoHeadingDashboard(boolean force) {
    Command selectedAuto = getSelectedAutoCommand();
    Pose2d autoStartingPose = Autos.getStartingPose(selectedAuto);

    double nowSeconds = Timer.getFPGATimestamp();
    boolean updateDashboard =
        force
            || lastAutoHeadingDashboardUpdateSeconds < 0.0
            || nowSeconds - lastAutoHeadingDashboardUpdateSeconds
                >= AUTO_HEADING_DASHBOARD_UPDATE_INTERVAL_SECONDS;
    if (updateDashboard) {
      lastAutoHeadingDashboardUpdateSeconds = nowSeconds;
      SmartDashboard.putString(
          "Selected Auto", selectedAuto != null ? selectedAuto.getName() : "None");
      SmartDashboard.putBoolean("Auto Start Pose Available", autoStartingPose != null);
      SmartDashboard.putBoolean("Auto Heading Seed Locked", autoHeadingSeedLocked);

      if (autoStartingPose != null) {
        SmartDashboard.putNumber("Auto Start X", autoStartingPose.getX());
        SmartDashboard.putNumber("Auto Start Y", autoStartingPose.getY());
        SmartDashboard.putNumber("Auto Seed Heading", autoStartingPose.getRotation().getDegrees());
      }
    }

    return autoStartingPose;
  }

  private void seedSelectedAutoHeading(boolean force) {
    Pose2d autoStartingPose = updateSelectedAutoHeadingDashboard(force);

    if (autoHeadingSeedLocked && !force) {
      return;
    }

    if (autoStartingPose == null) {
      lastSeededAutoPose = null;
      return;
    }

    if (force
        || lastSeededAutoPose == null
        || Math.abs(
                lastSeededAutoPose.getRotation().minus(autoStartingPose.getRotation()).getDegrees())
            > 1e-3) {
      s_Swerve.seedFieldHeading(autoStartingPose.getRotation());
      lastSeededAutoPose = autoStartingPose;
    }
  }

  public void lockAutoHeadingSeed() {
    autoHeadingSeedLocked = true;
    s_Swerve.setLimelightImuSeedingEnabled(false);
    SmartDashboard.putBoolean("Auto Heading Seed Locked", autoHeadingSeedLocked);
  }

  public void captureDriverForwardHeading() {
    s_Swerve.captureDriverForwardHeadingFromLimelightForward(frontLimelight);
  }

  private Command getSelectedAutoCommand() {
    return chooser != null ? chooser.getSelected() : null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    seedSelectedAutoHeading(true);
    lockAutoHeadingSeed();

    Command selectedAuto = getSelectedAutoCommand();
    return selectedAuto != null ? selectedAuto : Commands.none();
  }
}
