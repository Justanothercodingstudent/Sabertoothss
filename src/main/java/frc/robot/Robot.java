// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final double DASHBOARD_UPDATE_INTERVAL_SECONDS = 0.10;

  private Command m_autonomousCommand;

  private Swerve swerve;

  private Limelight lime;

  private final RobotContainer m_robotContainer;
  private BooleanSubscriber wonAutoSub;

  private BooleanSubscriber ToggledX;

  public static boolean XToggle;

  private double lastDashboardUpdateSeconds = -1.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {

    LimelightHelpers.setRewindEnabled("limelight-front", true);
    LimelightHelpers.setRewindEnabled("limelight-left", true);

    // Seed the dashboard entry so Elastic can toggle it before a match.
    SmartDashboard.putBoolean("WonAuto", false);

    BooleanTopic wonAutoTopic =
        NetworkTableInstance.getDefault().getBooleanTopic("/SmartDashboard/WonAuto");
    wonAutoSub = wonAutoTopic.subscribe(false);

    SmartDashboard.putBoolean("XToggle", false);

    BooleanTopic XToggleTopic =
        NetworkTableInstance.getDefault().getBooleanTopic("/SmartDashboard/XToggle");
    ToggledX = XToggleTopic.subscribe(false);
  }

  public static boolean isHubActive(boolean wonAuto, double matchTime) {
    return Constants.TeamDependentFactors.isHubActive(wonAuto, matchTime);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    boolean wonAuto = wonAutoSub != null && wonAutoSub.get();
    double matchTime = DriverStation.getMatchTime();
    boolean hubActive = isHubActive(wonAuto, matchTime);
    int secondsUntilHubShiftChange =
        (int) Constants.TeamDependentFactors.getSecondsUntilNextHubShiftChange(matchTime);

    Constants.TeamDependentFactors.wonAuto = wonAuto;
    Constants.TeamDependentFactors.hubActive = hubActive;

    // lime.getRecording(matchTime);

    double nowSeconds = Timer.getFPGATimestamp();
    if (lastDashboardUpdateSeconds < 0.0
        || nowSeconds - lastDashboardUpdateSeconds >= DASHBOARD_UPDATE_INTERVAL_SECONDS) {
      lastDashboardUpdateSeconds = nowSeconds;
      SmartDashboard.putBoolean("HubActive", hubActive);
      SmartDashboard.putNumber("Hub Shift Change In", secondsUntilHubShiftChange);
      SmartDashboard.putNumber("MatchTime", matchTime);
    }

    XToggle = ToggledX != null && ToggledX.get();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    boolean isRedAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
    Constants.TeamDependentFactors.isRedTeam = isRedAlliance;

    m_robotContainer.updateSelectedAutoHeadingDashboard();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.captureDriverForwardHeading();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.lockAutoHeadingSeed();
    m_robotContainer.resetDriverForwardHeading();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
