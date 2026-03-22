// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autos.AutoController;
import frc.robot.autos.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LimelightCmd;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SpinnerAndShooterCmd;
import frc.robot.commands.climberCmd;
import frc.robot.commands.AnglerCmd;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.Angler;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kX.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final intake Intake;
  private final SpinnerAndShooter Spin;
  private final climber climb;
  private final Angler angler;

  private Limelight limelight;

  /* Commands */
  private LimelightCmd limelightCmd;

  private IntakeCmd intakeCmd;
  private SpinnerAndShooterCmd ShootCmd;
  private climberCmd climbCmd;
  private AnglerCmd anglerCmd;

  private AutoController autoController;

  private SendableChooser<Command> chooser;
  private SendableChooser<Constants.TeamDependentFactors.TeamColorSelection> teamColorChooser;
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    limelight = new Limelight();
    limelightCmd = new LimelightCmd(limelight);
    limelight.setDefaultCommand(limelightCmd);

    angler = new Angler(limelight);
    anglerCmd = new AnglerCmd(angler, operator);
    angler.setDefaultCommand(anglerCmd);

    Intake = new intake();
    intakeCmd = new IntakeCmd(Intake, operator);
    Intake.setDefaultCommand(intakeCmd);

    Spin = new SpinnerAndShooter();
    ShootCmd = new SpinnerAndShooterCmd(Spin, operator, angler, Intake);
    Spin.setDefaultCommand(ShootCmd);

    climb = new climber();
    climbCmd = new climberCmd(climb, driver);
    climb.setDefaultCommand(climbCmd);
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> false,
            limelight,
            () -> driver.getAButton()
        )
    );

    autoController = new AutoController(Intake, Spin, angler);
    Autos.registerNamedCommands(s_Swerve, autoController);
    configureTeamColorChooser();
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
}

private void configureTeamColorChooser() {
  teamColorChooser = Constants.TeamDependentFactors.getTeamColorChooser();
  SmartDashboard.putData("Team Color", teamColorChooser);
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser != null && chooser.getSelected() != null
        ? chooser.getSelected()
        : Commands.none();
}
  

}
