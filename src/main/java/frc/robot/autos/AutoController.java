package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.subsystems.intake;

public class AutoController {

  private intake Intake;
  private SpinnerAndShooter Shoot;
  private Angler angler;

  private double activeShooterRequirementRPS = 0.0;

  public AutoController(intake Intake, SpinnerAndShooter Shoot, Angler angler) {
    this.Intake = Intake;
    this.Shoot = Shoot;
    this.angler = angler;
  }

  private void primeShooterFromAngler() {
    angler.updateFromTrackedAprilTag();

    double anglerTarget = angler.getAngleTarget();
    double shooterTargetRPS = Constants.Spin.getShooterRPSForAngle(anglerTarget);
    activeShooterRequirementRPS = Constants.Spin.getShooterFeedRPSForAngle(anglerTarget);

    if (Constants.Spin.ClosedLoopShooter) {
      Shoot.setShooterRPS(shooterTargetRPS, shooterTargetRPS);
    } else {
      Shoot.OpenShootSpeed(Constants.Spin.ShootSpeed);
    }
  }

  private boolean shooterReadyToFeed() {
    return Shoot.isAtOrAboveRPS(activeShooterRequirementRPS);
  }

  public Command Intake() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> Intake.setIntakeSpeed(Constants.Intake.IntakeSpeed), Intake));
  }

  public Command IntakeStop() {
    return new ParallelCommandGroup(new InstantCommand(() -> Intake.setIntakeSpeed(0), Intake));
  }

  public Command IntakeExtend() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.maxExtend), Intake));
  }

  public Command IntakeUp() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.minExtend), Intake));
  }

  public Command ShootAndScore() {
    return new SequentialCommandGroup(
        new InstantCommand(this::primeShooterFromAngler, Shoot, angler),
        new WaitUntilCommand(this::shooterReadyToFeed),
        new InstantCommand(() -> Shoot.SpinSpeed(Constants.Spin.SpinSpeed), Shoot),
        new InstantCommand(() -> Shoot.roller(Constants.Spin.Rollerspeed), Shoot),
        new InstantCommand(() -> Intake.setIntakeSpeed(Constants.Intake.IntakeSpeed), Intake),
        new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.JigExtend), Intake),
        new WaitCommand(0.25),
        new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.maxExtend), Intake),
        new WaitCommand(0.25),
        new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.JigExtend), Intake),
        new WaitCommand(0.25),
        new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.maxExtend), Intake),
        new WaitCommand(2),
        new InstantCommand(() -> Intake.setIntakeSpeed(0), Intake),
        new InstantCommand(() -> angler.setAnglePosition(0), angler),
        new InstantCommand(() -> Shoot.STOPSHOOTER(), Shoot),
        new InstantCommand(() -> Shoot.roller(0), Shoot),
        new InstantCommand(() -> Shoot.SpinSpeed(0), Shoot));
  }

  /*public Command FIRE(){
      return new SequentialCommandGroup(
          new InstantCommand(() -> IntakeExtend()),
          new InstantCommand(() -> Shoot()),
          new WaitCommand(0.5),
          new InstantCommand(() -> Uptake()),
          new InstantCommand(() -> Rollers()),
          new WaitCommand(1),
          new InstantCommand(() -> ShootStop()),
          new InstantCommand(() -> RollersStop()),
          new InstantCommand(() -> UptakeStop())
      );
  }*/
}
