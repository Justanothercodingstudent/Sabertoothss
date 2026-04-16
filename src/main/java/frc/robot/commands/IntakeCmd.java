package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake;

public class IntakeCmd extends Command {
  private static final double DASHBOARD_UPDATE_INTERVAL_SECONDS = 0.10;

  private final intake Intake;
  private final XboxController Operator;
  private final XboxController Driver;

  private double IntakePos;
  private double speed;
  private double Rollerspeed;
  private Command activeIntakeCycle;
  private double lastDashboardUpdateSeconds = -1.0;

  public IntakeCmd(intake Intake, XboxController Operator, XboxController Driver) {
    this.Intake = Intake;
    addRequirements(this.Intake);

    this.Operator = Operator;
    this.Driver = Driver;

    IntakePos = Intake.getExtensionPos();
  }

  public class IntakeCycle extends SequentialCommandGroup {
    public IntakeCycle(intake intake) {
      addCommands(
          new InstantCommand(() -> intake.setIntakePosition(Constants.Intake.JigExtend)),
          new WaitCommand(0.25),
          new InstantCommand(() -> intake.setIntakePosition(Constants.Intake.maxExtend)),
          new WaitCommand(0.25) // ,
          /* new InstantCommand(() -> intake.setIntakePosition(Constants.Intake.JigExtend)),
          new WaitCommand(0.25),
          new InstantCommand(() -> intake.setIntakePosition(Constants.Intake.maxExtend)),
          new WaitCommand(0.25)*/
          );
    }
  }

  @Override
  public void initialize() {
    Intake.setExtendSpeed(Constants.Intake.ExtendSpeed);
    activeIntakeCycle = null;
    lastDashboardUpdateSeconds = -1.0;
  }

  @Override
  public void execute() {
    if (DriverStation.isTeleop()) {

      boolean lbPressed = Operator.getLeftBumperButton();
      boolean yPressed = Operator.getYButtonPressed();
      boolean bPressed = Operator.getBButtonPressed();
      boolean ltpressed =
          Operator.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
      boolean rbPressed = Operator.getRightBumperButton();
      double nowSeconds = Timer.getFPGATimestamp();
      if (lastDashboardUpdateSeconds < 0.0
          || nowSeconds - lastDashboardUpdateSeconds >= DASHBOARD_UPDATE_INTERVAL_SECONDS) {
        lastDashboardUpdateSeconds = nowSeconds;
        SmartDashboard.putBoolean("Right Trigger Button Pressed", ltpressed);
      }

      /*if (rtPressed && (activeIntakeCycle == null || !activeIntakeCycle.isScheduled())){
          if (shoot.FrontLeftRPM() <= -20){
              activeIntakeCycle = new IntakeCycle(Intake)
                  .finallyDo(interrupted -> Intake.setExtendSpeed(Constants.Intake.ExtendSpeed));
              CommandScheduler.getInstance().schedule(activeIntakeCycle);
          }
      }*/

      if (ltpressed) {
        speed = Constants.Intake.IntakeSpeed;
        Intake.setIntakeSpeed(speed);
      } else if (rbPressed) {
        speed = Constants.Intake.IntakeSpeed;
        Intake.setIntakeSpeed(-speed);
      } else if (!ltpressed && !rbPressed && !lbPressed) {
        Intake.setIntakeSpeed(0);
      }

      if (bPressed) {
        IntakePos = Constants.Intake.maxExtend;
        Intake.setIntakePosition(IntakePos);
      }

      if (yPressed) {
        IntakePos = Constants.Intake.minExtend;
        Intake.setIntakePosition(IntakePos);
      }

      if (lbPressed && (activeIntakeCycle == null || !activeIntakeCycle.isScheduled())) {
        activeIntakeCycle =
            new IntakeCycle(Intake)
                .finallyDo(interrupted -> Intake.setExtendSpeed(Constants.Intake.ExtendSpeed));
        CommandScheduler.getInstance().schedule(activeIntakeCycle);

        speed = Constants.Intake.IntakeSpeed;
        Intake.setIntakeSpeed(speed);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    Intake.setExtendSpeed(Constants.Intake.ExtendSpeed);
  }
}
