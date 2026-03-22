package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake;
import frc.robot.Constants;

public class IntakeCmd extends Command {
    private final intake Intake;
    private final XboxController xbox;

    private double IntakePos;
    private double speed;
    private double Rollerspeed;
    private Command activeIntakeCycle;

    public IntakeCmd(intake Intake, XboxController xbox){
        this.Intake = Intake;
        addRequirements(this.Intake);

        this.xbox = xbox;

        IntakePos = Intake.getExtensionPos();

    }

    public class IntakeCycle extends SequentialCommandGroup {
                 public IntakeCycle(intake intake) {
                    addCommands(
                        new InstantCommand(() -> intake.setExtendSpeed(Constants.Intake.JigSpeed)),
                        new InstantCommand(() -> intake.setIntakePosition(Constants.Intake.JigExtend)),
                        new WaitCommand(0.25),
                        new InstantCommand(() -> intake.setIntakePosition(Constants.Intake.maxExtend)),
                        new WaitCommand(0.25)//,
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
    }

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean ltPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean yPressed = xbox.getYButtonPressed();
            boolean bPressed = xbox.getBButtonPressed();
            boolean lbPressed = xbox.getLeftBumperButton();
            boolean joyLeftPressed = xbox.getLeftStickButtonPressed();
            boolean apressed = xbox.getAButton();
            SmartDashboard.putBoolean("Right Trigger Button Pressed", ltPressed); // Debugging

            /*if (rtPressed && (activeIntakeCycle == null || !activeIntakeCycle.isScheduled())){
                if (shoot.FrontLeftRPM() <= -20){
                    activeIntakeCycle = new IntakeCycle(Intake)
                        .finallyDo(interrupted -> Intake.setExtendSpeed(Constants.Intake.ExtendSpeed));
                    CommandScheduler.getInstance().schedule(activeIntakeCycle);
                }
            }*/

            if (ltPressed){
                speed = Constants.Intake.IntakeSpeed;
                Intake.setIntakeSpeed(speed);
            }else if(apressed){
                speed = Constants.Intake.IntakeSpeed;
                Intake.setIntakeSpeed(-speed);
            } else {
                Intake.setIntakeSpeed(0);
            }

            

            if(bPressed){
                IntakePos = Constants.Intake.maxExtend;
                Intake.setIntakePosition(IntakePos);
            } 
            
            if (yPressed){
                IntakePos = Constants.Intake.minExtend;
                Intake.setIntakePosition(IntakePos);
            }

            if (lbPressed && (activeIntakeCycle == null || !activeIntakeCycle.isScheduled())) {
                activeIntakeCycle = new IntakeCycle(Intake)
                    .finallyDo(interrupted -> Intake.setExtendSpeed(Constants.Intake.ExtendSpeed));
                CommandScheduler.getInstance().schedule(activeIntakeCycle);
            }

        }

        
    }

    @Override
    public void end(boolean interrupted) {
        Intake.setExtendSpeed(Constants.Intake.ExtendSpeed);
    }
    
}
