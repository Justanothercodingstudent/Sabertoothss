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

public class IntakeRoller extends Command {
    private final intake Intake;
    private final XboxController xbox;

    private double speed;

    public IntakeRoller(intake Intake, XboxController xbox){
        this.Intake = Intake;
        addRequirements(this.Intake);

        this.xbox = xbox;
    }

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean ltPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean apressed = xbox.getAButton();
            SmartDashboard.putBoolean("Right Trigger Button Pressed", ltPressed); // Debugging

            if (ltPressed){
                speed = Constants.Intake.IntakeSpeed;
                Intake.setIntakeSpeed(speed);
            }else if(apressed){
                speed = Constants.Intake.IntakeSpeed;
                Intake.setIntakeSpeed(-speed);
            } else {
                Intake.setIntakeSpeed(0);
            }


        }

        
    }
    
}
