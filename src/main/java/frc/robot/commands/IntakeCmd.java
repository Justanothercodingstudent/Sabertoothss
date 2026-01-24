package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;
import frc.robot.Constants;

public class IntakeCmd extends Command {
    private final intake Intake;
    private final XboxController xbox;

    private double speed;

    public IntakeCmd(intake Intake, XboxController xbox){
        this.Intake = Intake;
        addRequirements(this.Intake);

        this.xbox = xbox;

    }

     @Override
    public void initialize() {
    }

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean ltPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;

            SmartDashboard.putBoolean("Right Trigger Button Pressed", ltPressed); // Debugging

            if (ltPressed){
                speed = Constants.Intake.IntakeSpeed;
                Intake.IntakeSpeed(speed);
            } else if (!ltPressed){
                Intake.IntakeSpeed(0);
            }

        }

        
    }
    
}
