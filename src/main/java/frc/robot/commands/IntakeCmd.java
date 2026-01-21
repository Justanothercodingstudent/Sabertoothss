package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.intake;
import frc.robot.Constants;

public class IntakeCmd extends Command {
    private final intake Intake;
    private final XboxController xbox;

    private double speed;

    private double IntakeSpin;

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

            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;

            SmartDashboard.putBoolean("Right Trigger Button Pressed", rtPressed); // Debugging

            if (rtPressed){
                speed = Constants.Intake.IntakeSpeed;
                Intake.IntakeSpeed(speed);
            } else if (!rtPressed){
                Intake.IntakeSpeed(0);
            }

        }

        
    }
    
}
